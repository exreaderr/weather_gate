// ============================================================================
// Bme280Driver.cpp — реализация драйвера BME280/BMP280
// ============================================================================
// Карта регистров (даташит Bosch BST-BME280-DS002):
//   0xD0       chip ID (0x58 = BMP280, 0x60 = BME280)
//   0xE0       soft reset (запись 0xB6)
//   0xF2       ctrl_hum   (osrs_h; писать ДО ctrl_meas — даташит 5.4.5)
//   0xF4       ctrl_meas  (osrs_t | osrs_p | mode)
//   0xF5       config     (standby | filter | spi3w_en)
//   0x88..0xA1 калибровка T/P (+ H1 в 0xA1) — 26 байт
//   0xE1..0xE7 калибровка H — 7 байт
//   0xF7..0xFE burst: press[3] temp[3] hum[2] (20/20/16 бит)
// ============================================================================
#include "Bme280Driver.h"
#include "../core/ResourceManager.h"
#include "../services/ConfigService.h"
#include <Arduino.h>
#include <cstring>

Bme280Driver& Bme280Driver::getInstance() {
    static Bme280Driver instance;
    return instance;
}

// ============================================================================
// INIT
// ============================================================================
bool Bme280Driver::init() {
    _addr = detectAddress();
    if (_addr == 0) {
        BusManager::getInstance().busFault();   // датчик молчит на обоих адресах
        _healthy = false;
        return false;
    }

    // Адрес найден — регистрируем в реестре ресурсов (A2). Конфликт
    // (адрес занят) — громкий лог ResourceManager, но датчик уже отвечает,
    // поэтому продолжаем: конфликт поднимет conformance-стенд на стенде.
    ResourceManager::getInstance().claimI2cAddress(_addr, "wg.bme280");

    _healthy = readChipIdAndCalib() && configure();
    if (_healthy) BusManager::getInstance().busOk();
    return _healthy;
}

// --- Автодетект адреса (wx.i2c_addr: "auto" | "0x76" | "0x77") ---------------
uint8_t Bme280Driver::detectAddress() {
    char want[8];
    cfgGetStr("wx.i2c_addr", want, sizeof(want), "auto");

    BusManager& bus = BusManager::getInstance();
    if (strcmp(want, "auto") != 0) {
        uint8_t fixed = (uint8_t)strtol(want, nullptr, 0);   // понимает "0x.."
        if (!bus.i2cLock()) return 0;
        bool ok = bus.probe(fixed);
        bus.i2cUnlock();
        return ok ? fixed : 0;
    }
    if (!bus.i2cLock()) return 0;
    bool lo = bus.probe(bme280::ADDR_LOW);
    bool hi = lo ? false : bus.probe(bme280::ADDR_HIGH);
    bus.i2cUnlock();
    if (lo) return bme280::ADDR_LOW;
    if (hi) return bme280::ADDR_HIGH;
    return 0;
}

// --- Chip ID -> soft reset -> калибровка -------------------------------------
bool Bme280Driver::readChipIdAndCalib() {
    BusManager& bus = BusManager::getInstance();
    if (!bus.i2cLock()) return false;

    bool ok = readRegs(0xD0, &_chipId, 1);
    if (ok && _chipId != bme280::CHIP_ID_BME280 && _chipId != bme280::CHIP_ID_BMP280) {
        ok = false;   // на адресе что-то чужое
    }
    if (ok) ok = writeReg(0xE0, 0xB6);          // soft reset
    bus.i2cUnlock();
    if (!ok) { bus.busFault(); return false; }

    delay(10);                                  // даташит: старт после reset ~2 мс

    if (!bus.i2cLock()) return false;
    uint8_t tp[26];
    ok = readRegs(0x88, tp, sizeof(tp));
    if (ok) bme280::parseCalibTP(tp, _calib);
    if (ok && _chipId == bme280::CHIP_ID_BME280) {
        uint8_t h[7];
        ok = readRegs(0xE1, h, sizeof(h));
        if (ok) bme280::parseCalibH(h, _calib);
    }
    bus.i2cUnlock();

    if (!ok) { bus.busFault(); return false; }
    bus.busOk();
    return true;
}

// --- Конфигурация измерений: osrs x1/x1/x1, normal, standby 1000 мс -----------
bool Bme280Driver::configure() {
    BusManager& bus = BusManager::getInstance();
    if (!bus.i2cLock()) return false;
    bool ok = true;
    if (_chipId == bme280::CHIP_ID_BME280) {
        ok = writeReg(0xF2, 0x01);              // ctrl_hum: osrs_h x1 (до ctrl_meas!)
    }
    if (ok) ok = writeReg(0xF5, 0xA0);          // config: standby 1000 мс, filter off
    if (ok) ok = writeReg(0xF4, 0x27);          // ctrl_meas: osrs_t x1, osrs_p x1, normal
    bus.i2cUnlock();

    if (!ok) { bus.busFault(); return false; }
    bus.busOk();
    return true;
}

// ============================================================================
// POLL: burst 0xF7..0xFE -> компенсация -> последние значения
// ============================================================================
void Bme280Driver::poll() {
    if (_addr == 0) return;                     // датчик так и не найден

    BusManager& bus = BusManager::getInstance();
    if (!bus.i2cLock()) return;

    // До 3 попыток внутри ОДНОЙ операции — как в Ds3231Driver (урок 5.5.12:
    // I2C-флаки после power-on; busFault засчитываем один на операцию).
    uint8_t r[8];
    bool ok = false;
    for (uint8_t attempt = 0; attempt < 3 && !ok; ++attempt) {
        if (attempt > 0) { bus.i2cUnlock(); delay(10); if (!bus.i2cLock()) return; }
        ok = readRegs(0xF7, r, sizeof(r));
    }
    bus.i2cUnlock();

    if (!ok) {
        bus.busFault();
        _healthy = false;
        return;
    }
    bus.busOk();
    _healthy = true;

    const int32_t adcP = ((int32_t)r[0] << 12) | ((int32_t)r[1] << 4) | (r[2] >> 4);
    const int32_t adcT = ((int32_t)r[3] << 12) | ((int32_t)r[4] << 4) | (r[5] >> 4);
    const int32_t adcH = ((int32_t)r[6] << 8) | r[7];

    _tempC    = bme280::compensateTemp(adcT, _calib, _tFine) / 100.0f;
    const uint32_t pQ8 = bme280::compensatePress(adcP, _calib, _tFine);
    if (pQ8 == 0) return;                        // вырожденная калибровка — точку пропускаем
    float press = (pQ8 / 256.0f) / 100.0f;       // Па -> гПа

    // Стендовая калибровка (wx.press_offset_hpa) и приведение к у.м.
    // (wx.altitude_m) — оба поля читаем каждый цикл: оператор может
    // крутить их с панели без перезагрузки.
    press += cfgGetFloat("wx.press_offset_hpa", 0.0f);
    _pressHpa    = press;
    _pressSeaHpa = bme280::seaLevelPressureHpa(press, cfgGetFloat("wx.altitude_m", 0.0f));

    if (humidityValid()) {
        _humPct = bme280::compensateHum(adcH, _calib, _tFine) / 1024.0f;
    }

    _lastReadMs = millis();
    ++_readSeq;
}

// ============================================================================
// НИЗКИЙ УРОВЕНЬ (по образцу Ds3231Driver: repeated start, вызывающий держит lock)
// ============================================================================
bool Bme280Driver::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    BusManager& bus = BusManager::getInstance();
    bus.i2c().beginTransmission(_addr);
    bus.i2c().write(reg);
    if (bus.i2c().endTransmission(false) != 0) return false;
    if (bus.i2c().requestFrom(_addr, len) != len) return false;
    for (uint8_t i = 0; i < len; ++i) buf[i] = bus.i2c().read();
    return true;
}

bool Bme280Driver::writeReg(uint8_t reg, uint8_t val) {
    BusManager& bus = BusManager::getInstance();
    bus.i2c().beginTransmission(_addr);
    bus.i2c().write(reg);
    bus.i2c().write(val);
    return bus.i2c().endTransmission() == 0;
}

// ============================================================================
// МОДЕЛЬ ЧИПА (для телеметрии и публичной страницы)
// ============================================================================
const char* Bme280Driver::model() const {
    if (_chipId == bme280::CHIP_ID_BME280) return "bme280";
    if (_chipId == bme280::CHIP_ID_BMP280) return "bmp280";
    return "none";
}
