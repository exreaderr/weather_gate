// ============================================================================
// Cc1101Driver.cpp — реализация приёмника CC1101 (receive-only)
// ============================================================================
#include "Cc1101Driver.h"
#include "../services/ConfigService.h"
#include "../../projects/weather_gate/src/WeatherGateProfile.h"
#include <cstring>

// --- СТАТИКА ISR (файловая область — правило ISR платформы) -------------------
volatile uint16_t Cc1101Driver::_wHead = 0;
volatile uint16_t Cc1101Driver::_rTail = 0;
volatile bool     Cc1101Driver::_csActive = false;
Cc1101Driver::Edge Cc1101Driver::_ring[CC1101_EDGE_RING];

// Метка межпакетного зазора в кольце (шторка закрылась) — декодер сбросится
static constexpr uint16_t EDGE_GAP = 0xFFFF;

// Отсечка ISR GDO0 (файловая статика — перевзводится при открытии шторки)
static uint32_t s_lastUs  = 0;
static uint8_t  s_lastLvl = 0;
// Счётчик потерянных фронтов (переполнение кольца). Пишется из ISR,
// переносится в метрику драйвера из poll() — getInstance() в ISR запрещён.
static volatile uint32_t s_edgesDropped = 0;

Cc1101Driver& Cc1101Driver::getInstance() {
    static Cc1101Driver instance;
    return instance;
}

// ============================================================================
// ISR: GDO2 (Carrier Sense) — шторка; GDO0 — фронты данных
// ============================================================================
void IRAM_ATTR Cc1101Driver::isrGdo2() {
    _csActive = (digitalRead(WeatherGateProfile::pins().cc1101Gdo2) == HIGH);
    if (_csActive) {
        // Шторка открылась: перевзводим отсечку — первый фронт пакета
        // не должен унаследовать длительность тишины до него.
        s_lastUs  = micros();
        s_lastLvl = (digitalRead(WeatherGateProfile::pins().cc1101Gdo0) == HIGH) ? 1 : 0;
    } else {
        // Шторка закрылась: метка зазора — декодер сбросится на feed()
        uint16_t next = (uint16_t)(_wHead + 1) % CC1101_EDGE_RING;
        if (next != _rTail) {
            _ring[_wHead] = { EDGE_GAP, 0 };
            _wHead = next;
        }
    }
}

void IRAM_ATTR Cc1101Driver::isrGdo0() {
    if (!_csActive) return;                 // шторка закрыта — эфирный шум
    // Длительность завершившегося уровня. micros() в ISR допустим
    // (чтение таймера, без heap/объектов); millis() в ISR — запрещён.
    uint32_t now = micros();
    uint8_t lvl = (digitalRead(WeatherGateProfile::pins().cc1101Gdo0) == HIGH) ? 1 : 0;
    uint32_t dur = now - s_lastUs;
    if (dur >= EDGE_GAP) dur = EDGE_GAP - 1;   // кламп: 0xFFFF = метка зазора
    uint16_t next = (uint16_t)(_wHead + 1) % CC1101_EDGE_RING;
    if (next != _rTail) {
        _ring[_wHead] = { (uint16_t)dur, s_lastLvl };
        _wHead = next;
    } else {
        ++s_edgesDropped;   // кольцо полно — фронт лучше потерять,
                            // но метрику сохранить (урок тихих потерь)
    }
    s_lastUs  = now;
    s_lastLvl = lvl;
}

// ============================================================================
// INIT
// ============================================================================
bool Cc1101Driver::init() {
    const WeatherGatePins& p = WeatherGateProfile::pins();
    _freqMHz = cfgGetFloat("wx.rf_freq_mhz", 915.0f);

    pinMode(p.cc1101Cs,   OUTPUT);
    digitalWrite(p.cc1101Cs, HIGH);          // CS idle HIGH — отпущен
    pinMode(p.cc1101Gdo0, INPUT);            // input-only; подтяжки нет — CC1101 push-pull
    pinMode(p.cc1101Gdo2, INPUT);

    // SPI: хост HSPI задаём ЯВНО (правило нумерации classic ESP32).
    // new(std::nothrow) — heap-блок один раз из init (урок outbox/BSS).
    _spi = new (std::nothrow) SPIClass(HSPI);
    if (_spi == nullptr) { _healthy = false; return false; }
    _spi->begin((int8_t)p.cc1101Sck, (int8_t)p.cc1101Miso,
                (int8_t)p.cc1101Mosi, (int8_t)p.cc1101Cs);

    _healthy = detectChip();
    if (!_healthy) return false;

    writeRxTable();

    // Рабочий режим — только приём. STX не существует в этом драйвере.
    xferReg(cc1101::STROBE_SIDLE, 0);
    xferReg(cc1101::STROBE_SFRX, 0);
    xferReg(cc1101::STROBE_SRX, 0);

    attachInterrupt(digitalPinToInterrupt(p.cc1101Gdo2), isrGdo2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(p.cc1101Gdo0), isrGdo0, CHANGE);
    return true;
}

// ============================================================================
// POLL: дрейн кольца -> декодер -> дедупликация пары (31 мс)
// ============================================================================
void Cc1101Driver::poll() {
    if (!_healthy) return;
    _edgesDropped = s_edgesDropped;

    fo::WeatherPacket pkt;
    while (_rTail != _wHead) {
        const Edge e = _ring[_rTail];
        _rTail = (uint16_t)(_rTail + 1) % CC1101_EDGE_RING;
        uint16_t dur = (e.durUs == EDGE_GAP) ? fo::GAP_US : e.durUs;
        if (_dec.feed(dur, e.level, pkt)) {
            // Пакет собран. Дедупликация: станция шлёт пару за 31 мс —
            // идентичные 10 байт в пределах 2 с = второй экземпляр пары.
            // Считаем отдельно, данные НЕ перетираем повтором.
            uint32_t now = millis();
            bool dup = (_pktSeq != 0) && (now - _lastPktMs < 2000) &&
                       (memcmp(_lastRaw, _dec.lastRaw(), sizeof(_lastRaw)) == 0);
            if (dup) {
                ++_dupSeq;
            } else {
                memcpy(_lastRaw, _dec.lastRaw(), sizeof(_lastRaw));
                _lastPkt = pkt;
                _lastPktMs = now;
                ++_pktSeq;
                _rssiDbm = readRssiDbm();
            }
        }
    }
}

// ============================================================================
// НИЗКИЙ УРОВЕНЬ SPI
// ============================================================================
uint8_t Cc1101Driver::xferReg(uint8_t addr, uint8_t val) {
    const WeatherGatePins& p = WeatherGateProfile::pins();
    _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(p.cc1101Cs, LOW);
    uint8_t status = _spi->transfer(addr);
    _spi->transfer(val);
    digitalWrite(p.cc1101Cs, HIGH);
    _spi->endTransaction();
    return status;
}

uint8_t Cc1101Driver::readStatus(uint8_t addr) {
    const WeatherGatePins& p = WeatherGateProfile::pins();
    _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(p.cc1101Cs, LOW);
    _spi->transfer((uint8_t)(addr | 0xC0));
    uint8_t v = _spi->transfer(0x00);
    digitalWrite(p.cc1101Cs, HIGH);
    _spi->endTransaction();
    return v;
}

bool Cc1101Driver::detectChip() {
    // Сброс, затем идентификация. 0x00/0xFF в VERSION = «чип молчит»
    // (обрыв SPI, нет питания модуля, чужой чип).
    xferReg(cc1101::STROBE_SRES, 0);
    delay(5);
    uint8_t pn  = readStatus(cc1101::REG_PARTNUM);
    uint8_t ver = readStatus(cc1101::REG_VERSION);
    return pn == cc1101::PARTNUM_EXPECT &&
           (ver == cc1101::VERSION_EXPECT || ver == cc1101::VERSION_LEGACY);
}

void Cc1101Driver::writeRxTable() {
    xferReg(cc1101::STROBE_SIDLE, 0);
    size_t n = 0;
    const cc1101::RegVal* t = cc1101::rxTableBase(n);
    for (size_t i = 0; i < n; ++i) xferReg(t[i].reg, t[i].val);
    // FREQ2/1/0 из конфигурируемой частоты (по умолчанию 915.00 -> 0x23313B)
    uint32_t f = cc1101::freqWord(_freqMHz);
    xferReg(0x0D, (uint8_t)(f >> 16));
    xferReg(0x0E, (uint8_t)(f >> 8));
    xferReg(0x0F, (uint8_t)f);
}

int16_t Cc1101Driver::readRssiDbm() {
    int8_t raw = (int8_t)readStatus(cc1101::REG_RSSI);
    return (int16_t)raw / 2 - 74;        // даташит TI: RSSI_dBm = raw/2 - 74
}
