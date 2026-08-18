// ============================================================================
// Bme280Driver.h — ДАТЧИК ДАВЛЕНИЯ/ТЕМПЕРАТУРЫ/ВЛАЖНОСТИ BME280 (системная I2C)
// ============================================================================
// Стадия W1 профиля weather_gate. Третий драйвер слоя drivers/ — по образцу
// Ds3231Driver: шина ТОЛЬКО через BusManager (i2cLock/probe/busOk/busFault),
// никакой политики (тренды, прогноз, MQTT — дело WeatherGateApp, W3+).
//
// Решения владельца (16.08.2026):
//   · шина — СИСТЕМНАЯ 32/33 под BusManager (не отдельная I2C1, как в
//     легаси-монолите v5.2);
//   · адрес — автодетект 0x76/0x77 через probe, поле wx.i2c_addr может
//     зафиксировать адрес вручную;
//   · модуль «GY-BM280» может оказаться BMP280 (chip ID 0x58) — тогда
//     влажность отсутствует (humidityValid() == false), телеметрия
//     опускает ключ "humidity" (smart_lock это переживёт: неизвестные и
//     отсутствующие ключи игнорируются).
//
// Математика — в Bme280Core.h (чистая логика, host-тесты D2 по эталону).
// ============================================================================
#pragma once

#include "../core/IDeviceDriver.h"
#include "../core/BusManager.h"
#include "Bme280Core.h"
#include <cmath>

// Опрос: раз в 10 с (уличная станция шлёт раз в 48 с; давление для тренда
// Замбретти нужно чаще — RAW-кольцо DataLogService получит точки из W3+)
constexpr uint32_t BME280_POLL_MS = 10000;

class Bme280Driver : public IDeviceDriver {
public:
    static Bme280Driver& getInstance();

    // --- IDeviceDriver ---------------------------------------------------
    const char* driverName() const override { return "bme280"; }
    bool init() override;
    void poll() override;
    uint32_t getPollIntervalMs() const override { return BME280_POLL_MS; }
    bool isHealthy() const override { return _healthy; }

    // --- ДОСТУП К ПОСЛЕДНЕМУ ИЗМЕРЕНИЮ (вызывает WeatherGateApp) ------------
    /// Модель чипа: "bme280" / "bmp280" / "none" (до init или при отказе).
    const char* model() const;
    uint8_t  chipId()      const { return _chipId; }
    uint8_t  address()     const { return _addr; }
    bool     humidityValid() const { return _chipId == bme280::CHIP_ID_BME280; }

    float    temperatureC()   const { return _tempC; }
    /// Станционное давление с учётом wx.press_offset_hpa.
    float    pressureHpa()    const { return _pressHpa; }
    /// Приведённое к у.м. (wx.altitude_m); при altitude=0 — равно станционному.
    float    pressureSeaHpa() const { return _pressSeaHpa; }
    /// Валидна только при humidityValid(); иначе NAN.
    float    humidityPct()    const { return _humPct; }

    /// millis() последнего успешного чтения (0 — ещё не было).
    uint32_t lastReadMs()  const { return _lastReadMs; }
    /// Счётчик успешных чтений (сторож «залипшего датчика» ПАЗ, W3).
    uint32_t readSeq()     const { return _readSeq; }

private:
    Bme280Driver() = default;

    // --- НИЗКИЙ УРОВЕНЬ (вызывающий обязан держать i2cLock) -----------------
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
    bool writeReg(uint8_t reg, uint8_t val);

    // --- ЭТАПЫ INIT ----------------------------------------------------------
    uint8_t detectAddress();          // auto: 0x76 -> 0x77; иначе из конфига
    bool    readChipIdAndCalib();     // chip ID, soft reset, калибровка
    bool    configure();              // osrs x1, normal mode, standby 1000 мс

    // --- ДАННЫЕ ----------------------------------------------------------------
    bme280::Calib _calib;
    int32_t  _tFine      = 0;
    uint8_t  _addr       = 0;          // 0 — не найден
    uint8_t  _chipId     = 0;
    bool     _healthy    = false;

    float    _tempC       = NAN;       // NAN до первого успешного чтения
    float    _pressHpa    = NAN;
    float    _pressSeaHpa = NAN;
    float    _humPct      = NAN;
    uint32_t _lastReadMs  = 0;
    uint32_t _readSeq     = 0;
};
