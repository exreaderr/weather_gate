// ============================================================================
// WeatherGateProfile.h — ПРОФИЛЬ ШЛЮЗА ПОГОДНОЙ СТАНЦИИ (композиционный корень)
// ============================================================================
// МикроОС 5.0, плата WT32-ETH01. Третий профиль платформы.
// Единственное место, где профиль знает про свою периферию.
//
// Точка входа устройства (weather_gate.ino):
//   void setup() { Kernel::getInstance().run<WeatherGateProfile>(); }
//
// ПИН-ПЛАН (утверждён владельцем 17.08.2026, официальная распиновка
// WT32-ETH01; GPIO13 на гребёнке отсутствует — находка Н-5):
//   CC1101 SPI:   SCK=14, MOSI=4, MISO=35, CS=17
//   CC1101 сигналы: GDO0=36 (данные, IRQ), GDO2=39 (резерв)
//   LED профиля:  GPIO2 (при прошивке держать LOW — страппинг)
//   Safe Mode:    GPIO15, кнопка на GND (активный низ)
//   Резерв:       GPIO5 (после прозвонки эрраты шелкографии IO5/IO35),
//                 GPIO12 (никогда не CS: HIGH при буте = нет загрузки)
//   Ядерные:      I2C 32/33 (DS3231 0x68 + BME280 0x76/0x77), RMII, GPIO0/1/3.
// ============================================================================
#pragma once

#include <core/IDeviceProfile.h>

struct WeatherGatePins {
    // CC1101 (E07-900M10S), receive-only — W2
    uint8_t cc1101Sck  = 14;
    uint8_t cc1101Mosi = 4;
    uint8_t cc1101Miso = 35;   // input-only; ⚠ эррата: прозвонить шелк IO5/IO35
    uint8_t cc1101Cs   = 17;   // LED на 17 = индикация обмена с радио
    uint8_t cc1101Gdo0 = 36;   // input-only, прерывание — сырой демодулятор
    uint8_t cc1101Gdo2 = 39;   // input-only, резерв

    uint8_t led       = 2;     // LED статуса профиля
    uint8_t safeMode  = 15;    // кнопка на GND (активный низ)
};

class WeatherGateProfile : public IDeviceProfile {
public:
    const char* profileId() const override { return "weather_gate"; }

    /// Манифест: пины -> HardwareManifest. Быстро, без железа (до Safe Mode).
    void describeHardware(HardwareManifest& m) override;

    /// Драйверы профиля: Bme280Driver (слой drivers/). DS3231/EspTemp
    /// поднимает ядро как базу платформы. CC1101 — стадия W2.
    void registerDrivers(const HardwareManifest& m) override;

    /// Модули профиля: WeatherGateApp. Здесь же — claimEventRange для wg_ev.
    void registerModules(Kernel& k) override;

    /// Статический доступ к пинам для модулей и драйверов профиля.
    static const WeatherGatePins& pins() { return _pins; }

private:
    static WeatherGatePins _pins;
};
