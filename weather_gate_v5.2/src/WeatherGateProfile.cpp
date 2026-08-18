// ============================================================================
// WeatherGateProfile.cpp — реализация композиционного корня weather_gate
// ============================================================================
#include "WeatherGateProfile.h"
#include "WeatherGateEvents.h"
#include "WeatherGateApp.h"
#include <drivers/Bme280Driver.h>
#include <drivers/Cc1101Driver.h>
#include <core/DriverRegistry.h>
#include <core/ResourceManager.h>
#include <core/Kernel.h>

WeatherGatePins WeatherGateProfile::_pins;

// ============================================================================
// МАНИФЕСТ ПЕРИФЕРИИ
// ============================================================================
void WeatherGateProfile::describeHardware(HardwareManifest& m) {
    // Быстро и без железа: вызывается ДО детекта Safe Mode.
    m.safeModePin = (int8_t)_pins.safeMode;   // GPIO15, кнопка на GND

    // Периферия профиля — универсальным механизмом (уровень 3 драйверной
    // модели). safeModePin НЕ добавляем: пин уже сообщён ядру.
    m.addGpio(_pins.cc1101Sck,  "wg.cc1101.sck");
    m.addGpio(_pins.cc1101Mosi, "wg.cc1101.mosi");
    m.addGpio(_pins.cc1101Miso, "wg.cc1101.miso");
    m.addGpio(_pins.cc1101Cs,   "wg.cc1101.cs");
    m.addGpio(_pins.cc1101Gdo0, "wg.cc1101.gdo0");
    m.addGpio(_pins.cc1101Gdo2, "wg.cc1101.gdo2");
    m.addGpio(_pins.led,        "wg.led");
}

// ============================================================================
// ДРАЙВЕРЫ
// ============================================================================
void WeatherGateProfile::registerDrivers(const HardwareManifest& m) {
    (void)m;   // пины — из WeatherGatePins (манифест уже валидирован RM)

    // BME280/BMP280 на системной шине (32/33) под BusManager: адрес и
    // модель чипа драйвер определяет сам (автодетект 0x76/0x77, chip ID).
    DriverRegistry::getInstance().add(&Bme280Driver::getInstance());

    // W2: приёмник CC1101, строго receive-only (TX-путей в драйвере нет
    // физически). Пины SPI/GDO — из WeatherGatePins (манифест выше).
    DriverRegistry::getInstance().add(&Cc1101Driver::getInstance());
}

// ============================================================================
// МОДУЛИ ПРОФИЛЯ
// ============================================================================
void WeatherGateProfile::registerModules(Kernel& k) {
    // Диапазон событий профиля — из реестра. До этой строки wg_ev::*
    // недействительны.
    wg_ev::g_base = ResourceManager::getInstance().claimEventRange("weather_gate");

    // Единственный прикладной модуль (W1). Приоритет 9 — в профильной
    // зоне, после ядерных сервисов (образец: SmartLockApp).
    k.registerModule(&WeatherGateApp::getInstance(), 9, 0);
}
