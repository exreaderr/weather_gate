// ============================================================================
// weather_gate.ino — ШЛЮЗ ПОГОДНОЙ СТАНЦИИ (МикроОС 5.0, WT32-ETH01)
// ============================================================================
// Профиль WeatherGateProfile. Стадия W1: скелет + Bme280Driver
// (host-тесты компенсации по эталону — host/tests.cpp, testBme280).
// CC1101 RX-only + декодер Fine Offset — W2; телеметрия/UI — W3;
// даталог wx_* на home_master — W4; Замбретти — W5.
// ============================================================================
#include <MicroOS.h>
#include "src/WeatherGateProfile.h"

void setup() {
    Kernel::getInstance().run<WeatherGateProfile>();
}

void loop() {
    Kernel::getInstance().loop();
}
