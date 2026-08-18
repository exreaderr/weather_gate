// ============================================================================
// WeatherGateEvents.h — СОБЫТИЯ ПРОФИЛЯ WEATHER_GATE (диапазон 0x1000+)
// ============================================================================
// База диапазона — от ResourceManager::claimEventRange("weather_gate")
// (урок v4.2.2: жёсткие ID = коллизии). До регистрации база = 0 — любое
// использование до этого момента является ошибкой программирования.
// ============================================================================
#pragma once

#include <cstdint>

namespace wg_ev {

// Записывается один раз из WeatherGateProfile::registerModules().
inline int32_t g_base = 0;

// Смещения внутри диапазона (шаг claimEventRange = 0x40 — запас 64 ID).
// W1: объявлены, но ещё не издаются — наполнение по мере стадий.
inline int32_t sensorLost()      { return g_base + 0x00; } // BME280 перестал отвечать (W3, ПАЗ)
inline int32_t sensorRestored()  { return g_base + 0x01; } // BME280 вернулся
inline int32_t radioPacket()     { return g_base + 0x02; } // валидный пакет WS1080 (W2)
inline int32_t radioSilence()    { return g_base + 0x03; } // эфир молчит дольше нормы (W3, ПАЗ)
inline int32_t forecastChanged() { return g_base + 0x04; } // Замбретти сменил прогноз (W5)

} // namespace wg_ev
