// ============================================================================
// WeatherCore.h — ЧИСТАЯ ЛОГИКА ПОГОДНЫХ ПРОИЗВОДНЫХ (стадия W3)
// ============================================================================
// Header-only, без Arduino/heap — host-тестируемая (host/tests.cpp).
// Три задачи:
//   1) «Ощущается как»: ветро-холодовой индекс (Environment Canada/NOAA)
//      и тепловой индекс Rothfusz с поправками NOAA — та же пара формул,
//      что использует УД, чтобы цифра на замке совпадала с HA;
//   2) интенсивность осадков из сырого 12-бит счётчика опрокидываний
//      Fine Offset (0.3 мм/тип, wrap 4096, защита от сброса счётчика
//      при смене батареи);
//   3) код состояния по контракту weather-JSON smart_lock (коды HA:
//      pouring/rainy/windy/cloudy; «sunny» честно не выдаём — датчика
//      облачности нет, обогащение состояния — Замбретти, стадия W5).
// ============================================================================
#pragma once

#include <stdint.h>
#include <math.h>

namespace wxc {

// --- «ОЩУЩАЕТСЯ КАК» ---------------------------------------------------------
// tC <= +10 и ветер > 4.8 км/ч -> wind chill;
// tC >= +27 и влажность >= 40% -> heat index;
// иначе ощущаемая == фактическая.
inline float feelsLikeC(float tC, float rhPct, float windMs) {
    float vKmh = windMs * 3.6f;
    if (tC <= 10.0f && vKmh > 4.8f) {
        float v016 = powf(vKmh, 0.16f);
        return 13.12f + 0.6215f * tC - 11.37f * v016 + 0.3965f * tC * v016;
    }
    if (tC >= 27.0f && rhPct >= 40.0f) {
        float T = tC * 9.0f / 5.0f + 32.0f;   // Rothfusz работает в °F
        float R = rhPct;
        float hi = -42.379f + 2.04901523f * T + 10.14333127f * R
                 - 0.22475541f * T * R - 0.00683783f * T * T
                 - 0.05481717f * R * R + 0.00122874f * T * T * R
                 + 0.00085282f * T * R * R - 0.00000199f * T * T * R * R;
        if (R < 13.0f && T >= 80.0f && T <= 112.0f)
            hi -= ((13.0f - R) / 4.0f) * sqrtf((17.0f - fabsf(T - 95.0f)) / 17.0f);
        else if (R > 85.0f && T >= 80.0f && T <= 87.0f)
            hi += ((R - 85.0f) / 10.0f) * ((87.0f - T) / 5.0f);
        return (hi - 32.0f) * 5.0f / 9.0f;
    }
    return tC;
}

// --- ОСАДКИ ------------------------------------------------------------------
// Дельта 12-бит счётчика опрокидываний с учётом wrap 4096.
inline uint16_t rainDelta(uint16_t prevRaw, uint16_t nowRaw) {
    return (uint16_t)((nowRaw - prevRaw) & 0x0FFF);
}

constexpr float    WX_RAIN_MM_PER_TIP  = 0.3f;    // из протокола (rtl_433)
constexpr uint32_t WX_RAIN_WINDOW_SEC  = 3600;    // окно интенсивности — час
constexpr uint32_t WX_RAIN_MINSPAN_SEC = 600;     // короче — данных мало, 0
constexpr uint16_t WX_RAIN_MAX_DELTA   = 200;     // > 60 мм за окно = сброс
                                                   // счётчика (батарея), не ливень

// Трекер интенсивности дождя: кольцо отсчётов с разрешением 5 минут
// (пакеты летят каждые ~48 с, хранить все — пустая трата RAM; 13 точек
// покрывают часовое окно). Чистая логика — времена подаёт вызывающий.
class RainTracker {
public:
    static constexpr uint8_t CAP = 13;            // 12 шагов по 5 мин + запас

    void reset() { _count = 0; _head = 0; _lastStoreTs = 0; }

    /// Вызов на каждый принятый пакет. ts — unix-секунды (0 — нет времени,
    /// точка игнорируется: без времени данные — мусор, урок DataLogService).
    void add(uint16_t raw, uint32_t ts) {
        if (ts == 0) return;
        if (_count > 0 && (uint32_t)(ts - _lastStoreTs) < 300) return;
        _ts[_head]  = ts;
        _raw[_head] = raw;
        _head = (uint8_t)((_head + 1) % CAP);
        if (_count < CAP) ++_count;
        _lastStoreTs = ts;
    }

    /// Интенсивность, мм/ч. 0 — дождя нет ИЛИ данных пока недостаточно.
    float rateMmPh(uint32_t nowTs) const {
        if (_count < 2 || nowTs == 0) return 0.0f;
        // Новейшая точка
        uint8_t newest = (uint8_t)((_head + CAP - 1) % CAP);
        // Старейшая в пределах окна
        uint8_t oldest = newest;
        for (uint8_t i = 0; i < _count; ++i) {
            uint8_t idx = (uint8_t)((_head + CAP - 1 - i) % CAP);
            if ((uint32_t)(nowTs - _ts[idx]) <= WX_RAIN_WINDOW_SEC) oldest = idx;
            else break;
        }
        uint32_t span = _ts[newest] - _ts[oldest];
        if (span < WX_RAIN_MINSPAN_SEC) return 0.0f;
        uint16_t d = rainDelta(_raw[oldest], _raw[newest]);
        if (d > WX_RAIN_MAX_DELTA) return 0.0f;   // сброс счётчика, не ливень
        return d * WX_RAIN_MM_PER_TIP * 3600.0f / (float)span;
    }

    uint8_t count() const { return _count; }

private:
    uint32_t _ts[CAP]  = {};
    uint16_t _raw[CAP] = {};
    uint8_t  _count = 0;
    uint8_t  _head  = 0;
    uint32_t _lastStoreTs = 0;
};

// --- КОД СОСТОЯНИЯ (контракт weather-JSON smart_lock: коды HA) ---------------
// Дождь важнее ветра: при ливне с ветром карточка говорит про ливень.
inline const char* weatherState(float rainMmPh, float windMs, float gustMs) {
    if (rainMmPh >= 4.0f)                    return "pouring";
    if (rainMmPh >  0.1f)                    return "rainy";
    if (windMs >= 10.8f || gustMs >= 15.0f)  return "windy";   // 6 Бф / шквал
    return "cloudy";   // нейтрально: облачность не измеряем (W5 — Замбретти)
}

} // namespace wxc
