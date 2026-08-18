// ============================================================================
// FineOffsetCore.h — ЧИСТЫЙ ДЕКОДЕР ПРОТОКОЛА FINE OFFSET WH1080/WS1080 (FSK)
// ============================================================================
// Стадия W2 профиля weather_gate. Эталон формата — rtl_433
// (fineoffset_wh1080.c, вариант FSK_PULSE_PCM). Зависимостей от
// Arduino/FreeRTOS НЕТ — компилируется на хосте (D2: host-тесты, эталон
// теста — независимый кодер кадра в tests.cpp).
//
// Физика канала (915 МГц, 2-FSK, 17.24 кБод): CC1101 в async-режиме
// (PKTCTRL0=0x32, IOCFG0=0x0D) выдаёт на GDO0 ДЕМОДУЛИРОВАННЫЕ биты
// уровнями ~58 мкс. Декодер питается фронтами: (длительность уровня,
// значение уровня). Никакого Манчестера на этом этапе нет — уточнение
// к находке Н-2 (Манчестер был неточностью ИИ-консультации в docs/).
//
// Кадр (rtl_433): preamble+sync AA 2D D4, затем 10 байт:
//   [0]    F (4 бита тип: 0xA=weather) + старшие 4 бита device id
//   [1]    младшие 4 бита id + старшие 4 бита температуры (12 бит,
//          FSK-знак: бит 0x800 = минус, magnitude)
//   [2]    температура, младшие 8 бит (0.1 °C)
//   [3]    влажность, %
//   [4]    средний ветер, 0.34 м/с
//   [5]    порыв, 0.34 м/с
//   [6]    верхний ниббл не определён + старшие 4 бита счётчика дождя
//   [7]    дождь, младшие 8 бит (0.3 мм/импульс)
//   [8]    флаги (старший ниббл: 0x1 = battery_low) + направление 0..15
//   [9]    CRC-8 (poly 0x31, init 0xFF) по кадру FF + байты [0..9];
//          crc8(FF + [0..9]) == 0 у валидного пакета
// Станция шлёт ДВА одинаковых пакета с интервалом 31 мс каждые ~48 с —
// дедупликация реализуется вызывающим (Cc1101Driver).
// ============================================================================
#pragma once

#include <cstdint>
#include <cstring>

namespace fo {

// --- Тайминги PCM (rtl_433: short=long=58 мкс, reset_limit=5800 мкс) --------
constexpr uint16_t BIT_US       = 58;    // длительность бита
constexpr uint16_t GAP_US       = 5800;  // тишина дольше — межпакетный зазор
constexpr uint16_t MIN_PULSE_US = 20;    // короче — иглоподобная помеха
constexpr uint8_t  MAX_RUN_BITS = 100;   // серия одного бита длиннее — мусор

// --- ТАБЛИЦА НАПРАВЛЕНИЙ (rtl_433, 16 румбов с округлением 22.5°) -------------
inline uint16_t dirDeg(uint8_t idx) {
    static const uint16_t DEG[16] = {
        0, 23, 45, 68, 90, 113, 135, 158,
        180, 203, 225, 248, 270, 293, 315, 338
    };
    return DEG[idx & 0x0F];
}

// --- CRC-8 poly 0x31, init 0xFF, MSB-first (контроль кадра) -------------------
inline uint8_t crc8(const uint8_t* d, uint8_t n) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < n; ++i) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

// --- РАСПАКОВАННЫЙ ПАКЕТ ------------------------------------------------------
struct WeatherPacket {
    uint8_t  deviceId;     // 8 бит; меняется у станции при смене батарей
    bool     batteryLow;
    float    tempC;        // -40..+65
    uint8_t  humidity;     // 10..99 %
    float    windMs;       // средний ветер, м/с
    float    gustMs;       // порыв, м/с
    uint16_t dirDeg;       // азимут флюгера, градусы
    float    rainMm;       // накопительный счётчик, мм
    uint16_t rainRaw;      // сырой счётчик (12 бит) — для дельт и ролловера
};

// --- РАЗБОР КАДРА (10 байт после sync) ------------------------------------------
/// true — это weather-кадр (тип 0xA) и поля извлечены. Кадры времени
/// (0xB) и UV/light WH3080 (0x07) молча отклоняются — шлюз их не потребляет.
inline bool parseFrame(const uint8_t* b /*10 байт*/, WeatherPacket& out) {
    if ((b[0] >> 4) != 0x0A) return false;         // только weather
    out.deviceId = (uint8_t)((b[0] << 4) & 0xF0) | (b[1] >> 4);
    int tempRaw = ((b[1] & 0x0F) << 8) | b[2];
    if (tempRaw & 0x800) {                         // FSK: знак + magnitude
        tempRaw = -(tempRaw & 0x7FF);
    }
    out.tempC      = tempRaw * 0.1f;
    out.humidity   = b[3];
    out.windMs     = b[4] * 0.34f;
    out.gustMs     = b[5] * 0.34f;
    out.rainRaw    = (uint16_t)(((b[6] & 0x0F) << 8) | b[7]);
    out.rainMm     = out.rainRaw * 0.3f;
    out.batteryLow = (b[8] >> 4) == 1;
    out.dirDeg     = dirDeg(b[8] & 0x0F);
    return true;
}

// --- ПОТОКОВЫЙ ДЕКОДЕР PCM ------------------------------------------------------
/// Кормить фронтами: level — значение уровня, который держался durUs мкс.
/// Возвращает true ровно один раз на валидный пакет (заполнен `out`).
/// Чистая функция состояния — никакого времени/heap; кольцо фронтов
/// дрейнит вызывающий (Cc1101Driver из poll()).
class Decoder {
public:
    Decoder() { reset(); }

    void reset() {
        _shift = 0;
        _synced = false;
        _bitCount = 0;
        memset(_data, 0, sizeof(_data));
    }

    bool feed(uint16_t durUs, uint8_t level, WeatherPacket& out) {
        // Межпакетный зазор — полный сброс (reset_limit эталона)
        if (durUs >= GAP_US) { reset(); return false; }
        // Игла короче трети бита: в поиске — игнор, в данных — кадр битый
        if (durUs < MIN_PULSE_US) {
            if (_synced) reset();
            return false;
        }
        // Квантование длительности в число бит (округление к ближайшему)
        uint32_t nBits = (durUs + BIT_US / 2) / BIT_US;
        if (nBits == 0 || nBits > MAX_RUN_BITS) { reset(); return false; }
        bool got = false;
        for (uint32_t i = 0; i < nBits && !got; ++i)
            got = pushBit(level & 1, out);
        return got;
    }

    bool synced() const { return _synced; }

    /// Сырые 10 байт последнего валидного пакета (для дедупликации пары,
    /// которую станция шлёт с интервалом 31 мс — сравнение содержимого).
    const uint8_t* lastRaw() const { return _lastRaw; }

private:
    bool pushBit(uint8_t bit, WeatherPacket& out) {
        if (!_synced) {
            _shift = (_shift << 1) | bit;
            if ((_shift & 0x00FFFFFFUL) == 0x00AA2DD4UL) {
                _synced = true;
                _bitCount = 0;
                memset(_data, 0, sizeof(_data));
            }
            return false;
        }
        // Данные: 80 бит, MSB-first
        _data[_bitCount >> 3] |= (uint8_t)(bit << (7 - (_bitCount & 7)));
        ++_bitCount;
        if (_bitCount < 80) return false;

        // Кадр собран: FF + 10 байт, CRC-8 должен сойтись в ноль
        uint8_t frame[11];
        frame[0] = 0xFF;
        memcpy(frame + 1, _data, 10);
        bool ok = (crc8(frame, sizeof(frame)) == 0) && parseFrame(_data, out);
        if (ok) memcpy(_lastRaw, _data, sizeof(_lastRaw));
        reset();                                 // готов к следующему пакету
        return ok;
    }

    uint32_t _shift;      // сдвиговое окно поиска sync (24 бита)
    bool     _synced;
    uint8_t  _bitCount;   // бит данных, 0..80
    uint8_t  _data[10];   // полезная часть после sync
    uint8_t  _lastRaw[10]{}; // последний валидный кадр (сырьё)
};

} // namespace fo
