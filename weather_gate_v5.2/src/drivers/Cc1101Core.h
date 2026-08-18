// ============================================================================
// Cc1101Core.h — ЧИСТАЯ ЛОГИКА CC1101 (RX-only): регистры и частотное слово
// ============================================================================
// Стадия W2 профиля weather_gate. Зависимостей от Arduino/FreeRTOS НЕТ —
// host-тесты D2 проверяют таблицу и расчёт FREQ[23:0].
//
// ДОКТРИНА РАДИО (брифинг): модуль E07-900M10S работает СТРОГО на приём.
// Здесь нет и не будет ничего относящегося к передаче: ни строба STX,
// ни записи в TX FIFO, ни режимов с автопереходом в TX. Передатчик не
// конфигурируется НИКОГДА — «выключен отсутствием кода».
//
// Параметры радиотракта — полевой конструктор из docs/init_cc1101.pdf
// (подтверждён рабочим легаси-монолитом и rtl_433): 915 МГц, 2-FSK,
// 17.24 кБод, девиация 47.6 кГц, полоса ПЧ 325 кГц (правило Карсона
// ~117 кГц + дрейф кварцев датчика и модуля на морозе/жаре).
// ============================================================================
#pragma once

#include <cstdint>
#include <cstddef>

namespace cc1101 {

// --- ИДЕНТИФИКАЦИЯ ЧИПА (статус-регистры, чтение: addr | 0xC0) ----------------
constexpr uint8_t REG_PARTNUM = 0x30;   // ожидается 0x00
constexpr uint8_t REG_VERSION = 0x31;   // ожидается 0x14 (0x04 — старые клоны)
constexpr uint8_t REG_RSSI    = 0x34;   // текущий RSSI
constexpr uint8_t PARTNUM_EXPECT  = 0x00;
constexpr uint8_t VERSION_EXPECT  = 0x14;
constexpr uint8_t VERSION_LEGACY  = 0x04;

// --- СТРОБЫ (только RX-безопасные; STX=0x35 ЗАПРЕЩЁН доктриной) ---------------
constexpr uint8_t STROBE_SRES  = 0x30;  // сброс
constexpr uint8_t STROBE_SRX   = 0x34;  // включить приём
constexpr uint8_t STROBE_SIDLE = 0x36;  // ожидание (для перезаписи регистров)
constexpr uint8_t STROBE_SFRX  = 0x3A;  // сброс RX FIFO

// --- ЧАСТОТНОЕ СЛОВО -----------------------------------------------------------
/// FREQ[23:0] = round(f_MHz * 2^16 / 26 МГц). Контрольная точка из полевых
/// испытаний (init_cc1101.pdf): 915.00 МГц -> 0x23313B.
inline uint32_t freqWord(float freqMHz) {
    return (uint32_t)(freqMHz * (65536.0f / 26.0f) + 0.5f);
}

// --- ТАБЛИЦА ИНИЦИАЛИЗАЦИИ RX ---------------------------------------------------
struct RegVal { uint8_t reg; uint8_t val; };

// Каноническая адресация регистров конфигурации (даташит TI SWRA295):
// IOCFG2=0x00, IOCFG1=0x01, IOCFG0=0x02, PKTCTRL1=0x07, PKTCTRL0=0x08,
// FSCTRL1=0x0B, FREQ2=0x0D, FREQ1=0x0E, FREQ0=0x0F, MDMCFG4=0x10..0x14,
// DEVIATN=0x15, MCSM0=0x18, FOCCFG=0x19.
// NB: в init_cc1101.pdf фигурировала константа обёртки ELECHOUSE
// «CC1101_IOCFG0» — это её собственное имя для адреса 0x02.
/// Базовая таблица (FREQ2/1/0 вычисляются отдельно — частота может
/// подстраиваться из конфига wx.rf_freq_mhz; см. Cc1101Driver::init).
/// Значения — из проверенного на живой станции легаси-конфига.
inline const RegVal* rxTableBase(size_t& count) {
    static const RegVal T[] = {
        { 0x00, 0x0E },  // IOCFG2:   GDO2 = Carrier Sense (шторка ISR)
        { 0x02, 0x0D },  // IOCFG0:   GDO0 = сырой демодулятор (данные)
        { 0x08, 0x32 },  // PKTCTRL0: async serial, infinite length
        { 0x0B, 0x08 },  // FSCTRL1:  ПЧ для полосы 270–325 кГц
        { 0x10, 0x2C },  // MDMCFG4:  RX BW = 325 кГц
        { 0x11, 0x44 },  // MDMCFG3:  скорость 17.241 кБод
        { 0x12, 0x02 },  // MDMCFG2:  2-FSK, sync off (мы в async)
        { 0x15, 0x45 },  // DEVIATN:  девиация 47.6 кГц
        { 0x18, 0x18 },  // MCSM0:    автокалибровка IDLE->RX
        { 0x19, 0x1D },  // FOCCFG:   коррекция частотного смещения (FSK)
    };
    count = sizeof(T) / sizeof(T[0]);
    return T;
}

} // namespace cc1101
