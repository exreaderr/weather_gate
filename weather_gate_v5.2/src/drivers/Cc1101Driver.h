// ============================================================================
// Cc1101Driver.h — ПРИЁМНИК CC1101 (E07-900M10S), СТРОГО RECEIVE-ONLY
// ============================================================================
// Стадия W2 профиля weather_gate. Доктрина радио (брифинг): передающих
// путей здесь НЕТ физически — ни STX, ни TX FIFO, ни автопереходов в TX.
// Единственный строб рабочего режима — SRX.
//
// Архитектура приёма:
//   · CC1101 в async-режиме выдаёт демодулированные биты на GDO0;
//   · GDO2 настроен на Carrier Sense (RSSI выше порога) — «шторка»:
//     ISR GDO0 записывает фронты в кольцо ТОЛЬКО пока шторка открыта.
//     Это защищает от шумового флуда async-выхода между пакетами
//     (пакет летит ~5 мс раз в ~48 с — остальное время эфир мусорит);
//   · ISR только меряет длительности (micros) и кладёт в SPSC-кольцо —
//     никаких getInstance()/millis()/heap в прерывании (правила платформы);
//   · poll() (из DriverRegistry) дрейнит кольцо в чистый декодер
//     FineOffsetCore и дедуплицирует второй пакет пары (31 мс).
//
// SPI: хост HSPI задаётся ЯВНО (правило нумерации classic ESP32:
// FSPI=1, HSPI=2, VSPI=3 — не полагаемся на умолчание). Пины — из
// WeatherGateProfile::pins() (SCK=14, MOSI=4, MISO=35, CS=17,
// GDO0=36, GDO2=39).
// ============================================================================
#pragma once

#include "../core/IDeviceDriver.h"
#include "FineOffsetCore.h"
#include "Cc1101Core.h"
#include <SPI.h>

// Частота опроса: дрейн кольца фронтов. Кольца на 512 фронтов хватает
// на два полных кадра (~176 фронтов каждый) с запасом под шум шторки.
constexpr uint32_t CC1101_POLL_MS   = 20;
constexpr uint16_t CC1101_EDGE_RING = 512;

class Cc1101Driver : public IDeviceDriver {
public:
    static Cc1101Driver& getInstance();

    // --- IDeviceDriver ---------------------------------------------------
    const char* driverName() const override { return "cc1101"; }
    bool init() override;
    void poll() override;
    uint32_t getPollIntervalMs() const override { return CC1101_POLL_MS; }
    bool isHealthy() const override { return _healthy; }

    // --- ДОСТУП К ПОСЛЕДНЕМУ ПАКЕТУ (вызывает WeatherGateApp, W3) ----------
    bool hasPacket()  const { return _pktSeq != 0; }
    const fo::WeatherPacket& lastPacket() const { return _lastPkt; }
    uint32_t packetSeq()    const { return _pktSeq; }
    uint32_t dupSeq()       const { return _dupSeq; }   // вторые пакеты пар
    uint32_t edgesDropped() const { return _edgesDropped; }
    uint32_t lastPacketMs() const { return _lastPktMs; }
    float    freqMHz()      const { return _freqMHz; }
    /// RSSI последнего принятого пакета, дБм.
    int16_t  rssiDbm()      const { return _rssiDbm; }

    // --- ТОЧКИ ВХОДА ISR (static-обёртки; НЕ для прикладного кода) -----------
    static void IRAM_ATTR isrGdo0();
    static void IRAM_ATTR isrGdo2();

private:
    Cc1101Driver() = default;

    // --- НИЗКИЙ УРОВЕНЬ SPI (CS всегда отпускается, транзакции короткие) ------
    uint8_t xferReg(uint8_t addr, uint8_t val);   // запись/строб
    uint8_t readStatus(uint8_t addr);             // статус-регистр (addr|0xC0)
    bool    detectChip();                          // PARTNUM + VERSION
    void    writeRxTable();                        // базовая таблица + FREQ
    int16_t readRssiDbm();                         // RSSI -> дБм

    // --- ISR-КОЛЬЦО (SPSC: писатель — ISR GDO0, читатель — poll) ---------------
    struct Edge { uint16_t durUs; uint8_t level; };
    static volatile uint16_t _wHead;
    static volatile uint16_t _rTail;
    static volatile bool     _csActive;            // шторка Carrier Sense
    static Edge              _ring[CC1101_EDGE_RING];

    // --- ДАННЫЕ -------------------------------------------------------------------
    fo::Decoder       _dec;
    fo::WeatherPacket _lastPkt{};
    uint8_t           _lastRaw[10]{};              // для дедупликации пары
    SPIClass*         _spi = nullptr;              // heap-блок из init (урок outbox)
    uint32_t _pktSeq       = 0;
    uint32_t _dupSeq       = 0;
    uint32_t _edgesDropped = 0;
    uint32_t _lastPktMs    = 0;
    int16_t  _rssiDbm      = 0;
    float    _freqMHz      = 915.0f;
    bool     _healthy      = false;
};
