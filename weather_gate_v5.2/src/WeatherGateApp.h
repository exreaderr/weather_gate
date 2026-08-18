// ============================================================================
// WeatherGateApp.h — ПРИКЛАДНОЙ МОДУЛЬ WEATHER_GATE (стадия W3: полный профиль)
// ============================================================================
// W1: конфиг-схема датчика, минимальный UI, conformance-стенд.
// W2: радиотракт CC1101 + декодер Fine Offset, endpoint /api/dev/radio.
// W3: агрегация уличных показаний (пакет -> снимок + производные:
//     feels-like, интенсивность дождя, код состояния HA), weather-JSON
//     для smart_lock (HTTP /api/dev/weather + MQTT <prefix>/<id>/weather,
//     retained), каналы DataLog (графики uPlot), ПАЗ-проверки домена
//     (датчик давления, молчание эфира), авто-высота по координатам
//     при FULL-сети (разовая запись wx.altitude_m).
// Замбретти — W5.
// ============================================================================
#pragma once

#include <core/ModuleBase.h>
#include <services/IUiProvider.h>
#include <drivers/WeatherCore.h>

// ============================================================================
// UI-ПРОВАЙДЕР ПРОФИЛЯ
// ============================================================================
class WeatherGateUi : public IUiProvider {
public:
    static WeatherGateUi& getInstance() {
        static WeatherGateUi instance;
        return instance;
    }

    const char* uiTitle() const override { return "weather_gate"; }

    /// Публичная карточка на "/" (бюджет ~2 КБ): улица + давление + эфир.
    size_t renderPublicHtml(char* buf, size_t bufSize) override;

    bool handleApi(const char* pathTail, const ShUiRequest& req,
                   char* responseBuf, size_t bufSize,
                   int& statusCode) override;

private:
    WeatherGateUi() = default;
};

// ============================================================================
// МОДУЛЬ
// ============================================================================
class WeatherGateApp : public ModuleBase {
public:
    static WeatherGateApp& getInstance();

    // --- IModule ---------------------------------------------------------
    const char* getName() const override { return "WeatherGateApp"; }
    const char* getVersion() const override { return "0.3.0"; }   // стадия W3
    ModuleId getModuleId() const override { return 0x1000; }      // приложения

    void registerExtensions() override;   // конфиг wx.*, UI, ПАЗ-проверки
    void init() override;
    void start() override;                // подписки, DataLog, conformance
    void stop() override;
    void tick() override;                 // 1 с: пакеты, лог, публикации
    uint32_t getTickIntervalMs() const override { return 1000; }
    void onEvent(int32_t eventId, const ShEventData* data) override;
    bool canHandleEvent(int32_t eventId) const override;

    // --- СНИМОК УЛИЦЫ (для UI и weather-JSON) ------------------------------
    struct Outdoor {
        bool     valid = false;     // был хотя бы один валидный пакет
        uint32_t rxMs = 0;          // millis() приёма
        float    tempC = 0;
        float    humidityPct = 0;
        float    windMs = 0;
        float    gustMs = 0;
        float    rainMmPh = 0;      // интенсивность за часовое окно
        uint16_t dirDeg = 0;
        bool     batteryLow = false;
        uint8_t  deviceId = 0;
    };
    const Outdoor& outdoor() const { return _out; }

    float       feelsLikeC() const;   // wxc::feelsLikeC от снимка
    const char* weatherState() const; // код HA (wxc::weatherState)

    /// Контракт smart_lock {"temp","feels_like","state"} + расширения
    /// (humidity, wind, gust, dir, rain, press_sea, press, rssi, age_s,
    /// batt). smart_lock читает свои три поля — расширения безопасны.
    /// Возвращает длину; если улица невалидна — {"valid":0} (публикация
    /// в MQTT при этом НЕ делается — см. tick).
    size_t weatherJson(char* buf, size_t bufSize) const;

private:
    WeatherGateApp() = default;

    void onNewRadioPacket();          // пакет -> снимок, лог, MQTT, событие
    void publishWeatherMqtt();        // retained <prefix>/<id>/weather

    // --- АВТО-ВЫСОТА (wx.lat/wx.lon -> wx.altitude_m, разово) ---------------
    void maybeRequestAltitude();      // условия + постановка флага
    static void altitudeTask(void*);  // одноразовая задача: HTTP GET,
                                      // парсинг, ConfigService::set

    Outdoor          _out;
    wxc::RainTracker _rain;
    uint32_t _seenPktSeq     = 0;
    uint32_t _lastPubMs      = 0;
    uint32_t _lastPressLogMs = 0;

    // Каналы DataLog (-1 — не зарегистрирован)
    int8_t _chOutT = -1, _chOutH = -1, _chPress = -1, _chWind = -1, _chRain = -1;

    // Авто-высота
    bool _altRequested  = false;      // есть повод попробовать
    bool _altDone       = false;      // успешно записана (больше не лезем)
    uint32_t _altNextRetryMs = 0;     // неудача -> повтор не раньше часа
    static volatile bool s_altTaskRunning;
};
