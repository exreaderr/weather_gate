// ============================================================================
// WeatherGateApp.cpp — реализация прикладного модуля (стадия W3)
// ============================================================================
#include "WeatherGateApp.h"
#include "WeatherGateProfile.h"
#include "WeatherGateEvents.h"
#include <core/Kernel.h>
#include <core/ConformanceTest.h>
#include <core/EventBus.h>
#include <core/Events.h>
#include <services/ConfigService.h>
#include <services/HttpService.h>
#include <services/HealthMonitor.h>
#include <services/MqttTransport.h>
#include <services/DataLogService.h>
#include <services/TimeService.h>
#include <services/NetworkManager.h>
#include <drivers/Bme280Driver.h>
#include <drivers/Cc1101Driver.h>
#include <HTTPClient.h>              // авто-высота (одноразовая задача)
#include <WiFiClient.h>

// ============================================================================
// ПАЗ-ПРОВЕРКИ ДОМЕНА (IHealthCheck; HealthMonitor владеет механизмом,
// профиль — содержимым. Объекты статические: ПАЗ их не удаляет).
// ============================================================================

// Датчик давления: потерян (не отвечает после init) -> CRITICAL,
// чтения старше 2 минут -> WARNING. Доменные события — на переходах.
class WgSensorCheck : public IHealthCheck {
public:
    const char* checkName() const override { return "wg.bme280"; }
    uint32_t intervalMs() const override { return 5000; }
    HealthResult run() override {
        const Bme280Driver& d = Bme280Driver::getInstance();
        if (!d.isHealthy()) {
            if (!_lost) {
                _lost = true;
                EventBus::getInstance().post(wg_ev::sensorLost());
            }
            return HealthResult::critical("BME280_LOST");
        }
        if (_lost) {
            _lost = false;
            EventBus::getInstance().post(wg_ev::sensorRestored());
        }
        if (d.lastReadMs() == 0) return HealthResult::ok();  // ещё не читал
        if (millis() - d.lastReadMs() > 120000UL)
            return HealthResult::warning("BME280_STALE");
        return HealthResult::ok();
    }
private:
    bool _lost = false;
};

// Радиотракт: чип потерян -> CRITICAL; тишина эфира дольше wx.silence_min ->
// WARNING, дольше двух таймаутов -> CRITICAL. После boot — льгота 5 минут
// (станция шлёт раз в ~48 с; первый пакет может опоздать).
class WgRadioCheck : public IHealthCheck {
public:
    const char* checkName() const override { return "wg.radio"; }
    uint32_t intervalMs() const override { return 10000; }
    HealthResult run() override {
        const Cc1101Driver& r = Cc1101Driver::getInstance();
        if (!r.isHealthy()) return HealthResult::critical("CC1101_LOST");

        uint32_t silenceMs =
            cfgGetUInt("wx.silence_min", 15) * 60000UL;
        if (r.lastPacketMs() == 0) {
            // Ни одного пакета со старта
            if (millis() > 5UL * 60 * 1000) {
                markSilent();
                return HealthResult::warning("RADIO_NO_PACKETS");
            }
            return HealthResult::ok();
        }
        uint32_t age = millis() - r.lastPacketMs();
        if (age > silenceMs * 2) {
            markSilent();
            return HealthResult::critical("RADIO_DARK");
        }
        if (age > silenceMs) {
            markSilent();
            return HealthResult::warning("RADIO_SILENCE");
        }
        _silenced = false;
        return HealthResult::ok();
    }
private:
    void markSilent() {
        if (!_silenced) {
            _silenced = true;
            EventBus::getInstance().post(wg_ev::radioSilence());
        }
    }
    bool _silenced = false;
};

static WgSensorCheck s_sensorCheck;
static WgRadioCheck  s_radioCheck;

// ============================================================================
// UI-ПРОВАЙДЕР
// ============================================================================
size_t WeatherGateUi::renderPublicHtml(char* buf, size_t bufSize) {
    // Бюджет публичной секции ~2 КБ (правило ядра). Улица + давление + эфир.
    const WeatherGateApp& app = WeatherGateApp::getInstance();
    const Bme280Driver&   d   = Bme280Driver::getInstance();
    const Cc1101Driver&   r   = Cc1101Driver::getInstance();
    int n = snprintf(buf, bufSize, "<b>Погодный шлюз</b>");
    if (n <= 0) return 0;
    size_t pos = (size_t)n;

    const WeatherGateApp::Outdoor& o = app.outdoor();
    if (o.valid) {
        n = snprintf(buf + pos, bufSize - pos,
            "<p>На улице: %.1f&deg;C (ощущ. %.1f), влажн. %.0f%%, "
            "ветер %.1f м/с, %u&deg;, дождь %.1f мм/ч%s<br>"
            "<small>обновлено %lu с назад, RSSI %d дБм</small></p>",
            (double)o.tempC, (double)app.feelsLikeC(),
            (double)o.humidityPct, (double)o.windMs, (unsigned)o.dirDeg,
            (double)o.rainMmPh,
            o.batteryLow ? " <b>&#9888; батарея</b>" : "",
            (unsigned long)((millis() - o.rxMs) / 1000), (int)r.rssiDbm());
        if (n > 0) pos += (size_t)n;
    } else {
        n = snprintf(buf + pos, bufSize - pos,
                     "<p>Улица: пакетов ещё не было.</p>");
        if (n > 0) pos += (size_t)n;
    }
    if (d.isHealthy()) {
        n = snprintf(buf + pos, bufSize - pos,
            "<p>Давление: %.0f мм рт.ст. (%.1f гПа у.м.)</p>",
            (double)(d.pressureSeaHpa() * 0.750062f),
            (double)d.pressureSeaHpa());
        if (n > 0) pos += (size_t)n;
    }
    n = snprintf(buf + pos, bufSize - pos,
                 "<p><a href=\"/web/wx.html\">Панель погоды и графики</a></p>");
    if (n > 0) pos += (size_t)n;
    return pos;
}

// Рабочий буфер даталог-запросов — BSS файла, НЕ стек HTTP-задачи.
// UNION как в smart_lock: сырые точки и агрегаты в одном запросе не
// встречаются (постмортем: два раздельных буфера ломали линковку DRAM).
union WgDlogQueryBuf {
    DlogPoint raw[DLOG_RAW_CAP];
    DlogAggr  aggr[320];
};
static WgDlogQueryBuf s_dlogQ;

static bool wgApiDlogChannels(char* buf, size_t size) {
    DataLogService& dl = DataLogService::getInstance();
    size_t pos = 0;
    int n = snprintf(buf, size, "{\"channels\":[");
    if (n < 0) return true;
    pos = (size_t)n;
    for (uint8_t i = 0; i < dl.channelCount(); ++i) {
        char id[12], name[28], unit[8];
        if (!dl.channelInfo(i, id, sizeof(id), name, sizeof(name),
                            unit, sizeof(unit))) continue;
        n = snprintf(buf + pos, size - pos,
            "%s{\"i\":%u,\"id\":\"%s\",\"name\":\"%s\",\"unit\":\"%s\"}",
            i ? "," : "", i, id, name, unit);
        if (n < 0 || (size_t)n >= size - pos) break;
        pos += (size_t)n;
    }
    snprintf(buf + pos, size - pos, "]}");
    return true;
}

static bool wgApiDlog(const ShUiRequest& req, char* buf, size_t size,
                      int& status) {
    DataLogService& dl = DataLogService::getInstance();
    const char* chArg = req.getArg("ch");
    uint8_t ch = chArg ? (uint8_t)atoi(chArg) : 0;
    if (ch >= dl.channelCount()) {
        status = 404;
        snprintf(buf, size, "{\"err\":\"no_channel\"}");
        return true;
    }
    // Диапазон -> ярус и фильтр времени (паттерн smart_lock).
    const char* range = req.getArg("range");
    uint32_t now = (uint32_t)TimeService::getInstance().getUnixTime();
    uint32_t fromTs = 0;
    bool raw = true, daily = false;
    if (range == nullptr || strcmp(range, "6h") == 0) {
        fromTs = now > 6UL * 3600 ? now - 6UL * 3600 : 0;
    } else if (strcmp(range, "24h") == 0) {
        raw = false; fromTs = now > 24UL * 3600 ? now - 24UL * 3600 : 0;
    } else if (strcmp(range, "7d") == 0) {
        raw = false; fromTs = now > 7UL * 86400 ? now - 7UL * 86400 : 0;
    } else if (strcmp(range, "30d") == 0) {
        raw = false; fromTs = 0;
    } else if (strcmp(range, "1y") == 0) {
        raw = false; daily = true; fromTs = 0;
    } else {
        status = 400;
        snprintf(buf, size, "{\"err\":\"range: 6h|24h|7d|30d|1y\"}");
        return true;
    }

    size_t pos = 0;
    int n;
    if (raw) {
        uint16_t cnt = dl.getRaw(ch, s_dlogQ.raw, DLOG_RAW_CAP, fromTs);
        cnt = dlog::decimateRaw(s_dlogQ.raw, cnt, s_dlogQ.raw,
                                DLOG_JSON_POINTS);
        n = snprintf(buf, size, "{\"fmt\":\"raw\",\"n\":%u,\"ts\":[", cnt);
        if (n < 0) return true;
        pos = (size_t)n;
        for (uint16_t i = 0; i < cnt; ++i) {
            n = snprintf(buf + pos, size - pos, "%s%lu", i ? "," : "",
                         (unsigned long)s_dlogQ.raw[i].ts);
            if (n < 0 || (size_t)n >= size - pos) { pos = size - 8; break; }
            pos += (size_t)n;
        }
        n = snprintf(buf + pos, size - pos, "],\"v\":[");
        if (n > 0) pos += (size_t)n;
        for (uint16_t i = 0; i < cnt; ++i) {
            n = snprintf(buf + pos, size - pos, "%s%.1f", i ? "," : "",
                         (double)s_dlogQ.raw[i].v);
            if (n < 0 || (size_t)n >= size - pos) { pos = size - 8; break; }
            pos += (size_t)n;
        }
        snprintf(buf + pos, size - pos, "]}");
        return true;
    }

    uint16_t cnt = dl.getTier(ch, daily, s_dlogQ.aggr, 320, fromTs);
    cnt = dlog::decimateAggr(s_dlogQ.aggr, cnt, s_dlogQ.aggr,
                             DLOG_JSON_POINTS);
    n = snprintf(buf, size, "{\"fmt\":\"aggr\",\"n\":%u,\"ts\":[", cnt);
    if (n < 0) return true;
    pos = (size_t)n;
    for (uint16_t i = 0; i < cnt; ++i) {
        n = snprintf(buf + pos, size - pos, "%s%lu", i ? "," : "",
                     (unsigned long)s_dlogQ.aggr[i].ts);
        if (n < 0 || (size_t)n >= size - pos) { pos = size - 16; break; }
        pos += (size_t)n;
    }
    static const char* KEYS[3] = { "\"avg\":[", "\"min\":[", "\"max\":[" };
    for (uint8_t k = 0; k < 3; ++k) {
        n = snprintf(buf + pos, size - pos, "],%s", KEYS[k]);
        if (n > 0) pos += (size_t)n;
        for (uint16_t i = 0; i < cnt; ++i) {
            float v = k == 0 ? s_dlogQ.aggr[i].avg :
                      k == 1 ? s_dlogQ.aggr[i].mn  : s_dlogQ.aggr[i].mx;
            n = snprintf(buf + pos, size - pos, "%s%.1f", i ? "," : "",
                         (double)v);
            if (n < 0 || (size_t)n >= size - pos) { pos = size - 16; break; }
            pos += (size_t)n;
        }
    }
    snprintf(buf + pos, size - pos, "]}");
    return true;
}

bool WeatherGateUi::handleApi(const char* pathTail, const ShUiRequest& req,
                              char* responseBuf, size_t bufSize,
                              int& statusCode) {
    if (strcmp(pathTail, "ping") == 0) {
        snprintf(responseBuf, bufSize, "{\"pong\":1,\"ms\":%lu}",
                 (unsigned long)millis());
        statusCode = 200;
        return true;
    }
    if (strcmp(pathTail, "wx") == 0) {
        // Текущие показания BME280 (диагностика датчика)
        const Bme280Driver& d = Bme280Driver::getInstance();
        char t[12] = "null", p[12] = "null", ps[12] = "null", h[12] = "null";
        if (d.isHealthy() && d.lastReadMs() != 0) {
            snprintf(t,  sizeof(t),  "%.2f", (double)d.temperatureC());
            snprintf(p,  sizeof(p),  "%.2f", (double)d.pressureHpa());
            snprintf(ps, sizeof(ps), "%.2f", (double)d.pressureSeaHpa());
            if (d.humidityValid())
                snprintf(h, sizeof(h), "%.1f", (double)d.humidityPct());
        }
        snprintf(responseBuf, bufSize,
                 "{\"model\":\"%s\",\"addr\":\"0x%02X\",\"temp\":%s,"
                 "\"press\":%s,\"press_sea\":%s,\"humidity\":%s,"
                 "\"age_ms\":%lu}",
                 d.model(), (unsigned)d.address(), t, p, ps, h,
                 d.lastReadMs() ? (unsigned long)(millis() - d.lastReadMs())
                                : 0UL);
        statusCode = 200;
        return true;
    }
    if (strcmp(pathTail, "radio") == 0) {
        // Статус приёмника CC1101 (диагностика эфира)
        const Cc1101Driver& r = Cc1101Driver::getInstance();
        snprintf(responseBuf, bufSize,
                 "{\"healthy\":%d,\"freq\":%.2f,\"pkt\":%lu,\"dup\":%lu,"
                 "\"edges_dropped\":%lu,\"rssi\":%d,\"age_ms\":%lu}",
                 r.isHealthy() ? 1 : 0, (double)r.freqMHz(),
                 (unsigned long)r.packetSeq(), (unsigned long)r.dupSeq(),
                 (unsigned long)r.edgesDropped(), (int)r.rssiDbm(),
                 r.lastPacketMs() ? (unsigned long)(millis() - r.lastPacketMs())
                                  : 0UL);
        statusCode = 200;
        return true;
    }
    if (strcmp(pathTail, "weather") == 0) {
        // Контракт smart_lock + расширения (публичный: погода — не секрет)
        WeatherGateApp::getInstance().weatherJson(responseBuf, bufSize);
        statusCode = 200;
        return true;
    }
    // Дальше — только админ (графики = история, паттерн smart_lock)
    if (!HttpService::getInstance().isAdminToken(req.token)) {
        return false;   // 404 ядра: не раскрываем существование путей
    }
    if (strcmp(pathTail, "dlog/channels") == 0) {
        statusCode = 200;
        return wgApiDlogChannels(responseBuf, bufSize);
    }
    if (strcmp(pathTail, "dlog") == 0) {
        statusCode = 200;
        return wgApiDlog(req, responseBuf, bufSize, statusCode);
    }
    return false;   // неизвестный профильный путь -> 404 ядра
}

// ============================================================================
// МОДУЛЬ
// ============================================================================
volatile bool WeatherGateApp::s_altTaskRunning = false;

WeatherGateApp& WeatherGateApp::getInstance() {
    static WeatherGateApp instance;
    return instance;
}

void WeatherGateApp::registerExtensions() {
    // Конфиг-схема профиля. Новые поля — ТОЛЬКО в конец списка (правило
    // JSON-конфига: сдвиг индексов ломает сохранённые значения).
    bool ok = ConfigService::getInstance().addFields("Датчик BME280", {
        {"wx.i2c_addr",        ConfigType::STRING, "auto", 0, 0,
         CFG_CRITICAL, "Датчик BME280", "Адрес I2C (auto/0x76/0x77)"},
        {"wx.altitude_m",      ConfigType::FLOAT, "0", -500, 9000,
         CFG_NONE, "Датчик BME280", "Высота установки над у.м., м"},
        {"wx.press_offset_hpa", ConfigType::FLOAT, "0", -50, 50,
         CFG_NONE, "Датчик BME280", "Поправка давления, гПа"},
        {"wx.lat",             ConfigType::FLOAT, "0", -90, 90,
         CFG_NONE, "Датчик BME280", "Широта (для авто-высоты)"},
        {"wx.lon",             ConfigType::FLOAT, "0", -180, 180,
         CFG_NONE, "Датчик BME280", "Долгота (для авто-высоты)"},
    });
    if (!ok) {
        // fail-fast: схема не влезла/поле отвергнуто — стоп, смотреть
        // CFG_MAX_FIELDS и валидность описаний (правило руководства).
        log(LogLevel::Error, "addFields 'Датчик BME280' failed");
    }

    // W2: радиотракт CC1101.
    ok = ConfigService::getInstance().addFields("Радио CC1101", {
        {"wx.rf_freq_mhz",     ConfigType::FLOAT, "915.00", 914, 916,
         CFG_CRITICAL, "Радио CC1101", "Частота приёма, МГц"},
    });
    if (!ok) log(LogLevel::Error, "addFields 'Радио CC1101' failed");

    // W3: телеметрия и сторожа.
    ok = ConfigService::getInstance().addFields("Телеметрия и сторожа", {
        {"wx.mqtt_en",         ConfigType::BOOL, "1", 0, 0,
         CFG_NONE, "Телеметрия и сторожа", "Публикация weather-JSON в MQTT"},
        {"wx.pub_min",         ConfigType::UINT, "5", 1, 60,
         CFG_NONE, "Телеметрия и сторожа", "Период retained-публикации, мин"},
        {"wx.silence_min",     ConfigType::UINT, "15", 2, 120,
         CFG_NONE, "Телеметрия и сторожа", "Тишина эфира -> тревога, мин"},
        {"wx.autoalt_en",      ConfigType::BOOL, "1", 0, 0,
         CFG_NONE, "Телеметрия и сторожа",
         "Авто-высота по координатам (при FULL-сети)"},
    });
    if (!ok) log(LogLevel::Error, "addFields 'Телеметрия и сторожа' failed");

    // ПАЗ-проверки домена (механизм — HealthMonitor, содержимое — профиль)
    HealthMonitor::getInstance().registerCheck(&s_sensorCheck);
    HealthMonitor::getInstance().registerCheck(&s_radioCheck);

    HttpService::getInstance().setUiProvider(&WeatherGateUi::getInstance());
}

void WeatherGateApp::init() {
    _initialized = true;   // ресурсов у модуля нет; железо — у драйверов
    log(LogLevel::Info, "init: profile weather_gate, stage W3");
}

void WeatherGateApp::start() {
    _started = true;

    // События инфраструктуры
    EventBus::getInstance().subscribe(SH_EVENT_DEGRADED_LEVEL, this);

    // Каналы даталоггера (сервис пассивен — каналы объявляет профиль)
    DataLogService& dlog = DataLogService::getInstance();
    _chOutT  = dlog.registerChannel("wx_ot", "Улица, температура", "°C");
    _chOutH  = dlog.registerChannel("wx_oh", "Улица, влажность", "%");
    _chPress = dlog.registerChannel("wx_p",  "Давление у.м.", "гПа");
    _chWind  = dlog.registerChannel("wx_w",  "Ветер", "м/с");
    _chRain  = dlog.registerChannel("wx_r",  "Дождь", "мм/ч");

    // Авто-высота: если сеть уже FULL (старт после стабильной линии)
    maybeRequestAltitude();

    // Стенд соответствия D1: манифест собираем заново из пинов профиля
    // (describeHardware чист — только константы, без железа).
    HardwareManifest m;
    WeatherGateProfile p;
    p.describeHardware(m);
    conformance::runAll("weather_gate", m);
}

void WeatherGateApp::stop() {
    EventBus::getInstance().unsubscribeAll(this);
    _started = false;
    _initialized = false;
}

bool WeatherGateApp::canHandleEvent(int32_t id) const {
    return id == SH_EVENT_DEGRADED_LEVEL;
}

void WeatherGateApp::onEvent(int32_t eventId, const ShEventData* data) {
    if (eventId == SH_EVENT_DEGRADED_LEVEL) {
        // Сеть поднялась до FULL — повод для авто-высоты
        if (data != nullptr && data->code == (int32_t)DegradationLevel::Full)
            maybeRequestAltitude();
    }
}

// ============================================================================
// TICK (1 с): новые пакеты, даталог давления, периодическая публикация,
// запуск задачи авто-высоты. Бюджет 50 мс — всё короткое, HTTP уехал
// в отдельную задачу.
// ============================================================================
void WeatherGateApp::tick() {
    uint32_t now = millis();

    // Новый радиопакет?
    const Cc1101Driver& r = Cc1101Driver::getInstance();
    if (r.hasPacket() && r.packetSeq() != _seenPktSeq) {
        _seenPktSeq = r.packetSeq();
        onNewRadioPacket();
    }

    // Давление — по своему ритму (раз в минуту), независимо от эфира
    const Bme280Driver& d = Bme280Driver::getInstance();
    if (_chPress >= 0 && d.isHealthy() && d.lastReadMs() != 0 &&
        now - _lastPressLogMs >= 60000UL) {
        _lastPressLogMs = now;
        DataLogService::getInstance().logPoint(_chPress, d.pressureSeaHpa());
    }

    // Периодическая retained-публикация (давление дрейфует и без пакетов;
    // retained-топик должен быть свежим для подписчиков после ребута УД)
    uint32_t pubMs = cfgGetUInt("wx.pub_min", 5) * 60000UL;
    if (cfgGetBool("wx.mqtt_en", true) && _out.valid &&
        now - _lastPubMs >= pubMs) {
        publishWeatherMqtt();
    }

    // Авто-высота: запуск одноразовой задачи (loop не блокируем)
    if (_altRequested && !_altDone && !s_altTaskRunning &&
        now >= _altNextRetryMs) {
        _altRequested = false;
        xTaskCreate(&WeatherGateApp::altitudeTask, "wg_alt", 8192,
                    nullptr, 1, nullptr);
    }
}

// ============================================================================
// ОБРАБОТКА ПАКЕТА
// ============================================================================
void WeatherGateApp::onNewRadioPacket() {
    const Cc1101Driver& r = Cc1101Driver::getInstance();
    const fo::WeatherPacket& p = r.lastPacket();
    TimeService& ts = TimeService::getInstance();
    uint32_t unix = ts.isTimeValid() ? (uint32_t)ts.getUnixTime() : 0;

    _out.valid      = true;
    _out.rxMs       = millis();
    _out.tempC      = p.tempC;
    _out.humidityPct= p.humidity;
    _out.windMs     = p.windMs;
    _out.gustMs     = p.gustMs;
    _out.dirDeg     = p.dirDeg;
    _out.batteryLow = p.batteryLow;
    _out.deviceId   = p.deviceId;
    if (unix != 0) {
        _rain.add(p.rainRaw, unix);
        _out.rainMmPh = _rain.rateMmPh(unix);
    }

    // Даталог уличных каналов (logPoint сам отбросит точку без времени)
    DataLogService& dl = DataLogService::getInstance();
    if (_chOutT >= 0) dl.logPoint(_chOutT, p.tempC);
    if (_chOutH >= 0) dl.logPoint(_chOutH, p.humidity);
    if (_chWind >= 0) dl.logPoint(_chWind, p.windMs);
    if (_chRain >= 0) dl.logPoint(_chRain, _out.rainMmPh);

    // MQTT + событие шины
    if (cfgGetBool("wx.mqtt_en", true)) publishWeatherMqtt();
    ShEventData ev;
    ev.clear();
    ev.sourceModule = getModuleId();
    snprintf(ev.payload, sizeof(ev.payload), "T=%.1f W=%.1f R=%.1f",
             (double)p.tempC, (double)p.windMs, (double)_out.rainMmPh);
    EventBus::getInstance().post(wg_ev::radioPacket(), &ev);
}

// ============================================================================
// WEATHER-JSON (контракт smart_lock + расширения)
// ============================================================================
float WeatherGateApp::feelsLikeC() const {
    return wxc::feelsLikeC(_out.tempC, _out.humidityPct, _out.windMs);
}

const char* WeatherGateApp::weatherState() const {
    return wxc::weatherState(_out.rainMmPh, _out.windMs, _out.gustMs);
}

size_t WeatherGateApp::weatherJson(char* buf, size_t bufSize) const {
    if (!_out.valid) {
        int n = snprintf(buf, bufSize, "{\"valid\":0}");
        return n > 0 ? (size_t)n : 0;
    }
    const Bme280Driver& d = Bme280Driver::getInstance();
    const Cc1101Driver& r = Cc1101Driver::getInstance();
    char p[12] = "null", ps[12] = "null";
    if (d.isHealthy() && d.lastReadMs() != 0) {
        snprintf(p,  sizeof(p),  "%.2f", (double)d.pressureHpa());
        snprintf(ps, sizeof(ps), "%.2f", (double)d.pressureSeaHpa());
    }
    // Бюджет MQTT_BODY_LEN (256): строка ~190 байт с запасом.
    int n = snprintf(buf, bufSize,
        "{\"valid\":1,\"temp\":%.2f,\"feels_like\":%.2f,\"state\":\"%s\","
        "\"humidity\":%.1f,\"wind\":%.2f,\"gust\":%.2f,\"dir\":%u,"
        "\"rain\":%.2f,\"press\":%s,\"press_sea\":%s,"
        "\"rssi\":%d,\"batt\":%d,\"age_s\":%lu}",
        (double)_out.tempC, (double)feelsLikeC(), weatherState(),
        (double)_out.humidityPct, (double)_out.windMs, (double)_out.gustMs,
        (unsigned)_out.dirDeg, (double)_out.rainMmPh, p, ps,
        (int)r.rssiDbm(), _out.batteryLow ? 0 : 1,
        (unsigned long)((millis() - _out.rxMs) / 1000));
    return n > 0 ? (size_t)n : 0;
}

void WeatherGateApp::publishWeatherMqtt() {
    if (!_out.valid) return;
    char js[MQTT_BODY_LEN];
    weatherJson(js, sizeof(js));
    // retained: подписчик (smart_lock, HA) получает свежую погоду сразу
    // после подписки, не дожидаясь следующего пакета
    MqttTransport::getInstance().publishStateSuffix("weather", js, true);
    _lastPubMs = millis();
}

// ============================================================================
// АВТО-ВЫСОТА (wx.lat/wx.lon -> wx.altitude_m)
// ============================================================================
// Одноразовый HTTP GET api.open-meteo.com/v1/elevation. Условия: сеть FULL,
// высота не задана (0), координаты заданы, не записывали ранее. Запись —
// через ConfigService::set: валидация диапазона + персистентность (после
// ребута повторный запрос не нужен); Bme280Driver читает поле каждый цикл
// и подхватывает значение мгновенно. Неудача -> повтор не раньше часа.
void WeatherGateApp::maybeRequestAltitude() {
    if (_altDone || !cfgGetBool("wx.autoalt_en", true)) return;
    if (cfgGetFloat("wx.altitude_m", 0.0f) != 0.0f) { _altDone = true; return; }
    if (cfgGetFloat("wx.lat", 0.0f) == 0.0f &&
        cfgGetFloat("wx.lon", 0.0f) == 0.0f) return;   // координат нет — ждём
    if (NetworkService::getInstance().degradationLevel() !=
        DegradationLevel::Full) return;
    _altRequested = true;
}

void WeatherGateApp::altitudeTask(void*) {
    s_altTaskRunning = true;
    WeatherGateApp& self = WeatherGateApp::getInstance();
    float lat = cfgGetFloat("wx.lat", 0.0f);
    float lon = cfgGetFloat("wx.lon", 0.0f);
    char url[160];
    snprintf(url, sizeof(url),
             "http://api.open-meteo.com/v1/elevation?latitude=%.4f&longitude=%.4f",
             (double)lat, (double)lon);

    WiFiClient client;
    HTTPClient http;
    http.setTimeout(4000);
    bool ok = false;
    if (http.begin(client, url)) {
        if (http.GET() == 200) {
            // Тело ~40 байт: {"elevation":[123.0]}. Буфер, не String
            // (правило платформы — без динамики даже в одноразовой задаче).
            char body[128];
            size_t got = http.getStream().readBytes(body, sizeof(body) - 1);
            body[got] = '\0';
            const char* p = strstr(body, "\"elevation\":[");
            if (p != nullptr) {
                float v = (float)atof(p + 13);
                if (v > -500.0f && v < 9000.0f) {
                    char val[16];
                    snprintf(val, sizeof(val), "%.0f", (double)v);
                    if (ConfigService::getInstance().set("wx.altitude_m", val)) {
                        self._altDone = true;
                        ok = true;
                        self.log(LogLevel::Info,
                                 "auto-altitude: %.0f m by %.4f,%.4f",
                                 (double)v, (double)lat, (double)lon);
                    }
                }
            }
        }
        http.end();
    }
    if (!ok) {
        self._altNextRetryMs = millis() + 3600000UL;   // повтор через час
        self.log(LogLevel::Warning, "auto-altitude: fetch failed, retry in 1h");
    }
    s_altTaskRunning = false;
    vTaskDelete(nullptr);
}
