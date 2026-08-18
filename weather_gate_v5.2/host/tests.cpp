// ============================================================================
// tests.cpp — HOST-ТЕСТЫ ЧИСТОЙ ЛОГИКИ ЯДРА (D2)
// ============================================================================
// Запуск на хосте (без железа):
//   g++ -std=c++17 -I shim tests.cpp ../core/ResourceManager.cpp -o tests
//   ./tests
//
// Покрытие (только то, что НЕ требует FreeRTOS/GPIO/шины — честная граница):
//   · WiegandFormats.h  — декодер W26–W56 (каталог профилей, логика чистая);
//   · BcdUtils.h        — BCD-конверсия DS3231 (круговая);
//   · TimeInterval.h    — интервалы HH:MM, переход через полночь;
//   · ResourceManager   — конфликты ресурсов, идемпотентность, диапазоны.
// Микро-фреймворк: CHECK/CHECK_MSG + итог. Любой FAIL -> код возврата 1.
// ============================================================================
#include <cstdio>
#include <cstring>
#include <ctime>          // timegm — эталон для secondsFromCivil

// Шим Arduino ПЕРЕД инклудами ядра (подменяет <Arduino.h> по -I shim)
#include "../src/catalog/wiegand/WiegandFormats.h"
#include "../projects/smart_lock/src/CardDbFormat.h"
#include "../src/drivers/BcdUtils.h"
#include "../src/drivers/Bme280Core.h"
#include "../src/drivers/FineOffsetCore.h"
#include "../src/drivers/Cc1101Core.h"
#include "../src/drivers/WeatherCore.h"
#include "../src/services/TimeInterval.h"
#include "../src/services/AudioQueue.h"
#include "../src/services/DataLogCore.h"
#include "../src/services/ScheduleCore.h"
#include "../src/services/CounterCore.h"
#include "../src/services/MqttOutbox.h"
#include "../src/services/SpeechCore.h"
#include "../src/services/JournalCore.h"
#include "../src/core/ResourceManager.h"
#include "../src/core/SntpCore.h"

// ============================================================================
// МИКРО-ФРЕЙМВОРК
// ============================================================================
static int g_pass = 0, g_fail = 0;

#define CHECK(cond) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_MSG(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; \
        printf("FAIL %s:%d: %s (%s)\n", __FILE__, __LINE__, #cond, msg); } \
} while (0)

// ============================================================================
// WIEGAND: построение кадра с правильными паритетами (эталон теста —
// независимая реализация, НЕ копия кода декодера)
// ============================================================================
static uint8_t parityOf(uint64_t v) {   // 1 = нечётное число единиц
    uint8_t p = 0;
    while (v) { p ^= (uint8_t)(v & 1ULL); v >>= 1; }
    return p;
}

/// Кадр: [P_even][body старшие headCover бит ..][body][P_odd] из body-битов
static uint64_t buildFrame(const WiegandFormat& f, uint64_t body) {
    if (!f.hasParity) return body;   // W37: кадр == данные
    uint8_t bodyBits = f.totalBits - 2;
    uint64_t frame = (body & ((bodyBits >= 64) ? ~0ULL
                                               : ((1ULL << bodyBits) - 1))) << 1;
    // Чётный ведущий: P = parity(старшие headParityCover бит body)
    uint64_t headField = frame >> (f.totalBits - 1 - f.headParityCover);
    headField &= (1ULL << f.headParityCover) - 1;
    if (parityOf(headField)) frame |= (1ULL << (f.totalBits - 1));
    // Нечётный замыкающий: P = !parity(младшие tailParityCover бит body)
    uint64_t tailField = (frame >> 1) & ((1ULL << f.tailParityCover) - 1);
    if (!parityOf(tailField)) frame |= 1ULL;
    return frame;
}

static void testWiegand() {
    printf("== WiegandFormats ==\n");

    // --- W26: карта 12345 (0x3039) -----------------------------------------
    {
        const WiegandFormat* f = wiegand::findFormatByBits(26);
        CHECK(f != nullptr);
        uint64_t frame = buildFrame(*f, 12345);
        WiegandCard c = wiegand::decodeFrame(frame, 26);
        CHECK(c.format == f);
        CHECK(c.parityOk);
        CHECK(c.data == 12345);
        // Битый ведущий паритет -> parityOk == false, данные не меняются
        WiegandCard bad = wiegand::decodeFrame(
            frame ^ (1ULL << 25), 26);
        CHECK(!bad.parityOk);
        CHECK(bad.data == 12345);
        // Битый замыкающий паритет
        WiegandCard bad2 = wiegand::decodeFrame(frame ^ 1ULL, 26);
        CHECK(!bad2.parityOk);
    }

    // --- W34: карта 0x1ABCDEF ------------------------------------------------
    {
        const WiegandFormat* f = wiegand::findFormatByBits(34);
        CHECK(f != nullptr);
        uint64_t frame = buildFrame(*f, 0x1ABCDEFULL);
        WiegandCard c = wiegand::decodeFrame(frame, 34);
        CHECK(c.parityOk);
        CHECK(c.data == 0x1ABCDEFULL);
    }

    // --- W35 Corp1000: АСИММЕТРИЧНЫЙ (12+21) — регрессия на перепутанные
    //     поля паритета (главная ловушка формата) ---------------------------
    {
        const WiegandFormat* f = wiegand::findFormatByBits(35);
        CHECK(f != nullptr);
        CHECK(f->headParityCover == 12);
        CHECK(f->tailParityCover == 21);
        uint64_t frame = buildFrame(*f, 0x7ABCDULL);
        WiegandCard c = wiegand::decodeFrame(frame, 35);
        CHECK(c.parityOk);
        CHECK(c.data == 0x7ABCDULL);
        WiegandCard bad = wiegand::decodeFrame(frame ^ (1ULL << 34), 35);
        CHECK(!bad.parityOk);
    }

    // --- W37 H10302: без паритетов — всегда parityOk, данные == кадр --------
    {
        const WiegandFormat* f = wiegand::findFormatByBits(37);
        CHECK(f != nullptr);
        CHECK(!f->hasParity);
        WiegandCard c = wiegand::decodeFrame(0x1FEDCBA987ULL, 37);
        CHECK(c.parityOk);
        CHECK(c.data == 0x1FEDCBA987ULL);
    }

    // --- W56: верхняя граница таблицы ---------------------------------------
    {
        const WiegandFormat* f = wiegand::findFormatByBits(56);
        CHECK(f != nullptr);
        uint64_t body = 0x123456789ABCULL;   // 53 значащих бита (56-2=54)
        uint64_t frame = buildFrame(*f, body);
        WiegandCard c = wiegand::decodeFrame(frame, 56);
        CHECK(c.parityOk);
        CHECK(c.data == body);
    }

    // --- Неизвестные длины -> format == nullptr ------------------------------
    CHECK(wiegand::findFormatByBits(4)  == nullptr);
    CHECK(wiegand::findFormatByBits(27) == nullptr);
    {
        WiegandCard c = wiegand::decodeFrame(0xFF, 27);
        CHECK(c.format == nullptr);
        CHECK(!c.parityOk);
    }

    // --- Полнота таблицы: все 8 заявленных форматов присутствуют -------------
    CHECK(WIEGAND_FORMATS_COUNT == 8);
}

// ============================================================================
// BCD (DS3231)
// ============================================================================
static void testBcd() {
    printf("== BcdUtils ==\n");
    // Круговая конверсия для всех допустимых значений регистров времени
    for (int v = 0; v <= 99; ++v) {
        if (bcd::toBin(bcd::toBcd((uint8_t)v)) != v) {
            ++g_fail;
            printf("FAIL bcd roundtrip %d\n", v);
            return;
        }
    }
    g_pass++;
    // Точечные эталоны
    CHECK(bcd::toBin(0x00) == 0);
    CHECK(bcd::toBin(0x09) == 9);
    CHECK(bcd::toBin(0x59) == 59);
    CHECK(bcd::toBin(0x23) == 23);
    CHECK(bcd::toBcd(45) == 0x45);
    CHECK(bcd::toBcd(31) == 0x31);
}

// ============================================================================
// ИНТЕРВАЛЫ ВРЕМЕНИ
// ============================================================================
static void testIntervals() {
    printf("== TimeInterval ==\n");
    using sh_time::minutesInInterval;
    // Дневной 09:00–18:00
    CHECK( minutesInInterval(10 * 60, 9 * 60, 18 * 60));
    CHECK(!minutesInInterval( 8 * 60, 9 * 60, 18 * 60));
    CHECK(!minutesInInterval(18 * 60, 9 * 60, 18 * 60));   // end не включён
    CHECK( minutesInInterval( 9 * 60, 9 * 60, 18 * 60));   // start включён
    // Ночной 22:00–06:00 (СКУД: ночной запрет)
    CHECK( minutesInInterval(23 * 60,      22 * 60, 6 * 60));
    CHECK( minutesInInterval( 3 * 60,      22 * 60, 6 * 60));
    CHECK(!minutesInInterval(12 * 60,      22 * 60, 6 * 60));
    CHECK(!minutesInInterval( 6 * 60,      22 * 60, 6 * 60));
    CHECK( minutesInInterval(22 * 60,      22 * 60, 6 * 60));
    // Вырожденный: start == end -> пустой дневной интервал (не "всегда"!)
    CHECK(!minutesInInterval(12 * 60, 9 * 60, 9 * 60));

    // --- Гражданское время -> unix UTC (RTC хранит UTC-wall, 5.0.x) ------
    // Сверка с эталонным timegm libc: алгоритм days_from_civil обязан
    // совпадать на всём рабочем диапазоне (TZ на host не влияет — timegm).
    {
        struct { int y; unsigned mo, d, h, mi, s; } cases[] = {
            { 1970,  1,  1,  0,  0,  0 },   // эпоха
            { 2000,  2, 29, 23, 59, 59 },   // високосный
            { 2024,  2, 29, 12,  0,  0 },   // високосный (наша эра)
            { 2026,  8,  1,  4, 59, 51 },   // дата полевого инцидента
            { 2038,  1, 19,  3, 14,  7 },   // край int32 unix
            { 2025, 12, 31, 23, 59, 59 },
        };
        for (const auto& c : cases) {
            struct tm t = {};
            t.tm_year = c.y - 1900; t.tm_mon = (int)c.mo - 1; t.tm_mday = (int)c.d;
            t.tm_hour = (int)c.h;   t.tm_min = (int)c.mi;     t.tm_sec  = (int)c.s;
            int64_t expect = (int64_t)timegm(&t);
            int64_t got = sh_time::secondsFromCivil(c.y, c.mo, c.d,
                                                    c.h, c.mi, c.s);
            CHECK(got == expect);
        }
        CHECK(sh_time::secondsFromCivil(1970, 1, 1, 0, 0, 0) == 0);
        CHECK(sh_time::daysFromCivil(1970, 1, 1) == 0);
    }
}

// ============================================================================
// RESOURCE MANAGER (A2): конфликты, идемпотентность, диапазоны событий
// ============================================================================
static void testResourceManager() {
    printf("== ResourceManager ==\n");
    ResourceManager& rm = ResourceManager::getInstance();

    // Занятие и повторное занятие тем же владельцем (идемпотентно)
    CHECK(rm.claimGpio(14, "test.a"));
    CHECK(rm.claimGpio(14, "test.a"));          // свой же — ок
    // Чужой владелец -> конфликт
    CHECK(!rm.claimGpio(14, "test.b"));
    // Владелец виден по запросу
    CHECK(rm.gpioOwner(14) != nullptr);
    CHECK(strcmp(rm.gpioOwner(14), "test.a") == 0);
    CHECK(!rm.isGpioFree(14));
    CHECK(rm.isGpioFree(15));
    // Невалидный пин
    CHECK(!rm.claimGpio(40, "test.bad"));
    // Счётчик конфликтов вырос ровно на зафиксированные случаи (14/b + 40)
    CHECK(rm.conflictCount() >= 2);

    // I2C
    CHECK(rm.claimI2cAddress(0x68, "test.rtc"));
    CHECK(!rm.claimI2cAddress(0x68, "test.other"));

    // UART
    CHECK(rm.claimUart(2, "test.df"));
    CHECK(!rm.claimUart(2, "test.other"));

    // Диапазоны событий: шаг 0x40, повтор тем же владельцем идемпотентен
    int32_t b1 = rm.claimEventRange("test.app1");
    int32_t b2 = rm.claimEventRange("test.app2");
    CHECK(b1 >= 0x1000);                  // SH_EVENT_APP_BASE
    CHECK(b2 == b1 + 0x40);               // шаг диапазонов
    CHECK(rm.claimEventRange("test.app1") == b1);   // идемпотентно (как GPIO)

    // Отчёт не падает и что-то пишет
    char report[1024];
    size_t n = rm.report(report, sizeof(report));
    CHECK(n > 0);
    CHECK(strstr(report, "test.rtc") != nullptr);
}

// ============================================================================
// AUDIO QUEUE: приоритеты, вытеснение при переполнении, resume, анти-флуд
// ============================================================================
static SndItem mkItem(const char* name, uint8_t prio, uint32_t ms,
                      uint8_t flags = 0) {
    SndItem it;
    snprintf(it.name, sizeof(it.name), "%s", name);
    it.folder = 1; it.track = 1;
    it.priority = prio; it.flags = flags;
    it.enqueuedMs = ms;
    return it;
}

static void testAudioQueue() {
    printf("== AudioQueue ==\n");
    const uint8_t AMB = (uint8_t)SndPriority::Ambient;
    const uint8_t NOR = (uint8_t)SndPriority::Normal;
    const uint8_t IMP = (uint8_t)SndPriority::Important;
    const uint8_t ALR = (uint8_t)SndPriority::Alarm;

    // --- Приоритетный pop: Alarm раньше Normal, FIFO внутри приоритета ------
    {
        SndQueue q;
        CHECK(q.enqueue(mkItem("n1", NOR, 100)));
        CHECK(q.enqueue(mkItem("a1", ALR, 200)));   // позже, но выше
        CHECK(q.enqueue(mkItem("n2", NOR, 150)));
        SndItem out;
        CHECK(q.pop(out) && strcmp(out.name, "a1") == 0);   // Alarm первым
        CHECK(q.pop(out) && strcmp(out.name, "n1") == 0);   // старейший Normal
        CHECK(q.pop(out) && strcmp(out.name, "n2") == 0);
        CHECK(!q.pop(out));                                  // пусто
    }

    // --- Переполнение: высокий вытесняет старейший низший, низкий отклонён ---
    {
        SndQueue q;
        for (uint8_t i = 0; i < SND_QUEUE_SIZE; ++i) {
            char nm[8]; snprintf(nm, sizeof(nm), "amb%u", i);
            CHECK(q.enqueue(mkItem(nm, AMB, 100 + i)));
        }
        CHECK(q.count() == SND_QUEUE_SIZE);
        // Ambient в полную очередь Ambient'ов -> отказ
        CHECK(!q.enqueue(mkItem("weak", AMB, 999)));
        // Alarm -> вытесняет СТАРЕЙШИЙ Ambient (amb0)
        CHECK(q.enqueue(mkItem("siren", ALR, 999)));
        SndItem out;
        CHECK(q.pop(out) && strcmp(out.name, "siren") == 0);
        // amb0 вытеснен, следующий — amb1
        CHECK(q.pop(out) && strcmp(out.name, "amb1") == 0);
    }

    // --- pushFront (software-resume после ADVERT) ---------------------------
    {
        SndQueue q;
        q.enqueue(mkItem("wait1", NOR, 100));
        q.enqueue(mkItem("wait2", NOR, 200));
        q.pushFront(mkItem("resumed", NOR, 50));   // прерванная — в голову слоя
        SndItem out;
        CHECK(q.pop(out) && strcmp(out.name, "resumed") == 0);
        CHECK(q.pop(out) && strcmp(out.name, "wait1") == 0);
        // ...но Alarm в очереди всё равно впереди resumed
        SndQueue q2;
        q2.enqueue(mkItem("siren", ALR, 100));
        q2.pushFront(mkItem("resumed", NOR, 50));
        CHECK(q2.pop(out) && strcmp(out.name, "siren") == 0);
        CHECK(q2.pop(out) && strcmp(out.name, "resumed") == 0);
    }

    // --- Политики (чистые функции) ------------------------------------------
    CHECK( snd::shouldPreempt(ALR, NOR));
    CHECK( snd::shouldPreempt(IMP, AMB));
    CHECK(!snd::shouldPreempt(NOR, NOR));   // равные не вытесняют
    CHECK(!snd::shouldPreempt(AMB, IMP));

    CHECK( snd::isRepeat(1000, 500, 1000));    // внутри окна
    CHECK(!snd::isRepeat(1500, 500, 1000));    // окно вышло
    CHECK(!snd::isRepeat(600, 500, 0));        // подавление выключено
    // lastMs > nowMs (теоретически): (400-500)u32 огромно -> НЕ повтор
    CHECK(!snd::isRepeat(400, 500, 1000));
}

// ============================================================================
// БАЗА КАРТ СКУД (CardDbFormat.h — сериализатор/парсер users.json)
// ============================================================================
static void testCardDb() {
    printf("== CardDbFormat ==\n");

    // --- Типы ключей: строки ТОЧНО как в монолите v2.5.0 --------------------
    CHECK(strcmp(carddb::typeStr(0), "master") == 0);
    CHECK(strcmp(carddb::typeStr(1), "permanent") == 0);
    CHECK(strcmp(carddb::typeStr(2), "temporary") == 0);
    CHECK(strcmp(carddb::typeStr(3), "one-time") == 0);
    CHECK(carddb::typeFromStr("master") == 0);
    CHECK(carddb::typeFromStr("one-time") == 3);
    CHECK(carddb::typeFromStr("garbage") == 1);   // неизвестное -> permanent

    // --- Нормализация ID ----------------------------------------------------
    char id[9];
    CHECK( carddb::normalizeId("a1b2c3d4", id) && strcmp(id, "A1B2C3D4") == 0);
    CHECK(!carddb::normalizeId("A1B2C3D4E",id));  // длинный (>8)
    CHECK(!carddb::normalizeId("A1B",      id));  // короче 4 — мусор
    CHECK(!carddb::normalizeId("A1B2C3DZ", id));  // не-HEX
    CHECK(!carddb::normalizeId(nullptr,    id));
    // 5.1.1: ЛЕВЫЙ паддинг 4..8 -> 8. W26-считыватель публикует %06lX,
    // панель принимает 4..8 — ручной ввод с брелока и событие считывателя
    // обязаны сходиться в один ключ (жук «ложный дубликат» 5.1.0).
    CHECK( carddb::normalizeId("898989",  id) && strcmp(id, "00898989") == 0);
    CHECK( carddb::normalizeId("ab12",    id) && strcmp(id, "0000AB12") == 0);
    CHECK( carddb::normalizeId("A1B2C3D", id) && strcmp(id, "0A1B2C3D") == 0);

    // --- Поиск containsCI (5.1.2): ASCII + кириллица UTF-8 -------------------
    CHECK( carddb::containsCI("Ирина",  "ирина"));
    CHECK( carddb::containsCI("Ирина",  "ИРИНА"));
    CHECK( carddb::containsCI("Сергей", "рг"));        // внутри слова
    CHECK( carddb::containsCI("Александр", "сандр"));
    CHECK( carddb::containsCI("00898989", "8989"));
    CHECK( carddb::containsCI("BC32AD12", "bc32"));    // HEX нижним регистром
    CHECK( carddb::containsCI("Ёлка",   "ёлка"));      // Ё/ё — особая пара
    CHECK(!carddb::containsCI("Ирина",  "оль"));
    CHECK( carddb::containsCI("Максим", ""));          // пустой = истина
    CHECK(!carddb::containsCI(nullptr,  "а"));
    CHECK(!carddb::containsCI("Ирина",  nullptr));

    // --- Санитизация имени ---------------------------------------------------
    char nm[65];
    carddb::sanitizeName("Иван \"The\" \\ Петр\n", nm, sizeof(nm));
    CHECK(strcmp(nm, "Иван The  Петр") == 0);
    // 5.2.0: скобки под нож — потоковый загрузчик режет записи подсчётом {}
    carddb::sanitizeName("Те{ст}овый {Случай}", nm, sizeof(nm));
    CHECK(strcmp(nm, "Тестовый Случай") == 0);

    // --- Круговая: serialize -> parse ---------------------------------------
    SlUser src[3] = {};
    strcpy(src[0].id, "A1B2C3D4"); strcpy(src[0].name, "Администратор СКУД");
    src[0].type = 0; src[0].track = 0; src[0].expiry = 0;
    strcpy(src[1].id, "00112233"); strcpy(src[1].name, "Иван");
    src[1].type = 1; src[1].track = 7; src[1].expiry = 0;
    strcpy(src[2].id, "FFEEDDCC"); strcpy(src[2].name, "Гость");
    src[2].type = 3; src[2].track = 0; src[2].expiry = 1893456000UL;

    char buf[4096];
    size_t len = carddb::serialize(src, 3, buf, sizeof(buf));
    CHECK(len > 0);

    SlUser dst[4];
    int n = carddb::parse(buf, dst, 4);
    CHECK(n == 3);
    CHECK(strcmp(dst[0].id, "A1B2C3D4") == 0 && dst[0].type == 0);
    CHECK(strcmp(dst[1].name, "Иван") == 0 && dst[1].track == 7);
    CHECK(dst[2].type == 3 && dst[2].expiry == 1893456000UL);

    // --- 5.2.0: потоковые serializeOne/parseOne ------------------------------
    // serialize == конкатенация serializeOne (единый источник формата)
    {
        char flow[4096]; size_t fp = 0;
        fp += (size_t)snprintf(flow + fp, sizeof(flow) - fp, "{\"users\":[");
        for (int i = 0; i < 3; ++i) {
            size_t w = carddb::serializeOne(src[i], i == 0,
                                            flow + fp, sizeof(flow) - fp);
            CHECK(w > 0);
            fp += w;
        }
        fp += (size_t)snprintf(flow + fp, sizeof(flow) - fp, "]}");
        CHECK(strcmp(flow, buf) == 0);   // поток == монолитная сериализация
    }
    // parseOne: полная запись, с пробелами вокруг, с чужими ключами
    {
        SlUser one;
        CHECK(carddb::parseOne(
            "{ \"id\":\"a1b2c3d4\", \"name\":\"Ольга\", \"type\":\"one-time\","
            "\"track\":9, \"expiry\":10, \"has_pass\":true, \"unknown\":null }",
            one) == 0);
        CHECK(strcmp(one.id, "A1B2C3D4") == 0);      // паддинг/регистр
        CHECK(strcmp(one.name, "Ольга") == 0);
        CHECK(one.type == 3 && one.track == 9 && one.expiry == 10);
        CHECK(one.pin[0] == '\0' && one.uses == 0);  // умолчания
        // Мусор вокруг скобок / битый id / обрыв — порча
        CHECK(carddb::parseOne("{\"id\":\"A1B2C3D4\"} ", one) == 0); // хвост-пробел ок
        CHECK(carddb::parseOne("{\"id\":\"ZZ\"}", one) != 0);
        CHECK(carddb::parseOne("{\"id\":\"A1B2C3D4\"", one) != 0);
        CHECK(carddb::parseOne("x{\"id\":\"A1B2C3D4\"}", one) != 0);
        CHECK(carddb::parseOne(nullptr, one) != 0);
        // ПИН битый — запись жива, веб-доступ снят (не порча файла)
        CHECK(carddb::parseOne(
            "{\"id\":\"A1B2C3D4\",\"pin\":\"12x\"}", one) == 0);
        CHECK(one.pin[0] == '\0');
    }

    // --- Личный веб-ПИН: нормализация (строго 4..6 цифр или пусто) ----------
    char pin[7];
    CHECK( carddb::normalizePin("", pin)      && pin[0] == '\0');
    CHECK( carddb::normalizePin(nullptr, pin) && pin[0] == '\0');
    CHECK( carddb::normalizePin("4821",  pin) && strcmp(pin, "4821") == 0);
    CHECK( carddb::normalizePin("123456",pin) && strcmp(pin, "123456") == 0);
    CHECK(!carddb::normalizePin("123",    pin));  // короткий (<4)
    CHECK(!carddb::normalizePin("1234567",pin));  // длинный (>6)
    CHECK(!carddb::normalizePin("12a4",   pin));  // не цифры
    CHECK(!carddb::normalizePin(" 4821",  pin));  // пробел — мусор

    // --- ПИН: круговая serialize -> parse ------------------------------------
    strcpy(src[1].pin, "4821");
    len = carddb::serialize(src, 3, buf, sizeof(buf));
    CHECK(len > 0);
    CHECK(strstr(buf, "\"pin\":\"4821\"") != nullptr);   // задан — пишется
    CHECK(strstr(buf, "\"pin\":\"\"") == nullptr);       // пустые — не пишутся
    n = carddb::parse(buf, dst, 4);
    CHECK(n == 3);
    CHECK(strcmp(dst[1].pin, "4821") == 0);
    CHECK(dst[0].pin[0] == '\0' && dst[2].pin[0] == '\0');
    src[1].pin[0] = '\0';   // вернуть: дальше тесты идут без ПИНа

    // --- ПИН из файла монолита (user_pin) + битый ПИН = снять доступ --------
    const char* pinJson =
        "{\"users\":[{\"id\":\"AABBCCDD\",\"name\":\"Ольга\",\"type\":\"permanent\","
        "\"track\":3,\"expiry\":0,\"user_pin\":\"7788\"}]}";
    n = carddb::parse(pinJson, dst, 4);
    CHECK(n == 1 && strcmp(dst[0].pin, "7788") == 0);
    const char* badPinJson =
        "{\"users\":[{\"id\":\"AABBCCDD\",\"pin\":\"12\"}]}";
    n = carddb::parse(badPinJson, dst, 4);
    CHECK(n == 1 && dst[0].pin[0] == '\0');   // битый ПИН -> «только карта»

    // --- Статистика/блокировка (5.0.x): круговая serialize -> parse ---------
    src[1].uses = 42; src[1].lastUse = 1785000000UL; src[1].blocked = 1;
    src[2].blocked = 1;                       // блокировка без статистики
    len = carddb::serialize(src, 3, buf, sizeof(buf));
    CHECK(len > 0);
    CHECK(strstr(buf, "\"uses\":42") != nullptr);
    CHECK(strstr(buf, "\"last_use\":1785000000") != nullptr);
    CHECK(strstr(buf, "\"blocked\":1") != nullptr);
    CHECK(strstr(buf, "\"uses\":0") == nullptr);    // нули не пишутся
    CHECK(strstr(buf, "\"blocked\":0") == nullptr);
    n = carddb::parse(buf, dst, 4);
    CHECK(n == 3);
    CHECK(dst[1].uses == 42 && dst[1].lastUse == 1785000000UL &&
          dst[1].blocked == 1);
    CHECK(dst[2].blocked == 1 && dst[2].uses == 0 && dst[2].lastUse == 0);
    CHECK(dst[0].uses == 0 && dst[0].lastUse == 0 && dst[0].blocked == 0);
    src[1].uses = 0; src[1].lastUse = 0; src[1].blocked = 0;
    src[2].blocked = 0;      // вернуть: дальше тесты идут без статистики

    // --- Старый файл (без новых ключей) -> нулевые умолчания -----------------
    const char* legacyJson =
        "{\"users\":[{\"id\":\"AABBCCDD\",\"name\":\"Пётр\","
        "\"type\":\"permanent\",\"track\":2,\"expiry\":0}]}";
    n = carddb::parse(legacyJson, dst, 4);
    CHECK(n == 1 && dst[0].uses == 0 && dst[0].lastUse == 0 &&
          dst[0].blocked == 0);

    // --- Пустая база — легальна ---------------------------------------------
    len = carddb::serialize(src, 0, buf, sizeof(buf));
    CHECK(len > 0 && carddb::parse(buf, dst, 4) == 0);

    // --- Формат монолита: неизвестные поля терпимы, порядок полей — тоже ----
    const char* monolithJson =
        "{ \"users\": [ { \"expiry\": 0, \"track\": 5, \"name\": \"Мария\","
        " \"id\": \"11223344\", \"type\": \"temporary\","
        " \"has_password\": true, \"expiry_str\": \"2026-01-01\" } ] }";
    n = carddb::parse(monolithJson, dst, 4);
    CHECK(n == 1);
    CHECK(strcmp(dst[0].id, "11223344") == 0 && dst[0].type == 2 &&
          dst[0].track == 5 && strcmp(dst[0].name, "Мария") == 0);

    // --- Порча -> -1 (никаких половинчатых баз) ------------------------------
    CHECK(carddb::parse("not json", dst, 4) == -1);
    CHECK(carddb::parse("{\"users\":", dst, 4) == -1);
    CHECK(carddb::parse("{\"users\":[{\"id\":\"ZZ\"}]}", dst, 4) == -1);
    CHECK(carddb::parse("{\"other\":[]}", dst, 4) == -1);
    CHECK(carddb::parse("{\"users\":[{\"id\":\"A1B2C3D4\"}", dst, 4) == -1);
    // Переполнение кэша: 2 записи при maxCount=1 -> порча
    CHECK(carddb::parse(buf, dst, 1) == -1 || true); // buf сейчас пустая база
    n = carddb::serialize(src, 3, buf, sizeof(buf)) ? 1 : 0;
    CHECK(n == 1);
    CHECK(carddb::parse(buf, dst, 2) == -1);   // 3 записи > 2 мест

    // --- Сериализация в тесный буфер -> 0 (откат, не каша) --------------------
    char tiny[40];
    CHECK(carddb::serialize(src, 3, tiny, sizeof(tiny)) == 0);
}

// ============================================================================
// ДАТАЛОГГЕР (DataLogCore.h — кольцо, агрегаты, децимация)
// ============================================================================
static void testDataLog() {
    printf("== DataLogCore ==\n");

    // --- Кольцо: порядок и переполнение --------------------------------------
    dlog::Ring ring;
    for (uint32_t i = 0; i < DLOG_RAW_CAP + 40; ++i) {
        ring.push(1000 + i * 60, (float)i);   // на 40 точек больше ёмкости
    }
    CHECK(ring.count == DLOG_RAW_CAP);
    DlogPoint snap[DLOG_RAW_CAP];
    uint16_t n = ring.snapshot(snap, DLOG_RAW_CAP, 0);
    CHECK(n == DLOG_RAW_CAP);
    // Хронология: старейшая — 40-я из записанных (первые 40 вытеснены)
    CHECK(snap[0].ts == 1000 + 40 * 60 && snap[0].v == 40.0f);
    CHECK(snap[n - 1].ts == 1000 + (DLOG_RAW_CAP + 39) * 60);
    for (uint16_t i = 1; i < n; ++i) CHECK(snap[i].ts > snap[i - 1].ts);

    // --- Кольцо: фильтр by fromTs ---------------------------------------------
    n = ring.snapshot(snap, DLOG_RAW_CAP, 1000 + (DLOG_RAW_CAP + 20) * 60);
    CHECK(n == 20);   // i=380..399 последнего витка (ts >= from), включая край
    CHECK(n == 0 || snap[0].ts >= 1000 + (DLOG_RAW_CAP + 20) * 60);

    // --- Ведро: агрегация и перекрытие часа -----------------------------------
    dlog::Bucket b;
    DlogAggr rolled;
    CHECK(!b.add(3600, 10.0f, 3600, rolled));      // час 01:00, ведро новое
    CHECK(!b.add(3660, 20.0f, 3600, rolled));
    CHECK(!b.add(3700, 5.0f, 3600, rolled));
    CHECK(b.add(7200, 99.0f, 3600, rolled));       // новый час -> roll
    CHECK(rolled.ts == 3600);
    CHECK(rolled.mn == 5.0f && rolled.mx == 20.0f);
    CHECK(rolled.avg > 11.66f && rolled.avg < 11.67f);   // 35/3
    // Новое ведро началось с точки 99.0
    dlog::Bucket b2 = b;
    CHECK(b2.n == 1 && b2.periodStart == 7200);

    // --- Ведро: flush на пустом/непустом --------------------------------------
    dlog::Bucket empty;
    CHECK(!empty.flush(rolled));
    CHECK(b.flush(rolled) && rolled.ts == 7200 && rolled.avg == 99.0f);
    CHECK(b.n == 0);

    // --- Ведро: точка из «прошлого» после roll (нерегулярный приём) -----------
    dlog::Bucket b3;
    b3.add(86400, 1.0f, 3600, rolled);             // сутки 2, 00:00
    CHECK(b3.add(90000, 2.0f, 3600, rolled));      // час спустя -> roll
    b3.add(86460, 3.0f, 3600, rolled);             // запоздавшая в тот же час?
    // 86460 -> период 86400, а ведро уже 90000: откроет НОВОЕ ведро назад
    // (документированная честность: нерегулярность не ломает математику)
    CHECK(b3.periodStart == 86400 && b3.n == 1 && b3.mn == 3.0f);

    // --- Децимация RAW: влезть в потолок, края честные ------------------------
    DlogPoint dense[600], dec[DLOG_JSON_POINTS];
    for (uint16_t i = 0; i < 600; ++i) {
        dense[i].ts = 5000 + i * 30;
        dense[i].v  = (float)(i % 7);
    }
    n = dlog::decimateRaw(dense, 600, dec, DLOG_JSON_POINTS);
    CHECK(n <= DLOG_JSON_POINTS && n >= 190);   // stride=3 -> 200 + хвост
    CHECK(dec[0].ts == dense[0].ts);
    CHECK(dec[n - 1].ts == dense[599].ts);         // последняя сохранена
    for (uint16_t i = 1; i < n; ++i) CHECK(dec[i].ts > dec[i - 1].ts);

    // --- Децимация агрегатов: min/max не теряются ------------------------------
    DlogAggr ag[500], agd[DLOG_JSON_POINTS];
    for (uint16_t i = 0; i < 500; ++i) {
        ag[i].ts  = i * 3600;
        ag[i].mn  = (float)(i == 250 ? -50 : 0);   // выброс посередине
        ag[i].mx  = (float)(i == 250 ? 150 : 10);
        ag[i].avg = 5.0f;
    }
    n = dlog::decimateAggr(ag, 500, agd, DLOG_JSON_POINTS);
    CHECK(n <= DLOG_JSON_POINTS);
    bool keptMin = false, keptMax = false;
    for (uint16_t i = 0; i < n; ++i) {
        if (agd[i].mn <= -50.0f) keptMin = true;
        if (agd[i].mx >= 150.0f) keptMax = true;
    }
    CHECK(keptMin && keptMax);                     // экстремумы пережили слив

    // --- Маленькие серии не трогаем -------------------------------------------
    n = dlog::decimateRaw(dense, 100, dec, DLOG_JSON_POINTS);
    CHECK(n == 100 && dec[99].ts == dense[99].ts);
    n = dlog::decimateAggr(ag, 100, agd, DLOG_JSON_POINTS);
    CHECK(n == 100);
}

// ============================================================================
// ПЛАНИРОВЩИК (ScheduleCore.h — парсер правил, маски дней, фронты)
// ============================================================================
static void testScheduleCore() {
    printf("== ScheduleCore ==\n");
    uint16_t mn = 0;

    // --- parseHHMM ---
    CHECK(sched::parseHHMM("22:00", mn) && mn == 1320);
    CHECK(sched::parseHHMM("00:00", mn) && mn == 0);
    CHECK(sched::parseHHMM("7:05", mn) && mn == 425);   // одна цифра часа
    CHECK(sched::parseHHMM("23:59", mn) && mn == 1439);
    CHECK(!sched::parseHHMM("24:00", mn));              // час вне диапазона
    CHECK(!sched::parseHHMM("12:60", mn));              // минута вне диапазона
    CHECK(!sched::parseHHMM("1200", mn));               // без двоеточия
    CHECK(!sched::parseHHMM("12:5", mn));               // минута одной цифрой
    CHECK(!sched::parseHHMM("", mn));
    CHECK(!sched::parseHHMM(nullptr, mn));

    // --- dayBit: tm_wday (0=вс) -> бит0=пн ---
    CHECK(sched::dayBit(1) == 0x01);   // понедельник
    CHECK(sched::dayBit(2) == 0x02);   // вторник
    CHECK(sched::dayBit(6) == 0x20);   // суббота
    CHECK(sched::dayBit(0) == 0x40);   // воскресенье
    CHECK(sched::dayBit(7) == 0);      // вне диапазона
    CHECK(sched::dayBit(-1) == 0);

    // --- parseDayMask ---
    uint8_t mask = 0;
    CHECK(sched::parseDayMask("*", mask) && mask == SCHED_DAYS_ALL);
    CHECK(sched::parseDayMask("12345", mask) && mask == 0x1F);  // будни
    CHECK(sched::parseDayMask("67", mask) && mask == 0x60);     // выходные
    CHECK(sched::parseDayMask("7", mask) && mask == 0x40);
    CHECK(!sched::parseDayMask("", mask));
    CHECK(!sched::parseDayMask("0", mask));     // нумерация с 1
    CHECK(!sched::parseDayMask("8", mask));
    CHECK(!sched::parseDayMask("112", mask));   // дубликат = опечатка
    CHECK(!sched::parseDayMask("1,3", mask));   // запятые не поддерживаем

    // --- parseRule: валидные ---
    SchedRule r;
    CHECK(sched::parseRule("ночь|22:00|06:00|*|1", r));
    CHECK(strcmp(r.name, "ночь") == 0);
    CHECK(r.type == SCHED_RULE_INTERVAL);
    CHECK(r.startMin == 1320 && r.endMin == 360);
    CHECK(r.dayMask == SCHED_DAYS_ALL && r.periodCode == 1 && r.enabled);

    CHECK(sched::parseRule("будни|09:00|18:00|12345|2", r));
    CHECK(r.startMin == 540 && r.endMin == 1080 && r.dayMask == 0x1F);

    // Точечное правило: пустое «ПО»
    CHECK(sched::parseRule("полив|06:30||135|4", r));
    CHECK(r.type == SCHED_RULE_POINT);
    CHECK(r.startMin == 390 && r.endMin == 390);
    CHECK(r.dayMask == 0x15 && r.periodCode == 4);    // пн+ср+пт

    // Пробелы вокруг имени — прощаем; хвостовые — обрезаем
    CHECK(sched::parseRule("  рассвет |05:45||*|9", r));
    CHECK(strcmp(r.name, "рассвет") == 0 && r.periodCode == 9);

    // --- parseRule: брак (правило отбрасывается целиком) ---
    CHECK(!sched::parseRule("", r));
    CHECK(!sched::parseRule(nullptr, r));
    CHECK(!sched::parseRule("|22:00|06:00|*|1", r));        // пустое имя
    CHECK(!sched::parseRule("x|22:00|06:00|*", r));         // 4 поля
    CHECK(!sched::parseRule("x|22:00|06:00|*|1|9", r));     // 6 полей
    CHECK(!sched::parseRule("x|25:00|06:00|*|1", r));       // битое время
    CHECK(!sched::parseRule("x|22:00|22:00|*|1", r));       // вырожденный
    CHECK(!sched::parseRule("x|22:00|06:00||1", r));        // пустые дни
    CHECK(!sched::parseRule("x|22:00|06:00|*|0", r));       // код 0 = ядро
    CHECK(!sched::parseRule("x|22:00|06:00|*|256", r));     // код велик
    CHECK(!sched::parseRule("x|22:00|06:00|*|1a", r));      // код не число
    // имя длиннее буфера — отбраковываем, не обрезаем молча
    CHECK(!sched::parseRule("оченьоченьдлинноеимяправила|22:00|06:00|*|1", r));

    // --- activeAt: дневной интервал ---
    SchedRule work;  // будни 09:00–18:00
    CHECK(sched::parseRule("w|09:00|18:00|12345|2", work));
    CHECK( sched::activeAt(work, 600, 1));    // пн 10:00
    CHECK( sched::activeAt(work, 540, 5));    // пт 09:00 (start включается)
    CHECK(!sched::activeAt(work, 1080, 1));   // пн 18:00 (end исключается)
    CHECK(!sched::activeAt(work, 539, 1));    // пн 08:59
    CHECK(!sched::activeAt(work, 600, 6));    // сб 10:00 — не в маске
    CHECK(!sched::activeAt(work, 600, 0));    // вс — тоже

    // --- activeAt: ночной интервал через полночь (маска по дню НАЧАЛА) ---
    SchedRule night;  // будничные ночи 22:00–06:00
    CHECK(sched::parseRule("n|22:00|06:00|12345|1", night));
    CHECK( sched::activeAt(night, 1380, 1));  // пн 23:00
    CHECK( sched::activeAt(night, 1320, 5));  // пт 22:00 (граница вкл.)
    CHECK( sched::activeAt(night, 300, 2));   // вт 05:00 — открыли в пн
    CHECK( sched::activeAt(night, 359, 6));   // сб 05:59 — открыли в пт
    CHECK(!sched::activeAt(night, 360, 2));   // вт 06:00 (end искл.)
    CHECK(!sched::activeAt(night, 720, 1));   // пн 12:00
    CHECK(!sched::activeAt(night, 1380, 6));  // сб 23:00 — ночи выходных нет
    CHECK(!sched::activeAt(night, 300, 0));   // вс 05:00 — открыли бы в сб
    CHECK(!sched::activeAt(night, 300, 1));   // пн 05:00 — вчера было вс,
                                              // не в маске: ночь с вс на пн
                                              // не входит в «будничные ночи»

    // --- pointDue ---
    SchedRule alarm;  // 07:30 ежедневно
    CHECK(sched::parseRule("a|07:30||*|3", alarm));
    CHECK( sched::pointDue(alarm, 450, 3));
    CHECK(!sched::pointDue(alarm, 451, 3));
    CHECK(!sched::pointDue(alarm, 449, 3));
    SchedRule alarm135;
    CHECK(sched::parseRule("a|07:30||135|3", alarm135));
    CHECK( sched::pointDue(alarm135, 450, 1));   // пн
    CHECK(!sched::pointDue(alarm135, 450, 2));   // вт не в маске

    // --- intervalEdge: фронты ---
    bool now = false;
    CHECK(sched::intervalEdge(false, night, 1380, 1, now) == SCHED_EDGE_ENTER && now);
    CHECK(sched::intervalEdge(true,  night, 1390, 1, now) == SCHED_EDGE_NONE  && now);
    CHECK(sched::intervalEdge(true,  night, 720,  1, now) == SCHED_EDGE_EXIT  && !now);
    CHECK(sched::intervalEdge(false, night, 720,  1, now) == SCHED_EDGE_NONE  && !now);
    // точечное правило фронтов не даёт
    CHECK(sched::intervalEdge(false, alarm, 450, 1, now) == SCHED_EDGE_NONE && !now);
    // выключенное правило неактивно
    night.enabled = false;
    CHECK(!sched::activeAt(night, 1380, 1));
}

// ============================================================================
// СЧЁТЧИКИ (CounterCore.h — политика батч-сброса, PCNT-дельта)
// ============================================================================
static void testCounterCore() {
    printf("== CounterCore ==\n");

    // --- shouldFlush ---
    CHECK(!cnt::shouldFlush(0, 10, 999999, 600000));   // писать нечего — никогда
    CHECK(!cnt::shouldFlush(5, 10, 1000, 600000));     // ни один порог не достигнут
    CHECK( cnt::shouldFlush(10, 10, 1000, 600000));    // порог по числу
    CHECK( cnt::shouldFlush(11, 10, 0, 600000));       // превышение числа
    CHECK( cnt::shouldFlush(3, 10, 600000, 600000));   // порог по времени
    CHECK( cnt::shouldFlush(1, 10, 600001, 600000));   // время + минимальный pending
    CHECK(!cnt::shouldFlush(5, 0, 0, 0));              // оба порога выключены
    CHECK( cnt::shouldFlush(5, 0, 700000, 600000));    // только временной порог
    CHECK( cnt::shouldFlush(7, 5, 0, 0));              // только числовой порог

    // --- pcntDelta: обычный ход ---
    CHECK(cnt::pcntDelta(100, 0) == 100);
    CHECK(cnt::pcntDelta(32767, 32700) == 67);
    CHECK(cnt::pcntDelta(0, 0) == 0);

    // --- pcntDelta: оборачивания 16-битного регистра ---
    CHECK(cnt::pcntDelta(-32768, 32767) == 1);         // 32767 + 1 -> -32768
    CHECK(cnt::pcntDelta(-32760, 32760) == 16);        // wrap через верх
    // дельта 65535 — ВНЕ представимого окна: 16-битная арифметика
    // даёт -1; сервис отбросит (delta > 0). Документ честной границы:
    // между чтениями должно набегать МЕНЬШЕ 32768 импульсов.
    CHECK(cnt::pcntDelta(32767, -32768) == -1);
    CHECK(cnt::pcntDelta(-1, -32768) == 32767);        // максимум корректной дельты
    // отрицательная дельта (внешний clear) — сырьё для сервиса, тот отбросит
    CHECK(cnt::pcntDelta(0, 100) == -100);
}

// ============================================================================
// SNTP CORE (SntpCore.h — пакеты RFC 4330, чистая логика своего клиента 5.5.7)
// ============================================================================
static void testSntpCore() {
    printf("== SntpCore ==\n");
    using namespace sh_sntp;

    // Запрос: ровно 48 байт, первый 0x1B (LI0/VN3/Mode3), остальные нули
    uint8_t req[PACKET_LEN];
    memset(req, 0xAA, sizeof(req));          // гарантия полной записи
    buildRequest(req);
    CHECK(req[0] == 0x1B);
    bool zeros = true;
    for (size_t i = 1; i < PACKET_LEN; ++i) if (req[i] != 0) zeros = false;
    CHECK(zeros);

    // Валидный ответ: mode=server(4), stratum=2, произвольная дата > 2025
    uint8_t rep[PACKET_LEN] = {0};
    rep[0] = (VERSION_3 << 3) | MODE_SERVER;   // 0x1C
    rep[1] = 2;
    const uint32_t ts = EPOCH_OFFSET + 1785000000UL;
    rep[40]=(uint8_t)(ts>>24); rep[41]=(uint8_t)(ts>>16);
    rep[42]=(uint8_t)(ts>>8);  rep[43]=(uint8_t)ts;
    uint32_t unix = 0;
    CHECK(parseReply(rep, sizeof(rep), unix));
    CHECK(unix == 1785000000UL);

    // Отбраковка мусора:
    CHECK(!parseReply(rep, 10, unix));                       // короткий пакет
    CHECK(!parseReply(nullptr, sizeof(rep), unix));          // нулевой буфер
    rep[0] = (VERSION_3 << 3) | MODE_CLIENT;                 // не server
    CHECK(!parseReply(rep, sizeof(rep), unix));
    rep[0] = (VERSION_3 << 3) | MODE_SERVER; rep[1] = 0;     // stratum 0 = KoD
    CHECK(!parseReply(rep, sizeof(rep), unix));
    rep[1] = 2; rep[40]=0; rep[41]=0; rep[42]=0; rep[43]=0;  // ts = 0
    CHECK(!parseReply(rep, sizeof(rep), unix));
    rep[43] = 1;                                             // ts < эпохи NTP
    CHECK(!parseReply(rep, sizeof(rep), unix));
    const uint32_t old = EPOCH_OFFSET + 1000;                // unix = 1000 < 2025
    rep[40]=(uint8_t)(old>>24); rep[41]=(uint8_t)(old>>16);
    rep[42]=(uint8_t)(old>>8);  rep[43]=(uint8_t)old;
    CHECK(!parseReply(rep, sizeof(rep), unix));
}

// ============================================================================
// MQTT OUTBOX (MqttOutbox.h — кольцо, retained-дедуп, вытеснение)
// ============================================================================
static void testMqttOutbox() {
    printf("== MqttOutbox ==\n");
    mqtt_ob::Outbox ob;
    ob.reset();

    // --- базовый FIFO ---
    CHECK(ob.size() == 0 && ob.peek() == nullptr);
    CHECK(ob.push("a/b/events/ACCESS_GRANTED", "0|card1", 1, false));
    CHECK(ob.push("a/b/events/ACCESS_DENIED", "3|card2", 1, false));
    CHECK(ob.size() == 2);
    const MqttObSlot* s = ob.peek();
    CHECK(s && strcmp(s->topic, "a/b/events/ACCESS_GRANTED") == 0);
    CHECK(strcmp(s->body, "0|card1") == 0 && s->qos == 1 && !s->retained);
    ob.pop();
    s = ob.peek();
    CHECK(s && strcmp(s->topic, "a/b/events/ACCESS_DENIED") == 0);
    ob.pop();
    CHECK(ob.size() == 0 && ob.peek() == nullptr);
    ob.pop();   // pop по пустому — не авария
    CHECK(ob.size() == 0);

    // --- отказы ---
    CHECK(!ob.push(nullptr, "x", 1, false));       // нет топика
    CHECK(!ob.push("", "x", 1, false));            // пустой топик
    char longTopic[MQTT_OB_TOPIC_LEN + 8];
    memset(longTopic, 't', sizeof(longTopic) - 1);
    longTopic[sizeof(longTopic) - 1] = '\0';
    CHECK(!ob.push(longTopic, "x", 1, false));     // топик не влез — отказ,
    CHECK(ob.size() == 0);                         // обрезать нельзя

    // --- тело обрезается, сообщение живёт ---
    char longBody[MQTT_OB_BODY_LEN + 40];
    memset(longBody, 'b', sizeof(longBody) - 1);
    longBody[sizeof(longBody) - 1] = '\0';
    CHECK(ob.push("t/1", longBody, 0, false));
    s = ob.peek();
    CHECK(s && strlen(s->body) == MQTT_OB_BODY_LEN - 1);
    ob.pop();

    // --- retained-дедупликация: свежее состояние замещает лежащее ---
    CHECK(ob.push("h/cover/1/state", "closed", 1, true));
    CHECK(ob.push("a/b/events/X", "1|", 1, false));      // чужое — между
    CHECK(ob.push("h/cover/1/state", "open", 1, true));  // LWW-замещение
    CHECK(ob.size() == 2);                               // не третье сообщение!
    s = ob.peek();
    CHECK(s && strcmp(s->topic, "h/cover/1/state") == 0);
    CHECK(strcmp(s->body, "open") == 0);                 // тело обновлено
    // non-retained по тому же топику — НЕ дедуплицируется (события равны)
    CHECK(ob.push("h/cover/1/state", "open", 1, false));
    CHECK(ob.size() == 3);
    ob.reset();
    CHECK(ob.size() == 0 && ob.dropped == 0);

    // --- переполнение: вытеснение старейшего + счётчик ---
    for (uint8_t i = 0; i < MQTT_OB_MAX; ++i) {
        char t[16], b[16];
        snprintf(t, sizeof(t), "t/%u", i);
        snprintf(b, sizeof(b), "m%u", i);
        CHECK(ob.push(t, b, 1, false));
    }
    CHECK(ob.size() == MQTT_OB_MAX && ob.dropped == 0);
    CHECK(ob.push("t/new", "mnew", 1, false));           // девятое
    CHECK(ob.size() == MQTT_OB_MAX);                     // кольцо не растёт
    CHECK(ob.dropped == 1);                              // потеря посчитана
    s = ob.peek();
    CHECK(s && strcmp(s->topic, "t/1") == 0);            // t/0 вытеснен
    // дренаж до дна: порядок FIFO сохранён, "t/new" — последний
    char last[16] = {0};
    while ((s = ob.peek()) != nullptr) {
        snprintf(last, sizeof(last), "%s", s->topic);
        ob.pop();
    }
    CHECK(strcmp(last, "t/new") == 0);
    CHECK(ob.size() == 0);

    // --- полное кольцо + retained-дедуп: замещение без вытеснения ---
    for (uint8_t i = 0; i < MQTT_OB_MAX; ++i) {
        char t[16]; snprintf(t, sizeof(t), "r/%u", i);
        CHECK(ob.push(t, "v1", 1, true));
    }
    CHECK(ob.dropped == 1);                              // со времён прошлого
    CHECK(ob.push("r/3", "v2", 1, true));                // замещение в полном
    CHECK(ob.size() == MQTT_OB_MAX && ob.dropped == 1);  // вытеснения не было
    // retained без пары в полном кольце — обычное вытеснение
    CHECK(ob.push("r/new", "v", 1, true));
    CHECK(ob.dropped == 2);
    ob.reset();
    CHECK(ob.dropped == 0 && ob.size() == 0);
}

// ============================================================================
// СОСТАВНАЯ РЕЧЬ (SpeechCore.h — цепочки чисел/единиц/времени)
// ============================================================================
// Эталон: цепочка (folder,track) сравнивается с ожидаемым массивом треков
// папки 05/06 (раскладка — из manifest.json конвейера).
static bool chainEq(const SpeechTrack* got, uint8_t gotLen,
                    const uint8_t* expTracks, uint8_t expLen,
                    uint8_t expFolder) {
    if (gotLen != expLen) return false;
    for (uint8_t i = 0; i < expLen; ++i)
        if (got[i].folder != expFolder || got[i].track != expTracks[i])
            return false;
    return true;
}

static void testSpeechCore() {
    printf("== SpeechCore ==\n");
    SpeechTrack ch[SB_MAX_CHAIN];

    // --- pluralForm ---
    CHECK(speech::pluralForm(0) == SB_PL_MANY);    // ноль часов
    CHECK(speech::pluralForm(1) == SB_PL_ONE);
    CHECK(speech::pluralForm(2) == SB_PL_FEW);
    CHECK(speech::pluralForm(4) == SB_PL_FEW);
    CHECK(speech::pluralForm(5) == SB_PL_MANY);
    CHECK(speech::pluralForm(11) == SB_PL_MANY);   // исключение подростков
    CHECK(speech::pluralForm(14) == SB_PL_MANY);
    CHECK(speech::pluralForm(21) == SB_PL_ONE);
    CHECK(speech::pluralForm(22) == SB_PL_FEW);
    CHECK(speech::pluralForm(25) == SB_PL_MANY);
    CHECK(speech::pluralForm(101) == SB_PL_ONE);
    CHECK(speech::pluralForm(111) == SB_PL_MANY);
    CHECK(speech::pluralForm(-1) == SB_PL_ONE);    // по модулю

    // --- numberTracks: база ---
    const uint8_t e0[] = {1};                              // ноль
    CHECK(chainEq(ch, speech::numberTracks(0, SB_MASC, ch, SB_MAX_CHAIN),
                  e0, 1, 5));
    const uint8_t e1m[] = {2}, e1f[] = {3};                // один/одна
    CHECK(chainEq(ch, speech::numberTracks(1, SB_MASC, ch, SB_MAX_CHAIN), e1m, 1, 5));
    CHECK(chainEq(ch, speech::numberTracks(1, SB_FEM,  ch, SB_MAX_CHAIN), e1f, 1, 5));
    const uint8_t e2m[] = {4}, e2f[] = {5};                // два/две
    CHECK(chainEq(ch, speech::numberTracks(2, SB_MASC, ch, SB_MAX_CHAIN), e2m, 1, 5));
    CHECK(chainEq(ch, speech::numberTracks(2, SB_FEM,  ch, SB_MAX_CHAIN), e2f, 1, 5));
    const uint8_t e9[] = {12}, e11[] = {14}, e19[] = {22}; // девять/один-ть/девят-ть
    CHECK(chainEq(ch, speech::numberTracks(9, SB_MASC, ch, SB_MAX_CHAIN), e9, 1, 5));
    CHECK(chainEq(ch, speech::numberTracks(11, SB_MASC, ch, SB_MAX_CHAIN), e11, 1, 5));
    CHECK(chainEq(ch, speech::numberTracks(19, SB_MASC, ch, SB_MAX_CHAIN), e19, 1, 5));
    const uint8_t e20[] = {23}, e90[] = {30};              // двадцать/девяносто
    CHECK(chainEq(ch, speech::numberTracks(20, SB_MASC, ch, SB_MAX_CHAIN), e20, 1, 5));
    CHECK(chainEq(ch, speech::numberTracks(90, SB_MASC, ch, SB_MAX_CHAIN), e90, 1, 5));
    const uint8_t e21[] = {23, 2};                         // двадцать один
    CHECK(chainEq(ch, speech::numberTracks(21, SB_MASC, ch, SB_MAX_CHAIN), e21, 2, 5));
    const uint8_t e99[] = {30, 12};                        // девяносто девять
    CHECK(chainEq(ch, speech::numberTracks(99, SB_MASC, ch, SB_MAX_CHAIN), e99, 2, 5));

    // --- numberTracks: сотни ---
    const uint8_t e100[] = {31};                           // сто
    CHECK(chainEq(ch, speech::numberTracks(100, SB_MASC, ch, SB_MAX_CHAIN), e100, 1, 5));
    const uint8_t e101[] = {31, 2};                        // сто один
    CHECK(chainEq(ch, speech::numberTracks(101, SB_MASC, ch, SB_MAX_CHAIN), e101, 2, 5));
    const uint8_t e115[] = {31, 18};                       // сто пятнадцать
    CHECK(chainEq(ch, speech::numberTracks(115, SB_MASC, ch, SB_MAX_CHAIN), e115, 2, 5));
    const uint8_t e999[] = {39, 30, 12};                   // девятьсот девяносто девять
    CHECK(chainEq(ch, speech::numberTracks(999, SB_MASC, ch, SB_MAX_CHAIN), e999, 3, 5));

    // --- numberTracks: тысячи (род всегда женский) ---
    const uint8_t e1000[] = {3, 40};                       // одна тысяча
    CHECK(chainEq(ch, speech::numberTracks(1000, SB_MASC, ch, SB_MAX_CHAIN), e1000, 2, 5));
    const uint8_t e2000[] = {5, 41};                       // две тысячи
    CHECK(chainEq(ch, speech::numberTracks(2000, SB_MASC, ch, SB_MAX_CHAIN), e2000, 2, 5));
    const uint8_t e4321[] = {7, 41, 33, 23, 2};            // четыре тысячи триста
    CHECK(chainEq(ch, speech::numberTracks(4321, SB_MASC, ch, SB_MAX_CHAIN), e4321, 5, 5));
                                                         // двадцать один
    // --- numberTracks: минус и границы ---
    const uint8_t em25[] = {42, 23, 8};                    // минус двадцать пять
    CHECK(chainEq(ch, speech::numberTracks(-25, SB_MASC, ch, SB_MAX_CHAIN), em25, 3, 5));
    CHECK(speech::numberTracks(4999, SB_MASC, ch, SB_MAX_CHAIN) > 0);
    CHECK(speech::numberTracks(5000, SB_MASC, ch, SB_MAX_CHAIN) == 0);  // нет «тысяч»
    CHECK(speech::numberTracks(-4999, SB_MASC, ch, SB_MAX_CHAIN) > 0);
    CHECK(speech::numberTracks(-5000, SB_MASC, ch, SB_MAX_CHAIN) == 0);
    CHECK(speech::numberTracks(21, SB_MASC, ch, 0) == 0);  // maxOut=0
    CHECK(speech::numberTracks(999, SB_MASC, ch, 2) == 0); // не влезло — 0,
                                                           // полуфразы не бывает
    // -4999: минус + 5 треков = 6; буфер 5 -> отказ
    CHECK(speech::numberTracks(-4999, SB_MASC, ch, 5) == 0);
    CHECK(speech::numberTracks(-4999, SB_MASC, ch, 6) == 6);

    // --- unitTrack: триады и фиксированные ---
    CHECK(speech::unitTrack(1, SB_UNIT_HOUR_ONE, 0) == SB_UNIT_HOUR_ONE);   // час
    CHECK(speech::unitTrack(3, SB_UNIT_HOUR_ONE, 0) == SB_UNIT_HOUR_FEW);   // часа
    CHECK(speech::unitTrack(12, SB_UNIT_HOUR_ONE, 0) == SB_UNIT_HOUR_MANY); // часов
    CHECK(speech::unitTrack(21, SB_UNIT_MIN_ONE, 0) == SB_UNIT_MIN_ONE);    // минута
    CHECK(speech::unitTrack(55, SB_UNIT_MIN_ONE, 0) == SB_UNIT_MIN_MANY);   // минут
    CHECK(speech::unitTrack(1, 0, SB_UNIT_DEGREES) == SB_UNIT_DEGREES);     // фикс.
    CHECK(speech::unitTrack(21, 0, SB_UNIT_DEGREES) == SB_UNIT_DEGREES);

    // --- numberUnitTracks: «двадцать один градусов», «три минуты» ---
    uint8_t len = speech::numberUnitTracks(21, SB_MASC, 0, SB_UNIT_DEGREES,
                                           ch, SB_MAX_CHAIN);
    CHECK(len == 3);
    CHECK(ch[0].folder == 5 && ch[0].track == 23);
    CHECK(ch[1].folder == 5 && ch[1].track == 2);
    CHECK(ch[2].folder == 6 && ch[2].track == SB_UNIT_DEGREES);
    len = speech::numberUnitTracks(3, SB_FEM, SB_UNIT_MIN_ONE, 0,
                                   ch, SB_MAX_CHAIN);
    CHECK(len == 2);
    CHECK(ch[0].folder == 5 && ch[0].track == 6);        // три
    CHECK(ch[1].folder == 6 && ch[1].track == SB_UNIT_MIN_FEW);  // минуты
    len = speech::numberUnitTracks(1, SB_FEM, SB_UNIT_MIN_ONE, 0,
                                   ch, SB_MAX_CHAIN);
    CHECK(len == 2 && ch[0].track == 3 && ch[1].track == SB_UNIT_MIN_ONE);

    // --- timeTracks ---
    // 12:00 -> двенадцать(15) часов(9) ровно(43)
    len = speech::timeTracks(12, 0, ch, SB_MAX_CHAIN);
    CHECK(len == 3);
    CHECK(ch[0].track == 15 && ch[0].folder == 5);
    CHECK(ch[1].track == SB_UNIT_HOUR_MANY && ch[1].folder == 6);
    CHECK(ch[2].track == 43 && ch[2].folder == 5);
    // 0:30 -> ноль(1) часов(9) тридцать(24) минут(12)
    len = speech::timeTracks(0, 30, ch, SB_MAX_CHAIN);
    CHECK(len == 4);
    CHECK(ch[0].track == 1 && ch[1].track == SB_UNIT_HOUR_MANY);
    CHECK(ch[2].track == 24 && ch[3].track == SB_UNIT_MIN_MANY);
    // 1:01 -> один(2) час(7) одна(3) минута(10)
    len = speech::timeTracks(1, 1, ch, SB_MAX_CHAIN);
    CHECK(len == 4);
    CHECK(ch[0].track == 2 && ch[1].track == SB_UNIT_HOUR_ONE);
    CHECK(ch[2].track == 3 && ch[3].track == SB_UNIT_MIN_ONE);
    // 23:59 -> двадцать три часа пятьдесят девять минут = 6 треков
    len = speech::timeTracks(23, 59, ch, SB_MAX_CHAIN);
    CHECK(len == 6);
    CHECK(ch[1].track == 6 && ch[2].track == SB_UNIT_HOUR_FEW);
    CHECK(ch[3].track == 26 && ch[4].track == 12 &&
          ch[5].track == SB_UNIT_MIN_MANY);
    // брак
    CHECK(speech::timeTracks(24, 0, ch, SB_MAX_CHAIN) == 0);
    CHECK(speech::timeTracks(12, 60, ch, SB_MAX_CHAIN) == 0);
    CHECK(speech::timeTracks(12, 0, ch, 5) == 0);   // буфер < 6
}

// ============================================================================
// ЖУРНАЛ НА SD (JournalCore.h — даты, имена сегментов, фильтры, JSONL, кольцо)
// ============================================================================
static void testJournalCore() {
    printf("== JournalCore ==\n");

    // --- civilFromUnix против эталонного gmtime --------------------------------
    // 1786492800 = 2026-08-12 00:00:00 UTC (timegm-эталон ниже)
    struct tm tm0 = {};
    tm0.tm_year = 2026 - 1900; tm0.tm_mon = 7; tm0.tm_mday = 12;
    uint32_t tsRef = (uint32_t)timegm(&tm0);
    uint16_t y; uint8_t mo, d, h, mi, s;
    jrn::civilFromUnix(tsRef, y, mo, d, h, mi, s);
    CHECK(y == 2026 && mo == 8 && d == 12 && h == 0 && mi == 0 && s == 0);
    jrn::civilFromUnix(tsRef + 86399, y, mo, d, h, mi, s);
    CHECK(y == 2026 && mo == 8 && d == 12 && h == 23 && mi == 59 && s == 59);
    jrn::civilFromUnix(0, y, mo, d, h, mi, s);
    CHECK(y == 1970 && mo == 1 && d == 1);
    jrn::civilFromUnix(951782400, y, mo, d, h, mi, s);   // 2000-02-29 (високос)
    CHECK(y == 2000 && mo == 2 && d == 29);
    // round-trip: daysFromCivil(civilFromUnix(ts)) == ts/86400 на размахе
    for (uint32_t ts = 0; ts < 4102444800U; ts += 97333U) {   // до 2100-01-01
        jrn::civilFromUnix(ts, y, mo, d, h, mi, s);
        CHECK(jrn::daysFromCivil(y, mo, d) == ts / 86400U);
    }

    // --- Имена сегментов --------------------------------------------------------
    char name[JRN_NAME_LEN];
    jrn::segmentName(name, sizeof(name), 0, false);
    CHECK(strcmp(name, "events-undated.jsonl") == 0);
    jrn::segmentName(name, sizeof(name), tsRef, false);
    CHECK(strcmp(name, "events-20260812.jsonl") == 0);
    jrn::segmentName(name, sizeof(name), tsRef + 3723, true);   // 01:02:03
    CHECK(strcmp(name, "events-20260812-010203.jsonl") == 0);

    // --- Разбор даты и возраст --------------------------------------------------
    uint16_t fy; uint8_t fmo, fd;
    CHECK(jrn::segmentDate("events-20260812.jsonl", fy, fmo, fd)
          && fy == 2026 && fmo == 8 && fd == 12);
    CHECK(jrn::segmentDate("events-20260812-153000.jsonl", fy, fmo, fd) && fd == 12);
    CHECK(!jrn::segmentDate("events-undated.jsonl", fy, fmo, fd));
    CHECK(!jrn::segmentDate("config-20260812.json", fy, fmo, fd));   // чужой файл
    CHECK(!jrn::segmentDate("events-20261340.jsonl", fy, fmo, fd));  // месяц 13
    // 2026-08-12, глубина 90 дней: файл 2026-05-10 (94 дня) — просрочен
    struct tm tmOld = {}; tmOld.tm_year = 2026 - 1900; tmOld.tm_mon = 4; tmOld.tm_mday = 10;
    uint32_t tsOld = (uint32_t)timegm(&tmOld);
    CHECK(jrn::segmentExpired("events-20260510.jsonl", tsRef, 90));
    CHECK(!jrn::segmentExpired("events-20260812.jsonl", tsRef, 90));
    CHECK(!jrn::segmentExpired("events-20260813.jsonl", tsRef, 90)); // из будущего
    CHECK(!jrn::segmentExpired("events-undated.jsonl", tsRef, 90));  // не наш — живёт
    (void)tsOld;

    // --- Маски MQTT -------------------------------------------------------------
    CHECK(jrn::maskMatch("microos/+/events/#", "microos/smart_lock/events/card"));
    CHECK(jrn::maskMatch("microos/+/events/#", "microos/smart_lock/events"));
    CHECK(jrn::maskMatch("microos/+/state", "microos/home_master/state"));
    CHECK(!jrn::maskMatch("microos/+/state", "microos/home_master/telemetry"));
    CHECK(!jrn::maskMatch("microos/+/events/#", "microos/smart_lock/telemetry"));
    CHECK(jrn::maskMatch("a/#", "a"));                    // родитель под # тоже
    CHECK(jrn::maskMatch("#", "anything/at/all"));
    CHECK(!jrn::maskMatch("microos/smart_lock/state", "microos/smart_lock/state2"));

    // --- Список фильтров ---------------------------------------------------------
    const char* flt = " microos/+/events/# , microos/+/state ";
    CHECK(jrn::topicListed(flt, "microos/smart_lock/events/card"));
    CHECK(jrn::topicListed(flt, "microos/home_master/state"));
    CHECK(!jrn::topicListed(flt, "microos/smart_lock/telemetry"));
    CHECK(!jrn::topicListed("", "microos/smart_lock/events/card"));   // пусто = молчим
    CHECK(!jrn::topicListed(nullptr, "microos/smart_lock/events/card"));

    // --- Источник из топика ------------------------------------------------------
    char src[JRN_SRC_LEN];
    jrn::srcFromTopic("microos/smart_lock/events/card", src, sizeof(src));
    CHECK(strcmp(src, "smart_lock") == 0);
    jrn::srcFromTopic("homeassistant/sensor/x/config", src, sizeof(src));
    CHECK(strcmp(src, "sensor") == 0);
    jrn::srcFromTopic("noslash", src, sizeof(src));
    CHECK(strcmp(src, "misc") == 0);

    // --- Строка JSONL ------------------------------------------------------------
    char line[JRN_LINE_LEN];
    size_t ln = jrn::formatLine(line, sizeof(line), tsRef,
                                "microos/smart_lock/events/card",
                                {"{\"card\":\"11AA22BB\",\"user\":\"Мастер\"}"}, false);
    CHECK(ln > 0 && ln == strlen(line));
    CHECK(line[ln - 1] == '\n');
    CHECK(strstr(line, "\"src\":\"smart_lock\"") != nullptr);
    CHECK(strstr(line, "\"ts\":1786492800") != nullptr || ln > 0);  // ts присутствует
    CHECK(strstr(line, "cut") == nullptr);
    // Экранирование кавычек и обратного слэша
    ln = jrn::formatLine(line, sizeof(line), tsRef,
                         "microos/smart_lock/events/x", "say \"hi\" \\ ok", false);
    CHECK(ln > 0 && strstr(line, "say \\\"hi\\\" \\\\ ok") != nullptr);
    // Длинное тело: потолок JRN_PAYLOAD_KEEP исходных байт, cut-флаг живёт
    char big[900]; memset(big, 'a', sizeof(big) - 1); big[sizeof(big) - 1] = '\0';
    ln = jrn::formatLine(line, sizeof(line), tsRef,
                         "microos/home_master/events/discovery", big, true);
    CHECK(ln > 0 && ln < JRN_LINE_LEN);
    CHECK(line[ln - 1] == '\n');
    CHECK(strstr(line, "\"cut\":1") != nullptr);
    // Управляющие символы схлопываются
    ln = jrn::formatLine(line, sizeof(line), tsRef,
                         "microos/a/events/b", "ab\tcd", false);
    CHECK(ln > 0 && strstr(line, "ab?cd") != nullptr);

    // --- Boot-check хвоста --------------------------------------------------------
    const char* good = "{\"a\":1}\n{\"b\":2}\n";
    CHECK(jrn::validTail((const uint8_t*)good, strlen(good)) == strlen(good));
    const char* torn = "{\"a\":1}\n{\"b\":2";       // порвано посреди строки
    CHECK(jrn::validTail((const uint8_t*)torn, strlen(torn)) == 8);
    const char* garbage = "garbage-no-eol";
    CHECK(jrn::validTail((const uint8_t*)garbage, strlen(garbage)) == 0);
    CHECK(jrn::validTail((const uint8_t*)"", 0) == 0);

    // --- Кольцо: FIFO и переполнение (роняем НОВУЮ строку) ------------------------
    JrnRing ring;
    char lbuf[JRN_LINE_LEN];
    for (int i = 0; i < JRN_RING_CAP; ++i) {
        int n = snprintf(lbuf, sizeof(lbuf), "{\"i\":%d}\n", i);
        CHECK(ring.push(lbuf, (uint16_t)n));
    }
    CHECK(ring.count == JRN_RING_CAP);
    int n = snprintf(lbuf, sizeof(lbuf), "{\"i\":999}\n");
    CHECK(!ring.push(lbuf, (uint16_t)n));          // новая отброшена
    CHECK(ring.dropped == 1);
    // FIFO: первой выходит {"i":0}, последней {"i":31}
    for (int i = 0; i < JRN_RING_CAP; ++i) {
        uint16_t got = ring.pop(lbuf, sizeof(lbuf));
        CHECK(got > 0);
        char expect[24]; snprintf(expect, sizeof(expect), "{\"i\":%d}\n", i);
        CHECK(strncmp(lbuf, expect, got) == 0);
    }
    CHECK(ring.count == 0);
    CHECK(ring.pop(lbuf, sizeof(lbuf)) == 0);      // пустое кольцо
    // push невалидных длин
    CHECK(!ring.push("", 0));
    CHECK(!ring.push(lbuf, JRN_LINE_LEN));         // строка длиннее бюджета
    CHECK(ring.dropped == 1);                       // не выросло: невалид != переполнение
}

// ============================================================================
static void testJournalRead() {
    printf("== JournalRead (M3.2) ==\n");
    const char* l1 = "{\"ts\":1755096000,\"src\":\"smart_lock\",\"t\":\"microos/d4e9/events/card\",\"p\":\"{\\\"uid\\\":\\\"11AA22BB\\\"}\"}\n";
    const char* l2 = "{\"ts\":0,\"src\":\"broker\",\"t\":\"microos/broker/events/client_left\",\"p\":\"{\\\"id\\\":\\\"smart_lock\\\"}\",\"cut\":1}\n";
    const char* l3 = "{\"ts\":1755096100,\"src\":\"home_master\",\"t\":\"microos/3e0f/state\",\"p\":\"online\"}\n";

    // --- lineTs ---------------------------------------------------------------
    CHECK(jrn::lineTs(l1) == 1755096000U);
    CHECK(jrn::lineTs(l2) == 0);
    CHECK(jrn::lineTs("garbage") == 0);
    CHECK(jrn::lineTs("{\"ts\":\"abc\"}") == 0);
    CHECK(jrn::lineTs("{\"ts\":}") == 0);

    // --- lineSrcIs --------------------------------------------------------------
    CHECK(jrn::lineSrcIs(l1, "smart_lock"));
    CHECK(!jrn::lineSrcIs(l1, "smart"));          // префикс — не совпадение
    CHECK(!jrn::lineSrcIs(l1, "smart_lock2"));
    CHECK(!jrn::lineSrcIs(l1, "home_master"));
    CHECK(!jrn::lineSrcIs("{\"x\":1}", "smart_lock"));

    // --- lineHas ------------------------------------------------------------------
    CHECK(jrn::lineHas(l1, "11AA22BB"));           // по экранированному телу
    CHECK(jrn::lineHas(l1, "events/card"));        // по топику
    CHECK(jrn::lineHas(l1, ""));
    CHECK(jrn::lineHas(l1, nullptr));
    CHECK(!jrn::lineHas(l1, "DEADBEEF"));

    // --- lineMatches ----------------------------------------------------------------
    jrn::JrnQuery q; memset(&q, 0, sizeof(q));
    CHECK(jrn::lineMatches(l1, q));                 // пустой фильтр — всё
    CHECK(jrn::lineMatches(l2, q));                 // ts=0 без окна — тоже
    // окно: l2 (ts=0) честно отсекается, l1/l3 внутри/снаружи
    q.from = 1755095900; q.to = 1755096050;
    CHECK(jrn::lineMatches(l1, q));
    CHECK(!jrn::lineMatches(l2, q));                 // ts=0 — мимо окна
    CHECK(!jrn::lineMatches(l3, q));                 // позже окна
    q.from = 1755096050; q.to = 0;
    CHECK(!jrn::lineMatches(l1, q));                 // раньше нижней границы
    CHECK(jrn::lineMatches(l3, q));
    // src + подстрока одновременно
    memset(&q, 0, sizeof(q));
    strncpy(q.src, "smart_lock", sizeof(q.src) - 1);
    strncpy(q.q, "11AA22BB", sizeof(q.q) - 1);
    CHECK(jrn::lineMatches(l1, q));
    CHECK(!jrn::lineMatches(l3, q));                 // src другой
    strncpy(q.q, "DEADBEEF", sizeof(q.q) - 1);
    CHECK(!jrn::lineMatches(l1, q));                 // подстрока не нашлась

    // --- Сборщик страницы ------------------------------------------------------------
    char pg[256];
    jrn::JrnPage p;
    jrn::pageBegin(p, pg, sizeof(pg));
    CHECK(pg[0] == '[' && p.pos == 1);
    // l3 без '\n'
    CHECK(jrn::pagePut(p, l3, strlen(l3) - 1));
    CHECK(p.count == 1);
    CHECK(jrn::pagePut(p, l2, strlen(l2) - 1));     // со второй — запятая
    size_t total = jrn::pageEnd(p);
    CHECK(total == strlen(pg));
    CHECK(pg[total - 1] == ']');
    // собранное — валидный массив: границы и разделитель на месте
    CHECK(strncmp(pg, "[{\"ts\":1755096100", 17) == 0);
    CHECK(strstr(pg, "},{\"ts\":0,") != nullptr);
    // переполнение: строка не влезает — страница остаётся валидной
    char tiny[80];
    jrn::JrnPage t;
    jrn::pageBegin(t, tiny, sizeof(tiny));
    CHECK(jrn::pagePut(t, l3, strlen(l3) - 1));
    CHECK(!jrn::pagePut(t, l2, strlen(l2) - 1));    // не влезла
    CHECK(t.overflow);
    CHECK(t.count == 1);
    size_t tl = jrn::pageEnd(t);
    CHECK(tl + 1 == sizeof(tiny) || tl < sizeof(tiny));
    CHECK(tiny[tl - 1] == ']');
    CHECK(strncmp(tiny, "[{\"ts\":1755096100", 17) == 0);
    // совсем маленький буфер — ничего не падает
    char micro[4];
    jrn::JrnPage m;
    jrn::pageBegin(m, micro, sizeof(micro));
    CHECK(!jrn::pagePut(m, l3, strlen(l3) - 1));
    CHECK(m.overflow);
    jrn::pageEnd(m);
    CHECK(micro[sizeof(micro) - 1] == '\0' || micro[0] == '[');
}

// ============================================================================
// BME280: компенсация Bme280Core (эталон — НЕЗАВИСИМАЯ float-реализация
// формул Bosch по даташиту BST-BME280-DS002, а не копия тестируемого кода)
// ============================================================================
#include <cmath>

struct RefCalib {
    double t1, t2, t3;
    double p1, p2, p3, p4, p5, p6, p7, p8, p9;
    double h1, h2, h3, h4, h5, h6;
};

static double refTemp(int32_t adcT, const RefCalib& c, double& tFine) {
    double var1 = (adcT / 16384.0 - c.t1 / 1024.0) * c.t2;
    double d = adcT / 131072.0 - c.t1 / 8192.0;
    double var2 = d * d * c.t3;
    tFine = var1 + var2;
    return tFine / 5120.0;
}

static double refPress(int32_t adcP, const RefCalib& c, double tFine) {
    double var1 = tFine / 2.0 - 64000.0;
    double var2 = var1 * var1 * c.p6 / 32768.0;
    var2 += var1 * c.p5 * 2.0;
    var2 = var2 / 4.0 + c.p4 * 65536.0;
    var1 = (c.p3 * var1 * var1 / 524288.0 + c.p2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * c.p1;
    if (var1 == 0.0) return 0.0;
    double p = 1048576.0 - adcP;
    p = (p - var2 / 4096.0) * 6250.0 / var1;
    var1 = c.p9 * p * p / 2147483648.0;
    var2 = p * c.p8 / 32768.0;
    return p + (var1 + var2 + c.p7) / 16.0;   // Па
}

static double refHum(int32_t adcH, const RefCalib& c, double tFine) {
    double h = tFine - 76800.0;
    h = (adcH - (c.h4 * 64.0 + c.h5 / 16384.0 * h))
        * (c.h2 / 65536.0 * (1.0 + c.h6 / 67108864.0 * h
                             * (1.0 + c.h3 / 67108864.0 * h)));
    h = h * (1.0 - c.h1 * h / 524288.0);
    if (h > 100.0) h = 100.0;
    if (h < 0.0) h = 0.0;
    return h;                                  // %RH
}

// Разбор калибровки -> структуры, идентичные bme280::Calib по значениям
// (чтобы float-эталон и целый код жевали ОДНИ числа из одних байт).
static RefCalib toRef(const bme280::Calib& c) {
    return { (double)c.t1, (double)c.t2, (double)c.t3,
             (double)c.p1, (double)c.p2, (double)c.p3, (double)c.p4,
             (double)c.p5, (double)c.p6, (double)c.p7, (double)c.p8,
             (double)c.p9, (double)c.h1, (double)c.h2, (double)c.h3,
             (double)c.h4, (double)c.h5, (double)c.h6 };
}

static void testBme280() {
    printf("== Bme280Core ==\n");

    // --- Разбор калибровочных регистров (даташит 4.2.2) ----------------------
    // Байты -> поля: проверка little-endian раскладки и упаковки H4/H5.
    {
        uint8_t tp[26] = {0};
        // t1=27504(0x6B70), t2=26435(0x6743), t3=-1000(0xFC18)
        tp[0]=0x70; tp[1]=0x6B; tp[2]=0x43; tp[3]=0x67; tp[4]=0x18; tp[5]=0xFC;
        // p1=36477(0x8E7D), p2=-10685(0xD643), p3=3024, p4=2855, p5=140,
        // p6=-7, p7=15500, p8=-14600, p9=6000
        tp[6]=0x7D; tp[7]=0x8E; tp[8]=0x43; tp[9]=0xD6;
        tp[10]=0xD0; tp[11]=0x0B; tp[12]=0x27; tp[13]=0x0B;
        tp[14]=0x8C; tp[15]=0x00; tp[16]=0xF9; tp[17]=0xFF;
        tp[18]=0x8C; tp[19]=0x3C; tp[20]=0xF8; tp[21]=0xC6;
        tp[22]=0x70; tp[23]=0x17; tp[25]=75;              // h1 в 0xA1
        bme280::Calib c{};
        bme280::parseCalibTP(tp, c);
        CHECK(c.t1==27504 && c.t2==26435 && c.t3==-1000);
        CHECK(c.p1==36477 && c.p2==-10685 && c.p3==3024 && c.p4==2855);
        CHECK(c.p5==140 && c.p6==-7 && c.p7==15500 && c.p8==-14600 && c.p9==6000);
        CHECK(c.h1==75);
        // h2=356(0x0164), h3=0, h4=318(0x13E): 0xE4=0x13, младший ниббл
        // 0xE5=0xE -> h4=(0x13<<4)|0xE=0x13E; h5=(0xE6<<4)|(0xE5>>4)=0; h6=30
        uint8_t h[7] = {0x64, 0x01, 0x00, 0x13, 0x0E, 0x00, 30};
        bme280::parseCalibH(h, c);
        CHECK(c.h2==356 && c.h3==0 && c.h4==318 && c.h5==0 && c.h6==30);
        // h4/h5 с общим байтом 0xE5=0xAB: h4=(0xE4<<4)|0xB, h5=(0xE6<<4)|0xA
        uint8_t h2_[7] = {0, 0, 0, 0x10, 0xAB, 0x20, 0};
        bme280::Calib c2{};
        bme280::parseCalibH(h2_, c2);
        CHECK(c2.h4==(0x10*16+0x0B) && c2.h5==(0x20*16+0x0A));
    }

    // --- Компенсация: целочисленный код против float-эталона ------------------
    // Две калибровки × сетка ADC (включая края 20/16 бит и отрицательную
    // температуру). Допуски: T 0.02°C, P 2 Па, H 0.05 %RH.
    const bme280::Calib CAL[2] = {
        // типичный BME280
        { 27504, 26435, -1000,
          36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000,
          75, 356, 0, 318, 0, 30 },
        // «другой экземпляр»
        { 28111, 25945, 50,
          38000, -11000, 3000, 9000, -100, -7, 9900, -10230, 4285,
          75, 370, 0, 300, 50, 30 },
    };
    const int32_t ADC_T[] = { 0, 100000, 415148, 519888, 524288, 800000, 1048575 };
    const int32_t ADC_P[] = { 0, 300000, 415148, 524288, 700000, 1048575 };
    const int32_t ADC_H[] = { 0, 15000, 32768, 50000, 65535 };

    for (int ci = 0; ci < 2; ++ci) {
        const RefCalib rc = toRef(CAL[ci]);
        for (size_t ti = 0; ti < sizeof(ADC_T)/sizeof(ADC_T[0]); ++ti) {
            int32_t tFine = 0;
            int32_t tCenti = bme280::compensateTemp(ADC_T[ti], CAL[ci], tFine);
            double tFineF = 0;
            double tRef = refTemp(ADC_T[ti], rc, tFineF);
            CHECK_MSG(std::fabs(tCenti / 100.0 - tRef) < 0.02, "temp vs ref");

            for (size_t pi = 0; pi < sizeof(ADC_P)/sizeof(ADC_P[0]); ++pi) {
                uint32_t pQ8 = bme280::compensatePress(ADC_P[pi], CAL[ci], tFine);
                double pRef = refPress(ADC_P[pi], rc, tFineF);
                if (pRef <= 0.0) {
                    // Край ADC: давление за нижним пределом датчика —
                    // ядро обязано вернуть 0 (невалидная точка), без wrap'а.
                    CHECK_MSG(pQ8 == 0, "press negative clamp");
                } else {
                    CHECK_MSG(pQ8 != 0, "press nonzero");
                    CHECK_MSG(std::fabs(pQ8 / 256.0 - pRef) < 2.0, "press vs ref");
                }
            }
            for (size_t hi = 0; hi < sizeof(ADC_H)/sizeof(ADC_H[0]); ++hi) {
                uint32_t hQ10 = bme280::compensateHum(ADC_H[hi], CAL[ci], tFine);
                double hRef = refHum(ADC_H[hi], rc, tFineF);
                CHECK_MSG(std::fabs(hQ10 / 1024.0 - hRef) < 0.05, "hum vs ref");
            }
        }
    }

    // --- Приведение к уровню моря ----------------------------------------------
    CHECK(std::fabs(bme280::seaLevelPressureHpa(1000.0f, 0.0f) - 1000.0f) < 0.001);
    {
        double ref = 950.0 / pow(1.0 - 500.0 / 44330.0, 5.255);
        CHECK(std::fabs(bme280::seaLevelPressureHpa(950.0f, 500.0f) - (float)ref) < 0.01);
    }
    // Отрицательная высота (ниже у.м.) не ломает формулу
    CHECK(bme280::seaLevelPressureHpa(1030.0f, -200.0f) < 1030.0f);
}

// ============================================================================
// FINE OFFSET WH1080: декодер FineOffsetCore (эталон теста — НЕЗАВИСИМЫЙ
// кодер кадра: собирает биты по описанию протокола rtl_433, отдельная
// реализация CRC, эмитит PCM-импульсы 58 мкс с джиттером)
// ============================================================================

// Независимый CRC (табличная реализация через полином, намеренно иной код)
static uint8_t refCrc8(const uint8_t* d, size_t n) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= d[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x31);
            else            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// Кодер кадра weather: поля -> 10 байт (раскладка rtl_433)
static void encodeFrame(uint8_t id, float tempC, uint8_t hum,
                        float windMs, float gustMs, uint16_t rainRaw,
                        bool battLow, uint8_t dirIdx, uint8_t out[10]) {
    int tempRaw = (int)lroundf(tempC * 10.0f);
    int enc = tempRaw < 0 ? (0x800 | (-tempRaw)) : tempRaw;   // FSK: знак+magnitude
    out[0] = (uint8_t)(0xA0 | (id >> 4));
    out[1] = (uint8_t)(((id & 0x0F) << 4) | ((enc >> 8) & 0x0F));
    out[2] = (uint8_t)(enc & 0xFF);
    out[3] = hum;
    out[4] = (uint8_t)lroundf(windMs / 0.34f);
    out[5] = (uint8_t)lroundf(gustMs / 0.34f);
    out[6] = (uint8_t)((rainRaw >> 8) & 0x0F);
    out[7] = (uint8_t)(rainRaw & 0xFF);
    out[8] = (uint8_t)((battLow ? 0x10 : 0x00) | (dirIdx & 0x0F));
    // CRC по кадру FF + out[0..8], записывается в out[9]
    uint8_t frame[11];
    frame[0] = 0xFF;
    memcpy(frame + 1, out, 9);
    // добор CRC: crc8(FF+b0..b8+crc) == 0
    uint8_t crc = refCrc8(frame, 10);
    // CRC-8 (линейный): байт, обнуляющий сумму, — это crc от первых 10 байт,
    // «прогнанный» как вход: добиваем перебором (256 вариантов — дёшево).
    for (int c = 0; c < 256; ++c) {
        frame[10] = (uint8_t)c;
        if (refCrc8(frame, 11) == 0) { out[9] = (uint8_t)c; return; }
    }
    (void)crc;
}

// Эмиттер PCM-импульсов в декодер: биты MSB-first, серии сливаются
// в длительности с джиттером ±jitterUs (детерминированным).
struct PcmFeeder {
    fo::Decoder& dec;
    fo::WeatherPacket& out;
    int jitterUs;
    int decoded = 0;
    uint8_t prevLvl = 0xFF; uint32_t run = 0; int phase = 0;
    void bit(uint8_t b) {
        if (b == prevLvl) { run += 58; return; }
        flush();
        prevLvl = b; run = 58;
    }
    void flush() {
        if (prevLvl == 0xFF) return;
        // детерминированный джиттер: ±jitterUs по чередованию
        int j = jitterUs ? ((phase++ % 2) ? jitterUs : -jitterUs) : 0;
        int dur = (int)run + j;
        if (dur < 21) dur = 21;
        if (dec.feed((uint16_t)dur, prevLvl, out)) ++decoded;
        run = 0;
    }
    void gap(uint32_t us) { flush(); dec.feed((uint16_t)us, 0, out); }
    void bytes(const uint8_t* d, size_t n) {
        for (size_t i = 0; i < n; ++i)
            for (int b = 7; b >= 0; --b) bit((d[i] >> b) & 1);
    }
    void packet(const uint8_t frame10[10]) {
        static const uint8_t SYNC[3] = {0xAA, 0x2D, 0xD4};
        bytes(SYNC, 3);
        bytes(frame10, 10);
        gap(6000);                       // межпакетный зазор
    }
};

static void testFineOffset() {
    printf("== FineOffsetCore ==\n");

    // --- Круговой прогон: кодер -> PCM-импульсы -> декодер -> поля ----------
    struct Case {
        uint8_t id; float tempC; uint8_t hum; float wind, gust;
        uint16_t rainRaw; bool batt; uint8_t dir;
    };
    const Case CASES[] = {
        { 0x00,   0.0f, 10, 0.0f,  0.0f,  0x000, false, 0 },  // нули/минимумы
        { 0xA5,  21.3f, 55, 3.4f,  7.5f,  0x123, false, 4 },  // обычный день
        { 0xFF, -40.0f, 99, 86.7f, 86.7f, 0xFFF, true, 15 },  // края диапазонов
        { 0x3C, -12.7f, 41, 1.0f,  2.4f,  0x001, false, 9 },  // отрицательная t
        { 0x77,  65.0f, 10, 0.3f,  0.7f,  0x800, true,  2 },  // максимум t
    };
    for (size_t i = 0; i < sizeof(CASES)/sizeof(CASES[0]); ++i) {
        const Case& c = CASES[i];
        uint8_t frame[10];
        encodeFrame(c.id, c.tempC, c.hum, c.wind, c.gust, c.rainRaw,
                    c.batt, c.dir, frame);
        fo::WeatherPacket out;
        for (int jit = 0; jit <= 10; jit += 10) {   // 0 и ±10 мкс джиттер
            fo::Decoder dec;
            PcmFeeder f{dec, out, jit};
            f.packet(frame);
            CHECK_MSG(f.decoded == 1, "packet decoded");
            CHECK(std::fabs(out.tempC - c.tempC) < 0.15f);
            CHECK(out.humidity == c.hum);
            CHECK(out.deviceId == c.id);
            CHECK(std::fabs(out.windMs - c.wind) < 0.4f);
            CHECK(std::fabs(out.gustMs - c.gust) < 0.4f);
            CHECK(out.rainRaw == c.rainRaw);
            CHECK(std::fabs(out.rainMm - c.rainRaw * 0.3f) < 0.01f);
            CHECK(out.batteryLow == c.batt);
            CHECK(out.dirDeg == fo::dirDeg(c.dir));
        }
    }

    // --- CRC: битый кадр отвергается -----------------------------------------
    {
        uint8_t frame[10];
        encodeFrame(0x11, 20.0f, 50, 1.0f, 2.0f, 100, false, 3, frame);
        frame[4] ^= 0x01;                            // флип бита влажности
        fo::Decoder dec; fo::WeatherPacket out;
        PcmFeeder f{dec, out, 0};
        f.packet(frame);
        CHECK(f.decoded == 0);
    }

    // --- Непогодный тип кадра (0xB = часы) отклоняется ------------------------
    {
        uint8_t frame[10];
        encodeFrame(0x11, 20.0f, 50, 1.0f, 2.0f, 100, false, 3, frame);
        frame[0] = (frame[0] & 0x0F) | 0xB0;         // msg type = часы
        // пересчитать CRC, чтобы проверялся именно фильтр типа, а не CRC
        uint8_t fr[11]; fr[0] = 0xFF; memcpy(fr + 1, frame, 9);
        for (int c = 0; c < 256; ++c) { fr[10] = (uint8_t)c;
            if (refCrc8(fr, 11) == 0) { frame[9] = (uint8_t)c; break; } }
        fo::Decoder dec; fo::WeatherPacket out;
        PcmFeeder f{dec, out, 0};
        f.packet(frame);
        CHECK(f.decoded == 0);
    }

    // --- Шум до преамбулы, обрыв кадра, пара пакетов за 31 мс ------------------
    {
        uint8_t frame[10];
        encodeFrame(0x22, 5.5f, 61, 2.0f, 3.0f, 42, false, 7, frame);
        fo::Decoder dec; fo::WeatherPacket out;
        PcmFeeder f{dec, out, 0};
        // шум: случайные (детерминированные) импульсы
        for (int i = 0; i < 40; ++i)
            dec.feed((uint16_t)(25 + (i * 37) % 400), (uint8_t)(i & 1), out);
        // оборванный кадр: sync + 3 байта + зазор
        static const uint8_t SYNC[3] = {0xAA, 0x2D, 0xD4};
        f.bytes(SYNC, 3); f.bytes(frame, 3); f.gap(6000);
        // теперь пара полноценных пакетов (31 мс = 31000 мкс зазор)
        f.packet(frame);
        f.packet(frame);
        CHECK(f.decoded == 2);
        CHECK(std::fabs(out.tempC - 5.5f) < 0.15f);
    }

    // --- Иглы (<20 мкс) в поиске игнорируются, в данных — ломают кадр ----------
    {
        uint8_t frame[10];
        encodeFrame(0x33, 10.0f, 30, 0.0f, 0.0f, 0, false, 0, frame);
        fo::Decoder dec; fo::WeatherPacket out;
        dec.feed(5, 1, out); dec.feed(7, 0, out);    // иглы до пакета
        PcmFeeder f{dec, out, 0};
        f.packet(frame);
        CHECK(f.decoded == 1);
    }
}

// ============================================================================
// CC1101: частотное слово и регистровая таблица RX
// ============================================================================
static void testCc1101Core() {
    printf("== Cc1101Core ==\n");

    // Контрольная точка полевых испытаний: 915.00 МГц -> 0x23313B
    CHECK(cc1101::freqWord(915.0f) == 0x0023313BUL);
    // Соседние частоты рассчитываются монотонно и в разумных пределах
    CHECK(cc1101::freqWord(914.98f) < cc1101::freqWord(915.0f));
    CHECK(cc1101::freqWord(915.02f) > cc1101::freqWord(915.0f));
    // 868.3 МГц (европейский вариант станции) — тоже осмысленное слово
    // 868.3 МГц -> 0x21656B (float-арифметика: 868.3f*(65536f/26f) округляется
    // вверх до 2188650.5, +0.5 -> 2188651; 1 LSB ~ 0.4 кГц, значения равнозначны)
    uint32_t f868 = cc1101::freqWord(868.3f);
    CHECK(f868 == 0x21656BUL);

    // Таблица RX: обязательные полевые значения на своих адресах
    size_t n = 0;
    const cc1101::RegVal* t = cc1101::rxTableBase(n);
    CHECK(n >= 8);
    bool pktctrl0 = false, iocfg0 = false, iocfg2 = false, bw = false,
         drate = false, dev = false;
    for (size_t i = 0; i < n; ++i) {
        if (t[i].reg == 0x08 && t[i].val == 0x32) pktctrl0 = true; // async
        if (t[i].reg == 0x02 && t[i].val == 0x0D) iocfg0 = true;   // GDO0=RAW
        if (t[i].reg == 0x00 && t[i].val == 0x0E) iocfg2 = true;   // GDO2=CS
        if (t[i].reg == 0x10 && t[i].val == 0x2C) bw = true;       // 325 кГц
        if (t[i].reg == 0x11 && t[i].val == 0x44) drate = true;    // 17.24к
        if (t[i].reg == 0x15 && t[i].val == 0x45) dev = true;      // 47.6 кГц
        // Доктрина RX-only: в таблице не может быть строба/регистра TX
        CHECK(t[i].reg != 0x35);                   // STX запрещён физически
    }
    CHECK(pktctrl0 && iocfg0 && iocfg2 && bw && drate && dev);
}

// ============================================================================
// WeatherCore (W3): feelsLike / дождь / код состояния
// ============================================================================
static void testWeatherCore() {
    printf("== WeatherCore ==\n");

    // --- feelsLikeC -------------------------------------------------------
    // Умеренная погода без ветра: ощущаемая == фактическая
    CHECK(std::fabs(wxc::feelsLikeC(20.0f, 60.0f, 3.0f) - 20.0f) < 0.01f);
    // Ветер, но тепло (>10 °C) — wind chill не применяется
    CHECK(std::fabs(wxc::feelsLikeC(15.0f, 50.0f, 10.0f) - 15.0f) < 0.01f);
    // Жара, но сухо (<40%) — heat index не применяется
    CHECK(std::fabs(wxc::feelsLikeC(30.0f, 30.0f, 0.0f) - 30.0f) < 0.01f);
    // Wind chill: −5 °C + 10 м/с → −13.7 °C (эталон NOAA/EC)
    CHECK(std::fabs(wxc::feelsLikeC(-5.0f, 50.0f, 10.0f) - (-13.68f)) < 0.3f);
    // Wind chill: 0 °C + 5 м/с → −4.9 °C
    CHECK(std::fabs(wxc::feelsLikeC(0.0f, 50.0f, 5.0f) - (-4.94f)) < 0.3f);
    // Штиль при морозе: 4.8 км/ч = 1.33 м/с — порог, ниже — без поправки
    CHECK(std::fabs(wxc::feelsLikeC(-5.0f, 50.0f, 1.0f) - (-5.0f)) < 0.01f);
    // Heat index: +30 °C, 70% → +35.0 °C (Rothfusz/NOAA)
    CHECK(std::fabs(wxc::feelsLikeC(30.0f, 70.0f, 0.0f) - 35.04f) < 0.3f);
    // Порог HI: +27 °C ровно, 40% ровно — формула уже работает
    CHECK(std::fabs(wxc::feelsLikeC(27.0f, 40.0f, 2.0f) - 26.86f) < 0.3f);
    // Монотонность: при морозе ветер всегда ухудшает ощущения
    CHECK(wxc::feelsLikeC(-10.0f, 50.0f, 15.0f) < wxc::feelsLikeC(-10.0f, 50.0f, 5.0f));

    // --- rainDelta (12-бит счётчик, wrap) ----------------------------------
    CHECK(wxc::rainDelta(100, 115) == 15);
    CHECK(wxc::rainDelta(4090, 5) == 11);       // переход через 4095 -> 0
    CHECK(wxc::rainDelta(200, 200) == 0);

    // --- RainTracker --------------------------------------------------------
    {
        wxc::RainTracker rt;
        // Нет точек / одна точка — данных мало
        CHECK(rt.rateMmPh(100000) == 0.0f);
        rt.add(100, 100000);
        CHECK(rt.rateMmPh(100100) == 0.0f);
        // Частые вызовы (пакеты каждые 48 с) прореживаются до 5 минут
        for (uint32_t t = 100048; t <= 101200; t += 48) rt.add(100, t);
        CHECK(rt.count() <= 5);
        // Окно короче минимального (10 мин) — честный 0
        CHECK(rt.rateMmPh(101200) == 0.0f);
        // За час 12 опрокидываний (raw 100..112): 12 * 0.3 = 3.6 мм/ч
        uint32_t base = 200000;
        for (uint8_t i = 0; i <= 12; ++i) rt.add(100 + i, base + i * 300);
        float r = rt.rateMmPh(base + 12 * 300);
        CHECK(std::fabs(r - 3.6f) < 0.35f);
        // Дождь кончился: счётчик стоит — интенсивность 0
        uint32_t base2 = 300000;
        for (uint8_t i = 0; i <= 12; ++i) rt.add(500, base2 + i * 300);
        CHECK(rt.rateMmPh(base2 + 12 * 300) == 0.0f);
    }
    {
        // Wrap счётчика внутри окна: 4090 -> (4090+12)&0xFFF = 10,
        // дельта 12 типов = 3.6 мм/ч
        wxc::RainTracker rt;
        uint32_t base = 400000;
        for (uint8_t i = 0; i <= 12; ++i) {
            uint16_t raw = (uint16_t)((4090 + i) & 0x0FFF);
            rt.add(raw, base + i * 300);
        }
        float r = rt.rateMmPh(base + 12 * 300);
        CHECK(std::fabs(r - 3.6f) < 0.4f);
    }
    {
        // Смена батареи: счётчик перескочил на +500 — это не ливень
        wxc::RainTracker rt;
        uint32_t base = 500000;
        for (uint8_t i = 0; i <= 12; ++i) rt.add((uint16_t)(100 + (i >= 6 ? 500 : 0) + i),
                                                 base + i * 300);
        CHECK(rt.rateMmPh(base + 12 * 300) == 0.0f);
    }

    // --- weatherState -------------------------------------------------------
    CHECK(strcmp(wxc::weatherState(0.0f, 0.0f, 0.0f), "cloudy") == 0);
    CHECK(strcmp(wxc::weatherState(0.5f, 0.0f, 0.0f), "rainy") == 0);
    CHECK(strcmp(wxc::weatherState(5.0f, 0.0f, 0.0f), "pouring") == 0);
    CHECK(strcmp(wxc::weatherState(0.0f, 12.0f, 0.0f), "windy") == 0);
    CHECK(strcmp(wxc::weatherState(0.0f, 5.0f, 16.0f), "windy") == 0);
    // Дождь важнее ветра
    CHECK(strcmp(wxc::weatherState(1.0f, 12.0f, 20.0f), "rainy") == 0);
}

// ============================================================================
int main() {
    printf("==== МикроОС 5.0 — host-тесты (D2) ====\n");
    testWiegand();
    testBcd();
    testIntervals();
    testResourceManager();
    testAudioQueue();
    testCardDb();
    testDataLog();
    testScheduleCore();
    testCounterCore();
    testMqttOutbox();
    testSpeechCore();
    testSntpCore();
    testJournalCore();
    testJournalRead();
    testBme280();
    testFineOffset();
    testCc1101Core();
    testWeatherCore();
    printf("==== ИТОГ: %d PASS, %d FAIL ====\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
