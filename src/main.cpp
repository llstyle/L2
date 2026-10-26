// =====================================================================
//  L2. Информационные технологии в микроконтроллере
//  Сбор сигнала и его программная обработка (версия 2)
//
//  Use case (из L1): домашнее Wi-Fi устройство обнаружения
//  вмешательства / удара / вибрации.
//  Датчик: MPU6050 (3-осевой акселерометр, цифровой интерфейс I2C).
//  Платформа: ESP32, Arduino framework + FreeRTOS. Симуляция: Wokwi.
//
//  Цепочка обработки (для каждой оси X, Y, Z):
//    Датчик -> Сбор (I2C) -> Проверка достоверности -> Медианный фильтр ->
//    Усреднение -> Преобразование (м/с^2) -> Физическая величина
//
//  Решение use case — два канала:
//    1) медленный канал: dev = |a_f - a_ref| по сглаженному сигналу,
//       обнаруживает сдвиг и наклон устройства, устойчив к шуму;
//    2) канал удара: dev_pk = |a - a_f| — насколько текущий достоверный
//       отсчёт (БЕЗ медианы и усреднения) отличается от сглаженного
//       сигнала. Это быстрая составляющая: удары длиной 1-2 отсчёта.
//  tamper = медленный канал ИЛИ удар за последние SHOCK_HOLD_MS.
//
//  Вывод в STDIO (Serial), формат CSV:
//    t_ms,dt_us,ax_raw,ay_raw,az_raw,ok,ax_f,ay_f,az_f,a_raw,a_f,dev,dev_pk,shock,tamper,mot
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// =====================================================================
//  1. КОНФИГУРАЦИЯ (все настраиваемые параметры)
// =====================================================================
// ---- Подключение ----
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ_HZ 400000 // Fast-mode I2C (datasheet: до 400 кГц)
#define LED_PIN 2          // индикатор "вмешательство"

// ---- MPU6050 (регистры из Register Map, rev. 4.2) ----
#define MPU_ADDR 0x68         // AD0 = GND
#define REG_MOT_THR 0x1F      // порог детектора движения
#define REG_MOT_DUR 0x20      // длительность для детектора движения
#define REG_INT_ENABLE 0x38   // разрешение прерываний
#define REG_INT_STATUS 0x3A   // флаги прерываний (бит 6 — MOT_INT)
#define REG_PWR_MGMT_1 0x6B   // управление питанием (выход из сна)
#define REG_ACCEL_CONFIG 0x1C // выбор диапазона акселерометра
#define REG_ACCEL_XOUT_H 0x3B // начало 6 байт данных X,Y,Z
#define REG_WHO_AM_I 0x75     // идентификатор, должен быть 0x68

// Диапазон акселерометра AFS_SEL = 2 -> ±8 g, чувствительность 4096 LSB/g
// (datasheet MPU-6000/6050, табл. "Accelerometer Specifications").
// ±2 g не подходит: удар по корпусу даёт несколько g, и при ±2 g
// датчик насыщается на 32767 — амплитуда удара теряется.
#define ACCEL_FS_G 8.0f
#define ACCEL_LSB_PER_G 4096.0f
#define ACCEL_CONFIG_VAL 0x10 // AFS_SEL = 2 (биты 4:3 = 10)
#define G_MS2 9.80665f        // стандартное ускорение свободного падения

// ---- Период дискретизации (vTaskDelayUntil) ----
#define SAMPLE_PERIOD_MS 20 // 20 мс -> fs = 50 Гц

// ---- Проверка достоверности (вместо ограничения по шкале) ----
// У цифрового датчика код не может выйти за int16, поэтому ограничение
// по шкале ничего не делает. Вместо него отсчёт проверяется на
// физическую возможность: устройство закреплено, свободного падения нет,
// значит модуль ускорения не может быть почти нулевым. |a| < 0,3 g
// означает сбой чтения (например, все байты 0xFF -> X=Y=Z=-1).
// Недостоверный отсчёт заменяется последним достоверным.
#define VALID_MIN_G 0.3f
#define SAT_RAW 32767 // код насыщения: отсчёт достоверен, но обрезан шкалой

// ---- Окна фильтров (медленный канал) ----
#define MEDIAN_WINDOW 5 // N — нечётное (3, 5, 7, 9 ...)
#define AVG_WINDOW 8    // M — окно скользящего среднего (в опыте 3: 32)

// ---- Решение use case ----
#define CALIB_SAMPLES 50          // отсчётов на калибровку покоя (1 с)
#define TAMPER_THRESHOLD_MS2 2.0f // медленный канал: порог dev, м/с^2 (~0,2 g)
#define SHOCK_THRESHOLD_MS2 4.9f  // канал удара: порог dev_pk, м/с^2 (0,5 g)
#define SHOCK_HOLD_MS 1000        // сколько держать тревогу после удара, мс

// ---- Детектор движения MPU6050 (аппаратный, внутри датчика) ----
// Работает на внутренней частоте датчика и не зависит от периода опроса.
// Значения подбираются на реальной плате.
#define MOTION_DETECT 1
#define MOT_THR_VAL 20 // порог
#define MOT_DUR_VAL 1  // длительность, мс

// ---- Вывод ----
#define PRINT_EVERY 1 // печатать каждый N-й отсчёт

// ---- Имитация (ТОЛЬКО для Wokwi) ----
// В Wokwi датчик выдаёт идеально чистые значения, заданные ползунками.
// Добавляется шум, медленный дрейф и сбои чтения шины I2C.
// На реальной плате поставить 0!
#define SIMULATE_DISTURBANCE 1
#define SIM_NOISE_G 0.025f    // амплитуда шума, g
#define SIM_DRIFT_G 0.04f     // амплитуда дрейфа, g
#define SIM_DRIFT_PERIOD_S 30 // период дрейфа, с
#define SIM_GLITCH_PROB 40    // вероятность сбоя чтения = 1 / SIM_GLITCH_PROB

// Имитация коротких ударов по оси X (опыт 4).
// Удары длиной 1, 2 и 3 отсчёта на 3-й, 6-й и 9-й секунде.
#define SIMULATE_SHOCK 0
#define SIM_SHOCK_G 1.5f // амплитуда удара, g

#if (MEDIAN_WINDOW % 2) == 0
#error "MEDIAN_WINDOW должно быть нечётным"
#endif

// =====================================================================
//  2. ТИПЫ ДАННЫХ
// =====================================================================
struct AccelRaw
{ // "сырые" данные датчика, коды LSB
    int32_t x, y, z;
};

struct MedianFilter
{ // кольцевой буфер медианного фильтра
    int32_t buf[MEDIAN_WINDOW];
    uint8_t idx;   // куда писать следующий отсчёт
    uint8_t count; // сколько отсчётов накоплено (<= N)
};

struct MovingAverage
{ // кольцевой буфер скользящего среднего
    int32_t buf[AVG_WINDOW];
    int32_t sum; // бегущая сумма -> O(1) на отсчёт
    uint8_t idx;
    uint8_t count;
};

// =====================================================================
//  3. ПРОТОТИПЫ ФУНКЦИЙ
// =====================================================================
bool mpuWriteReg(uint8_t reg, uint8_t val);
bool mpuReadReg(uint8_t reg, uint8_t &val);
bool mpuInit();
bool readRaw(AccelRaw &out);
int32_t simulateDisturbance(int32_t v, uint32_t t_ms, float phase);
int32_t simulateShock(uint32_t sample);
bool simulateGlitch(AccelRaw &r);
int checkSample(const AccelRaw &r);
void medianInit(MedianFilter &f);
int32_t medianFilter(MedianFilter &f, int32_t x);
void avgInit(MovingAverage &f);
float movingAverage(MovingAverage &f, int32_t x);
float rawToMs2(float raw);
float magnitude(float x, float y, float z);
void acquisitionTask(void *arg);

// =====================================================================
//  4. SETUP / LOOP
// =====================================================================
void setup()
{
    Serial.begin(115200);
    delay(200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    if (!mpuInit())
    {
        Serial.println("# ОШИБКА: MPU6050 не найден на шине I2C");
        while (true)
            delay(1000);
    }

    Serial.println();
    Serial.println("# L2 v2: MPU6050 tamper detection (slow channel + shock channel)");
    Serial.printf("# fs=%d Hz (T=%d ms), median N=%d, avg M=%d, range=±%.0f g, valid |a|>=%.1f g\n",
                  1000 / SAMPLE_PERIOD_MS, SAMPLE_PERIOD_MS, MEDIAN_WINDOW, AVG_WINDOW,
                  ACCEL_FS_G, VALID_MIN_G);
    Serial.printf("# thresholds: slow %.1f m/s^2, shock %.1f m/s^2, hold %d ms\n",
                  TAMPER_THRESHOLD_MS2, SHOCK_THRESHOLD_MS2, SHOCK_HOLD_MS);
    Serial.printf("# simulated disturbance: %s, simulated shocks: %s\n",
                  SIMULATE_DISTURBANCE ? "ON (Wokwi)" : "OFF",
                  SIMULATE_SHOCK ? "ON (1, 2, 3 samples at 3, 6, 9 s)" : "OFF");
    Serial.println("t_ms,dt_us,ax_raw,ay_raw,az_raw,ok,ax_f,ay_f,az_f,a_raw,a_f,dev,dev_pk,shock,tamper,mot");

    // Единственная задача сбора данных (приоритет 2, ядро 1)
    xTaskCreatePinnedToCore(acquisitionTask, "acq", 4096, nullptr, 2, nullptr, 1);
}

void loop()
{
    // Вся работа — в задаче FreeRTOS, loop() не нужен
    vTaskDelete(nullptr);
}

// =====================================================================
//  5. ЗАДАЧА СБОРА ДАННЫХ (FreeRTOS)
// =====================================================================
void acquisitionTask(void *arg)
{
    (void)arg;

    MedianFilter medX, medY, medZ;
    MovingAverage avgX, avgY, avgZ;
    medianInit(medX);
    medianInit(medY);
    medianInit(medZ);
    avgInit(avgX);
    avgInit(avgY);
    avgInit(avgZ);

    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t prevUs = micros();
    uint32_t n = 0;      // номер выведенной строки
    uint32_t sample = 0; // номер отсчёта с момента запуска

    // вектор покоя (м/с^2), накапливается первые CALIB_SAMPLES отсчётов
    float refX = 0, refY = 0, refZ = 0;
    uint32_t calib = 0;

    AccelRaw last = {0, 0, (int32_t)ACCEL_LSB_PER_G}; // последний достоверный отсчёт
    uint32_t lastShockMs = 0;
    bool shockSeen = false;

    for (;;)
    {
        // Ждём до lastWake + period: период дискретизации постоянный и
        // не зависит от времени выполнения тела цикла (в отличие от vTaskDelay)
        vTaskDelayUntil(&lastWake, period);

        uint32_t nowUs = micros();
        int32_t dtUs = (int32_t)(nowUs - prevUs); // фактический период (джиттер)
        prevUs = nowUs;
        uint32_t tMs = millis();

        // ---- 0. Сбор данных ----
        AccelRaw r;
        if (!readRaw(r))
        {
            Serial.println("# I2C read error");
            continue;
        }
        r.x = simulateDisturbance(r.x, tMs, 0.0f); // только для Wokwi
        r.y = simulateDisturbance(r.y, tMs, 2.1f);
        r.z = simulateDisturbance(r.z, tMs, 4.2f);
        int32_t shockRaw = simulateShock(sample++);
        r.x += shockRaw;
        if (shockRaw == 0)
            simulateGlitch(r); // сбой чтения шины (не во время удара)

        // аппаратный детектор движения: флаг MOT_INT (сбрасывается чтением)
        uint8_t intStatus = 0;
#if MOTION_DETECT
        mpuReadReg(REG_INT_STATUS, intStatus);
#endif
        int mot = (intStatus & 0x40) ? 1 : 0;

        // ---- 1. Проверка достоверности ----
        int ok = checkSample(r); // 1 — норма, 2 — насыщение, 0 — сбой
        AccelRaw v = ok ? r : last;
        if (ok)
            last = r;

        // ---- 2. Медианный фильтр (медленный канал) ----
        int32_t mx = medianFilter(medX, v.x);
        int32_t my = medianFilter(medY, v.y);
        int32_t mz = medianFilter(medZ, v.z);

        // ---- 3. Усреднение (медленный канал) ----
        float fx = movingAverage(avgX, mx);
        float fy = movingAverage(avgY, my);
        float fz = movingAverage(avgZ, mz);

        // ---- 4. Преобразование в физическую величину (м/с^2) ----
        float aRaw = magnitude(rawToMs2(r.x), rawToMs2(r.y), rawToMs2(r.z)); // без обработки
        float aF = magnitude(rawToMs2(fx), rawToMs2(fy), rawToMs2(fz));      // после обработки
        float ax = rawToMs2(fx), ay = rawToMs2(fy), az = rawToMs2(fz);

        // калибровка положения покоя (после прогрева фильтров)
        if (calib < CALIB_SAMPLES)
        {
            if (calib >= AVG_WINDOW)
            { // первые M отсчётов — прогрев окна
                refX += ax;
                refY += ay;
                refZ += az;
            }
            if (++calib == CALIB_SAMPLES)
            {
                float k = 1.0f / (CALIB_SAMPLES - AVG_WINDOW);
                refX *= k;
                refY *= k;
                refZ *= k;
                Serial.printf("# calibrated rest vector: %.2f %.2f %.2f m/s^2 (|a| = %.2f g)\n",
                              refX, refY, refZ, magnitude(refX, refY, refZ) / G_MS2);
            }
            continue; // во время калибровки не выводим
        }

        // медленный канал: отклонение сглаженного вектора от покоя
        float dev = magnitude(ax - refX, ay - refY, az - refZ);
        bool slow = dev >= TAMPER_THRESHOLD_MS2;

        // канал удара: отличие достоверного отсчёта (без фильтров)
        // от сглаженного сигнала — быстрая составляющая
        float devPk = magnitude(rawToMs2(v.x) - ax, rawToMs2(v.y) - ay, rawToMs2(v.z) - az);
        bool shock = ok && (devPk >= SHOCK_THRESHOLD_MS2);
        if (shock || mot)
        {
            lastShockMs = tMs;
            shockSeen = true;
        }
        bool shockHold = shockSeen && (tMs - lastShockMs < SHOCK_HOLD_MS);

        bool tamper = slow || shockHold;
        digitalWrite(LED_PIN, tamper ? HIGH : LOW);

        // ---- 5. Вывод: исходные и обработанные значения + timestamp ----
        if (n++ % PRINT_EVERY == 0)
        {
            Serial.printf("%lu,%ld,%ld,%ld,%ld,%d,%.0f,%.0f,%.0f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d\n",
                          (unsigned long)tMs, (long)dtUs,
                          (long)r.x, (long)r.y, (long)r.z, ok,
                          fx, fy, fz, aRaw, aF, dev, devPk,
                          shock ? 1 : 0, tamper ? 1 : 0, mot);
        }
    }
}

// =====================================================================
//  6. ДРАЙВЕР ДАТЧИКА (чтение по I2C)
// =====================================================================
bool mpuWriteReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool mpuReadReg(uint8_t reg, uint8_t &val)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) != 1)
        return false;
    val = Wire.read();
    return true;
}

bool mpuInit()
{
    uint8_t who = 0;
    if (!mpuReadReg(REG_WHO_AM_I, who)) // проверка идентификатора
        return false;
    Serial.printf("# WHO_AM_I = 0x%02X\n", who);

    if (!mpuWriteReg(REG_PWR_MGMT_1, 0x00))
        return false; // выйти из режима сна
    if (!mpuWriteReg(REG_ACCEL_CONFIG, ACCEL_CONFIG_VAL))
        return false; // AFS_SEL=2 -> ±8 g
#if MOTION_DETECT
    mpuWriteReg(REG_MOT_THR, MOT_THR_VAL);
    mpuWriteReg(REG_MOT_DUR, MOT_DUR_VAL);
    mpuWriteReg(REG_INT_ENABLE, 0x40); // MOT_EN: прерывание по движению
#endif
    delay(10);
    return true;
}

// Чтение "сырых" данных: 6 байт (X_H, X_L, Y_H, Y_L, Z_H, Z_L) ->
// три 16-битных знаковых числа. Никакой обработки здесь нет.
bool readRaw(AccelRaw &out)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6) != 6)
        return false;

    out.x = (int16_t)((Wire.read() << 8) | Wire.read());
    out.y = (int16_t)((Wire.read() << 8) | Wire.read());
    out.z = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
}

// ---- Имитация для симулятора ----
// Белый шум + медленный дрейф. Результат приводится к int16, как у датчика.
int32_t simulateDisturbance(int32_t v, uint32_t t_ms, float phase)
{
#if SIMULATE_DISTURBANCE
    const int32_t noiseLsb = (int32_t)(SIM_NOISE_G * ACCEL_LSB_PER_G);
    int32_t noise = 0;
    for (int i = 0; i < 4; ++i)
        noise += random(-noiseLsb, noiseLsb + 1);
    noise /= 2; // ~ нормальное распределение

    float drift = SIM_DRIFT_G * ACCEL_LSB_PER_G *
                  sinf(2.0f * (float)M_PI * t_ms / (SIM_DRIFT_PERIOD_S * 1000.0f) + phase);

    v += noise + (int32_t)drift;
    if (v > SAT_RAW)
        v = SAT_RAW;
    if (v < -SAT_RAW - 1)
        v = -SAT_RAW - 1;
    return v;
#else
    (void)t_ms;
    (void)phase;
    return v;
#endif
}

// Сбой чтения шины I2C: все 6 байт приходят как 0xFF -> X = Y = Z = -1.
bool simulateGlitch(AccelRaw &r)
{
#if SIMULATE_DISTURBANCE
    if (random(SIM_GLITCH_PROB) == 0)
    {
        r.x = r.y = r.z = -1;
        return true;
    }
#else
    (void)r;
#endif
    return false;
}

// Короткие удары по оси X: 1, 2 и 3 отсчёта на 3-й, 6-й и 9-й секунде.
int32_t simulateShock(uint32_t sample)
{
#if SIMULATE_SHOCK
    const uint32_t fs = 1000 / SAMPLE_PERIOD_MS;
    const int32_t amp = (int32_t)(SIM_SHOCK_G * ACCEL_LSB_PER_G);
    if (sample >= 3 * fs && sample < 3 * fs + 1)
        return amp;
    if (sample >= 6 * fs && sample < 6 * fs + 2)
        return amp;
    if (sample >= 9 * fs && sample < 9 * fs + 3)
        return amp;
#else
    (void)sample;
#endif
    return 0;
}

// =====================================================================
//  7. ОБРАБОТКА СИГНАЛА
// =====================================================================
// ---- 7.1 Проверка достоверности отсчёта ----
// 0 — сбой (|a| < VALID_MIN_G: физически невозможно для закреплённого
//     устройства), 2 — насыщение (хотя бы одна ось на краю шкалы:
//     отсчёт верен, но амплитуда обрезана), 1 — норма.
int checkSample(const AccelRaw &r)
{
    float aG = magnitude((float)r.x, (float)r.y, (float)r.z) / ACCEL_LSB_PER_G;
    if (aG < VALID_MIN_G)
        return 0;
    if (abs(r.x) >= SAT_RAW || abs(r.y) >= SAT_RAW || abs(r.z) >= SAT_RAW)
        return 2;
    return 1;
}

// ---- 7.2 Медианный фильтр со скользящим окном N ----
void medianInit(MedianFilter &f)
{
    f.idx = 0;
    f.count = 0;
    for (int i = 0; i < MEDIAN_WINDOW; ++i)
        f.buf[i] = 0;
}

int32_t medianFilter(MedianFilter &f, int32_t x)
{
    f.buf[f.idx] = x; // новый отсчёт вытесняет старейший
    f.idx = (f.idx + 1) % MEDIAN_WINDOW;
    if (f.count < MEDIAN_WINDOW)
        f.count++;

    int32_t tmp[MEDIAN_WINDOW]; // копия окна
    for (int i = 0; i < f.count; ++i)
        tmp[i] = f.buf[i];
    for (int i = 1; i < f.count; ++i)
    { // сортировка вставками
        int32_t key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key)
        {
            tmp[j + 1] = tmp[j];
            --j;
        }
        tmp[j + 1] = key;
    }
    return tmp[f.count / 2]; // центральный элемент
}

// ---- 7.3 Скользящее среднее с окном M ----
void avgInit(MovingAverage &f)
{
    f.sum = 0;
    f.idx = 0;
    f.count = 0;
    for (int i = 0; i < AVG_WINDOW; ++i)
        f.buf[i] = 0;
}

float movingAverage(MovingAverage &f, int32_t x)
{
    f.sum -= f.buf[f.idx]; // убрать старейший отсчёт
    f.buf[f.idx] = x;
    f.sum += x; // добавить новый
    f.idx = (f.idx + 1) % AVG_WINDOW;
    if (f.count < AVG_WINDOW)
        f.count++;
    return (float)f.sum / (float)f.count;
}

// ---- 7.4 Преобразование в физическую величину ----
// Передаточная характеристика (datasheet): a[g] = raw / 4096 при ±8 g
// a[м/с^2] = raw / 4096 * 9.80665
float rawToMs2(float raw)
{
    return raw / ACCEL_LSB_PER_G * G_MS2;
}

// Модуль вектора ускорения |a| = sqrt(ax^2 + ay^2 + az^2)
float magnitude(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}
