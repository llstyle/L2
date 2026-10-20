// =====================================================================
//  L2. Информационные технологии в микроконтроллере
//  Сбор сигнала и его программная обработка
//
//  Use case (из L1): домашнее Wi-Fi устройство обнаружения
//  вмешательства / удара / вибрации.
//  Датчик: MPU6050 (3-осевой акселерометр, цифровой интерфейс I2C).
//  Платформа: ESP32, Arduino framework + FreeRTOS. Симуляция: Wokwi.
//
//  Цепочка обработки (для каждой оси X, Y, Z):
//    Датчик -> Сбор (I2C) -> Ограничение -> Медианный фильтр ->
//    Усреднение -> Преобразование (м/с^2) -> Физическая величина
//  Решение use case: при старте запоминается вектор ускорения в покое
//  (калибровка). Отклонение текущего вектора от него
//  dev = |a_f - a_ref| > порога -> устройство сдвинули / наклонили /
//  ударили: вмешательство (горит красный LED).
//
//  Вывод в STDIO (Serial), формат CSV:
//    t_ms,dt_us,ax_raw,ay_raw,az_raw,ax_f,ay_f,az_f,a_raw,a_f,dev,tamper
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
#define LED_PIN 2          // индикатор "удар / вмешательство"

// ---- MPU6050 (регистры из Register Map, rev. 4.2) ----
#define MPU_ADDR 0x68         // AD0 = GND
#define REG_PWR_MGMT_1 0x6B   // управление питанием (выход из сна)
#define REG_ACCEL_CONFIG 0x1C // выбор диапазона акселерометра
#define REG_ACCEL_XOUT_H 0x3B // начало 6 байт данных X,Y,Z
#define REG_WHO_AM_I 0x75     // идентификатор, должен быть 0x68

// Диапазон акселерометра AFS_SEL = 0 -> ±2 g, чувствительность 16384 LSB/g
// (datasheet MPU-6000/6050, табл. "Accelerometer Specifications")
#define ACCEL_FS_G 2.0f
#define ACCEL_LSB_PER_G 16384.0f
#define G_MS2 9.80665f // стандартное ускорение свободного падения

// ---- Период дискретизации (vTaskDelayUntil) ----
#define SAMPLE_PERIOD_MS 20 // 20 мс -> fs = 50 Гц

// ---- Ограничение (saturation) ----
// Границы = полная шкала выбранного диапазона из datasheet: ±2 g.
// Всё, что вне шкалы, датчик измерить не может -> значение недостоверно.
#define CLAMP_MIN_RAW ((int32_t)(-ACCEL_FS_G * ACCEL_LSB_PER_G))    // -32768
#define CLAMP_MAX_RAW ((int32_t)(ACCEL_FS_G * ACCEL_LSB_PER_G) - 1) // +32767

// ---- Окна фильтров ----
#define MEDIAN_WINDOW 5 // N — нечётное (3, 5, 7, 9 ...)
#define AVG_WINDOW 32   // M — окно скользящего среднего

// ---- Решение use case ----
#define CALIB_SAMPLES 50          // отсчётов на калибровку покоя (1 с)
#define TAMPER_THRESHOLD_MS2 2.0f // порог отклонения |a_f - a_ref|, м/с^2 (~0.2 g)

// ---- Вывод ----
#define PRINT_EVERY 1 // печатать каждый N-й отсчёт

// ---- Имитация помех (ТОЛЬКО для Wokwi) ----
// В Wokwi датчик выдаёт идеально чистые значения, заданные ползунками.
// Чтобы было что фильтровать, к ним добавляется искусственный шум,
// редкие выбросы и медленный дрейф. На реальной плате поставить 0!
#define SIMULATE_DISTURBANCE 1
#define SIM_NOISE_LSB 400     // амплитуда шума (~0.025 g)
#define SIM_SPIKE_PROB 40     // вероятность выброса = 1 / SIM_SPIKE_PROB
#define SIM_DRIFT_LSB 600     // амплитуда дрейфа (~0.04 g)
#define SIM_DRIFT_PERIOD_S 30 // период дрейфа, с

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
bool mpuInit();
bool readRaw(AccelRaw &out);
int32_t simulateDisturbance(int32_t v, uint32_t t_ms, float phase);
int32_t clampValue(int32_t x, int32_t lo, int32_t hi);
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
    Serial.println("# L2: MPU6050 vibration/tamper signal acquisition");
    Serial.printf("# fs=%d Hz (T=%d ms), median N=%d, avg M=%d, range=±%.0f g, clamp=[%ld..%ld] LSB\n",
                  1000 / SAMPLE_PERIOD_MS, SAMPLE_PERIOD_MS, MEDIAN_WINDOW, AVG_WINDOW,
                  ACCEL_FS_G, (long)CLAMP_MIN_RAW, (long)CLAMP_MAX_RAW);
    Serial.printf("# simulated disturbance: %s\n", SIMULATE_DISTURBANCE ? "ON (Wokwi)" : "OFF");
    Serial.println("t_ms,dt_us,ax_raw,ay_raw,az_raw,ax_f,ay_f,az_f,a_raw,a_f,dev,tamper");

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
    uint32_t n = 0;

    // вектор покоя (м/с^2), накапливается первые CALIB_SAMPLES отсчётов
    float refX = 0, refY = 0, refZ = 0;
    uint32_t calib = 0;

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

        // ---- 1. Ограничение ----
        int32_t cx = clampValue(r.x, CLAMP_MIN_RAW, CLAMP_MAX_RAW);
        int32_t cy = clampValue(r.y, CLAMP_MIN_RAW, CLAMP_MAX_RAW);
        int32_t cz = clampValue(r.z, CLAMP_MIN_RAW, CLAMP_MAX_RAW);

        // ---- 2. Медианный фильтр ----
        int32_t mx = medianFilter(medX, cx);
        int32_t my = medianFilter(medY, cy);
        int32_t mz = medianFilter(medZ, cz);

        // ---- 3. Усреднение ----
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
                Serial.printf("# calibrated rest vector: %.2f %.2f %.2f m/s^2\n", refX, refY, refZ);
            }
            continue; // во время калибровки не выводим
        }

        // отклонение от положения покоя, м/с^2
        float dev = magnitude(ax - refX, ay - refY, az - refZ);

        bool tamper = dev >= TAMPER_THRESHOLD_MS2;
        digitalWrite(LED_PIN, tamper ? HIGH : LOW);

        // ---- 5. Вывод: исходные и обработанные значения + timestamp ----
        if (n++ % PRINT_EVERY == 0)
        {
            Serial.printf("%lu,%ld,%ld,%ld,%ld,%.0f,%.0f,%.0f,%.3f,%.3f,%.3f,%d\n",
                          (unsigned long)tMs, (long)dtUs,
                          (long)r.x, (long)r.y, (long)r.z,
                          fx, fy, fz, aRaw, aF, dev, tamper ? 1 : 0);
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

bool mpuInit()
{
    // проверка идентификатора
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_WHO_AM_I);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) != 1)
        return false;
    uint8_t who = Wire.read();
    Serial.printf("# WHO_AM_I = 0x%02X\n", who);

    if (!mpuWriteReg(REG_PWR_MGMT_1, 0x00))
        return false; // выйти из режима сна
    if (!mpuWriteReg(REG_ACCEL_CONFIG, 0x00))
        return false; // AFS_SEL=0 -> ±2 g
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

// Имитация помех для симулятора: белый шум + медленный дрейф +
// редкие одиночные выбросы до края шкалы. Результат не ограничивается —
// это задача следующего этапа.
int32_t simulateDisturbance(int32_t v, uint32_t t_ms, float phase)
{
#if SIMULATE_DISTURBANCE
    int32_t noise = 0;
    for (int i = 0; i < 4; ++i)
        noise += random(-SIM_NOISE_LSB, SIM_NOISE_LSB + 1);
    noise /= 2; // ~ нормальное распределение

    float drift = SIM_DRIFT_LSB *
                  sinf(2.0f * (float)M_PI * t_ms / (SIM_DRIFT_PERIOD_S * 1000.0f) + phase);

    v += noise + (int32_t)drift;

    if (random(SIM_SPIKE_PROB) == 0)
    {                                   // импульсная помеха
        v = random(2) ? 40000 : -40000; // за пределами шкалы ±2 g
    }
    return v;
#else
    (void)t_ms;
    (void)phase;
    return v;
#endif
}

// =====================================================================
//  7. ОБРАБОТКА СИГНАЛА
// =====================================================================
// ---- 7.1 Ограничение: привести значение в диапазон [lo, hi] ----
int32_t clampValue(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
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
// Передаточная характеристика (datasheet): a[g] = raw / 16384 при ±2 g
// a[м/с^2] = raw / 16384 * 9.80665
float rawToMs2(float raw)
{
    return raw / ACCEL_LSB_PER_G * G_MS2;
}

// Модуль вектора ускорения |a| = sqrt(ax^2 + ay^2 + az^2)
float magnitude(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}
