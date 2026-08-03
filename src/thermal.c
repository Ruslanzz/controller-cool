/**
  ******************************************************************************
  * @file    thermal.c
  * @brief   Реализация тепловой защиты платы по двум датчикам.
  *
  *          Датчики TEMP_SENS_1 (PA7, ADC CH7) и TEMP_SENS_2 (PB0, ADC CH8) —
  *          NTC 10 кОм B=3950 в нижнем плече делителя с подтяжкой 10 кОм на
  *          3.3 В:
  *              R(T) = 10000 · exp(3950 · (1/T − 1/298.15)),  T в кельвинах
  *              ADC  = 4095 · R / (R + 10000)
  *          При таком включении рост температуры УМЕНЬШАЕТ отсчёт — это и
  *          задаёт TEMP_SENSOR_NTC = 1 и направление сравнений (TEMP_HOTTER).
  ******************************************************************************
  */

#include "thermal.h"
#include "config.h"
#include "bsp.h"

/* ===== Действующие пороги (ОЗУ) =========================================== */
volatile uint16_t temp_trip_adc  = 0;
volatile uint16_t temp_clear_adc = 0;

/* Последняя ПРИНЯТАЯ пара — из неё восстанавливаются пороги, если кто-то
 * записал в переменные бессмыслицу напрямую, мимо проверок (см. Thermal_Update). */
static uint16_t trip_accepted  = 0;
static uint16_t clear_accepted = 0;
static uint16_t bad_writes     = 0;

/* ===== Состояние датчиков ================================================= */
static const uint8_t temp_idx[TEMP_SENSOR_COUNT] = {
  TEMP_SENS_1_IDX, TEMP_SENS_2_IDX
};

static uint16_t adc_raw[TEMP_SENSOR_COUNT];
static int16_t  temp_c[TEMP_SENSOR_COUNT];
static uint8_t  valid[TEMP_SENSOR_COUNT];
static uint8_t  over[TEMP_SENSOR_COUNT];
static uint32_t over_since[TEMP_SENSOR_COUNT];

static uint8_t  limit;
static uint8_t  no_sensor;

/* ===== Таблица NTC ========================================================
 * Отсчёты АЦП от -20 до 150 °C с шагом 5 °C — по формуле выше, округлено.
 * Сверено с точками, которые и раньше стояли в config.h узлов:
 * 25 °C -> 2048, 75 °C -> 532, 85 °C -> 401, 95 °C -> 305, 105 °C -> 234.
 *
 * Таблица описывает КОНКРЕТНЫЙ датчик (NTC B=3950 с подтяжкой 10 кОм). Если
 * датчики когда-нибудь заменят на другие, менять надо и таблицу, и
 * TEMP_SENSOR_NTC; пороги в отсчётах АЦП (Thermal_SetTripAdc) при этом
 * продолжают работать, а градусы теряют смысл.                             */
#define NTC_T_MIN_C   (-20)
#define NTC_T_STEP_C     5
#define NTC_POINTS      35

static const uint16_t ntc_adc[NTC_POINTS] = {
  3740, 3629, 3495, 3337, 3156, 2955, 2738, 2510, 2278, 2048,  /* -20..25 °C */
  1825, 1614, 1419, 1241, 1081,  940,  815,  707,  613,  532,  /*  30..70 °C */
   462,  401,  350,  305,  267,  234,  206,  181,  160,  142,  /*  75..115 °C */
   126,  112,  100,   89,   80                                 /* 120..150 °C */
};

#define NTC_T_MAX_C  (NTC_T_MIN_C + (NTC_POINTS - 1) * NTC_T_STEP_C)  /* 150 */

uint16_t Thermal_CToAdc(int16_t t_c)
{
  if (t_c <= NTC_T_MIN_C) {
    return ntc_adc[0];
  }
  if (t_c >= NTC_T_MAX_C) {
    return ntc_adc[NTC_POINTS - 1];
  }

  int32_t  off = (int32_t)t_c - NTC_T_MIN_C;
  uint8_t  i   = (uint8_t)(off / NTC_T_STEP_C);
  int32_t  rem = off % NTC_T_STEP_C;

  /* Таблица убывающая: интерполируем вниз от точки i. */
  int32_t hi = ntc_adc[i];
  int32_t lo = ntc_adc[i + 1];

  return (uint16_t)(hi - ((hi - lo) * rem) / NTC_T_STEP_C);
}

int16_t Thermal_AdcToC(uint16_t adc)
{
  if (adc >= ntc_adc[0]) {
    return NTC_T_MIN_C;
  }
  if (adc <= ntc_adc[NTC_POINTS - 1]) {
    return NTC_T_MAX_C;
  }

  for (uint8_t i = 0; i < (NTC_POINTS - 1); i++) {
    int32_t hi = ntc_adc[i];
    int32_t lo = ntc_adc[i + 1];

    if (adc <= hi && adc > lo) {
      int32_t t = (int32_t)NTC_T_MIN_C + (int32_t)i * NTC_T_STEP_C;
      return (int16_t)(t + ((hi - (int32_t)adc) * NTC_T_STEP_C) / (hi - lo));
    }
  }
  return NTC_T_MAX_C;
}

/* ===== Пороги ============================================================= */

/* Пара осмысленна, если порог срабатывания ГОРЯЧЕЕ порога возврата и между
 * ними есть гистерезис не меньше TEMP_HYST_MIN_C. Проверка ведётся в
 * градусах: в отсчётах АЦП «горячее» — это «меньше», и на этом легко
 * ошибиться.                                                                */
static uint8_t Thermal_PairSane(uint16_t trip, uint16_t clear)
{
  if (trip == 0 || clear == 0) {
    return 0;
  }
  if (!TEMP_HOTTER(trip, clear)) {
    return 0;                       /* возврат не холоднее срабатывания */
  }
  return (uint8_t)((Thermal_AdcToC(trip) - Thermal_AdcToC(clear)) >= TEMP_HYST_MIN_C);
}

uint8_t Thermal_SetTripAdc(uint16_t trip, uint16_t clear)
{
  if (!Thermal_PairSane(trip, clear)) {
    return 0;
  }

  temp_trip_adc  = trip;
  temp_clear_adc = clear;
  trip_accepted  = trip;
  clear_accepted = clear;
  return 1;
}

uint8_t Thermal_SetTripC(int16_t trip_c, int16_t clear_c)
{
  if (trip_c < NTC_T_MIN_C || trip_c > NTC_T_MAX_C ||
      clear_c < NTC_T_MIN_C || clear_c > NTC_T_MAX_C) {
    return 0;
  }
  if ((trip_c - clear_c) < TEMP_HYST_MIN_C) {
    return 0;
  }
  return Thermal_SetTripAdc(Thermal_CToAdc(trip_c), Thermal_CToAdc(clear_c));
}

/* ===== Измерение ========================================================== */

void Thermal_Init(void)
{
  temp_trip_adc  = Thermal_CToAdc(TEMP_TRIP_C);
  temp_clear_adc = Thermal_CToAdc(TEMP_CLEAR_C);
  trip_accepted  = temp_trip_adc;
  clear_accepted = temp_clear_adc;
  bad_writes     = 0;

  for (uint8_t s = 0; s < TEMP_SENSOR_COUNT; s++) {
    adc_raw[s]    = 0;
    temp_c[s]     = TEMP_INVALID_C;
    valid[s]      = 0;
    over[s]       = 0;
    over_since[s] = 0;
  }

  limit     = 0;
  no_sensor = 1;
}

void Thermal_Update(void)
{
  static uint32_t sample_tick = 0;

  uint32_t now = HAL_GetTick();

  if ((now - sample_tick) < TEMP_SAMPLE_MS) {
    return;
  }
  sample_tick = now;

  /* Пороги могли быть записаны напрямую, мимо проверок. Испорченную пару
   * (возврат не холоднее срабатывания) восстанавливаем из последней принятой:
   * иначе защита либо залипнет включённой, либо задребезжит на границе.     */
  if (!Thermal_PairSane(temp_trip_adc, temp_clear_adc)) {
    temp_trip_adc  = trip_accepted;
    temp_clear_adc = clear_accepted;
    if (bad_writes < 0xFFFF) {
      bad_writes++;
    }
  }

  uint8_t any_over  = 0;
  uint8_t any_valid = 0;

  for (uint8_t s = 0; s < TEMP_SENSOR_COUNT; s++) {
    uint16_t adc = (uint16_t)ADS_RES_BUFFER[temp_idx[s]];

    adc_raw[s] = adc;

    /* Оборванный (отсчёт у потолка) или закороченный (у нуля) датчик из
     * решения исключается — но защиту не отменяет: второй продолжает
     * защищать. Ограничение по неизвестной температуре не вводится.         */
    if (adc < TEMP_VALID_MIN_ADC || adc > TEMP_VALID_MAX_ADC) {
      valid[s]      = 0;
      temp_c[s]     = TEMP_INVALID_C;
      over[s]       = 0;
      over_since[s] = 0;
      continue;
    }

    valid[s]  = 1;
    any_valid = 1;
    temp_c[s] = Thermal_AdcToC(adc);

    if (TEMP_HOTTER(adc, temp_trip_adc)) {
      /* Выше порога — с выдержкой: одиночная выборка ничего не включает. */
      if (over_since[s] == 0) {
        over_since[s] = (now != 0) ? now : 1;
      } else if ((now - over_since[s]) >= TEMP_TRIP_MS) {
        over[s] = 1;
      }
    } else if (!TEMP_HOTTER(adc, temp_clear_adc)) {
      /* Ниже порога возврата — перегрев снят. */
      over[s]       = 0;
      over_since[s] = 0;
    } else {
      /* Между порогами: держим текущее состояние (гистерезис). */
      over_since[s] = 0;
    }

    if (over[s]) {
      any_over = 1;
    }
  }

  no_sensor = (uint8_t)(!any_valid);
  limit     = any_over;
}

uint8_t Thermal_Limit(void)
{
  return limit;
}

void Thermal_GetStatus(ThermalStatus *out)
{
  if (out == 0) {
    return;
  }

  for (uint8_t s = 0; s < TEMP_SENSOR_COUNT; s++) {
    out->temp_c[s] = temp_c[s];
    out->adc[s]    = adc_raw[s];
    out->valid[s]  = valid[s];
    out->over[s]   = over[s];
  }

  out->limit      = limit;
  out->no_sensor  = no_sensor;
  out->trip_adc   = temp_trip_adc;
  out->clear_adc  = temp_clear_adc;
  out->trip_c     = Thermal_AdcToC(temp_trip_adc);
  out->clear_c    = Thermal_AdcToC(temp_clear_adc);
  out->bad_writes = bad_writes;
}
