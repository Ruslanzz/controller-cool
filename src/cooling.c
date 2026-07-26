/**
  ******************************************************************************
  * @file    cooling.c
  * @brief   Реализация управления вентиляторами радиатора (L1 = DRV1, L2 = DRV2).
  *
  *          Полярность моста — как у балки на controller-front и у катушек на
  *          ведущем: IN_A (регулируемое плечо) со сравнением PWM_PERIOD даёт
  *          нулевой ток, 0 — полный; IN_B держится открытым (PWM_PERIOD).
  *
  *          Из этого следует ключевое для индуктивной нагрузки свойство: в
  *          паузе ШИМ оба плеча подняты и обмотка замкнута сама на себя через
  *          верхние ключи. Ток якоря циркулирует по этому контуру, а не рвётся.
  *          Поэтому «скважность 0 при поднятом EN» — не выключение, а режим
  *          выбега, в котором индуктивность стекает без выброса напряжения.
  *          Мост обесточивается снятием EN только после FAN_DECAY_MS в этом
  *          режиме.
  ******************************************************************************
  */

#include "cooling.h"
#include "config.h"
#include "bsp.h"

/* Скважность хранится в сотых долях процента: шаг разгона получается плавным
 * без накопления ошибки целочисленного деления.                             */
#define DUTY_SCALE      100u
#define DUTY_MAX_Q      (100u * DUTY_SCALE)
#define DUTY_TARGET_Q   ((uint32_t)FAN_DUTY_TARGET * DUTY_SCALE)

/* ===== Неизменная привязка канала к железу ================================ */
typedef struct {
  uint32_t      ch_a;      /* канал TIM4: регулируемое плечо IN_A            */
  uint32_t      ch_b;      /* канал TIM4: постоянно открытое плечо IN_B      */
  GPIO_TypeDef *en_port;   /* порт линий разрешения драйвера                 */
  uint16_t      en_a_pin;
  uint16_t      en_b_pin;
  uint8_t       adc_idx;   /* индекс датчика тока в ADS_RES_BUFFER           */
} FanHw;

static const FanHw fan_hw[FAN_COUNT] = {
  /* L1 = DRV1: TIM4 CH1/CH2, EN на порту A, ток на ADC CH5 (PA5). */
  { TIM_CHANNEL_1, TIM_CHANNEL_2, GPIOA, DRV1_EN_A_Pin, DRV1_EN_B_Pin, DRV1_CURRENT_IDX },
  /* L2 = DRV2: TIM4 CH3/CH4, EN на порту B, ток на ADC CH6 (PA6). */
  { TIM_CHANNEL_3, TIM_CHANNEL_4, GPIOB, DRV2_EN_A_Pin, DRV2_EN_B_Pin, DRV2_CURRENT_IDX }
};

/* ===== Изменяемое состояние канала ======================================== */
typedef struct {
  FanState state;
  FanFault fault;
  uint8_t  enabled;      /* EN драйвера поднят                              */

  uint32_t duty_q;       /* текущая скважность, сотые доли процента         */
  uint32_t ramp_from_q;  /* скважность на начало текущей рампы              */
  uint32_t ramp_to_q;    /* цель текущей рампы                              */
  uint32_t ramp_ms;      /* длительность текущей рампы                      */
  uint32_t ramp_tick;    /* отметка начала рампы                            */

  uint32_t state_tick;   /* отметка входа в текущее состояние               */

  uint32_t zero_adc;     /* ноль датчика тока (измеренный или номинальный)  */
  uint8_t  zero_valid;
  uint32_t i_acc;        /* аккумулятор ЭФ среднего тока (масштаб 2^SHIFT)  */
  uint32_t oc_peak_acc;  /* «дырявое ведро» пиковой защиты                  */
  uint32_t ovl_since;    /* начало непрерывной перегрузки, 0 — нет          */
  uint32_t open_since;   /* начало отсутствия тока под нагрузкой, 0 — нет   */

  uint8_t  retries;      /* повторных пусков после аварии подряд            */
  uint16_t trips;        /* всего срабатываний защиты                       */
} FanRt;

static FanRt fan[FAN_COUNT];

/* Команда ведущего и момент её последнего фронта включения — по нему
 * отсчитывается разнос пусков двух вентиляторов.                            */
static volatile uint8_t  fan_command   = 0;
static volatile uint32_t fan_cmd_tick  = 0;

/* ==========================================================================
 * Низкий уровень: мост и разрешение драйвера.
 * ========================================================================== */

/* Выставить скважность на регулируемом плече. Плечо IN_B держится открытым,
 * поэтому duty_q = 0 -> оба плеча подняты -> контур циркуляции замкнут.     */
static void Fan_ApplyDuty(uint8_t i)
{
  uint32_t on = ((uint32_t)PWM_PERIOD * fan[i].duty_q) / DUTY_MAX_Q;

  __HAL_TIM_SET_COMPARE(&htim4, fan_hw[i].ch_b, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, fan_hw[i].ch_a, PWM_PERIOD - on);
}

static void Fan_EnableDriver(uint8_t i)
{
  HAL_GPIO_WritePin(fan_hw[i].en_port, fan_hw[i].en_a_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(fan_hw[i].en_port, fan_hw[i].en_b_pin, GPIO_PIN_SET);
  fan[i].enabled = 1;
}

static void Fan_DisableDriver(uint8_t i)
{
  HAL_GPIO_WritePin(fan_hw[i].en_port, fan_hw[i].en_a_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(fan_hw[i].en_port, fan_hw[i].en_b_pin, GPIO_PIN_RESET);
  fan[i].enabled = 0;
}

/* ==========================================================================
 * Рампы скважности.
 * ========================================================================== */

/* Взвести рампу к цели @p to_q. Длительность масштабируется по остатку пути,
 * поэтому крутизна не зависит от того, с какой скважности рампа начата
 * (например, при повторном пуске прямо в фазе выбега).                      */
static void Fan_StartRamp(uint8_t i, uint32_t to_q, uint32_t full_ms, uint32_t now)
{
  uint32_t from_q = fan[i].duty_q;
  uint32_t span   = (to_q > from_q) ? (to_q - from_q) : (from_q - to_q);

  fan[i].ramp_from_q = from_q;
  fan[i].ramp_to_q   = to_q;
  fan[i].ramp_tick   = now;
  fan[i].ramp_ms     = (DUTY_TARGET_Q == 0) ? 0 : (full_ms * span) / DUTY_TARGET_Q;
}

/* Пересчитать скважность по времени от начала рампы. Возвращает 1, когда
 * цель достигнута.                                                          */
static uint8_t Fan_StepRamp(uint8_t i, uint32_t now)
{
  uint32_t elapsed = now - fan[i].ramp_tick;

  if (fan[i].ramp_ms == 0 || elapsed >= fan[i].ramp_ms) {
    fan[i].duty_q = fan[i].ramp_to_q;
    Fan_ApplyDuty(i);
    return 1;
  }

  if (fan[i].ramp_to_q > fan[i].ramp_from_q) {
    uint32_t span = fan[i].ramp_to_q - fan[i].ramp_from_q;
    fan[i].duty_q = fan[i].ramp_from_q + (span * elapsed) / fan[i].ramp_ms;
  } else {
    uint32_t span = fan[i].ramp_from_q - fan[i].ramp_to_q;
    fan[i].duty_q = fan[i].ramp_from_q - (span * elapsed) / fan[i].ramp_ms;
  }

  Fan_ApplyDuty(i);
  return 0;
}

/* ==========================================================================
 * Переходы состояний.
 * ========================================================================== */

static void Fan_ResetProtection(uint8_t i)
{
  fan[i].i_acc       = 0;
  fan[i].oc_peak_acc = 0;
  fan[i].ovl_since   = 0;
  fan[i].open_since  = 0;
}

/* Средний ток канала в отсчётах АЦП (модуль отклонения от нуля датчика). */
static uint32_t Fan_AvgCurrent(uint8_t i)
{
  return fan[i].i_acc >> FAN_I_AVG_SHIFT;
}

static void Fan_EnterState(uint8_t i, FanState state, uint32_t now)
{
  fan[i].state      = state;
  fan[i].state_tick = now;
}

/* Начать управляемый останов: плавный сброс скважности до нуля. Дальше
 * автомат сам выдержит стекание индукции и снимет EN.                       */
static void Fan_BeginStop(uint8_t i, uint32_t now)
{
  Fan_StartRamp(i, 0, FAN_RAMP_DOWN_MS, now);
  Fan_EnterState(i, FAN_STATE_STOPPING, now);
}

/* Начать разгон к рабочей скважности (мост уже под разрешением). */
static void Fan_BeginStart(uint8_t i, uint32_t now)
{
  Fan_StartRamp(i, DUTY_TARGET_Q, FAN_RAMP_UP_MS, now);
  Fan_EnterState(i, FAN_STATE_STARTING, now);
}

/* Аварийное отключение канала.
 *
 * Скважность обнуляется НЕМЕДЛЕННО, но EN остаётся поднятым: мост переходит в
 * ту же циркуляцию, что и в любой паузе ШИМ, и ток якоря стекает по обмотке.
 * Снятие EN выполняет автомат по истечении FAN_DECAY_MS. Рвать контур тока
 * прямо в момент аварии нельзя — выброс индуктивности пришёлся бы на ключи,
 * которые и так работают на пределе.                                        */
static void Fan_Trip(uint8_t i, FanFault fault, uint32_t now)
{
  fan[i].duty_q = 0;
  Fan_ApplyDuty(i);

  fan[i].fault = fault;
  if (fan[i].trips < 0xFFFF) {
    fan[i].trips++;
  }
  Fan_ResetProtection(i);
  Fan_EnterState(i, FAN_STATE_FAULT, now);
}

/* ==========================================================================
 * Инициализация.
 * ========================================================================== */

void Cooling_Init(void)
{
  for (uint8_t i = 0; i < FAN_COUNT; i++) {
    fan[i].state      = FAN_STATE_OFF;
    fan[i].fault      = FAN_FAULT_NONE;
    fan[i].duty_q     = 0;
    fan[i].zero_adc   = ACS724_ZERO_ADC;
    fan[i].zero_valid = 0;
    fan[i].retries    = 0;
    fan[i].trips      = 0;
    fan[i].state_tick = HAL_GetTick();
    Fan_ResetProtection(i);

    /* Сначала безопасное положение плеч, только потом (в автомате) EN. */
    Fan_ApplyDuty(i);
    Fan_DisableDriver(i);
  }
}

void Cooling_CalibrateCurrentSensors(void)
{
  uint32_t sum[FAN_COUNT] = {0};

  /* АЦП уже крутится по DMA; дать буферу заполниться и усреднить. */
  HAL_Delay(50);
  for (uint8_t s = 0; s < ACS_CAL_SAMPLES; s++) {
    for (uint8_t i = 0; i < FAN_COUNT; i++) {
      sum[i] += ADS_RES_BUFFER[fan_hw[i].adc_idx];
    }
    HAL_Delay(2);
  }

  for (uint8_t i = 0; i < FAN_COUNT; i++) {
    uint32_t zero = sum[i] / ACS_CAL_SAMPLES;

    if (zero >= ACS_CAL_MIN_ADC && zero <= ACS_CAL_MAX_ADC) {
      fan[i].zero_adc   = zero;
      fan[i].zero_valid = 1;
    } else {
      /* Датчик неисправен или не подключён. Защиту не отключаем — вентилятор
       * без защиты по току не оставляем; берём номинальный ноль и поднимаем
       * признак в телеметрию.                                              */
      fan[i].zero_adc   = ACS724_ZERO_ADC;
      fan[i].zero_valid = 0;
    }
  }
}

/* ==========================================================================
 * Команда ведущего.
 * ========================================================================== */

void Cooling_SetCommand(uint8_t on)
{
  uint8_t cmd = (on != 0);

  if (cmd && !fan_command) {
    fan_cmd_tick = HAL_GetTick();   /* фронт включения — отсчёт разноса пусков */
  }
  fan_command = cmd;
}

/* Команда конкретному каналу. Второй вентилятор стартует на FAN_STAGGER_MS
 * позже первого: пусковые токи двух двигателей не складываются на общем
 * питании. На выключение разнос не действует — останов синхронный.          */
static uint8_t Fan_Command(uint8_t i, uint32_t now)
{
  if (!fan_command) {
    return 0;
  }
  if (i == 0) {
    return 1;
  }
  return ((now - fan_cmd_tick) >= FAN_STAGGER_MS) ? 1 : 0;
}

/* ==========================================================================
 * Автомат канала.
 * ========================================================================== */

static void Fan_Step(uint8_t i, uint32_t now)
{
  uint8_t cmd = Fan_Command(i, now);

  switch (fan[i].state) {

    case FAN_STATE_OFF:
      /* Мост обесточен. Пуск — только после паузы, за которую драйвер
       * гарантированно разрядился.                                         */
      if (cmd && (now - fan[i].state_tick) >= FAN_RESTART_DELAY_MS) {
        fan[i].duty_q = 0;
        Fan_ApplyDuty(i);        /* безопасное положение плеч до подачи EN  */
        Fan_EnableDriver(i);
        Fan_ResetProtection(i);
        Fan_BeginStart(i, now);
      }
      break;

    case FAN_STATE_STARTING:
      if (!cmd) {
        Fan_BeginStop(i, now);
        break;
      }
      if (Fan_StepRamp(i, now)) {
        Fan_EnterState(i, FAN_STATE_RUNNING, now);
        /* Разгон закончился штатно — счётчик попыток обнуляем: следующая
         * авария снова получит полный лимит повторных пусков.              */
        fan[i].retries = 0;
      }
      break;

    case FAN_STATE_RUNNING:
      if (!cmd) {
        Fan_BeginStop(i, now);
        break;
      }
      fan[i].duty_q = DUTY_TARGET_Q;
      Fan_ApplyDuty(i);
      break;

    case FAN_STATE_STOPPING:
      /* Команду вернули посреди выбега — плавно уходим обратно в разгон,
       * EN снимать не начинали.                                            */
      if (cmd) {
        Fan_BeginStart(i, now);
        break;
      }
      if (Fan_StepRamp(i, now)) {
        Fan_EnterState(i, FAN_STATE_DECAY, now);
      }
      break;

    case FAN_STATE_DECAY:
      /* Скважность 0, EN ещё поднят: обмотка замкнута через верхние ключи,
       * ток якоря стекает по этому контуру. Крыльчатка при этом ещё
       * вращается и работает генератором — её ЭДС тоже гасится здесь.      */
      fan[i].duty_q = 0;
      Fan_ApplyDuty(i);

      if (cmd) {
        Fan_BeginStart(i, now);  /* мост под разрешением — можно сразу      */
        break;
      }
      if ((now - fan[i].state_tick) >= FAN_DECAY_MS) {
        Fan_DisableDriver(i);    /* ток уже стёк — размыкать безопасно      */
        Fan_EnterState(i, FAN_STATE_OFF, now);
      }
      break;

    case FAN_STATE_FAULT:
      /* Скважность обнулена ещё в Fan_Trip; ждём стекания и снимаем EN. */
      fan[i].duty_q = 0;
      Fan_ApplyDuty(i);

      if (fan[i].enabled && (now - fan[i].state_tick) >= FAN_DECAY_MS) {
        Fan_DisableDriver(i);
      }

      if (!cmd) {
        /* Оператор снял команду — авария сбрасывается, лимит попыток тоже. */
        fan[i].fault   = FAN_FAULT_NONE;
        fan[i].retries = 0;
        Fan_EnterState(i, FAN_STATE_OFF, now);
        break;
      }

      /* Охлаждение важнее «красивой» защёлки: канал сам пробует запуститься
       * ещё раз. Если причина не ушла — после FAN_FAULT_RETRY_MAX попыток
       * канал остаётся выключенным до снятия команды.                      */
      if (fan[i].retries < FAN_FAULT_RETRY_MAX &&
          (now - fan[i].state_tick) >= FAN_FAULT_RETRY_MS) {
        fan[i].retries++;
        fan[i].fault = FAN_FAULT_NONE;
        Fan_EnterState(i, FAN_STATE_OFF, now);
      }
      break;

    default:
      Fan_BeginStop(i, now);
      break;
  }
}

void Cooling_Update(void)
{
  uint32_t now = HAL_GetTick();

  for (uint8_t i = 0; i < FAN_COUNT; i++) {
    Fan_Step(i, now);
  }
}

/* ==========================================================================
 * Защита по току.
 *
 * АЦП обходит 8 каналов примерно за 224 мкс и не синхронизирован с ШИМ
 * 16 кГц, поэтому отдельная выборка случайно попадает либо на такт
 * проводимости, либо на паузу. Отсюда две независимые ветви:
 *   - пиковая — «дырявое ведро» по мгновенным выборкам (ловит короткое
 *     замыкание и клин ротора за единицы миллисекунд, не реагируя на
 *     одиночные выбросы АЦП);
 *   - средняя — экспоненциальный фильтр (ловит длительную перегрузку и
 *     обрыв нагрузки, не реагируя на разрывность выборок).
 * ========================================================================== */

static void Fan_CheckCurrent(uint8_t i, uint32_t now)
{
  /* Ток контролируем только пока мост реально питает обмотку. В DECAY и
   * FAULT скважность нулевая, в OFF мост обесточен — накопители сбрасываем,
   * чтобы следующий пуск начинался с чистого состояния.                     */
  if (fan[i].state != FAN_STATE_STARTING &&
      fan[i].state != FAN_STATE_RUNNING  &&
      fan[i].state != FAN_STATE_STOPPING) {
    Fan_ResetProtection(i);
    return;
  }

  int32_t  raw   = (int32_t)ADS_RES_BUFFER[fan_hw[i].adc_idx];
  int32_t  diff  = raw - (int32_t)fan[i].zero_adc;
  uint32_t delta = (uint32_t)((diff < 0) ? -diff : diff);

  /* --- Пиковая защита: короткое замыкание / заклиненный ротор. --- */
  if (delta > FAN_OC_PEAK_ADC) {
    fan[i].oc_peak_acc += FAN_OC_PEAK_INC;
    if (fan[i].oc_peak_acc >= FAN_OC_PEAK_LIMIT) {
      Fan_Trip(i, FAN_FAULT_PEAK, now);
      return;
    }
  } else if (fan[i].oc_peak_acc > 0) {
    fan[i].oc_peak_acc--;
  }

  /* --- Средний ток: экспоненциальный фильтр по модулю отклонения.
   * Аккумулятор хранится домноженным на 2^FAN_I_AVG_SHIFT — иначе выборки
   * меньше 2^SHIFT отсчётов терялись бы при сдвиге и фильтр «слепнул» бы на
   * малых токах, ровно там, где работает контроль обрыва нагрузки.
   * Максимум аккумулятора 4095 << 6 = 262080 — в uint32 помещается.        */
  fan[i].i_acc = fan[i].i_acc - Fan_AvgCurrent(i) + delta;
  uint32_t i_avg = Fan_AvgCurrent(i);

  /* --- Длительная перегрузка. --- */
  if (i_avg > FAN_OC_AVG_ADC) {
    if (fan[i].ovl_since == 0) {
      fan[i].ovl_since = (now != 0) ? now : 1;
    } else if ((now - fan[i].ovl_since) >= FAN_OC_AVG_TRIP_MS) {
      Fan_Trip(i, FAN_FAULT_OVERLOAD, now);
      return;
    }
  } else {
    fan[i].ovl_since = 0;
  }

  /* --- Обрыв нагрузки: проверяем только на установившейся рабочей
   * скважности, где ток заведомо должен быть. В разгоне и выбеге низкий
   * средний ток нормален.                                                   */
  if (fan[i].state == FAN_STATE_RUNNING && i_avg < FAN_OPEN_LOAD_ADC) {
    if (fan[i].open_since == 0) {
      fan[i].open_since = (now != 0) ? now : 1;
    } else if ((now - fan[i].open_since) >= FAN_OPEN_LOAD_MS) {
      /* Тока нет — стекать нечему, но проходим тем же путём: единый
       * безопасный порядок «скважность 0 -> выдержка -> снятие EN».        */
      Fan_Trip(i, FAN_FAULT_OPEN_LOAD, now);
      return;
    }
  } else {
    fan[i].open_since = 0;
  }
}

void Cooling_CheckOvercurrent(void)
{
  static uint32_t sample_tick = 0;

  uint32_t now = HAL_GetTick();

  /* Собственный тик выборки: главный цикл быстрее АЦП в сотни раз, и без
   * этого ограничения одно и то же значение буфера попадало бы в счётчики
   * защиты многократно (см. FAN_I_SAMPLE_MS в config.h).                   */
  if ((now - sample_tick) < FAN_I_SAMPLE_MS) {
    return;
  }
  sample_tick = now;

  for (uint8_t i = 0; i < FAN_COUNT; i++) {
    Fan_CheckCurrent(i, now);
  }
}

/* ========================================================================== */
void Cooling_GetStatus(uint8_t fan_index, FanStatus *out)
{
  if (fan_index >= FAN_COUNT || out == 0) {
    return;
  }

  out->state      = fan[fan_index].state;
  out->fault      = fan[fan_index].fault;
  out->duty_pct   = (uint16_t)(fan[fan_index].duty_q / DUTY_SCALE);
  out->i_avg_adc  = (uint16_t)Fan_AvgCurrent(fan_index);
  out->zero_adc   = (uint16_t)fan[fan_index].zero_adc;
  out->zero_valid = fan[fan_index].zero_valid;
  out->retries    = fan[fan_index].retries;
  out->trips      = fan[fan_index].trips;
  out->enabled    = fan[fan_index].enabled;
}
