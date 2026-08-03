/**
  ******************************************************************************
  * @file    cooling.c
  * @brief   Реализация управления вентиляторами радиатора (L1 = DRV1, L2 = DRV2).
  *
  *          Полярность моста — как у балки на controller-front и у катушек на
  *          ведущем: IN_A (регулируемое плечо, верхний P-канальный ключ) со
  *          сравнением PWM_PERIOD даёт нулевой ток, 0 — полный; IN_B (нижний
  *          N-канальный) при работе держится открытым (PWM_PERIOD).
  *
  *          Из этого следует ключевое для индуктивной нагрузки свойство: в
  *          паузе ШИМ ток якоря замыкается по контуру
  *          GND -> VD32/33/34 -> обмотка -> датчик -> нижний ключ -> GND и
  *          не рвётся. Поэтому «скважность 0 при открытом нижнем плече» — не
  *          выключение, а режим гашения. Обмотка размыкается закрытием
  *          НИЖНЕГО плеча после FAN_DECAY_MS в этом режиме.
  *
  *          EN обоих драйверов поднимается один раз в Cooling_EnableDrivers и
  *          больше не трогается: снятие EN открывает верхний ключ ПОЛНОСТЬЮ,
  *          то есть включает вентилятор на полный ход (DRV_OUTPUT_STAGE.md).
  *          Прежняя версия этого модуля снимала EN в OFF и FAULT — то есть
  *          «останов» и срабатывание защиты давали полные обороты. Здесь
  *          «выключено» всегда означает положение ПЛЕЧ, а не состояние EN.
  ******************************************************************************
  */

#include "cooling.h"
#include "config.h"
#include "bsp.h"
#include "thermal.h"

/* Скважность хранится в сотых долях процента: шаг разгона получается плавным
 * без накопления ошибки целочисленного деления.                             */
#define DUTY_SCALE      100u
#define DUTY_MAX_Q      (100u * DUTY_SCALE)
#define DUTY_TARGET_Q   ((uint32_t)FAN_DUTY_TARGET * DUTY_SCALE)

/* ===== Неизменная привязка канала к железу ================================ */
typedef struct {
  uint32_t      ch_a;      /* канал TIM4: регулируемое плечо IN_A (верхнее)  */
  uint32_t      ch_b;      /* канал TIM4: нижнее плечо IN_B                  */
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
  uint8_t  connected;    /* нижнее плечо открыто: обмотка подключена к мосту */

  uint32_t duty_q;       /* текущая скважность, сотые доли процента         */
  uint32_t ramp_from_q;  /* скважность на начало текущей рампы              */
  uint32_t ramp_to_q;    /* цель текущей рампы                              */
  uint32_t ramp_ms;      /* длительность текущей рампы                      */
  uint32_t ramp_tick;    /* отметка начала рампы                            */

  uint32_t state_tick;   /* отметка входа в текущее состояние               */

  uint32_t zero_adc;     /* ноль датчика тока (измеренный или номинальный)  */
  uint8_t  zero_valid;
  uint32_t ceiling_ma;   /* потолок измерения при этом нуле, мА             */
  uint32_t oc_peak_ma;   /* действующий пиковый порог, мА                   */
  uint8_t  oc_enabled;   /* 0 — защита канала отключена (нет доверия/запаса)*/
  uint8_t  saturated;    /* показание упирается в потолок АЦП               */
  uint32_t i_acc;        /* аккумулятор ЭФ среднего тока (масштаб 2^SHIFT)  */
  uint32_t oc_hits[FAN_OC_WINDOW_HITS];  /* окно превышений пикового порога */
  uint8_t  oc_head;
  uint8_t  oc_count;
  uint32_t ovl_since;    /* начало непрерывной перегрузки, 0 — нет          */
  uint32_t open_since;   /* начало отсутствия тока под нагрузкой, 0 — нет   */

  uint8_t  retries;      /* повторных пусков после аварии подряд            */
  uint16_t trips;        /* всего срабатываний защиты                       */

  uint8_t  start_count;  /* «дырявый» счётчик пусков                        */
  uint32_t decay_tick;   /* отметка последнего распада счётчика пусков      */
  uint32_t cooldown_tick;/* начало паузы по частоте пусков, 0 — паузы нет   */

  /* Механический выбег: скважность, на которой крыльчатку перестали крутить,
   * и момент этого события. По ним оценивается текущая скорость вращения
   * (см. Fan_CoastDuty) — она нужна для подхвата при повторном пуске.      */
  uint32_t coast_q;
  uint32_t coast_tick;
} FanRt;

static FanRt fan[FAN_COUNT];

/* --- Команда: сырая (из CAN) -> после антидребезга -> эффективная ---------
 * Сырое значение приходит из прерывания приёма CAN. Устойчивым оно считается
 * только продержавшись FAN_CMD_DEBOUNCE_MS: дребезг механического контакта
 * deadman на ведущем не должен доходить до силовой части. Эффективная команда
 * — устойчивая И снятая тепловой защитой; по фронту её включения
 * отсчитывается разнос пусков двух вентиляторов.                            */
static volatile uint8_t  cmd_raw          = 0;
static volatile uint32_t cmd_raw_tick     = 0;
static uint8_t           cmd_stable       = 0;
static uint8_t           cmd_effective    = 0;
static uint32_t          cmd_effective_tick = 0;

/* --- Запрет пуска сразу после сброса МК -----------------------------------
 * Если МК перезагрузился на ходу (просадка питания, watchdog, перепрошивка),
 * EN снялись — и крыльчатка осталась вращаться, а её обороты после сброса
 * измерить нечем: состояние выбега не пережило перезагрузку.
 *
 * Гадать здесь опаснее, чем ждать, причём в обе стороны. Считать «стоит»,
 * когда она крутится, — значит подать EN на нулевой скважности, то есть
 * замкнуть накоротко генератор (десятки ампер). Считать «крутится», когда
 * она стоит, — значит с ходу подать заметную скважность на неподвижный
 * ротор, то есть тот же бросок тока.
 *
 * Поэтому первый пуск после сброса просто откладывается на FAN_COAST_MS:
 * за это время любой выбег заведомо закончится, и обычный пуск с нуля станет
 * безопасным. Охлаждение при этом не теряется — ровно эти секунды крыльчатка
 * и продолжает гнать воздух по инерции.                                     */
static uint32_t boot_tick = 0;

/* ==========================================================================
 * Низкий уровень: положение плеч.
 *
 * EN здесь не фигурирует вообще, и это главное отличие от прежней версии:
 * снятие EN открывает верхний ключ ПОЛНОСТЬЮ (DRV_OUTPUT_STAGE.md), поэтому
 * управлять им нельзя — он поднимается один раз в Cooling_EnableDrivers.
 * Всё управление — это положение плеч:
 *
 *   работа:    нижнее открыто, верхнее по скважности   (Fan_ApplyDuty)
 *   гашение:   нижнее открыто, верхнее закрыто          (Fan_ApplyDuty, duty 0)
 *   разомкнуто: оба закрыты, выход высокоомный          (Fan_Disconnect)
 * ========================================================================== */

/* Выставить скважность на регулируемом плече и открыть нижнее: обмотка
 * подключена к мосту. duty_q = 0 -> верхнее плечо закрыто, нижнее открыто,
 * то есть контур гашения замкнут (ток якоря идёт GND -> VD32/33/34 ->
 * обмотка -> датчик -> нижний ключ -> GND).
 *
 * Слишком короткие импульсы не выдаются вовсе. В начале разгона расчётная
 * длительность падает до единиц наносекунд — драйвер за такое время не успеет
 * полностью открыть ключ, и тот окажется в линейном режиме, рассеивая всю
 * мощность на кристалле. Отсечка симметрична: и «чуть-чуть открыт», и
 * «почти полностью открыт» приводятся к ближайшему крайнему состоянию.      */
static void Fan_ApplyDuty(uint8_t i)
{
  uint32_t on = ((uint32_t)PWM_PERIOD * fan[i].duty_q) / DUTY_MAX_Q;

  if (on < FAN_PWM_MIN_PULSE) {
    on = 0;                                   /* закрыто, без огрызка       */
  } else if (on > (uint32_t)(PWM_PERIOD - FAN_PWM_MIN_PULSE)) {
    on = PWM_PERIOD;                          /* открыто полностью          */
  }

  __HAL_TIM_SET_COMPARE(&htim4, fan_hw[i].ch_b, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, fan_hw[i].ch_a, PWM_PERIOD - on);
  fan[i].connected = 1;
}

/* Разомкнуть обмотку: ОБА плеча закрыты, выход в высокоомном состоянии.
 * Это и есть настоящее «выключено» на данном каскаде — то, что прежде
 * пытались получить снятием EN. Крыльчатка после этого выбегает свободно:
 * ЭДС генератора ниже питания, банки обратных диодов закрыты, тока нет.
 *
 * Вызывать только после выдержки гашения (FAN_DECAY_MS) при нулевой
 * скважности: размыкать обмотку с током значило бы оборвать индуктивность.  */
static void Fan_Disconnect(uint8_t i)
{
  __HAL_TIM_SET_COMPARE(&htim4, fan_hw[i].ch_a, PWM_PERIOD);  /* верхнее закр. */
  __HAL_TIM_SET_COMPARE(&htim4, fan_hw[i].ch_b, 0);           /* нижнее закр.  */
  fan[i].connected = 0;
}

void Cooling_EnableDrivers(void)
{
  /* РОВНО ОДИН РАЗ за всё время работы прошивки, ПОСЛЕ Cooling_Init и запуска
   * ШИМ. Обратной операции в этом модуле нет вообще: снятие EN открывает
   * верхний P-канальный ключ полностью и обесценивает записи в регистры
   * сравнения.                                                              */
  for (uint8_t i = 0; i < FAN_COUNT; i++) {
    HAL_GPIO_WritePin(fan_hw[i].en_port, fan_hw[i].en_a_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(fan_hw[i].en_port, fan_hw[i].en_b_pin, GPIO_PIN_SET);
  }
}

/* Отсчёты АЦП от нуля датчика -> миллиамперы. Масштаб выводится из
 * ИЗМЕРЕННОГО нуля (датчик ратиометричный) — вывод в config.h.              */
static uint32_t Fan_AdcToMa(uint8_t i, uint32_t adc_delta)
{
  if (fan[i].zero_adc == 0) {
    return 0;
  }
  return (adc_delta * (uint32_t)FAN_ADC_FULL_SCALE_MA) / fan[i].zero_adc;
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
  fan[i].i_acc      = 0;
  fan[i].oc_head    = 0;
  fan[i].oc_count   = 0;
  fan[i].ovl_since  = 0;
  fan[i].open_since = 0;
}

/* Средний ток канала в отсчётах АЦП (положительное отклонение от нуля). */
static uint32_t Fan_AvgCurrent(uint8_t i)
{
  return fan[i].i_acc >> FAN_I_AVG_SHIFT;
}

/* Средний ток канала в миллиамперах. */
static uint32_t Fan_AvgCurrentMa(uint8_t i)
{
  return Fan_AdcToMa(i, Fan_AvgCurrent(i));
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

/* --------------------------------------------------------------------------
 * Ограничитель частоты пусков.
 *
 * Каждый пуск — всплеск тока и порция тепла в ключах моста. Счётчик пусков
 * «дырявый»: пуск добавляет 1, каждые FAN_CYCLE_DECAY_MS вычитается 1.
 * Набралось FAN_CYCLE_LIMIT — канал уходит в паузу FAN_CYCLE_COOLDOWN_MS,
 * за которую кристалл успевает остыть. Редкие штатные включения до лимита
 * не доходят никогда.
 * -------------------------------------------------------------------------- */
static uint8_t Fan_InCooldown(uint8_t i, uint32_t now)
{
  if (fan[i].cooldown_tick == 0) {
    return 0;
  }
  if ((now - fan[i].cooldown_tick) < FAN_CYCLE_COOLDOWN_MS) {
    return 1;
  }
  /* Пауза выдержана — счётчик обнуляется, но если дребезг не прекратился,
   * следующая пачка пусков заново упрётся в лимит.                         */
  fan[i].cooldown_tick = 0;
  fan[i].start_count   = 0;
  return 0;
}

/* Плавный распад счётчика пусков. */
static void Fan_DecayStartCount(uint8_t i, uint32_t now)
{
  if ((now - fan[i].decay_tick) < FAN_CYCLE_DECAY_MS) {
    return;
  }
  fan[i].decay_tick = now;
  if (fan[i].start_count > 0) {
    fan[i].start_count--;
  }
}

/* --------------------------------------------------------------------------
 * Механический выбег крыльчатки.
 *
 * Крыльчатка Ø280 мм после снятия питания вращается ещё 10..20 с, и всё это
 * время двигатель остаётся генератором. Для повторного пуска это принципиально:
 * при duty = 0 оба плеча моста подняты, то есть обмотка ЗАМКНУТА НАКОРОТКО —
 * подать EN на вращающуюся крыльчатку значит замкнуть накоротко генератор и
 * получить ток ЭДС/Ra в десятки ампер. Поэтому мост сначала выставляется на
 * скважность, эквивалентную текущим оборотам, и лишь потом получает EN.
 *
 * Оборотов измерять нечем, поэтому они оцениваются по времени выбега:
 * сопротивление воздуха растёт как квадрат оборотов, значит выбег
 * гиперболический — w(t) = w0 / (1 + t/tau).
 * -------------------------------------------------------------------------- */

/* Запомнить точку начала выбега. @p duty_at_stop — скважность, на которой
 * крыльчатку перестали крутить (0, если её успели затормозить рампой).      */
static void Fan_MarkCoast(uint8_t i, uint32_t duty_at_stop, uint32_t now)
{
  fan[i].coast_q    = duty_at_stop;
  fan[i].coast_tick = now;
}

/* Оценка скважности, эквивалентной текущим оборотам выбегающей крыльчатки.
 * 0 — крыльчатка стоит (или её остановили управляемой рампой).              */
static uint32_t Fan_CoastDuty(uint8_t i, uint32_t now)
{
  if (fan[i].coast_q == 0) {
    return 0;
  }

  /* Без побочных эффектов: функция вызывается и из телеметрии (прерывание
   * TIM1), и из главного цикла.                                            */
  uint32_t t = now - fan[i].coast_tick;
  if (t >= FAN_COAST_MS) {
    return 0;                       /* выбег заведомо закончился */
  }

  uint32_t q = (fan[i].coast_q * FAN_COAST_TAU_MS) / (FAN_COAST_TAU_MS + t);
  return (q > DUTY_TARGET_Q) ? DUTY_TARGET_Q : q;
}

/* Разрешён ли пуск прямо сейчас. Разрешение РАСХОДУЕТ кредит счётчика, поэтому
 * вызывается ровно один раз на пуск.                                        */
static uint8_t Fan_StartAllowed(uint8_t i, uint32_t now)
{
  if (Fan_InCooldown(i, now)) {
    return 0;
  }
  if (fan[i].start_count >= FAN_CYCLE_LIMIT) {
    fan[i].cooldown_tick = (now != 0) ? now : 1;
    return 0;
  }
  fan[i].start_count++;
  return 1;
}

/* Начать разгон к рабочей скважности (мост уже под разрешением).
 * Скважность сначала подводится к текущим оборотам выбегающей крыльчатки —
 * иначе разгон «с нуля» означал бы замыкание вращающегося генератора.
 * Возвращает 0, если пуск запрещён ограничителем частоты.                   */
static uint8_t Fan_BeginStart(uint8_t i, uint32_t now)
{
  if (!Fan_StartAllowed(i, now)) {
    return 0;
  }

  uint32_t spin_q = Fan_CoastDuty(i, now);
  if (spin_q > fan[i].duty_q) {
    fan[i].duty_q = spin_q;         /* подхват вращающейся крыльчатки */
    Fan_ApplyDuty(i);
  }
  /* Оценка использована: дальше обороты задаёт скважность, а не выбег. */
  fan[i].coast_q = 0;

  Fan_StartRamp(i, DUTY_TARGET_Q, FAN_RAMP_UP_MS, now);
  Fan_EnterState(i, FAN_STATE_STARTING, now);
  return 1;
}

/* Аварийное отключение канала.
 *
 * Скважность обнуляется НЕМЕДЛЕННО, но нижнее плечо остаётся открытым: мост
 * переходит в то же гашение, что и в любой паузе ШИМ, и ток якоря замыкается
 * в контур гашения. Размыкание обмотки выполняет автомат по истечении
 * FAN_DECAY_MS. Рвать контур тока прямо в момент аварии нельзя — выброс
 * индуктивности пришёлся бы на ключи, которые и так работают на пределе.
 *
 * EN при этом не трогается: его снятие открыло бы верхний ключ полностью, то
 * есть защита включила бы вентилятор на полный ход вместо отключения.       */
static void Fan_Trip(uint8_t i, FanFault fault, uint32_t now)
{
  /* Крыльчатку никто не тормозил — она продолжает вращаться на оборотах,
   * соответствующих текущей скважности. Запоминаем точку начала выбега,
   * чтобы повторный пуск подхватил её, а не замкнул накоротко.             */
  Fan_MarkCoast(i, fan[i].duty_q, now);

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
  boot_tick = HAL_GetTick();

  for (uint8_t i = 0; i < FAN_COUNT; i++) {
    fan[i].state      = FAN_STATE_OFF;
    fan[i].fault      = FAN_FAULT_NONE;
    fan[i].duty_q     = 0;
    fan[i].zero_adc   = FAN_ZERO_NOMINAL_ADC;
    fan[i].zero_valid = 0;
    fan[i].ceiling_ma = 0;
    fan[i].oc_peak_ma = FAN_OC_PEAK_MA;
    fan[i].oc_enabled = 0;
    fan[i].saturated  = 0;
    fan[i].retries    = 0;
    fan[i].trips      = 0;
    fan[i].state_tick = HAL_GetTick();

    fan[i].start_count   = 0;
    fan[i].decay_tick    = HAL_GetTick();
    fan[i].cooldown_tick = 0;

    fan[i].coast_q    = 0;
    fan[i].coast_tick = HAL_GetTick();

    Fan_ResetProtection(i);

    /* Безопасное положение плеч ДО запуска ШИМ и до подачи EN: оба плеча
     * закрыты, выход высокоомный. Ноль на верхнем плече (значение сравнения
     * по умолчанию) означал бы вентилятор на полном ходу.                   */
    Fan_Disconnect(i);
  }
}

void Cooling_CalibrateCurrentSensors(void)
{
  uint32_t sum[FAN_COUNT] = {0};

  /* АЦП уже крутится по DMA; дать буферу заполниться и усреднить. Оба плеча в
   * этот момент закрыты при уже поднятом EN — тока через датчики нет, они
   * показывают собственный ноль. Ждать «пока мосты обесточены» здесь нельзя:
   * снятый EN означает открытый верхний ключ, то есть работающий вентилятор. */
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
      /* Датчик неисправен или не подключён: берём номинальный ноль и
       * ОТКЛЮЧАЕМ защиту канала. Прежде здесь защита оставалась в работе с
       * номинальным нулём — но при неисправном датчике она работает наоборот:
       * показание около нуля читается как отсутствие тока, и контроль обрыва
       * нагрузки сам останавливает исправный вентилятор (см. config.h).     */
      fan[i].zero_adc   = FAN_ZERO_NOMINAL_ADC;
      fan[i].zero_valid = 0;
    }

    /* Потолок измерения: выше 4095 отсчётов АЦП не видит ничего. Если пиковый
     * порог не помещается под него с запасом, защита канала честно
     * отключается — мёртвый порог хуже честно выключенного, он создаёт ложное
     * ощущение защиты (факт виден в телеметрии).                            */
    fan[i].ceiling_ma = (fan[i].zero_adc < 4095u)
                          ? Fan_AdcToMa(i, 4095u - fan[i].zero_adc) : 0u;

    fan[i].oc_peak_ma = FAN_OC_PEAK_MA;
    fan[i].oc_enabled = (uint8_t)(fan[i].zero_valid &&
                                  fan[i].ceiling_ma >=
                                    ((uint32_t)FAN_OC_PEAK_MA +
                                     (uint32_t)FAN_OC_MIN_HEADROOM_MA));
  }
}

/* ==========================================================================
 * Команда ведущего.
 * ========================================================================== */

void Cooling_SetCommand(uint8_t on)
{
  uint8_t cmd = (on != 0);

  /* Здесь только фиксируем сырое значение и момент его изменения; решение
   * принимает антидребезг в Cooling_Update.                                */
  if (cmd != cmd_raw) {
    cmd_raw      = cmd;
    cmd_raw_tick = HAL_GetTick();
  }
}

/* Антидребезг команды: сырое значение становится устойчивым, только
 * продержавшись FAN_CMD_DEBOUNCE_MS. Дребезг контакта deadman на ведущем
 * иначе превратился бы в пачку пусков и остановов силовой части.           */
static void Cooling_UpdateCommand(uint32_t now)
{
  if (cmd_raw != cmd_stable && (now - cmd_raw_tick) >= FAN_CMD_DEBOUNCE_MS) {
    cmd_stable = cmd_raw;
  }

  /* Эффективная команда: устойчивая и не снятая тепловой защитой. Фронт её
   * включения — точка отсчёта разноса пусков; поэтому после снятия перегрева
   * вентиляторы снова стартуют вразнобой, а не оба разом.                  */
  uint8_t eff = (uint8_t)(cmd_stable && !Thermal_Limit());
  if (eff && !cmd_effective) {
    cmd_effective_tick = now;
  }
  cmd_effective = eff;
}

/* Команда конкретному каналу. Второй вентилятор стартует на FAN_STAGGER_MS
 * позже первого: пусковые токи двух двигателей не складываются на общем
 * питании. На выключение разнос не действует — останов синхронный.          */
static uint8_t Fan_Command(uint8_t i, uint32_t now)
{
  if (!cmd_effective) {
    return 0;
  }
  if (i == 0) {
    return 1;
  }
  return ((now - cmd_effective_tick) >= FAN_STAGGER_MS) ? 1 : 0;
}


/* ==========================================================================
 * Автомат канала.
 * ========================================================================== */

static void Fan_Step(uint8_t i, uint32_t now)
{
  uint8_t cmd = Fan_Command(i, now);

  Fan_DecayStartCount(i, now);

  switch (fan[i].state) {

    case FAN_STATE_OFF:
      /* Обмотка разомкнута (оба плеча закрыты). Страховка на случай прихода
       * сюда из FAULT до истечения выдержки гашения: размыкаем, выдержав ту
       * же паузу, чтобы не рвать обмотку с током.                          */
      if (fan[i].connected && (now - fan[i].state_tick) >= FAN_DECAY_MS) {
        Fan_Disconnect(i);
      }

      /* Пуск — только после FAN_MIN_OFF_MS: пауза ограничивает темп пусков и
       * гарантирует, что любой переходный процесс в обмотке закончился.    */
      if (cmd && (now - fan[i].state_tick) >= FAN_MIN_OFF_MS &&
          (now - boot_tick) >= FAN_COAST_MS &&
          Fan_StartAllowed(i, now)) {
        /* Крыльчатка может ещё выбегать (после аварии — до 20 с). Скважность
         * подводится к её текущим оборотам ТЕМ ЖЕ действием, которым обмотка
         * подключается к мосту: подключить её на нулевой скважности значит
         * замкнуть вращающийся генератор на контур гашения и получить ток
         * ЭДС/Ra в десятки ампер.                                          */
        fan[i].duty_q = Fan_CoastDuty(i, now);
        Fan_ApplyDuty(i);
        fan[i].coast_q = 0;

        Fan_ResetProtection(i);
        Fan_StartRamp(i, DUTY_TARGET_Q, FAN_RAMP_UP_MS, now);
        Fan_EnterState(i, FAN_STATE_STARTING, now);
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
       * обмотку размыкать не начинали. Если ограничитель частоты пусков не
       * пустил, останов продолжается: бросать канал на полпути нельзя.     */
      if (cmd && Fan_BeginStart(i, now)) {
        break;
      }
      if (Fan_StepRamp(i, now)) {
        /* Рампа тормозила крыльчатку вместе с собой (мост всё это время
         * управлял током), поэтому к концу выбега почти не остаётся.
         * Остаточное отставание рампы от ротора — единицы процентов
         * скважности, это единицы ампер при подхвате, втрое ниже порога
         * пиковой защиты.                                                   */
        Fan_MarkCoast(i, fan[i].duty_q, now);
        Fan_EnterState(i, FAN_STATE_DECAY, now);
      }
      break;

    case FAN_STATE_DECAY:
      /* Скважность 0, нижнее плечо ещё открыто: ток якоря замкнут в контур
       * гашения GND -> VD32/33/34 -> обмотка -> датчик -> нижний ключ -> GND
       * и коммутируется в него без выброса напряжения.
       *
       * Это состояние КОРОТКОЕ и намеренно: замкнутая обмотка вращающегося
       * двигателя — не «стекание», а динамическое торможение, ток в ней не
       * спадает, а устанавливается на ЭДС/Ra. Держать замыкание дольше
       * необходимого значит греть ключи кинетической энергией крыльчатки.  */
      if (fan[i].connected) {
        fan[i].duty_q = 0;
        Fan_ApplyDuty(i);
      }

      /* Обмотка ещё подключена — повторный пуск можно начать сразу. Но он
       * всё равно проходит через ограничитель частоты: именно дребезг
       * команды и создаёт цикл «стоп-пуск».                                */
      if (cmd && Fan_BeginStart(i, now)) {
        break;
      }
      if ((now - fan[i].state_tick) >= FAN_DECAY_MS) {
        /* Ток якоря скоммутирован — размыкаем обмотку закрытием нижнего
         * плеча. Дальше выход высокоомный: ЭДС генератора ниже питания,
         * банки обратных диодов закрыты, тока нет, и крыльчатка выбегает
         * СВОБОДНО (10..20 с).                                             */
        Fan_Disconnect(i);
        Fan_EnterState(i, FAN_STATE_OFF, now);
      }
      break;

    case FAN_STATE_FAULT:
      /* Скважность обнулена ещё в Fan_Trip; ждём гашения и размыкаем. */
      if (fan[i].connected) {
        fan[i].duty_q = 0;
        Fan_ApplyDuty(i);

        if ((now - fan[i].state_tick) >= FAN_DECAY_MS) {
          Fan_Disconnect(i);
        }
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

  Cooling_UpdateCommand(now);

  for (uint8_t i = 0; i < FAN_COUNT; i++) {
    Fan_Step(i, now);
  }
}

/* ==========================================================================
 * Защита по току.
 *
 * АЦП обходит 8 каналов примерно за 224 мкс и не синхронизирован с ШИМ,
 * поэтому отдельные выборки садятся в случайные точки пульсации тока. Отсюда
 * две независимые ветви:
 *   - пиковая — счёт превышений в окне (ловит короткое замыкание и клин
 *     ротора за единицы миллисекунд, не реагируя на одиночные выбросы АЦП);
 *   - средняя — экспоненциальный фильтр (ловит длительную перегрузку и
 *     обрыв нагрузки, не реагируя на разрывность выборок).
 *
 * Обе работают в МИЛЛИАМПЕРАХ и только по ПОЛОЖИТЕЛЬНОМУ отклонению от нуля:
 * ток однонаправленный, поэтому отрицательное отклонение означает не переток,
 * а неисправность измерения (обоснование — в config.h).
 * ========================================================================== */

/* Зарегистрировать превышение пикового порога и проверить, набралось ли их
 * достаточно в окне. Кольцо из FAN_OC_WINDOW_HITS отметок: после записи новой
 * oc_head указывает на САМУЮ СТАРУЮ, поэтому условие срабатывания — «кольцо
 * заполнено и самая старая отметка не старше FAN_OC_WINDOW_MS».             */
static uint8_t Fan_RegisterOverCurrent(uint8_t i, uint32_t now)
{
  fan[i].oc_hits[fan[i].oc_head] = now;
  fan[i].oc_head = (uint8_t)((fan[i].oc_head + 1) % FAN_OC_WINDOW_HITS);
  if (fan[i].oc_count < FAN_OC_WINDOW_HITS) {
    fan[i].oc_count++;
  }

  if (fan[i].oc_count < FAN_OC_WINDOW_HITS) {
    return 0;
  }
  return ((now - fan[i].oc_hits[fan[i].oc_head]) <= FAN_OC_WINDOW_MS) ? 1 : 0;
}

static void Fan_CheckCurrent(uint8_t i, uint32_t now)
{
  /* Ток контролируем только пока мост реально питает обмотку. В DECAY и
   * FAULT скважность нулевая, в OFF обмотка разомкнута — накопители
   * сбрасываем, чтобы следующий пуск начинался с чистого состояния.         */
  if (fan[i].state != FAN_STATE_STARTING &&
      fan[i].state != FAN_STATE_RUNNING  &&
      fan[i].state != FAN_STATE_STOPPING) {
    Fan_ResetProtection(i);
    return;
  }

  uint32_t raw_u = ADS_RES_BUFFER[fan_hw[i].adc_idx];
  int32_t  diff  = (int32_t)raw_u - (int32_t)fan[i].zero_adc;
  uint32_t delta = (diff > 0) ? (uint32_t)diff : 0u;

  /* Показание упёрлось в потолок АЦП: истинный ток больше измеренного. */
  fan[i].saturated = (uint8_t)(raw_u >= 4090u);

  /* Датчику нельзя верить — ни одна из ветвей работать не должна: на
   * недостоверном показании «перегрузка» и «обрыв нагрузки» одинаково
   * означали бы останов исправного вентилятора (см. config.h).             */
  if (!fan[i].oc_enabled) {
    Fan_ResetProtection(i);
    return;
  }

  /* --- Пиковая защита: короткое замыкание / заклиненный ротор. --- */
  if (Fan_AdcToMa(i, delta) > fan[i].oc_peak_ma &&
      Fan_RegisterOverCurrent(i, now)) {
    Fan_Trip(i, FAN_FAULT_PEAK, now);
    return;
  }

  /* --- Средний ток: экспоненциальный фильтр по положительному отклонению.
   * Аккумулятор хранится домноженным на 2^FAN_I_AVG_SHIFT — иначе выборки
   * меньше 2^SHIFT отсчётов терялись бы при сдвиге и фильтр «слепнул» бы на
   * малых токах, ровно там, где работает контроль обрыва нагрузки.
   * Максимум аккумулятора 4095 << 6 = 262080 — в uint32 помещается.        */
  fan[i].i_acc = fan[i].i_acc - Fan_AvgCurrent(i) + delta;
  uint32_t i_avg_ma = Fan_AvgCurrentMa(i);

  /* --- Длительная перегрузка. --- */
  if (i_avg_ma > FAN_OC_AVG_MA) {
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
  if (fan[i].state == FAN_STATE_RUNNING && i_avg_ma < FAN_OPEN_LOAD_MA) {
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
  out->ma_avg     = (uint16_t)((Fan_AvgCurrentMa(fan_index) > 0xFFFFu)
                               ? 0xFFFFu : Fan_AvgCurrentMa(fan_index));
  out->zero_adc   = (uint16_t)fan[fan_index].zero_adc;
  out->ceiling_ma = (uint16_t)((fan[fan_index].ceiling_ma > 0xFFFFu)
                               ? 0xFFFFu : fan[fan_index].ceiling_ma);
  out->oc_peak_ma = (uint16_t)(fan[fan_index].oc_enabled
                               ? fan[fan_index].oc_peak_ma : 0u);
  out->oc_enabled  = fan[fan_index].oc_enabled;
  out->saturated   = fan[fan_index].saturated;
  out->zero_valid  = fan[fan_index].zero_valid;
  out->retries     = fan[fan_index].retries;
  out->trips       = fan[fan_index].trips;
  out->connected   = fan[fan_index].connected;
  out->start_count = fan[fan_index].start_count;
  out->cooldown    = (uint8_t)(fan[fan_index].cooldown_tick != 0);
  out->coast_pct   = (uint16_t)(Fan_CoastDuty(fan_index, HAL_GetTick()) / DUTY_SCALE);
}

void Cooling_GetGlobalStatus(CoolingStatus *out)
{
  if (out == 0) {
    return;
  }

  ThermalStatus th = {0};
  Thermal_GetStatus(&th);

  out->thermal_shutdown = th.limit;
  out->temp_valid[0]    = th.valid[0];
  out->temp_valid[1]    = th.valid[1];
  out->temp_over[0]     = th.over[0];
  out->temp_over[1]     = th.over[1];
  out->temp_c[0]        = th.temp_c[0];
  out->temp_c[1]        = th.temp_c[1];
  out->trip_c           = th.trip_c;
  out->clear_c          = th.clear_c;
  out->no_sensor        = th.no_sensor;
  out->cmd_raw          = cmd_raw;
  out->cmd_active       = cmd_effective;
}
