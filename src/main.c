/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Точка входа и главный цикл — узел ОХЛАЖДЕНИЯ.
  *
  *  Узел управляет двумя вентиляторами SPAL VA26-AP50/C-44A на силовых выходах
  *  L1 (DRV1) и L2 (DRV2). Команда пуска приходит от ведущего (device_id 0x01)
  *  в кадре comp — байт COMP_DEADMAN, тот же вход компаратора, по которому
  *  ведущий разрешает выдачу оборотов на VESC. Вентиляторы плавно
  *  раскручиваются до половины оборотов, останавливаются управляемым выбегом
  *  со стеканием индукции и защищены по току (пик, перегрузка, обрыв
  *  нагрузки), по температуре платы и по частоте пусков.
  *
  *  Модули: bsp, io, cooling, can_bus.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "bsp.h"
#include "config.h"
#include "io.h"
#include "cooling.h"
#include "thermal.h"
#include "can_bus.h"

/**
  * @brief  The application entry point.
  */
int main(void)
{
  BSP_Init();

  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Вывести трансивер CAN из режима ожидания и запустить шину с приёмом. */
  HAL_GPIO_WritePin(GPIOA, CAN_STB_Pin, GPIO_PIN_RESET);
  CanBus_Start();

  /* ПОРЯДОК ЗДЕСЬ ПРИНЦИПИАЛЕН, и причина — в особенности каскада
   * (DRV_OUTPUT_STAGE.md, раздел про ловушку с EN):
   *
   *   1) Cooling_Init закрывает ОБА плеча у обоих каналов — до того, как ШИМ
   *      попадёт на ноги, иначе на плечи ушли бы значения сравнения по
   *      умолчанию, а ноль на верхнем плече означает полный ход вентилятора;
   *   2) ШИМ запускается;
   *   3) и только теперь поднимается EN — один раз за всё время работы.
   *
   * Снимать EN после этого нельзя: он открывает верхний P-канальный ключ
   * ПОЛНОСТЬЮ, и записи в регистры сравнения перестают на что-либо влиять.
   * Именно поэтому «останов» больше не трогает EN, а закрывает плечи. */
  Cooling_Init();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  Cooling_EnableDrivers();

  /* TIM1: база + канал CH3 — периодический тик отправки телеметрии. */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);

  /* Запуск непрерывного измерения АЦП по DMA (8 каналов): датчики тока
   * вентиляторов (NSM2012 на CH5/CH6) и температуры. */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADS_RES_BUFFER, 8);

  /* Ноль датчиков тока снимается при ПОДНЯТОМ EN и закрытых плечах: обмотки
   * разомкнуты, тока через датчики нет. Ждать «пока мосты обесточены» здесь
   * нельзя — снятый EN означает открытый верхний ключ. */
  Cooling_CalibrateCurrentSensors();

  /* Тепловая защита платы по двум датчикам. Пороги отсюда — только начальные:
   * действующие лежат в temp_trip_adc/temp_clear_adc и меняются на ходу. */
  Thermal_Init();

  /* Общее реле питания. */
  IO_RelayOn(RELAY_POWER);

  while (1)
  {
    CanBus_CheckTimeout();        /* потеря связи -> управляемый останов     */
    Cooling_CheckOvercurrent();   /* пик / перегрузка / обрыв нагрузки       */
    Thermal_Update();             /* перегрев платы -> останов до остывания  */
    Cooling_Update();             /* разгон, выбег, стекание индукции, EN    */
  }
}

/* ==========================================================================
 * Колбэки и обработчики прерываний прикладного уровня.
 * ========================================================================== */

/** Прерывание сравнения TIM1. */
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/** Диспетчер событий Output Compare TIM1 по каналам. */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1) {
    return;
  }

  switch (htim->Channel) {
    case HAL_TIM_ACTIVE_CHANNEL_3:
      /* Периодическая отправка телеметрии и перепланирование тика. */
      CanBus_TxTask();
      BSP_TimerAdvanceCompare(&htim1, TIM_CHANNEL_3, tim1_ch3_pulse);
      break;

    default:
      /* CH1/CH2/CH4 узлом охлаждения не используются. */
      break;
  }
}
