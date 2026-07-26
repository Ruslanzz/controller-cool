/**
  ******************************************************************************
  * @file    bsp.h
  * @brief   Board Support Package: периферия STM32F103, её инициализация и
  *          низкоуровневые помощники.
  *
  *          Сюда вынесена вся сгенерированная CubeMX инициализация (тактирование,
  *          GPIO, ADC+DMA, CAN, TIM1, TIM4), хэндлы периферии и общий помощник
  *          для перепланирования сравнения таймера.
  *
  *          У узла охлаждения TIM4 несёт ШИМ обоих вентиляторов: CH1/CH2 —
  *          мост DRV1 (выход L1), CH3/CH4 — мост DRV2 (выход L2).
  ******************************************************************************
  */

#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===== Хэндлы периферии =================================================== */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

/* Буфер результатов АЦП (8 каналов), заполняется по DMA.                     */
extern uint32_t ADS_RES_BUFFER[8];

/* ===== Конфигурация импульсов TIM1 (каналы сравнения) ===================== */
/* CH3 — тик рассылки CAN; CH1/CH2/CH4 узлом охлаждения не используются.      */
extern volatile uint32_t tim1_ch1_pulse;
extern volatile uint32_t tim1_ch2_pulse;
extern volatile uint32_t tim1_ch3_pulse;
extern volatile uint32_t tim1_ch4_pulse;

/* ===== Инициализация ====================================================== */
void BSP_Init(void);              /* тактирование + вся периферия            */
void SystemClock_Config(void);

/* ===== Помощники ========================================================== */
/**
 * @brief  Сдвинуть значение сравнения канала таймера на @p pulse отсчётов
 *         вперёд с защитой от переполнения ARR (перепланирование прерывания).
 */
void BSP_TimerAdvanceCompare(TIM_HandleTypeDef *htim, uint32_t channel,
                             uint32_t pulse);

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */
