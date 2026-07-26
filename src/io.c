/**
  ******************************************************************************
  * @file    io.c
  * @brief   Реализация дискретного ввода-вывода.
  ******************************************************************************
  */

#include "io.h"
#include "config.h"

GPIO_Config comp[8] = {
  { GPIOB, COMP_ADC_1_Pin },
  { GPIOB, COMP_ADC_2_Pin },
  { GPIOB, COMP_ADC_3_Pin },
  { GPIOB, COMP_ADC_4_Pin },
  { GPIOC, COMP_ADC_5_Pin },
  { GPIOC, COMP_ADC_6_Pin },
  { GPIOC, COMP_ADC_7_Pin },
  { GPIOA, COMP_ADC_8_Pin }
};

GPIO_Config relay[5] = {
  { GPIOB, EN_RELAY_1_Pin },
  { GPIOB, EN_RELAY_2_Pin },
  { GPIOB, EN_RELAY_3_Pin },
  { GPIOB, EN_RELAY_4_Pin },
  { GPIOB, EN_RELAY_5_Pin }
};

int IO_ReadPin(GPIO_Config config)
{
  return (HAL_GPIO_ReadPin(config.port, config.pin) == GPIO_PIN_SET) ? 1 : 0;
}

void IO_RelayOn(uint8_t relay_index)
{
  if (relay_index < RELAY_COUNT) {
    HAL_GPIO_WritePin(relay[relay_index].port, relay[relay_index].pin, GPIO_PIN_SET);
  }
}

void IO_RelayOff(uint8_t relay_index)
{
  if (relay_index < RELAY_COUNT) {
    HAL_GPIO_WritePin(relay[relay_index].port, relay[relay_index].pin, GPIO_PIN_RESET);
  }
}
