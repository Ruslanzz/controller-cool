/**
  ******************************************************************************
  * @file    io.h
  * @brief   Низкоуровневый дискретный ввод-вывод: массивы входов comp[] и
  *          выходных реле relay[], помощники чтения/записи.
  ******************************************************************************
  */

#ifndef __IO_H
#define __IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct {
    GPIO_TypeDef *port;  /* порт GPIO  */
    uint16_t      pin;   /* номер пина */
} GPIO_Config;

/* 8 дискретных входов (компараторы/концевики) и 5 силовых реле. */
extern GPIO_Config comp[8];
extern GPIO_Config relay[5];

/* Прочитать состояние входа: 1 — активен, 0 — нет. */
int  IO_ReadPin(GPIO_Config config);

void IO_RelayOn(uint8_t relay_index);
void IO_RelayOff(uint8_t relay_index);

#ifdef __cplusplus
}
#endif

#endif /* __IO_H */
