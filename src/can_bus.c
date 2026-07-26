/**
  ******************************************************************************
  * @file    can_bus.c
  * @brief   Обмен по шине CAN — узел охлаждения.
  *
  *          Приём (FIFO0) кадров ведущего (device_id 0x01):
  *            BASE_COMP — снимок дискретных входов ведущего; байт
  *            COMP_DEADMAN (тот же вход компаратора, по которому ведущий
  *            разрешает выдачу оборотов на VESC) включает вентиляторы.
  *          Расширенные (29-бит) кадры — трафик VESC, игнорируются.
  *
  *          Передача: кадр BASE_COMP с состоянием собственных дискретных
  *          входов и телеметрия вентиляторов на BASE_DEBUG.
  ******************************************************************************
  */

#include "can_bus.h"
#include "config.h"
#include "bsp.h"
#include "io.h"
#include "cooling.h"

/* Заголовки и буферы передачи/приёма. */
static CAN_TxHeaderTypeDef TxHeader_Std;
static uint8_t  TxData_Std[8];

volatile CAN_Debug_t can_debug = {0};
volatile uint32_t    can_last_control_tick = 0;

static void CanBus_SendDebugFrame(void);

/* ========================================================================== */
void CanBus_Start(void)
{
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint32_t CanBus_GenerateStdId(uint8_t dev_id, uint8_t base_index,
                              uint8_t parameter_index)
{
  return ((uint32_t)dev_id << 8) | (uint32_t)(base_index + parameter_index);
}

void CanBus_SendStd(uint32_t std_id, uint8_t *data, uint8_t length)
{
  TxHeader_Std.StdId = std_id;
  TxHeader_Std.ExtId = 0x00;
  TxHeader_Std.IDE   = CAN_ID_STD;
  TxHeader_Std.RTR   = CAN_RTR_DATA;
  TxHeader_Std.DLC   = length;

  for (uint8_t i = 0; i < length; i++) {
    TxData_Std[i] = data[i];
  }
  for (uint8_t i = length; i < 8; i++) {
    TxData_Std[i] = 0x00;
  }

  uint32_t mailbox;
  HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan, &TxHeader_Std, TxData_Std, &mailbox);

  switch (result) {
    case HAL_OK:
      can_debug.hal_ok++;
      break;
    case HAL_ERROR:
      can_debug.hal_error++;
      can_debug.last_error_code = result;
      can_debug.last_error_mailbox = mailbox;
      break;
    case HAL_BUSY:
      can_debug.hal_busy++;
      can_debug.last_error_code = result;
      break;
    case HAL_TIMEOUT:
      can_debug.hal_timeout++;
      can_debug.last_error_code = result;
      break;
  }
}

/* --------------------------------------------------------------------------
 * Периодическая отправка состояния узла.
 *
 * У bxCAN всего 3 почтовых ящика передачи: ставим ровно столько кадров,
 * сколько ящиков сейчас свободно, по кругу — иначе лишние получают HAL_BUSY
 * и теряются (та же схема, что у ведущего).
 * -------------------------------------------------------------------------- */
void CanBus_TxTask(void)
{
  static uint8_t phase = 0;

  uint8_t free_mb = (uint8_t)HAL_CAN_GetTxMailboxesFreeLevel(&hcan);

  while (free_mb-- > 0) {
    if (phase == 0) {
      /* Состояние собственных дискретных входов (диагностика). */
      uint8_t data_comp[8];
      for (uint8_t i = 0; i < 8; i++) {
        data_comp[i] = IO_ReadPin(comp[i]);
      }
      CanBus_SendStd(CanBus_GenerateStdId(device_id, BASE_COMP, COMP_COUNT), data_comp, 8);
    } else {
      CanBus_SendDebugFrame();
    }
    phase = (uint8_t)((phase + 1) % 2);
  }
}

/* --------------------------------------------------------------------------
 * Телеметрия вентиляторов (BASE_DEBUG, поочерёдно один кадр за слот):
 *
 *   0x5A1 — сводка каналов:
 *     [0] вентилятор 1: биты 0-2 состояние (FanState), биты 3-5 авария
 *         (FanFault), бит 6 — EN драйвера поднят, бит 7 — ноль датчика тока
 *         откалиброван;
 *     [1] вентилятор 1: скважность, %;
 *     [2..3] вентилятор 1: средний ток, отсчёты АЦП от нуля датчика (LE16);
 *     [4..7] — то же для вентилятора 2.
 *
 *   0x5A2 — сырые каналы АЦП: ток DRV1, ток DRV2, температура 1,
 *     температура 2 (LE16).
 *
 *   0x5A3 — нули датчиков тока (LE16), счётчики срабатываний защиты и
 *     повторных пусков по каналам.
 *
 *   0x5A4 — тепловая защита и ограничитель частоты пусков:
 *     [0] флаги: бит 0 — тепловое отключение активно; биты 1/2 — датчик
 *         температуры 1/2 исправен; биты 3/4 — датчик 1/2 подтвердил
 *         перегрев; бит 5 — сырая команда из CAN; бит 6 — эффективная
 *         команда (после антидребезга и тепловой защиты);
 *     [1] счётчик пусков канала 1, [2] — канала 2;
 *     [3] биты 0/1 — канал 1/2 в паузе по частоте пусков.
 *
 * Пересчёт тока в амперы: I = (значение) / ACS724_ADC_PER_AMP.
 * -------------------------------------------------------------------------- */
static void CanBus_SendDebugFrame(void)
{
  static uint8_t dbg_sel = 0;

  FanStatus f1 = {0};
  FanStatus f2 = {0};
  Cooling_GetStatus(0, &f1);
  Cooling_GetStatus(1, &f2);

  switch (dbg_sel) {
    case 0: {
      uint8_t s1 = (uint8_t)((f1.state & 0x07) | ((f1.fault & 0x07) << 3) |
                             (f1.enabled ? 0x40 : 0) | (f1.zero_valid ? 0x80 : 0));
      uint8_t s2 = (uint8_t)((f2.state & 0x07) | ((f2.fault & 0x07) << 3) |
                             (f2.enabled ? 0x40 : 0) | (f2.zero_valid ? 0x80 : 0));
      uint8_t data_fan[8] = {
        s1, (uint8_t)f1.duty_pct,
        (uint8_t)(f1.i_avg_adc & 0xFF), (uint8_t)(f1.i_avg_adc >> 8),
        s2, (uint8_t)f2.duty_pct,
        (uint8_t)(f2.i_avg_adc & 0xFF), (uint8_t)(f2.i_avg_adc >> 8)
      };
      CanBus_SendStd(CanBus_GenerateStdId(device_id, BASE_DEBUG, 1), data_fan, 8);
      break;
    }
    case 1: {
      uint16_t i_1 = (uint16_t)ADS_RES_BUFFER[DRV1_CURRENT_IDX];
      uint16_t i_2 = (uint16_t)ADS_RES_BUFFER[DRV2_CURRENT_IDX];
      uint16_t t_1 = (uint16_t)ADS_RES_BUFFER[TEMP_SENS_1_IDX];
      uint16_t t_2 = (uint16_t)ADS_RES_BUFFER[TEMP_SENS_2_IDX];
      uint8_t data_adc[8] = {
        (uint8_t)(i_1 & 0xFF), (uint8_t)(i_1 >> 8),
        (uint8_t)(i_2 & 0xFF), (uint8_t)(i_2 >> 8),
        (uint8_t)(t_1 & 0xFF), (uint8_t)(t_1 >> 8),
        (uint8_t)(t_2 & 0xFF), (uint8_t)(t_2 >> 8)
      };
      CanBus_SendStd(CanBus_GenerateStdId(device_id, BASE_DEBUG, 2), data_adc, 8);
      break;
    }
    case 2: {
      uint8_t data_cal[8] = {
        (uint8_t)(f1.zero_adc & 0xFF), (uint8_t)(f1.zero_adc >> 8),
        (uint8_t)(f2.zero_adc & 0xFF), (uint8_t)(f2.zero_adc >> 8),
        (uint8_t)(f1.trips > 255 ? 255 : f1.trips),
        (uint8_t)(f2.trips > 255 ? 255 : f2.trips),
        f1.retries, f2.retries
      };
      CanBus_SendStd(CanBus_GenerateStdId(device_id, BASE_DEBUG, 3), data_cal, 8);
      break;
    }
    default: {
      CoolingStatus cs = {0};
      Cooling_GetGlobalStatus(&cs);

      uint8_t flags = (uint8_t)((cs.thermal_shutdown ? 0x01 : 0) |
                                (cs.temp_valid[0]    ? 0x02 : 0) |
                                (cs.temp_valid[1]    ? 0x04 : 0) |
                                (cs.temp_over[0]     ? 0x08 : 0) |
                                (cs.temp_over[1]     ? 0x10 : 0) |
                                (cs.cmd_raw          ? 0x20 : 0) |
                                (cs.cmd_active       ? 0x40 : 0));
      uint8_t data_th[4] = {
        flags,
        f1.start_count,
        f2.start_count,
        (uint8_t)((f1.cooldown ? 0x01 : 0) | (f2.cooldown ? 0x02 : 0))
      };
      CanBus_SendStd(CanBus_GenerateStdId(device_id, BASE_DEBUG, 4), data_th, 4);
      break;
    }
  }

  dbg_sel = (uint8_t)((dbg_sel + 1) % 4);
}

/* --------------------------------------------------------------------------
 * Приём кадров ведущего.
 * -------------------------------------------------------------------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
    return;
  }
  if (RxHeader.IDE == CAN_ID_EXT) {
    return;  /* расширенные кадры — обмен ведущего с VESC, не наш трафик */
  }

  uint32_t stdid           = RxHeader.StdId;
  uint8_t  std_device_id   = (stdid >> 8) & 0xFF;
  uint8_t  parameter_index = stdid & 0xFF;

  if (std_device_id != MASTER_DEVICE_ID) {
    return;
  }

  /* Кадр comp ведущего — снимок его дискретных входов. Байт COMP_DEADMAN
   * повторяет вход компаратора, по которому ведущий разрешает работу мотора
   * складывания и выдачу оборотов на VESC: активен — вентиляторы плавно
   * запускаются, снят — уходят в управляемый останов.                       */
  if (parameter_index == BASE_COMP + COMP_COUNT) {
    Cooling_SetCommand(RxData[COMP_DEADMAN]);
    can_last_control_tick = HAL_GetTick();
  }
}

/* --------------------------------------------------------------------------
 * Защита от потери связи с ведущим.
 * -------------------------------------------------------------------------- */
void CanBus_CheckTimeout(void)
{
  if ((HAL_GetTick() - can_last_control_tick) > CAN_CONTROL_TIMEOUT_MS) {
    /* Команду снимаем, но питание не рвём: останов идёт штатным путём —
     * плавный сброс скважности, выдержка на стекание индукции, снятие EN. */
    Cooling_SetCommand(0);
  }
}
