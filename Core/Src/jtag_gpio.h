#pragma once
#include "main.h"

#define TCLK_HI() \
  HAL_GPIO_WritePin(JTAG_TCK_GPIO_Port, JTAG_TCK_Pin, GPIO_PIN_SET)

#define TCLK_LO() \
  HAL_GPIO_WritePin(JTAG_TCK_GPIO_Port, JTAG_TCK_Pin, GPIO_PIN_RESET)

#define TRST_HI() \
  HAL_GPIO_WritePin(JTAG_RST_GPIO_Port, JTAG_RST_Pin, GPIO_PIN_SET)

#define TRST_LO() \
  HAL_GPIO_WritePin(JTAG_RST_GPIO_Port, JTAG_RST_Pin, GPIO_PIN_RESET)

#define TDI_HI() \
  HAL_GPIO_WritePin(JTAG_TDI_GPIO_Port, JTAG_TDI_Pin, GPIO_PIN_SET)

#define TDI_LO() \
  HAL_GPIO_WritePin(JTAG_TDI_GPIO_Port, JTAG_TDI_Pin, GPIO_PIN_RESET)

#define TDO_READ() \
    HAL_GPIO_ReadPin(JTAG_TDO_GPIO_Port, JTAG_TDO_Pin)

#define RTCK_READ() \
    HAL_GPIO_ReadPin(JTAG_RTCK_GPIO_Port, JTAG_RTCK_Pin)

#define TMS_HI() \
  HAL_GPIO_WritePin(JTAG_TMS_GPIO_Port, JTAG_TMS_Pin, GPIO_PIN_SET)

#define TMS_LO() \
  HAL_GPIO_WritePin(JTAG_TMS_GPIO_Port, JTAG_TMS_Pin, GPIO_PIN_RESET)

#define TCLK_TICK() \
  do { \
    TCLK_HI(); \
    RTCK_READ(); \
    HAL_Delay(100); \
    TCLK_LO(); \
    RTCK_READ(); \
    HAL_Delay(100); \
  } while(0)

static inline void jtag_tms_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = JTAG_TMS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(JTAG_TMS_GPIO_Port, &GPIO_InitStruct);
}
