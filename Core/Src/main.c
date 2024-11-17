/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "jtag_state.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
#define SWD_ACK_OK    0
#define SWD_ACK_WAIT  1
#define SWD_ACK_FAULT 2

#define DP_REG_ADDR_DPIDR     0
#define DP_REG_ADDR_SELECT    8
#define DP_REG_ADDR_RDBUFF    0xc

#define DP_REG_ADDR_CTRL_STAT 0x04
#define DP_REG_ADDR_DLCR      0x14
#define DP_REG_ADDR_TARGETID  0x24
#define DP_REG_ADDR_DLPIDR    0x34

#define AP_REG_ADDR_CSW  0x00
#define AP_REG_ADDR_TAR  0x04
#define AP_REG_ADDR_DRW  0x0c
#define AP_REG_ADDR_CFG  0xf4
#define AP_REG_ADDR_BASE 0xf8
#define AP_REG_ADDR_IDR  0xfc

#define DP 0
#define AP 1

#define OP_READ 0
#define OP_WRITE 1


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TCLK_HI() \
  HAL_GPIO_WritePin(JTAG_TCK_GPIO_Port, JTAG_TCK_Pin, GPIO_PIN_SET)

#define TCLK_LO() \
  HAL_GPIO_WritePin(JTAG_TCK_GPIO_Port, JTAG_TCK_Pin, GPIO_PIN_RESET)

#define TCLK_TICK() \
  do { \
    TCLK_HI(); \
    RTCK_READ(); \
    HAL_Delay(100); \
    TCLK_LO(); \
    RTCK_READ(); \
    HAL_Delay(100); \
    num_ticks++; \
  } while(0)

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

volatile int num_ticks = 0;
volatile int tdo = 0;

jtag_state_t jtag_state;
int jtag_state_errors = 0;

void jtag_tms_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = JTAG_TMS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(JTAG_TMS_GPIO_Port, &GPIO_InitStruct);
}

void jtagtap_srst(bool assert)
{
  (void)assert;
#ifdef SRST_SET_VAL
  SRST_SET_VAL(assert);
  if(assert) {
  int i;
  for(i = 0; i < 10000; i++)
    asm volatile("nop");
  }
#endif
}

volatile uint32_t idcode = 0;

#define NUM_TRIES 1000

uint64_t tdo_history2 = 0;

uint8_t jtag_next(uint8_t tms, uint8_t databit)
{
  uint8_t tdo_value;
  if (tms)
    TMS_HI();
  else
    TMS_LO();

  if (databit)
    TDI_HI();
  else
    TDI_LO();

  tdo_value = TDO_READ();
  TCLK_HI();
  HAL_Delay(2);
  TCLK_LO();
  HAL_Delay(2);

  tdo_history2 = (tdo_history2 >> 1) | (((uint64_t)tdo_value) << 63);

  return tdo_value;
}

void jtag_tms_seq(uint32_t value, size_t len)
{
  for (size_t i = 0; i < len; ++i) {
    jtag_next(value & 1, 1);
    value >>= 1;
  }
}

void jtag_soft_reset(void)
{
  jtag_tms_seq(0b11111, 5);
  jtag_state = JTAG_STATE_TEST_LOGIC_RESET;
}

bool jtag_to_state(jtag_state_t to)
{
  const struct jtag_state_path *path;

  path = jtag_get_state_path(jtag_state, to);
  if (!path) {
    jtag_state_errors++;
    return false;
  }

  jtag_tms_seq(path->path, path->n);
  jtag_state = to;
  return true;
}

void jtag_shift_dr(void)
{
  jtag_to_state(JTAG_STATE_SHIFT_DR);
}

void jtag_shift_ir(void)
{
  jtag_to_state(JTAG_STATE_SHIFT_IR);
}


void jtag_shift_to_update_ir(void)
{
  jtag_tms_seq(0x03, 4);
}

void jtag_init(void)
{
  jtag_tms_init();
  for (int i = 0; i < 50; ++i)
    jtag_next(1, 0);

  jtag_tms_seq(0xe73c, 16);
  jtag_soft_reset();
}

void jtag_reset(void)
{
  TRST_LO();
  for (volatile int i = 0; i < 10000; ++i)
    asm("nop");
  TRST_HI();
  jtag_soft_reset();
}

volatile int last_ir = 0;

int irlen0 = 5;
int irlen1 = 4;

static int jtag_scan_num_devs(void)
{
  int num_devs;
  size_t i;
  size_t irchain_len = 4;

  jtag_shift_ir();

  /* Set to BYPASS mode */
  for (i = 0; i < irchain_len; ++i)
    jtag_next(0, 1);

  /* SHIFT-IR to SHIFT-DR */
  jtag_to_state(JTAG_STATE_SHIFT_DR);

  /* Set a lot of zeroes */
  for (i = 0; i < 500; ++i)
    jtag_next(0, 0);

  /* Expect delay of 2 clocks before seeing first 1 */
  for (i = 0; i < 500; ++i)
    if (jtag_next(0, 1))
      break;
  
  /* SHIFT-DR to Run-Test/Idle */
  jtag_soft_reset();
  num_devs = i;
  return num_devs;
}

static void jtag_shift(uint64_t value, uint64_t *out_value, size_t num_bits)
{
  size_t i;
  uint64_t out = 0;
  uint64_t out_bit;
  uint8_t tms;

  for (i = 0; i < num_bits; ++i) {
    uint8_t bit = (value & (1 << i)) ? 1 : 0;
    tms = (i == num_bits - 1) ? 1: 0;

    out_bit = jtag_next(tms, bit) & 1;
    if (tms)
      jtag_state++;

    out |= out_bit << i;
  }

  *out_value = out;
}

#define DPACC 0b1010
#define APACC 0b1011

struct dap {
  uint32_t idcode;
  uint32_t target_id;
  uint32_t dpidr;
  uint32_t dlpidr;
  uint32_t base;
  uint32_t idr;
  uint32_t ctl_stat;
  uint32_t select;
  uint32_t ir;
  uint32_t rom_table[1024];
};

struct dap dap = { 0 };

static void jtag_write_ir(uint8_t ir)
{
  uint64_t out_value = 0;

  if (dap.ir == ir)
    return;

  jtag_to_state(JTAG_STATE_SHIFT_IR);
  jtag_shift(ir, &out_value, 4);
  jtag_to_state(JTAG_STATE_RUN_TEST_IDLE);
  dap.ir = ir;
}

#define ACK_OK 0b010

static bool jtag_reg_write(int addr, uint32_t value)
{
  uint64_t dap_response;
  int ack;

  uint64_t dr_value = 0;

  /* Actual register value at [34:3] */
  dr_value |= ((uint64_t)value) << 3;

  /* A[3:2] of address value at [2:1] */
  dr_value |= ((addr >> 2) & 3) << 1;

  /* RnW is 0 for WRITE OP */

  jtag_to_state(JTAG_STATE_SHIFT_DR);
  jtag_shift(dr_value, &dap_response, 35);
  jtag_to_state(JTAG_STATE_SHIFT_DR);

  dr_value = ((0xc >> 2) & 3) << 1;
  dr_value |= 1;

  jtag_shift(dr_value, &dap_response, 35);
  ack = dap_response & 7;

  return ack == ACK_OK;
}


static bool jtag_set_select_reg(int dpbank, int apbank, int apsel)
{
  uint64_t select = (dpbank & 0xf)
    | ((apbank & 0xf) << 4)
    | ((apsel & 0xf) << 24);

  if (select == dap.select)
    return true;

  jtag_write_ir(DPACC);

  if (jtag_reg_write(DP_REG_ADDR_SELECT, select)) {
    dap.select = select;
    return true;
  }

  return false;
}

static void jtag_shift_to_rdbuf(uint64_t *out)
{
  uint64_t dr_value_in;
  uint64_t dr_value_out;

  jtag_set_select_reg(0, 0, 0);
  jtag_write_ir(DPACC);
  jtag_to_state(JTAG_STATE_SHIFT_DR);
  dr_value_in = ((0xc >> 2) & 3) << 1;
  dr_value_in |= 1;
  jtag_shift(dr_value_in, &dr_value_out, 35);
  *out = dr_value_out;
}

#define ADI_READ_BIT (1<<0)
#define ADI_WRITE_BIT (0<<0)
#define ADI_ADDR_BITS(__a) (((__a >> 2) & 3) << 1)
#define ADI_MSG_LEN 35
#define ADI_RESP_STATUS_MASK 7
#define ADI_RESP2DATA(__r) (uint32_t)(__r >> 3)

static inline bool jtag_read_rdbuf_resp(uint32_t *out)
{
  uint64_t dap_response;
  uint64_t dr_value;
  int ack;

  jtag_write_ir(DPACC);
  dr_value = ADI_ADDR_BITS(DP_REG_ADDR_RDBUFF) | ADI_READ_BIT;

  while(1) {
    jtag_to_state(JTAG_STATE_SHIFT_DR);
    jtag_shift(dr_value, &dap_response, ADI_MSG_LEN);
    ack = dap_response & ADI_RESP_STATUS_MASK;
    if (ack == ACK_OK) {
      *out = ADI_RESP2DATA(dap_response);
      return true;
    }
  }
  return false;
}

static bool jtag_adi_io_shift(int op, int addr, uint32_t value_in,
  uint32_t *value_out)
{
  uint64_t dap_response;

  uint64_t dr_value = (op == OP_READ ? ADI_READ_BIT : ADI_WRITE_BIT)
    | ADI_ADDR_BITS(addr)
    | (((uint64_t)(op == OP_WRITE ? value_in : 0)) << 3);

  jtag_to_state(JTAG_STATE_SHIFT_DR);
  jtag_shift(dr_value, &dap_response, 35);
  return jtag_read_rdbuf_resp(value_out);
}

static bool jtag_adi_io(int type, int op, int reg_addr, uint32_t value_in,
  uint32_t *value_out)
{
  uint32_t v = 0;
  int ap_bank = 0;
  int dp_bank = 0;

  int addr = reg_addr & 0xf;
  if (type == DP)
    dp_bank = (reg_addr >> 4) & 0xf;
  else
    ap_bank = (reg_addr >> 4) & 0xf;

  if (!jtag_set_select_reg(dp_bank, ap_bank, 0))
    return false;

  if (type == DP)
    jtag_write_ir(DPACC);
  else
    jtag_write_ir(APACC);

  if (!jtag_adi_io_shift(op, addr, value_in, &v))
    return false;

  *value_out = v;
  return true;
}

static bool jtag_read_dpacc_ctr_stat(uint32_t *out)
{
  return jtag_adi_io(DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, out);
}

static bool jtag_write_dpacc_ctr_stat(uint32_t value)
{
  /* WRITE to SELECT */
  if (!jtag_set_select_reg(0, 0, 0))
    return false;

  jtag_write_ir(DPACC);
  return jtag_reg_write(DP_REG_ADDR_CTRL_STAT, value);
}

#define CTRL_STAT_CDBGRSTREQ   (1<<26)
#define CTRL_STAT_CDBGRSTACK   (1<<27)
#define CTRL_STAT_CDBGPWRUPREQ (1<<28)
#define CTRL_STAT_CDBGPWRUPACK (1<<29)
#define CTRL_STAT_CSYSPWRUPREQ (1<<30)
#define CTRL_STAT_CSYSPWRUPACK (1<<31)

#define CDBGRSTACK_NUM_REPS 5000

static void dap_init(void)
{
  dap.select = 0xffffffff;
  dap.ir = 0xffffffff;
}

static bool jtag_dap_reset(void)
{
  size_t i;
  bool timeout = true;

  if (!jtag_write_dpacc_ctr_stat(dap.ctl_stat | CTRL_STAT_CDBGRSTREQ))
    return false;

  for (i = 0; i < CDBGRSTACK_NUM_REPS; ++i) {
    if (!jtag_read_dpacc_ctr_stat(&dap.ctl_stat))
      return false;

    if (dap.ctl_stat & CTRL_STAT_CDBGRSTACK) {
      timeout = false;
      break;
    }
  }

  if (timeout)
    return false;

  if (!jtag_write_dpacc_ctr_stat(dap.ctl_stat & ~CTRL_STAT_CDBGRSTREQ))
    return false;

  for (i = 0; i < CDBGRSTACK_NUM_REPS; ++i) {
    if (!jtag_read_dpacc_ctr_stat(&dap.ctl_stat))
      return false;

    if (!(dap.ctl_stat & CTRL_STAT_CDBGRSTACK))
      return true;
  }

  return false;
}

static bool jtag_dap_dbg_power_up(void)
{
  if (!jtag_write_dpacc_ctr_stat(dap.ctl_stat |
    CTRL_STAT_CDBGPWRUPREQ | CTRL_STAT_CSYSPWRUPREQ))
    return false;

  while (1) {
    if (!jtag_read_dpacc_ctr_stat(&dap.ctl_stat))
      return false;

    if (dap.ctl_stat & CTRL_STAT_CDBGPWRUPACK)
      break;
  }

#if 0
  if (!jtag_write_dpacc_ctr_stat(dap.ctl_stat | CTRL_STAT_CSYSPWRUPREQ))
    return false;

  while(1) {
    if (!jtag_read_dpacc_ctr_stat(&dap.ctl_stat))
      return false;

    if (dap.ctl_stat & CTRL_STAT_CSYSPWRUPACK)
      break;
  }
#endif
  return true;
}

static bool jtag_dap_dbg_power_down(void)
{
  const uint32_t req_mask = CTRL_STAT_CSYSPWRUPREQ;// | CTRL_STAT_CDBGPWRUPREQ;
  const uint32_t ack_mask = CTRL_STAT_CSYSPWRUPACK;// | CTRL_STAT_CDBGPWRUPACK;

  if (!jtag_write_dpacc_ctr_stat(0))
    return false;

  while (1) {
    if (!jtag_read_dpacc_ctr_stat(&dap.ctl_stat))
      return false;

    if (!(dap.ctl_stat & ack_mask))
      break;
  }

  return true;
}

static bool jtag_dap_power_init(void)
{
  jtag_write_ir(DPACC);

  if (!jtag_read_dpacc_ctr_stat(&dap.ctl_stat))
    return false;

  if (!jtag_dap_dbg_power_down())
    return false;

  if (!jtag_dap_reset())
    return false;

  if (!jtag_dap_dbg_power_up())
    return false;
  
  return true;
}

uint32_t jtag_read_idcode(void)
{
  // jtag_shift_ir();
  jtag_shift_dr();
  dap.idcode = 0;
  for (int i = 0; i < 32; ++i) {
    last_ir = jtag_next(0, 0);
    dap.idcode |= last_ir << i;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  int num_devs = 0;

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  dap_init();
  jtag_init();
  jtag_reset();

  // arm_test();

  num_devs = jtag_scan_num_devs();

  jtag_read_idcode();

  bool success;
  uint32_t reg;
  success = jtag_dap_power_init();
  success = jtag_adi_io(DP, OP_READ, DP_REG_ADDR_TARGETID, 0, &dap.target_id);
  success = jtag_adi_io(AP, OP_READ, AP_REG_ADDR_BASE, 0, &dap.base);
  success = jtag_adi_io(DP, OP_READ, DP_REG_ADDR_DPIDR, 0, &dap.dpidr);
  success = jtag_adi_io(DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &dap.ctl_stat);
  success = jtag_adi_io(DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &dap.ctl_stat);
  success = jtag_adi_io(DP, OP_READ, DP_REG_ADDR_DLPIDR, 0, &dap.dlpidr);
  success = jtag_adi_io(DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &dap.ctl_stat);
  success = jtag_adi_io(DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &dap.ctl_stat);
  success = jtag_adi_io(AP, OP_READ, AP_REG_ADDR_IDR, 0, &dap.idr);
  success = jtag_adi_io(DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &dap.ctl_stat);
  success = jtag_adi_io(AP, OP_READ, AP_REG_ADDR_CFG, 0, &reg);
  success = jtag_adi_io(DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &dap.ctl_stat);

  uint32_t tar_value = 0;
  uint32_t data;

  for (int i = 0; i < 1024; ++i) {
    tar_value = dap.base + i * 4;
    jtag_adi_io(AP, OP_WRITE, AP_REG_ADDR_TAR, tar_value, &data);
    jtag_adi_io(AP, OP_READ, AP_REG_ADDR_DRW, 0, &dap.rom_table[i]);
  }

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, JTAG_TDI_Pin|JTAG_TMS_Pin|JTAG_TCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(JTAG_RST_GPIO_Port, JTAG_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : JTAG_RTCK_Pin */
  GPIO_InitStruct.Pin = JTAG_RTCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JTAG_RTCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin JTAG_TDO_Pin */
  GPIO_InitStruct.Pin = B1_Pin|JTAG_TDO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : JTAG_TDI_Pin JTAG_TMS_Pin JTAG_TCK_Pin */
  GPIO_InitStruct.Pin = JTAG_TDI_Pin|JTAG_TMS_Pin|JTAG_TCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : JTAG_RST_Pin */
  GPIO_InitStruct.Pin = JTAG_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(JTAG_RST_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
