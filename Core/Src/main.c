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
volatile int rtck = 0;


#define delay_us(__n) HAL_Delay(__n)
#define swclk_lo() TCLK_LO()
#define swclk_hi() TCLK_HI()
#define swdio_lo() TMS_LO()
#define swdio_hi() TMS_HI()

jtag_state_t jtag_state;
int jtag_state_errors = 0;

void clock_pulse()
{
    swclk_lo();
    delay_us(1);
    swclk_hi();
    delay_us(1);
}

void swdio_as_output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = JTAG_TMS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(JTAG_TMS_GPIO_Port, &GPIO_InitStruct);
}

void jtag_tms_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = JTAG_TMS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(JTAG_TMS_GPIO_Port, &GPIO_InitStruct);
}

void swdio_as_input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = JTAG_TMS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(JTAG_TMS_GPIO_Port, &GPIO_InitStruct);
}

void swd_write_bit(uint8_t bit)
{
    swdio_as_output();

    if (bit)
        swdio_hi();
    else
        swdio_lo();

    clock_pulse();
}

bool swdio_read(void)
{
  return HAL_GPIO_ReadPin(JTAG_TMS_GPIO_Port, JTAG_TMS_Pin);
}

uint8_t swd_read_bit(void)
{
  swdio_as_input();
  swclk_lo();
  delay_us(1);
  uint8_t bit = swdio_read();
  swclk_hi();
  delay_us(1);
  return bit;
}

void swd_line_reset(void)
{
    for (int i = 0; i < 50; i++)
    {
        swdio_hi();
        clock_pulse();
    }
}


#define SWDIO_MODE_FLOAT() gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, \
                                           GPIO_PUPD_NONE, SWDIO_PIN);

#define SWDIO_MODE_DRIVE() gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, \
                                           GPIO_PUPD_NONE, SWDIO_PIN);

static void swd_turnaround(uint8_t dir)
{
  static uint8_t olddir = 0;

  if(dir == olddir)
    return;

  olddir = dir;

  if(dir)
    swdio_as_input();

  swclk_hi();
  swclk_lo();

  if(!dir)
    swdio_as_output();
}

static void swd_bit_out(uint8_t val)
{
  if (val)
    swdio_hi();
  else
    swdio_lo();
  swclk_hi();
  swclk_lo();
}


void swd_reset(void)
{
  swd_turnaround(0);
  /* 50 clocks with TMS high */
  for(int i = 0; i < 50; i++)
    swd_bit_out(1);
}

void swd_seq_out(uint32_t bits, int num)
{
  swd_turnaround(0);

  while(num--) {
    swd_bit_out(bits & 1);
    bits >>= 1;
  }
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

static bool swd_bit_in(void)
{
  bool res;
  res = swdio_read();
  swclk_hi();
  swclk_lo();
  return res;
}

uint32_t swd_seq_in(int num)
{
  uint32_t index = 1;
  uint32_t ret = 0;

  swd_turnaround(1);

  while (num--) {
    if (swd_bit_in())
      ret |= index;
    index <<= 1;
  }

  return ret;
}

uint8_t swd_seq_in_parity(uint32_t *ret, int num)
{
  uint32_t index = 1;
  uint8_t parity = 0;
  *ret = 0;

  swd_turnaround(1);

  while (num--) {
    if (swd_bit_in()) {
      *ret |= index;
      parity ^= 1;
    }
    index <<= 1;
  }

  if (swd_bit_in())
    parity ^= 1;

  return parity;
}

volatile uint32_t idcode = 0;
volatile uint32_t idcode2 = 0;

#define SWD_PARITY_BIT 0x20
#define NUM_TRIES 1000

void swd_seq_out_parity(uint32_t value, int num_bits)
{
  uint8_t parity = 0;

  swd_turnaround(0);

  for (int i = 0; i < num_bits; ++i) {
    swd_bit_out(value & 1);
    parity ^= value;
    value >>= 1;
  }
  swd_bit_out(parity & 1);
}

static uint32_t swd_low_access(bool is_ap, bool is_read_op, uint8_t addr,
  uint32_t value)
{
  uint8_t request = 0x81;
  uint32_t response;
  uint8_t ack;

#if 0
  if(is_ap && dp->fault)
    return 0;
#endif

  if (is_ap)
    request ^= (0x02 | SWD_PARITY_BIT);

  if (is_read_op)
    request ^= (0x04 | SWD_PARITY_BIT);

  addr &= 0xC;
  request |= (addr << 1) & 0x18;

  if(addr == 4 || addr == 8)
    request ^= SWD_PARITY_BIT;

  for (int i = 0; i < NUM_TRIES; ++i) {
    swd_seq_out(request, 8);
    ack = swd_seq_in(3);
    if (ack != SWD_ACK_WAIT)
      break;
  };

  if (ack == SWD_ACK_WAIT)
    return -1;

  if (ack == SWD_ACK_FAULT)
    return -1;

  if (ack != SWD_ACK_OK)
    return -1;

  if (is_read_op) {
    /* Give up on parity error */
    if (swd_seq_in_parity(&response, 32))
      return -1;
  } else
    swd_seq_out_parity(value, 32);

// /* REMOVE THIS */
//  swd_seq_out(0, 8);
  return response;
}

uint8_t jtag_next(uint8_t tms, uint8_t databit)
{
  if (tms)
    TMS_HI();
  else
    TMS_LO();

  if (databit)
    TDI_HI();
  else
    TDI_LO();

  TCLK_HI();
  tdo = TDO_READ();

  rtck = RTCK_READ();
  HAL_Delay(1);
  TCLK_LO();
  rtck = RTCK_READ();

  return tdo != 0;
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
  jtag_tms_seq(0b011111, 6);
  jtag_state = JTAG_STATE_RUN_TEST_IDLE;
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


// swd-to-jtag switch sequence
void swd_init(void)
{
  swd_reset();
  int ack;
  uint32_t code;

  /* 0b0111100111100111 */
  swd_seq_out(0xe79e, 16);

  swd_reset();

  swd_seq_out(0, 16);
  jtagtap_srst(true);
  /* 1010 0101 -> 10100101 */
  /*              ^^^^^^^^
                  ||||||||
                  ||||||||
                  |||||||Park
                  ||||||Stop
                  |||||Parity
                  |||A[2:3]
                  ||RnW (Read)
                  |APDP (DP)
                  Start
                  */
  swd_seq_out(0xa5, 8);
  ack = swd_seq_in(3);
  if (ack != 1 || swd_seq_in_parity(&code, 32))
  {
    while(1);
  }
  else
  {
    idcode = code;
    while(1);
  }

}

volatile int last_ir = 0;

volatile int vvv = 0;

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
  uint8_t tms;

  for (i = 0; i < num_bits; ++i) {
    uint8_t bit = (value & (1 << i)) ? 1 : 0;
    if (i == num_bits - 1)
      tms = 1;
    else
      tms = 0;

    uint8_t out_bit = jtag_next(tms, bit) & 1;
    if (tms)
      jtag_state++;

    out |= (out_bit << i);
  }

  *out_value = out;
}

#define DPACC 0b1010
static void jtag_write_ir(uint8_t ir)
{
  uint64_t out_value = 0;

  jtag_to_state(JTAG_STATE_SHIFT_IR);
  jtag_shift(ir, &out_value, 4);
  jtag_to_state(JTAG_STATE_RUN_TEST_IDLE);
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

#define DAP_REG_ADDR_CTRL_STAT 4
#define DAP_REG_ADDR_SELECT    8

uint32_t last_select = 0;

static bool jtag_reg_read(int addr, uint32_t *value)
{
  uint64_t dap_response;
  int ack;

  uint64_t dr_value = 0;

  dr_value |= ((addr >> 2) & 3) << 1;
  dr_value |= 1;

  jtag_to_state(JTAG_STATE_SHIFT_DR);
  jtag_shift(dr_value, &dap_response, 35);

  jtag_to_state(JTAG_STATE_SHIFT_DR);

  dr_value = ((0xc >> 2) & 3) << 1;
  dr_value |= 1;
  jtag_shift(dr_value, &dap_response, 35);
  ack = dap_response & 7;
  if (ack == ACK_OK) {
    *value = (uint32_t)(dap_response >> 3);
    return true;
  }

  return false;
}

static bool jtag_set_select_reg(int dpbank, int apbank, int apsel)
{
  uint64_t select = (dpbank & 0xf)
    | ((apbank & 0xf) << 4)
    | ((apsel & 0xf) << 24);

  return jtag_reg_write(DAP_REG_ADDR_SELECT, select);
}

static bool jtag_read_dpacc_ctr_stat(uint32_t *out)
{
  uint32_t v;

  /* WRITE to SELECT */
  if (!jtag_set_select_reg(0, 0, 0))
    return false;

  if (!jtag_reg_read(DAP_REG_ADDR_CTRL_STAT, &v))
    return false;

  *out = v;
  return true;
}

static bool jtag_write_dpacc_ctr_stat(uint32_t value)
{
  /* WRITE to SELECT */
  if (!jtag_set_select_reg(0, 0, 0))
    return false;

  return jtag_reg_write(DAP_REG_ADDR_CTRL_STAT, value);
}

char testreg[4];
int testreg_idx = 0;

#define TESTREG_SHIFT(__bit) \
  testreg[testreg_idx++] = jtag_next(0, __bit); \
  if (testreg_idx == 4) \
    testreg_idx = 0; 

#if 0
  TESTREG_SHIFT(1);
  TESTREG_SHIFT(1);
  TESTREG_SHIFT(1);
  TESTREG_SHIFT(1);

  TESTREG_SHIFT(0);
  TESTREG_SHIFT(1);
  TESTREG_SHIFT(0);
  TESTREG_SHIFT(1);

  TESTREG_SHIFT(0);
  TESTREG_SHIFT(0);
  TESTREG_SHIFT(0);
  TESTREG_SHIFT(1);
#endif


#define CTRL_STAT_CDBGRSTREQ   (1<<26)
#define CTRL_STAT_CDBGRSTACK   (1<<27)
#define CTRL_STAT_CDBGPWRUPREQ (1<<28)
#define CTRL_STAT_CDBGPWRUPACK (1<<29)
#define CTRL_STAT_CSYSPWRUPREQ (1<<30)
#define CTRL_STAT_CSYSPWRUPACK (1<<31)

struct dap {
  uint32_t ctl_stat;
};

struct dap dap = { 0 };

#define CDBGRSTACK_NUM_REPS 5000

static bool jtag_dap_reset(void)
{
  size_t i;
  const int max_reps = 5000;
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
  if (!jtag_write_dpacc_ctr_stat(dap.ctl_stat | CTRL_STAT_CDBGPWRUPREQ))
    return false;

  while (1) {
    if (!jtag_read_dpacc_ctr_stat(&dap.ctl_stat))
      return false;

    if (dap.ctl_stat & CTRL_STAT_CDBGPWRUPACK)
      break;
  }

  if (!jtag_write_dpacc_ctr_stat(dap.ctl_stat | CTRL_STAT_CSYSPWRUPREQ))
    return false;

  while(1) {
    if (!jtag_read_dpacc_ctr_stat(&dap.ctl_stat))
      return false;

    if (dap.ctl_stat & CTRL_STAT_CSYSPWRUPACK)
      break;
  }
  return true;
}

static bool jtag_dap_dbg_power_down(void)
{
  const uint32_t req_mask = CTRL_STAT_CSYSPWRUPREQ;// | CTRL_STAT_CDBGPWRUPREQ;
  const uint32_t ack_mask = CTRL_STAT_CSYSPWRUPACK;// | CTRL_STAT_CDBGPWRUPACK;

  if (!jtag_write_dpacc_ctr_stat(dap.ctl_stat & ~req_mask))
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

uint64_t ctl_stat = 0;

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
  while(vvv);
  const struct jtag_state_path *p = jtag_get_state_path(JTAG_STATE_EXIT1_DR, JTAG_STATE_EXIT1_IR);
  if (!p) {
    vvv = 1;
  }
  jtag_init();
  jtag_reset();

  num_devs = jtag_scan_num_devs();

  jtag_shift_dr();

  for (int i = 0; i < 32; ++i) {
    last_ir = jtag_next(0, 0);
    idcode |= last_ir << i;
  }

  ctl_stat = jtag_dap_power_init();

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
