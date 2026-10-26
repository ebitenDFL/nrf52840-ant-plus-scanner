/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * @file           : main.c

  * @brief          : Main program body

  ******************************************************************************

  * @attention

  *

  * Copyright (c) 2026 STMicroelectronics.

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
#include "app_usbx_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_cdc_bridge.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LSM6DSV32X_WHO_AM_I_REG   0x0FU

#define LSM6DSV32X_WHO_AM_I_VAL   0x70U

#define LSM6DSV32X_IF_CFG         0x03U

#define LSM6DSV32X_I2C_I3C_DISABLE 0x01U

#define LSM6DSV32X_CTRL1          0x10U

#define LSM6DSV32X_CTRL2          0x11U

#define LSM6DSV32X_CTRL3          0x12U

#define LSM6DSV32X_CTRL6          0x15U

#define LSM6DSV32X_CTRL8          0x17U

#define LSM6DSV32X_OUTX_L_G       0x22U

/* 240 Hz, high-performance; accel +/-4 g; gyro +/-500 dps */

#define IMU_CTRL1_240HZ            0x07U

#define IMU_CTRL2_240HZ            0x07U

#define IMU_CTRL3_BDU_IFINC        0x44U

#define IMU_CTRL6_500DPS           0x02U

#define IMU_CTRL8_4G               0x04U

#define ACCEL_G_PER_LSB            0.000122f

#define GYRO_DPS_PER_LSB           0.0175f

/* Hall wheel-speed measurement */
#define HALL_TIMER_TARGET_HZ       10000U
#define HALL_TIMEOUT_MS            5000U

/* Steering ADC */
#define STEER_ADC_VDDA_MV           3300U
#define STEER_ADC_FULL_SCALE        4095U
#define STEER_SAMPLE_PERIOD_MS      10U

/* IMU and stream scheduling */
#define IMU_READ_PERIOD_MS          5U
#define STREAM_PERIOD_MS            20U

/* USB command parser */
#define USB_CMD_BUFFER_SIZE         96U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim3;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */

volatile uint8_t imu1_who_am_i = 0x00U;

volatile uint8_t imu2_who_am_i = 0x00U;

volatile HAL_StatusTypeDef imu1_spi_status = HAL_ERROR;

volatile HAL_StatusTypeDef imu2_spi_status = HAL_ERROR;

volatile uint8_t imu1_init_ok = 0U;

volatile uint8_t imu2_init_ok = 0U;

volatile int16_t imu1_gx_raw = 0;

volatile int16_t imu1_gy_raw = 0;

volatile int16_t imu1_gz_raw = 0;

volatile int16_t imu1_ax_raw = 0;

volatile int16_t imu1_ay_raw = 0;

volatile int16_t imu1_az_raw = 0;

volatile float imu1_gx_dps = 0.0f;

volatile float imu1_gy_dps = 0.0f;

volatile float imu1_gz_dps = 0.0f;

volatile float imu1_ax_g = 0.0f;

volatile float imu1_ay_g = 0.0f;

volatile float imu1_az_g = 0.0f;

volatile int16_t imu2_gx_raw = 0;

volatile int16_t imu2_gy_raw = 0;

volatile int16_t imu2_gz_raw = 0;

volatile int16_t imu2_ax_raw = 0;

volatile int16_t imu2_ay_raw = 0;

volatile int16_t imu2_az_raw = 0;

volatile float imu2_gx_dps = 0.0f;

volatile float imu2_gy_dps = 0.0f;

volatile float imu2_gz_dps = 0.0f;

volatile float imu2_ax_g = 0.0f;

volatile float imu2_ay_g = 0.0f;

volatile float imu2_az_g = 0.0f;

/* Hall sensor live/debug values */
volatile float hall1_hz = 0.0f;
volatile float hall2_hz = 0.0f;

volatile uint32_t hall1_period_counts = 0U;
volatile uint32_t hall2_period_counts = 0U;

volatile uint32_t hall1_edge_count = 0U;
volatile uint32_t hall2_edge_count = 0U;

volatile uint32_t hall1_last_capture = 0U;
volatile uint32_t hall2_last_capture = 0U;

volatile uint32_t hall1_last_edge_ms = 0U;
volatile uint32_t hall2_last_edge_ms = 0U;

volatile uint8_t hall1_have_previous = 0U;
volatile uint8_t hall2_have_previous = 0U;

volatile uint32_t tim3_counter_hz = 0U;

/* Steering ADC */
volatile uint32_t steer_adc_raw = 0U;
volatile uint32_t steer_adc_mv = 0U;

/* CAN / USB bridge diagnostics */
volatile uint32_t can_rx_count = 0U;
volatile uint32_t can_tx_count = 0U;
volatile uint32_t can_tx_error_count = 0U;
volatile uint32_t usb_command_count = 0U;
volatile HAL_StatusTypeDef can_start_status = HAL_ERROR;

/* Boot / USB diagnostics */
volatile uint32_t boot_stage = 0U;
volatile UINT usbx_init_status = UX_ERROR;

static uint8_t can_started = 0U;
static uint8_t usb_slcan_mode = 0U;       /* 0 = sensor stream, 1 = strict SLCAN */
static uint8_t slcan_timestamp_enable = 0U;

static char usb_cmd_buffer[USB_CMD_BUFFER_SIZE];
static uint32_t usb_cmd_length = 0U;

static uint32_t last_imu_read_ms = 0U;
static uint32_t last_steer_read_ms = 0U;
static uint32_t last_stream_ms = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

static HAL_StatusTypeDef IMU_ReadReg(GPIO_TypeDef *cs_port, uint16_t cs_pin,

                                     uint8_t reg, uint8_t *value);

static HAL_StatusTypeDef IMU_WriteReg(GPIO_TypeDef *cs_port, uint16_t cs_pin,

                                      uint8_t reg, uint8_t value);

static HAL_StatusTypeDef IMU_Init(GPIO_TypeDef *cs_port, uint16_t cs_pin,

                                  volatile uint8_t *who_am_i);

static HAL_StatusTypeDef IMU_ReadMotion(GPIO_TypeDef *cs_port, uint16_t cs_pin,

                                        volatile int16_t *gx_raw,

                                        volatile int16_t *gy_raw,

                                        volatile int16_t *gz_raw,

                                        volatile int16_t *ax_raw,

                                        volatile int16_t *ay_raw,

                                        volatile int16_t *az_raw,

                                        volatile float *gx_dps,

                                        volatile float *gy_dps,

                                        volatile float *gz_dps,

                                        volatile float *ax_g,

                                        volatile float *ay_g,

                                        volatile float *az_g);

static HAL_StatusTypeDef HALL_Init(void);

static HAL_StatusTypeDef CAN_Bridge_Start(void);
static void CAN_Bridge_Stop(void);
static void CAN_ServiceRx(void);
static HAL_StatusTypeDef CAN_SendFrame(uint32_t identifier,
                                       uint8_t extended,
                                       uint8_t remote,
                                       const uint8_t *data,
                                       uint8_t dlc);

static void Steering_ADC_Service(uint32_t now_ms);
static void Sensor_Stream_Service(uint32_t now_ms);
static void USB_Command_Service(void);
static void SLCAN_ExecuteCommand(const char *cmd);
static uint8_t ParseHex(const char *text, uint32_t digits, uint32_t *value);
static uint32_t DLC_To_HAL(uint8_t dlc);
static uint8_t HAL_DLC_To_Bytes(uint32_t dlc);
static char HexNibble(uint8_t value);
static void CAN_FormatSLCAN(const FDCAN_RxHeaderTypeDef *header,
                            const uint8_t *data,
                            char *out,
                            uint32_t out_size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef IMU_ReadReg(GPIO_TypeDef *cs_port, uint16_t cs_pin,

                                     uint8_t reg, uint8_t *value)

{

  uint8_t tx[2] = {(uint8_t)(reg | 0x80U), 0x00U};

  uint8_t rx[2] = {0U, 0U};

  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

  status = HAL_SPI_TransmitReceive(&hspi4, tx, rx, 2U, 100U);

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

  if ((status == HAL_OK) && (value != NULL))

  {

    *value = rx[1];

  }

  return status;

}

static HAL_StatusTypeDef IMU_WriteReg(GPIO_TypeDef *cs_port, uint16_t cs_pin,

                                      uint8_t reg, uint8_t value)

{

  uint8_t tx[2] = {(uint8_t)(reg & 0x7FU), value};

  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

  status = HAL_SPI_Transmit(&hspi4, tx, 2U, 100U);

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

  return status;

}

static HAL_StatusTypeDef IMU_Init(GPIO_TypeDef *cs_port, uint16_t cs_pin,

                                  volatile uint8_t *who_am_i)

{

  uint8_t who = 0U;

  uint8_t verify = 0U;

  if (IMU_ReadReg(cs_port, cs_pin, LSM6DSV32X_WHO_AM_I_REG, &who) != HAL_OK)

    return HAL_ERROR;

  *who_am_i = who;

  if (who != LSM6DSV32X_WHO_AM_I_VAL)

    return HAL_ERROR;

  /*

   * Important for multiple LSM6DSV32X devices on the same SPI bus:

   * disable the I2C/I3C interface while this device is selected.

   * IF_CFG (0x03), bit 0 = I2C_I3C_disable.

   */

  if (IMU_WriteReg(cs_port, cs_pin,

                   LSM6DSV32X_IF_CFG,

                   LSM6DSV32X_I2C_I3C_DISABLE) != HAL_OK)

    return HAL_ERROR;

  if (IMU_WriteReg(cs_port, cs_pin, LSM6DSV32X_CTRL3, IMU_CTRL3_BDU_IFINC) != HAL_OK)

    return HAL_ERROR;

  if (IMU_WriteReg(cs_port, cs_pin, LSM6DSV32X_CTRL8, IMU_CTRL8_4G) != HAL_OK)

    return HAL_ERROR;

  if (IMU_WriteReg(cs_port, cs_pin, LSM6DSV32X_CTRL6, IMU_CTRL6_500DPS) != HAL_OK)

    return HAL_ERROR;

  if (IMU_WriteReg(cs_port, cs_pin, LSM6DSV32X_CTRL1, IMU_CTRL1_240HZ) != HAL_OK)

    return HAL_ERROR;

  if (IMU_WriteReg(cs_port, cs_pin, LSM6DSV32X_CTRL2, IMU_CTRL2_240HZ) != HAL_OK)

    return HAL_ERROR;

  HAL_Delay(20U);

  if (IMU_ReadReg(cs_port, cs_pin, LSM6DSV32X_CTRL1, &verify) != HAL_OK)

    return HAL_ERROR;

  if (verify != IMU_CTRL1_240HZ)

    return HAL_ERROR;

  return HAL_OK;

}

static HAL_StatusTypeDef IMU_ReadMotion(GPIO_TypeDef *cs_port, uint16_t cs_pin,

                                        volatile int16_t *gx_raw,

                                        volatile int16_t *gy_raw,

                                        volatile int16_t *gz_raw,

                                        volatile int16_t *ax_raw,

                                        volatile int16_t *ay_raw,

                                        volatile int16_t *az_raw,

                                        volatile float *gx_dps,

                                        volatile float *gy_dps,

                                        volatile float *gz_dps,

                                        volatile float *ax_g,

                                        volatile float *ay_g,

                                        volatile float *az_g)

{

  uint8_t tx[13] = {0U};

  uint8_t rx[13] = {0U};

  HAL_StatusTypeDef status;

  int16_t gx, gy, gz, ax, ay, az;

  tx[0] = (uint8_t)(LSM6DSV32X_OUTX_L_G | 0x80U);

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

  status = HAL_SPI_TransmitReceive(&hspi4, tx, rx, 13U, 100U);

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

  if (status != HAL_OK)

    return status;

  gx = (int16_t)((uint16_t)rx[1]  | ((uint16_t)rx[2]  << 8));

  gy = (int16_t)((uint16_t)rx[3]  | ((uint16_t)rx[4]  << 8));

  gz = (int16_t)((uint16_t)rx[5]  | ((uint16_t)rx[6]  << 8));

  ax = (int16_t)((uint16_t)rx[7]  | ((uint16_t)rx[8]  << 8));

  ay = (int16_t)((uint16_t)rx[9]  | ((uint16_t)rx[10] << 8));

  az = (int16_t)((uint16_t)rx[11] | ((uint16_t)rx[12] << 8));

  *gx_raw = gx;

  *gy_raw = gy;

  *gz_raw = gz;

  *ax_raw = ax;

  *ay_raw = ay;

  *az_raw = az;

  *gx_dps = (float)gx * GYRO_DPS_PER_LSB;

  *gy_dps = (float)gy * GYRO_DPS_PER_LSB;

  *gz_dps = (float)gz * GYRO_DPS_PER_LSB;

  *ax_g = (float)ax * ACCEL_G_PER_LSB;

  *ay_g = (float)ay * ACCEL_G_PER_LSB;

  *az_g = (float)az * ACCEL_G_PER_LSB;

  return HAL_OK;

}

static HAL_StatusTypeDef HALL_Init(void)
{
  RCC_ClkInitTypeDef clk = {0};
  uint32_t flash_latency = 0U;
  uint32_t pclk1_hz;
  uint32_t tim3_input_hz;
  uint32_t divider;

  /*
   * TIM3 is on APB1. If APB1 is prescaled, the timer clock is normally
   * twice PCLK1; otherwise it is equal to PCLK1.
   */
  HAL_RCC_GetClockConfig(&clk, &flash_latency);
  pclk1_hz = HAL_RCC_GetPCLK1Freq();

  if (clk.APB1CLKDivider == RCC_HCLK_DIV1)
  {
    tim3_input_hz = pclk1_hz;
  }
  else
  {
    tim3_input_hz = pclk1_hz * 2U;
  }

  /* Choose a prescaler that gives approximately 10 kHz timer ticks. */
  divider = (tim3_input_hz + (HALL_TIMER_TARGET_HZ / 2U)) / HALL_TIMER_TARGET_HZ;
  if (divider == 0U)
  {
    divider = 1U;
  }

  __HAL_TIM_DISABLE(&htim3);
  __HAL_TIM_SET_PRESCALER(&htim3, divider - 1U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);

  /* Force the new prescaler value to load immediately. */
  htim3.Instance->EGR = TIM_EGR_UG;

  tim3_counter_hz = tim3_input_hz / divider;

  hall1_hz = 0.0f;
  hall2_hz = 0.0f;
  hall1_have_previous = 0U;
  hall2_have_previous = 0U;

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}


static char HexNibble(uint8_t value)
{
  value &= 0x0FU;
  return (value < 10U) ? (char)('0' + value) : (char)('A' + (value - 10U));
}

static uint8_t ParseHex(const char *text, uint32_t digits, uint32_t *value)
{
  uint32_t result = 0U;
  uint32_t i;

  if ((text == NULL) || (value == NULL))
  {
    return 0U;
  }

  for (i = 0U; i < digits; i++)
  {
    char c = text[i];
    uint8_t n;

    if ((c >= '0') && (c <= '9'))
    {
      n = (uint8_t)(c - '0');
    }
    else if ((c >= 'A') && (c <= 'F'))
    {
      n = (uint8_t)(c - 'A' + 10);
    }
    else if ((c >= 'a') && (c <= 'f'))
    {
      n = (uint8_t)(c - 'a' + 10);
    }
    else
    {
      return 0U;
    }

    result = (result << 4) | n;
  }

  *value = result;
  return 1U;
}

static uint32_t DLC_To_HAL(uint8_t dlc)
{
  switch (dlc)
  {
    case 0: return FDCAN_DLC_BYTES_0;
    case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2;
    case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4;
    case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6;
    case 7: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
  }
}

static uint8_t HAL_DLC_To_Bytes(uint32_t dlc)
{
  switch (dlc)
  {
    case FDCAN_DLC_BYTES_0: return 0U;
    case FDCAN_DLC_BYTES_1: return 1U;
    case FDCAN_DLC_BYTES_2: return 2U;
    case FDCAN_DLC_BYTES_3: return 3U;
    case FDCAN_DLC_BYTES_4: return 4U;
    case FDCAN_DLC_BYTES_5: return 5U;
    case FDCAN_DLC_BYTES_6: return 6U;
    case FDCAN_DLC_BYTES_7: return 7U;
    default: return 8U;
  }
}

static HAL_StatusTypeDef CAN_Bridge_Start(void)
{
  if (can_started != 0U)
  {
    return HAL_OK;
  }

  HAL_GPIO_WritePin(FDCAN_STBY_GPIO_Port, FDCAN_STBY_Pin, GPIO_PIN_RESET);

  /*
   * No explicit ID filters are configured. Accept every non-matching
   * standard and extended frame into FIFO0, including remote frames.
   */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  can_started = 1U;
  return HAL_OK;
}

static void CAN_Bridge_Stop(void)
{
  if (can_started != 0U)
  {
    (void)HAL_FDCAN_Stop(&hfdcan1);
    can_started = 0U;
  }
}

static HAL_StatusTypeDef CAN_SendFrame(uint32_t identifier,
                                       uint8_t extended,
                                       uint8_t remote,
                                       const uint8_t *data,
                                       uint8_t dlc)
{
  FDCAN_TxHeaderTypeDef tx = {0};

  if ((can_started == 0U) || (dlc > 8U))
  {
    return HAL_ERROR;
  }

  tx.Identifier = identifier;
  tx.IdType = (extended != 0U) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  tx.TxFrameType = (remote != 0U) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  tx.DataLength = DLC_To_HAL(dlc);
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch = FDCAN_BRS_OFF;
  tx.FDFormat = FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker = 0U;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
                                    &tx,
                                    (uint8_t *)data) != HAL_OK)
  {
    can_tx_error_count++;
    return HAL_ERROR;
  }

  can_tx_count++;
  return HAL_OK;
}

static void CAN_FormatSLCAN(const FDCAN_RxHeaderTypeDef *header,
                            const uint8_t *data,
                            char *out,
                            uint32_t out_size)
{
  uint8_t dlc;
  uint32_t pos = 0U;
  uint32_t id_digits;
  uint32_t shift;
  uint8_t i;
  uint16_t ts;

  if ((header == NULL) || (out == NULL) || (out_size < 8U))
  {
    return;
  }

  dlc = HAL_DLC_To_Bytes(header->DataLength);

  if (header->IdType == FDCAN_EXTENDED_ID)
  {
    out[pos++] = (header->RxFrameType == FDCAN_REMOTE_FRAME) ? 'R' : 'T';
    id_digits = 8U;
  }
  else
  {
    out[pos++] = (header->RxFrameType == FDCAN_REMOTE_FRAME) ? 'r' : 't';
    id_digits = 3U;
  }

  for (i = 0U; i < id_digits; i++)
  {
    shift = (id_digits - 1U - i) * 4U;
    out[pos++] = HexNibble((uint8_t)(header->Identifier >> shift));
  }

  out[pos++] = HexNibble(dlc);

  if (header->RxFrameType == FDCAN_DATA_FRAME)
  {
    for (i = 0U; (i < dlc) && ((pos + 2U) < out_size); i++)
    {
      out[pos++] = HexNibble((uint8_t)(data[i] >> 4));
      out[pos++] = HexNibble(data[i]);
    }
  }

  if ((slcan_timestamp_enable != 0U) && ((pos + 4U) < out_size))
  {
    ts = (uint16_t)(HAL_GetTick() & 0xFFFFU);
    out[pos++] = HexNibble((uint8_t)(ts >> 12));
    out[pos++] = HexNibble((uint8_t)(ts >> 8));
    out[pos++] = HexNibble((uint8_t)(ts >> 4));
    out[pos++] = HexNibble((uint8_t)ts);
  }

  out[pos++] = '\r';
  out[pos] = '\0';
}

static void CAN_ServiceRx(void)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t data[8];
  char frame[48];
  char line[80];

  if (can_started == 0U)
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0U)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1,
                               FDCAN_RX_FIFO0,
                               &rx_header,
                               data) != HAL_OK)
    {
      break;
    }

    can_rx_count++;
    CAN_FormatSLCAN(&rx_header, data, frame, sizeof(frame));

    if (usb_slcan_mode != 0U)
    {
      (void)USB_CDC_QueueText(frame);
    }
    else
    {
      int n = snprintf(line, sizeof(line), "C,%lu,%s\n",
                       (unsigned long)HAL_GetTick(), frame);
      if (n > 0)
      {
        (void)USB_CDC_QueueBytes((const uint8_t *)line, (uint32_t)n);
      }
    }
  }
}

static void Steering_ADC_Service(uint32_t now_ms)
{
  if ((now_ms - last_steer_read_ms) < STEER_SAMPLE_PERIOD_MS)
  {
    return;
  }

  last_steer_read_ms = now_ms;

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK)
  {
    steer_adc_raw = HAL_ADC_GetValue(&hadc1);
    steer_adc_mv = (steer_adc_raw * STEER_ADC_VDDA_MV +
                    (STEER_ADC_FULL_SCALE / 2U)) /
                   STEER_ADC_FULL_SCALE;
  }

  (void)HAL_ADC_Stop(&hadc1);
}

static void Sensor_Stream_Service(uint32_t now_ms)
{
  char line[180];
  int n;

  int32_t i1_ax_mg, i1_ay_mg, i1_az_mg;
  int32_t i2_ax_mg, i2_ay_mg, i2_az_mg;
  int32_t i1_gx_mdps, i1_gy_mdps, i1_gz_mdps;
  int32_t i2_gx_mdps, i2_gy_mdps, i2_gz_mdps;
  uint32_t h1_mhz, h2_mhz;

  if ((usb_slcan_mode != 0U) ||
      ((now_ms - last_stream_ms) < STREAM_PERIOD_MS))
  {
    return;
  }

  last_stream_ms = now_ms;

  /* Integer engineering units avoid requiring printf floating-point support. */
  i1_ax_mg = ((int32_t)imu1_ax_raw * 122) / 1000;
  i1_ay_mg = ((int32_t)imu1_ay_raw * 122) / 1000;
  i1_az_mg = ((int32_t)imu1_az_raw * 122) / 1000;

  i2_ax_mg = ((int32_t)imu2_ax_raw * 122) / 1000;
  i2_ay_mg = ((int32_t)imu2_ay_raw * 122) / 1000;
  i2_az_mg = ((int32_t)imu2_az_raw * 122) / 1000;

  i1_gx_mdps = ((int32_t)imu1_gx_raw * 175) / 10;
  i1_gy_mdps = ((int32_t)imu1_gy_raw * 175) / 10;
  i1_gz_mdps = ((int32_t)imu1_gz_raw * 175) / 10;

  i2_gx_mdps = ((int32_t)imu2_gx_raw * 175) / 10;
  i2_gy_mdps = ((int32_t)imu2_gy_raw * 175) / 10;
  i2_gz_mdps = ((int32_t)imu2_gz_raw * 175) / 10;

  h1_mhz = (uint32_t)((hall1_hz * 1000.0f) + 0.5f);
  h2_mhz = (uint32_t)((hall2_hz * 1000.0f) + 0.5f);

  n = snprintf(line, sizeof(line),
               "I1,%lu,AXmg=%ld,AYmg=%ld,AZmg=%ld,GXmdps=%ld,GYmdps=%ld,GZmdps=%ld\r\n",
               (unsigned long)now_ms,
               (long)i1_ax_mg, (long)i1_ay_mg, (long)i1_az_mg,
               (long)i1_gx_mdps, (long)i1_gy_mdps, (long)i1_gz_mdps);
  if (n > 0)
  {
    (void)USB_CDC_QueueBytes((const uint8_t *)line, (uint32_t)n);
  }

  n = snprintf(line, sizeof(line),
               "I2,%lu,AXmg=%ld,AYmg=%ld,AZmg=%ld,GXmdps=%ld,GYmdps=%ld,GZmdps=%ld\r\n",
               (unsigned long)now_ms,
               (long)i2_ax_mg, (long)i2_ay_mg, (long)i2_az_mg,
               (long)i2_gx_mdps, (long)i2_gy_mdps, (long)i2_gz_mdps);
  if (n > 0)
  {
    (void)USB_CDC_QueueBytes((const uint8_t *)line, (uint32_t)n);
  }

  n = snprintf(line, sizeof(line),
               "W,%lu,H1mHz=%lu,H2mHz=%lu\r\n",
               (unsigned long)now_ms,
               (unsigned long)h1_mhz,
               (unsigned long)h2_mhz);
  if (n > 0)
  {
    (void)USB_CDC_QueueBytes((const uint8_t *)line, (uint32_t)n);
  }

  n = snprintf(line, sizeof(line),
               "S,%lu,ADC=%lu,MV=%lu\r\n",
               (unsigned long)now_ms,
               (unsigned long)steer_adc_raw,
               (unsigned long)steer_adc_mv);
  if (n > 0)
  {
    (void)USB_CDC_QueueBytes((const uint8_t *)line, (uint32_t)n);
  }
}

static void SLCAN_ExecuteCommand(const char *cmd)
{
  uint32_t id = 0U;
  uint32_t tmp = 0U;
  uint8_t data[8] = {0U};
  uint8_t dlc = 0U;
  uint8_t extended = 0U;
  uint8_t remote = 0U;
  uint32_t id_digits;
  uint32_t i;
  uint32_t expected_length;
  size_t len;

  if ((cmd == NULL) || (cmd[0] == '\0'))
  {
    return;
  }

  usb_command_count++;
  len = strlen(cmd);

  switch (cmd[0])
  {
    case 'S':
      /* Fixed firmware bitrate: S6 = 500 kbit/s. */
      if ((len == 2U) && (cmd[1] == '6'))
      {
        (void)USB_CDC_QueueText("\r");
      }
      else
      {
        (void)USB_CDC_QueueText("\a");
      }
      return;

    case 'O':
      if (CAN_Bridge_Start() == HAL_OK)
      {
        usb_slcan_mode = 1U;
        (void)USB_CDC_QueueText("\r");
      }
      else
      {
        (void)USB_CDC_QueueText("\a");
      }
      return;

    case 'C':
      CAN_Bridge_Stop();
      (void)USB_CDC_QueueText("\r");
      return;

    case 'Z':
      if ((len == 2U) && ((cmd[1] == '0') || (cmd[1] == '1')))
      {
        slcan_timestamp_enable = (cmd[1] == '1') ? 1U : 0U;
        (void)USB_CDC_QueueText("\r");
      }
      else
      {
        (void)USB_CDC_QueueText("\a");
      }
      return;

    case 'V':
      (void)USB_CDC_QueueText("V0101\r");
      return;

    case 'v':
      (void)USB_CDC_QueueText("v0101\r");
      return;

    case 'N':
      (void)USB_CDC_QueueText("N0001\r");
      return;

    case 'F':
      (void)USB_CDC_QueueText("F00\r");
      return;

    case 'X':
      /* Custom extension: X0 = strict SLCAN, X1 = sensor/CAN stream. */
      if ((len == 2U) && (cmd[1] == '0'))
      {
        usb_slcan_mode = 1U;
        (void)USB_CDC_QueueText("\r");
      }
      else if ((len == 2U) && (cmd[1] == '1'))
      {
        usb_slcan_mode = 0U;
        if (CAN_Bridge_Start() == HAL_OK)
        {
          (void)USB_CDC_QueueText("\r");
        }
        else
        {
          (void)USB_CDC_QueueText("\a");
        }
      }
      else
      {
        (void)USB_CDC_QueueText("\a");
      }
      return;

    case 't':
    case 'T':
    case 'r':
    case 'R':
      extended = ((cmd[0] == 'T') || (cmd[0] == 'R')) ? 1U : 0U;
      remote = ((cmd[0] == 'r') || (cmd[0] == 'R')) ? 1U : 0U;
      id_digits = (extended != 0U) ? 8U : 3U;

      if (len < (size_t)(1U + id_digits + 1U))
      {
        (void)USB_CDC_QueueText("\a");
        return;
      }

      if (ParseHex(&cmd[1], id_digits, &id) == 0U)
      {
        (void)USB_CDC_QueueText("\a");
        return;
      }

      if (ParseHex(&cmd[1U + id_digits], 1U, &tmp) == 0U)
      {
        (void)USB_CDC_QueueText("\a");
        return;
      }

      dlc = (uint8_t)tmp;
      if (dlc > 8U)
      {
        (void)USB_CDC_QueueText("\a");
        return;
      }

      if ((extended == 0U) && (id > 0x7FFU))
      {
        (void)USB_CDC_QueueText("\a");
        return;
      }

      if ((extended != 0U) && (id > 0x1FFFFFFFU))
      {
        (void)USB_CDC_QueueText("\a");
        return;
      }

      expected_length = 1U + id_digits + 1U;
      if (remote == 0U)
      {
        expected_length += ((uint32_t)dlc * 2U);
      }

      if (len != (size_t)expected_length)
      {
        (void)USB_CDC_QueueText("\a");
        return;
      }

      if (remote == 0U)
      {
        for (i = 0U; i < dlc; i++)
        {
          if (ParseHex(&cmd[1U + id_digits + 1U + (i * 2U)],
                       2U, &tmp) == 0U)
          {
            (void)USB_CDC_QueueText("\a");
            return;
          }
          data[i] = (uint8_t)tmp;
        }
      }

      if (CAN_SendFrame(id, extended, remote, data, dlc) == HAL_OK)
      {
        (void)USB_CDC_QueueText("\r");
      }
      else
      {
        (void)USB_CDC_QueueText("\a");
      }
      return;

    default:
      (void)USB_CDC_QueueText("\a");
      return;
  }
}

static void USB_Command_Service(void)
{
  uint8_t byte;

  while (USB_CDC_ReadByte(&byte) != 0U)
  {
    if ((byte == '\r') || (byte == '\n'))
    {
      if (usb_cmd_length != 0U)
      {
        usb_cmd_buffer[usb_cmd_length] = '\0';
        SLCAN_ExecuteCommand(usb_cmd_buffer);
        usb_cmd_length = 0U;
      }
      continue;
    }

    if (usb_cmd_length < (USB_CMD_BUFFER_SIZE - 1U))
    {
      usb_cmd_buffer[usb_cmd_length++] = (char)byte;
    }
    else
    {
      usb_cmd_length = 0U;
      (void)USB_CDC_QueueText("\a");
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_SPI4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  /* USBX is intentionally started later, after IMU/Hall/CAN startup. */
  /* USER CODE BEGIN 2 */

  boot_stage = 20U;

  /* Both devices share SPI4, so both CS lines must idle HIGH. */
  HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU2_CS_GPIO_Port, IMU2_CS_Pin, GPIO_PIN_SET);

  HAL_Delay(100U);

  imu1_spi_status = IMU_Init(IMU1_CS_GPIO_Port, IMU1_CS_Pin, &imu1_who_am_i);
  imu1_init_ok = (imu1_spi_status == HAL_OK) ? 1U : 0U;

  imu2_spi_status = IMU_Init(IMU2_CS_GPIO_Port, IMU2_CS_Pin, &imu2_who_am_i);
  imu2_init_ok = (imu2_spi_status == HAL_OK) ? 1U : 0U;

  boot_stage = 30U;

  /* Start TIM3 input capture for both Hall sensors. */
  if (HALL_Init() != HAL_OK)
  {
    boot_stage = 31U;
    Error_Handler();
  }

  boot_stage = 40U;

  /*
   * Bring USB up BEFORE CAN.
   * CAN startup must never prevent the USB diagnostic/interface from appearing.
   */
  usbx_init_status = MX_USBX_Device_Init();

  if (usbx_init_status == UX_SUCCESS)
  {
    boot_stage = 50U;
  }
  else
  {
    boot_stage = 90U;
  }

  /*
   * Start CAN at 500 kbit/s, but keep CAN failure non-fatal.
   * can_start_status is visible in Live Expressions.
   */
  can_start_status = CAN_Bridge_Start();

  if ((usbx_init_status == UX_SUCCESS) && (can_start_status == HAL_OK))
  {
    boot_stage = 100U;
  }
  else if ((usbx_init_status == UX_SUCCESS) && (can_start_status != HAL_OK))
  {
    boot_stage = 60U;   /* USB good, CAN start failed */
  }

  last_imu_read_ms = HAL_GetTick();
  last_steer_read_ms = HAL_GetTick();
  last_stream_ms = HAL_GetTick();

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_BLUE);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)

  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t now_ms = HAL_GetTick();

    /* Standalone USBX needs frequent background service, but only if init succeeded. */
    if (usbx_init_status == UX_SUCCESS)
    {
      USB_CDC_Service();
      USB_Command_Service();
    }

    /* Keep draining CAN even if USB is not available. */
    CAN_ServiceRx();

    /* Read both IMUs at ~200 Hz; sensor ODR is 240 Hz. */
    if ((now_ms - last_imu_read_ms) >= IMU_READ_PERIOD_MS)
    {
      last_imu_read_ms = now_ms;

      if (imu1_init_ok != 0U)
      {
        imu1_spi_status = IMU_ReadMotion(IMU1_CS_GPIO_Port, IMU1_CS_Pin,
                                         &imu1_gx_raw, &imu1_gy_raw, &imu1_gz_raw,
                                         &imu1_ax_raw, &imu1_ay_raw, &imu1_az_raw,
                                         &imu1_gx_dps, &imu1_gy_dps, &imu1_gz_dps,
                                         &imu1_ax_g, &imu1_ay_g, &imu1_az_g);
      }

      if (imu2_init_ok != 0U)
      {
        imu2_spi_status = IMU_ReadMotion(IMU2_CS_GPIO_Port, IMU2_CS_Pin,
                                         &imu2_gx_raw, &imu2_gy_raw, &imu2_gz_raw,
                                         &imu2_ax_raw, &imu2_ay_raw, &imu2_az_raw,
                                         &imu2_gx_dps, &imu2_gy_dps, &imu2_gz_dps,
                                         &imu2_ax_g, &imu2_ay_g, &imu2_az_g);
      }
    }

    Steering_ADC_Service(now_ms);

    /* Hall stop timeout. */
    if ((hall1_have_previous != 0U) &&
        ((now_ms - hall1_last_edge_ms) > HALL_TIMEOUT_MS))
    {
      hall1_hz = 0.0f;
      hall1_period_counts = 0U;
      hall1_have_previous = 0U;
    }

    if ((hall2_have_previous != 0U) &&
        ((now_ms - hall2_last_edge_ms) > HALL_TIMEOUT_MS))
    {
      hall2_hz = 0.0f;
      hall2_period_counts = 0U;
      hall2_have_previous = 0U;
    }

    if (usbx_init_status == UX_SUCCESS)
    {
      Sensor_Stream_Service(now_ms);
    }

    if ((imu1_init_ok != 0U) &&
        (imu2_init_ok != 0U) &&
        (imu1_spi_status == HAL_OK) &&
        (imu2_spi_status == HAL_OK))
    {
      BSP_LED_On(LED_GREEN);
      BSP_LED_Off(LED_RED);
    }
    else
    {
      BSP_LED_Off(LED_GREEN);
      BSP_LED_On(LED_RED);
    }

    /* Give USBX another chance after queuing stream/CAN data. */
    if (usbx_init_status == UX_SUCCESS)
    {
      USB_CDC_Service();
    }

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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the System Power Supply
  */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable Epod Booster
  */
  if (HAL_RCCEx_EpodBoosterClkConfig(RCC_EPODBOOSTER_SOURCE_MSIS, RCC_EPODBOOSTER_DIV1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_PWREx_EnableEpodBooster() != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set Flash latency before increasing MSIS
  */
  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_3);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_MSIS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSISState = RCC_MSI_ON;
  RCC_OscInitStruct.MSISSource = RCC_MSI_RC0;
  RCC_OscInitStruct.MSISDiv = RCC_MSI_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSIS;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /* Synchronize HSI48 to USB SOF, as in ST's USB CDC reference setup. */
  __HAL_RCC_CRS_CLK_ENABLE();

  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue =
      __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U);
  RCC_CRSInitStruct.ErrorLimitValue = 34U;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32U;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_46CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 12;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x7;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi4.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi4.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi4, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /*
   * USB packet memory allocation (PMA).
   * These addresses match ST's official NUCLEO-U3C5ZI-Q
   * Ux_Device_CDC_ACM reference application.
   *
   * EP0 OUT  0x00 -> 0x014
   * EP0 IN   0x80 -> 0x054
   * CDC IN   0x81 -> 0x094
   * CDC OUT  0x01 -> 0x0D4
   * CDC CMD  0x82 -> 0x114
   */
  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x00, PCD_SNG_BUF, 0x014);
  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x80, PCD_SNG_BUF, 0x054);
  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x81, PCD_SNG_BUF, 0x094);
  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x01, PCD_SNG_BUF, 0x0D4);
  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x82, PCD_SNG_BUF, 0x114);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, IMU1_CS_Pin|IMU2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FDCAN_STBY_GPIO_Port, FDCAN_STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IMU1_CS_Pin IMU2_CS_Pin */
  GPIO_InitStruct.Pin = IMU1_CS_Pin|IMU2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : VSENSE_Pin */
  GPIO_InitStruct.Pin = VSENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VSENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_DET_Pin */
  GPIO_InitStruct.Pin = VBUS_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FDCAN_STBY_Pin */
  GPIO_InitStruct.Pin = FDCAN_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FDCAN_STBY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint32_t capture;
  uint32_t delta;

  if (htim->Instance != TIM3)
  {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    hall1_edge_count++;

    if (hall1_have_previous != 0U)
    {
      /* 16-bit free-running timer: unsigned wrap subtraction handles one rollover. */
      delta = (uint16_t)(capture - hall1_last_capture);

      if (delta != 0U)
      {
        hall1_period_counts = delta;
        hall1_hz = (float)tim3_counter_hz / (float)delta;
      }
    }

    hall1_last_capture = capture;
    hall1_last_edge_ms = HAL_GetTick();
    hall1_have_previous = 1U;
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    hall2_edge_count++;

    if (hall2_have_previous != 0U)
    {
      delta = (uint16_t)(capture - hall2_last_capture);

      if (delta != 0U)
      {
        hall2_period_counts = delta;
        hall2_hz = (float)tim3_counter_hz / (float)delta;
      }
    }

    hall2_last_capture = capture;
    hall2_last_edge_ms = HAL_GetTick();
    hall2_have_previous = 1U;
  }
}

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
#ifdef USE_FULL_ASSERT
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
