/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
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
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_cdc_bridge.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_CDC_RX_PACKET_SIZE   64U
#define USB_CDC_TX_CHUNK_SIZE    63U
#define USB_CDC_RX_RING_SIZE     1024U
#define USB_CDC_TX_RING_SIZE     8192U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static UX_SLAVE_CLASS_CDC_ACM *g_cdc_acm = UX_NULL;

static UCHAR g_rx_packet[USB_CDC_RX_PACKET_SIZE];
static ULONG g_rx_actual = 0U;

static uint8_t g_rx_ring[USB_CDC_RX_RING_SIZE];
static volatile uint32_t g_rx_head = 0U;
static volatile uint32_t g_rx_tail = 0U;

static uint8_t g_tx_ring[USB_CDC_TX_RING_SIZE];
static volatile uint32_t g_tx_head = 0U;
static volatile uint32_t g_tx_tail = 0U;

static UCHAR g_tx_packet[USB_CDC_TX_CHUNK_SIZE];
static ULONG g_tx_length = 0U;
static ULONG g_tx_actual = 0U;
static uint8_t g_tx_active = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static uint32_t USB_CDC_RingCount(volatile uint32_t head,
                                  volatile uint32_t tail,
                                  uint32_t size);
static uint32_t USB_CDC_RingFree(volatile uint32_t head,
                                 volatile uint32_t tail,
                                 uint32_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t USB_CDC_RingCount(volatile uint32_t head,
                                  volatile uint32_t tail,
                                  uint32_t size)
{
  if (head >= tail)
  {
    return head - tail;
  }

  return size - tail + head;
}

static uint32_t USB_CDC_RingFree(volatile uint32_t head,
                                 volatile uint32_t tail,
                                 uint32_t size)
{
  return (size - 1U) - USB_CDC_RingCount(head, tail, size);
}

uint8_t USB_CDC_IsReady(void)
{
  return (g_cdc_acm != UX_NULL) ? 1U : 0U;
}

uint8_t USB_CDC_QueueBytes(const uint8_t *data, uint32_t length)
{
  uint32_t i;

  if ((data == NULL) || (length == 0U) || (g_cdc_acm == UX_NULL))
  {
    return 0U;
  }

  if (USB_CDC_RingFree(g_tx_head, g_tx_tail, USB_CDC_TX_RING_SIZE) < length)
  {
    return 0U;
  }

  for (i = 0U; i < length; i++)
  {
    g_tx_ring[g_tx_head] = data[i];
    g_tx_head++;
    if (g_tx_head >= USB_CDC_TX_RING_SIZE)
    {
      g_tx_head = 0U;
    }
  }

  return 1U;
}

uint8_t USB_CDC_QueueText(const char *text)
{
  if (text == NULL)
  {
    return 0U;
  }

  return USB_CDC_QueueBytes((const uint8_t *)text, (uint32_t)strlen(text));
}

uint8_t USB_CDC_ReadByte(uint8_t *byte)
{
  if ((byte == NULL) || (g_rx_tail == g_rx_head))
  {
    return 0U;
  }

  *byte = g_rx_ring[g_rx_tail];
  g_rx_tail++;
  if (g_rx_tail >= USB_CDC_RX_RING_SIZE)
  {
    g_rx_tail = 0U;
  }

  return 1U;
}

void USB_CDC_Service(void)
{
  UINT state;
  uint32_t i;

  /* USBX standalone mode must be serviced continuously. */
  ux_device_stack_tasks_run();

  if (g_cdc_acm == UX_NULL)
  {
    g_tx_active = 0U;
    return;
  }

  /* PC -> MCU */
  g_rx_actual = 0U;
  state = ux_device_class_cdc_acm_read_run(g_cdc_acm,
                                            g_rx_packet,
                                            USB_CDC_RX_PACKET_SIZE,
                                            &g_rx_actual);

  if (state == UX_STATE_NEXT)
  {
    for (i = 0U; i < (uint32_t)g_rx_actual; i++)
    {
      uint32_t next = g_rx_head + 1U;
      if (next >= USB_CDC_RX_RING_SIZE)
      {
        next = 0U;
      }

      if (next == g_rx_tail)
      {
        /* RX ring full: discard remaining bytes in this USB packet. */
        break;
      }

      g_rx_ring[g_rx_head] = g_rx_packet[i];
      g_rx_head = next;
    }
  }

  /* MCU -> PC. Use <=63 byte chunks so every transfer is a short packet. */
  if (g_tx_active == 0U)
  {
    uint32_t available = USB_CDC_RingCount(g_tx_head, g_tx_tail,
                                           USB_CDC_TX_RING_SIZE);

    if (available != 0U)
    {
      g_tx_length = (available > USB_CDC_TX_CHUNK_SIZE) ?
                    USB_CDC_TX_CHUNK_SIZE : available;

      for (i = 0U; i < (uint32_t)g_tx_length; i++)
      {
        g_tx_packet[i] = g_tx_ring[g_tx_tail];
        g_tx_tail++;
        if (g_tx_tail >= USB_CDC_TX_RING_SIZE)
        {
          g_tx_tail = 0U;
        }
      }

      g_tx_active = 1U;
      g_tx_actual = 0U;
    }
  }

  if (g_tx_active != 0U)
  {
    state = ux_device_class_cdc_acm_write_run(g_cdc_acm,
                                               g_tx_packet,
                                               g_tx_length,
                                               &g_tx_actual);

    if (state == UX_STATE_NEXT)
    {
      g_tx_active = 0U;
      g_tx_length = 0U;
    }
    else if ((state == UX_STATE_EXIT) || (state == UX_STATE_ERROR))
    {
      g_tx_active = 0U;
      g_tx_length = 0U;
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
  g_cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;
  g_rx_head = 0U;
  g_rx_tail = 0U;
  g_tx_head = 0U;
  g_tx_tail = 0U;
  g_tx_active = 0U;
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  g_cdc_acm = UX_NULL;
  g_tx_active = 0U;
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
