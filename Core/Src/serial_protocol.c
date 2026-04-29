/**
  ******************************************************************************
  * @file           : serial_protocol.c
  * @brief          : FireWater serial protocol implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "serial_protocol.h"
#include "usart.h"
#include "dma.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define SERIAL_TX_BUFFER_SIZE    256

/* Private variables ---------------------------------------------------------*/
static uint8_t g_tx_buffer[SERIAL_TX_BUFFER_SIZE];
static uint16_t g_tx_length = 0;

/* Private function prototypes -----------------------------------------------*/
static void SerialProtocol_BuildFrame(
  uint8_t cmd,
  const uint8_t *data,
  uint8_t data_len,
  uint8_t *frame_buffer,
  uint16_t *frame_size);

/* Public function definitions -----------------------------------------------*/

void SerialProtocol_Init(void)
{
  /* USART2 is already initialized via MX_USART2_UART_Init() */
  /* DMA is already configured for USART2 TX/RX */
  memset(g_tx_buffer, 0, SERIAL_TX_BUFFER_SIZE);
}

uint8_t SerialProtocol_CalcCRC8(const uint8_t *data, uint16_t length)
{
  uint8_t crc = 0;
  
  for (uint16_t i = 0; i < length; i++)
  {
    crc ^= data[i];
  }
  
  return crc;
}

uint16_t SerialProtocol_CreateFrame(
  uint8_t cmd,
  const uint8_t *data,
  uint8_t data_len,
  uint8_t *frame_buffer)
{
  if (frame_buffer == NULL || data_len > FIREWATER_MAX_PAYLOAD)
  {
    return 0;
  }

  uint16_t frame_size = 0;
  
  /* Add header */
  frame_buffer[frame_size++] = FIREWATER_HEADER_BYTE1;
  frame_buffer[frame_size++] = FIREWATER_HEADER_BYTE2;
  
  /* Add length */
  frame_buffer[frame_size++] = data_len + 1;  /* +1 for command byte */
  
  /* Add command */
  frame_buffer[frame_size++] = cmd;
  
  /* Add payload */
  if (data != NULL && data_len > 0)
  {
    memcpy(&frame_buffer[frame_size], data, data_len);
    frame_size += data_len;
  }
  
  /* Calculate and add CRC8 */
  uint8_t crc = SerialProtocol_CalcCRC8(&frame_buffer[2], frame_size - 2);
  frame_buffer[frame_size++] = crc;
  
  return frame_size;
}

uint8_t SerialProtocol_Send(const uint8_t *data, uint16_t length)
{
  if (data == NULL || length == 0 || length > SERIAL_TX_BUFFER_SIZE)
  {
    return 0;
  }

  /* Copy data to TX buffer */
  memcpy(g_tx_buffer, data, length);
  g_tx_length = length;
  
  /* Send via DMA (using USART2) */
  HAL_UART_Transmit_DMA(&huart2, g_tx_buffer, length);
  
  return 1;
}

void SerialProtocol_SendHello(void)
{
  const char *hello_msg = "hello\r\n";
  SerialProtocol_Send((const uint8_t *)hello_msg, 7);
}

uint16_t SerialProtocol_SendMotorStatus(
  uint8_t state,
  uint16_t speed_rpm,
  uint8_t sector,
  uint8_t pwm_duty)
{
  uint8_t payload[5];
  uint8_t frame_buffer[FIREWATER_FRAME_MAX_SIZE];
  uint16_t frame_size;
  
  /* Build payload: [STATE:1B] [SPEED:2B] [SECTOR:1B] [PWM_DUTY:1B] */
  payload[0] = state;
  payload[1] = (speed_rpm >> 8) & 0xFF;
  payload[2] = speed_rpm & 0xFF;
  payload[3] = sector & 0x0F;
  payload[4] = pwm_duty;
  
  /* Create frame */
  frame_size = SerialProtocol_CreateFrame(
    CMD_MOTOR_STATUS,
    payload,
    5,
    frame_buffer);
  
  /* Send frame */
  if (frame_size > 0)
  {
    SerialProtocol_Send(frame_buffer, frame_size);
  }
  
  return frame_size;
}

uint16_t SerialProtocol_SendBEMF(
  uint16_t bemf_u,
  uint16_t bemf_v,
  uint16_t bemf_w)
{
  uint8_t payload[6];
  uint8_t frame_buffer[FIREWATER_FRAME_MAX_SIZE];
  uint16_t frame_size;
  
  /* Build payload: [BEMF_U:2B] [BEMF_V:2B] [BEMF_W:2B] */
  payload[0] = (bemf_u >> 8) & 0xFF;
  payload[1] = bemf_u & 0xFF;
  payload[2] = (bemf_v >> 8) & 0xFF;
  payload[3] = bemf_v & 0xFF;
  payload[4] = (bemf_w >> 8) & 0xFF;
  payload[5] = bemf_w & 0xFF;
  
  /* Create frame */
  frame_size = SerialProtocol_CreateFrame(
    CMD_BEMF_VALUES,
    payload,
    6,
    frame_buffer);
  
  /* Send frame */
  if (frame_size > 0)
  {
    SerialProtocol_Send(frame_buffer, frame_size);
  }
  
  return frame_size;
}

uint16_t SerialProtocol_SendADC(
  uint16_t bus_voltage,
  uint16_t temperature,
  uint16_t potentiometer)
{
  uint8_t payload[6];
  uint8_t frame_buffer[FIREWATER_FRAME_MAX_SIZE];
  uint16_t frame_size;
  
  /* Build payload: [BUS_VOLTAGE:2B] [TEMPERATURE:2B] [POTENTIOMETER:2B] */
  payload[0] = (bus_voltage >> 8) & 0xFF;
  payload[1] = bus_voltage & 0xFF;
  payload[2] = (temperature >> 8) & 0xFF;
  payload[3] = temperature & 0xFF;
  payload[4] = (potentiometer >> 8) & 0xFF;
  payload[5] = potentiometer & 0xFF;
  
  /* Create frame */
  frame_size = SerialProtocol_CreateFrame(
    CMD_ADC_VALUES,
    payload,
    6,
    frame_buffer);
  
  /* Send frame */
  if (frame_size > 0)
  {
    SerialProtocol_Send(frame_buffer, frame_size);
  }
  
  return frame_size;
}

uint16_t SerialProtocol_SendTelemetryStream(
  uint8_t state,
  uint16_t speed_rpm,
  uint16_t bemf_u,
  uint16_t bemf_v,
  uint16_t bemf_w,
  uint16_t bus_voltage,
  uint16_t temperature,
  uint16_t potentiometer)
{
  uint8_t payload[17];
  uint8_t frame_buffer[FIREWATER_FRAME_MAX_SIZE];
  uint16_t frame_size;
  uint8_t idx = 0;
  
  /* Build combined payload */
  payload[idx++] = state;
  
  payload[idx++] = (speed_rpm >> 8) & 0xFF;
  payload[idx++] = speed_rpm & 0xFF;
  
  payload[idx++] = (bemf_u >> 8) & 0xFF;
  payload[idx++] = bemf_u & 0xFF;
  
  payload[idx++] = (bemf_v >> 8) & 0xFF;
  payload[idx++] = bemf_v & 0xFF;
  
  payload[idx++] = (bemf_w >> 8) & 0xFF;
  payload[idx++] = bemf_w & 0xFF;
  
  payload[idx++] = (bus_voltage >> 8) & 0xFF;
  payload[idx++] = bus_voltage & 0xFF;
  
  payload[idx++] = (temperature >> 8) & 0xFF;
  payload[idx++] = temperature & 0xFF;
  
  payload[idx++] = (potentiometer >> 8) & 0xFF;
  payload[idx++] = potentiometer & 0xFF;
  
  /* Create frame */
  frame_size = SerialProtocol_CreateFrame(
    CMD_TELEMETRY_STREAM,
    payload,
    idx,
    frame_buffer);
  
  /* Send frame */
  if (frame_size > 0)
  {
    SerialProtocol_Send(frame_buffer, frame_size);
  }
  
  return frame_size;
}
