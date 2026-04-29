/**
  ******************************************************************************
  * @file           : serial_protocol.h
  * @brief          : FireWater serial protocol for motor telemetry data
  ******************************************************************************
  * FireWater Protocol Format:
  * [0xAA] [0x55] [LEN] [CMD] [DATA...] [CRC8]
  * - Header: 0xAA 0x55 (2 bytes)
  * - Length: Data length excluding header and CRC (1 byte)
  * - Command: Function code (1 byte)
  * - Data: Variable length payload
  * - CRC8: Simple checksum
  ******************************************************************************
  */

#ifndef __SERIAL_PROTOCOL_H
#define __SERIAL_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* FireWater Protocol Constants */
#define FIREWATER_HEADER_BYTE1       0xAA
#define FIREWATER_HEADER_BYTE2       0x55
#define FIREWATER_MAX_PAYLOAD        64
#define FIREWATER_FRAME_MAX_SIZE     (FIREWATER_MAX_PAYLOAD + 5)  /* Header + CRC + Len + Cmd */

/* Command Codes */
typedef enum
{
  CMD_HEARTBEAT              = 0x00,  /* Keep-alive / heartbeat */
  CMD_MOTOR_STATUS           = 0x01,  /* Motor status (speed, state, etc.) */
  CMD_BEMF_VALUES            = 0x02,  /* BEMF readings (3 phase) */
  CMD_ADC_VALUES             = 0x03,  /* ADC readings (bus voltage, temp, etc.) */
  CMD_CONTROL_COMMAND        = 0x04,  /* Control command (start, stop, speed) */
  CMD_ERROR_LOG              = 0x05,  /* Error/warning log */
  CMD_CONFIG_PARAMETERS      = 0x06,  /* Configuration parameters */
  CMD_TELEMETRY_STREAM       = 0x07,  /* Combined telemetry stream */
} FireWater_Cmd_E;

/* Serial frame structure */
typedef struct
{
  uint8_t header[2];                               /* 0xAA, 0x55 */
  uint8_t length;                                  /* Payload length */
  uint8_t command;                                 /* Command code */
  uint8_t payload[FIREWATER_MAX_PAYLOAD];         /* Data payload */
  uint8_t crc;                                     /* CRC8 checksum */
} FireWater_Frame_T;

/* Exported Functions */

/**
 * @brief Initialize serial protocol module
 * @param None
 * @retval None
 */
void SerialProtocol_Init(void);

/**
 * @brief Create FireWater frame with command and payload
 * @param cmd: Command code
 * @param data: Payload data pointer
 * @param data_len: Payload length in bytes
 * @param frame: Output frame buffer (pre-allocated by caller)
 * @retval Frame total size in bytes (including header, CRC, length, command)
 */
uint16_t SerialProtocol_CreateFrame(
  uint8_t cmd,
  const uint8_t *data,
  uint8_t data_len,
  uint8_t *frame_buffer);

/**
 * @brief Calculate CRC8 checksum
 * @param data: Data buffer
 * @param length: Data length
 * @retval CRC8 value
 */
uint8_t SerialProtocol_CalcCRC8(const uint8_t *data, uint16_t length);

/**
 * @brief Send data via UART with DMA
 * @param data: Data buffer to send
 * @param length: Data length
 * @retval HAL status code
 */
uint8_t SerialProtocol_Send(const uint8_t *data, uint16_t length);

/**
 * @brief Send simple "hello" message for testing
 * @param None
 * @retval Frame size sent
 */
void SerialProtocol_SendHello(void);

/**
 * @brief Transmit motor status frame
 * Format: [STATE:1B] [SPEED:2B] [SECTOR:1B] [PWM_DUTY:1B]
 * @param state: Motor state (0=IDLE, 1=RUNNING)
 * @param speed_rpm: Motor speed in RPM
 * @param sector: Current sector (0-5)
 * @param pwm_duty: PWM duty cycle (0-100%)
 * @retval Frame size sent
 */
uint16_t SerialProtocol_SendMotorStatus(
  uint8_t state,
  uint16_t speed_rpm,
  uint8_t sector,
  uint8_t pwm_duty);

/**
 * @brief Transmit BEMF values frame
 * Format: [BEMF_U:2B] [BEMF_V:2B] [BEMF_W:2B]
 * @param bemf_u: U-phase BEMF ADC value
 * @param bemf_v: V-phase BEMF ADC value
 * @param bemf_w: W-phase BEMF ADC value
 * @retval Frame size sent
 */
uint16_t SerialProtocol_SendBEMF(
  uint16_t bemf_u,
  uint16_t bemf_v,
  uint16_t bemf_w);

/**
 * @brief Transmit ADC values frame
 * Format: [BUS_VOLTAGE:2B] [TEMPERATURE:2B] [POTENTIOMETER:2B]
 * @param bus_voltage: Bus voltage ADC value
 * @param temperature: Temperature sensor ADC value
 * @param potentiometer: Potentiometer ADC value (for speed control)
 * @retval Frame size sent
 */
uint16_t SerialProtocol_SendADC(
  uint16_t bus_voltage,
  uint16_t temperature,
  uint16_t potentiometer);

/**
 * @brief Transmit telemetry stream (combined data)
 * Format: [STATE:1B] [SPEED:2B] [BEMF_U:2B] [BEMF_V:2B] [BEMF_W:2B] 
 *         [BUS_V:2B] [TEMP:2B] [POT:2B]
 * @param state: Motor state
 * @param speed_rpm: Motor speed
 * @param bemf_u: U-phase BEMF
 * @param bemf_v: V-phase BEMF
 * @param bemf_w: W-phase BEMF
 * @param bus_voltage: Bus voltage
 * @param temperature: Temperature
 * @param potentiometer: Potentiometer reading
 * @retval Frame size sent
 */
uint16_t SerialProtocol_SendTelemetryStream(
  uint8_t state,
  uint16_t speed_rpm,
  uint16_t bemf_u,
  uint16_t bemf_v,
  uint16_t bemf_w,
  uint16_t bus_voltage,
  uint16_t temperature,
  uint16_t potentiometer);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_PROTOCOL_H */
