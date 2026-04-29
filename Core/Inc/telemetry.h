/**
  ******************************************************************************
  * @file           : telemetry.h
  * @brief          : Telemetry data collection and management module
  ******************************************************************************
  * This module collects motor control data (BEMF, speed, temperature, etc.)
  * and provides functions to output telemetry via serial protocol
  ******************************************************************************
  */

#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Telemetry output modes */
typedef enum
{
  TELEMETRY_MODE_DISABLED      = 0,  /* No output */
  TELEMETRY_MODE_MOTOR_STATUS  = 1,  /* Only motor status */
  TELEMETRY_MODE_BEMF_ONLY     = 2,  /* Only BEMF readings */
  TELEMETRY_MODE_ADC_ONLY      = 3,  /* Only ADC readings */
  TELEMETRY_MODE_COMBINED      = 4,  /* All telemetry combined */
} Telemetry_Mode_E;

/* Telemetry configuration */
typedef struct
{
  Telemetry_Mode_E mode;           /* Output mode */
  uint16_t output_interval_ms;     /* Output interval in milliseconds */
  uint8_t enable_bemf_output;      /* Enable BEMF output (0/1) */
  uint8_t enable_speed_output;     /* Enable speed output (0/1) */
  uint8_t enable_temp_output;      /* Enable temperature output (0/1) */
} Telemetry_Config_T;

/* Exported Functions */

/**
 * @brief Initialize telemetry module
 * @param config: Telemetry configuration
 * @retval None
 */
void Telemetry_Init(const Telemetry_Config_T *config);

/**
 * @brief Update telemetry data (call periodically, typically from main loop)
 * Handles output interval timing and data transmission
 * @retval None
 */
void Telemetry_Update(void);

/**
 * @brief Set output mode
 * @param mode: New output mode
 * @retval None
 */
void Telemetry_SetMode(Telemetry_Mode_E mode);

/**
 * @brief Set output interval (in milliseconds)
 * @param interval_ms: Output interval
 * @retval None
 */
void Telemetry_SetOutputInterval(uint16_t interval_ms);

/**
 * @brief Update motor status data
 * @param state: Motor state (0=IDLE, 1=RUNNING)
 * @param speed_rpm: Current motor speed
 * @param sector: Current commutation sector
 * @param pwm_duty: PWM duty cycle (0-100%)
 * @retval None
 */
void Telemetry_UpdateMotorStatus(
  uint8_t state,
  uint16_t speed_rpm,
  uint8_t sector,
  uint8_t pwm_duty);

/**
 * @brief Update BEMF readings
 * @param bemf_u: U-phase BEMF ADC value
 * @param bemf_v: V-phase BEMF ADC value
 * @param bemf_w: W-phase BEMF ADC value
 * @retval None
 */
void Telemetry_UpdateBEMF(
  uint16_t bemf_u,
  uint16_t bemf_v,
  uint16_t bemf_w);

/**
 * @brief Update ADC readings
 * @param bus_voltage: Bus voltage ADC value
 * @param temperature: Temperature sensor ADC value
 * @param potentiometer: Potentiometer ADC value for speed control
 * @retval None
 */
void Telemetry_UpdateADC(
  uint16_t bus_voltage,
  uint16_t temperature,
  uint16_t potentiometer);

/**
 * @brief Force immediate telemetry output (ignores interval timing)
 * @retval None
 */
void Telemetry_ForceOutput(void);

/**
 * @brief Get BEMF output status
 * @retval 1 if BEMF output enabled, 0 otherwise
 */
uint8_t Telemetry_IsBEMFOutputEnabled(void);

/**
 * @brief Get speed output status
 * @retval 1 if speed output enabled, 0 otherwise
 */
uint8_t Telemetry_IsSpeedOutputEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H */
