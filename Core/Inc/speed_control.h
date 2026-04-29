/**
  ******************************************************************************
  * @file           : speed_control.h
  * @brief          : Motor speed control module with potentiometer input
  ******************************************************************************
  * This module handles:
  * - Potentiometer ADC reading (PC5)
  * - Speed setpoint calculation
  * - Speed ramp control (smooth acceleration/deceleration)
  * - Speed limit enforcement
  ******************************************************************************
  */

#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Speed control modes */
typedef enum
{
  SPEED_MODE_MANUAL          = 0,  /* Manual PWM duty control */
  SPEED_MODE_POTENTIOMETER   = 1,  /* Potentiometer-based speed control */
  SPEED_MODE_SETPOINT        = 2,  /* External setpoint control */
} Speed_Mode_E;

/* Speed control configuration */
typedef struct
{
  Speed_Mode_E mode;                    /* Speed control mode */
  uint16_t min_speed_rpm;               /* Minimum speed (RPM) */
  uint16_t max_speed_rpm;               /* Maximum speed (RPM) */
  uint16_t min_pwm_duty;                /* Minimum PWM duty (0-100%) */
  uint16_t max_pwm_duty;                /* Maximum PWM duty (0-100%) */
  uint16_t acceleration_rate_rpm_per_ms;  /* Acceleration ramp rate */
  uint16_t pot_adc_min;                 /* Potentiometer ADC min value */
  uint16_t pot_adc_max;                 /* Potentiometer ADC max value */
} Speed_Control_Config_T;

/* Exported Functions */

/**
 * @brief Initialize speed control module
 * @param config: Speed control configuration
 * @retval None
 */
void SpeedControl_Init(const Speed_Control_Config_T *config);

/**
 * @brief Update speed control (call periodically from main loop)
 * Handles potentiometer reading and speed ramping
 * @param delta_time_ms: Time delta since last update (milliseconds)
 * @retval None
 */
void SpeedControl_Update(uint16_t delta_time_ms);

/**
 * @brief Set speed control mode
 * @param mode: New speed control mode
 * @retval None
 */
void SpeedControl_SetMode(Speed_Mode_E mode);

/**
 * @brief Set manual PWM duty cycle
 * Only effective in MANUAL mode
 * @param duty_percent: PWM duty cycle (0-100%)
 * @retval None
 */
void SpeedControl_SetPWMDuty(uint8_t duty_percent);

/**
 * @brief Set speed setpoint in RPM
 * Only effective in SETPOINT mode
 * @param speed_rpm: Target speed in RPM
 * @retval None
 */
void SpeedControl_SetSpeedSetpoint(uint16_t speed_rpm);

/**
 * @brief Update potentiometer ADC reading
 * @param pot_adc_value: Raw ADC value (0-4095 for 12-bit ADC)
 * @retval None
 */
void SpeedControl_UpdatePotentiometer(uint16_t pot_adc_value);

/**
 * @brief Get current PWM duty cycle output
 * @retval PWM duty cycle (0-100%)
 */
uint16_t SpeedControl_GetPWMDuty(void);

/**
 * @brief Get current speed setpoint
 * @retval Speed setpoint in RPM
 */
uint16_t SpeedControl_GetSpeedSetpoint(void);

/**
 * @brief Get potentiometer ADC value
 * @retval Last potentiometer ADC reading
 */
uint16_t SpeedControl_GetPotentiometerValue(void);

/**
 * @brief Get raw potentiometer percentage
 * @retval Potentiometer reading as percentage (0-100%)
 */
uint8_t SpeedControl_GetPotentiometerPercent(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPEED_CONTROL_H */
