/**
  ******************************************************************************
  * @file           : speed_control.c
  * @brief          : Motor speed control implementation with potentiometer
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_control.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static Speed_Control_Config_T g_speed_config;
static Speed_Mode_E g_current_mode;
static uint16_t g_manual_pwm_duty = 50;           /* Default 50% */
static uint16_t g_speed_setpoint_rpm = 1000;     /* Default 1000 RPM */
static uint16_t g_actual_speed_rpm = 0;          /* Current actual speed */
static uint16_t g_current_pwm_duty = 0;
static uint16_t g_pot_adc_value = 0;
static uint16_t g_pot_speed_rpm = 0;

/* Private function prototypes -----------------------------------------------*/
static uint16_t SpeedControl_ADCToPWMDuty(uint16_t adc_value);
static uint16_t SpeedControl_ADCToSpeed(uint16_t adc_value);

/* Public function definitions -----------------------------------------------*/

void SpeedControl_Init(const Speed_Control_Config_T *config)
{
  if (config == NULL)
  {
    /* Use default configuration */
    g_speed_config.mode = SPEED_MODE_MANUAL;
    g_speed_config.min_speed_rpm = 100;
    g_speed_config.max_speed_rpm = 8000;
    g_speed_config.min_pwm_duty = 20;
    g_speed_config.max_pwm_duty = 100;
    g_speed_config.acceleration_rate_rpm_per_ms = 5;  /* 5 RPM/ms = 5000 RPM/s */
    g_speed_config.pot_adc_min = 0;
    g_speed_config.pot_adc_max = 4095;
  }
  else
  {
    g_speed_config = *config;
  }
  
  g_current_mode = g_speed_config.mode;
  g_current_pwm_duty = 0;
}

void SpeedControl_Update(uint16_t delta_time_ms)
{
  uint16_t target_duty = 0;
  
  /* Get target duty based on mode */
  switch (g_current_mode)
  {
    case SPEED_MODE_MANUAL:
      target_duty = g_manual_pwm_duty;
      break;
      
    case SPEED_MODE_POTENTIOMETER:
      /* Convert potentiometer ADC to PWM duty */
      target_duty = SpeedControl_ADCToPWMDuty(g_pot_adc_value);
      break;
      
    case SPEED_MODE_SETPOINT:
      /* Convert speed setpoint to PWM duty (simple linear mapping) */
      if (g_speed_setpoint_rpm >= g_speed_config.min_speed_rpm)
      {
        uint16_t speed_range = g_speed_config.max_speed_rpm - g_speed_config.min_speed_rpm;
        uint16_t pwm_range = g_speed_config.max_pwm_duty - g_speed_config.min_pwm_duty;
        
        if (speed_range > 0)
        {
          uint16_t speed_offset = g_speed_setpoint_rpm - g_speed_config.min_speed_rpm;
          target_duty = g_speed_config.min_pwm_duty + 
                       ((speed_offset * pwm_range) / speed_range);
        }
      }
      else
      {
        target_duty = 0;
      }
      break;
      
    default:
      target_duty = 0;
      break;
  }
  
  /* Enforce PWM duty limits */
  if (target_duty > g_speed_config.max_pwm_duty)
  {
    target_duty = g_speed_config.max_pwm_duty;
  }
  
  /* Apply acceleration ramp if configured */
  if (g_speed_config.acceleration_rate_rpm_per_ms > 0 && delta_time_ms > 0)
  {
    int16_t duty_diff = (int16_t)target_duty - (int16_t)g_current_pwm_duty;
    
    /* Calculate max allowed change per update */
    int16_t max_change = (int16_t)((g_speed_config.acceleration_rate_rpm_per_ms * 
                                    delta_time_ms) / 10);  /* Scale factor */
    
    if (duty_diff > max_change)
    {
      g_current_pwm_duty += max_change;
    }
    else if (duty_diff < -max_change)
    {
      g_current_pwm_duty -= max_change;
    }
    else
    {
      g_current_pwm_duty = target_duty;
    }
  }
  else
  {
    g_current_pwm_duty = target_duty;
  }
}

void SpeedControl_SetMode(Speed_Mode_E mode)
{
  g_current_mode = mode;
}

void SpeedControl_SetPWMDuty(uint8_t duty_percent)
{
  if (duty_percent > 100)
  {
    duty_percent = 100;
  }
  
  g_manual_pwm_duty = duty_percent;
  
  /* If in manual mode, update immediately */
  if (g_current_mode == SPEED_MODE_MANUAL)
  {
    g_current_pwm_duty = duty_percent;
  }
}

void SpeedControl_SetSpeedSetpoint(uint16_t speed_rpm)
{
  g_speed_setpoint_rpm = speed_rpm;
  
  /* Enforce limits */
  if (g_speed_setpoint_rpm > g_speed_config.max_speed_rpm)
  {
    g_speed_setpoint_rpm = g_speed_config.max_speed_rpm;
  }
  else if (g_speed_setpoint_rpm < g_speed_config.min_speed_rpm)
  {
    g_speed_setpoint_rpm = g_speed_config.min_speed_rpm;
  }
}

void SpeedControl_UpdatePotentiometer(uint16_t pot_adc_value)
{
  g_pot_adc_value = pot_adc_value;
  
  /* Calculate speed from potentiometer */
  g_pot_speed_rpm = SpeedControl_ADCToSpeed(pot_adc_value);
}

uint16_t SpeedControl_GetPWMDuty(void)
{
  return g_current_pwm_duty;
}

uint16_t SpeedControl_GetSpeedSetpoint(void)
{
  return g_speed_setpoint_rpm;
}

uint16_t SpeedControl_GetPotentiometerValue(void)
{
  return g_pot_adc_value;
}

uint8_t SpeedControl_GetPotentiometerPercent(void)
{
  /* Convert ADC value to percentage */
  if (g_speed_config.pot_adc_max <= g_speed_config.pot_adc_min)
  {
    return 0;
  }
  
  uint16_t adc_range = g_speed_config.pot_adc_max - g_speed_config.pot_adc_min;
  uint16_t adc_offset = g_pot_adc_value - g_speed_config.pot_adc_min;
  
  if (adc_offset > adc_range)
  {
    return 100;
  }
  
  return (uint8_t)((adc_offset * 100) / adc_range);
}

/* Private function definitions -----------------------------------------------*/

/**
 * @brief Convert potentiometer ADC value to PWM duty cycle
 * Linear mapping from ADC range to PWM duty range
 */
static uint16_t SpeedControl_ADCToPWMDuty(uint16_t adc_value)
{
  if (g_speed_config.pot_adc_max <= g_speed_config.pot_adc_min)
  {
    return 0;
  }
  
  uint16_t adc_range = g_speed_config.pot_adc_max - g_speed_config.pot_adc_min;
  uint16_t pwm_range = g_speed_config.max_pwm_duty - g_speed_config.min_pwm_duty;
  
  uint16_t adc_offset = adc_value - g_speed_config.pot_adc_min;
  
  if (adc_offset > adc_range)
  {
    return g_speed_config.max_pwm_duty;
  }
  
  if (adc_offset == 0)
  {
    return 0;  /* No output if potentiometer is at minimum */
  }
  
  uint16_t duty = g_speed_config.min_pwm_duty + 
                  ((adc_offset * pwm_range) / adc_range);
  
  if (duty > g_speed_config.max_pwm_duty)
  {
    duty = g_speed_config.max_pwm_duty;
  }
  
  return duty;
}

/**
 * @brief Convert potentiometer ADC value to speed RPM
 * Linear mapping from ADC range to speed range
 */
static uint16_t SpeedControl_ADCToSpeed(uint16_t adc_value)
{
  if (g_speed_config.pot_adc_max <= g_speed_config.pot_adc_min)
  {
    return 0;
  }
  
  uint16_t adc_range = g_speed_config.pot_adc_max - g_speed_config.pot_adc_min;
  uint16_t speed_range = g_speed_config.max_speed_rpm - g_speed_config.min_speed_rpm;
  
  uint16_t adc_offset = adc_value - g_speed_config.pot_adc_min;
  
  if (adc_offset > adc_range)
  {
    return g_speed_config.max_speed_rpm;
  }
  
  if (adc_offset == 0)
  {
    return 0;  /* No speed if potentiometer is at minimum */
  }
  
  uint16_t speed = g_speed_config.min_speed_rpm + 
                   ((adc_offset * speed_range) / adc_range);
  
  if (speed > g_speed_config.max_speed_rpm)
  {
    speed = g_speed_config.max_speed_rpm;
  }
  
  return speed;
}
