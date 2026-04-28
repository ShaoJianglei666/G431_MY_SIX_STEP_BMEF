/**
  ******************************************************************************
  * @file           : motor_control.h
  * @brief          : Six-step commutation motor control module header file
  ******************************************************************************
  * @attention
  * This module implements BLDC six-step commutation control with BEMF sensing
  * Modular design with low coupling to main application code
  ******************************************************************************
  */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stddef.h>

/* Exported types/enums ------------------------------------------------------*/

/**
 * @brief Motor control state machine states
 */
typedef enum
{
  MOTOR_STATE_IDLE = 0,      /*!< Motor is stopped (idle) */
  MOTOR_STATE_ALIGNMENT,     /*!< Motor alignment phase (optional) */
  MOTOR_STATE_RUNNING,       /*!< Motor is running */
  MOTOR_STATE_ERROR          /*!< Motor error state */
} Motor_State_E;

/**
 * @brief Six-step commutation sector (0-5)
 * Sector 0-5 represent 60бу electrical angle intervals
 */
typedef enum
{
  SECTOR_0 = 0,  /*!< Sector 0: UH-VL */
  SECTOR_1 = 1,  /*!< Sector 1: UH-WL */
  SECTOR_2 = 2,  /*!< Sector 2: VH-WL */
  SECTOR_3 = 3,  /*!< Sector 3: VH-UL */
  SECTOR_4 = 4,  /*!< Sector 4: WH-UL */
  SECTOR_5 = 5   /*!< Sector 5: WH-VL */
} Motor_Sector_E;

/**
 * @brief BEMF comparison edge (rising or falling)
 */
typedef enum
{
  BEMF_EDGE_RISING = 0,
  BEMF_EDGE_FALLING = 1
} BEMF_Edge_E;

/**
 * @brief Motor control configuration parameters
 */
typedef struct
{
  uint16_t pwm_frequency;      /*!< PWM frequency in Hz */
  uint16_t dead_time_ns;       /*!< Dead time in nanoseconds */
  uint8_t  pole_pair_num;      /*!< Number of pole pairs */
  uint16_t min_speed_rpm;      /*!< Minimum motor speed (RPM) */
  uint16_t max_speed_rpm;      /*!< Maximum motor speed (RPM) */
  uint16_t acceleration_ramp;  /*!< Acceleration ramp time (ms) */
} Motor_Config_T;

/**
 * @brief Motor runtime status
 */
typedef struct
{
  Motor_State_E    state;           /*!< Current motor state */
  Motor_Sector_E   sector;          /*!< Current commutation sector */
  uint16_t         speed_rpm;       /*!< Current motor speed (RPM) */
  uint16_t         bemf_u;          /*!< BEMF U phase ADC reading */
  uint16_t         bemf_v;          /*!< BEMF V phase ADC reading */
  uint16_t         bemf_w;          /*!< BEMF W phase ADC reading */
  uint16_t         bus_voltage;     /*!< Bus voltage (ADC reading) */
  uint16_t         temperature;     /*!< Temperature (ADC reading) */
  uint32_t         commutation_count; /*!< Total commutation events count */
} Motor_Status_T;

/* Exported constants --------------------------------------------------------*/

/* BEMF ADC channel configuration */
#define BEMF_ADC_INSTANCE         ADC1
#define BEMF_U_CHANNEL            ADC_CHANNEL_6      /* PC0 */
#define BEMF_V_CHANNEL            ADC_CHANNEL_7      /* PC1 */
#define BEMF_W_CHANNEL            ADC_CHANNEL_8      /* PC2 */

/* BEMF threshold for zero-crossing detection (in ADC counts, 0-4095) */
#define BEMF_ZC_THRESHOLD         2048  /* Mid-voltage approximately */

/* Commutation timing: delay after zero-crossing detection (microseconds) */
#define COMMUTATION_DELAY_US      30

/* Exported function prototypes -----------------------------------------------*/

/**
 * @brief Initialize motor control module
 * @param htim1: TIM1 handle for PWM generation
 * @param config: Motor configuration parameters
 * @retval None
 */
void Motor_Init(TIM_HandleTypeDef *htim1, const Motor_Config_T *config);

/**
 * @brief Start motor (transition from IDLE to RUNNING state)
 * @retval None
 */
void Motor_Start(void);

/**
 * @brief Stop motor (transition from RUNNING to IDLE state)
 * @retval None
 */
void Motor_Stop(void);

/**
 * @brief Update motor state machine (call periodically from main loop)
 * Handles BEMF sampling, zero-crossing detection, and commutation
 * @retval None
 */
void Motor_Update(void);

/**
 * @brief Apply PWM pattern for specified sector
 * @param sector: Target commutation sector (0-5)
 * @retval None
 */
void Motor_SetSector(Motor_Sector_E sector);

/**
 * @brief Read BEMF ADC values
 * Should be called when ADC conversion completes
 * @retval None
 */
void Motor_ReadBEMF(void);

/**
 * @brief Detect BEMF zero-crossing in current sector
 * @retval 1 if zero-crossing detected, 0 otherwise
 */
uint8_t Motor_DetectZeroCrossing(void);

/**
 * @brief Get current motor status
 * @return Motor_Status_T: Current motor status structure
 */
Motor_Status_T Motor_GetStatus(void);

/**
 * @brief Set PWM duty cycle (0-100%)
 * @param duty_percent: Duty cycle percentage (0-100)
 * @retval None
 */
void Motor_SetDuty(uint8_t duty_percent);

/**
 * @brief Perform motor commutation to next sector
 * Called when zero-crossing is detected
 * @retval None
 */
void Motor_Commutate(void);

/* Debug/Test Point Functions ------------------------------------------------*/

/**
 * @brief Get BEMF ADC readings for debugging
 * @return Raw ADC values as Motor_Status_T (only BEMF fields populated)
 */
Motor_Status_T Motor_GetBEMFReadings(void);

/**
 * @brief Get current commutation sector
 * @return Current sector (0-5)
 */
Motor_Sector_E Motor_GetSector(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
