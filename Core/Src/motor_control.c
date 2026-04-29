/**
  ******************************************************************************
  * @file           : motor_control.c
  * @brief          : Six-step commutation motor control module implementation
  ******************************************************************************
  * @attention
  * This module implements BLDC six-step commutation control
  * BEMF-based zero-crossing detection triggers commutation
  * Button triggers start/stop state transitions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
#include "main.h"
#include "tim.h"
#include "adc.h"
#include <stddef.h>
#include <stdint.h>

/* Private variables ---------------------------------------------------------*/

/** Motor control state machine state */
static Motor_State_E g_motor_state = MOTOR_STATE_IDLE;

/** Current commutation sector */
static Motor_Sector_E g_current_sector = SECTOR_0;

/** Motor configuration parameters */
static Motor_Config_T g_motor_config;

/** Motor runtime status */
static Motor_Status_T g_motor_status;

/** TIM1 handle pointer */
static TIM_HandleTypeDef *g_htim1 = NULL;

/** Current PWM duty cycle (0-10000, where 10000 = 100%) */
static uint16_t g_pwm_duty = 1500;  /* Default 50% */

/** Zero-crossing detection counter */
static uint16_t g_zc_stable_count = 0;

/** Commutation delay timer */
static uint16_t g_comm_delay_timer = 0;

/* Six-step commutation pattern table */
/* Format: {UH, VH, WH, UL, VL, WL} */
/* ON=1, OFF=0 */
typedef struct
{
  uint8_t uh;
  uint8_t vh;
  uint8_t wh;
  uint8_t ul;
  uint8_t vl;
  uint8_t wl;
} Commutation_Pattern_T;

static const Commutation_Pattern_T g_commutation_table[6] =
{
  /* SECTOR_0 */
  {.uh = 1, .vh = 0, .wh = 0, .ul = 0, .vl = 1, .wl = 0},  /* UH-VL on */
  /* SECTOR_1 */
  {.uh = 1, .vh = 0, .wh = 0, .ul = 0, .vl = 0, .wl = 1},  /* UH-WL on */
  /* SECTOR_2 */
  {.uh = 0, .vh = 1, .wh = 0, .ul = 0, .vl = 0, .wl = 1},  /* VH-WL on */
  /* SECTOR_3 */
  {.uh = 0, .vh = 1, .wh = 0, .ul = 1, .vl = 0, .wl = 0},  /* VH-UL on */
  /* SECTOR_4 */
  {.uh = 0, .vh = 0, .wh = 1, .ul = 1, .vl = 0, .wl = 0},  /* WH-UL on */
  /* SECTOR_5 */
  {.uh = 0, .vh = 0, .wh = 1, .ul = 0, .vl = 1, .wl = 0},  /* WH-VL on */
};

/* BEMF monitored phase for each sector (for zero-crossing detection) */
/* Value: 0=U phase, 1=V phase, 2=W phase */
static const uint8_t g_bemf_monitored[6] = {
  2,  /* SECTOR_0: Monitor W phase */
  1,  /* SECTOR_1: Monitor V phase */
  0,  /* SECTOR_2: Monitor U phase */
  2,  /* SECTOR_3: Monitor W phase */
  1,  /* SECTOR_4: Monitor V phase */
  0,  /* SECTOR_5: Monitor U phase */
};

/* Private function prototypes -----------------------------------------------*/
static void Motor_PWM_Init(void);
static void Motor_PWM_Disable(void);
static void Motor_UpdatePWMOutputs(void);
static void Motor_SetNextSector(void);
static uint8_t Motor_CompareWithThreshold(uint16_t bemf_value);

/* Public function definitions -----------------------------------------------*/

void Motor_Init(TIM_HandleTypeDef *htim1, const Motor_Config_T *config)
{
  if (htim1 == NULL || config == NULL)
  {
    return;
  }

  g_htim1 = htim1;
  g_motor_config = *config;

  /* Initialize motor status */
  g_motor_status.state = MOTOR_STATE_IDLE;
  g_motor_status.sector = SECTOR_0;
  g_motor_status.speed_rpm = 0;
  g_motor_status.bemf_u = 2048;
  g_motor_status.bemf_v = 2048;
  g_motor_status.bemf_w = 2048;
  g_motor_status.bus_voltage = 0;
  g_motor_status.temperature = 0;
  g_motor_status.commutation_count = 0;

  /* Initialize PWM */
  Motor_PWM_Init();

  /* Motor ready in IDLE state */
  g_motor_state = MOTOR_STATE_IDLE;
}

void Motor_Start(void)
{
  if (g_motor_state == MOTOR_STATE_IDLE)
  {
    g_motor_state = MOTOR_STATE_RUNNING;
    g_current_sector = SECTOR_0;
    g_pwm_duty = 5000;  /* Start with 50% duty */
    g_zc_stable_count = 0;
    g_motor_status.commutation_count = 0;

    /* Apply initial commutation pattern */
    Motor_SetSector(SECTOR_0);

    /* Enable TIM1 PWM output */
    HAL_TIM_PWM_Start(g_htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(g_htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(g_htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(g_htim1, TIM_CHANNEL_4);
    HAL_TIMEx_PWMN_Start(g_htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(g_htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(g_htim1, TIM_CHANNEL_3);

    g_motor_status.state = MOTOR_STATE_RUNNING;
  }
}

void Motor_Stop(void)
{
  if (g_motor_state == MOTOR_STATE_RUNNING)
  {
    /* Disable all PWM outputs */
    Motor_PWM_Disable();

    g_motor_state = MOTOR_STATE_IDLE;
    g_motor_status.state = MOTOR_STATE_IDLE;
    g_motor_status.speed_rpm = 0;
  }
}

void Motor_Update(void)
{
  if (g_motor_state != MOTOR_STATE_RUNNING)
  {
    return;
  }

  /* Read current BEMF values */
  Motor_ReadBEMF();

  /* Detect zero-crossing */
  if (Motor_DetectZeroCrossing())
  {
    g_zc_stable_count++;

    /* Require multiple detections for stability */
    if (g_zc_stable_count >= 3)
    {
      Motor_Commutate();
      g_zc_stable_count = 0;
    }
  }
  else
  {
    g_zc_stable_count = 0;
  }

  /* Update status */
  g_motor_status.state = g_motor_state;
  g_motor_status.sector = g_current_sector;
}

void Motor_SetSector(Motor_Sector_E sector)
{
  if (sector < 6)
  {
    g_current_sector = sector;
    Motor_UpdatePWMOutputs();
  }
}

void Motor_ReadBEMF(void)
{
  uint32_t raw_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  uint32_t raw_v = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  uint32_t raw_w = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

  if (hadc1.Init.DataAlign == ADC_DATAALIGN_LEFT)
  {
    raw_u >>= 4;
    raw_v >>= 4;
    raw_w >>= 4;
  }

  g_motor_status.bemf_u = (uint16_t)raw_u;
  g_motor_status.bemf_v = (uint16_t)raw_v;
  g_motor_status.bemf_w = (uint16_t)raw_w;
}

uint8_t Motor_DetectZeroCrossing(void)
{
  uint8_t monitored_phase = g_bemf_monitored[g_current_sector];
  uint16_t bemf_value = 0;

  /* Get BEMF value of monitored phase */
  switch (monitored_phase)
  {
    case 0:
      bemf_value = g_motor_status.bemf_u;
      break;
    case 1:
      bemf_value = g_motor_status.bemf_v;
      break;
    case 2:
      bemf_value = g_motor_status.bemf_w;
      break;
    default:
      return 0;
  }

  /* Simple threshold comparison for zero-crossing detection */
  return Motor_CompareWithThreshold(bemf_value);
}

Motor_Status_T Motor_GetStatus(void)
{
  return g_motor_status;
}

void Motor_SetDuty(uint8_t duty_percent)
{
  if (duty_percent > 100)
  {
    duty_percent = 95;
  }

  /* Convert percentage to counter value (0-10000 represents 0-100%) */
  g_pwm_duty = (uint16_t)((duty_percent * 10000) / 100);

  if (g_motor_state == MOTOR_STATE_RUNNING)
  {
    Motor_UpdatePWMOutputs();
  }
}

void Motor_Commutate(void)
{
  if (g_motor_state == MOTOR_STATE_RUNNING)
  {
    /* Move to next sector */
    Motor_SetNextSector();

    /* Update PWM pattern */
    Motor_UpdatePWMOutputs();

    /* Increment commutation counter for debugging */
    g_motor_status.commutation_count++;

    /* Toggle LED for visual feedback (test point) */
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

Motor_Status_T Motor_GetBEMFReadings(void)
{
  Motor_Status_T readings;
  readings.bemf_u = g_motor_status.bemf_u;
  readings.bemf_v = g_motor_status.bemf_v;
  readings.bemf_w = g_motor_status.bemf_w;
  return readings;
}

Motor_Sector_E Motor_GetSector(void)
{
  return g_current_sector;
}

/* Private function definitions -----------------------------------------------*/

static void Motor_PWM_Init(void)
{
  if (g_htim1 == NULL)
  {
    return;
  }

  /* TIM1 PWM channels are already initialized via MX_TIM1_Init()
   * This function ensures consistency with motor control settings
   */

  /* Set initial duty cycle */
  g_htim1->Instance->CCR1 = g_pwm_duty;
  g_htim1->Instance->CCR2 = g_pwm_duty;
  g_htim1->Instance->CCR3 = g_pwm_duty;
  g_htim1->Instance->CCR4 = g_pwm_duty;
}

static void Motor_PWM_Disable(void)
{
  if (g_htim1 == NULL)
  {
    return;
  }

  /* Disable all PWM outputs */
  HAL_TIM_PWM_Stop(g_htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(g_htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(g_htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(g_htim1, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Stop(g_htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(g_htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(g_htim1, TIM_CHANNEL_3);
}

static void Motor_UpdatePWMOutputs(void)
{
  if (g_htim1 == NULL)
  {
    return;
  }

  /* Get commutation pattern for current sector */
  uint8_t uh = g_commutation_table[g_current_sector].uh;
  uint8_t vh = g_commutation_table[g_current_sector].vh;
  uint8_t wh = g_commutation_table[g_current_sector].wh;
  uint8_t ul = g_commutation_table[g_current_sector].ul;
  uint8_t vl = g_commutation_table[g_current_sector].vl;
  uint8_t wl = g_commutation_table[g_current_sector].wl;

  /* Update TIM1 CCR values based on commutation pattern */
  /* Channel 1 = UH (PA8), Channel 2 = VH (PA9), Channel 3 = WH (PA10) */
  /* Channel 4 = UL (PB13), Complementary = VL (PA12), Complementary = WL (PB15) */

  /* High-side PWM control */
  g_htim1->Instance->CCR1 = uh ? g_pwm_duty : 0;  /* UH */
  g_htim1->Instance->CCR2 = vh ? g_pwm_duty : 0;  /* VH */
  g_htim1->Instance->CCR3 = wh ? g_pwm_duty : 0;  /* WH */

  /* Low-side PWM control (via complementary channels or separate control) */
  /* Note: Your hardware may have different PWM topology
   * Adjust these mappings according to your driver circuit
   */
  g_htim1->Instance->CCR4 = ul ? g_pwm_duty : 0;  /* UL (if using CCR4) */

  /* For VL (PA12) and WL (PB15), you may need separate PWM timers
   * or use GPIO-based switching depending on your driver configuration
   */
}

static void Motor_SetNextSector(void)
{
  if (g_current_sector < SECTOR_5)
  {
    g_current_sector++;
  }
  else
  {
    g_current_sector = SECTOR_0;
  }
}

static uint8_t Motor_CompareWithThreshold(uint16_t bemf_value)
{
  /* Zero-crossing detection: monitor when BEMF crosses the mid-voltage point
   * Simple implementation: check if value is near threshold (¡À100 counts)
   */

  if (bemf_value > (BEMF_ZC_THRESHOLD - 100) &&
      bemf_value < (BEMF_ZC_THRESHOLD + 100))
  {
    return 1;  /* Zero-crossing detected */
  }

  return 0;  /* No zero-crossing detected */
}

/* Exported utility function for ADC interrupt integration */

/**
 * @brief Set BEMF ADC values (typically called from ADC interrupt)
 * This allows integration with your ADC interrupt handler
 * @param bemf_u: U-phase BEMF ADC reading
 * @param bemf_v: V-phase BEMF ADC reading
 * @param bemf_w: W-phase BEMF ADC reading
 * @retval None
 */
void Motor_SetBEMFValues(uint16_t bemf_u, uint16_t bemf_v, uint16_t bemf_w)
{
  g_motor_status.bemf_u = bemf_u;
  g_motor_status.bemf_v = bemf_v;
  g_motor_status.bemf_w = bemf_w;
}

/**
 * @brief Set bus voltage and temperature (typically called from ADC interrupt)
 * @param bus_voltage: Bus voltage ADC reading
 * @param temperature: Temperature sensor ADC reading
 * @retval None
 */
void Motor_SetAuxValues(uint16_t bus_voltage, uint16_t temperature)
{
  g_motor_status.bus_voltage = bus_voltage;
  g_motor_status.temperature = temperature;
}
