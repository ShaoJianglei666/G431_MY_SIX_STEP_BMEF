/**
  ******************************************************************************
  * @file           : telemetry.c
  * @brief          : Telemetry data collection and output implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "telemetry.h"
#include "serial_protocol.h"
#include "main.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define DEFAULT_OUTPUT_INTERVAL_MS    100  /* 100ms default output interval */

/* Private types */
typedef struct
{
  /* Motor status data */
  uint8_t motor_state;
  uint16_t speed_rpm;
  uint8_t sector;
  uint8_t pwm_duty;
  
  /* BEMF data */
  uint16_t bemf_u;
  uint16_t bemf_v;
  uint16_t bemf_w;
  
  /* ADC data */
  uint16_t bus_voltage;
  uint16_t temperature;
  uint16_t potentiometer;
} Telemetry_Data_T;

/* Private variables ---------------------------------------------------------*/
static Telemetry_Config_T g_telemetry_config;
static Telemetry_Data_T g_telemetry_data;
static uint32_t g_last_output_time_ms = 0;
static uint32_t g_current_time_ms = 0;

/* Private function prototypes -----------------------------------------------*/
static void Telemetry_OutputMotorStatus(void);
static void Telemetry_OutputBEMF(void);
static void Telemetry_OutputADC(void);
static void Telemetry_OutputCombined(void);
static uint32_t Telemetry_GetTime_ms(void);

/* Public function definitions -----------------------------------------------*/

void Telemetry_Init(const Telemetry_Config_T *config)
{
  if (config == NULL)
  {
    /* Use default configuration */
    g_telemetry_config.mode = TELEMETRY_MODE_DISABLED;
    g_telemetry_config.output_interval_ms = DEFAULT_OUTPUT_INTERVAL_MS;
    g_telemetry_config.enable_bemf_output = 0;
    g_telemetry_config.enable_speed_output = 0;
    g_telemetry_config.enable_temp_output = 0;
  }
  else
  {
    g_telemetry_config = *config;
  }
  
  /* Initialize telemetry data */
  memset(&g_telemetry_data, 0, sizeof(Telemetry_Data_T));
  g_last_output_time_ms = Telemetry_GetTime_ms();
}

void Telemetry_Update(void)
{
  if (g_telemetry_config.mode == TELEMETRY_MODE_DISABLED)
  {
    return;
  }
  
  g_current_time_ms = Telemetry_GetTime_ms();
  
  /* Check if output interval elapsed */
  if ((g_current_time_ms - g_last_output_time_ms) >= g_telemetry_config.output_interval_ms)
  {
    /* Output telemetry data based on mode */
    switch (g_telemetry_config.mode)
    {
      case TELEMETRY_MODE_MOTOR_STATUS:
        Telemetry_OutputMotorStatus();
        break;
        
      case TELEMETRY_MODE_BEMF_ONLY:
        Telemetry_OutputBEMF();
        break;
        
      case TELEMETRY_MODE_ADC_ONLY:
        Telemetry_OutputADC();
        break;
        
      case TELEMETRY_MODE_COMBINED:
        Telemetry_OutputCombined();
        break;
        
      default:
        break;
    }
    
    g_last_output_time_ms = g_current_time_ms;
  }
}

void Telemetry_SetMode(Telemetry_Mode_E mode)
{
  g_telemetry_config.mode = mode;
}

void Telemetry_SetOutputInterval(uint16_t interval_ms)
{
  g_telemetry_config.output_interval_ms = interval_ms;
}

void Telemetry_UpdateMotorStatus(
  uint8_t state,
  uint16_t speed_rpm,
  uint8_t sector,
  uint8_t pwm_duty)
{
  g_telemetry_data.motor_state = state;
  g_telemetry_data.speed_rpm = speed_rpm;
  g_telemetry_data.sector = sector;
  g_telemetry_data.pwm_duty = pwm_duty;
}

void Telemetry_UpdateBEMF(
  uint16_t bemf_u,
  uint16_t bemf_v,
  uint16_t bemf_w)
{
  g_telemetry_data.bemf_u = bemf_u;
  g_telemetry_data.bemf_v = bemf_v;
  g_telemetry_data.bemf_w = bemf_w;
}

void Telemetry_UpdateADC(
  uint16_t bus_voltage,
  uint16_t temperature,
  uint16_t potentiometer)
{
  g_telemetry_data.bus_voltage = bus_voltage;
  g_telemetry_data.temperature = temperature;
  g_telemetry_data.potentiometer = potentiometer;
}

void Telemetry_ForceOutput(void)
{
  if (g_telemetry_config.mode == TELEMETRY_MODE_DISABLED)
  {
    return;
  }
  
  /* Output immediately, then reset timer */
  switch (g_telemetry_config.mode)
  {
    case TELEMETRY_MODE_MOTOR_STATUS:
      Telemetry_OutputMotorStatus();
      break;
      
    case TELEMETRY_MODE_BEMF_ONLY:
      Telemetry_OutputBEMF();
      break;
      
    case TELEMETRY_MODE_ADC_ONLY:
      Telemetry_OutputADC();
      break;
      
    case TELEMETRY_MODE_COMBINED:
      Telemetry_OutputCombined();
      break;
      
    default:
      break;
  }
  
  g_last_output_time_ms = Telemetry_GetTime_ms();
}

uint8_t Telemetry_IsBEMFOutputEnabled(void)
{
  return g_telemetry_config.enable_bemf_output;
}

uint8_t Telemetry_IsSpeedOutputEnabled(void)
{
  return g_telemetry_config.enable_speed_output;
}

/* Private function definitions -----------------------------------------------*/

static void Telemetry_OutputMotorStatus(void)
{
  SerialProtocol_SendMotorStatus(
    g_telemetry_data.motor_state,
    g_telemetry_data.speed_rpm,
    g_telemetry_data.sector,
    g_telemetry_data.pwm_duty);
}

static void Telemetry_OutputBEMF(void)
{
  SerialProtocol_SendBEMF(
    g_telemetry_data.bemf_u,
    g_telemetry_data.bemf_v,
    g_telemetry_data.bemf_w);
}

static void Telemetry_OutputADC(void)
{
  SerialProtocol_SendADC(
    g_telemetry_data.bus_voltage,
    g_telemetry_data.temperature,
    g_telemetry_data.potentiometer);
}

static void Telemetry_OutputCombined(void)
{
  SerialProtocol_SendTelemetryStream(
    g_telemetry_data.motor_state,
    g_telemetry_data.speed_rpm,
    g_telemetry_data.bemf_u,
    g_telemetry_data.bemf_v,
    g_telemetry_data.bemf_w,
    g_telemetry_data.bus_voltage,
    g_telemetry_data.temperature,
    g_telemetry_data.potentiometer);
}

/**
 * @brief Get current time in milliseconds
 * Uses SysTick timer (HAL_GetTick())
 */
static uint32_t Telemetry_GetTime_ms(void)
{
  return HAL_GetTick();
}
