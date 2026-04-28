/**
 * @file    main.c
 * @brief   Main application – STM32G431 six-step BLDC motor template
 *
 * Integration guide
 * ─────────────────
 * 1. Open STM32CubeMX and configure:
 *      - TIM1: Centre-aligned PWM mode 1, complementary outputs CH1–CH3,
 *              dead-time, CC4 used to trigger ADC injected conversion.
 *      - ADC1: Injected channels IN1–IN3, triggered by TIM1 CC4.
 *      - SysTick: 1 ms period (default with HAL_Init).
 *      - GPIOs: PA5 LED output, PC13 button input with pull-up.
 * 2. Include this file and Core/Inc files in your project.
 * 3. Remove the stub HAL includes below and add "stm32g4xx_hal.h".
 * 4. Replace /* HAL stub */ comments with real HAL calls.
 * 5. Build and flash.
 *
 * Control flow
 * ────────────
 * - Press USER_BTN (PC13) to start the motor at DEFAULT_DUTY_PERMILLE.
 * - Press again to stop.
 * - The LED blinks at 1 Hz while the motor is running.
 * - A fault (overcurrent / stall) turns the LED on solid; press the button
 *   to clear the fault.
 */

#include "main.h"
#include "motor_control.h"
#include "bemf.h"

/* ─── Default operating point ─────────────────────────────────────────────── */

/** Default target duty cycle when the motor is started via button */
#define DEFAULT_DUTY_PERMILLE   400U

/* ─── Global handles ──────────────────────────────────────────────────────── */

static MotorHandle_t g_motor;
static BemfHandle_t  g_bemf;

/* ─── Button debounce ─────────────────────────────────────────────────────── */

static uint32_t s_btnDebounceMs  = 0U;
static bool     s_btnLastPressed = false;

/* ─── LED blink ───────────────────────────────────────────────────────────── */

static uint32_t s_ledTickMs = 0U;

/* ─── 1 ms tick counter (stub – replace with HAL_GetTick()) ──────────────── */

static volatile uint32_t s_sysTick = 0U;

/**
 * @brief Return current system tick in milliseconds.
 *
 * Replace with HAL_GetTick() when using STM32 HAL.
 */
static uint32_t GetTickMs(void)
{
    /* return HAL_GetTick(); */
    return s_sysTick;
}

/* ─── Peripheral initialisation stubs ────────────────────────────────────── */

/**
 * @brief System clock configuration stub.
 *
 * In a CubeMX project this is generated as SystemClock_Config().
 * Target: PLL from HSI16 → 170 MHz SYSCLK for STM32G431.
 */
static void SystemClock_Config(void)
{
    /*
     * RCC_OscInitTypeDef RCC_OscInitStruct = {0};
     * RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
     *
     * RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
     * RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
     * RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
     * RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
     * RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
     * RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV4;
     * RCC_OscInitStruct.PLL.PLLN       = 85;
     * RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
     * RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
     * RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
     * HAL_RCC_OscConfig(&RCC_OscInitStruct);
     *
     * RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | ...;
     * RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
     * RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
     * RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
     * RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
     * HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
     */
}

/**
 * @brief GPIO initialisation stub.
 */
static void MX_GPIO_Init(void)
{
    /*
     * __HAL_RCC_GPIOA_CLK_ENABLE();
     * __HAL_RCC_GPIOC_CLK_ENABLE();
     *
     * // LED PA5
     * GPIO_InitTypeDef GPIO_InitStruct = {0};
     * GPIO_InitStruct.Pin   = LED_GPIO_PIN;
     * GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
     * GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
     * HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
     *
     * // Button PC13 (active low)
     * GPIO_InitStruct.Pin  = BTN_GPIO_PIN;
     * GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
     * GPIO_InitStruct.Pull = GPIO_PULLUP;
     * HAL_GPIO_Init(BTN_GPIO_PORT, &GPIO_InitStruct);
     */
}

/**
 * @brief TIM1 PWM initialisation stub.
 *
 * Centre-aligned, complementary outputs, dead-time insertion.
 * CC4 used to trigger ADC injected conversions at PWM centre.
 */
static void MX_TIM1_Init(void)
{
    /*
     * TIM_ClockConfigTypeDef  sClockSourceConfig  = {0};
     * TIM_MasterConfigTypeDef sMasterConfig        = {0};
     * TIM_OC_InitTypeDef      sConfigOC            = {0};
     * TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
     *
     * htim1.Instance               = TIM1;
     * htim1.Init.Prescaler         = 0;
     * htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
     * htim1.Init.Period            = PWM_ARR_VALUE;
     * htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
     * htim1.Init.RepetitionCounter = 1;   // update event every 2 PWM periods
     * htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
     * HAL_TIM_PWM_Init(&htim1);
     *
     * sConfigOC.OCMode     = TIM_OCMODE_PWM1;
     * sConfigOC.Pulse      = 0;
     * sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
     * sConfigOC.OCNPolarity= TIM_OCNPOLARITY_HIGH;
     * sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
     * sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
     * sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
     * HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
     * HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
     * HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
     *
     * // CC4 at ARR/2 to trigger ADC at PWM centre (avoid switching noise)
     * sConfigOC.OCMode = TIM_OCMODE_PWM2;
     * sConfigOC.Pulse  = PWM_ARR_VALUE / 2U;
     * HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
     *
     * sBreakDeadTimeConfig.OffStateRunMode   = TIM_OSSR_ENABLE;
     * sBreakDeadTimeConfig.OffStateIDLEMode  = TIM_OSSI_ENABLE;
     * sBreakDeadTimeConfig.LockLevel         = TIM_LOCKLEVEL_OFF;
     * sBreakDeadTimeConfig.DeadTime          = 20;  // ~117 ns @ 170 MHz
     * sBreakDeadTimeConfig.BreakState        = TIM_BREAK_ENABLE;
     * sBreakDeadTimeConfig.BreakPolarity     = TIM_BREAKPOLARITY_HIGH;
     * sBreakDeadTimeConfig.AutomaticOutput   = TIM_AUTOMATICOUTPUT_DISABLE;
     * HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
     *
     * HAL_TIM_Base_Start(&htim1);
     */
}

/**
 * @brief ADC1 initialisation stub.
 *
 * Injected group: IN1 (PA0), IN2 (PA1), IN3 (PA2) triggered by TIM1 CC4.
 */
static void MX_ADC1_Init(void)
{
    /*
     * hadc1.Instance = ADC1;
     * hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
     * hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
     * hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
     * hadc1.Init.GainCompensation      = 0;
     * hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
     * hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
     * hadc1.Init.LowPowerAutoWait      = DISABLE;
     * hadc1.Init.ContinuousConvMode    = DISABLE;
     * hadc1.Init.NbrOfConversion       = 1;
     * hadc1.Init.DiscontinuousConvMode = DISABLE;
     * hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T1_CC4;
     * hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
     * hadc1.Init.DMAContinuousRequests = DISABLE;
     * hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
     * HAL_ADC_Init(&hadc1);
     *
     * ADC_InjectionConfTypeDef sConfigInjected = {0};
     * sConfigInjected.InjectedChannel               = ADC_CHANNEL_1;
     * sConfigInjected.InjectedRank                  = ADC_INJECTED_RANK_1;
     * sConfigInjected.InjectedSamplingTime          = ADC_SAMPLETIME_2CYCLES_5;
     * sConfigInjected.InjectedSingleDiff            = ADC_SINGLE_ENDED;
     * sConfigInjected.InjectedOffsetNumber          = ADC_OFFSET_NONE;
     * sConfigInjected.InjectedNbrOfConversion       = 3;
     * sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
     * sConfigInjected.AutoInjectedConv              = DISABLE;
     * sConfigInjected.QueueInjectedContext          = DISABLE;
     * sConfigInjected.ExternalTrigInjecConv         = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
     * sConfigInjected.ExternalTrigInjecConvEdge     = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
     * HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);
     * // Repeat for RANK_2 (IN2) and RANK_3 (IN3) ...
     *
     * HAL_ADCEx_InjectedStart_IT(&hadc1);
     */
}

/* ─── Interrupt handlers ──────────────────────────────────────────────────── */

/**
 * @brief SysTick handler – called every 1 ms.
 *
 * In STM32 HAL this is HAL_IncTick() + any application tick work.
 * Register this via the HAL_SYSTICK_Callback() weak function or
 * override SysTick_Handler() in your project.
 */
void App_SysTick_Callback(void)
{
    s_sysTick++;
    Motor_Task1ms(&g_motor);
}

/**
 * @brief ADC injected conversion complete callback.
 *
 * Called from HAL_ADCEx_InjectedConvCpltCallback() in your adc.c.
 *
 * @param hadc  ADC handle (unused – only one ADC in this template).
 */
void App_ADC_InjectedConvCpltCallback(void)
{
    /*
     * uint16_t adcA = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
     * uint16_t adcB = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
     * uint16_t adcC = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
     */
    uint16_t adcA = 0U; /* stub */
    uint16_t adcB = 0U; /* stub */
    uint16_t adcC = 0U; /* stub */

    Bemf_DecrementBlank(&g_bemf);

    if (g_motor.state == MOTOR_STATE_RUNNING ||
        g_motor.state == MOTOR_STATE_STARTUP)
    {
        bool crossing = Bemf_Process(&g_bemf, &g_motor, adcA, adcB, adcC);
        if (crossing)
        {
            g_motor.commPeriodUs = Bemf_GetPeriodUs(&g_bemf);

            if (g_motor.state == MOTOR_STATE_STARTUP)
            {
                g_motor.bemfValidCount++;
            }

            Motor_Commutate(&g_motor);
            Bemf_OnCommutation(&g_bemf, &g_motor);
        }
    }
}

/**
 * @brief TIM1 update event callback – decrement BEMF blank counter.
 *
 * Registered from HAL_TIM_PeriodElapsedCallback() in your tim.c.
 */
void App_TIM1_UpdateCallback(void)
{
    Bemf_DecrementBlank(&g_bemf);
}

/* ─── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    /* 1. HAL and system clock initialisation */
    /* HAL_Init(); */
    SystemClock_Config();

    /* 2. Peripheral initialisation */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_ADC1_Init();

    /* 3. Motor and BEMF initialisation */
    Motor_Init(&g_motor);
    Bemf_Init(&g_bemf, &g_motor);

    /* 4. Main loop */
    while (1)
    {
        uint32_t now = GetTickMs();

        /* ── Button handling (debounced, active low) ── */
        bool btnPressed = false; /* stub: HAL_GPIO_ReadPin(BTN_GPIO_PORT, BTN_GPIO_PIN) == GPIO_PIN_RESET */

        if (btnPressed && !s_btnLastPressed &&
            (now - s_btnDebounceMs) > 50U)
        {
            s_btnDebounceMs = now;

            MotorState_t state = Motor_GetState(&g_motor);
            if (state == MOTOR_STATE_FAULT)
            {
                Motor_ClearFault(&g_motor);
            }
            else if (state == MOTOR_STATE_IDLE)
            {
                Bemf_OnCommutation(&g_bemf, &g_motor);
                Motor_Start(&g_motor, DEFAULT_DUTY_PERMILLE);
            }
            else
            {
                Motor_Stop(&g_motor);
            }
        }
        s_btnLastPressed = btnPressed;

        /* ── LED status indicator ── */
        if ((now - s_ledTickMs) >= 500U)
        {
            s_ledTickMs = now;
            MotorState_t state = Motor_GetState(&g_motor);
            if (state == MOTOR_STATE_RUNNING || state == MOTOR_STATE_STARTUP)
            {
                /* Blink LED at 1 Hz while motor runs */
                /* HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN); */
            }
            else if (state == MOTOR_STATE_FAULT)
            {
                /* LED on solid for fault */
                /* HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET); */
            }
            else
            {
                /* LED off when idle */
                /* HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET); */
            }
        }
    }
}

/* ─── Error handler ────────────────────────────────────────────────────────── */

void Error_Handler(void)
{
    /* Disable interrupts and loop forever */
    /* __disable_irq(); */
    while (1)
    {
        /* HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET); */
    }
}
