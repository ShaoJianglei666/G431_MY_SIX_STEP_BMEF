/**
 * @file    bemf.c
 * @brief   BEMF zero-crossing detection implementation for STM32G431
 *
 * Supports ADC-based detection (BEMF_MODE_ADC) where injected ADC
 * conversions are triggered at the centre of each PWM period by TIM1 CC4.
 * The three phase voltages are compared against the virtual neutral to
 * identify zero-crossings on the floating phase.
 *
 * To use comparator-based detection (BEMF_MODE_COMP), swap the
 * BEMF_MODE_ADC define for BEMF_MODE_COMP and adapt Bemf_Init() to
 * configure COMP1/COMP2/COMP3 with TIM1 OCREF clear.
 */

#include "bemf.h"
#include <string.h>

/* ─── Expected crossing polarity per commutation step ─────────────────────── */
/*
 * For each step the floating phase is:
 *   Step 0: C floating – expect FALLING crossing (motor spinning fwd)
 *   Step 1: B floating – expect RISING  crossing
 *   Step 2: A floating – expect FALLING crossing
 *   Step 3: C floating – expect RISING  crossing
 *   Step 4: B floating – expect FALLING crossing
 *   Step 5: A floating – expect RISING  crossing
 *
 * "Rising"  = BEMF goes from below virtual neutral to above neutral.
 * "Falling" = BEMF goes from above virtual neutral to below neutral.
 */
static const BemfCrossingPolarity_t s_expectedPolarity[MOTOR_STEP_COUNT] = {
    BEMF_CROSSING_FALLING, /* Step 0 – floating C */
    BEMF_CROSSING_RISING,  /* Step 1 – floating B */
    BEMF_CROSSING_FALLING, /* Step 2 – floating A */
    BEMF_CROSSING_RISING,  /* Step 3 – floating C */
    BEMF_CROSSING_FALLING, /* Step 4 – floating B */
    BEMF_CROSSING_RISING,  /* Step 5 – floating A */
};

/* ─── Internal helpers ─────────────────────────────────────────────────────── */

/**
 * @brief Return the ADC value of the floating phase for the current step.
 *
 * @param step  Current commutation step (0–5).
 * @param adcA  Phase A ADC count.
 * @param adcB  Phase B ADC count.
 * @param adcC  Phase C ADC count.
 * @return ADC count of the floating phase.
 */
static uint16_t GetFloatingPhaseAdc(uint8_t step,
                                    uint16_t adcA,
                                    uint16_t adcB,
                                    uint16_t adcC)
{
    /* Floating phase per step (see commutation table in motor_control.c):
     *   Step 0, 3 → C
     *   Step 1, 4 → B
     *   Step 2, 5 → A
     */
    switch (step % MOTOR_STEP_COUNT)
    {
    case 0U: /* fall-through */
    case 3U: return adcC;
    case 1U: /* fall-through */
    case 4U: return adcB;
    case 2U: /* fall-through */
    case 5U: return adcA;
    default: return adcA;
    }
}

/**
 * @brief Return a simple 32-bit system tick (milliseconds).
 *
 * Replace with HAL_GetTick() when integrating with STM32 HAL.
 */
static uint32_t GetTick(void)
{
    /* return HAL_GetTick(); */
    return 0U; /* stub */
}

/* ─── Public API ───────────────────────────────────────────────────────────── */

void Bemf_Init(BemfHandle_t *hBemf, const MotorHandle_t *hMotor)
{
    memset(hBemf, 0, sizeof(*hBemf));
    hBemf->blankCyclesLeft   = BEMF_BLANK_CYCLES;
    hBemf->confirmCount      = 0U;
    hBemf->crossingDetected  = false;
    hBemf->lastCrossingTick  = 0U;
    hBemf->periodUs          = 0U;

    if (hMotor->step < MOTOR_STEP_COUNT)
    {
        hBemf->expectedPolarity = s_expectedPolarity[hMotor->step];
    }
    else
    {
        hBemf->expectedPolarity = BEMF_CROSSING_RISING;
    }

    /*
     * ADC / comparator peripheral initialisation stubs.
     *
     * For BEMF_MODE_ADC:
     *   - Configure ADC1 injected channels IN1, IN2, IN3 (PA0, PA1, PA2).
     *   - Set injected trigger source to TIM1_CC4.
     *   - Set sampling time to e.g. 2.5 ADC cycles for fast conversion.
     *   - HAL_ADCEx_InjectedStart_IT(&hadc1);
     *
     * For BEMF_MODE_COMP:
     *   - Configure COMP1/COMP2/COMP3 non-inverting input to phase voltage.
     *   - Set inverting input to virtual neutral (DAC or resistor divider).
     *   - Enable COMP output to TIM1 OCREF clear for automatic blanking.
     *   - HAL_COMP_Start(&hcomp1); etc.
     */
}

void Bemf_OnCommutation(BemfHandle_t *hBemf, const MotorHandle_t *hMotor)
{
    hBemf->blankCyclesLeft  = BEMF_BLANK_CYCLES;
    hBemf->confirmCount     = 0U;
    hBemf->crossingDetected = false;

    uint8_t nextStep = (hMotor->step < MOTOR_STEP_COUNT)
                       ? hMotor->step
                       : 0U;
    hBemf->expectedPolarity = s_expectedPolarity[nextStep];
}

bool Bemf_Process(BemfHandle_t *hBemf, const MotorHandle_t *hMotor,
                  uint16_t adcA, uint16_t adcB, uint16_t adcC)
{
    /* Don't process during the blank window */
    if (hBemf->blankCyclesLeft > 0U) { return false; }

    /* Already confirmed a crossing – wait for next commutation */
    if (hBemf->crossingDetected) { return false; }

    /*
     * NOTE: For motors running at high speed the commutation period may be
     * shorter than 1 ms.  For accurate period measurement replace GetTick()
     * (1 ms resolution) with a 1 µs free-running timer counter.
     */
    uint32_t neutral    = BEMF_NEUTRAL(adcA, adcB, adcC);
    uint16_t floatAdc   = GetFloatingPhaseAdc(hMotor->step, adcA, adcB, adcC);
    bool crossingNow    = false;

    if (hBemf->expectedPolarity == BEMF_CROSSING_RISING)
    {
        /* Rising: floating phase voltage crosses neutral from below */
        crossingNow = ((uint32_t)floatAdc >= neutral);
    }
    else
    {
        /* Falling: floating phase voltage crosses neutral from above */
        crossingNow = ((uint32_t)floatAdc < neutral);
    }

    if (crossingNow)
    {
        hBemf->confirmCount++;
        if (hBemf->confirmCount >= BEMF_CONFIRM_COUNT)
        {
            /* Zero-crossing confirmed */
            uint32_t now = GetTick();
            if (hBemf->lastCrossingTick != 0U)
            {
                /* Period in µs (tick is in ms – multiply by 1000).
                 * Guard against zero period when two crossings fall in the
                 * same millisecond tick (high-speed motors); a zero period
                 * would cause division-by-zero in the speed estimator.
                 * For motors where this resolution matters, replace GetTick()
                 * with a 1 µs free-running hardware counter. */
                uint32_t periodUs = (now - hBemf->lastCrossingTick) * 1000U;
                if (periodUs == 0U) { periodUs = 1U; }
                hBemf->periodUs = periodUs;
            }
            hBemf->lastCrossingTick = now;
            hBemf->crossingDetected = true;
            hBemf->confirmCount     = 0U;
            return true;
        }
    }
    else
    {
        /* Reset confirmation counter on non-crossing sample */
        hBemf->confirmCount = 0U;
    }

    return false;
}

void Bemf_DecrementBlank(BemfHandle_t *hBemf)
{
    if (hBemf->blankCyclesLeft > 0U)
    {
        hBemf->blankCyclesLeft--;
    }
}

bool Bemf_IsActive(const BemfHandle_t *hBemf)
{
    return (hBemf->blankCyclesLeft == 0U);
}

uint32_t Bemf_GetPeriodUs(const BemfHandle_t *hBemf)
{
    return hBemf->periodUs;
}
