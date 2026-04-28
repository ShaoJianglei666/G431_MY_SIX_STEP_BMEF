/**
 * @file    bemf.h
 * @brief   Back-EMF (BEMF) zero-crossing detection interface for STM32G431
 *
 * Supports two detection modes selectable at compile time:
 *   - BEMF_MODE_ADC  : Injected ADC conversion sampled at PWM centre.
 *   - BEMF_MODE_COMP : On-chip comparators (COMP1/COMP2/COMP3).
 *
 * Zero-crossing events trigger Motor_Commutate() after a configurable
 * blank time to avoid false triggers from PWM switching noise.
 */

#ifndef BEMF_H
#define BEMF_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_control.h"

/* ─── Detection mode selection ────────────────────────────────────────────── */

/**
 * @defgroup BEMF_MODE Detection mode
 * Define exactly one of these in your project (e.g. compiler -D flag or here).
 * @{
 */
#ifndef BEMF_MODE_ADC
  #ifndef BEMF_MODE_COMP
    #define BEMF_MODE_ADC   /**< Default: ADC-based detection */
  #endif
#endif
/** @} */

/* ─── Timing parameters ────────────────────────────────────────────────────── */

/**
 * @brief PWM cycles to blank the BEMF input after each commutation event.
 *
 * Prevents false zero-crossing detection caused by PWM switching transients
 * (freewheeling diode recovery).  Typical value: 3–6 PWM periods.
 */
#define BEMF_BLANK_CYCLES       4U

/**
 * @brief Number of consecutive samples that must agree to confirm a crossing.
 *
 * Increases noise immunity at the cost of a small phase delay.
 */
#define BEMF_CONFIRM_COUNT      2U

/* ─── ADC-specific parameters (BEMF_MODE_ADC) ─────────────────────────────── */

/** ADC resolution in bits (12-bit on STM32G431) */
#define BEMF_ADC_RESOLUTION_BITS    12U

/** Full-scale ADC count */
#define BEMF_ADC_FULL_SCALE         ((1U << BEMF_ADC_RESOLUTION_BITS) - 1U)

/**
 * @brief Virtual neutral (star-point) voltage estimate.
 *
 * For a floating star-point the neutral is approximated as
 * (V_A + V_B + V_C) / 3.  This macro computes the integer equivalent.
 */
#define BEMF_NEUTRAL(a, b, c)   (((uint32_t)(a) + (b) + (c)) / 3U)

/* ─── BEMF handle ──────────────────────────────────────────────────────────── */

/**
 * @brief Expected polarity of the next zero-crossing.
 */
typedef enum {
    BEMF_CROSSING_RISING  = 0, /**< BEMF crossing from below neutral to above */
    BEMF_CROSSING_FALLING = 1  /**< BEMF crossing from above neutral to below */
} BemfCrossingPolarity_t;

/**
 * @brief BEMF detector internal state.
 */
typedef struct {
    uint8_t                blankCyclesLeft; /**< Remaining PWM blank cycles   */
    uint8_t                confirmCount;    /**< Consecutive matching samples  */
    BemfCrossingPolarity_t expectedPolarity;/**< Expected crossing direction   */
    bool                   crossingDetected;/**< Set when crossing is confirmed*/
    uint32_t               lastCrossingTick;/**< SysTick at last crossing      */
    uint32_t               periodUs;        /**< Last commutation period (µs)  */
} BemfHandle_t;

/* ─── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the BEMF detector.
 *
 * Configures ADC injected channels or on-chip comparators depending on
 * the selected BEMF_MODE, and arms the first blank window.
 *
 * @param  hBemf   BEMF handle to initialise.
 * @param  hMotor  Associated motor handle (read for step/state context).
 */
void Bemf_Init(BemfHandle_t *hBemf, const MotorHandle_t *hMotor);

/**
 * @brief  Notify the BEMF detector that a commutation just occurred.
 *
 * Resets the blank timer, latches the expected crossing polarity for the
 * new floating phase, and clears the previous crossing flag.
 *
 * @param  hBemf   BEMF handle.
 * @param  hMotor  Motor handle (used for current step and state).
 */
void Bemf_OnCommutation(BemfHandle_t *hBemf, const MotorHandle_t *hMotor);

/**
 * @brief  Process one BEMF ADC sample set.
 *
 * Call from the ADC injected end-of-conversion ISR (or TIM1 update ISR
 * when using comparators).  Returns true if a zero-crossing was confirmed.
 *
 * @param  hBemf   BEMF handle.
 * @param  hMotor  Motor handle.
 * @param  adcA    ADC count for phase A.
 * @param  adcB    ADC count for phase B.
 * @param  adcC    ADC count for phase C.
 * @return true if a zero-crossing was confirmed this call.
 */
bool Bemf_Process(BemfHandle_t *hBemf, const MotorHandle_t *hMotor,
                  uint16_t adcA, uint16_t adcB, uint16_t adcC);

/**
 * @brief  Decrement the blank counter by one PWM cycle.
 *
 * Call once per PWM period from TIM1 update ISR.
 *
 * @param  hBemf  BEMF handle.
 */
void Bemf_DecrementBlank(BemfHandle_t *hBemf);

/**
 * @brief  Return true if the blank window has elapsed.
 *
 * @param  hBemf  BEMF handle.
 * @return true when BEMF detection is active (blank = 0).
 */
bool Bemf_IsActive(const BemfHandle_t *hBemf);

/**
 * @brief  Return the last measured commutation period in microseconds.
 *
 * @param  hBemf  BEMF handle.
 * @return Period in µs, or 0 if not yet available.
 */
uint32_t Bemf_GetPeriodUs(const BemfHandle_t *hBemf);

#endif /* BEMF_H */
