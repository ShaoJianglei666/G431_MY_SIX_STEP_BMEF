/**
 * @file    motor_control.h
 * @brief   Six-step BLDC motor control interface for STM32G431
 *
 * Implements sensorless six-step commutation using Back-EMF (BEMF)
 * zero-crossing detection on STM32G431 with TIM1 advanced PWM and ADC.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* ─── Motor electrical parameters ─────────────────────────────────────────── */

/** Supply voltage (mV) */
#define MOTOR_SUPPLY_VOLTAGE_MV     24000U

/** Pole pairs */
#define MOTOR_POLE_PAIRS            4U

/** Phase resistance (mΩ) */
#define MOTOR_PHASE_RESISTANCE_MOHM 500U

/* ─── PWM / timer configuration ───────────────────────────────────────────── */

/** TIM1 clock frequency after prescaler (Hz) – 170 MHz default */
#define PWM_TIMER_CLOCK_HZ          170000000U

/** PWM switching frequency (Hz) */
#define PWM_FREQUENCY_HZ            20000U

/** PWM auto-reload value derived from clock and frequency */
#define PWM_ARR_VALUE               (PWM_TIMER_CLOCK_HZ / PWM_FREQUENCY_HZ - 1U)

/** Minimum duty cycle (0–1000 per-mille) */
#define PWM_DUTY_MIN_PERMILLE       50U

/** Maximum duty cycle (0–1000 per-mille) */
#define PWM_DUTY_MAX_PERMILLE       950U

/* ─── Startup (open-loop) parameters ──────────────────────────────────────── */

/** Initial commutation period for open-loop start (µs) */
#define STARTUP_COMM_PERIOD_INIT_US 5000U

/** Minimum commutation period before switching to closed-loop (µs) */
#define STARTUP_COMM_PERIOD_MIN_US  1000U

/** Duty cycle applied during open-loop startup (per-mille) */
#define STARTUP_DUTY_PERMILLE       200U

/** Number of successful BEMF zero-crossings required to enter closed-loop */
#define STARTUP_BEMF_VALID_COUNT    6U

/* ─── Commutation step definitions ────────────────────────────────────────── */

/** Number of electrical steps per revolution */
#define MOTOR_STEP_COUNT            6U

/**
 * @brief Phase output state for a single step.
 *
 * Each phase can be HIGH (PWM), LOW (GND), or FLOATING (Hi-Z).
 */
typedef enum {
    PHASE_HIGH     = 0, /**< Connected to high-side switch (PWM output) */
    PHASE_LOW      = 1, /**< Connected to low-side switch (GND)         */
    PHASE_FLOATING = 2  /**< Disconnected – BEMF monitored here         */
} PhaseState_t;

/**
 * @brief Output state for all three phases in one commutation step.
 */
typedef struct {
    PhaseState_t phA; /**< Phase A state */
    PhaseState_t phB; /**< Phase B state */
    PhaseState_t phC; /**< Phase C state */
} CommStep_t;

/* ─── Motor state machine ──────────────────────────────────────────────────── */

/**
 * @brief Motor controller operating states.
 */
typedef enum {
    MOTOR_STATE_IDLE    = 0, /**< Motor stopped, outputs disabled   */
    MOTOR_STATE_STARTUP = 1, /**< Open-loop acceleration phase      */
    MOTOR_STATE_RUNNING = 2, /**< Closed-loop BEMF commutation      */
    MOTOR_STATE_FAULT   = 3  /**< Fault – outputs disabled          */
} MotorState_t;

/**
 * @brief Fault type flags (can be OR-ed together).
 */
typedef enum {
    MOTOR_FAULT_NONE        = 0x00U, /**< No fault                          */
    MOTOR_FAULT_OVERCURRENT = 0x01U, /**< Phase overcurrent detected        */
    MOTOR_FAULT_STALL       = 0x02U, /**< Motor stall (no BEMF detected)    */
    MOTOR_FAULT_UNDERVOLTAGE= 0x04U  /**< Supply voltage too low            */
} MotorFault_t;

/**
 * @brief Central motor control handle.
 *
 * All motor state is contained in this structure so that the same code
 * can drive multiple motor instances if needed.
 */
typedef struct {
    MotorState_t  state;           /**< Current operating state           */
    MotorFault_t  fault;           /**< Active fault flags                */
    uint8_t       step;            /**< Current commutation step (0–5)    */
    uint16_t      dutyPermille;    /**< Active duty cycle (0–1000)        */
    uint16_t      targetDuty;      /**< Requested duty cycle              */
    uint32_t      commPeriodUs;    /**< Last measured commutation period  */
    uint32_t      speedRpm;        /**< Estimated mechanical speed (RPM)  */
    uint8_t       bemfValidCount;  /**< Consecutive valid BEMF crossings  */
} MotorHandle_t;

/* ─── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the motor controller.
 *
 * Must be called once after peripheral initialisation (TIM1, ADC, GPIO)
 * and before any other motor API.
 *
 * @param  hMotor  Pointer to the motor handle to initialise.
 */
void Motor_Init(MotorHandle_t *hMotor);

/**
 * @brief  Start the motor at the requested duty cycle.
 *
 * Transitions the motor from IDLE → STARTUP.
 *
 * @param  hMotor      Motor handle.
 * @param  dutyPermille Target duty cycle (PWM_DUTY_MIN_PERMILLE –
 *                      PWM_DUTY_MAX_PERMILLE).
 */
void Motor_Start(MotorHandle_t *hMotor, uint16_t dutyPermille);

/**
 * @brief  Stop the motor and disable all phase outputs.
 *
 * Transitions to IDLE state.
 *
 * @param  hMotor  Motor handle.
 */
void Motor_Stop(MotorHandle_t *hMotor);

/**
 * @brief  Update the target duty cycle while the motor is running.
 *
 * @param  hMotor      Motor handle.
 * @param  dutyPermille New target duty cycle.
 */
void Motor_SetDuty(MotorHandle_t *hMotor, uint16_t dutyPermille);

/**
 * @brief  Advance to the next commutation step.
 *
 * Applies the next entry in the six-step table and updates PWM outputs.
 * Called from the BEMF zero-crossing callback or the open-loop timer ISR.
 *
 * @param  hMotor  Motor handle.
 */
void Motor_Commutate(MotorHandle_t *hMotor);

/**
 * @brief  Periodic motor control task – call at 1 ms intervals.
 *
 * Handles open-loop startup timing, speed estimation, ramp, and fault
 * monitoring.  Must be called from a 1 ms SysTick handler or equivalent.
 *
 * @param  hMotor  Motor handle.
 */
void Motor_Task1ms(MotorHandle_t *hMotor);

/**
 * @brief  Clear any latched fault and return to IDLE.
 *
 * @param  hMotor  Motor handle.
 */
void Motor_ClearFault(MotorHandle_t *hMotor);

/**
 * @brief  Return the current mechanical speed estimate.
 *
 * @param  hMotor  Motor handle.
 * @return Speed in RPM.
 */
uint32_t Motor_GetSpeedRpm(const MotorHandle_t *hMotor);

/**
 * @brief  Return the current motor state.
 *
 * @param  hMotor  Motor handle.
 * @return Current MotorState_t value.
 */
MotorState_t Motor_GetState(const MotorHandle_t *hMotor);

#endif /* MOTOR_CONTROL_H */
