/**
 * @file    main.h
 * @brief   Main application header – STM32G431 six-step BLDC motor template
 *
 * Pin mapping (adjust to your hardware):
 *
 *   TIM1_CH1 / TIM1_CH1N  →  PA8  / PB13   Phase A high / low
 *   TIM1_CH2 / TIM1_CH2N  →  PA9  / PB14   Phase B high / low
 *   TIM1_CH3 / TIM1_CH3N  →  PA10 / PB15   Phase C high / low
 *
 *   ADC1_IN1  →  PA0   Phase A BEMF voltage divider
 *   ADC1_IN2  →  PA1   Phase B BEMF voltage divider
 *   ADC1_IN3  →  PA2   Phase C BEMF voltage divider
 *
 *   USER_BTN  →  PC13  Start / stop button (active low)
 *   LED       →  PA5   Status LED
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>

/* ─── STM32G431 HAL include ────────────────────────────────────────────────── */
/* Uncomment the line below when integrating with STM32CubeIDE / HAL library   */
/* #include "stm32g4xx_hal.h"                                                  */

/* ─── Application version ─────────────────────────────────────────────────── */
#define FW_VERSION_MAJOR    1U
#define FW_VERSION_MINOR    0U
#define FW_VERSION_PATCH    0U

/* ─── GPIO pin helpers (adapt to your HAL / register-level BSP) ───────────── */

/** LED GPIO port / pin */
#define LED_GPIO_PORT       GPIOA
#define LED_GPIO_PIN        GPIO_PIN_5

/** User button GPIO port / pin */
#define BTN_GPIO_PORT       GPIOC
#define BTN_GPIO_PIN        GPIO_PIN_13

/* ─── Error handler ────────────────────────────────────────────────────────── */

/**
 * @brief  Infinite-loop error handler.
 *
 * Called on unrecoverable errors.  Override by providing your own
 * implementation in main.c.
 */
void Error_Handler(void);

#endif /* MAIN_H */
