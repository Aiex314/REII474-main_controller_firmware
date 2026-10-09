/*
 * motor_controller.c
 *
 *  Created on: Oct 9, 2026
 *      Author: hapro
 */

#include "motor_controller.h"

static uint8_t beeping = 0;

// Maximum motor pwm counter reset value
const float MAX_PWM_CRR = 5000.0f;

void setMotorBeep(float freq) {
  beeping = 1;

  if (freq > 10000) return;
  if (freq < 1.5259) return;

  uint16_t beep_val = (uint16_t)floor(100000/freq);

  // Set beeper
  TIM4->PSC  = 999;
  TIM4->CCR2 = beep_val/20;
  TIM4->ARR  = beep_val;
}

void stopBeep() {
  beeping = 0;

  // Stop beeper (we're just setting it to 20kHz - the default)
  TIM4->CCR2 = 0;
  TIM4->PSC  = 0;
  TIM4->ARR  = 4999;
}

void setMotorOutput(float v) {
  if (beeping) return;

  int16_t motor_output = (int16_t)(MAX_PWM_CRR*v);

  // Input clipping and taking absolute
  if (v > 1.0f || v < -1.0f)
    motor_output = MAX_PWM_CRR;
  else if (v < 0.0f)
    motor_output = -motor_output;

  // Set the CRR value
  TIM4->CCR2 = motor_output;

  // Set direction pins
  if (v > 0.0f) {
    HAL_GPIO_WritePin(MOT_fwd_GPIO_Port, MOT_fwd_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOT_bwd_GPIO_Port, MOT_bwd_Pin, GPIO_PIN_RESET);
  }
  else {
    HAL_GPIO_WritePin(MOT_fwd_GPIO_Port, MOT_fwd_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOT_bwd_GPIO_Port, MOT_bwd_Pin, GPIO_PIN_SET);
  }
}
