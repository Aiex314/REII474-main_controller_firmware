/*
 * motor_controller.c
 *
 *  Created on: Oct 9, 2026
 *      Author: hapro
 */

#include "motor_controller.h"

// Maximum motor pwm counter reset value
const float MAX_PWM_CRR = 5000.0f;

// When 0, setMotorOutput works as normal.
// When 1, setMotorOutput is disabled until stopBeep is called.
static uint8_t beeping = 0;

// This configures the motor's PWM frequency to the specified value
// and then drives the motor lightly (5% duty cycle).
// This makes the motor beep!
void setMotorBeep(float freq) {
  // While beeping, other code should never set the motor speed
  beeping = 1;

  // Maximum PWM freq is 100kHz. 5% of this is 5kHz
  if (freq > 5000) return;
  // Minimum is 1.5259...Hz
  if (freq < 1.5259) return;

  // Calculate required ARR value for beeping
  uint16_t beep_val = (uint16_t)floor(100000/freq);

  // Set PWM registers
  TIM4->PSC  = 999;         // Constant prescaler (yields freq range of ~1.5Hz-5kHz)
  TIM4->CCR2 = beep_val/20; // 5% duty cycle
  TIM4->ARR  = beep_val;    // Set desired PWM frequency
}

// Restores the PWM registers to their original values
void stopBeep() {
  TIM4->CCR2 = 0;
  TIM4->PSC  = 0;
  TIM4->ARR  = 4999;

  // It is now safe to drive the motor again
  beeping = 0;
}

// Sets the motor output. v ranges from -1.0 to 1.0, meaning
// 100% CW through to 100% CCW.
void setMotorOutput(float v) {
  // If the motor is beeping, don't drive the motor.
  if (beeping) return;

  // Convert from float to a register value
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


// 12-TET chromatic scale from A4 to A5
#define nA4  440.00
#define nB4  493.88
#define nC4  523.25
#define nD4  587.33
#define nE4  659.26
#define nF4  698.46
#define nG4  783.99
#define nAs4 466.16
#define nCs4 554.37
#define nDs4 622.25
#define nFs4 739.99
#define nGs4 830.61
#define nA5  880.00

// Plays a tune using the motor as a beeper
void playStartupTune() {
  HAL_GPIO_WritePin(MOT_fwd_GPIO_Port, MOT_fwd_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOT_bwd_GPIO_Port, MOT_bwd_Pin, GPIO_PIN_SET);
  // Rising A maj
  setMotorBeep(nA4);
  HAL_Delay(120);
  setMotorBeep(nCs4);
  HAL_Delay(120);
  setMotorBeep(nE4);
  HAL_Delay(120);
  setMotorBeep(nA5);
  HAL_Delay(120);
  stopBeep();
  HAL_Delay(240);
  HAL_GPIO_WritePin(MOT_fwd_GPIO_Port, MOT_fwd_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOT_bwd_GPIO_Port, MOT_bwd_Pin, GPIO_PIN_RESET);
  // Falling A maj
  setMotorBeep(nA5);
  HAL_Delay(120);
  setMotorBeep(nE4);
  HAL_Delay(120);
  setMotorBeep(nCs4);
  HAL_Delay(120);
  setMotorBeep(nA4);
  HAL_Delay(120);
  stopBeep();
}
