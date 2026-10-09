/*
 * motor_controller.h
 *
 *  Created on: Oct 9, 2026
 *      Author: hapro
 */

#ifndef SRC_MOTOR_CONTROLLER_H_
#define SRC_MOTOR_CONTROLLER_H_

#include <stdint.h>
#include <math.h>

#include "main.h"

void setMotorBeep(float freq);
void stopBeep();
void setMotorOutput(float v);

#endif /* SRC_MOTOR_CONTROLLER_H_ */
