/*
 * auto_controller.h
 *
 *  Created on: Nov 11, 2026
 *      Author: hapro
 */

#ifndef SRC_AUTO_CONTROLLER_H_
#define SRC_AUTO_CONTROLLER_H_

#include "message_protocol.h"
#include "math.h"

float getMotorCtrlValue();
RWP_CTRL_MODE getCtrlState();
void  setSensorState(float, float, float, float, float);

#endif /* SRC_AUTO_CONTROLLER_H_ */
