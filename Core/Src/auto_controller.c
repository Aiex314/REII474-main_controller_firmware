/*
 * auto_controller.c
 *
 *  Created on: Nov 11, 2026
 *      Author: hapro
 */


#include "auto_controller.h"


// ======================== CONSTANTS =======================
// pi
#define PI 3.14159265358979323
// Angle conversion constants
const static float DEG_TO_RAD = PI/180.0;

// Hysteresis limits for linear controller
static float LIM_LIN_START_ANGLE  = 3.5*DEG_TO_RAD;
static float LIM_LIN_START_SPEED  = 1.0;
static float LIM_LIN_STOP_ANGLE = 7.5*DEG_TO_RAD;
static float LIM_LIN_STOP_SPEED = 999.0;
static float SWING_DOWN_SPEED_MULT = 5.0;


// Physical parameters of RWP
const static float J_f = 0.000058;   // Reaction wheel moment of inertia as kg/m^2
static float J_p = 0.0240;     // Pendulum arm moment of inertia as kg/m^2
const static float K_g = 1.3726;     // Potential energy constant


// Sensor state
static float a_p = 0, a_pl = 0, v_p = 0, v_pl = 0, v_f = 0;



// State variables
static RWP_CTRL_MODE rwp_ctrl = RWP_CTRL_SWINGUP;



// Performance metrics


// This is kind of an odd function. We use this to detect
// things like the number of near misses during swingup, the
// number of times a swingup shot through the linear region without
// activating the linear controller, etc.
// This is used to tune the physical parameters on the fly.
void updatePerformance() {
  uint8_t in_lin_region = (a_pl > -LIM_LIN_START_ANGLE && a_pl < LIM_LIN_START_ANGLE);
  uint8_t pend_invert = (v_p < 0.1 && v_p > -0.1) && (a_pl < 45.0 && a_pl > -45.0);

  // Previous state
  static uint8_t in_lin_region_prev = 0;

  // Detect movement inversions while the pendulum is near the linear region
  // This constitutes a "near miss", and the pendulum should be more aggressive
  static uint8_t high_inversion = 0;
  if (rwp_ctrl == RWP_CTRL_SWINGUP && pend_invert && !in_lin_region) {
	if (high_inversion == 0) {
	  high_inversion = 1;
	  // Adjust the pendulum inertia accordingly
	  J_p *= 0.975;
	}
  }
  else {
    high_inversion = 0;
  }

  // Detect swing throughs (pendulum moves through linear region without activating lin controller)
  // The pendulum should be less aggressive during swingup.
  static uint8_t in_linreg_no_lincon = 0;
  if (in_lin_region) {
    // If we've entered the linear region, set a flag
    if (!in_lin_region_prev) in_linreg_no_lincon = 1;
    // If we've remained in the linear region, clear the flag if we actually use the linear controller
    else if (rwp_ctrl == RWP_CTRL_LINEAR) in_linreg_no_lincon = 0;
  }
  else {
    // If we've left the linear region with the flag set, a swingthrough occurred
    if (in_lin_region_prev && in_linreg_no_lincon == 1) {
      J_p *= 1.1;
    }
  }

  in_lin_region_prev = in_lin_region;
}


void updateControllerState() {
  switch (rwp_ctrl) {
    case RWP_CTRL_LINEAR:
      // If outside of speed or angle limits, disable linear controller
      if (a_pl > LIM_LIN_STOP_ANGLE || a_pl < -LIM_LIN_STOP_ANGLE || v_p > LIM_LIN_STOP_SPEED || v_p < -LIM_LIN_STOP_SPEED)
        rwp_ctrl = RWP_CTRL_SWINGUP;
      break;
    case RWP_CTRL_SWINGUP:
      // If inside of speed and angle limits, enable linear controller
      if (a_pl > -LIM_LIN_START_ANGLE && a_pl < LIM_LIN_START_ANGLE && v_p > -LIM_LIN_START_SPEED && v_p < LIM_LIN_START_SPEED)
        rwp_ctrl = RWP_CTRL_LINEAR;
      // If outside of speed limits, swing down a bit before retrying...
      else if (a_pl > -LIM_LIN_STOP_ANGLE && a_pl < LIM_LIN_STOP_ANGLE && (v_p < -SWING_DOWN_SPEED_MULT*LIM_LIN_START_SPEED || v_p > SWING_DOWN_SPEED_MULT*LIM_LIN_START_SPEED)) {
        rwp_ctrl = RWP_CTRL_SWINGDOWN;
      }
      break;
    case RWP_CTRL_SWINGDOWN:
      if (v_p < 0.1 && v_p > -0.1)
        rwp_ctrl = RWP_CTRL_SWINGUP;
      break;
    default:
      rwp_ctrl = RWP_CTRL_SWINGDOWN;
      break;
  }
}

float getMotorCtrlValue() {
  // Update controller parameters
  updatePerformance();

  // Update the auto controller state
  updateControllerState();

  // System energy estimation (all J)
  float KE_p = v_p*v_p*J_p/2.0;
  float KE_f = v_f*v_f*J_f/2.0;
  float PE_p = K_g * (cos(a_p)-1);
  float total_energy = KE_p + KE_f + PE_p;

  static float motor_output = 0;

  if (rwp_ctrl == RWP_CTRL_LINEAR)
    motor_output = -(36.62*a_pl + 5.16*v_pl + 0.044*v_f);
  else if (rwp_ctrl == RWP_CTRL_SWINGUP) {
    static double prev_output = 0.0;
    double output = 0.3*total_energy + 0.7*prev_output;
    prev_output = output;
    motor_output = -(float)(output * v_p)*(24.0/24.0);
  }
  else {
	motor_output *= 0.95;
  }

  return motor_output;
}

void setSensorState(float a_pn, float a_pln, float v_pn, float v_pln, float v_fn) {
  a_p  = a_pn;
  a_pl = a_pln;
  v_p  = v_pn;
  v_pl = v_pln;
  v_f  = v_fn;
}

RWP_CTRL_MODE getCtrlState() {
  return rwp_ctrl;
}
