/*
 * bno_driver.h
 *
 *  Created on: Sep 14, 2026
 *      Author: hapro
 */

#ifndef INC_BNO_DRIVER_H_
#define INC_BNO_DRIVER_H_


#include "main.h"
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <math.h>


typedef enum {
  BNO_UNINITIALIZED,
  BNO_BOOTING,
  BNO_INIT_SLEEP,
  BNO_CONFIGURED
} BNO_STATE;


// BLOCKING procedures to do things on the BNO085
void BNO_startup();

void BNO_getProductID();

uint8_t BNO_readFRSrecord(uint16_t record_num, uint16_t read_offset, uint16_t read_len, uint8_t* out_buf);
uint8_t BNO_writeFRSrecord(uint16_t record_num, uint16_t write_offset, uint16_t write_len, const uint8_t* out_buf);


// IRQ callback to receive a rotation vector from BNO085
void BNO_IRQcallback();

typedef enum {
  DISABLE_CALIBRATION = 0x00,
  ACCEL_ENABLE = 0x01,
  GYRO_ENABLE  = 0x02,
  MAG_ENABLE   = 0x04,
  PLANAR_CAL   = 0x08

} CALIBRATION_FLAG;

uint8_t BNO_setCalibration(CALIBRATION_FLAG);

void BNO_setFeature();

void BNO_startRotationVector();

void BNO_getYPR(float* y, float* p, float* r);


#endif /* INC_BNO_DRIVER_H_ */
