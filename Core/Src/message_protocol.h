/*
 * message_protocol.h
 *
 *  Created on: Oct 6, 2026
 *      Author: hapro
 */

#ifndef SRC_MESSAGE_PROTOCOL_H_
#define SRC_MESSAGE_PROTOCOL_H_



typedef enum {
  RWP_MODE_NO_COMMS,
  RWP_MODE_HALTED,
  RWP_MODE_MANUAL,
  RWP_MODE_AUTO,
  RWP_MODE_LOCK,
  RWP_MODE_TRICKS,
  RWP_MODE_CALIBRATION,
  RWP_MODE_DEBUG
} RWP_MODE;

typedef enum {
  SENSOR_DATA,
  CONTROL_DATA,
} RWP_DATA_KIND;


// Each of the below must be exactly 32 bytes
typedef struct {
  uint8_t cur_mode;           // + 1B     = 1B
  RWP_DATA_KIND data_type;    // + 1B     = 2B
  uint8_t _pad1[2];           // + 2B     = 4B
  float yaw, roll, pitch, RPM;// + 4B * 4 = 20B
  uint8_t _pad2[12];          // + 12B    = 32B
} RWP_SENSOR_DATA;

typedef struct {
  uint8_t cur_mode;           // +1B     = 1B
  RWP_DATA_KIND data_type;    // +1B     = 2B
  uint8_t control_mode;       // +1B     = 3B
  uint8_t _pad1[1];           // +1B     = 4B
  float v_a, v_b, v_c;        // +4B * 3 = 16B
  float v_d, v_e, v_f;        // +4B * 3 = 28B
  float ctrl_value;           // +4B     = 32B
} RWP_CONTROL_DATA;

typedef union {
  RWP_SENSOR_DATA sensor_data;
  RWP_CONTROL_DATA control_data;
  uint8_t bytes[32];
} RWP_MESSAGE_FRAME;


// Each of the below must be exactly 16 bytes
typedef struct {
  uint8_t req_mode;           // +1B     = 1B
  RWP_DATA_KIND req_data_type;// +1B     = 2B
  int16_t pot_value;          // +2B     = 4B
  uint8_t _pad[12];           // +12B    = 16B
} RMT_CONTROL_MESSAGE;

typedef union {
  RMT_CONTROL_MESSAGE control_msg;
  uint8_t bytes[16];
} RMT_MESSAGE_FRAME;


#endif /* SRC_MESSAGE_PROTOCOL_H_ */
