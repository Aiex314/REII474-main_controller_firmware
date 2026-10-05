/*
 * bno_driver.c
 *
 *  Created on: Sep 14, 2026
 *      Author: hapro
 */


/* I apologise to whomever has to read this code.
 * This driver devolved into a cesspool of hotfixes
 * very quickly, and I never got around to fixing it
 */



#include "bno_driver.h"


extern SPI_HandleTypeDef* hspi_bno;


// CHANNEL DESIGNATIONS:
// 0 - SHTP control channel (BNO085 configuration)
// 1 - SH-2 control channel (sensor configuration)
// 2 - Normal channel (sensor data)
// 3, 4, 5 - I don't know, and I don't care...
static uint8_t channel_seq_nums[6] = {0, 0, 0, 0, 0, 0};
static uint8_t cmdSequenceNum = 0;

static BNO_STATE BNO_status = BNO_UNINITIALIZED;

static const float RAD_TO_DEG = 180.0f/3.1415926f;

static void waitForIRQ(GPIO_PinState reqState) {
  for (;;)
    if (reqState == HAL_GPIO_ReadPin(BNO_IRQ_GPIO_Port, BNO_IRQ_Pin)) break;
}

static void transmitFrame(uint8_t* buffer, uint8_t size, uint8_t channel) {
  // Only up to channel 5 allowed
  if (channel >= 6) return;

  // Set up the header
  uint8_t header[4] = {0x00, 0x00, 0x00, 0x00};
  header[0] = size+4;
  header[2] = channel;
  header[3] = channel_seq_nums[channel]++;

  // Transmit the entire frame
  HAL_GPIO_WritePin(BNO_CSN_GPIO_Port, BNO_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi_bno, header, 4, 10);
  HAL_SPI_Transmit(hspi_bno, buffer, size, 10);
  HAL_GPIO_WritePin(BNO_CSN_GPIO_Port, BNO_CSN_Pin, GPIO_PIN_SET);
}

static void waitTransmitFrame(GPIO_PinState reqState, uint8_t* buffer, uint8_t size, uint8_t channel) {
  waitForIRQ(reqState);
  transmitFrame(buffer, size, channel);
}

static uint16_t receiveFrame(uint8_t* buffer, uint16_t max_size, uint8_t* channel) {
  // Allocate a spot for the header
  uint8_t header[4] = {0x00, 0x00, 0x00, 0x00};

  // Get the header
  HAL_GPIO_WritePin(BNO_CSN_GPIO_Port, BNO_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Receive(hspi_bno, header, 4, 10);

  // Obtain the frame length and channel number
  uint16_t recv_len = (header[0] + 256*(0b01111111 & header[1]))-4;
  *channel  = header[2];

  // If the frame length is 0, then there's no data...
  if (recv_len == 0) {
    HAL_GPIO_WritePin(BNO_CSN_GPIO_Port, BNO_CSN_Pin, GPIO_PIN_SET);
    return 0;
  }

  // If the buffer is big enough to accept the frame, read it in in one piece
  if (recv_len <= max_size) {
    memset(buffer, 0x00, max_size);
    HAL_SPI_Receive(hspi_bno, buffer, recv_len, 10);
  }
  // Else, read it in chunks. This destroys the received data...
  else {
    uint16_t remaining_len = recv_len;

    while (remaining_len > max_size) {
      memset(buffer, 0x00, max_size);
      HAL_SPI_Receive(hspi_bno, buffer, max_size, 10);
      remaining_len -= max_size;
    }
    if (remaining_len > 0) {
      memset(buffer, 0x00, max_size);
      HAL_SPI_Receive(hspi_bno, buffer, remaining_len, 10);
    }
  }

  // Deselect the device
  HAL_GPIO_WritePin(BNO_CSN_GPIO_Port, BNO_CSN_Pin, GPIO_PIN_SET);

  return recv_len;
}

static uint16_t waitReceiveFrame(GPIO_PinState req_state, uint8_t* buffer, uint16_t max_size, uint8_t* channel) {
  waitForIRQ(req_state);
  return receiveFrame(buffer, max_size, channel);
}

// Whose idea was it to make the two SPI devices with incompatible clock configs share a bus?
// oh wait..
static void setSPIconfig() {
  if (hspi_bno->Init.CLKPhase != SPI_PHASE_2EDGE || hspi_bno->Init.CLKPolarity != SPI_POLARITY_HIGH) {
    HAL_SPI_DeInit(hspi_bno);
    hspi_bno->Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi_bno->Init.CLKPolarity = SPI_POLARITY_HIGH;
    HAL_SPI_Init(hspi_bno);
  }
}




void BNO_startup() {
  uint8_t spi_buf[64];
  uint8_t  recv_channel;
  memset(spi_buf, 0x00, 64);

  // Configure the SPI1 clock correctly for the BNO
  setSPIconfig();


  // Still need to make this thing non-blocking....
  BNO_status = BNO_UNINITIALIZED;

  HAL_GPIO_WritePin(BNO_RESET_GPIO_Port, BNO_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(BNO_RESET_GPIO_Port, BNO_RESET_Pin, GPIO_PIN_SET);

  BNO_status = BNO_BOOTING;

  // Wait until IRQ goes high
  waitForIRQ(GPIO_PIN_SET);

  // Wait for three messages from the BNO
  for (uint8_t recv_count = 0; recv_count < 3;) {
    GPIO_PinState irq_state = HAL_GPIO_ReadPin(BNO_IRQ_GPIO_Port, BNO_IRQ_Pin);

    if (irq_state == GPIO_PIN_SET) continue;

    HAL_GPIO_WritePin(BNO_CSN_GPIO_Port, BNO_CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Receive(hspi_bno, spi_buf, 4, 10);

	uint16_t recv_len = (spi_buf[0] + 256*(0b01111111 & spi_buf[1]))-4;
	uint8_t  channel  = spi_buf[2];

	while (recv_len > 64) {
	  recv_len -= 64;

	  memset(spi_buf, 0x00, 64);
	  HAL_SPI_Receive(hspi_bno, spi_buf, 64, 10);
	}
	if (recv_len != 0) {
	  memset(spi_buf, 0x00, 64);
	  HAL_SPI_Receive(hspi_bno, spi_buf, recv_len, 10);
	}

	uint8_t command = spi_buf[0];

    HAL_GPIO_WritePin(BNO_CSN_GPIO_Port, BNO_CSN_Pin, GPIO_PIN_SET);

    if (recv_count == 0 || (command == 0xF1 && channel == 0x02) || (command == 0x01 && channel == 0x01))
      recv_count++;

    memset(spi_buf, 0x00, 64);
  }



  // Transmit a sensor reorientation request
  uint8_t REORIENT_SENSOR[] = {
    0xf2, cmdSequenceNum++,
    0x03, 0x02,
    0b00000000, 0b10100000, // x
    0b00000000, 0b00100000, // y
    0b00000000, 0b10100000, // z
    0b00000000, 0b00100000  // w
  };

  // WAKE UP
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_RESET);
  waitForIRQ(GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_SET);

  // Send the reorientation request
  waitTransmitFrame(GPIO_PIN_RESET, REORIENT_SENSOR, 12, 0x02);



//  // Transmit a sensor reorientation request
//  uint8_t SENSOR_ORIENTATION[] = {
//    0b00000000, 0b00000000, 0b00000000, 0b00000000, //  0.5 as 32-bit fixed point number
//    0b00000000, 0b00000000, 0b00000000, 0b01000000, // -0.5 as 32-bit fixed point number
//    0b00000000, 0b00000000, 0b00000000, 0b00000000, // This defines a quaternion corresponding
//    0b00000000, 0b00000000, 0b00000000, 0b00000000, // to the correct reorientation
//  };
//
//
//  uint8_t resp = BNO_writeFRSrecord(0x2d3e, 0x00, 0x04, &SENSOR_ORIENTATION[0]);
//  resp = BNO_writeFRSrecord(0x2d3e, 0x02, 0x02, &SENSOR_ORIENTATION[8]);
//
//  memset(SENSOR_ORIENTATION, 0xff, 16);
//
//  BNO_readFRSrecord(0x2D3E, 0x00, 0x04, SENSOR_ORIENTATION);


  // WAKE UP
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_RESET);
  waitForIRQ(GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_SET);

  // Transmit a product ID request
  uint8_t REQ_PROD_ID[] =
  {
    0xF9, 0x00
  };
  transmitFrame(REQ_PROD_ID, sizeof(REQ_PROD_ID), 0x02);



  // Receive the product ID response
  waitReceiveFrame(GPIO_PIN_RESET, spi_buf, 64, &recv_channel);

  // Receive the other half of the fucking response. Cause it's just in two pieces for no reason
  waitReceiveFrame(GPIO_PIN_RESET, spi_buf, 64, &recv_channel);



  // At this point, the BNO should be initialized and asleep.
  BNO_status = BNO_INIT_SLEEP;
}

void BNO_startRotationVector() {

  // Check whether the IMU is ready
  if (BNO_status != BNO_INIT_SLEEP) return;

  // Wake the IMU
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_RESET);
  waitForIRQ(GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_SET);

//  // Transmit a feature set request
  uint8_t START_BNO[] =
  {
  		0xFD,
  		0x05,
  		0x00,
  		0x00, 0x00,
  		0x10, 0x27, 0x00, 0x00,
  		0x00, 0x00, 0x00, 0x00,
  		0x00, 0x00, 0x00, 0x00
  };
  transmitFrame(START_BNO, sizeof(START_BNO), 0x02);

  BNO_status = BNO_CONFIGURED;
}



// TODO: Fix this. Not critical, but this does not work...
//       - Need to treat status codes properly
//       - Need to treat separated responses properly (reads longer than length 2)
//       - Need to provide a better interface (error codes, true read length, etc.)
//       - Need input validation
uint8_t BNO_readFRSrecord(uint16_t record_num, uint16_t read_offset, uint16_t read_len, uint8_t* out_buf) {
  uint8_t recv_buf[16];
  // See page 40 of SH-2 datasheet for FRS read response format & length
  // Each read response is 16 bytes.

  memset(out_buf, 0x00, read_len*4);

  uint8_t READ_FRS[] =
  {
  		0xF4, 0x00,
  		0x00, 0x00,
  		0x00, 0x00,
  		0x00, 0x00
  };

  READ_FRS[2] = read_offset%256;
  READ_FRS[3] = read_offset/256;
  READ_FRS[4] = record_num%256;
  READ_FRS[5] = record_num/256;
  READ_FRS[6] = read_len%256;
  READ_FRS[7] = read_len/256;

  // WAKE UP
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_RESET);
  waitForIRQ(GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_SET);

  waitTransmitFrame(GPIO_PIN_RESET, READ_FRS, 8, 0x02);


  // Receive the message in chunks
  uint16_t cur_offset = 0;
  uint16_t rem_length = read_len;

  while (rem_length > 0x00) {
    uint8_t recv_channel = 0;
    do {
      recv_channel = 0xff;
      waitReceiveFrame(GPIO_PIN_RESET, recv_buf, 16, &recv_channel);
    } while (recv_channel != 0x02 && recv_buf[0] != 0xF3);

    if (recv_buf[12] != record_num%256 && recv_buf[13] != record_num/256) continue;

    uint8_t status = 0x0f & recv_buf[1];

    if (status == 2) continue;
    if (status != 3 && status != 6 && status != 7 && status != 0) return status;

    uint8_t data_length = (0xf0 & recv_buf[1]) >> 2;

    memcpy(&out_buf[cur_offset], &recv_buf[4], data_length);

    HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, GPIO_PIN_RESET);

    cur_offset += data_length;
    rem_length -= data_length/4;

    if (status == 7) return 0;
  }

  return 0;
}


uint8_t BNO_writeFRSrecord(uint16_t record_num, uint16_t write_offset, uint16_t write_len, const uint8_t* out_buf) {
  uint8_t recv_buf[4];
  // See page 39 of SH-2 datasheet for FRS write response format & length
  // Write responses are always 4 bytes...

  uint8_t WRITE_FRS[] =
  {
  		0xF7, 0x00,
  		0x00, 0x00,
  		0x00, 0x00
  };

  WRITE_FRS[2] = write_len%256;
  WRITE_FRS[3] = write_len/256;
  WRITE_FRS[4] = record_num%256;
  WRITE_FRS[5] = record_num/256;


  // WAKE UP
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_RESET);
  waitForIRQ(GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_SET);

  // Start an FRS write
  waitTransmitFrame(GPIO_PIN_RESET, WRITE_FRS, 6, 0x02);


  // Wait for an FRS write response
  uint8_t recv_channel;

  do {
    recv_channel = 0xff;
    waitReceiveFrame(GPIO_PIN_RESET, recv_buf, 4, &recv_channel);
  } while (recv_channel != 0x02 && recv_buf[0] != 0xF5);

  if (recv_buf[1] != 0x04)
    return recv_buf[1];


  // Write the FRS record
  uint8_t WRITE_FRS_DATA[] =
  {
  		0xF6, 0x00,
  		0x00, 0x00,
  		0x00, 0x00,
  		0x00, 0x00,
  		0x00, 0x00,
  		0x00, 0x00,
  };

  // WAKE UP
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_RESET);
  waitForIRQ(GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_SET);

  uint16_t cur_offset;
  uint16_t cur_written = 0;
  int16_t rem_length = write_len;

  while (rem_length > 0x00) {
    cur_offset = write_offset + cur_written;
    memset(&WRITE_FRS_DATA[4], 0x00, 8);

    // Set FRS data offset
    WRITE_FRS_DATA[2] = cur_offset % 256;
    WRITE_FRS_DATA[3] = cur_offset / 256;

    // If remaining length odd, copy 4 bytes, else copy 8
    uint8_t cpy_len = (rem_length % 2 == 1) ? 4 : 8;
    memcpy(&WRITE_FRS_DATA[4], &out_buf[cur_offset*4], cpy_len);

    rem_length -= 2;
    cur_offset += 2;

    // Transmit the data write request
    waitTransmitFrame(GPIO_PIN_RESET, WRITE_FRS_DATA, 12, 0x02);



    // Wait for an acknowledge...
    do {
      recv_channel = 0xff;
      waitReceiveFrame(GPIO_PIN_RESET, recv_buf, 4, &recv_channel);
    } while (recv_channel != 0x02 && recv_buf[0] != 0xF5);

    if (recv_buf[1] != 0x00)
      return recv_buf[1];

    // And a record valid response...
    do {
      recv_channel = 0xff;
      waitReceiveFrame(GPIO_PIN_RESET, recv_buf, 4, &recv_channel);
    } while (recv_channel != 0x02 && recv_buf[0] != 0xF5);

    if (recv_buf[1] != 0x08)
      return recv_buf[1];

    // AND a write complete..........................
    do {
      recv_channel = 0xff;
      waitReceiveFrame(GPIO_PIN_RESET, recv_buf, 4, &recv_channel);
    } while (recv_channel != 0x02 && recv_buf[0] != 0xF5);

    if (recv_buf[1] != 0x03)
      return recv_buf[1];
  }

  return 0;
}


uint8_t BNO_setCalibration(CALIBRATION_FLAG flags) {
  uint8_t recv_buf[16];

  uint8_t SET_CAL[] =
  {
  		0xF2, 0x00,
  		0x07, 0x00,
  		0x00, 0x00,
  		0x00, 0x00,
  		0x00, 0x00,
  		0x00, 0x00
  };

  SET_CAL[1] = cmdSequenceNum++;

  SET_CAL[3] = ((flags & ACCEL_ENABLE) != 0) ? 1 : 0;
  SET_CAL[4] = ((flags & GYRO_ENABLE)  != 0) ? 1 : 0;
  SET_CAL[5] = ((flags & MAG_ENABLE)   != 0) ? 1 : 0;
  SET_CAL[7] = ((flags & PLANAR_CAL)   != 0) ? 1 : 0;


  // WAKE UP
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_RESET);
  waitForIRQ(GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BNO_WAKE_GPIO_Port, BNO_WAKE_Pin, GPIO_PIN_SET);

  // Write the calibration config
  waitTransmitFrame(GPIO_PIN_RESET, SET_CAL, 12, 0x02);

  // Wait for the response
  uint8_t recv_channel;

  do {
    recv_channel = 0xff;
    waitReceiveFrame(GPIO_PIN_RESET, recv_buf, 16, &recv_channel);
  } while (recv_channel != 0x02 && recv_buf[0] != 0xF5);

  if (recv_buf[5] != 0) return 0xff;

  uint8_t ret = 0;
  if (recv_buf[6] == 1) ret |= ACCEL_ENABLE;
  if (recv_buf[7] == 1) ret |= GYRO_ENABLE;
  if (recv_buf[8] == 1) ret |= MAG_ENABLE;
  if (recv_buf[9] == 1) ret |= PLANAR_CAL;

  return ret;

}


static float curYaw;
static float curPitch;
static float curRoll;

void BNO_IRQcallback() {
  uint8_t recv_len, recv_channel;
  uint8_t spi_buf[64];

  setSPIconfig();

  if (BNO_status != BNO_CONFIGURED) return;

  // Receive a packet every time IRQ goes low
  recv_len = waitReceiveFrame(GPIO_PIN_RESET, spi_buf, 64, &recv_channel);


  // Assume it's a data packet, and get the contents as a thing. (??)
  if (recv_len <= 64) {
    float i = (int16_t)( spi_buf[9] + spi_buf[10] * 256)/16384.0f;
    float j = (int16_t)(spi_buf[11] + spi_buf[12] * 256)/16384.0f;
    float k = (int16_t)(spi_buf[13] + spi_buf[14] * 256)/16384.0f;
    float r = (int16_t)(spi_buf[15] + spi_buf[16] * 256)/16384.0f;

    curYaw   = RAD_TO_DEG * atan2f(2*(r*i + j*k), 1 - 2*(i*i + j*j));
    curPitch = RAD_TO_DEG * asinf(2*(r*j - i*k));
    curRoll  = RAD_TO_DEG * atan2f(2*(r*k + i*j), 1 - 2*(j*j + k*k));
  }
}


void BNO_getYPR(float* y, float* p, float* r) {
  *y = curYaw;
  *p = curPitch;
  *r = curRoll;
}
