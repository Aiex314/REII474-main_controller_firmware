/*
 * nRF24_driver.h
 *
 *  Created on: Aug 28, 2026
 *      Author: hapro
 */

#ifndef SRC_NRF24_DRIVER_H_
#define SRC_NRF24_DRIVER_H_

#include <string.h>
#include <stdint.h>

typedef enum {
  N24_R_RX_PAYLOAD=0b01100001,
  N24_ACTIVATE=0b01010000,
  N24_W_TX_PAYLOAD=0b10100000,
  N24_W_ACK_PAYLOAD=0b10101000,
  N24_FLUSH_TX=0b11100001,
  N24_FLUSH_RX=0b11100010,
  N24_NOP=0b11111111
} n24Command;


typedef enum {
  N24_CONFIG,
  N24_EN_AA,
  N24_EN_RXADDR,
  N24_SETUP_AW,
  N24_SETUP_RETR,
  N24_RF_CH,
  N24_RF_SETUP,
  N24_STATUS,
  N24_OBSERVE_TX,
  N24_CD,
  N24_RX_ADDR_P0,
  N24_RX_ADDR_P1,
  N24_RX_ADDR_P2,
  N24_RX_ADDR_P3,
  N24_RX_ADDR_P4,
  N24_RX_ADDR_P5,
  N24_TX_ADDR,
  N24_RX_PW_P0,
  N24_RX_PW_P1,
  N24_RX_PW_P2,
  N24_RX_PW_P3,
  N24_RX_PW_P4,
  N24_RX_PW_P5,
  N24_FIFO_STATUS,
  N24_DYNPD=0x1C,
  N24_FEATURE=0x1D
} n24Reg;

void    writeNRF24register(uint8_t index, uint8_t value);
uint8_t readNRF24register(uint8_t index);
uint8_t readNRF24status();
void    setNRF24TXaddr(const uint8_t* addr, uint8_t addr_len);
void    setNRF24RXaddr(const uint8_t* addr, uint8_t addr_len, uint8_t pipe_no);
void    sendNRF24payload(const uint8_t* data, uint8_t data_len);
void    readNRF24payload(uint8_t* buffer, uint8_t data_len);
void    toggleNRF24extraFeatures();
void    writeNRF24ackPayload(const uint8_t* data, uint8_t data_len, uint8_t pipe_no);
void    flushNRF24buffers();

#endif /* SRC_NRF24_DRIVER_H_ */
