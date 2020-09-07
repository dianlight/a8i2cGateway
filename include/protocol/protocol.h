#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the commands
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project A8i2cGateway
 *
 **/
#include <Arduino.h>
#include <new.h>

#define MTU 64
#define RES_MTU MTU - sizeof(device_t)
#define REQ_MTU RES_MTU - sizeof(cmd_t)

namespace a8i2cG {

typedef enum {
  kDio = 'G',      // Generic Digital GPIO
  kAio = 'A',      // Generic Analog GPIO
  kUart = 'U',     // Uart
  kDht = 'H',      // DHT11
  kEncoder = 'E',  // Rotary Encoder
} device_t;

typedef enum {
  kSet = 'S',    // Set
  kRead = 'R',   // Read
  kWrite = 'W',  // Write
} cmd_t;

typedef enum {
  kNoError = 0x00,
  kInvalidPacketSize = 'S',
  kProtocolError = 'P',
  kInvalidDevice = 'D',
  kInvalidCommand = 'C',
  kMaxCapacityFull = 'F',
  kInvalidPin = 'I'
} error_t;

typedef struct __attribute__((packed, aligned(1))) {
  error_t code;
} error_data_t;

// Digital Gpio/DIO
#ifdef HAS_GPIO
typedef struct __attribute__((packed, aligned(1))) {
  uint8_t pin;
  byte mode;
  uint16_t hz;  // 0 means 1 read. >0 continuos read and enqueue data in buffer (64bit buffer for dio e MTU for aio)
} cmd_gpio_dio_set_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t pin;
  bool value;
} cmd_gpio_dio_write_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t pin;
} cmd_gpio_dio_read_t;

typedef struct __attribute__((packed, aligned(1))) {
  cmd_gpio_dio_set_t set;
  uint32_t last_read;
  uint64_t values;
  uint8_t values_len;
} cmd_gpio_dio_data_t;

// Analog Gpio
typedef cmd_gpio_dio_set_t cmd_gpio_aio_set_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t pin;
  uint16_t value;
} cmd_gpio_aio_write_t;

typedef cmd_gpio_dio_read_t cmd_gpio_aio_read_t;

typedef struct __attribute__((packed, aligned(1))) {
  cmd_gpio_dio_set_t set;
  uint32_t last_read;
  uint8_t values[RES_MTU-1];
  uint8_t values_len;
} cmd_gpio_aio_data_t;
#endif

// Uart
#ifdef HAS_UART
typedef struct __attribute__((packed, aligned(1))) {
  uint16_t speed;
} cmd_uart_set_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t buffer[REQ_MTU - sizeof(uint8_t) - sizeof(bool)];
  uint8_t buffer_len;
  bool overflow;
} cmd_uart_write_t;

typedef struct __attribute__((packed, aligned(1))) {
} cmd_uart_read_t;

typedef cmd_uart_write_t cmd_uart_data_t;
#endif

// DHT11
#ifdef HAS_DHT
typedef enum {
  kDHT11,
  kDHT12,
  kDHT22
} dht_model;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t pin;
  dht_model model;
  uint8_t samples;
  float correction;
} cmd_dht11_set_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t pin;
} cmd_dht11_read_t;

typedef struct __attribute__((packed, aligned(1))) {
  cmd_dht11_set_t set;
  float temperature;
  float humidity;
} cmd_dht11_data_t;
#endif

// Rotary Encoder
#ifdef HAS_ROTARY_ENCODER
typedef enum {
      kOpen,
      kClosed,
      kPressed,
      kHeld,
      kReleased,
      kClicked,
      kDoubleClicked
} enc_button_state;


typedef struct __attribute__((packed, aligned(1))) {
  uint8_t swpin, clkpin, dtpin;
  uint8_t steps;
} cmd_encoder_set_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t swpin;
} cmd_encoder_read_t;

typedef struct __attribute__((packed, aligned(1))) {
  cmd_encoder_set_t set;
  int8_t encoder;
  enc_button_state select;
} cmd_encoder_data_t;
#endif
//

typedef union {
  #ifdef HAS_GPIO
  cmd_gpio_dio_set_t cmd_gpio_dio_set;
  cmd_gpio_aio_set_t cmd_gpio_aio_set;
  #endif
  #ifdef HAS_UART
  cmd_uart_set_t cmd_uart_set;
  #endif
  #ifdef HAS_DHT
  cmd_dht11_set_t cmd_dht11_set;
  #endif
  #ifdef HAS_ROTARY_ENCODER
  cmd_encoder_set_t cmd_encoder_set;
  #endif
} cmd_set_t;

typedef union {
  #ifdef HAS_GPIO
  cmd_gpio_dio_read_t cmd_gpio_dio_read;
  cmd_gpio_aio_read_t cmd_gpio_aio_read;
  #endif
  #ifdef HAS_UART
  cmd_uart_read_t cmd_uart_read;
  #endif
  #ifdef HAS_DHT
  cmd_dht11_read_t cmd_dht11_read;
  #endif
  #ifdef HAS_ROTARY_ENCODER
  cmd_encoder_read_t cmd_encoder_read;
  #endif
} cmd_read_t;

typedef union {
  #ifdef HAS_GPIO
  cmd_gpio_dio_write_t cmd_gpio_dio_write;
  cmd_gpio_aio_write_t cmd_gpio_aio_write;
  #endif
  #ifdef HAS_UART
  cmd_uart_write_t cmd_uart_write;
  #endif
} cmd_write_t;

typedef union {
  #ifdef HAS_GPIO
  cmd_gpio_dio_data_t cmd_gpio_dio_data;
  cmd_gpio_aio_data_t cmd_gpio_aio_data;
  #endif
  #ifdef HAS_UART
  cmd_uart_data_t cmd_uart_data;
  #endif
  #ifdef HAS_DHT
  cmd_dht11_data_t cmd_dht11_data;
  #endif
  #ifdef HAS_ROTARY_ENCODER
  cmd_encoder_data_t cmd_encoder_data;
  #endif
} cmd_data_t;

//

typedef struct __attribute__((packed, aligned(1))) {  // 64Byte
  cmd_t cmd;                                          // 1byte
  device_t device;                                    // 1byte
  union data {
    cmd_set_t set;
    cmd_read_t read;
    cmd_write_t write;
    char raw[];
  } data;
} request_t;

typedef struct __attribute__((packed, aligned(1))) {  // 64Byte
  device_t device;                                    // 1byte
  bool error;
  union data {
    error_data_t error;
    cmd_data_t data;
    char raw[];
  } data;
} response_t;

typedef union __attribute__((packed, aligned(1))) {
  request_t request;
  response_t reponse;
  byte I2CPacket[];
} I2C_Packet_t;

}  // namespace a8i2cG
