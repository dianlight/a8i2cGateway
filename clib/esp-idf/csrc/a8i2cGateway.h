#pragma once
#include <inttypes.h>
#include <esp_err.h>

#define I2CADDRESS CONFIG_A8I2CGW_I2C_ADDRESS
#if CONFIG_A8I2CGW_GPIO_MAX > 0
#define HAS_GPIO
#define MAX_GPIO CONFIG_A8I2CGW_GPIO_MAX
#endif
#ifdef CONFIG_A8I2CGW_SYSCTR
#define HAS_SYSCTR
#endif
#ifdef CONFIG_A8I2CGW_UART0
#define HAS_UART
#endif
#if CONFIG_A8I2CGW_DHT_MAX > 0
#define HAS_DHT
#define MAX_DHT CONFIG_A8I2CGW_DHT_MAX
#endif
#ifdef CONFIG_A8I2CGW_ROTARY_ENCODER
#define HAS_ROTARY_ENCODER
#endif

#include "../../../include/Config.h"
#include "../../../include/protocol/protocol.h"

/*
#ifdef CONFIG_A8I2CGW_DEBUG_LOG_UART0
  #define DHAS_LOG_UART
#endif
#ifdef CONFIG_A8I2CGW_DEBUG_LOG_SOFTUART
  #define HAS_LOG_SOFTUART
  #define SOFTUART_TX_PIN CONFIG_A8I2CGW_DEBUG_LOG_SOFTUART_TX_PIN
#endif
*/

#include "../include/Config.h"


esp_err_t a8i2cGw_init(int sda, int scl);
esp_err_t a8i2cGw_read(I2C_Packet_t send, I2C_Packet_t *receive);
// void a8i2cGw_send(I2C_Packet_t packet);
int8_t a8i2cGw_cmd_set(device_t device, cmd_set_t set);

#ifdef HAS_GPIO
int8_t setupGpio(cmd_gpio_set_t set);
int8_t writeGpio(uint8_t pin, uint16_t value);
int8_t readGpio(uint8_t pin, cmd_gpio_data_t *data);
int16_t readLastGpio(uint8_t pin);
#endif

#ifdef HAS_UART
int8_t setupUart(cmd_uart_set_t set);
int8_t writeUart(uint8_t value[], size_t len);
int8_t readUart(cmd_uart_data_t *data);
#endif

#ifdef HAS_SYSCTR
int8_t setupSysCtr(cmd_sysctr_set_t set);
#endif

#ifdef HAS_DHT
int8_t setupDht(cmd_dht11_set_t set);
int8_t readDht(uint8_t id, cmd_dht11_data_t *data);
#endif

#ifdef HAS_ROTARY_ENCODER
int8_t setupRotaryEncoder(cmd_encoder_set_t set);
int8_t readRotaryEncoder(cmd_encoder_data_t *data);
#endif
