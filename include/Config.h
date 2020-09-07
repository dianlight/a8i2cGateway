#pragma once


/**
 * @brief Debug
 *
 */
//#define DEBUG_I2C_OUT
//#define DEBUG_I2C_IN
//#define DEBUG_EVENT

/**
 * @brief I2C Config
 *
 */
#ifndef I2CADDRESS
#define I2CADDRESS 0x08
#endif  // !I2CADDRESS

/**
 * @brief GPIO sensors
 *
 */
#ifdef HAS_GPIO
#ifndef MAX_DIO
#define MAX_DIO 4
#endif  // !MAX_DIO
#ifndef MAX_AIO
#define MAX_AIO 2
#endif  // !MAX_AIO
#endif

/**
 * @brief DHT sensors
 *
 */
#ifdef HAS_DHT
#ifndef HAS_DHT_MAX_TEMP_COUNT
#define HAS_DHT_MAX_TEMP_COUNT 5
#endif  // !HAS_DHT_MAX_TEMP_COUNT
#ifndef MAX_DHT
#define MAX_DHT 2
#endif  // !MAX_DHT
#endif

/**
 * @brief Relay
 * @DEPRECATED
 *
 */
//#define HAS_RELAY
//#ifdef HAS_RELAY
//    #define RELAYPIN   PD6
//#endif

/**
 * @brief Rotary Encoder
 *
 */
#ifdef HAS_ROTARY_ENCODER
//#define MAX_ENCODER 1
#define ENCODERSWPIN PD5
#define ENCODERCLKPIN PD4
#define ENCODERDTPIN PD3
#define STEPS 4
#endif

/**
 * @brief Uart
 *
 */
#ifdef HAS_UART
#define UART_MTU 64
#else
#define UART_MTU 1
#endif
