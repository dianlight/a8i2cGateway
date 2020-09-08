#pragma once

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
#ifndef MAX_GPIO
#define MAX_GPIO 4
#endif  // !MAX_GPIO
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
// #define HAS_RELAY
// #ifdef HAS_RELAY
//    #define RELAYPIN   PD6
// #endif

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
#define UART_MTU 32
#else
#define UART_MTU 1
#endif

/**
 * @brief Log
 *
 */
#if (defined(HAS_LOG_UART) || defined(HAS_LOG_SOFTUART)) && !defined(HAS_LOG)
#define HAS_LOG
#endif
#ifdef HAS_LOG
#if defined(HAS_LOG_UART) && defined(HAS_LOG_SOFTUART)
#error "HAS_LOG_UART and HAS_LOG_SOFTUART can't both enabled"
#endif
#if defined(HAS_UART) && defined(HAS_LOG_UART)
#error "HAS_UART and HAS_LOG can't both enabled"
#endif
#if defined(HAS_LOG_SOFTUART) && !defined(SOFTUART_TX_PIN)
#error "SOFTUART_TX_PIN mut be defined for HAS_LOG_SOFTUART"
#endif
#define DEBUG_I2C_OUT
#define DEBUG_I2C_IN
#define DEBUG_EVENT
#endif

/**
 * @brief System Control
 *
 * Control some AVR specific functionality as reboot
 *
 */
#ifdef HAS_SYSCTR
// specific functionalities
#endif
