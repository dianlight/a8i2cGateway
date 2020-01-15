#pragma once

/**
 * @brief Debug 
 * 
 */
//#define DEBUG_I2C_OUT
//#define DEBUG_I2C_IN
#define DEBUG_EVENT

/**
 * @brief I2C Config
 * 
 */
#define I2CADDRESS 0x08 


/**
 * @brief DHT sensors
 * 
 */
#define HAS_DHT
#ifdef HAS_DHT
    #define DHTPIN     PD7 
    #define TEMP_COUNT 5
    #define FIXED_CORRECTION -2.0
#endif

/**
 * @brief Relay
 * 
 */
#define HAS_RELAY
#ifdef HAS_RELAY
    #define RELAYPIN   PD6
#endif

/**
 * @brief Rotary Encoder
 * 
 */
#define HAS_ROTARY_ENCODER
#ifdef HAS_ROTARY_ENCODER
    #define ENCODERSWPIN  PD5 
    #define ENCODERCLKPIN PD4
    #define ENCODERDTPIN  PD3
    #define STEPS 4
#endif


