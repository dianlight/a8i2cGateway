#pragma once
#include "Config.h"

#ifdef HAS_LOG

#define HAL_LOG_FORMAT(letter, format) \
  "[" #letter "][%s:%u] %s(): " format "\r\n", __FILE__, __LINE__, __FUNCTION__

#ifdef HAS_LOG_UART

#define log_speed(speed) Serial.begin(speed)
#define log_d(format, ...) \
  Serial.printf(HAL_LOG_FORMAT(V, format), ##__VA_ARGS__)
#define log_i(format, ...) \
  Serial.printf(HAL_LOG_FORMAT(V, format), ##__VA_ARGS__)
#define log_w(format, ...) \
  Serial.printf(HAL_LOG_FORMAT(V, format), ##__VA_ARGS__)
#define log_e(format, ...) \
  Serial.printf(HAL_LOG_FORMAT(V, format), ##__VA_ARGS__)

#else if defined(HAS_LOG_SOFTUART)

#include <SoftwareSerial.h>

extern SoftwareSerial SwSerial;

#define log_speed(speed) SwSerial.begin(speed)
#define log_d(format, ...) \
  SwSerial.printf(HAL_LOG_FORMAT(V, format), ##__VA_ARGS__)
#define log_i(format, ...) \
  SwSerial.printf(HAL_LOG_FORMAT(V, format), ##__VA_ARGS__)
#define log_w(format, ...) \
  SwSerial.printf(HAL_LOG_FORMAT(V, format), ##__VA_ARGS__)
#define log_e(format, ...) \
  SwSerial.printf(HAL_LOG_FORMAT(V, format), ##__VA_ARGS__)

#endif
#else
#define log_speed(speed)
#define log_d(format, ...)
#define log_i(format, ...)
#define log_w(format, ...)
#define log_e(format, ...)

#endif  // HAS_LOG
