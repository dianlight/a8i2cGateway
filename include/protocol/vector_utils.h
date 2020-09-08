#pragma once

#include "Config.h"
#include "protocol/protocol.h"

namespace a8i2cG {

#ifdef HAS_GPIO
int8_t findGPIO(uint8_t pin, cmd_gpio_data_t darray[MAX_GPIO],
                                   uint8_t max) {
  for (uint8_t p = 0; p < max; p++) {
    if (darray[p].set.pin == pin) {
        return p;
    }
  }
  return -1;
}
#endif

#ifdef HAS_DHT
int8_t findDHT(uint8_t pin, cmd_dht11_data_t darray[MAX_DHT],
                                   uint8_t max) {
  for (uint8_t p = 0; p < max; p++) {
    if (darray[p].set.pin == pin) {
        return p;
    }
  }
  return -1;
}
#endif


}  // namespace a8i2cG
