#pragma once
#include "Config.h"

#ifdef HAS_DHT
#include <dht.h>
#include <protocol/protocol.h>

dht DHT;

float temperature[HAS_DHT_MAX_TEMP_COUNT];
int8_t tppos = 1;

void loopDHT(a8i2cG::cmd_dht11_data_t *data) {
  int chk;
  switch (data->set.model) {
    case a8i2cG::kDHT11:
    case a8i2cG::kDHT12:
      chk = DHT.read11(data->set.pin);
      break;
    default:
      chk = DHT.read(data->set.pin);
      break;
  }
  switch (chk) {
    case DHTLIB_OK:
#ifdef DEBUG_EVENT
      Serial.print("OK,\t");
      Serial.print(DHT.humidity, 1);
      Serial.print(",\t");
      Serial.println(DHT.temperature, 1);
#endif
      temperature[tppos] = DHT.temperature;
      tppos = (tppos + 1) % data->set.samples;
      if (temperature[0] != 0) {
        float avg = 0;
        for (byte i = 0; i < data->set.samples; i++) {
          avg += temperature[i] + data->set.correction;
        }
        data->temperature = avg / data->set.samples;
      }
      data->humidity = DHT.humidity;
      break;
#ifdef DEBUG_EVENT
    case DHTLIB_ERROR_CHECKSUM:
      Serial.println("Checksum error,\t");
      break;
    case DHTLIB_ERROR_TIMEOUT:
      Serial.println("Time out error,\t");
      break;
    default:
      Serial.println("Unknown error,\t");
      break;
#endif
  }
}

#endif
