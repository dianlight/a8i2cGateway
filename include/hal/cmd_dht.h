#pragma once
#include "Config.h"

#ifdef HAS_DHT
#include <dht.h>
#include <hal/cmd_log.h>
#include <protocol/protocol.h>

dht DHT;

float temperature[HAS_DHT_MAX_TEMP_COUNT];
int8_t tppos = 1;

void loopDHT(cmd_dht11_data_t *data) {
  int chk;
  switch (data->set.model) {
    case kDHT11:
    case kDHT12:
      chk = DHT.read11(data->set.pin);
      break;
    default:
      chk = DHT.read(data->set.pin);
      break;
  }
  switch (chk) {
    case DHTLIB_OK:
#ifdef DEBUG_EVENT
      log_d("OK,\t%.1f\t%.1f", DHT.humidity, DHT.temperature);
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
      log_e("Checksum error,\t");
      break;
    case DHTLIB_ERROR_TIMEOUT:
      log_e("Time out error,\t");
      break;
    default:
      log_e("Unknown error,\t");
      break;
#endif
  }
}

#endif
