/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This main file to manage the i2c extender
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project A8i2cGateway
 *
 **/
#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "hal/cmd_dht.h"
#include "hal/cmd_encoder.h"
#include "protocol/protocol.h"
#include "protocol/vector_utils.h"

void requestI2CEvent();
void receiveI2CEvent(int numBytes);

void setup() {
  // Serial.begin(9600);
  //  while(!Serial);

  //  Wire.setClock(10000);
  Wire.begin(I2CADDRESS);
  Wire.onReceive(receiveI2CEvent);
  Wire.onRequest(requestI2CEvent);
}

a8i2cG::I2C_Packet_t outData;
bool sendOutData = false;
uint8_t numI2CRequest = 0;

#ifdef HAS_GPIO
a8i2cG::cmd_gpio_data_t gpio_vector[MAX_GPIO];
uint8_t gpio_vector_configured = 0;
#endif
#ifdef HAS_DHT
a8i2cG::cmd_dht11_data_t dht_vector[MAX_DHT];
uint8_t dht_vector_configured = 0;
#endif
#ifdef HAS_ROTARY_ENCODER
a8i2cG::cmd_encoder_data_t encoder_single;
#endif
#ifdef HAS_UART
a8i2cG::cmd_uart_data_t uart_single;
#endif

// unsigned long lastRead;

void sendI2CPacket() {
#ifdef DEBUG_I2C_OUT
  for (byte i = 0; i < sizeof(a8i2cG::I2C_Packet_t); i++) {
    Serial.write(outData.I2CPacket[i]);
    Serial.write(" [");
    Serial.print(outData.I2CPacket[i], HEX);
    Serial.write("] ");
  }
#endif
  size_t sent = Wire.write(outData.I2CPacket, sizeof(a8i2cG::I2C_Packet_t));
  if (sent > 0) {
    sendOutData = false;
    numI2CRequest--;
  }
#ifdef DEBUG_I2C_OUT
  Serial.println(sent);
#endif
}

void loop() {
  if (numI2CRequest > 0 && sendOutData) sendI2CPacket();
  uint32_t cur = millis();

#ifdef HAS_GPIO
  // GPIO
  for (uint8_t p = 0; p < gpio_vector_configured; p++) {
    if (cur < gpio_vector[p].last_read)
      gpio_vector[p].last_read = 0;  // Fix reset overflow every 70min.
    if (gpio_vector[p].set.hz == 0 && cur - gpio_vector[p].last_read > 1000) {
      if (gpio_vector[p].set.type == a8i2cG::kAnalog) {
        gpio_vector[p].values[0] = analogRead(gpio_vector[p].set.pin);
      } else {
        gpio_vector[p].values[0] =
            digitalRead(gpio_vector[p].set.pin) == HIGH ? 1 : 0;
      }
      gpio_vector[p].values_len = 1;
    } else if (cur - gpio_vector[p].last_read > 1000 / gpio_vector[p].set.hz) {
      if (gpio_vector[p].values_len < 64) gpio_vector[p].values_len++;
      for (uint8_t i = 0; i < sizeof(gpio_vector[p].values) - 2; i++) {
        gpio_vector[p].values[i + 1] = gpio_vector[p].values[0];
      }
      gpio_vector[p].last_read = cur;
      if (gpio_vector[p].set.type == a8i2cG::kAnalog) {
        gpio_vector[p].values[0] = analogRead(gpio_vector[p].set.pin);
      } else {
        gpio_vector[p].values[0] =
            digitalRead(gpio_vector[p].set.pin) == HIGH ? 1 : 0;
      }
    }
  }
#endif
#ifdef HAS_DHT
  // DHT
  for (uint8_t dh = 0; dh < dht_vector_configured; dh++)
    loopDHT(&dht_vector[dh]);
#endif  // HAS_DHT
#ifdef HAS_ROTARY_ENCODER
  // ENCODER
  loopClickEncoder(&encoder_single);
#endif
#ifdef HAS_UART
  // UART
  while (Serial.available() > 0) {
    int byte = Serial.read();
    if (uart_single.buffer_len < sizeof(uart_single.buffer)) {
      uart_single.buffer_len++;
    } else {  // Buffer scroll.
      for (uint8_t p = 1; p < sizeof(uart_single.buffer); p++) {
        uart_single.buffer[p - 1] = uart_single.buffer[p];
        uart_single.overflow = true;
      }
    }
    uart_single.buffer[uart_single.buffer_len] = byte;
  }
#endif  // HAS_UART
}

void receiveI2CEvent(int numBytes) {
  a8i2cG::I2C_Packet_t inData;
#ifdef DEBUG_I2C_IN
  Serial.print(F("IN I2C Event:"));
  Serial.print(numBytes);
  Serial.print(F(":"));
  Serial.print(Wire.available());
  Serial.print(F(":"));
  Serial.print(sizeof(a8i2cG::I2C_Packet_t));
  Serial.print(F(":"));
#endif
  if (numBytes > static_cast<int>(sizeof(a8i2cG::I2C_Packet_t))) {
    memset(&outData, 0x00, sizeof(a8i2cG::I2C_Packet_t));
    outData.reponse.data.error.code = a8i2cG::error_t::kInvalidPacketSize;
    sendOutData = true;
#ifdef DEBUG_I2C_IN
    Serial.print(F("*OWF*"));
#endif
  } else {
    if (numBytes > 0) {
      Wire.setTimeout(500);
      size_t readed =
          Wire.readBytes(inData.I2CPacket, sizeof(a8i2cG::I2C_Packet_t));
      if (readed < static_cast<uint8_t>(numBytes)) {
#ifdef DEBUG_I2C_IN
        Serial.println("Wrong num data read!");
#endif
      }

#ifdef DEBUG_I2C_IN
      for (size_t i = 0; i < readed; i++) {
        Serial.write(inData.I2CPacket[i]);
        Serial.write(" [");
        Serial.print(inData.I2CPacket[i], HEX);
        Serial.write("] ");
      }
      Serial.print(readed == numBytes ? F(":OK") : F(":KO"));
#endif
      // Empty the bus!
      while (Wire.available()) Wire.read();

      // Do request work.
      switch (inData.request.device) {
#ifdef HAS_GPIO
        case a8i2cG::device_t::kGpio:
          switch (inData.request.cmd) {
            case a8i2cG::cmd_t::kSet:
              pinMode(inData.request.data.set.cmd_gpio_set.pin,
                      inData.request.data.set.cmd_gpio_set.mode);
              if (gpio_vector_configured < MAX_GPIO) {
                int8_t fp =
                    a8i2cG::findGPIO(inData.request.data.set.cmd_gpio_set.pin,
                                     gpio_vector, MAX_GPIO);
                if (fp >= 0) {
                  memcpy(&gpio_vector[fp].set,
                         &inData.request.data.set.cmd_gpio_set,
                         sizeof(a8i2cG::cmd_gpio_set_t));
                } else {
                  memcpy(&gpio_vector[gpio_vector_configured].set,
                         &inData.request.data.set.cmd_gpio_set,
                         sizeof(a8i2cG::cmd_gpio_set_t));
                  gpio_vector_configured++;
                }
              } else {
                memset(&outData, 0x00, sizeof(a8i2cG::I2C_Packet_t));
                outData.reponse.device = a8i2cG::kGpio;
                outData.reponse.data.error.code =
                    a8i2cG::error_t::kMaxCapacityFull;
                sendOutData = true;
              }
              break;
            case a8i2cG::cmd_t::kRead: {
              int8_t fp =
                  a8i2cG::findGPIO(inData.request.data.read.cmd_gpio_read.pin,
                                   gpio_vector, gpio_vector_configured);
              memset(&outData, 0x00, sizeof(a8i2cG::I2C_Packet_t));
              outData.reponse.device = a8i2cG::kGpio;
              if (fp >= 0) {
                memcpy(&outData.reponse.data, &gpio_vector[fp],
                       sizeof(a8i2cG::cmd_gpio_data_t));
                gpio_vector[fp].values_len = 0;
              } else {
                outData.reponse.data.error.code = a8i2cG::error_t::kInvalidPin;
              }
              sendOutData = true;
              break;
            }
            case a8i2cG::cmd_t::kWrite: {
              int8_t fp =
                  a8i2cG::findGPIO(inData.request.data.read.cmd_gpio_read.pin,
                                   gpio_vector, gpio_vector_configured);
              if (gpio_vector[fp].set.type == a8i2cG::kAnalog) {
                analogWrite(inData.request.data.write.cmd_gpio_write.pin,
                            inData.request.data.write.cmd_gpio_write.value);
              } else {
                digitalWrite(inData.request.data.write.cmd_gpio_write.pin,
                             inData.request.data.write.cmd_gpio_write.value);
              }
              break;
            }
            default:
              memset(&outData, 0x00, sizeof(a8i2cG::I2C_Packet_t));
              outData.reponse.device = a8i2cG::kGpio;
              outData.reponse.data.error.code =
                  a8i2cG::error_t::kInvalidCommand;
              sendOutData = true;
              break;
          }
          break;
#endif
#ifdef HAS_DHT
        case a8i2cG::device_t::kDht:
          switch (inData.request.cmd) {
            case a8i2cG::cmd_t::kSet:
              if (dht_vector_configured < MAX_DHT &&
                  inData.request.data.set.cmd_dht11_set.samples <
                      HAS_DHT_MAX_TEMP_COUNT) {
                int8_t fp =
                    a8i2cG::findDHT(inData.request.data.set.cmd_dht11_set.pin,
                                    dht_vector, MAX_DHT);
                if (fp >= 0) {
                  memcpy(&dht_vector[fp].set,
                         &inData.request.data.set.cmd_dht11_set,
                         sizeof(a8i2cG::cmd_dht11_set_t));
                } else {
                  memcpy(&dht_vector[dht_vector_configured].set,
                         &inData.request.data.set.cmd_dht11_set,
                         sizeof(a8i2cG::cmd_dht11_set_t));
                  dht_vector_configured++;
                }
              } else {
                memset(&outData, 0x00, sizeof(a8i2cG::I2C_Packet_t));
                outData.reponse.device = a8i2cG::kDht;
                outData.reponse.data.error.code =
                    a8i2cG::error_t::kMaxCapacityFull;
                sendOutData = true;
              }
              break;
            case a8i2cG::cmd_t::kRead: {
              int8_t fp =
                  a8i2cG::findDHT(inData.request.data.read.cmd_dht11_read.pin,
                                  dht_vector, MAX_DHT);
              memset(&outData, 0x00, sizeof(a8i2cG::I2C_Packet_t));
              outData.reponse.device = a8i2cG::kDht;
              if (fp >= 0) {
                memcpy(&outData.reponse.data, &dht_vector[fp],
                       sizeof(a8i2cG::cmd_dht11_data_t));
                dht_vector[fp].humidity = __FLT_MIN__;
                dht_vector[fp].temperature = __FLT_MIN__;
              } else {
                outData.reponse.data.error.code = a8i2cG::error_t::kInvalidPin;
              }
              sendOutData = true;
              break;
            }
            default:
              memset(&outData, 0x00, sizeof(a8i2cG::I2C_Packet_t));
              outData.reponse.device = a8i2cG::kDht;
              outData.reponse.data.error.code =
                  a8i2cG::error_t::kInvalidCommand;
              sendOutData = true;
              break;
          }
          break;
#endif
#ifdef HAS_ROTARY_ENCODER
        case a8i2cG::device_t::kEncoder:
          switch (inData.request.cmd) {
            case a8i2cG::cmd_t::kSet:
              setUpEncoder(inData.request.data.set.cmd_encoder_set);
              memcpy(&encoder_single.set,
                     &inData.request.data.set.cmd_encoder_set,
                     sizeof(a8i2cG::cmd_encoder_data_t));
              break;
            case a8i2cG::cmd_t::kRead: {
              memcpy(&outData.reponse.data, &encoder_single,
                     sizeof(a8i2cG::cmd_encoder_data_t));
              outData.reponse.device = a8i2cG::kEncoder;
              encoder_single.select = a8i2cG::kOpen;
              sendOutData = true;
              break;
            }
            default:
              memset(&outData, 0x00, sizeof(a8i2cG::I2C_Packet_t));
              outData.reponse.device = a8i2cG::kEncoder;
              outData.reponse.data.error.code =
                  a8i2cG::error_t::kInvalidCommand;
              sendOutData = true;
              break;
          }
          break;
#endif
#ifdef HAS_UART
        case a8i2cG::device_t::kUart:
          switch (inData.request.cmd) {
            case a8i2cG::cmd_t::kSet:
              Serial.begin(inData.request.data.set.cmd_uart_set.speed);
              break;
            case a8i2cG::cmd_t::kRead: {
              memcpy(&outData.reponse.data, &uart_single,
                     sizeof(a8i2cG::cmd_uart_data_t));
              outData.reponse.device = a8i2cG::kUart;
              uart_single.buffer_len = 0x00;
              uart_single.overflow = false;
              sendOutData = true;
              break;
            }
            case a8i2cG::cmd_t::kWrite:
              Serial.write(inData.request.data.write.cmd_uart_write.buffer,
                           inData.request.data.write.cmd_uart_write.buffer_len);
              break;
            default:
              memset(&outData, 0x00, sizeof(a8i2cG::I2C_Packet_t));
              outData.reponse.device = a8i2cG::kUart;
              outData.reponse.data.error.code =
                  a8i2cG::error_t::kInvalidCommand;
              sendOutData = true;
              break;
          }
          break;
#endif
        default:
          memset(&outData, 0x00, sizeof(a8i2cG::I2C_Packet_t));
          outData.reponse.data.error.code = a8i2cG::error_t::kInvalidDevice;
          sendOutData = true;
          break;
      }
    }
  }
}

void requestI2CEvent() { numI2CRequest++; }
