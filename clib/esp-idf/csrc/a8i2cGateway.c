#include "a8i2cGateway.h"

#include <string.h>
#include <freertos/FreeRTOS.h>
//#include <freertos/event_groups.h>
//#include <freertos/task.h>

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>
#include <i2cdev.h>

static const char* TAG = "a8i2cGateway";

#define I2C_FREQ_HZ 1000000

static i2c_dev_t device;

esp_err_t a8i2cGw_init(int sda, int scl) {
  ESP_LOGD(TAG, "Request init of i2c device %02X.", CONFIG_A8I2CGW_I2C_ADDRESS);
  memset(&device, 0, sizeof(i2c_dev_t));
  device.port = 0;
  device.addr = CONFIG_A8I2CGW_I2C_ADDRESS;
  device.cfg.sda_io_num = sda;
  device.cfg.scl_io_num = scl;
  device.cfg.master.clk_speed = I2C_FREQ_HZ;
  ESP_ERROR_CHECK(i2c_dev_create_mutex(&device));
  // vTaskDelay(1000 / portTICK_PERIOD_MS / uart->baudrate);
  ESP_LOGD(TAG, "Done init of i2c device");
  //  if (xTaskCreate(tcp_server_task, "a8i2c_watcher", 4096, NULL, 5, NULL) ==
  //      pdPASS) {
  //    ESP_LOGD(TAG, "Create A8i2cGw listener task");
  //    return ESP_OK;
  //  } else {
  //    return ESP_FAIL;
  //  }
  return ESP_OK;
}
/*
void a8i2cGw_send(I2C_Packet_t packet) {
  ESP_LOGD(TAG, "Send %d bytes to i2c", sizeof(I2C_Packet_t));
  I2C_DEV_TAKE_MUTEX(&device);
  I2C_DEV_CHECK(&device,
                i2c_dev_write(&device, NULL, 0, &packet, sizeof(I2C_Packet_t)));
  I2C_DEV_GIVE_MUTEX(&device);
}
*/

esp_err_t a8i2cGw_read(I2C_Packet_t send, I2C_Packet_t* receive) {
  ESP_LOGD(TAG, "Send for read %d bytes to i2c", sizeof(I2C_Packet_t));
  I2C_DEV_TAKE_MUTEX(&device);
  I2C_DEV_CHECK(&device, i2c_dev_read(&device, &send, sizeof(I2C_Packet_t),
                                      receive, sizeof(I2C_Packet_t)));
  I2C_DEV_GIVE_MUTEX(&device);
  return ESP_OK;
}

// Generic
int8_t a8i2cGw_packet_io(request_t pkt, response_t* rkt) {
  a8i2cGw_read((I2C_Packet_t)pkt, (I2C_Packet_t *)&rkt);
  if (rkt->rescode >= 0) {
    return rkt->rescode;
  } else {
    ESP_LOGE(TAG, "Error %X", rkt->data.error.code);
    return -1;
  }
}

int8_t a8i2cGw_cmd_set(device_t device, cmd_set_t set) {
  I2C_Packet_t pkt, rkt;
  pkt.request.device = device;
  pkt.request.cmd = kSet;
  memcpy(&pkt.request.data, &set, sizeof(cmd_set_t));
  return a8i2cGw_packet_io(pkt.request, &rkt.reponse);
}

int8_t a8i2cGw_cmd_write(device_t device, cmd_write_t write) {
  I2C_Packet_t pkt, rkt;
  pkt.request.device = device;
  pkt.request.cmd = kWrite;
  memcpy(&pkt.request.data, &write, sizeof(cmd_write_t));
  return a8i2cGw_packet_io(pkt.request, &rkt.reponse);
}

int8_t a8i2cGw_cmd_read(device_t device, cmd_read_t write, cmd_data_t* data) {
  I2C_Packet_t pkt, rkt;
  pkt.request.device = device;
  pkt.request.cmd = kRead;
  memcpy(&pkt.request.data, &write, sizeof(cmd_read_t));
  int8_t res = a8i2cGw_packet_io(pkt.request, &rkt.reponse);
  memcpy(data, &rkt.reponse.data.data, sizeof(cmd_data_t));
  return res;
}

// GPIO
#ifdef HAS_GPIO
int8_t setupGpio(cmd_gpio_set_t set) { return a8i2cGw_cmd_set(kGpio, (cmd_set_t)set); }

int8_t writeGpio(uint8_t pin, uint16_t value) {
  cmd_gpio_write_t data;
  data.pin = pin;
  data.value = value;
  return a8i2cGw_cmd_write(kGpio, (cmd_write_t)data);
}
int8_t readGpio(uint8_t pin, cmd_gpio_data_t* data) {
  cmd_gpio_read_t rdata;
  rdata.pin = pin;
  return a8i2cGw_cmd_read(kGpio, (cmd_read_t)rdata, (cmd_data_t *)data);
}
int16_t readLastGpio(uint8_t pin) {
  cmd_gpio_read_t rdata;
  cmd_gpio_data_t data;
  rdata.pin = pin;
  if( a8i2cGw_cmd_read(kGpio, (cmd_read_t)rdata, (cmd_data_t *)&data) >= 0){
    return data.values[0];
  } else {
    return -1;
  }
}
#endif

// UART
#ifdef HAS_UART
int8_t setupUart(cmd_uart_set_t set) { return a8i2cGw_cmd_set(kUart, (cmd_set_t)set); }

int8_t writeUart(uint8_t value[], size_t len) {
  cmd_uart_write_t data;
  memcpy(&data.buffer, value, len);
  data.buffer_len = len;
  return a8i2cGw_cmd_write(kUart, (cmd_write_t)data);
}
int8_t readUart(cmd_uart_data_t *data) {
  cmd_uart_read_t rdata;
  return a8i2cGw_cmd_read(kUart, (cmd_read_t)rdata, (cmd_data_t *)data);
}
#endif

// SYSCTR
#ifdef HAS_SYSCTR
int8_t setupSysCtr(cmd_sysctr_set_t set) { return a8i2cGw_cmd_set(kSysCtr, (cmd_set_t)set); }
#endif

// DHT
#ifdef HAS_DHT
int8_t setupDht(cmd_dht11_set_t set) { return a8i2cGw_cmd_set(kDht, (cmd_set_t)set); }

int8_t readDht(uint8_t id, cmd_dht11_data_t *data) {
  cmd_dht11_read_t rdata;
  rdata.pin = id;
  return a8i2cGw_cmd_read(kDht, (cmd_read_t)rdata, (cmd_data_t *)data);
}
#endif

// ROTARY ENCODER
#ifdef HAS_ROTARY_ENCODER
int8_t setupRotaryEncoder(cmd_encoder_set_t set) { return a8i2cGw_cmd_set(kDht, (cmd_set_t)set); }
int8_t readRotaryEncoder(cmd_encoder_data_t *data) {
  cmd_encoder_read_t rdata;
  return a8i2cGw_cmd_read(kDht, (cmd_read_t)rdata, (cmd_data_t *)data);
}
#endif



