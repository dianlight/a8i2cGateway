#include <Arduino.h>
#include <Wire.h>

//#define DEBUG_I2C_OUT
//#define DEBUG_I2C_IN
#define DEBUG_EVENT



#define DHTPIN     PD7 
#include <dht.h>
dht DHT;

#define RELAYPIN   PD6

#define ENCODERSWPIN  PD5 
#define ENCODERCLKPIN PD4
#define ENCODERDTPIN  PD3

/*
#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>
Encoder myEnc(ENCODERCLKPIN, ENCODERDTPIN);
*/

// ClickEncoder
#include <ClickEncoder.h>
//#include <TimerOne.h>
//#include <Ticker.h>

int16_t oldEncPos, encPos;
uint8_t buttonState;

#define STEPS 4

// Analogici?!?
ClickEncoder encoder(ENCODERCLKPIN, ENCODERDTPIN, ENCODERSWPIN, STEPS, LOW);
void timerIsr() {
  encoder.service();
}

//Ticker timerIsrTicker(timerIsr,1000,0,MICROS);
// - ClickEncoder


#define I2CADDRESS 0x08 

enum __attribute__((__packed__)) COMMAND {
  RELAY = 'R',
};

struct __attribute__((packed, aligned(1))) sensorInValue_t {
    COMMAND cmd;
    bool    relay;
};

struct __attribute__((packed, aligned(1))) sensorOutValue_t {
    float   temperature;
    float   humidity;
    int8_t  encoder;
    byte    select;
    bool    relay;
};

union __attribute__((packed, aligned(1))) sensorValue_t {
    sensorInValue_t in;
    sensorOutValue_t out;
};

union  __attribute__((packed, aligned(1))) I2C_Packet_t {
    sensorValue_t sensorData;
    byte I2CPacket[sizeof(sensorValue_t)];
};



void requestI2CEvent();
void receiveI2CEvent(int numBytes);

void setup() {
  Serial.begin(9600);
  while(!Serial);

//  Wire.setClock(10000);
  Wire.begin(I2CADDRESS);
  Wire.onReceive(receiveI2CEvent);
  Wire.onRequest(requestI2CEvent);

  // ClickEncoder
//  Timer1.initialize(1000);
//  Timer1.attachInterrupt(timerIsr);
//  timerIsrTicker.start();

  encoder.setAccelerationEnabled(true);

  Serial.print("Acceleration is ");
  Serial.println((encoder.getAccelerationEnabled()) ? "enabled" : "disabled");

  oldEncPos = -1;
  // End Click Encoder

  pinMode(RELAYPIN,OUTPUT);

}

unsigned long lastDHTRead;

I2C_Packet_t outData;

void loopClickEncoder() {
  encoder.service();
  encPos += encoder.getValue();
  buttonState = encoder.getButton();

    if (encPos != oldEncPos) {
      outData.sensorData.out.encoder = encPos;
      oldEncPos = encPos;
      #ifdef DEBUG_EVENT
        Serial.print(F("Encoder Value: "));
        Serial.println(encPos);
      #endif
    }


  if (buttonState != 0) {
    outData.sensorData.out.select = buttonState;
    #ifdef DEBUG_EVENT
      Serial.print("Button: "); Serial.println(buttonState);
    #endif
    switch (buttonState) {
      case ClickEncoder::Open:          //0
        break;

      case ClickEncoder::Closed:        //1
        break;

      case ClickEncoder::Pressed:       //2
        break;

      case ClickEncoder::Held:          //3
        break;

      case ClickEncoder::Released:      //4
        break;

      case ClickEncoder::Clicked:       //5
        break;

      case ClickEncoder::DoubleClicked: //6
        break;
    }
  }
}

#define TEMP_COUNT 5
float temperature[TEMP_COUNT];
int8_t tppos = 1;

void loop() {
  loopClickEncoder();

  if(millis() - lastDHTRead > 2000){
    outData.sensorData.out.relay       = digitalRead(RELAYPIN) == HIGH;
    // READ DATA
    #ifdef DEBUG_EVENT
      Serial.print("DHT11, \t");
    #endif
    int chk = DHT.read11(DHTPIN);
      switch (chk)
      {
        case DHTLIB_OK:  
          #ifdef DEBUG_EVENT
            Serial.print("OK,\t"); 
            Serial.print(DHT.humidity, 1);
            Serial.print(",\t");
            Serial.println(DHT.temperature, 1);
          #endif
          temperature[tppos] = DHT.temperature;
          tppos=(tppos+1) % TEMP_COUNT;
          if(temperature[0] != 0){
            float avg = 0;
            for(byte i=0; i < TEMP_COUNT;i++){
              avg+=temperature[i];
            }
            outData.sensorData.out.temperature = avg / TEMP_COUNT; 
          }
//          outData.sensorData.out.temperature = DHT.temperature;
          outData.sensorData.out.humidity    = DHT.humidity;
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
    lastDHTRead = millis();
  }

/*
  if(outData.sensorData.out.select && millis() - lastENCButtonRead > 5000){
    outData.sensorData.out.select = false;
  }

  if(!outData.sensorData.out.select){
     outData.sensorData.out.select = digitalRead(ENCODERSWPIN) == 0;
     if(outData.sensorData.out.select)lastENCButtonRead = millis();
  }
  */

  //outData.sensorData.out.encoder     = myEnc.read();

  //delay(10);
}

#define OUT_BUFFER sizeof(I2C_Packet_t)
#define IN_BUFFER  sizeof(I2C_Packet_t)


void receiveI2CEvent(int numBytes) {
  I2C_Packet_t inData;
  #ifdef DEBUG_I2C_IN
    Serial.print(F("IN I2C Event:"));
    Serial.print(numBytes);
    Serial.print(F(":"));
    Serial.print(Wire.available());
    Serial.print(F(":"));
    Serial.print(IN_BUFFER);
    Serial.print(F(":"));
  #endif
  if(numBytes > (int)IN_BUFFER){
    #ifdef DEBUG_I2C_IN
     Serial.print(F("*OWF*"));
    #endif
  }
  if(numBytes > 0){
    Wire.setTimeout(500);
    size_t readed = Wire.readBytes(inData.I2CPacket,IN_BUFFER);

    #ifdef DEBUG_I2C_IN
      for(size_t i=0; i < readed; i++){
        Serial.write(inData.I2CPacket[i]);
        Serial.write(" [");
        Serial.print(inData.I2CPacket[i],HEX);
        Serial.write("] ");
      }
      Serial.print(readed == numBytes?F(":OK"):F(":KO"));
    #endif
    // Empty the bus!
    while(Wire.available())Wire.read();

    switch(inData.sensorData.in.cmd){
      case RELAY:
        #ifdef DEBUG_EVENT
          Serial.print("Relay PIN:");
          Serial.print(inData.sensorData.in.relay?"HIGH":"LOW");
        #endif
        digitalWrite(RELAYPIN,inData.sensorData.in.relay?HIGH:LOW);
        break;
      default:
        Serial.print("*ERR UNK:");
        Serial.print(inData.sensorData.in.cmd); 
        break;  
    }

  }
  #if defined(DEBUG_I2C_IN) || defined(DEBUG_EVENT)
    Serial.println();
  #endif

        //Visualizzazione. 
        /*
        I2C_Packet_t testData;
        testData.sensorData.in.cmd = RELAY;
        testData.sensorData.in.relay = true;
      for(size_t i=0; i < IN_BUFFER; i++){
        Serial.write(testData.I2CPacket[i]);
        Serial.write(" [");
        Serial.print(testData.I2CPacket[i],HEX);
        Serial.write("] ");
      }
    Serial.println();
*/

}

void requestI2CEvent() {
  #ifdef DEBUG_I2C_OUT
    Serial.print(F("OUT I2C Event:"));
    Serial.print(" -T-> ");
    Serial.print(outData.sensorData.out.temperature);
    Serial.print(" -H-> ");
    Serial.print(outData.sensorData.out.humidity);
    Serial.print(" -R-> ");
    Serial.print(outData.sensorData.out.relay);
    Serial.print(" -E-> ");
    Serial.print(outData.sensorData.out.encoder);
    Serial.print(" -S-> ");
    Serial.print(outData.sensorData.out.select);
    Serial.print(F("D:'"));
    for(byte i=0; i < OUT_BUFFER; i++){
      Serial.write(outData.I2CPacket[i]);
      Serial.write(" [");
      Serial.print(outData.I2CPacket[i],HEX);
      Serial.write("] ");
    }
    Serial.print("' --> ");
  #endif
  size_t sent = Wire.write(outData.I2CPacket,OUT_BUFFER);
  #ifdef DEBUG_I2C_OUT
    Serial.println(sent);
  #endif 
  outData.sensorData.out.select = 0;
}


