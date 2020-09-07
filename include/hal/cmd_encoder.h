#pragma once
#include "Config.h"

#ifdef HAS_ROTARY_ENCODER
#include <ClickEncoder.h>
#include <protocol/protocol.h>
int16_t oldEncPos, encPos;
uint8_t buttonState;

// ClickEncoder encoder(ENCODERCLKPIN, ENCODERDTPIN, ENCODERSWPIN, STEPS, LOW);
ClickEncoder *encoder;
// void timerIsr() { encoder->service(); }

void setUpEncoder(a8i2cG::cmd_encoder_set_t set) {
  encoder = new ClickEncoder(set.clkpin, set.dtpin, set.swpin, set.steps, LOW);
  encoder->setAccelerationEnabled(true);
  oldEncPos = -1;
}

void loopClickEncoder(a8i2cG::cmd_encoder_data_t *data) {
  encoder->service();
  encPos += encoder->getValue();
  buttonState = encoder->getButton();

  if (encPos != oldEncPos) {
    data->encoder = encPos;
    oldEncPos = encPos;
#ifdef DEBUG_EVENT
    Serial.print(F("Encoder Value: "));
    Serial.println(encPos);
#endif
  }

  if (buttonState != 0) {
#ifdef DEBUG_EVENT
    Serial.print("Button: ");
    Serial.println(buttonState);
#endif
    switch (buttonState) {
      case ClickEncoder::Open:  // 0
        data->select = a8i2cG::kOpen;
        break;
      case ClickEncoder::Closed:  // 1
        data->select = a8i2cG::kClosed;
        break;
      case ClickEncoder::Pressed:  // 2
        data->select = a8i2cG::kPressed;
        break;
      case ClickEncoder::Held:  // 3
        data->select = a8i2cG::kHeld;
        break;
      case ClickEncoder::Released:  // 4
        data->select = a8i2cG::kReleased;
        break;
      case ClickEncoder::Clicked:  // 5
        data->select = a8i2cG::kClicked;
        break;
      case ClickEncoder::DoubleClicked:  // 6
        data->select = a8i2cG::kDoubleClicked;
        break;
    }
  }
}

#endif
