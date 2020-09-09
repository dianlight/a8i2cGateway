#pragma once
#include "Config.h"

#ifdef HAS_ROTARY_ENCODER
#include <ClickEncoder.h>
#include <hal/cmd_log.h>
#include <protocol/protocol.h>
int16_t oldEncPos, encPos;
uint8_t buttonState;

ClickEncoder *encoder;

void setUpEncoder(cmd_encoder_set_t set) {
  encoder = new ClickEncoder(set.clkpin, set.dtpin, set.swpin, set.steps, LOW);
  encoder->setAccelerationEnabled(true);
  oldEncPos = -1;
}

void loopClickEncoder(cmd_encoder_data_t *data) {
  encoder->service();
  encPos += encoder->getValue();
  buttonState = encoder->getButton();

  if (encPos != oldEncPos) {
    data->encoder = encPos;
    oldEncPos = encPos;
#ifdef DEBUG_EVENT
    log_d("EV: %d", encPos);
#endif
  }

  if (buttonState != 0) {
#ifdef DEBUG_EVENT
    log_d("EB: %X", buttonState);
#endif
    switch (buttonState) {
      case ClickEncoder::Open:  // 0
        data->select = kOpen;
        break;
      case ClickEncoder::Closed:  // 1
        data->select = kClosed;
        break;
      case ClickEncoder::Pressed:  // 2
        data->select = kPressed;
        break;
      case ClickEncoder::Held:  // 3
        data->select = kHeld;
        break;
      case ClickEncoder::Released:  // 4
        data->select = kReleased;
        break;
      case ClickEncoder::Clicked:  // 5
        data->select = kClicked;
        break;
      case ClickEncoder::DoubleClicked:  // 6
        data->select = kDoubleClicked;
        break;
    }
  }
}

#endif
