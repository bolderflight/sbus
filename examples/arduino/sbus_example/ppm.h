#ifndef _PPM_h_
#define _PPM_h_

#include "Arduino.h"

#define PPM_DEFAULT_CHANNELS 16

#define PPM_PULSE_LENGTH_uS 500      // TODO
#define PPM_FRAME_LENGTH_uS 22500    // TODO

class PPMEncoder {

  private:
    int16_t channels[18];
    uint16_t elapsedUs;

    uint8_t numChannels;
    uint8_t currentChannel;
    byte outputPin;
    boolean state;
    boolean enabled;
	
    uint8_t onState;
    uint8_t offState;


  public:
    static const uint16_t MIN = 900;
    static const uint16_t MAX = 2100;

    void setChannel(uint8_t channel, uint16_t value);
    void setChannelPercent(uint8_t channel, uint8_t percent);

    void begin(uint8_t pin);
    void begin(uint8_t pin, uint8_t ch);
    void begin(uint8_t pin, uint8_t ch, boolean inverted);

    void disable();
    void enable();

    void interrupt();
};

//extern PPMEncoder ppmEncoder;

void PPMEncoder::begin(uint8_t pin) {
  begin(pin, PPM_DEFAULT_CHANNELS, false);
}

void PPMEncoder::begin(uint8_t pin, uint8_t ch) {
  begin(pin, ch, false);
}

void PPMEncoder::begin(uint8_t pin, uint8_t ch, boolean inverted) {
  cli();

  // Store on/off-State in variable to avoid another if in timing-critical interrupt
  onState = (inverted) ? HIGH : LOW;
  offState = (inverted) ? LOW : HIGH;
  
  pinMode(pin, OUTPUT);
  digitalWrite(pin, offState);

  enabled = true;
  state = true;
  elapsedUs = 0;
  currentChannel = 0;

  numChannels = ch;
  outputPin = pin;

  for (uint8_t ch = 0; ch < numChannels; ch++) {
    setChannelPercent(ch, 0);
  }

  TCCR1A = 0;

  OCR1A = 100;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  TIMSK1 = (1 << OCIE1A); // enable timer compare interrupt

  sei();
}

void PPMEncoder::setChannel(uint8_t channel, uint16_t value) {
  channels[channel] = constrain(value, PPMEncoder::MIN, PPMEncoder::MAX);
}

void PPMEncoder::setChannelPercent(uint8_t channel, uint8_t percent) {
  percent = constrain(percent, 0, 100);
  setChannel(channel, map(percent, 0, 100, PPMEncoder::MIN, PPMEncoder::MAX));
}

void PPMEncoder::enable() {
 enabled = true;
}

void PPMEncoder::disable() {
 enabled = false;
 state = false;
 elapsedUs = 0;
 currentChannel = 0;
 
 digitalWrite(outputPin, offState);
}

void PPMEncoder::interrupt() {
  if (!enabled) {
    return;
  }

  TCNT1 = 0;

  if (state) {
    digitalWrite(outputPin, onState);
    OCR1A = PPM_PULSE_LENGTH_uS * 2;

  } else {
    digitalWrite(outputPin, offState);

    if (currentChannel >= numChannels) {
      currentChannel = 0;
      elapsedUs = elapsedUs + PPM_PULSE_LENGTH_uS;
      OCR1A = (PPM_FRAME_LENGTH_uS - elapsedUs) * 2;
      elapsedUs = 0;
    } else {
      OCR1A = (channels[currentChannel] - PPM_PULSE_LENGTH_uS) * 2;
      elapsedUs = elapsedUs + channels[currentChannel];

      currentChannel++;
    }
  }

  state = !state;
}

ISR(TIMER1_COMPA_vect) {
  ppmEncoder.interrupt();
}

#endif