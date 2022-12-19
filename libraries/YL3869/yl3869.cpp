#include "yl3869.h"

// Constructor
YL3869::YL3869(const unsigned int& pin, const bool& percentage = true, const float& v_res = 3.3, const unsigned int& adc_bit = 12) {
  _pin = pin;
  _perc = percentage;
  _v_res = v_res;
  adc_scale = int(pow(2, adc_bit) - 1);
}

// Destructor
YL3869::~YL3869() {}

void YL3869::init() {
  pinMode(_pin, INPUT);
}

float YL3869::read() {
  if (_perc) return map(analogRead(_pin), 0, adc_scale, 0, 100);
  else return analogRead(_pin);
}