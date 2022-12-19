#ifndef YL3869_H_
#define YL3869_H_

#include "Arduino.h"

class YL3869 {
  public:
    YL3869(const unsigned int& pin, const bool& percentage = true, const float& v_res = 3.3, const unsigned int& adc_bit = 12);
    ~YL3869();
    void init();
    float read();

  private:
    unsigned int _pin;
    unsigned int adc_scale;
    float _v_res;
    bool _perc;
};

#endif // YL_3869_H_