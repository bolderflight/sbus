//
// title:     SBUS.h
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      YYYY-MM-DD 
// license: 
//

#ifndef SBUS_h
#define SBUS_h

#include "Arduino.h"					

class SBUS{
  public:
    SBUS(int bus);
    void begin();

  private:
  	int _bus;
};

#endif
