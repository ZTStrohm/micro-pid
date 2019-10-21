/*======================================================
 * PID.h - Library for use in uPID Controller
 * Creator: Zachary Strohm
 * Date: 10/21/2019
 * Version: 0.1
 * =====================================================
 * Licensed Under GNU GENERAL PUBLIC LICENSE Version 3
 * =====================================================
 * http://github.com/ZTStrohm/micro-pid/
 * =====================================================
 */

#ifndef PID_h
#define PID_h
#include "Arduino.h"
 
class PID
{
  public:
    PID(int cvPin, int pvPin);
    void setSP();
    float cvCalc();
  private:
    bool enable;
    float sp, cv, pv;
};

#endif
