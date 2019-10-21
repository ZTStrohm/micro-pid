/*======================================================
   PID.cpp - Library for use in uPID Controller
   Creator: Zachary Strohm
   Date: 10/21/2019
   Version: 0.1
   =====================================================
   Licensed Under GNU GENERAL PUBLIC LICENSE Version 3
   =====================================================
   http://github.com/ZTStrohm/micro-pid/
   =====================================================
*/
#include "Arduino.h"
#include "PID.h"
PID::PID(float _sp, float _Kp, float _Ki, float _Kd, float _dt);
{
  sp = _sp;
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
  dt = _dt;
  pv = 0;
  previous_error = 0;
}
float PID::PIDCalc(int _Mode)
{
  Mode = _Mode;
  error = sp - pv;
  integral = error + previous_error * dt;
  derivative = (error - previous_error) / dt;
  //Selects between PID,PI and P Controllers
  switch (Mode)
  {
    case 1:
      cv = Kp + Ki * integral + Kd * derivative;
      break;
    case 2:
      cv = Kp + Ki * integral;
      break;
    case 3:
      cv = Kp;
      break;
    default:
      cv = Kp + Ki * integral + Kd * derivative;
      break;
  }
  previous_error = error;
  return cv;
}
