# μ PID
## Intro
The μ PID is a multi-function PID controller designed to be a drop-in module for use in larger projects. It is compatible with the Arduino Nano footprint and may be used as a shield or as a standalone module. The Micro-PID is compatible with various sensors, including analog and digital sensors for position, speed, and temperature. Control is accomplished using an ATTiny 1614, using its internal ADC and DAC, with a buffered output. The intention of this device is to easily add PID control to a project. I will likely include accessory boards in the future for driving larger loads, motors, and more.
## Documentation
### Creating a PID Object:
`PID::PID(float _sp, float _Kp, float _Ki, float _Kd, float _dt);`

sp: Setpoint

Kp: Proportional Constant

Ki: Integral Constant

Kd: Derivative Constant

dt: Time Constant

#### Example:
```
/*This creates a New PID Controller, EX, with a setpoint of 1, Kp of 1.3,
Ki of 1.2, and Kd of 1.1 with a time constant of 0.001 Seconds.*/

PID EX(1,1.3, 1.2, 1.1, 0.001);
```

### Running PID Calculation:
`float PID::PIDCalc(int _Mode);`
Mode: Sets type of controller
- 1: PID Controller
- 2: PI Controller
- 3: P Controller
#### Example:
```
/* Using a previously declared PID object, EX, we calculate a PI controller */
float cvResult = EX.PIDCalc(2);
```

