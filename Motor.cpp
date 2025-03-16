#include "Motor.h"
#include "QEI.h"
#include "C12832.h"

/**
 * If you want the Motor class to reference your global QEI objects
 * (encoderLeft, encoderRight), your global LCD (lcd), or your global
 * pidControl function, you’ll need these `extern` declarations:
 */
extern QEI encoderLeft;
extern QEI encoderRight;
extern C12832 lcd;

// If your pidControl is also in another file:
extern float pidControl(float setpoint, float currentValue);

Motor::Motor(PinName enablePin, 
             PinName LpwmPin, 
             PinName RpwmPin,
             PinName dirLPin, 
             PinName dirRPin, 
             PinName bipolarLPin, 
             PinName bipolarRPin,
             bool inverted)
    : _enable(enablePin),
      _Lpwm(LpwmPin),
      _Rpwm(RpwmPin),
      _dirL(dirLPin),
      _dirR(dirRPin),
      _bipolarL(bipolarLPin),
      _bipolarR(bipolarRPin),
      _inverted(inverted) 
{
    // By default, disable the driver
    _enable = 0;

    // Set PWM frequency. 20kHz => period_us(50)
    _Lpwm.period_us(50);
    _Rpwm.period_us(50);

    // Configure bipolar mode pins (depends on your hardware)
    _bipolarL = 0;
    _bipolarR = 0;

    // Default directions
    _dirL = 1; 
    _dirR = 1;
}

void Motor::setEnable(int enableVal) {
    _enable = enableVal;
}

void Motor::setSpeedL(float speed) {
    // If inverted is true, invert the logic
    if (_inverted) {
        _Lpwm = speed; 
    } else {
        _Lpwm = 1.0f - speed;
    }
}

void Motor::setSpeedR(float speed) {
    if (_inverted) {
        _Rpwm = speed;
    } else {
        _Rpwm = 1.0f - speed;
    }
}

void Motor::setDirectionLForward() {
    _dirL = 1; // forward
}

void Motor::setDirectionRForward() {
    _dirR = 1; // forward
}

void Motor::turnRight(float speed) {
    // Left channel spins, right channel stops
    if (_inverted) {
        _Lpwm = speed;   // spin
        _Rpwm = 0.0f;    // off
    } else {
        _Lpwm = 1.0f - speed; 
        _Rpwm = 1.0f; // “off”
    }
}

void Motor::turnLeft(float speed) {
    // Right channel spins, left channel stops
    if (_inverted) {
        _Lpwm = 0.0f;   
        _Rpwm = speed; 
    } else {
        _Lpwm = 1.0f;   
        _Rpwm = 1.0f - speed;
    }
}

void Motor::stop() {
    // Immediately stop both channels
    if (_inverted) {
        _Lpwm = 0.0f;
        _Rpwm = 0.0f;
    } else {
        _Lpwm = 1.0f;
        _Rpwm = 1.0f;
    }
}

void Motor::moveForward() {
    // Example function that uses your global QEI encoders and pidControl
    // 1) set directions to forward
    _dirL = 1; 
    _dirR = 1;

    // 2) read pulses from global QEI objects
    int leftCounts  = encoderLeft.getPulses();
    int rightCounts = encoderRight.getPulses();

    // 3) do any correction you like
    float correction = pidControl((float)rightCounts, (float)leftCounts);

    // 4) set PWM using that correction
    //    for demonstration: base speed is 0.6, subtract correction from the right side
    //    adjust for inverted if necessary
    if (_inverted) {
        _Lpwm = 0.4f;               // example
        _Rpwm = 0.4f - correction;  // example
    } else {
        _Lpwm = 0.6f;               
        _Rpwm = 0.6f - correction;  
    }
}

void Motor::calculateRPM() {
    // Example placeholder:
    // (You can also store pulses from last iteration if you want.)
    int leftPulses  = encoderLeft.getPulses();
    int rightPulses = encoderRight.getPulses();
    // ... do your calculation
}

void Motor::updateLCD() {
    // Example placeholder for printing something to the LCD
    lcd.locate(0, 0);
    lcd.printf("Left: %d  Right: %d", encoderLeft.getPulses(), encoderRight.getPulses());
}
