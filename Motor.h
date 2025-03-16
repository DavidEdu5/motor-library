#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

/**
 * Motor class for controlling a single dual-channel motor driver.
 * 
 * If your Motor class needs to read encoders or perform PID control,
 * you can either:
 *   1) Use extern references to global objects/functions in Motor.cpp, or
 *   2) Pass references to the QEI objects and/or PID functions into the Motor 
 *      class methods/constructor for a more modular design.
 */
class Motor {
private:
    DigitalOut _enable;   // Motor Enable Pin
    PwmOut     _Lpwm;     // PWM output for the left driver channel
    PwmOut     _Rpwm;     // PWM output for the right driver channel
    DigitalOut _dirL;     // Direction control for the left channel
    DigitalOut _dirR;     // Direction control for the right channel
    DigitalOut _bipolarL; // Bipolar mode control pin (left channel)
    DigitalOut _bipolarR; // Bipolar mode control pin (right channel)
    bool       _inverted; // If true, invert the PWM logic

public:
    /**
     * Constructor: Initializes motor control pins and sets PWM frequency.
     *
     * @param enablePin     Pin to enable/disable the motor driver
     * @param LpwmPin       PWM pin for left channel
     * @param RpwmPin       PWM pin for right channel
     * @param dirLPin       DigitalOut pin for left channel direction
     * @param dirRPin       DigitalOut pin for right channel direction
     * @param bipolarLPin   Pin to configure left channel in bipolar mode
     * @param bipolarRPin   Pin to configure right channel in bipolar mode
     * @param inverted      Whether to invert PWM signals (default=false)
     */
    Motor(PinName enablePin, 
          PinName LpwmPin, 
          PinName RpwmPin,
          PinName dirLPin, 
          PinName dirRPin, 
          PinName bipolarLPin, 
          PinName bipolarRPin,
          bool inverted = false);

    /**
     * Enable or disable the motor driver.
     * @param enableVal 1 = enabled, 0 = disabled
     */
    void setEnable(int enableVal);

    /**
     * Set speed for the left channel (0.0 to 1.0).
     */
    void setSpeedL(float speed);

    /**
     * Set speed for the right channel (0.0 to 1.0).
     */
    void setSpeedR(float speed);

    /**
     * Set left direction to forward (1) or reverse (0).
     */
    void setDirectionLForward();

    /**
     * Set right direction to forward (1) or reverse (0).
     */
    void setDirectionRForward();

    /**
     * Turn right by spinning left channel, disabling right channel.
     * @param speed  Speed for left channel (0.0 to 1.0).
     */
    void turnRight(float speed);

    /**
     * Turn left by spinning right channel, disabling left channel.
     * @param speed  Speed for right channel (0.0 to 1.0).
     */
    void turnLeft(float speed);

    /**
     * Immediately stop both channels.
     */
    void stop();

    /**
     * Example “move_forward” function that uses your global QEI encoders
     * and a global pidControl() for correction. Adjust to suit your needs.
     */
    void moveForward();

    /**
     * Example function to calculate RPM. 
     * Relies on global encoder objects if you choose.
     */
    void calculateRPM();

    /**
     * Example function to update an attached LCD. 
     * Relies on a global LCD object if you choose.
     */
    void updateLCD();
};

#endif // MOTOR_H
