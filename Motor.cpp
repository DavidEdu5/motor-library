#include "mbed.h"
#include "QEI.h"
#include "C12832.h"

// Define Encoder CPR
#define CPR 612

// Create QEI objects for both encoders
QEI encoderLeft(PC_10, PC_12, NC, 1024, QEI::X4_ENCODING); // Left Wheel Encoder
QEI encoderRight(PB_13, PB_14, NC, 1024, QEI::X4_ENCODING); // Right Wheel Encoder

// Initialize LCD (C12832)
C12832 lcd(D11, D13, D12, D7, D10); // SPI Pins for LCD

// Display Mode and Toggle Functionality
// -----------------------------
enum DisplayMode { MOTOR_DATA, SENSOR_DATA };
volatile DisplayMode currentDisplayMode = MOTOR_DATA;


void fireButtonPressed() {
if (currentDisplayMode == MOTOR_DATA){
currentDisplayMode = SENSOR_DATA;}
else {
currentDisplayMode = MOTOR_DATA;}
lcd.cls();
}
// automode decleration for main
bool autoMode = false;

void downButtonPressed() {
autoMode = true;
}

// Pin Assignments
AnalogIn potL(A0); // Potentiometer for Left Wheel
AnalogIn potR(A1); // Potentiometer for Right Wheel
int setter = 0;

// Timer for calculating RPM
Ticker rpm_ticker;

// Timer for LCD updates
Ticker display_update;

// Global variables for RPM calculation
volatile int lastLeftPulses = 0;
volatile int lastRightPulses = 0;
volatile float leftRPM = 0.0;
volatile float rightRPM = 0.0;
volatile float VelocityL = 0.0/*leftRPM*0.0041*/;
volatile float VelocityR = 0.0;

// Function prototyping
float pidControl(float setpoint, float currentValue);
void resetPID();
void fireButtonPressed();
void downButtonPressed();

// PID Constants
const float Kp = 0.00029; // Proportional gain
const float Ki = 0.000000005; // Integral gain
const float Kd = 0.0045; // Derivative gain

// PID Variables
float previousError = 0;
float integral = 0;

// speed
const float speed = 0.4;

// Motor Class
class Motor {
private:
DigitalOut enable; // Motor Enable Pin
PwmOut Rpwm; // PWM output for motor speed control
PwmOut Lpwm; // PWM output for motor speed control
DigitalOut dirL; // Direction control pin
DigitalOut dirR; // Direction control pin
DigitalOut bipolarL; // Bipolar mode control pin
DigitalOut bipolarR;
bool inverted; // Whether PWM is inverted

public:
// Constructor
Motor(PinName enablePin, PinName LpwmPin, PinName RpwmPin, PinName dirLPin, PinName dirRPin, PinName bipolarLPin, PinName bipolarRPin, bool inverted = false)
: enable(enablePin), Lpwm(LpwmPin), Rpwm(RpwmPin), dirR(dirRPin), dirL(dirLPin), bipolarL(bipolarLPin), bipolarR(bipolarRPin), inverted(inverted) {
enable = 0;
Lpwm.period_us(50); // Set PWM frequency to 20kHz
Rpwm.period_us(50); // Set PWM frequency to 20kHz
bipolarL = 0; // Set bipolar mode
bipolarR = 0;
dirL = 1;
dirR = 1;
}
void setEnable(int enableVal){
enable = 1;
}
// Set motor speed (duty cycle: 0.0 to 1.0)
void setSpeedL(float speed) {
Lpwm = 1-speed;
}

// Set motor speed (duty cycle: 0.0 to 1.0)
void setSpeedR(float speed) {
Rpwm = 1 - speed;
}

// Set motor direction (1 = forward, 0 = reverse)
void setDirectionL() {
dirL = 1;
}

// Set motor direction (1 = forward, 0 = reverse)
void setDirectionR() {
dirR = 1;
}

void turn_right(float speed) {
Lpwm = 1 - speed; // Inverted PWM
Rpwm = 1; // Right motor off (inverted)
}

void turn_left(float speed) {
Lpwm = 1; // Left motor off (inverted)
Rpwm = 1 - speed; // Inverted PWM
}

// Stop the motor
void stop() {
Lpwm = 1 ; // Stop motor (inverted or normal)
Rpwm = 1 ;
}

// Motor Control Functions
void move_forward() {
dirL = 1; // Both motors forward
dirR = 1;
// Get encoder counts
int leftCounts = encoderLeft.getPulses();
int rightCounts = encoderRight.getPulses();
// Calculate PID correction
float correction = pidControl(rightCounts, leftCounts);
// Apply correction to left wheel PWM
Lpwm = 0.6; // Inverted PWM
Rpwm = 0.6 - correction; // Inverted PWM
}

// Function to calculate RPM every second
void calculateRPM() {
// Get current pulse counts
int currentLeftPulses = encoderLeft.getPulses();
int currentRightPulses = encoderRight.getPulses();

// Calculate pulses per second
int leftPulsesPerSecond = currentLeftPulses - lastLeftPulses;
int rightPulsesPerSecond = currentRightPulses - lastRightPulses;

// Convert to RPM
leftRPM = ((float)leftPulsesPerSecond / CPR) * 60.0;
rightRPM = ((float)rightPulsesPerSecond / CPR) * 60.0;

VelocityL =(leftRPM * 2 * 3.14 * 0.04) / 60 ;
VelocityR = (rightRPM * 2 * 3.14 * 0.04) / 60 ;

// Store current pulses for next calculation
lastLeftPulses = currentLeftPulses;
lastRightPulses = currentRightPulses;
}

// Function to update the LCD display
void updateLCD1() {
// Clear screen once at the beginning
static bool firstRun = true;
if (firstRun) {
lcd.cls();
firstRun = false;
}

// Display pulse counts on the first line
lcd.locate(0, 0);
lcd.printf("L: %d R: %d", encoderLeft.getPulses(), encoderRight.getPulses());

// Display RPM values
lcd.locate(0, 10); // display on line 2
lcd.printf("L: %.2f RPM R: %.2f RPM", leftRPM, rightRPM);

// Display RPM values
lcd.locate(0, 20); // display on line 2
lcd.printf("veLm/s: %.2f veR(m/s): %.2f ", VelocityL, VelocityR);

// Display PWM duty cycles (we can remove if we want)
lcd.locate(55, 0); // display on line 3
lcd.printf("PL: %.2f PR: %.2f", potL.read(), potR.read());
}

// Function to update the LCD display based on the current display mode
void updateLCD() {
static bool firstRun = true;
if (firstRun) {
lcd.cls();
firstRun = false;
}
if (currentDisplayMode == MOTOR_DATA) {
lcd.locate(0, 0);
lcd.printf("L: %d R: %d", encoderLeft.getPulses(), encoderRight.getPulses());
lcd.locate(0, 10);
lcd.printf("L: %.2f RPM R: %.2f RPM", leftRPM, rightRPM);
lcd.locate(0, 20);
lcd.printf("veL(m/s): %.2f veR(m/s): %.2f ", VelocityL, VelocityR);
lcd.locate(55, 0);
lcd.printf("PL: %.2f PR: %.2f", potL.read(), potR.read());
} else if (currentDisplayMode == SENSOR_DATA) {
lcd.locate(0, 0);
lcd.printf("S1: %.2f S2: %.2f S3: %.2f", sensors.readSensor1(), sensors.readSensor2(), sensors.readSensor3());
lcd.locate(0, 10);
lcd.printf("S4: %.2f", sensors.readSensor4());
}
}
};

// Joystick class
class Joystick1 {
private:
InterruptIn fire;
InterruptIn down;
public:
Joystick1(PinName f, PinName d) : fire(f), down(d) {}
void attachFireInterrupt(Callback<void()> func) { fire.rise(func); }
void attachDownInterrupt(Callback<void()> func) { down.rise(func); }
};

// Global Joystick object
Joystick1 Joystick1(D4, A3);

// PID Control Function
float pidControl(float setpoint, float currentValue) {
float error = setpoint - currentValue;
integral += error;
float derivative = error - previousError;
previousError = error;
return Kp * error + Ki * integral + Kd * derivative;
}

// Reset PID Variables
void resetPID() {
previousError = 0;
integral = 0;
}

// Function to update LCD (called by Ticker)
void updateLCDWrapper() {
static Motor motor(PB_12, PA_15, PC_6, PC_8, PB_7, PA_14, PA_13, true); // Create Motor object
motor.updateLCD(); // Call updateLCD function
}

// Function to update LCD (called by Ticker)
void calculateLRPMWrapper() {
static Motor motor(PB_12, PA_15, PC_6, PC_8, PB_7, PA_14, PA_13, true); // Create Motor object
motor.calculateRPM(); // Call updateLCD function
}

int main() {
// Initialize motors and attach joystick interrupts
Joystick1.attachFireInterrupt(&fireButtonPressed);
Joystick1.attachDownInterrupt(&downButtonPressed);
// Create Motor objects
// Motor(PinName enablePin, PinName LpwmPin, PinName RpwmPin, PinName dirLPin, PinName dirRPin, PinName bipolarLPin, PinName bipolarRPin, bool inverted = false)
// : enable(enablePin), Lpwm(LpwmPin), Rpwm(RpwmPin), dirR(dirRPin), dirL(dirLPin), bipolarL(bipolarLPin), bipolarR(bipolarRPin), inverted(inverted) {
Motor motor(PB_12, PA_15, PC_6, PC_8, PB_7, PA_14, PA_13, true); // motor (inverted PWM)

DigitalOut tcrtOut1(PB_2);
DigitalOut tcrtOut2(PB_1);
DigitalOut tcrtOut3(PB_15);
//DigitalOut tcrtOut4(PB_2);

tcrtOut1.write(1);

// Initialize LCD
lcd.cls();
lcd.locate(0, 0);
lcd.printf("Initializing...");

// Run RPM calculation every 1 second
rpm_ticker.attach(&calculateLRPMWrapper, 1.0);

// Refresh LCD every 500ms (adjust as needed)
display_update.attach(&updateLCDWrapper, 0.5);

while (1) {
if (autoMode) {

motor.setEnable(1);
// Manual control using potentiometers
float dutyLeft = potL.read(); // Read left potentiometer
float dutyRight = potR.read(); // Read right potentiometer
motor.setSpeedL(dutyLeft); // Set left motor speed
motor.setSpeedR(dutyRight); // Set right motor speed
} else {
}

wait_ms(10); // Small delay to reduce CPU usage
}
}
