#include <PID_v1.h>

// Define constants
const int motor1EncoderPinA = 2;
const int motor1EncoderPinB = 3;
const int motor1PWMPin = 5;
const int motor2EncoderPinA = 4;
const int motor2EncoderPinB = 6;
const int motor2PWMPin = 9;
const int encoderResolution = 360; // Number of encoder ticks per revolution
const double pidKp = 1.0;
const double pidKi = 0.0;
const double pidKd = 0.0;

// Define variables
double motor1Setpoint = 0;
double motor2Setpoint = 0;
double motor1Input = 0;
double motor2Input = 0;
double motor1Output = 0;
double motor2Output = 0;

// Create PID objects
PID motor1PID(&motor1Input, &motor1Output, &motor1Setpoint, pidKp, pidKi, pidKd, DIRECT);
PID motor2PID(&motor2Input, &motor2Output, &motor2Setpoint, pidKp, pidKi, pidKd, DIRECT);

void setup()
{
    // Initialize encoder pins
    pinMode(motor1EncoderPinA, INPUT);
    pinMode(motor1EncoderPinB, INPUT);
    pinMode(motor2EncoderPinA, INPUT);
    pinMode(motor2EncoderPinB, INPUT);

    // Initialize PWM pins
    pinMode(motor1PWMPin, OUTPUT);
    pinMode(motor2PWMPin, OUTPUT);

    // Set PID tuning parameters
    motor1PID.SetMode(AUTOMATIC);
    motor1PID.SetSampleTime(10);
    motor1PID.SetOutputLimits(-255, 255);
    motor2PID.SetMode(AUTOMATIC);
    motor2PID.SetSampleTime(10);
    motor2PID.SetOutputLimits(-255, 255);
}

void loop()
{
    // Read encoder values
    int motor1EncoderValue = readEncoder(motor1EncoderPinA, motor1EncoderPinB);
    int motor2EncoderValue = readEncoder(motor2EncoderPinA, motor2EncoderPinB);

    // Calculate motor speeds (in RPM)
    double motor1Speed = calculateSpeed(motor1EncoderValue);
    double motor2Speed = calculateSpeed(motor2EncoderValue);

    // Update PID inputs
    motor1Input = motor1Speed;
    motor2Input = motor2Speed;

    // Compute PID outputs
    motor1PID.Compute();
    motor2PID.Compute();

    // Set motor speeds
    setMotorSpeed(motor1PWMPin, motor1Output);
    setMotorSpeed(motor2PWMPin, motor2Output);
}

// Function to read encoder value
int readEncoder(int pinA, int pinB)
{
    static int oldState = 0;
    static int encoderValue = 0;
    int newState = digitalRead(pinA) * 2 + digitalRead(pinB);
    if (newState != oldState)
    {
        if ((oldState == 0 && newState == 2) || (oldState == 2 && newState == 1) ||
            (oldState == 1 && newState == 3) || (oldState == 3 && newState == 0))
        {
            encoderValue++;
        }
        else
        {
            encoderValue--;
        }
        oldState = newState;
    }
    return encoderValue;
}

// Function to calculate motor speed (in RPM)
double calculateSpeed(int encoderValue)
{
    double speed = (double)encoderValue * 60 / encoderResolution;
    return speed;
}

// Function to set motor speed using PWM
void setMotorSpeed(int pwmPin, double speed)
{
    int pwmValue = (int)abs(speed);
    if (pwmValue > 255)
    {
        pwmValue = 255;
    }
    analogWrite(pwmPin, pwmValue);
    if (speed < 0)
    {
        digitalWrite(pwmPin + 1, HIGH);
    }
    else
    {
        digitalWrite(pwmPin + 1, LOW);
    }
}
