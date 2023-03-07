#include <NewPing.h>
#include <PID_v1_bc.h>
#define in1 13 
#define in2 12 
#define in3 9 
#define in4 8
#define ENA 10
#define ENB 11
#define LeftMotorENCA 2
#define RightMotorENCA 3
#define TRIGGER_PINF  A4  
#define ECHO_PINF     A5  
#define MAX_DISTANCE 293
volatile unsigned long count = 0;
unsigned long count_prev = 0;



float fSensor,oldFrontSensor;
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE); //setup front ultrasonic
//speed
int baseSpeed = 50 ;

int RMS ;  //right motor speeed
int LMS ;  //left motor speed

volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
long prevT = 0;
int posPrev = 0;

volatile long Pulse_Per_Rev_Left=0;
volatile long Pulse_Per_Rev_Right=0;
unsigned long currentTime;
unsigned long prevTime=0;
unsigned long RPM_Right=0;
unsigned long RPM_Left=0;

//PID constants
const double pidKp = 1.0;
const double pidKi = 0.0;
const double pidKd = 0.0;

//pid vars
double LeftSetpoint=189;
double RightSetpoint=125;
double LeftInput = 0;
double RightInput = 0;
double LeftOutput = 0;
double RightOutput = 0;

//PID instance
PID LeftPID(&LeftInput,&LeftOutput,&LeftSetpoint,pidKp, pidKi, pidKd, DIRECT);
PID RightPID(&RightInput,&RightOutput,&RightSetpoint,pidKp, pidKi, pidKd, DIRECT);


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(LeftMotorENCA, INPUT_PULLUP);
pinMode(RightMotorENCA, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(LeftMotorENCA), ISR_LeftMotor, RISING);
attachInterrupt(digitalPinToInterrupt(RightMotorENCA), ISR_RightMotor, RISING);
for(int i =8;i<14;i++){
  pinMode(i,OUTPUT);
}


//PID Tuning
  LeftPID.SetMode(AUTOMATIC);
  LeftPID.SetSampleTime(10);
  LeftPID.SetOutputLimits(-255, 255);
  RightPID.SetMode(AUTOMATIC);
  RightPID.SetSampleTime(10);
  RightPID.SetOutputLimits(-255, 255);
  
  analogWrite(ENA, 125);
  analogWrite(ENB, 125);
//delay(5000);  
}

void loop() {
  //delay(2000);
  Serial.println("started");
  Serial.println("Left Speed: ");
  
  
  //runMotors();

  // Calculate motor speeds (in RPM)
  //double LeftSpeed = calculateSpeed(Pulse_Per_Rev_Left);
  //double RightSpeed = calculateSpeed(Pulse_Per_Rev_Left);
  // read the position and velocity
  

  
  Serial.println(velocity1);
  double LeftSpeed = calculateSpeed(velocity1);

  // Update PID inputs
  LeftInput = LeftSpeed;
  //RightInput = RightSpeed;
  
   // Compute PID outputs
  LeftPID.Compute();
  RightPID.Compute();

  // Set motor speeds
  setMotorSpeed(ENB, LeftOutput);
  setMotorSpeed(ENA, RightOutput);

  //delay(100000);

}
void Forward(){
  
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);  
}

void Left(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);  
}
void Right(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);  
}
void Stop(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,HIGH);   
}

void runMotors() {
  // Pulse_Per_Rev_Left=0;
  // Pulse_Per_Rev_Right=0;

  
  // ReadFront();
  // oldFrontSensor = fSensor;
  // Serial.print("Front: "); 
  // Serial.println(fSensor);
  // prevTime=millis();

  

  //analogWrite(ENA, 125);
  //analogWrite(ENB, 125);
  // while(oldFrontSensor - 18.2 < fSensor){
  //   ReadFront();
  //   Serial.print("Front: ");
  //   Serial.println(fSensor);
  // }
  
  // Stop();
  // analogWrite(ENA, 0);
  // analogWrite(ENB, 0);
  // currentTime=millis() - prevTime;
  //delay(2000);
}

void ISR_LeftMotor(){
  Pulse_Per_Rev_Left++;
}

void ISR_RightMotor(){
  Pulse_Per_Rev_Right++;
}

void ReadFront(){
  fSensor = sonarFront.ping()/100.0;
}


double calculateSpeed(int encoderValue)
{
    double speed = (double)encoderValue * 60 / 206;
    return speed;
}


void setMotorSpeed(int pwmPin, double speed)
{
    int pwmValue = (int)abs(speed);
    if (pwmValue > 255)
    {
        pwmValue = 255;
    }
    analogWrite(pwmPin, pwmValue);
    // if (speed < 0)
    // {
    //     digitalWrite(pwmPin + 1, HIGH);
    // }
    // else
    // {
    //     digitalWrite(pwmPin + 1, LOW);
    // }
}
