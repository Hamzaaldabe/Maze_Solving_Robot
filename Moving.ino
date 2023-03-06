#define in1 8 
#define in2 9 
#define in3 12 
#define in4 13
#define ENA 6
#define ENB 11
#define LeftMotorENCA 2
#define RightMotorENCA 3


volatile unsigned long count = 0;
unsigned long count_prev = 0;
float Theta, RPM, RPM_d;
float Theta_prev = 0;
int dt;
float RPM_max = 230;
#define pi 3.1416
float Vmax = 6;
float Vmin = -6;
float V = 0.1;
float e, e_prev = 0, inte, inte_prev = 0;
int LeftMotorENCA_DATA,LeftMotorENCB_DATA;

int baseSpeed = 2 ;

int RMS ;  //right motor speeed
int LMS ;  //left motor speed

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(LeftMotorENCA, INPUT_PULLUP);
pinMode(RightMotorENCA, INPUT_PULLUP);
//attachInterrupt(digitalPinToInterrupt(LeftMotorENCA), ISR_LeftMotor, CHANGE);
//attachInterrupt(digitalPinToInterrupt(RightMotorENCA), ISR_RightMotor, CHANGE);
for(int i =8;i<14;i++){
  pinMode(i,OUTPUT);
}



cli();
TCCR1A = 0;
TCCR1B = 0;
TCNT1 = 0;
OCR1A = 12499; //Prescaler = 64
TCCR1B |= (1 << WGM12);
TCCR1B |= (1 << CS11 | 1 << CS10);
TIMSK1 |= (1 << OCIE1A);
sei();


}

void loop() {
  Forward();
  runMotors();
  delay(2000);
   analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(2000);
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
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);   
}

void runMotors() {
  RMS = map(baseSpeed, 0, 1023, 0 , 255);
  LMS = map(baseSpeed , 0, 1023, 0 , 255);
  analogWrite(ENA, RMS);
  analogWrite(ENB, LMS);
}
