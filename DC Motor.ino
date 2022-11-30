#include <movingAvg.h>
#include "CommandHandler.h"
#include "MegunoLink.h"
#include <util/atomic.h>

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 6
#define IN2 7

// globals
long prevT = 0;
int posPrev = 0;

// Use the "volatile" directive for variables
// used in an interrupt

volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

// command handler for GUI
CommandHandler<> SerialCommandHandler;

// global variables 
int Speed = 100;
int P = 3;
int I = 6;
int D = 0;

movingAvg mySensor(250);    // use 250 data points to smooth output

float eprev = 0;
float eintegral = 0;

// Instantiate Plot variable
   TimePlot PID_Plot;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);

  // Average senor command
  mySensor.begin();

  // Setup the serial commands we can respond to
  SerialCommandHandler.AddCommand(F("P"), changeKp);
  SerialCommandHandler.AddCommand(F("I"), changeKi);  
  SerialCommandHandler.AddCommand(F("D"), changeKd); 
  SerialCommandHandler.AddCommand(F("Speed"), changeRPM); 


  // Setup the serial PID plot
  
  PID_Plot.SetTitle("Desired vs Actual");
  PID_Plot.SetXlabel("Time");
  PID_Plot.SetYlabel("Value");
}

void loop() {

  // Check for the serial commands and dispatch them
  SerialCommandHandler.Process();

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  // changed from 600 to 300
  float v1 = velocity1/185.0*60.0;
  float v2 = velocity2/185.0*60.0;

  //float v1 = velocity1/1200.0*60.0;
  //float v2 = velocity2/1200.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;
  
  // Set a target
  //float vt = 125;

  // Compute the control signal u
  //float kp = 3;
  //float ki = 6;
  //float kd = 1;
  
  //float e = vt-v1Filt;

  float e = Speed-v1Filt;
  
  // integral
  eintegral = eintegral + e*deltaT;

  // derivative
  float dedt = (e-eprev)/(deltaT);
  
  //float u = kp*e + kd*dedt + ki*eintegral;
  float u = P*e + D*dedt + I*eintegral;

  int OutputData = v1Filt;
  int OutputMovingAverage = mySensor.reading(OutputData);

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  
  setMotor(dir,pwr,PWM,IN1,IN2);
  
  // store previous error
  
  eprev = e;
  
  //Serial.print(Speed);
  //Serial.print(Speed);
  //Serial.print(" ");
  //Serial.print(OutputMovingAverage);
  //Serial.println();
  //delay(1);

  PID_Plot.SendData("Desired Speed", Speed);
  PID_Plot.SendData("Actual Speed", OutputMovingAverage);

  delay(10);
  
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  //Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}
 
void changeKp(CommandParameter& Parameters){
  P = Parameters.NextParameterAsInteger();
}

void changeKi(CommandParameter& Parameters){
  I = Parameters.NextParameterAsInteger();
}

void changeKd(CommandParameter& Parameters){
  D = Parameters.NextParameterAsInteger();
}
void changeRPM(CommandParameter& Parameters){
  Speed = Parameters.NextParameterAsInteger();
}