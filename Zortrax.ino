#include <usbhid.h>
#include <hiduniversal.h>
#include <Servo.h>
#include "hidjoystickrptparser.h"

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

#define motor0_En 38
#define motor0_Step A0
#define motor0_Dir A1
#define motor1_En A2
#define motor1_Step A6
#define motor1_Dir A7
#define motor2_En A8
#define motor2_Step 46
#define motor2_Dir 48
#define endstop0 3
#define endstop1 2
#define endstop2 14
#define servoPin 11

//operation of USB device
USB usb;
HIDUniversal Hid(&usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

byte pinEn[] = {motor0_En, motor1_En, motor2_En};
byte pinStep[] = {motor0_Step, motor1_Step, motor2_Step};
byte pinDir[] = {motor0_Dir, motor1_Dir, motor2_Dir};
byte pinEndstops[] = {endstop0, endstop1, endstop2};
int positionOfMotors[3] = {0, 0, 0};
uint8_t controlValueOfMotorSpeed[4];
const int maxPositionOfMotor[3] = {1750, 640, 480};
const bool directionOfMotorPlus[3] = {HIGH, LOW, LOW};
bool calibrationFlag = false;
bool servoFlag = false;
bool servoNoRepeatFlag = false;
int breakValue=3000;
Servo servo;

void controlOfSpeed()
{
  for(uint8_t i=0; i<3; i++){
     if((statementOfController[i]<10&&positionOfMotors[i]<maxPositionOfMotor[i])||(statementOfController[i]>240&&positionOfMotors[i]>0)){
     controlValueOfMotorSpeed[i]=1;      
     }
  else controlValueOfMotorSpeed[i]=0;
  }
  controlValueOfMotorSpeed[3]=controlValueOfMotorSpeed[0]+controlValueOfMotorSpeed[1]+controlValueOfMotorSpeed[2];
  switch (controlValueOfMotorSpeed[3])
  {
    case 3:  breakValue=2000;
    break;

    case 2:  breakValue=3000;
    break;

    default: breakValue=6000;
    break;
  }
}

void calibration()
{  
  if(statementOfController[3]==72&&digitalRead(calibrationFlag)){
    breakValue=8000;
    for(uint8_t i=0; i<3; i++){
      while(!digitalRead(pinEndstops[i])) functionStep(i);
      positionOfMotors[i]=0;
    }
    calibrationFlag=true;
  }    
}

void functionStep(uint8_t numberOfMotor)
{
digitalWrite(pinStep[numberOfMotor], HIGH);
  delayMicroseconds(breakValue); 
digitalWrite(pinStep[numberOfMotor], LOW);
    delayMicroseconds(breakValue);                           
}

void functionCheckValuesOfSticks(){
  for(uint8_t i=0; i<3; i++){
    if(statementOfController[i]<10&&positionOfMotors[i]<maxPositionOfMotor[i]){
      digitalWrite(pinDir[i], directionOfMotorPlus[i]);
      functionStep(i);
      positionOfMotors[i]++;
    }
    else if(statementOfController[i]>240&&positionOfMotors[i]>0){
      digitalWrite(pinDir[i], ~directionOfMotorPlus[i]);
      functionStep(i);
      positionOfMotors[i]--;
    }
  }     
}

void functionServo(){ 
  if(statementOfController[3]==40){  
    if (!servoNoRepeatFlag){
      servoNoRepeatFlag=true;
      if(!servoFlag){
        servo.write(45);
        delay(1000);
        servoFlag=true;
      }
      else{
        servo.write(90);
        delay(1000);
        servoFlag=false;
      }
    }
  }
  else servoNoRepeatFlag=false;
}

void inverseKinematics(int Px, int Py, int Pz)
{
  double positionInRadians[3];
  const double pi=3.1416, l1=180, l2=155, l3=215; 
  const double stepMultiplier[3]={0.00359, 0.00491, 0.00714};
  int positionInSteps[3];
  if(Px>0&&Py>=0) positionInRadians[0] = atan(Py/Px);
  if(Px<=0&&Py>0) positionInRadians[0] = atan(-Px/Py)+pi/2;
  if(Px<0&&Py<=0) positionInRadians[0] = atan(Py/Px)+pi;
  if(Px>=0&&Py<0) positionInRadians[0] = atan(-Px/Py)+3*pi/2;
  positionInRadians[2] = acos((square(Px)+square(Py)+square(Pz-l1)-square(l2)-square(l3))/(2*l2*l3));
  positionInRadians[1] = pi-atan((Pz-l1)/sqrt(square(Px)+square(Py)))-atan((l3*sin(positionInRadians[2]))/(l2+l3*cos(positionInRadians[2])));
  for(uint8_t i=0; i<3; i++) positionInSteps[i] = positionInRadians[i]/stepMultiplier[i];
  functionDrive(positionInSteps[0], positionInSteps[1], positionInSteps[2]);
}

void functionDrive(int steps0, int steps1, int steps2){
  int positionInSteps[3]={steps0, steps1, steps2};
  int stepsToMake[3];
  int signOfMovementValue[3];
  for(uint8_t i=0; i<3; i++){
    stepsToMake[i] = positionInSteps[i]-positionOfMotors[i];
    if(stepsToMake[i]>0){
      digitalWrite(pinDir[i], directionOfMotorPlus[i]);
      signOfMovementValue[i]=1;
    }
    else{
      digitalWrite(pinDir[i], ~directionOfMotorPlus[i]);
      signOfMovementValue[i]=-1; 
    }
  }
    
  while(stepsToMake[0]>0||stepsToMake[1]>0||stepsToMake[2]>0){
    for(uint8_t i=0; i<3; i++){
      if(stepsToMake[i]>0){
        functionStep(i);
        positionOfMotors[i]=positionOfMotors[i]+signOfMovementValue[i];
        stepsToMake[0]--;
      }
    }
    usb.Task();
  }
}

void manualMode(){
controlOfSpeed();
functionCheckValuesOfSticks();
functionServo();
}

void autonomousMode(){
  functionDrive(875, 320, 220);
  inverseKinematics(160, 230, 40);
  inverseKinematics(160, 230, 200);
  inverseKinematics(-150, 220, 200);
  inverseKinematics(-150, 220, 40);
}

void setup() {
  if (!Hid.SetReportParser(0, &Joy)) ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
  for (uint8_t i=0; i<sizeof(pinEn); i++){
    pinMode(pinEn[i], OUTPUT);
    pinMode(pinStep[i], OUTPUT);
    pinMode(pinDir[i], OUTPUT);
    pinMode(pinEndstops[i], INPUT);
    digitalWrite(pinEn[i], LOW);
  }
  servo.attach(servoPin);
}

void loop(){
  usb.Task();
  calibration();
  if(!modeFlag&&calibrationFlag) manualMode();
  else if(calibrationFlag) autonomousMode();
}

