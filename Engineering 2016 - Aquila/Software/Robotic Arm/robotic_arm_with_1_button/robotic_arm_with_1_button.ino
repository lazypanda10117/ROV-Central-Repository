#include <Servo.h>
int pinLeft = 1;
int pinRight = 2;
int pinUp= 3;
int pinDown = 4;
int redBtnPin = 11;
int yServoPin = 5;
int zServoPin = 6;
int gripServoPin = 13;
int gripMinAngle = 115;
int gripMaxAngle = 180;
int yMinAngle = 0;
int yMaxAngle = 180;
int zMinAngle = 0;
int zMaxAngle = 180;

boolean buttonState =  true; //true == open and false == close

int yPos;
int zPos;
int gripPos;

Servo yServo;
Servo zServo;
Servo gripServo;

void setup() {
  pinMode(pinLeft, INPUT);
  pinMode(pinRight, INPUT);
  pinMode(pinUp, INPUT);
  pinMode(pinDown, INPUT);
  pinMode(redBtnPin, INPUT);
  yServo.attach(yServoPin);
  zServo.attach(zServoPin);  
  gripServo.attach(gripServoPin);
  yPos = yServo.read();
  zPos = zServo.read();
  gripPos = gripServo.read();
}

void loop() {
  gripMovement();
  armMovement();
  delay(40);
}

void armMovement(){
  if(digitalRead(pinLeft) == HIGH){
    rotateLeft();
  }else if(digitalRead(pinRight) == HIGH){
    rotateRight();
  }else if(digitalRead(pinUp) == HIGH){
    up();
  }else if(digitalRead(pinDown) == HIGH){
    down();
  }else{
    Serial.println("No Arm Input");
  }
}

void gripMovement(){
  changeState(buttonState);
  if(buttonState == true){
    openArm();
  }else{
    closeArm();
  }
}

void changeState(boolean btnState){
  if(btnState){
    btnState = false;
  }else{
    btnState = true;
  }
}

void openArm(){
  if(gripPos>gripMinAngle && gripPos<gripMaxAngle){
    gripPos--; //depends on the direction of installation
    gripServo.write(gripPos);
  }
}

void closeArm(){
  if(gripPos>gripMinAngle && gripPos<gripMaxAngle){
    gripPos++; //depends on the direction of installation
    gripServo.write(gripPos);
  }
}

void up(){
  if(yPos>yMinAngle && yPos<yMaxAngle){
    yPos++; //depends on the direction of installation
    yServo.write(yPos);
  }
}

void down(){
  if(yPos>yMinAngle && yPos<yMaxAngle){
    yPos--; //depends on the direction of installation
    yServo.write(yPos);
  }
}

void rotateLeft(){
  if(zPos>zMinAngle && zPos<zMaxAngle){
    zPos++; //depends on the direction of installation
    zServo.write(zPos);
  }
}

void rotateRight(){
  if(zPos>zMinAngle && zPos<zMaxAngle){
    zPos--; //depends on the direction of installation
    zServo.write(zPos);
  }
}
