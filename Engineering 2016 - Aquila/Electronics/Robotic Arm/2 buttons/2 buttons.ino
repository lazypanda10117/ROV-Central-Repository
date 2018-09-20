#include <Servo.h>
int serialSpeed = 9600;

int pinLeft = 1;
int pinRight = 2;
int pinUp= 3;
int pinDown = 4;
int ledOrangePin = 5;
int ledBluePin = 6;
int btn1Pin = 11;
int btn2Pin = 12;
int yServoPin = 5;
int zServoPin = 6;
int gripServoPin = 13;
int gripMinAngle = 115;
int gripMaxAngle = 180;
int yMinAngle = 0;
int yMaxAngle = 180;
int zMinAngle = 0;
int zMaxAngle = 180;

int led1State = HIGH;
int led2State = HIGH;

int yPos;
int zPos;
int gripPos;

Servo yServo;
Servo zServo;
Servo gripServo;

void setup() {
  Serial.begin(serialSpeed);
  pinMode(pinLeft, INPUT);
  pinMode(pinRight, INPUT);
  pinMode(pinUp, INPUT);
  pinMode(pinDown, INPUT);
  pinMode(btn1Pin, INPUT);
  pinMode(btn2Pin, INPUT);
  pinMode(ledOrangePin, OUTPUT);
  pinMode(ledBluePin, OUTPUT);
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
  delay(25);
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
  if(ifValid() == 1){
    openArm();
    turnOnLight(ledOrangePin);
    Serial.println("opening arm");
  }else if(ifValid() == 2){
    closeArm();
    turnOnLight(ledBluePin);
    Serial.println("closing arm");
  }else if(ifValid() == 3){
    Serial.println("no gripper input");
  }else{
    turnOffLight(ledOrangePin);
    turnOffLight(ledBluePin);
    Serial.println("Grip Input Error");
  }
}

int ifValid(){
  if(digitalRead(btn1Pin) == LOW && digitalRead(btn2Pin) == HIGH){
    return 1;
  }else if(digitalRead(btn1Pin) == HIGH && digitalRead(btn2Pin) == LOW){
    return 2;
  }else if(digitalRead(btn1Pin) == HIGH && digitalRead(btn2Pin) == HIGH){
    return 3;
  }else{
    return 0;    
  }
}

void openArm(){
  if(gripPos>gripMinAngle && gripPos<gripMaxAngle){
    gripPos--; //depends on the direction of installation
    gripServo.write(gripPos);
    Serial.println("open" + gripPos);
  }
}

void closeArm(){
  if(gripPos>gripMinAngle && gripPos<gripMaxAngle){
    gripPos++; //depends on the direction of installation
    gripServo.write(gripPos);
    Serial.println("close" + gripPos);
  }
}

void turnOnLight(int ledPin){
  digitalWrite(ledPin, HIGH);
}

void turnOffLight(int ledPin){
  digitalWrite(ledPin, LOW);
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
