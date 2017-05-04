//Author: Jeffrey Kam
//Date: 10th April 2017, 3rd Janurary 2017
#include "SoftwareSerial.h"
//software serial port for mpu and main arduino communcation
#define softRX 3
#define softTX 4

#include "Wire.h"
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif
#include <PS3USB.h>
#include <usbhub.h>

SoftwareSerial vrSerial(softRX, softTX);
USB Usb;
PS3USB PS3(&Usb); // This will just create the instance
USBHub Hub1(&Usb); // Some dongles have a hub inside

//variable for pins
int vrSwitch = 2;

//variables for different modes
int controlMode = 0; //0 for ps3 control
int processMode = 0; //0 for initial waiting, 1 for setup, 2 for stop, 3 for play
int vrMode = 0; //0 = ps3 camera control, 1 = vr camera control
int startUpSteps = 1;

//arm angle threshold
int armMinT = 0;
int armMaxT = 180;

//arm current angle
int armAngX = (armMinT + armMaxT) / 2;
int armAngY = (armMinT + armMaxT) / 2;

//camera angle threshold
int leftCamT = 0;
int rightCamT = 180;
int upCamT = 106;
int downCamT = 180;

int vrCamX = 0;
int vrCamY = 0;

//camera current angle
int camAngX = (leftCamT + rightCamT) / 2; //90 mid
int camAngY = (upCamT + downCamT) / 2; //143 mid

//variables for limits of control and speed
int joystickThresholdMin = 35;
int joystickThresholdMax = 220;

int minSpeed = -240;
int maxSpeed = 240;
int stationarySpeed = 0;

//speed variables for motors
int lSpeed = 0;
int rSpeed = 0;
int speedZ1 = 0;
int speedZ2 = 0;
int speedZ3 = 0;
int speedZ4 = 0;
boolean zStationary = true;

//send to rov section
String compiledString = "";

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  //Serial.println(1);
  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
  if (Usb.Init() == -1) {
    while (1); //halt
  }
  vrSerial.begin(9600);
}

void loop() {
  Usb.Task();
  if (processMode == 0) {
    //wait for startup command
    if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
      int tA = PS3.getAnalogButton(TRIANGLE);
      if (tA != 0) {
        processMode = 1;
      }
    }
    delay(50);
  } else if (processMode == 1) {
    startUpProcess(startUpSteps);
  } else if (processMode == 2) {
    if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
      int sA = PS3.getAnalogButton(SQUARE);
      if (sA >= 100) {
        processMode = 3;
      }
    }
    delay(100);
  } else if (processMode == 3) {
    if (controlMode == 0) {
      controlModePS3();
    }
  }
  //debugMotorOutput(lSpeed, rSpeed, speedZ1, speedZ2, speedZ3, speedZ4);
  sendROVStats();
  delay(120);
}

//---Section of Main Control Code---
void controlModePS3() {
  Usb.Task();
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    //left right motor control
    int leftValue = PS3.getAnalogHat(LeftHatY);
    int rightValue = PS3.getAnalogHat(RightHatY);
    //up down motor control
    int L2Val = PS3.getAnalogButton(L2); //up
    int R2Val = PS3.getAnalogButton(R2); //down
    //camera movement control
    int leftVal = PS3.getAnalogButton(LEFT);
    int rightVal = PS3.getAnalogButton(RIGHT);
    int upVal = PS3.getAnalogButton(UP);
    int downVal = PS3.getAnalogButton(DOWN);

    //arm servo control
    int L1Val = PS3.getAnalogButton(L1); //open
    int R1Val = PS3.getAnalogButton(R1); //close
    boolean startVal = PS3.getButtonClick(START); //rotate left
    Serial.println(startVal);
    boolean selectVal = PS3.getButtonClick(SELECT); //rotate right
    //process mode input
    int triangleVal = PS3.getAnalogButton(TRIANGLE); //emergency stop
    int squareVal = PS3.getAnalogButton(SQUARE); //start startup
    lSpeed = planarSpeed(leftValue);
    rSpeed = planarSpeed(rightValue);
    planarMovement(lSpeed, rSpeed);
    zMovement();
    checkVRMode();
    cameraMovement(leftVal, rightVal,upVal, downVal);
    armControlX(R1Val, L1Val);
    armControlY(startVal, selectVal);
    PS3CheckMode(triangleVal, squareVal);
  }
}

//---Section of Mode Changing Code---
void PS3CheckMode(int triVal, int squVal) {
  if (squVal >= 100) {
    processMode = 2;
  } else if (triVal >= 100) {
    processMode = 1;
  }
}
//---Section of Process 1 (Start Up Process)---
void startUpProcess(int steps) {
  if (steps == 1) {
    //debugMotorOutput(maxSpeed, maxSpeed, maxSpeed, maxSpeed, maxSpeed, maxSpeed);
    lSpeed = maxSpeed;
    rSpeed = maxSpeed;
    speedZ1 = maxSpeed;
    speedZ2 = maxSpeed;
    speedZ3 = maxSpeed;
    speedZ4 = maxSpeed;    
    startUpSteps++;
    delay(3000);
  } else if (steps == 2) {
    //debugMotorOutput(stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed);
    lSpeed = stationarySpeed;
    rSpeed = stationarySpeed;
    speedZ1 = stationarySpeed;
    speedZ2 = stationarySpeed;
    speedZ3 = stationarySpeed;
    speedZ4 = stationarySpeed;
    startUpSteps++;
    delay(2000);
  } else if (steps == 3) {
    //debugMotorOutput(minSpeed,minSpeed,minSpeed, minSpeed, minSpeed, minSpeed);
    lSpeed = minSpeed;
    rSpeed = minSpeed;
    speedZ1 = minSpeed;
    speedZ2 = minSpeed;
    speedZ3 = minSpeed;
    speedZ4 = minSpeed;
    startUpSteps++;
    delay(3000);
  } else if (steps == 4) {
    //debugMotorOutput(stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed);
    lSpeed = stationarySpeed;
    rSpeed = stationarySpeed;
    speedZ1 = stationarySpeed;
    speedZ2 = stationarySpeed;
    speedZ3 = stationarySpeed;
    speedZ4 = stationarySpeed;
    processMode = 3;
    startUpSteps = 1;
    delay(500);
  }
}

//---Section of Process 2 (STOP Process)---
void stopProcess() {
  processMode = 2;
  lSpeed = stationarySpeed;
  rSpeed = stationarySpeed;
  speedZ1 = stationarySpeed;
  speedZ2 = stationarySpeed;
  speedZ3 = stationarySpeed;
  speedZ4 = stationarySpeed;
}

void resumeProcess() {
  processMode = 3;
}

//---Section of Planar Movement---
int planarSpeed(int joystickVal) {
  return map(constrain(joystickVal, joystickThresholdMin, joystickThresholdMax), joystickThresholdMax, joystickThresholdMin, minSpeed, maxSpeed);
} 

void planarMovement(int m1, int m2) {
  lSpeed = m1;
  rSpeed = m2;
}

//---Section of Vertical Movement---
void zMovement() {
  int mag = getMagnitudeZ();
  //k is the respective values for up and down magnitude
  speedZ1 = mag;
  speedZ2 = mag;
  speedZ3 = mag;
  speedZ4 = mag;
}

int getMagnitudeZ() {
  int mag = 0;
  if (PS3.getAnalogButton(L2)) {
    mag = map(PS3.getAnalogButton(L2), 0, 255, stationarySpeed, minSpeed);
  } else if (PS3.getAnalogButton(R2)) {
    mag = map(PS3.getAnalogButton(R2), 0, 255, stationarySpeed, maxSpeed);
  }
  if (mag == stationarySpeed) {
    zStationary = true;
  } else {
    zStationary = false;
  }
  return mag;
}

//---Section of Camera Control---
void checkVRMode(){
  if(digitalRead(vrSwitch) == LOW){
    if(vrMode == 1){
      camAngX = (leftCamT + rightCamT) / 2; //90 mid
      camAngY = (upCamT + downCamT) / 2; //143 mid
    }
    vrMode = 0;
  }else{
    vrMode = 1;
  }
}

void retrieveGyroData(){
  String tempS = "";
  String a1 = "";
  String a2 = "";
  char buf1[4];
  char buf2[4];
  char c;
  boolean t = false;
  while(vrSerial.available()){
    if(c != '?'){
      c = vrSerial.read();
      tempS += c;
    }else{
      if(tempS.length()==6){
        a1 = tempS.substring(0,3);
        a2 = tempS.substring(3,6);
        a1.toCharArray(buf1, 4);
        vrCamX = atoi(buf1);
        vrCamX -= 100;
        a2.toCharArray(buf2, 4);
        vrCamY = atoi(buf2);
        vrCamY -= 100;
        break;
      }else{
        break;
      }
    }
  }
}

void cameraMovement(int lV, int rV, int uV, int dV){
  if(vrMode == 0){
    cameraControlX(lV, rV);
    cameraControlY(uV, dV);
  }else{
    //steven's code
    camAngX = 0;//some vr mapped value
    camAngY = 0;//some vr mapped value
  }
}

void cameraControlX(int lV, int rV) {
  if (lV >= 50) {
    if (inRange(camAngX, leftCamT, rightCamT)) {
      camAngX += 1;
    }
  } else if (rV >= 50) {
    if (inRange(camAngX, leftCamT, rightCamT)) {
      camAngX -= 1;
    }
  }
}

void cameraControlY(int uV, int dV) {
  if (uV >= 50) {
    if (inRange(camAngY, downCamT, upCamT)) {
      camAngY += 1;
    }
  }else if (dV >= 50) {
    if (inRange(camAngY, downCamT, upCamT)) {
      camAngY -= 1;
    }
  }
}

//---Section of Arm Control---
void armControlX(int openVal, int closeVal) {
  if (openVal >= 50) {
    if (inRange(armAngX, armMinT, armMaxT)) {
      armAngX += 1;
    }
  } else if (closeVal >= 50) {
    if (inRange(armAngX, armMinT, armMaxT)) {
      armAngX -= 1;
    }
  }
}

void armControlY(boolean openVal, boolean closeVal) {
  if (openVal) {
    armAngY += 1;
  } else if (closeVal) {
    armAngY -= 1;
  }
}
//---Section of Miscellaneous Functions---
bool inRange(double val, int minimum, int maximum) {
  return ((minimum < val) && (val < maximum));
}

int transformSpeedTo100(int val) {
  if(val == 0){
    return 0;
  }
  if(inRange(map(val, minSpeed, maxSpeed, -100, 100),-10,10)){
    return 0;
  }
  return map(val, minSpeed, maxSpeed, -100, 100);
}

//---Section of Sending ROV Status To Processing---
void sendROVStats() {
  //send current status to Processing
  //compile stats
  compiledString = "";
  compiledString += String(processMode);
  compiledString += ",";
  //arm angle
  compiledString += String(armAngX);
  compiledString += ",";
  compiledString += String(armAngY);
  compiledString += ",";
  //camera angle
  compiledString += String(camAngX);
  compiledString += ",";
  compiledString += String(camAngY);
  compiledString += ",";
  //speed stats
  compiledString += String(lSpeed);
  compiledString += ",";
  compiledString += String(rSpeed);
  compiledString += ",";
  compiledString += String(speedZ1);
  compiledString += ",!";

  Serial.println(compiledString);
  char charA[compiledString.length()];
  
  for(int i=0; i<compiledString.length(); i++){
    charA[i] = compiledString.charAt(i);
  }
  //Serial.println(compiledString.length());
  Wire.beginTransmission(9);
  Wire.write(charA,compiledString.length());
  Wire.endTransmission();    // stop transmitting
}

void debugMotorOutput(int spdRaw1, int spdRaw2, int spdRaw3, int spdRaw4, int spdRaw5, int spdRaw6) {
  //state 1 is all, 2 is vertical, 3 is planar
  String spd1 = String(transformSpeedTo100(spdRaw1));
  String spd2 = String(transformSpeedTo100(spdRaw2));
  String spd3 = String(transformSpeedTo100(spdRaw3));
  String spd4 = String(transformSpeedTo100(spdRaw4));
  String spd5 = String(transformSpeedTo100(spdRaw5));
  String spd6 = String(transformSpeedTo100(spdRaw6));
  Serial.println("Left Planar Motor : " + spd1);
  Serial.println("Right Planar Motor : " + spd2);
  Serial.println("Vertical Motor 1: " + spd3);
  Serial.println("Vertical Motor 2: " + spd4);
  Serial.println("Vertical Motor 3: " + spd5);
  Serial.println("Vertical Motor 4: " + spd6);
}
