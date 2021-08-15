//Author: Jeffrey Kam
//Date: 10th April 2017
#include "Wire.h"
#include "Servo.h"
#include "SoftwareSerial.h"
#include "PID_v1.h"

Servo cameraServoX,cameraServoY,armServoX,armServoY;
int cSA[11]; //processmode, camX, camY, armX, armY, leftSpd, rightSpd, z1Spd, z2Spd, z3Spd, z4Spd

//motor num: 0 = left horizontal, 1 = right horizontal, 2 = top left, 3 = top right, 4 = bottom left, 5 = bottom right 
int motorSpeedPins[6] = {7,6,5,4,3,2};
int motorDPins[6] = {27,31,35,39,43,47};
int cameraServos[2] = {11,10};
int armServos[2] = {13,12};

int prevMotorSpeed[6] = {0,0,0,0,0,0};

void setup() {
  Wire.begin(9); 
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
  cameraServoX.attach(cameraServos[0]);
  cameraServoY.attach(cameraServos[1]);
  armServoX.attach(armServos[0]);
  armServoY.attach(armServos[1]);
}

void loop() {

}

void receiveEvent(int bytes) {
  String compiledString = "";
  char c;
  c = Wire.read();
  while(Wire.available()){
    if(c == "!"){
      break;
    }else{
      compiledString += c;
      c = Wire.read();    // read one character from the I2C
    }
  }
  parseData(compiledString);
  debug();
  coreAlgorithm();
}

void parseData(String s){
  String tempS[8];
  int k = 0;
  for(int i=0; i<8;i++){
    while(s.charAt(k) != ','){
      tempS[i] += s.charAt(k);
      k++;
    }
    k++;
  }
  for(int j=0; j<8; j++){
    char buf[4];
    tempS[j].toCharArray(buf, 4);
    if(j == 7){
      int tempI = atoi(buf);
      cSA[j] = tempI;
      for(int k=j+1; k<=j+3; k++){
        cSA[k] = tempI;
      }
    }else{
      cSA[j] = atoi(buf);
    }
  }
}

void coreAlgorithm(){
  planarMovement();
  verticalMovement();
  cameraMovement();
  armMovement();
}

void planarMovement() {
  runMotor(0,cSA[5]);
  runMotor(1,cSA[6]);
}

void verticalMovement(){
  if(checkZStationary()){  
    runMotor(2,cSA[7]);
    runMotor(3,cSA[8]);
    runMotor(4,cSA[9]);
    runMotor(5,cSA[10]); 
  }else{
    runMotor(2,cSA[7]);
    runMotor(3,cSA[8]);
    runMotor(4,cSA[9]);
    runMotor(5,cSA[10]);  
  }
}

boolean checkZStationary(){
  if(cSA[6] == 0){
    return true;
  }
  return false;
}

void cameraMovement(){
  cameraServoX.write(cSA[1]);
  cameraServoY.write(cSA[2]);
}

void armMovement(){
  armServoX.write(cSA[3]);
  armServoY.write(cSA[4]);
}

void directionCheck(int motorNum, int motorSpeed){
  int tempSpeed = motorSpeed;
  if(numberProperty(prevMotorSpeed[motorNum]) == 0){
  }else if(numberProperty(prevMotorSpeed[motorNum]) == 1){
    if(numberProperty(motorSpeed) == 2){
      tempSpeed = 0;
    }
  }else if(numberProperty(prevMotorSpeed[motorNum]) == 2){
    if(numberProperty(motorSpeed) == 1){
      tempSpeed = 0;
    }
  }
  prevMotorSpeed[motorNum] = tempSpeed;
  runMotor(motorNum, tempSpeed);
}

void runMotor(int motorNum, int motorSpeed){
  if(motorNum == 0 || motorNum == 1){
    if(motorSpeed >= 20 && motorSpeed <= 255){
      digitalWrite(motorDPins[motorNum], HIGH);
      digitalWrite(motorDPins[motorNum]+2, LOW);
      analogWrite(motorSpeedPins[motorNum], mappedSpeed(motorSpeed));
    }else if(motorSpeed <= 20 && motorSpeed >= -255){
      digitalWrite(motorDPins[motorNum], LOW);
      digitalWrite(motorDPins[motorNum]+2, HIGH);
      analogWrite(motorSpeedPins[motorNum], mappedSpeed(motorSpeed));
    }else{
      digitalWrite(motorDPins[motorNum], LOW);
      digitalWrite(motorDPins[motorNum]+2, LOW);
      analogWrite(motorSpeedPins[motorNum], 0);
    }
  }else{
    if(motorSpeed >= 20 && motorSpeed <= 255){
      digitalWrite(motorDPins[motorNum], HIGH);
      analogWrite(motorSpeedPins[motorNum], mappedSpeed(motorSpeed));
    }else if(motorSpeed <= 20 && motorSpeed >= -255){
      digitalWrite(motorDPins[motorNum], LOW);
      analogWrite(motorSpeedPins[motorNum], mappedSpeed(motorSpeed));
    }else{
      digitalWrite(motorDPins[motorNum], LOW);
      analogWrite(motorSpeedPins[motorNum], 0);
    }
  }
}

int mappedSpeed(int motorSpeed){
  return map(abs(motorSpeed),0,255,255,0);
}

int numberProperty(int n){
  if(n == 0){
    return 0;
  }else if(n > 0){
    return 1;
  }else{
    return 2;
  }
}

void debug(){
  for(int i=0; i<11; i++){
    Serial.print(cSA[i]);
    Serial.print(",");
  }
  Serial.print("\n");
}

