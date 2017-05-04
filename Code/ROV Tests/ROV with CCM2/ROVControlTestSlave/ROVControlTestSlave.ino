//Author: Jeffrey Kam
//Date: 10th April 2017
#include "Wire.h"
#include "Servo.h"
#include "SoftwareSerial.h"

#define softTX 3
#define softRX 4

Servo cameraServoX,cameraServoY,armServoX,armServoY;
SoftwareSerial vrSerial(softTX, softRX);

int cSA[11]; //processmode, camX, camY, armX, armY, leftSpd, rightSpd, z1Spd, z2Spd, z3Spd, z4Spd

//motor num: 0 = left horizontal, 1 = right horizontal, 2 = top left, 3 = top right, 4 = bottom left, 5 = bottom right 
int motorFPins[6] = {1,2,3,4,5,6};
int motorBPins[6] = {1,2,3,4,5,6};
int motorSpeedPins[6] = {1,2,3,4,5,6};
int cameraServos[2] = {1,2};
int armServos[2] = {1,2};

int previousG[2] = {0,0};
int currentG[2] = {0,0};
int desiredG[2] = {0,0};
int futureG[2] = {0,0};
int errorG[2] = {0,0};

void setup() {
  Wire.begin(9); 
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
  vrSerial.begin(9600);
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
    autoBalancing();
  }else{
    runMotor(2,cSA[7]);
    runMotor(3,cSA[8]);
    runMotor(4,cSA[9]);
    runMotor(5,cSA[10]);  
  }
}

void autoBalancing(){
  //PID control here
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

void runMotor(int motorNum, int motorSpeed){
  if(motorSpeed > 20 && motorSpeed < 255){
    digitalWrite(motorFPins[motorNum], HIGH);
    digitalWrite(motorBPins[motorNum], LOW);
    analogWrite(motorSpeedPins[motorNum], mappedSpeed(motorSpeed));
  }else if(motorSpeed < 20 && motorSpeed > -255){
    digitalWrite(motorFPins[motorNum], LOW);
    digitalWrite(motorBPins[motorNum], HIGH);
    analogWrite(motorSpeedPins[motorNum], mappedSpeed(motorSpeed));
  }else{
    digitalWrite(motorFPins[motorNum], LOW);
    digitalWrite(motorBPins[motorNum], LOW);
    analogWrite(motorSpeedPins[motorNum], mappedSpeed(motorSpeed));
  }
}

int mappedSpeed(int motorSpeed){
  return map(abs(motorSpeed),0,255,255,0);
}

void debug(){

  for(int i=0; i<11; i++){
    Serial.print(cSA[i]);
    Serial.print(",");
  }
  Serial.print("\n");
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
      if(tempS.length==6){
        a1 = tempS.substring(0,3);
        a2 = tempS.substring(3,6);
        a1.toCharArray(buf1, 4);
        vrCamX = atof(buf1);
        vrCamX -= 100;
        a2.toCharArray(buf2, 4);
        vrCamY = atof(buf2);
        vrCamY -= 100;
        break;
      }else{
        break;
      }
    }
  }
}
