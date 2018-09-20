//Author: Jeffrey Kam
//Date: 10th April 2017
#include "Wire.h"
#include "Servo.h"
#include "SoftwareSerial.h"
#include "PID_v1.h"

#define softTX 8
#define softRX 9

boolean autoBalancingSet = false;
double kp = 1;
double ki = 0.2;
double kd = 0.6;
double o1,o2,o3,o4,o5,o6,o7,o8;
int oVector1,oVector2,oVector3,oVector4;
//double previousG[2] = {0,0};
double currentG[2] = {0,0};
double desiredG[2] = {0,0};
//double futureG[2] = {0,0};
//double errorG[2] = {0,0};

Servo cameraServoX,cameraServoY,armServoX,armServoY;
SoftwareSerial vrSerial(softTX, softRX);

PID autoPIDYTL(&currentG[0], &o1, &desiredG[0],kp,ki,kd, DIRECT);
PID autoPIDYTR(&currentG[0], &o2, &desiredG[0],kp,ki,kd, DIRECT);
PID autoPIDYBL(&currentG[0], &o3, &desiredG[0],kp,ki,kd, DIRECT);
PID autoPIDYBR(&currentG[0], &o4, &desiredG[0],kp,ki,kd, DIRECT);
PID autoPIDRTL(&currentG[1], &o5, &desiredG[1],kp,ki,kd, DIRECT);
PID autoPIDRTR(&currentG[1], &o6, &desiredG[1],kp,ki,kd, DIRECT);
PID autoPIDRBL(&currentG[1], &o7, &desiredG[1],kp,ki,kd, DIRECT);
PID autoPIDRBR(&currentG[1], &o8, &desiredG[1],kp,ki,kd, DIRECT);

int cSA[11]; //processmode, camX, camY, armX, armY, leftSpd, rightSpd, z1Spd, z2Spd, z3Spd, z4Spd

//motor num: 0 = left horizontal, 1 = right horizontal, 2 = top left, 3 = top right, 4 = bottom left, 5 = bottom right 
int motorDPins[6] = {7,6,5,4,3,2};
int motorSpeedPins[6] = {27,31,35,39,43,47};
int cameraServos[2] = {11,10};
int armServos[2] = {13,12};

void setup() {
  Wire.begin(9); 
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
  vrSerial.begin(9600);
  cameraServoX.attach(cameraServos[0]);
  cameraServoY.attach(cameraServos[1]);
  armServoX.attach(armServos[0]);
  armServoY.attach(armServos[1]);
  autoPIDYTL.SetMode(AUTOMATIC);
  autoPIDYTR.SetMode(AUTOMATIC);
  autoPIDYBL.SetMode(AUTOMATIC);
  autoPIDYBR.SetMode(AUTOMATIC);
  autoPIDRTL.SetMode(AUTOMATIC);
  autoPIDRTR.SetMode(AUTOMATIC);
  autoPIDRBL.SetMode(AUTOMATIC);
  autoPIDRBR.SetMode(AUTOMATIC);
  autoPIDYTL.SetOutputLimits(-255, 255);
  autoPIDYTR.SetOutputLimits(-255, 255);
  autoPIDYBL.SetOutputLimits(-255, 255);
  autoPIDYBR.SetOutputLimits(-255, 255);
  autoPIDRTL.SetOutputLimits(-255, 255);
  autoPIDRTR.SetOutputLimits(-255, 255);
  autoPIDRBL.SetOutputLimits(-255, 255);
  autoPIDRBR.SetOutputLimits(-255, 255);
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
  autoPIDYTL.Compute();
  autoPIDYTR.Compute();
  autoPIDYBL.Compute();
  autoPIDYBR.Compute();
  autoPIDRTL.Compute();
  autoPIDRTR.Compute();
  autoPIDRBL.Compute();
  autoPIDRBR.Compute();
  oVector1 = o1+o5;
  oVector2 = o2+o6;
  oVector3 = o3+o7;
  oVector4 = o4+o8;
  Serial.println("o1 vector: " + oVector1);
  Serial.println("o2 vector: " + oVector2);
  Serial.println("o3 vector: " + oVector3);
  Serial.println("o4 vector: " + oVector4);  
  runMotor(2,oVector1);
  runMotor(3,oVector2);
  runMotor(4,oVector3);
  runMotor(5,oVector4);
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

void setUpAutoBalancing(){
  //setup desiredG
  desiredG[0] = currentG[0];
  desiredG[1] = currentG[1];
  autoBalancingSet = true;
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
        //previousG[0] = currentG[0];
        currentG[0] = atof(buf1) - 100;
        //errorG[0] = desiredG[0] - currentG[0];
        a2.toCharArray(buf2, 4);
        //previousG[1] = currentG[1];
        currentG[1] = atof(buf2) - 100;
        //errorG[1] = desiredG[1] - currentG[1];
        if(!autoBalancingSet){
          setUpAutoBalancing();
        }
        break;
      }else{
        break;
      }  
    }
  }
}
