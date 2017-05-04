//Author: Jeffrey Kam
//Date: 3rd Janurary 2017
//Non-distributable, Copyrighted 2017
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define INTERRUPT_PIN 2

#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//#include "Wire.h"
//#endif
//#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <PS3USB.h>
#include <usbhub.h>

USB Usb;
//BTD Btd(&Usb);
//PS3BT PS3(&Btd);
PS3USB PS3(&Usb); // This will just create the instance

//MPU6050 mpu;

Servo armServo;
Servo camServoX;
Servo camServoY;

//variable for pins
int armServoPin = 7;
int camServoPinX = 14;
int camServoPinY = 15;
int motorPin1 = 8;
int dir1 = 1;
int motorPin2 = 9;
int dir2 = 2;
int zMotorPin1 = 10;
int zDir1 = 3;
int zMotorPin2 = 11;
int zDir2 = 4;
int zMotorPin3 = 12;
int zDir3 = 5;
int zMotorPin4 = 13;
int zDir4 = 6;


//variables for different modes
int controlMode = 0; //0 for ps3 control, 1 for keyboard
int processMode = 0; //0 for setup, 1 for stop, 2 for play
int startUpSteps = 1;

//arm angle threshold
int armMinT = 0;
int armMaxT = 180;

//arm current angle
double armAng = (armMinT + armMaxT) / 2;

//camera angle threshold
int leftCamT = 0;
int rightCamT = 180;
int upCamT = 106;
int downCamT = 180;

//camera current angle
double camAngX = (leftCamT + rightCamT) / 2; //90 mid
double camAngY = (upCamT + downCamT) / 2; //143 mid

//variables for limits of control and speed
int joystickThresholdMin = 35;
int joystickThresholdMax = 220;

int minSpeed = -255;
int maxSpeed = 255;
int stationarySpeed = 0;

//speed variables for motors
int lSpeed = 0;
int rSpeed = 0;
int speedZ1 = 0;
int speedZ2 = 0;
int speedZ3 = 0;
int speedZ4 = 0;
boolean zStationary = true;

/*
//Gyroscope(MPU6050) Variables
bool dmpReady = false;  // set true if DMP init was successful
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float prevYpr[3];
float offsetYpr[3]; //initial positions
boolean stable = false;
boolean set = false;
int rollRaw;
int pitchRaw;
int rollNew;
int pitchNew;
int rollInitial;
int pitchInitial;
*/

//autobalancing variables
//ignore yaw because planar movement is not of concern
double desiredPitch = 0;
double desiredRoll = 0;
double previousPitch = 0;
double previousRoll = 0;
double currentPitch = 0;
double currentRoll = 0;
double kp = 0;
double ki = 0;
double kd = 0;


//send to rov section
String compiledString = "";

void setup()
{
  Serial.begin(115200);
  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
  if (Usb.Init() == -1) {
    Serial.println(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.println(F("\r\nPS3 Bluetooth Library Started"));
  
  //initializeMPUSensor();
  armServo.attach(armServoPin);
  camServoX.attach(camServoPinX);
  camServoY.attach(camServoPinY);

  armServo.write(armAng);
  camServoX.write(camAngX);
  camServoY.write(camAngY);
  //Serial.println("ROVSTARTINGSECRETKEY");
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
    //start up process
    startUpProcess(startUpSteps);
  } else if (processMode == 2) {
    if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
      int sA = PS3.getAnalogButton(SQUARE);
      if (sA >= 100) {
        processMode = 3;
      }
      delay(100);
    }
    delay(50);
  } else if (processMode == 3) {
    if (controlMode == 0) {
      controlModePS3();
    }/*else if(controlMode == 1){
      controlModeKeyboard();
    }*/
    
  }
  debugMotorOutput(lSpeed, rSpeed, speedZ1, speedZ2, speedZ3, speedZ4);
  sendROVStats();
  delay(200);
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
    //process mode input
    int triangleVal = PS3.getAnalogButton(TRIANGLE); //emergency stop
    int squareVal = PS3.getAnalogButton(SQUARE); //start startup
    lSpeed = planarSpeed(leftValue);
    rSpeed = planarSpeed(rightValue);
    planarMovement(lSpeed, rSpeed);
    zMovement();
    cameraControlX(upVal, downVal);
    cameraControlY(leftVal, rightVal);
    armControl(L1Val, R1Val);
    PS3CheckMode(triangleVal, squareVal);
  }
}

/*void controlModeKeyboard(){
  }*/

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
    /*
    writeAsPWM(motorPin1, dir1, maxSpeed);
    writeAsPWM(motorPin2, dir2, maxSpeed);
    writeAsPWM(zMotorPin1, zDir1, maxSpeed);
    writeAsPWM(zMotorPin2, zDir2, maxSpeed);
    writeAsPWM(zMotorPin3, zDir3, maxSpeed);
    writeAsPWM(zMotorPin4, zDir4, maxSpeed);
    */
    debugMotorOutput(maxSpeed, maxSpeed, maxSpeed, maxSpeed, maxSpeed, maxSpeed);
    startUpSteps++;
    delay(3000);
  } else if (steps == 2) {
    /*
    writeAsPWM(motorPin1, dir1, stationarySpeed);
    writeAsPWM(motorPin2, dir2, stationarySpeed);
    writeAsPWM(zMotorPin1, zDir1, stationarySpeed);
    writeAsPWM(zMotorPin2, zDir2, stationarySpeed);
    writeAsPWM(zMotorPin3, zDir3, stationarySpeed);
    writeAsPWM(zMotorPin4, zDir4, stationarySpeed);
    */
    debugMotorOutput(stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed);
    startUpSteps++;
    delay(1000);
  } else if (steps == 3) {
    /*
    writeAsPWM(motorPin1, dir1, minSpeed);
    writeAsPWM(motorPin2, dir2, minSpeed);
    writeAsPWM(zMotorPin1, zDir1, minSpeed);
    writeAsPWM(zMotorPin2, zDir2, minSpeed);
    writeAsPWM(zMotorPin3, zDir3, minSpeed);
    writeAsPWM(zMotorPin4, zDir4, minSpeed);
    */
    debugMotorOutput(minSpeed,minSpeed,minSpeed, minSpeed, minSpeed, minSpeed);
    startUpSteps++;
    delay(3000);
  } else if (steps == 4) {
    /*
    writeAsPWM(motorPin1, dir1, stationarySpeed);
    writeAsPWM(motorPin2, dir2, stationarySpeed);
    writeAsPWM(zMotorPin1, zDir1, stationarySpeed);
    writeAsPWM(zMotorPin2, zDir2, stationarySpeed);
    writeAsPWM(zMotorPin3, zDir3, stationarySpeed);
    writeAsPWM(zMotorPin4, zDir4, stationarySpeed);
    */
    debugMotorOutput(stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed);
    processMode = 3;
    startUpSteps = 1;
    delay(500);
  }
}

//---Section of Process 2 (STOP Process)---
void stopProcess() {
  processMode = 2;
  /*
    writeAsPWM(motorPin1, dir1, stationarySpeed);
    writeAsPWM(motorPin2, dir2, stationarySpeed);
    writeAsPWM(zMotorPin1, zDir1, stationarySpeed);
    writeAsPWM(zMotorPin2, zDir2, stationarySpeed);
    writeAsPWM(zMotorPin3, zDir3, stationarySpeed);
    writeAsPWM(zMotorPin4, zDir4, stationarySpeed);
  */
  debugMotorOutput(stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed, stationarySpeed);
}

void resumeProcess() {
  processMode = 3;
}

//---Section of Planar Movement---
int planarSpeed(int joystickVal) {
  return map(constrain(joystickVal, joystickThresholdMin, joystickThresholdMax), joystickThresholdMin, joystickThresholdMax, maxSpeed, minSpeed);

}

void planarMovement(int m1, int m2) {
  //writeAsPWM(motorPin1, dir1, m1);
  //writeAsPWM(motorPin2, dir2, m2);
  lSpeed = m1;
  rSpeed = m2;
}

//---Section of Vertical Movement---
void zMovement() {
  int mag = getMagnitudeZ();
  //k is the respective values for up and down magnitude
  if (zStationary) {
    //autobalancing();
  } else {
    speedZ1 = mag;
    speedZ2 = mag;
    speedZ3 = mag;
    speedZ4 = mag;
    //writeAsPWM(zMotorPin1, zDir1, mag);
    //writeAsPWM(zMotorPin2, zDir2, mag);
    //writeAsPWM(zMotorPin3, zDir3, mag);
    //writeAsPWM(zMotorPin4, zDir4, mag);
  }
}

int getMagnitudeZ() {
  int mag = 0;
  if (PS3.getAnalogButton(L2)) {
    mag = map(PS3.getAnalogButton(L2), 0, 255, stationarySpeed, maxSpeed);
  } else if (PS3.getAnalogButton(R2)) {
    mag = map(PS3.getAnalogButton(R2), 0, 255, stationarySpeed, minSpeed);
  }
  if (mag == stationarySpeed) {
    zStationary = true;
  } else {
    zStationary = false;
  }
  return mag;
}
/*
  //---Section of Autobalancing---
  void initializeMPUSensor() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  while (!Serial);
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  //Setting initial YPR to 9999
  for (int i = 0; i < 3; i++) {
    prevYpr[i] = 9999;
  }

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nError in PS3 Controller Connection"));
    while (1);
  }
  Serial.print(F("\r\nController Initiation Process Completed"));

  }

  void dmpDataReady() {
  mpuInterrupt = true;
  }

  void stabalizeMPU() {
  //set up to be stable
  if (abs(prevYpr[2] - (ypr[2] * 180 / M_PI)) <= 0.005 && abs(prevYpr[1] - (ypr[1] * 180 / M_PI)) <= 0.005 && abs(prevYpr[2] - (ypr[2] * 180 / M_PI))) {
    delay(5000);
    if (abs(prevYpr[2] - (ypr[2] * 180 / M_PI)) <= 0.005 && abs(prevYpr[1] - (ypr[1] * 180 / M_PI)) <= 0.005 && abs(prevYpr[2] - (ypr[2] * 180 / M_PI))) {
      stable = true;
      for (int i = 0; i < 3; i++) {
        offsetYpr[i] = ypr[i] * 180 / M_PI;
      }
    }
  } else {
    prevYpr[0] = ypr[0] * 180 / M_PI;
    prevYpr[1] = ypr[1] * 180 / M_PI;
    prevYpr[2] = ypr[2] * 180 / M_PI;
    Serial.println("NOT STABLE");
  }
  }

  void getMPUValue() {
  if (!dmpReady) {
    return;
  }
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  #ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    if (stable) {
      if (!set) {
        rollInitial = rollRaw + 180;
        pitchInitial = pitchRaw + 80;//have to check
        set = true;
      }
      pitchRaw = ypr[1] * 180 / M_PI;
      rollRaw = ypr[2] * 180 / M_PI;
      pitchNew = pitchRaw + 80;//have to check
      rollNew = rollRaw + 180;
      currentPitch = pitchNew;
      currentRoll = rollNew;
    } else {
      stabalizeMPU();
    }
  #endif
  }
  }

  void autobalancing() {
  //Drive = kP*Error + kI*Σ Error + kD * dP/dT
  //set each speedZ"n" values for n=1,2,3,4

  }

*/
//---Section of Camera Control---
void cameraControlX(int lV, int rV) {
  if (lV >= 50) {
    if (inRange(camAngX, leftCamT, rightCamT)) {
      camAngX += 1;
      Serial.println("Camera Angle X: " + String(camAngX));
    }
  } else if (rV >= 50) {
    if (inRange(camAngX, leftCamT, rightCamT)) {
      camAngX -= 1;
      Serial.println("Camera Angle X: " + String(camAngX));
    }
  }
}

void cameraControlY(int uV, int dV) {
  if (uV >= 50) {
    if (inRange(camAngY, downCamT, upCamT)) {
      camAngY += 1;
      Serial.println("Camera Angle Y: " + String(camAngY));
    }
  }else if (dV >= 50) {
    if (inRange(camAngY, downCamT, upCamT)) {
      camAngY -= 1;
      Serial.println("Camera Angle Y: " + String(camAngY));
    }
  }
}


//---Section of Arm Control---
void armControl(int openVal, int closeVal) {
  if (openVal >= 50) {
    if (inRange(armAng, armMinT, armMaxT)) {
      armAng += 1;
      Serial.println("Arm Angle: " + String(armAng));
    }
  } else if (closeVal >= 50) {
    if (inRange(armAng, armMinT, armMaxT)) {
      armAng -= 1;
      Serial.println("Arm Angle: " + String(armAng));
    }
  }
}

//---Section of Command Processing---
void receiveCommand() {
  //get command array from Processing
}

void parseCommand() {
  //parse command from processing
  //set processMode and controlMode

}

//---Section of Miscellaneous Functions---
bool inRange(double val, int minimum, int maximum) {
  return ((minimum < val) && (val < maximum));
}

void writeAsPWM(int motorPin, int directionPin, int value){
  int absValue = abs(value);
  if(value > 0){
    digitalWrite(directionPin,HIGH);  
    analogWrite(motorPin,absValue);
  }else{
    digitalWrite(directionPin,LOW);  
    analogWrite(motorPin,absValue);
  }
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
  compiledString += String(controlMode);
  compiledString += ",";
  //pitch and roll
  compiledString += String(currentPitch);
  compiledString += ",";
  compiledString += String(currentRoll);
  compiledString += ",";
  //arm angle
  compiledString += String(armAng, 2);
  compiledString += ",";
  //camera angle
  compiledString += String(camAngX, 2);
  compiledString += ",";
  compiledString += String(camAngY, 2);
  compiledString += ",";
  //speed stats
  compiledString += String(transformSpeedTo100(lSpeed));
  compiledString += ",";
  compiledString += String(transformSpeedTo100(rSpeed));
  compiledString += ",";
  compiledString += String(transformSpeedTo100(speedZ1));
  compiledString += ",";
  compiledString += String(transformSpeedTo100(speedZ2));
  compiledString += ",";
  compiledString += String(transformSpeedTo100(speedZ3));
  compiledString += ",";
  compiledString += String(transformSpeedTo100(speedZ4));
  Serial.println(compiledString);
  Serial.print("\n");
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

