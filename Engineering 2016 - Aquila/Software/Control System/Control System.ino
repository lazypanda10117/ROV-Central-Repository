int xPin = A0; //potentiometer data pin for x axis
int yPin = A1; //potentiometer data pin for y axis
int upBtn = 3; //pin for the up button
int downBtn = 4; //pin for the down button
int changeStateBtn = 2;
//motors for forward and backward, left and right
int directionPin1 = 5;      // MotorPin set to pin 53
int speedPin1 = 9;   // MotorDirectionPin set to pin 9
int directionPin2 = 7;      // MotorPin set to pin2
int speedPin2 = 6;   // MotorDirectionPin set to pin3

//motors for up and down
int directionPin3 = 8;      // MotorPin set to pin2
int speedPin3 = 11;  // MotorDirectionPin set to pin3
int directionPin4 = 12;      // MotorPin set to pin2
int speedPin4 = 13;   // MotorDirectionPin set to pin3

int xVal = 0; //value of x-axis potentiometer
int yVal = 0; //value of y-axis potentiometer
int xSpeed = 0; //speed of the rov in x axis
int ySpeed = 0; //speed of the rov in x axis

int xStringSpeed = 0;
int yStringSpeed = 0;

int xSpeedForState = 0;
int ySpeedForState = 0;

int forwardBackwardState = 0; //0 = stable, 1 = forward, 2 = backward
int leftRightState = 0; //0 = stable, 1 = right, 2 = left
int upDownState = 0; //0= stable, 1= up, 2= down


int yState = LOW;

#define Full_Speed 254 //full pwm speed

void setup() {
  Serial.begin(9600);

  //pinMode(startUpPin, INPUT);
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(upBtn, INPUT);
  pinMode(downBtn, INPUT);
  pinMode(changeStateBtn, INPUT);
  pinMode(directionPin1, OUTPUT);
  pinMode(speedPin1, OUTPUT);
  pinMode(directionPin2, OUTPUT);
  pinMode(speedPin2, OUTPUT);
  pinMode(directionPin3, OUTPUT);
  pinMode(speedPin3, OUTPUT);
  pinMode(directionPin4, OUTPUT);
  pinMode(speedPin4, OUTPUT);
}

//the main function that will be recursively executed while the program is running
void loop() {
    setConstrainedAnalogValue();
    mapAnalogValue();
    xSpeedMapping();
    ySpeedMapping();
    totalMovement();
    delay(500);
}

void setConstrainedAnalogValue() {
  xVal = constrain(analogRead(xPin), 140, 920); //making 140 min and 920 max to prevent slight changes in extreme state
  yVal = constrain(analogRead(yPin), 100, 900); //making 100 min and 900 max to prevent slight changes in extreme state
}

void mapAnalogValue() {
  xVal = map(xVal, 140, 920, -10, 10) + 1; //make equilbrium point at 0
  yVal = - (map(yVal, 100, 900, -10, 10) + 1); //make equilbrium point at 0
}

void upDownMovement(int btnUpPin, int btnDownPin) {
  if (digitalRead(btnUpPin) == LOW && digitalRead(btnDownPin) == HIGH) { //moving up
    upDownState = 1;
    analogWrite(speedPin3, Full_Speed);
    digitalWrite(directionPin3, HIGH);
    analogWrite(speedPin4, Full_Speed);
    digitalWrite(directionPin4, HIGH);
    Serial.println("Moving up at full speed");
  } else if (digitalRead(btnUpPin) == HIGH && digitalRead(btnDownPin) == LOW) { //moving down
    upDownState = 2;
    analogWrite(speedPin3, Full_Speed);
    digitalWrite(directionPin3, LOW);
    analogWrite(speedPin4, Full_Speed);
    digitalWrite(directionPin4, LOW);
    Serial.println("Moving down at full speed");
  } else {
    upDownState = 0;
    analogWrite(speedPin3, 0);
    digitalWrite(directionPin3, LOW);
    analogWrite(speedPin4, 0);
    digitalWrite(directionPin4, LOW);
    Serial.println("No up down movement"); //not moving up or down
  }
}

void xyMovement() {
  if(digitalRead(changeStateBtn) == LOW){
    xMovement();
  }else{
    yMovement();
  }
}

void xMovement(){
  if (leftRightState == 1) { //rotate right
      analogWrite(speedPin1, 0);
      digitalWrite(directionPin1, LOW);
      analogWrite(speedPin2, xSpeed);
      digitalWrite(directionPin2, HIGH);
      Serial.println("Rotating right");
    } else if (leftRightState == 2) { //rotate left
      analogWrite(speedPin1, xSpeed);
      digitalWrite(directionPin1, HIGH);
      analogWrite(speedPin2, 0);
      digitalWrite(directionPin2, LOW);
      Serial.println("Rotating left");
    } else {
      analogWrite(speedPin1, 0);
      digitalWrite(directionPin1, LOW);
      analogWrite(speedPin2, 0);
      digitalWrite(directionPin2, LOW);
      Serial.println("No XY movemnt");
    }  
}

void yMovement(){
    if (forwardBackwardState == 1) {
      yState = HIGH; //going forward
      Serial.print("Moving forward at");
      Serial.println(ySpeed);
    } else if (forwardBackwardState == 2) {
      yState = LOW; //going backward
      Serial.print("Moving backward at");
      Serial.println(ySpeed);
    } else {
      yState = LOW; //not moving
      Serial.println("Not moving");
    }
    analogWrite(speedPin1, ySpeed);
    digitalWrite(directionPin1, yState);
    analogWrite(speedPin2, ySpeed);
    digitalWrite(directionPin2, yState);
}
//the movement function that controls the pwn that control the motors
void totalMovement() {
  upDownMovement(upBtn, downBtn);
  xyMovement();
}

void stopAllPin() {
  analogWrite(speedPin1, 0);
  digitalWrite(directionPin1, LOW);
  analogWrite(speedPin2, 0);
  digitalWrite(directionPin2, LOW);
  analogWrite(speedPin3, 0);
  digitalWrite(directionPin3, LOW);
  analogWrite(speedPin4, 0);
  digitalWrite(directionPin4, LOW);
}

//map xValue into xSpeed with max = 255 and min = 0
void xSpeedMapping() {
  if (xVal > 0) {
    leftRightState = 1;
    xSpeed = map(xVal, 1, 10, 0, 255);
    xSpeedForState = xSpeed;
  } else if (xVal < 0) {
    leftRightState = 2;
    xSpeed = map(xVal, -1, -9, 0, 255);
    xSpeedForState = -xSpeed;
  } else {
    leftRightState = 0;
    xSpeed = 0;
    xSpeedForState = xSpeed;
  }
}

//map yValue into ySpeed with max = 255 and min = 0
void ySpeedMapping() {
  if (yVal > 0 &&  yVal <= 9) {
    forwardBackwardState = 1;
    ySpeed = map(yVal, 1, 9, 0, 254);
    ySpeedForState = ySpeed;
  } else if (yVal < 0 && yVal >= -11) {
    forwardBackwardState = 2;
    ySpeed = map(yVal, -1, -11, 0, 254);
    ySpeedForState = -ySpeed;
  } else {
    forwardBackwardState = 0;
    ySpeed = 0;
    ySpeedForState = ySpeed;
  }
}

void delayAlternateState(boolean rState, boolean negativityState, int inputSpeed) {
 /* if (negativityState == true) {
    if (inputSpeed >= 0) {
      negativityState = false;
      rState = true;
      stopAllPin();
      Serial.println("Delaying 100 ms");
      delay(100);
    } else {
      //stable at 0
    }
  } else {
    if (inputSpeed < 0) {
      negativityState = true;
      rState = true;
      stopAllPin();
      Serial.println("Delaying 100 ms");
      delay(100);
    } else {
      //stable at 0
    }
  }
  */
}
