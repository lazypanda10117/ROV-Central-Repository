#include <PS2X_lib.h>  //for v1.6

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you conect the controller, 
//or call config_gamepad(pins) again after connecting the controller.
int error = 0; 
byte type = 0;
byte vibrate = 0;

int MotorPinV1 = 2;      // MotorPin set to pin2
int MotorForwardV1 = 3;   // MotorDirectionPin set to pin3
int MotorBackwardV1 = 5;   // MotorDirectionPin set to pin3

int MotorPinV2 = 7;      // MotorPin set to pin2
int MotorForwardV2 = 6;   // MotorDirectionPin set to pin3
int MotorBackwardV2 = 9;   // MotorDirectionPin set to pin3

int checkL1 = 13;
int checkR1 = 11;

void setup() {
  Serial.begin(57600); // start serial port at 57600 bps
   pinMode(MotorPinV1, OUTPUT);  // sets the pin as output
   pinMode(MotorForwardV1, OUTPUT); // sets the pin as output
   pinMode(MotorBackwardV1, OUTPUT);
   pinMode(MotorPinV2, OUTPUT);  // sets the pin as output
   pinMode(MotorForwardV2, OUTPUT); // sets the pin as output
   pinMode(MotorBackwardV2, OUTPUT);
   pinMode(checkL1, INPUT);
   pinMode(checkR1, INPUT);
}
void loop() {
  digitalWrite(MotorPinV1, HIGH); 
  digitalWrite(MotorPinV2, HIGH); 
  int L1bool = digitalRead(checkL1);
  int L2bool = digitalRead(checkR1);
  if (L1bool == 1 && L2bool == 0){
    int MoveUpward = 255;
    analogWrite(MotorForwardV1,MoveUpward);
    analogWrite(MotorBackwardV1,0);
    analogWrite(MotorForwardV2,MoveUpward);
    analogWrite(MotorBackwardV2,0);
    Serial.println("Upward");
    //end of L1 pressed
  }else if (L1bool == 0 && L2bool == 1){
    int MoveDownward = 255;
    analogWrite(MotorForwardV1,0);
    analogWrite(MotorBackwardV1,MoveDownward);
    analogWrite(MotorForwardV2,0);
    analogWrite(MotorBackwardV2,MoveDownward);
    Serial.println("Downward");
    //end of R1 pressed
  }else if(L1bool == 0 && L2bool == 0){
   Serial.println("Vertical Stationary"); 
   analogWrite(MotorForwardV1,0);
   analogWrite(MotorBackwardV1,0);
   analogWrite(MotorForwardV2,0);
   analogWrite(MotorBackwardV2,0);
  }else if(L1bool == 1 && L2bool == 1){
   Serial.println("Contradiction"); 
   analogWrite(MotorForwardV1,0);
   analogWrite(MotorBackwardV1,0);
   analogWrite(MotorForwardV2,0);
   analogWrite(MotorBackwardV2,0);
  }
  delay(10);
}
