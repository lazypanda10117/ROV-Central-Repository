#include <PS2X_lib.h>  //for v1.6

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your Arduino after you conect the controller,
//or call config_gamepad(pins) again after connecting the controller.
int error = 0;
byte type = 0;
byte vibrate = 0;

int directionPin1 = 4;      // MotorPin set to pin 53
int speedPin1 = 9;   // MotorDirectionPin set to pin 9

int directionPin2 = 8;      // MotorPin set to pin2
int speedPin2 = 10;   // MotorDirectionPin set to pin3

int directionPin3 = 7;      // MotorPin set to pin2
int speedPin3 = 11;  // MotorDirectionPin set to pin3

int directionPin4 = 12;      // MotorPin set to pin2
int speedPin4 = 13;   // MotorDirectionPin set to pin3

int L1Output = 0;
int R1Output = 1;

void setup() {
  Serial.begin(9600); // start serial port at 57600 bps
  pinMode(L1Output, OUTPUT);
  pinMode(R1Output, OUTPUT);
  pinMode(directionPin1, OUTPUT);
  pinMode(speedPin1, OUTPUT);
  pinMode(directionPin2, OUTPUT);
  pinMode(speedPin2, OUTPUT);
  pinMode(directionPin3, OUTPUT);
  pinMode(speedPin3, OUTPUT);
  pinMode(directionPin4, OUTPUT);
  pinMode(speedPin4, OUTPUT);

  error = ps2x.config_gamepad(6, 5, 3, 2, true, true); //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  if (error == 0) {
    Serial.println("Found Controller, configured successful");
  }

  else if (error == 1) {
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  }
  else if (error == 2) {
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  }
  else if (error == 3) {
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  }
  //Serial.print(ps2x.Analog(1), HEX);

  /*while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  */
}

void loop() {
  if (error == 1) { //skip loop if no controller found
    return;
  }

  ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed
  int analogSpeedLY = ps2x.Analog(PSS_LY);
  int analogSpeedLX = ps2x.Analog(PSS_LX);
  int analogSpeedRY = ps2x.Analog(PSS_RY);
  int analogSpeedRX = ps2x.Analog(PSS_RX);

  int analogSpeedUD = (analogSpeedLY);
  int analogSpeedSW = (analogSpeedLX);

  int valueOfLeftRotation = ps2x.Analog(PSAB_PAD_LEFT);
  int valueOfRightRotation = ps2x.Analog(PSAB_PAD_RIGHT);

  boolean isXpressed = ps2x.Button(PSB_BLUE);
  boolean isLeftPressed = ps2x.Button(PSB_PAD_LEFT);
  boolean isRightPressed = ps2x.Button(PSB_PAD_RIGHT);

  int isL1Pressed = ps2x.Button(PSB_L1);
  int isR1Pressed = ps2x.Button(PSB_R1);

  digitalWrite(L1Output, isL1Pressed);
  digitalWrite(R1Output, isR1Pressed);

  if (isXpressed == true) {
    Serial.println("State: Rotating");
    if (isLeftPressed == true && isRightPressed == false) {
      int RotationLeftSpeed = valueOfLeftRotation;
      analogWrite(speedPin1, RotationLeftSpeed);
      digitalWrite(directionPin1, HIGH);
      analogWrite(speedPin2,  RotationLeftSpeed);
      digitalWrite(directionPin2, LOW);
      analogWrite(speedPin3, RotationLeftSpeed);
      digitalWrite(directionPin3, LOW);
      analogWrite(speedPin4, RotationLeftSpeed);
      digitalWrite(directionPin4, HIGH);
      Serial.print("Rotating Left: ");
      Serial.println(RotationLeftSpeed);

    } else if (isRightPressed == true && isLeftPressed == false) {
      int RotationRightSpeed = valueOfRightRotation;
      analogWrite(speedPin1, RotationRightSpeed);
      digitalWrite(directionPin1, LOW);
      analogWrite(speedPin2,  RotationRightSpeed);
      digitalWrite(directionPin2, HIGH);
      analogWrite(speedPin3, RotationRightSpeed);
      digitalWrite(directionPin3, HIGH);
      analogWrite(speedPin4, RotationRightSpeed);
      digitalWrite(directionPin4, LOW);
      Serial.print("Rotating Right: ");
      Serial.println(RotationRightSpeed);
    }
    else if (isRightPressed == true && isLeftPressed == true) {
      int RotationRightSpeed = 0;
      analogWrite(speedPin1, RotationRightSpeed);
      digitalWrite(directionPin1, LOW);
      analogWrite(speedPin2,  RotationRightSpeed);
      digitalWrite(directionPin2, LOW);
      analogWrite(speedPin3, RotationRightSpeed);
      digitalWrite(directionPin3, LOW);
      analogWrite(speedPin4, RotationRightSpeed);
      digitalWrite(directionPin4, LOW);
      Serial.println("Contradiction");
    } else if (isRightPressed == false && isLeftPressed == false) {
      int RotationRightSpeed = 0;
      analogWrite(speedPin1, RotationRightSpeed);
      digitalWrite(directionPin1, LOW);
      analogWrite(speedPin2,  RotationRightSpeed);
      digitalWrite(directionPin2, LOW);
      analogWrite(speedPin3, RotationRightSpeed);
      digitalWrite(directionPin3, LOW);
      analogWrite(speedPin4, RotationRightSpeed);
      digitalWrite(directionPin4, LOW);
      Serial.println("Stationary");
    }
  } else if (isXpressed == false) {
    Serial.println("State: Moving");
    if (analogSpeedUD < 123) { //moving forward

      int ForwardYSpeed = 123 - analogSpeedUD;
      ForwardYSpeed = ( ForwardYSpeed * 2) + 9;

      if (analogSpeedSW > 123) { //moving right forward

        if (analogSpeedSW > 189) { // moving right forward variation forward
          int speedForwardRight = ((analogSpeedSW - 189) * 4) - 9;

          if (speedForwardRight < 0) {
            speedForwardRight = 0;
          }
          int speedRightPrint = (abs(123 - analogSpeedSW)) * 2 - 9;

          if (speedRightPrint < 0) {
            speedRightPrint = 0;
          }


          analogWrite(speedPin1, ForwardYSpeed);
          digitalWrite(directionPin1, LOW);

          analogWrite(speedPin2, speedForwardRight);
          digitalWrite(directionPin2, LOW);

          analogWrite(speedPin3, speedForwardRight);
          digitalWrite(directionPin3, HIGH);

          analogWrite(speedPin4, ForwardYSpeed);
          digitalWrite(directionPin4, HIGH);

          Serial.print("forward: ");
          Serial.println(ForwardYSpeed);
          Serial.print("right: ");
          Serial.println(speedRightPrint);

        }//end of analogSpeedSW > 189
        else if (analogSpeedSW <= 189) { // moving right forward variation backward
          int speedBackwardRight = ((189 - analogSpeedSW) * 4) - 9;

          if (speedBackwardRight < 0) {
            speedBackwardRight = 0;
          }
          int speedRightPrint = (abs(123 - analogSpeedSW)) * 2 - 9;

          if (speedRightPrint < 0) {
            speedRightPrint = 0;
          }

          analogWrite(speedPin1, ForwardYSpeed);
          digitalWrite(directionPin1, LOW);

          analogWrite(speedPin2, speedBackwardRight);
          digitalWrite(directionPin2, HIGH);

          analogWrite(speedPin3, speedBackwardRight);
          digitalWrite(directionPin3, LOW);

          analogWrite(speedPin4, ForwardYSpeed);
          digitalWrite(directionPin4, HIGH);

          Serial.print("forward: ");
          Serial.println(ForwardYSpeed);
          Serial.print("right: ");
          Serial.println(speedRightPrint);

        }//end of analogSpeedSW <= 189

      }//end of analogSpeedSW >= 123

      else if (analogSpeedSW < 123) { //moving left forward

        if (analogSpeedSW < 62) { //moving left forward variation forward
          int LeftSpeedForward = (analogSpeedSW - 62) * 4 + 7;
          int SpeedLeftForwardPrint = (abs(123 - analogSpeedSW)) * 2 + 9;

          if (LeftSpeedForward <= 7) {
            LeftSpeedForward = 0;
          }

          analogWrite(speedPin1, LeftSpeedForward);
          digitalWrite(directionPin1, LOW);

          analogWrite(speedPin2, ForwardYSpeed);
          digitalWrite(directionPin2, LOW);

          analogWrite(speedPin3, ForwardYSpeed);
          digitalWrite(directionPin3, HIGH);

          analogWrite(speedPin4, LeftSpeedForward);
          digitalWrite(directionPin4, HIGH);

          Serial.print("Forward: ");
          Serial.println(ForwardYSpeed);
          Serial.print("Left: ");
          Serial.println(SpeedLeftForwardPrint);

        }//end of analogSpeedSW < 62
        else if (analogSpeedSW >= 62) { //moving left forward variation backward
          int LeftSpeedBackward = (62 - analogSpeedSW) * 4 - 7;
          int LeftSpeedBackwardPrint = (abs(123 - analogSpeedSW)) * 2 + 9;

          if (LeftSpeedBackward <= 0) {
            LeftSpeedBackward = 0;
          }

          analogWrite(speedPin1, LeftSpeedBackward);
          digitalWrite(directionPin1, HIGH);

          analogWrite(speedPin2, ForwardYSpeed);
          digitalWrite(directionPin2, LOW);

          analogWrite(speedPin3, ForwardYSpeed);
          digitalWrite(directionPin3, HIGH);

          analogWrite(speedPin4, LeftSpeedBackward);
          digitalWrite(directionPin4, LOW);

          Serial.print("Forward: ");
          Serial.println(ForwardYSpeed);
          Serial.print("Left: ");
          Serial.println(LeftSpeedBackwardPrint);

        }//end of analogSpeedSW >= 62

      }//end of analogSpeedSW <123
      else if (analogSpeedSW = 123) { //moving forward with speed variation

        analogWrite(speedPin1, ForwardYSpeed);
        digitalWrite(directionPin1, LOW);

        analogWrite(speedPin2, ForwardYSpeed);
        digitalWrite(directionPin2, LOW);

        analogWrite(speedPin3, ForwardYSpeed);
        digitalWrite(directionPin3, HIGH);

        analogWrite(speedPin4, ForwardYSpeed);
        digitalWrite(directionPin4, HIGH);

        Serial.print("Forward: ");
        Serial.println(ForwardYSpeed);
        Serial.println("Just Moving Forward");

      } //end of analogSpeedSW = 123

    } //end of the forward portion : analogSpeedUD < 123

    else if (analogSpeedUD > 123) { //Moving Backward
      int BackwardYSpeed = analogSpeedUD - 123;
      BackwardYSpeed = (BackwardYSpeed * 2) - 9;

      if (BackwardYSpeed <= 0) {
        BackwardYSpeed = 0;
      }

      if (analogSpeedSW > 123) { //moving backward right
        if (analogSpeedSW > 189) {
          int speedForwardRight = ((analogSpeedSW - 189) * 4) - 9;
          int speedRightPrint = (abs(123 - analogSpeedSW)) * 2 - 9;
          if (speedRightPrint < 0) {
            speedRightPrint = 0;
          }
          if (speedForwardRight < 0) {
            speedForwardRight = 0;
          }

          analogWrite(speedPin1, speedForwardRight);
          digitalWrite(directionPin1, LOW);

          analogWrite(speedPin2, BackwardYSpeed);
          digitalWrite(directionPin2, HIGH);

          analogWrite(speedPin3, BackwardYSpeed);
          digitalWrite(directionPin3, LOW);

          analogWrite(speedPin4, speedForwardRight);
          digitalWrite(directionPin4, HIGH);

          Serial.print("backward: ");
          Serial.println(BackwardYSpeed);
          Serial.print("right: ");
          Serial.println(speedRightPrint);

        } //end of analogSpeedSW > 189
        else if (analogSpeedSW <= 189) {
          int speedBackwardRight = ((189 - analogSpeedSW) * 4) - 9;
          int speedRightPrint = (abs(123 - analogSpeedSW)) * 2 - 9;
          if (speedRightPrint < 0) {
            speedRightPrint = 0;
          }
          if (speedBackwardRight < 0) {
            speedBackwardRight = 0;
          }

          analogWrite(speedPin1, speedBackwardRight);
          digitalWrite(directionPin1, HIGH);

          analogWrite(speedPin2, BackwardYSpeed);
          digitalWrite(directionPin2, HIGH);

          analogWrite(speedPin3, BackwardYSpeed);
          digitalWrite(directionPin3, LOW);

          analogWrite(speedPin4, speedBackwardRight);
          digitalWrite(directionPin4, LOW);

          Serial.print("backward: ");
          Serial.println(BackwardYSpeed);
          Serial.print("right: ");
          Serial.println(speedRightPrint);
        }//end of analogSpeedSW <= 189
      } else if (analogSpeedSW < 123) { //moving backward left
        if (analogSpeedSW > 61) {
          int speedBackwardLeft = (analogSpeedSW - 61) * 4 + 7;
          int speedLeftPrint = (abs(123 - analogSpeedSW)) * 2 + 9;
          if (speedBackwardLeft = 7) {
            speedBackwardLeft = 0;
          }
          analogWrite(speedPin1, BackwardYSpeed);
          digitalWrite(directionPin1, HIGH);

          analogWrite(speedPin2, speedBackwardLeft);
          digitalWrite(directionPin2, HIGH);

          analogWrite(speedPin3, speedBackwardLeft);
          digitalWrite(directionPin3, LOW);

          analogWrite(speedPin4, BackwardYSpeed);
          digitalWrite(directionPin4, LOW);

          Serial.print("backward: ");
          Serial.println(BackwardYSpeed);
          Serial.print("left: ");
          Serial.println(speedLeftPrint);
        } else if (analogSpeedSW <= 61) {
          int speedForwardLeft = (61 - analogSpeedSW) * 4 + 7;
          int speedLeftPrint = (abs(123 - analogSpeedSW)) * 2 + 9;
          if (speedForwardLeft <= 7) {
            speedForwardLeft = 0;
          }

          analogWrite(speedPin1, BackwardYSpeed);
          digitalWrite(directionPin1, HIGH);

          analogWrite(speedPin2, speedForwardLeft);
          digitalWrite(directionPin2, LOW);

          analogWrite(speedPin3, speedForwardLeft);
          digitalWrite(directionPin3, HIGH);

          analogWrite(speedPin4, BackwardYSpeed);
          digitalWrite(directionPin4, LOW);

          Serial.println("backward: ");
          Serial.println(BackwardYSpeed);
          Serial.println("left: ");
          Serial.println(speedLeftPrint);
        }
      } else if (analogSpeedSW = 123) { //moving backward with speed variation
        analogWrite(speedPin1, BackwardYSpeed);
        digitalWrite(directionPin1, HIGH);

        analogWrite(speedPin2, BackwardYSpeed);
        digitalWrite(directionPin2, LOW);

        analogWrite(speedPin3, BackwardYSpeed);
        digitalWrite(directionPin3, HIGH);

        analogWrite(speedPin4, BackwardYSpeed);
        digitalWrite(directionPin4, LOW);

        Serial.print("backward: ");
        Serial.println(BackwardYSpeed);
        Serial.println("Moving Only Backward");
      }
      // end of analogSpeedUD > 123
    } else if (analogSpeedUD = 123) { //not moving forward or backward
      if (analogSpeedSW > 123) { //moving right only with speed variation
        int RightXSpeed = (analogSpeedSW - 123) * 2 - 9;
        if (RightXSpeed < 0) {
          RightXSpeed = 0;
        }
        analogWrite(speedPin1, RightXSpeed);
        digitalWrite(directionPin1, LOW);

        analogWrite(speedPin2, RightXSpeed);
        digitalWrite(directionPin2, HIGH);

        analogWrite(speedPin3, RightXSpeed);
        digitalWrite(directionPin3, LOW);

        analogWrite(speedPin4, RightXSpeed);
        digitalWrite(directionPin4, HIGH);

        Serial.print("Right: ");
        Serial.println(RightXSpeed);
        Serial.println("Moving Only Right");
      } else if (analogSpeedSW < 123) { //moving left only with speed variation
        int LeftXSpeed = (123 - analogSpeedSW) * 2 + 9;
        if (LeftXSpeed < 9) {
          LeftXSpeed = 0;
        }

        analogWrite(speedPin1, LeftXSpeed);
        digitalWrite(directionPin1, HIGH);

        analogWrite(speedPin2, LeftXSpeed);
        digitalWrite(directionPin2, LOW);

        analogWrite(speedPin3, LeftXSpeed);
        digitalWrite(directionPin3, HIGH);

        analogWrite(speedPin4, LeftXSpeed);
        digitalWrite(directionPin4, LOW);

        Serial.print("Left");
        Serial.println(LeftXSpeed);
        Serial.println("No up or down value!");
      } else if (analogSpeedSW = 123) { //Stationary
        analogWrite(speedPin1, 0);
        digitalWrite(directionPin1, LOW);

        analogWrite(speedPin2, 0);
        digitalWrite(directionPin2, LOW);

        analogWrite(speedPin3, 0);
        digitalWrite(directionPin3, LOW);

        analogWrite(speedPin4, 0);
        digitalWrite(directionPin4, LOW);

        Serial.println("Stationary Position");
      }// end of analogSpeedUD = 123

    }
  }
  delay(750);
}//close void loop()
