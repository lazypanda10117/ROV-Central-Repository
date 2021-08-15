#include <Servo.h>
int value = 0; // set values you need to zero
Servo motorTestESC;

void setup() {
  motorTestESC.attach(3);    // attached to pin 9 I just do this with 1 Servo
  Serial.begin(9600);    // start serial at 9600 baud
}

void loop() { 
  value = analogRead(0);
  Serial.println(map(value,0,1023,1100,1900));
  motorTestESC.writeMicroseconds(map(value,0,1023,1100,1900));
  delay(300);
}

