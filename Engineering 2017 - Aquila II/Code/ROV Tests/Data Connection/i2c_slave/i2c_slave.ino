// Include the required Wire library for I2C<br>#include <Wire.h>
#include "Wire.h"
int LED = 13;
int x = 0;
void setup() {
  // Define the LED pin as Output
  pinMode (LED, OUTPUT);
  // Start the I2C Bus as Slave on address 9
  Wire.begin(9); 
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
}
void receiveEvent(int bytes) {
  x = Wire.read();    // read one character from the I2C
  Serial.print(x);
}
void loop() {

}
