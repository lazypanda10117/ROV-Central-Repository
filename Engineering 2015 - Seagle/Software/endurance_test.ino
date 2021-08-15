
int movingDirection = 9;   // MotorDirectionPin set to pin3
int pwmSpeed = 13;   // MotorDirectionPin set to pin3

void setup() {
   Serial.begin(9600); // start serial port at 9600 bps
   randomSeed(analogRead(0));   
   pinMode(movingDirection, OUTPUT);  // sets the direction pin as output
   pinMode(pwmSpeed, OUTPUT); // sets the pwm pin as output
}

void loop() {
  int motorSpeed = random(50,255);
  int leftOrRight = random(2);
  //Serial.println(leftOrRight); 
  //if (leftOrRight = 0){
          analogWrite(pwmSpeed, motorSpeed);
          digitalWrite(movingDirection, LOW);
          Serial.println("forward: ");
          Serial.println(0);
	  Serial.println("backward:");
          Serial.println(motorSpeed);
  /*}else if(leftOrRight = 1){
          analogWrite(pwmSpeed, motorSpeed);
          digitalWrite(movingDirection, LOW);
          Serial.println("forward: ");
          Serial.println(0);
	  Serial.println("backward:");
          Serial.println(motorSpeed);
  }
  */
  delay(1000); 
}//close void loop()

