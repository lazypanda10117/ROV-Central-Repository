int pwm1Pin = 6;
int val = 0;
int prevVal = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(pwm1Pin, OUTPUT); // sets the pin as output
  pinMode(3, OUTPUT); // sets the pin as output
  pinMode(4, OUTPUT); // sets the pin as output
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  prevVal = val;
  //val = Serial.parseInt();
  //if(val == 0){
    //val = prevVal;
  //}
  float voltage= analogRead(A5) * (5.0 / 1023.0);
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);
  analogWrite(pwm1Pin, 130);
  Serial.println(voltage);
  delay(10);
}
