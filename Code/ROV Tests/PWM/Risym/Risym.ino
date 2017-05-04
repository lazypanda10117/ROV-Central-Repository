void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);  
}

void loop() {
  // put your main code here, to run repeatedly:
  //int x = 0;
  analogWrite(3,250);//250 is the speed, 0-255 is the range
  digitalWrite(4,LOW); //high = cw/ccw depending on wiring
  digitalWrite(5,HIGH); //high = ccw/cw depending on wiring
  analogWrite(6,230); //when low low or high high, analogwrite must be 0
  digitalWrite(7,LOW); //high = cw/ccw depending on wiring
  digitalWrite(8,HIGH); //high = ccw/cw depending on wiring
  delay(150);
  /*if(x%6==0){
    analogWrite(3,250);
    digitalWrite(2,HIGH);
    analogWrite(5,250);
    digitalWrite(4,HIGH);
  }else if(x%3 == 1){
    analogWrite(3,135);
    digitalWrite(2,HIGH);
    analogWrite(5,135);
    digitalWrite(4,HIGH);
  }else if(x%6 == 2){
    analogWrite(3,0);
    digitalWrite(2,LOW);
    analogWrite(5,0);
    digitalWrite(4,LOW);
  }else if(x%6 == 3){
    analogWrite(3,120);
    digitalWrite(2,LOW);
    analogWrite(5,120);
    digitalWrite(4,LOW);
  }else if(x%6 == 4){
    analogWrite(3,240);
    digitalWrite(2,LOW);
    analogWrite(5,240);
    digitalWrite(4,LOW);
  }else if(x%6 == 5){
    analogWrite(3,0);
    digitalWrite(2,LOW);
    analogWrite(5,0);
    digitalWrite(4,LOW);
  }
  delay(200);*/
  //x++;
}


