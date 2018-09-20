#include <Wire.h> 
#include <Servo.h>
Servo testServo;
int btnOpen = 4;
int btnClose = 5;
int gripMinAngle = 110;
int gripMaxAngle = 175;
int curPos = 0;

void setup() {
  Wire.begin(9); 
  Serial.begin(4800);
  testServo.attach(6); 
  testServo.write(120);
  Wire.onReceive(outputTemperature);
}
void outputTemperature(int bytes) {
  String s = "";
  char x;
  while(Wire.available()){
    x = Wire.read();
    s += x;
  }
  String sT = "";
  String SP = "";
  boolean p = false;
  for(i=0;i<11;i++){
    if(s.substring(i,i+1).equals("p")){
      p = true;
    }
    if(p==true){
      sT += s.substring(i,i+1);
    }else{
      sR += s.substring(i,i+1);
    }
  }
  Serial.print("Temperature : ");
  Serial.print(sT);
  Serial.println(" C");
  
  Serial.print("Depth : ");
  Serial.print(sP);
  Serial.println(" m");
}

void loop() {
  //Wierd electronic behaviour that causses low low and high high combination
  curPos = testServo.read();
  if(digitalRead(btnOpen) == LOW && digitalRead(btnClose) == HIGH){
    Serial.println(curPos);
    if(curPos<gripMaxAngle && curPos >= gripMinAngle){
      curPos++;
      testServo.write(curPos);
      /*
      Serial.print("Closing ");
      Serial.println(curPos);
      */
    }
  }else if(digitalRead(btnOpen) == HIGH && digitalRead(btnClose) == LOW){
    if(curPos<=gripMaxAngle && curPos > gripMinAngle){
      curPos--;
      testServo.write(curPos);
      /*
      Serial.print("Opening ");
      Serial.println (curPos);
      */
    }    
  }else{
    //Serial.println("Stationary at");
  }
  delay(40);
}
