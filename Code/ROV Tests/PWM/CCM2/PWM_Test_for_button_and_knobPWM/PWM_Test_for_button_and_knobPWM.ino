#define m1A 2
#define m1F 27
#define m1B 29
#define m2A 3
#define m2F 31
#define m2B 33
#define m3A 4
#define m3F 35
#define m3B 37
#define m4A 5
#define m4F 39
#define m4B 41
#define m5A 6
#define m5F 43
#define m5B 45
#define m6A 7
#define m6F 47
#define m6B 49

int total = 0;
boolean forward = true;
void setup() {
  // put your setup code here, to run once:
  pinMode(m1A,OUTPUT);
  pinMode(m1F,OUTPUT);
  pinMode(m1B,OUTPUT);
  pinMode(m2A,OUTPUT);
  pinMode(m2F,OUTPUT);
  pinMode(m2B,OUTPUT);
  pinMode(m3A,OUTPUT);
  pinMode(m3F,OUTPUT);
  pinMode(m3B,OUTPUT);
  pinMode(m4A,OUTPUT);
  pinMode(m4F,OUTPUT);
  pinMode(m4B,OUTPUT);
  pinMode(m5A,OUTPUT);
  pinMode(m5F,OUTPUT);
  pinMode(m5B,OUTPUT);
  pinMode(m6A,OUTPUT);
  pinMode(m6F,OUTPUT);
  pinMode(m6B,OUTPUT);  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(total < 255){
    if(forward == true){
      digitalWrite(m1F,HIGH);
      digitalWrite(m1B,LOW);
      analogWrite(m1A,total);
      digitalWrite(m2F,HIGH);
      digitalWrite(m2B,LOW);
      analogWrite(m2A,total);
      digitalWrite(m3F,HIGH);
      digitalWrite(m3B,LOW);
      analogWrite(m3A,total);
      digitalWrite(m4F,HIGH);
      digitalWrite(m4B,LOW);
      analogWrite(m4A,total);
      digitalWrite(m5F,HIGH);
      digitalWrite(m5B,LOW);
      analogWrite(m5A,total);
      digitalWrite(m6F,HIGH);
      digitalWrite(m6B,LOW);
      analogWrite(m6A,total);
    }else{
      digitalWrite(m1F,LOW);
      digitalWrite(m1B,HIGH);
      analogWrite(m1A,total);
      digitalWrite(m2F,LOW);
      digitalWrite(m2B,HIGH);
      analogWrite(m2A,total);
      digitalWrite(m3F,LOW);
      digitalWrite(m3B,HIGH);
      analogWrite(m3A,total);
      digitalWrite(m4F,HIGH);
      digitalWrite(m4B,LOW);
      analogWrite(m4A,total);
      digitalWrite(m5F,LOW);
      digitalWrite(m5B,HIGH);
      analogWrite(m5A,total);
      digitalWrite(m6F,LOW);
      digitalWrite(m6B,HIGH);
      analogWrite(m6A,total);
    }
    total+=10;
  }else{
    total = 0;
    if(forward == true){
      forward = false;
    }else{
      forward = true;
    }
  }
  delay(100);
}
