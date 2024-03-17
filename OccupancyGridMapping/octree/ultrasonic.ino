#include <Servo.h>

Servo myservo;
int servoPin = 9;

int trigPin = 6;
int echoPin = 7;
int angle = 0;
int angleOp = 1;

void setup() {
  // put your setup code here, to run once:

  myservo.attach(servoPin);
  Serial.begin(9600);
  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  long duration, distance;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration = pulseIn(echoPin,HIGH);
  distance = ((float)(340 * duration) / 1000) / 2;

  //Serial.print("distance:");
  Serial.print(distance);
  Serial.print(",");
  Serial.print(angle);
  Serial.println("\n");  
  delay(50);

  myservo.write(angle);
  angle = angle + angleOp;
  if (angleOp > 0) 
  {
    if (angle > 180) 
    {
      angle = 180;
      angleOp = -1;
    }
  } else {
    if (angle < 0) {
      angle = 0;
      angleOp = 1;
    }
  }
  delay(10);
}
