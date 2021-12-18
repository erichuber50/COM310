#include <SharpIR.h>
#include "Arduino.h"
#include "Servo.h"
#define model 1080

#define frontIRpin A0
#define backIRpin A1


Servo servoLeft;
Servo servoRight;

const int pingPin1 = 3;
const int pingPin2 = 2;
const int servoRightPin = 5;
const int servoLeftPin = 7;


SharpIR backIR = SharpIR(backIRpin, model);
SharpIR frontIR = SharpIR(frontIRpin, model);

int distance_cm_front;
int distance_cm_back;

int ObstacleFront;
int ObstacleBack;

void setup() {
  Serial.begin(9600);
  servoLeft.attach(servoLeftPin);
  servoRight.attach(servoRightPin);
  
}

void loop() {
  long duration1, duration2, inches1, inches2, cm1, cm2;

  //distance_cm_back = backIR.distance();

  //distance_cm_front = frontIR.distance();

  /*

  float volts_back = analogRead(backIRpin)*0.0048828125;
  float volts_front = analogRead(frontIRpin)*0.0048828125;

  int distance_back = 13*pow(volts_back, -1);
  int distance_front = 13*pow(volts_front, -1);
  
  Serial.print("Front distance: ");
  Serial.println(distance_front);

  Serial.print("Back distance: ");
  Serial.println(distance_back);

  if (distance_back > 200 or distance_front > 200){
    Serial.println("LINE");
  }
  */

  pinMode(pingPin1, OUTPUT);
  digitalWrite(pingPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin1, HIGH);
  delayMicroseconds(5); //10?
  digitalWrite(pingPin1, LOW);

  pinMode(pingPin1, INPUT);
  duration1 = pulseIn(pingPin1, HIGH);

  pinMode(pingPin2, OUTPUT);
  digitalWrite(pingPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin2, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin2, LOW);
  
  pinMode(pingPin2, INPUT);
  duration2 = pulseIn(pingPin2, HIGH);

  inches1 = microsecondsToInches(duration1);
  cm1 = microsecondsToCentimeters(duration1);

  inches2 = microsecondsToInches(duration2);
  cm2 = microsecondsToCentimeters(duration2);

  Serial.print(inches1);
  Serial.print("in, ");
  Serial.print(cm1);
  Serial.print("cm ");

  Serial.print(inches2);
  Serial.print("in, ");
  Serial.print(cm2);
  Serial.print("cm");
  
  Serial.println();

 
  //DELETE
  //servoLeft.attach(servoLeftPin);
  //servoRight.attach(servoRightPin);

  //ADD
  servoLeft.write(90);
  servoRight.write(90);
  
  if (inches1 > 0 or inches2 > 0){
    if (inches1 < 24 and inches2 < 24){
      Serial.println("forward");
      servoLeft.write(180);  // 2. turns servo CW in full speed. change the value in the brackets (180) to change the speed. As these numbers move closer to 90, the servo will move slower in that direction.
      servoRight.write(0);
    }
    else if (inches2 > 24 and inches1 < 24){
      Serial.println("left");
      servoLeft.write(90);
      servoRight.write(0);
    }
    else if (inches1 > 24 and inches2 < 24){
      Serial.println("right");
      servoRight.write(90);
      servoLeft.write(180);
    }
    else if (inches1 > 24 and inches2 > 24){
      Serial.println("rotate");
      servoLeft.write(100);
      servoRight.write(90);
    }
  }
  else{
    servoLeft.write(90);
    servoRight.write(90);
  }
  delay(100);
 
}

long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
