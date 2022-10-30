/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/
#include <Servo.h>
#include <TPC.h>

// Motor ESC 
int escPin = 14;
int minSpeed = 60;  //0;
int maxSpeed = 120; //180;
int motorSpeed = minSpeed;
int motorSpeedChange = 10;

int motorDelayMs = 500;

Servo ESC;

//int pinPot = 27;
//int minIn = 0;
//int maxIn = 4095;  //50000;
//int potValue = 0;

void setup() {
//  ESC.attach(escPin, 0, 255);
//  ESC.attach(escPin, minPWM, maxPWM);
//  ESC.attach(escPin, 1000, 2000);
  ESC.attach(escPin);
  ESC.write(0);

  Serial.begin(112500);

  // testing
  Serial.print("Testing ESC + Gimbal Motor...");
  
  for(motorSpeed = 0; motorSpeed <= minSpeed; motorSpeed += 5) { //Cycles speed up to 50% power for 1 second each step
    ESC.write(motorSpeed); //Creates variable for speed to be used in in for loop
    delay(motorDelayMs);
    Serial.print("motorSpeed="); Serial.println(motorSpeed);

  }

  for (int i=5; i>0; i--) {
    Serial.print("starting in "); Serial.println(i);
    delay(1000);
  }
}

void loop() {

  for(motorSpeed = minSpeed; motorSpeed <= maxSpeed; motorSpeed += motorSpeedChange) { //Cycles speed up to 50% power for 1 second each step
    ESC.write(motorSpeed); //Creates variable for speed to be used in in for loop
    delay(motorDelayMs);
    Serial.print("motorSpeed="); Serial.println(motorSpeed);

  }
  
  delay(2000); //Stays on for 4 seconds
  
  for(motorSpeed = maxSpeed; motorSpeed > minSpeed; motorSpeed -= motorSpeedChange) { // Cycles speed down to 0% power for 1 second
    ESC.write(motorSpeed); //Creates variable for speed to be used in in for loop
    delay(motorDelayMs);
    Serial.print("motorSpeed="); Serial.println(motorSpeed);
  }

//  ESC.write(65);
//  Serial.println("slow");
//  delay(1000); //Turns off for 1 second
//  ESC.write(115);
//  Serial.println("fast");
  
  delay(2000);
  

//  potValue = analogRead(pinPot);
////  Serial.print(" potValue: "); Serial.print(potValue);
//  motorSpeed = map(potValue, minIn, maxIn, minSpeed, maxSpeed);
//  Serial.print(" motorSpeed: "); Serial.println(motorSpeed);
//  ESC.write(motorSpeed);
//  delay(100);
  
}
