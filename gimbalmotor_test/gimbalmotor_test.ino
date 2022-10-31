/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/
#include <Servo.h>
//#include <TPC.h>

// Motor ESC 
int escPin = 14;
//// these were from before we attached ESC with bounds
//int minSpeed = 60;  //0;
//int maxSpeed = 120; //180;
// these are us trying to set the bounds of the ESC ourselves
int lowThrottle = 20;
int minSpeed = -60;  //0;
int maxSpeed = -120; //180;
int motorSpeed = minSpeed;
int motorSpeedChange = 5;

int motorDelayMs = 500;

int minPwm = 544;
int maxPwm = 1800;

Servo ESC;

//int pinPot = 27;
//int minIn = 0;
//int maxIn = 4095;  //50000;
//int potValue = 0;

void setup() {
//  ESC.attach(escPin, 0, 255);
//  ESC.attach(escPin, minPWM, maxPWM);
  Serial.begin(112500);

  ESC.attach(escPin);
  ESC.write(20);
//  ESC.attach(escPin, minPwm, maxPwm);
//  ESC.write(lowThrottle);
  Serial.println("attach now! Waiting for ESC initialization");
  for (int i=30; i>0; i--) {
    Serial.print("resuming in "); Serial.println(i);
    delay(1000);
  }

//  ESC.write(0);


  // testing
  Serial.print("Testing ESC + Gimbal Motor...");

  // increase
  for(motorSpeed = 0; motorSpeed <= minSpeed; motorSpeed += 3) { //Cycles speed up to 50% power for 1 second each step
    ESC.write(motorSpeed); //Creates variable for speed to be used in in for loop
    delay(motorDelayMs);
    Serial.print("motorSpeed="); Serial.println(motorSpeed);

  }

  // decrease
  for(motorSpeed = 0; motorSpeed >= minSpeed; motorSpeed -= 5) { //Cycles speed up to 50% power for 1 second each step
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

//  // increment
//  for(motorSpeed = minSpeed; motorSpeed <= maxSpeed; motorSpeed += motorSpeedChange) { //Cycles speed up to 50% power for 1 second each step
//    ESC.write(motorSpeed); //Creates variable for speed to be used in in for loop
//    delay(motorDelayMs);
//    Serial.print("motorSpeed="); Serial.println(motorSpeed);
//
//  }
//  
//  delay(2000); //Stays on for 4 seconds
//  
//  for(motorSpeed = maxSpeed; motorSpeed > minSpeed; motorSpeed -= motorSpeedChange) { // Cycles speed down to 0% power for 1 second
//    ESC.write(motorSpeed); //Creates variable for speed to be used in in for loop
//    delay(motorDelayMs);
//    Serial.print("motorSpeed="); Serial.println(motorSpeed);
//  }
//
//  ESC.write(0);

  // decrement
  for(motorSpeed = minSpeed; motorSpeed >= maxSpeed; motorSpeed -= motorSpeedChange) { //Cycles speed up to 50% power for 1 second each step
    ESC.write(motorSpeed); //Creates variable for speed to be used in in for loop
    delay(motorDelayMs);
    Serial.print("motorSpeed="); Serial.println(motorSpeed);

  }
  
  delay(2000); //Stays on for 4 seconds
  
  for(motorSpeed = maxSpeed; motorSpeed < minSpeed; motorSpeed += motorSpeedChange) { // Cycles speed down to 0% power for 1 second
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
