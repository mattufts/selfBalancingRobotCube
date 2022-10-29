/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/
#include <Servo.h>

// Motor ESC 
int escPin = 14;
int minSpeed = 0;
int maxSpeed = 180;
int motorSpeed = minSpeed;


Servo ESC;

//int pinPot = 27;
//int minIn = 0;
//int maxIn = 4095;  //50000;
//int potValue = 0;

void setup() {
//  ESC.attach(escPin, 0, 255);
//  ESC.attach(escPin, minPWM, maxPWM);
  ESC.attach(escPin);
  ESC.write(0);


  Serial.begin(112500);

  // testing
  Serial.print("Testing ESC + Gimbal Motor...");
  
}

void loop() {

  for(motorSpeed = minSpeed; motorSpeed <= maxSpeed; motorSpeed += 5) { //Cycles speed up to 50% power for 1 second each step
    ESC.write(motorSpeed); //Creates variable for speed to be used in in for loop
    delay(500);
    Serial.print("motorSpeed="); Serial.println(motorSpeed);

  }
  
  delay(4000); //Stays on for 4 seconds
  for(motorSpeed = maxSpeed; motorSpeed > minSpeed; motorSpeed -= 5) { // Cycles speed down to 0% power for 1 second
    ESC.write(motorSpeed); //Creates variable for speed to be used in in for loop
    delay(500 );
    Serial.print("motorSpeed="); Serial.println(motorSpeed);
  }

  ESC.write(60);
//  Serial.println("off");
  delay(500); //Turns off for 1 second


//  potValue = analogRead(pinPot);
////  Serial.print(" potValue: "); Serial.print(potValue);
//  motorSpeed = map(potValue, minIn, maxIn, minSpeed, maxSpeed);
//  Serial.print(" motorSpeed: "); Serial.println(motorSpeed);
//  ESC.write(motorSpeed);
//  delay(100);
  
}
