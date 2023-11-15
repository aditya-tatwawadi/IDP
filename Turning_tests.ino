#include <Adafruit_MotorShield.h>
#include "Arduino.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotorL = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorR = AFMS.getMotor(2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
if (!AFMS.begin()) {
  Serial.println("Could not find Motor Shield. Check wiring.");
  while (1);
}

}

void loop() {
  // put your main code here, to run repeatedly:
  myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(160);
  myMotorL->setSpeed(160);
  delay(1930);                  // this delay will need to be worked out until roughly 180* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);

  delay(3000);

  myMotorR->run(BACKWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(160);
  myMotorL->setSpeed(160);
  delay(890);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);

  delay(3000);

  myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(160);
  myMotorL->setSpeed(160);
  delay(890);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);

  delay(3000);



}
