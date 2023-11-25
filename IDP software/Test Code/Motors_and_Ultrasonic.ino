#include <Adafruit_MotorShield.h>

#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotorL = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorR = AFMS.getMotor(2);

int sensityPin = A0; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
}
}
float a, speed, dist, sensity;
void loop() {
  uint8_t i;
  sensity = analogRead(sensityPin);
  dist = sensity * MAX_RANG / ADC_SOLUTION;
  if (dist<=40){
    speed = 0;
    myMotorR->run(BACKWARD);
    myMotorR->setSpeed(speed);

    myMotorL->run(BACKWARD);
    myMotorL->setSpeed(speed);
    delay(500);
    speed = 255;
    myMotorR->run(FORWARD);
    myMotorR->setSpeed(speed);
    myMotorL->run(BACKWARD);
    myMotorL->setSpeed(speed);
    delay(1500);
    }

  else {
    speed = 255;
  }
  Serial.println(dist);
  Serial.println(speed);
  
  // put your main code here, to run repeatedly:
  myMotorR->run(BACKWARD);
  myMotorR->setSpeed(speed);

  myMotorL->run(BACKWARD);
  myMotorL->setSpeed(speed);
}

