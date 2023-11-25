// Import relevant libraries
#include <Adafruit_MotorShield.h>
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
DFRobot_VL53L0X TOFsensor;

#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotorL = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorR = AFMS.getMotor(2);


// Define all relevant variables
int USsensor = A0;
int LineM = 4;
int LineR = 7;
int LineL = 8;
int StartButton = 1;
int LEDG = 1;
int LEDB = 1;
int LEDW = 1;
int Magnetsens = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  TOFsensor.begin(0x50);
  TOFsensor.setMode(TOFsensor.eContinuous,TOFsensor.eHigh);

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
}
// speed = speed
// WaDist = distance to the wall in front
//BlDist = distance to the block in front
//USsens = the output of the ultra sonic sensor
// TOFsens = output of the Time Of Flight sensor
float speed, WaDist, BlDist, USsens, TOFsens;
int x;

// Go to the junction of START, facing north (from the very intial starting position)
void GoToWJunc() {
  while (1) {

    myMotorR->run(FORWARD);
    myMotorL->run(FORWARD);
    myMotorR->setSpeed(160);
    myMotorL->setSpeed(160);

    
    if (digitalRead(LineM) == HIGH) { // this statement checks to see if a junction has been reached - then stops it and ends while loop.
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);
      break;
    }                                             // robot should now be at the white zone junction.
  }
}

// Rotate 180 degrees
void Rotate180() {
  myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(160);
  myMotorL->setSpeed(160);
  delay(3000);                  // this delay will need to be worked out until roughly 180* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);
}

// Go to white junction to center of the START, pointing south
void GoToCentW() {
  myMotorR->run(FORWARD);
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(160);
  myMotorL->setSpeed(160);
  delay(1000);
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of white area
}

// Follow the line
void LineFollow() { 
  while (1) {

    if (digitalRead(LineM) == HIGH && digitalRead(LineL) == LOW && digitalRead(LineR) == LOW) { // is the robot on the line
      speed = 170;
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(speed);
      myMotorL->setSpeed(speed);
    }
    if (digitalRead(LineM) == LOW && digitalRead(LineL) == HIGH && digitalRead(LineR) == LOW) { // has the robot drifted to the left
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(180);
      myMotorL->setSpeed(160);
    }
    if (digitalRead(LineM) == LOW && digitalRead(LineL) == LOW && digitalRead(LineR) == HIGH) { // has the robot drifted to the right
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(160);
      myMotorL->setSpeed(180);
    }
    if (digitalRead(LineM) == HIGH && (digitalRead(LineL) == HIGH || digitalRead(LineR) == HIGH)) { // this statement checks to see if a junction has been reached - then stops it and ends while loop.
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);
      break;
    }
  }
}

// Setting up initial features of the robot
void InitialMovement() { 
  myMotorR->run(FORWARD);
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(160);
  myMotorL->setSpeed(160);    //  all this bit of code does is make sure the robot is slightly off the junction before resuming to line following code.
  delay(500);
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);
}

// Go to red from the centre of START when facing south
void GoToRed() { 
  //Take back to Red
      myMotorR->run(BACKWARD);       // rotating the robot in place, slowly, as to not lose the cube
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(160);
      myMotorL->setSpeed(160);
      delay(1500);                  // this delay will need to be worked out until roughly 90* is achieved
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);

      delay(1000);

      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);           // travel forwards to red area
      myMotorR->setSpeed(180);
      myMotorL->setSpeed(180);
      delay(3000);                           // change delay to work out best distance
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);                                      // robot now in center of red area  ------ hopefully...

      delay(1000);

      myMotorR->run(BACKWARD);
      myMotorL->run(BACKWARD);
      myMotorR->setSpeed(180);
      myMotorL->setSpeed(180);
      delay(3000);                           // change delay to work out best distance
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);                                      // robot now in center of white area  ------ hopefully...

      delay(1000);

      myMotorR->run(BACKWARD);       // rotating the robot in place
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(160);
      myMotorL->setSpeed(160);
      delay(1500);                  // this delay will need to be worked out until roughly 90* is achieved
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);

      delay(1000);
}

//Go to green from the centre of START when facing south
void GoToGreen() {
  myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(160);
  myMotorL->setSpeed(160);
  delay(1500);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);    

  delay(1000);

  myMotorR->run(FORWARD);       // travel forwards to green
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(180);
  myMotorL->setSpeed(180);
  delay(3000);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of green area  ------ hopefully...

  delay(1000);

  myMotorR->run(BACKWARD);
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(180);
  myMotorL->setSpeed(180);
  delay(3000);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of white area  ------ hopefully...

  delay(1000);

  myMotorR->run(FORWARD);       // roating the robot in place
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(160);
  myMotorL->setSpeed(160);
  delay(1500);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);

  delay(1000);
}

// Start the main loop
void loop() {
  //drive forward with line following corrections until junction reached.
  GoToWJunc();

  delay(1000);

  // Turn on TOF sensor to see if there is Block ahead.
  TOFsensor.start();
  delay(100);
  BlDist = TOFsensor.getDistance();
  //if block is within 60cm it must be at next junction
  if (BlDist <= 600) {

    InitialMovement();
    LineFollow();                                                                                
 
    Rotate180();

    delay(1000);
    
    InitialMovement();
    LineFollow();

    GoToCentW();


    if (digitalRead(Magnetsens) == HIGH) {
      GoToGreen();
    }    

    else {
      GoToRed();
    }

    GoToWJunc();

  }                                             //---------------------------------------  robot should have done first block, ready to restart from white zone node --------------------------------------

  InitialMovement();
  LineFollow();                                             //robot now at node F

  // Turn on TOF sensor to see if there is Block ahead.
  TOFsensor.start();
  delay(100);
  BlDist = TOFsensor.getDistance();
  //if block is within 60cm it must be at next junction
  if (BlDist <= 600) {

    InitialMovement();
    LineFollow();                                                                                 
 
    Rotate180();

    delay(1000);

    InitialMovement();
    LineFollow();                                                          // robot is now back at junction F

    InitialMovement();
    LineFollow();                                                          // robot is now back at white zone junction

    GoToCentW();

    if (digitalRead(Magnetsens) == HIGH) {
      GoToGreen();
    }    

    else {
      GoToRed();
    }

    GoToWJunc();

    }

    InitialMovement();
    LineFollow();                                                          // robot is now back at junction F
}                              //------------------------------------------------------------------------------------Stage One Complete-------------------------------------------------------------------------


