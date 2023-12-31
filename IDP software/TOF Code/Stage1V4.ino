// Import relevant libraries
#include <Adafruit_MotorShield.h>
#include "Arduino.h"
#include "Wire.h"
#include <VL53L0X.h> //Using a different type of VL53L0X.h library
#define HIGH_ACCURACY
#define LONG_RANGE
VL53L0X sensor;

#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotorR = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorL = AFMS.getMotor(2);

// Define all relevant variables
int USsensor = A0;
int LineFR = 9; //Far Right Line Sensor
int LineR = 8;
int LineL = 7;
int LineFL = 4; //Far Left Line Sensor
int StartButton = 1;
int LEDG = 1;
int LEDB = 1;
int LEDW = 1;
int Magnetsens = 11; // Hall effect sensor


// Motor variables - each motor requires different params to get working properly (and for both directions)
int MLF = 255;
int MLB = 255;

int MRF = 255;
int MRB = 255;

int delay90AC = 700;
int delay90CW = 700;
int delay180_1 = 700;
int delay180_2 = 700;

void setup() {
  // Relevant to TOF sensor:
  Serial.begin(9600);
  Wire.begin();
  //TOFsensor.begin(0x50);
  //TOFsensor.setMode(TOFsensor.eContinuous,TOFsensor.eHigh);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();


  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
}

// Defining what all the relevant variables mean
// speed = speed
// WaDist = distance to the wall in front
// BlDist = distance to the block in front
// USsens = the output of the ultra sonic sensor
// TOFsens = output of the Time Of Flight sensor
float speed, WaDist, BlDist, USsens, TOFsens;
int x;

// Rotate 90 degrees anticlockwise
void Rotate90AC() {
  myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLB);
  delay(delay90AC);

  while(1){
    myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
    myMotorL->run(BACKWARD);
    myMotorR->setSpeed(MRF);
    myMotorL->setSpeed(MLB);

    if (digitalRead(LineFR)==HIGH || digitalRead(LineFL)==HIGH) {
      myMotorR->run(RELEASE);
      myMotorL->run(RELEASE);
      break;
    }
  }
}

// Rotate 90 degrees clockwise
void Rotate90CW() {
  myMotorR->run(BACKWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(MRB);
  myMotorL->setSpeed(MLF);
  delay(delay90CW);

  while(1){
    myMotorR->run(BACKWARD);       // rotating the robot in place, slowly, as to not lose the cube
    myMotorL->run(FORWARD);
    myMotorR->setSpeed(MRB);
    myMotorL->setSpeed(MLF);

    if (digitalRead(LineFR)==HIGH || digitalRead(LineFL)==HIGH){
      myMotorR->run(RELEASE);
      myMotorL->run(RELEASE);
      break;
    }
  }
}

// Rotate 180 degrees 
void Rotate180(){
  myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLB);
  delay(delay180_1);

  while(1){
    myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
    myMotorL->run(BACKWARD);
    myMotorR->setSpeed(MRF);
    myMotorL->setSpeed(MLB);

    if (digitalRead(LineFR)==HIGH || digitalRead(LineFL)==HIGH){
      myMotorR->run(RELEASE);
      myMotorL->run(RELEASE);
      break;
    }
  }

  myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLB);
  delay(delay180_2);

  while(1){
    myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
    myMotorL->run(BACKWARD);
    myMotorR->setSpeed(MRF);
    myMotorL->setSpeed(MLB);

    if (digitalRead(LineFR)==HIGH || digitalRead(LineFL)==HIGH){
      delay(150);
      myMotorR->run(RELEASE);
      myMotorL->run(RELEASE);
      break;
    }
  } 
}

// Go to the junction of START, facing north (from the very intial starting position)
void GoToWJunc() {
  while (1) {
    myMotorR->run(FORWARD);
    myMotorL->run(FORWARD);
    myMotorR->setSpeed(MRF);
    myMotorL->setSpeed(MLF);

    if (digitalRead(LineFR) == HIGH || digitalRead(LineFL) == HIGH) { // this statement checks to see if a line has been reached - then stops it and ends while loop. Right or left sensor checked
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);
      break;
    } // robot should now be at the white zone junction.
  }
}

// Go to white junction to center of the START, pointing south
void GoToCentW() {
  myMotorR->run(FORWARD);
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLF);
  delay(1000);
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0); // robot now in center of white area
}

// Follow the line
void LineFollow() { 
  // 4 line sensors next to each other - sensing when both high
  while (1) {

    // goes forward
    if (digitalRead(LineR) == HIGH && digitalRead(LineL) == HIGH) {
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(MRF);
      myMotorL->setSpeed(MLF);
    }

    // drifted to the right, so turns a bit to the left to correct
    if (digitalRead(LineR) == LOW && digitalRead(LineL) == HIGH) {
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(MRF);
      myMotorL->setSpeed(MLF - 50);
    }

    // drifted to the left, so turns a bit to the right to correct
    if (digitalRead(LineR) == HIGH && digitalRead(LineL) == LOW) {
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(MRF - 50);
      myMotorL->setSpeed(MLF);
    }

    // stop when junction hit:
    // 1. both inner line sensors HIGH AND
    // 2. either the outer left or outer right HIGH
    if ((digitalRead(LineL) == HIGH || digitalRead(LineR) == HIGH) && (digitalRead(LineFL) == HIGH || digitalRead(LineFR) == HIGH)) {
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);
      break;
    }
  }
}

// Setting up initial features of the robot
void InitialMovement() {  
  //  all this bit of code does is make sure the robot is slightly off the junction before resuming to line following code.
  myMotorR->run(FORWARD);
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLF);    
  delay(250);
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);
}

// Go to green from the centre of START when facing south
void GoToGreen() { //Green means that it is not magnetic
  //Take back to green
  myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLB);
  delay(770);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);

  delay(1000);

  myMotorR->run(FORWARD);
  myMotorL->run(FORWARD);           // travel forwards to green area
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLF);
  delay(4000);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of green area  ------ hopefully...

  delay(1000);

  myMotorR->run(BACKWARD);
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRB);
  myMotorL->setSpeed(MLB);
  delay(4000);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of white area  ------ hopefully...

  delay(1000);

  myMotorR->run(FORWARD);       // rotating the robot in place
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLB);
  delay(770);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);

  delay(1000);
}

//Go to red from the centre of START when facing south
void GoToRed() {
  myMotorR->run(BACKWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(MRB);
  myMotorL->setSpeed(MLF);
  delay(770);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);    

  delay(1000);

  myMotorR->run(FORWARD);       // travel forwards to red
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLF);
  delay(4000);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of red area  ------ hopefully...

  delay(1000);

  myMotorR->run(BACKWARD);
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRB);
  myMotorL->setSpeed(MLB);
  delay(4000);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of white area  ------ hopefully...

  delay(1000);

  myMotorR->run(BACKWARD);       // roating the robot in place
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(MRB);
  myMotorL->setSpeed(MLF);
  delay(770);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);

  delay(1000);
}

////////////////////////////////
////////// STAGE ONE ///////////
////////////////////////////////
void Stage1() {
  // Drive forward with line following corrections until junction reached.
  GoToWJunc();
  delay(1000);

  BlDist = sensor.readRangeContinuousMillimeters();

  // If block at F (<60cm):
  if (BlDist <= 600) {
    //TOFsensor.stop();
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // Start line -> F
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // F -> Start line
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // 
    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

    delay(1000); // TO BE DELETED LATER - FOR TESTING
    GoToWJunc(); // At W junction facing north

  }
  // Now go to F and face north
  InitialMovement();
  LineFollow();

  delay(1000);
  //Take another distance
  BlDist = sensor.readRangeContinuousMillimeters();
  // Checks if there is a block at E
  if (BlDist <= 600) {
    //TOFsensor.stop();
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // F to E
    LineFollow();                                     
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // E -> F
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    InitialMovement();
    // F -> Start line
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    } else {
      GoToGreen();
    }

    delay(1000); // TO BE DELETED LATER - FOR TESTING
    GoToWJunc(); //Go to W junc and face north

    InitialMovement();
    LineFollow(); // Back to F facing North
  }
  Rotate90AC();
}

// This is to test values for motor speeds only
void loop() {
  Stage1();
  exit(0); //Completely terminates the code

}
