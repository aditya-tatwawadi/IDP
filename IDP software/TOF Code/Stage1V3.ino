// Import relevant libraries
#include <Adafruit_MotorShield.h>
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
DFRobot_VL53L0X TOFsensor;

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
  delay(700);

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

void Rotate90CW() {
  myMotorR->run(BACKWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(MRB);
  myMotorL->setSpeed(MLF);
  delay(700);

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
  delay(700);

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
  delay(500);

  while(1){
    myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
    myMotorL->run(BACKWARD);
    myMotorR->setSpeed(MRF);
    myMotorL->setSpeed(MLB);

    if (digitalRead(LineFR)==HIGH || digitalRead(LineFL)==HIGH){
      delay(300);
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
  // 4 line sensors next to each other
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
      myMotorL->setSpeed(MLF - 100);
    }

    // drifted to the left, so turns a bit to the right to correct
    if (digitalRead(LineR) == HIGH && digitalRead(LineL) == LOW) {
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(MRF - 100);
      myMotorL->setSpeed(MLF);
    }

    // stop when junction hit:
    // 1. both inner line sensors HIGH
    // 2. either the outer left or outer right HIGH
    if ((digitalRead(LineL) == HIGH && digitalRead(LineR) == HIGH) || ((digitalRead(LineR) == HIGH || digitalRead(LineL) == HIGH) && (digitalRead(LineFL) == HIGH || digitalRead(LineFR) == HIGH))) {
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
  delay(300);
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);
}

// Go to green from the centre of START when facing south
void GoToGreen() { 
  //Take back to green
  myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLB);
  delay(650);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);

  delay(1000);

  myMotorR->run(FORWARD);
  myMotorL->run(FORWARD);           // travel forwards to green area
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLF);
  delay(3500);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of green area  ------ hopefully...

  delay(1000);

  myMotorR->run(BACKWARD);
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRB);
  myMotorL->setSpeed(MLB);
  delay(3500);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of white area  ------ hopefully...

  delay(1000);

  myMotorR->run(FORWARD);       // rotating the robot in place
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLB);
  delay(650);                  // this delay will need to be worked out until roughly 90* is achieved
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
  delay(650);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);    

  delay(1000);

  myMotorR->run(FORWARD);       // travel forwards to red
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(MRF);
  myMotorL->setSpeed(MLF);
  delay(3500);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of red area  ------ hopefully...

  delay(1000);

  myMotorR->run(BACKWARD);
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(MRB);
  myMotorL->setSpeed(MLB);
  delay(3500);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of white area  ------ hopefully...

  delay(1000);

  myMotorR->run(BACKWARD);       // roating the robot in place
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(MRB);
  myMotorL->setSpeed(MLF);
  delay(650);                  // this delay will need to be worked out until roughly 90* is achieved
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

  TOFsensor.start();
  delay(100);
  BlDist = TOFsensor.getDistance();

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
      GoToGreen();
    } else {
      GoToRed();
    }

    delay(1000); // TO BE DELETED LATER - FOR TESTING
    GoToWJunc();
  }

  if (BlDist <= 1200) {
    //TOFsensor.stop();
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // Start line -> F
    LineFollow();                                     
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // F -> E
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

    // F -> Start line
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    if (digitalRead(Magnetsens) == HIGH) {
      GoToGreen();
    } else {
      GoToRed();
    }

    delay(1000); // TO BE DELETED LATER - FOR TESTING
    GoToWJunc();
  }
}

// This is to test values for motor speeds only
void loop() {
  delay(1000);
  LineFollow();
}
