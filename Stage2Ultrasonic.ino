//------------------------------------------Arduino Stage 2 code (ULTRASONIC)-------------------------------------------------------------------------------------
// Import relevant libraries
#include <Adafruit_MotorShield.h>
#include "Arduino.h"
#include "Wire.h"
#include <VL53L0X.h> //Using a different type of VL53L0X.h library
#define HIGH_ACCURACY
#define LONG_RANGE
VL53L0X sensor;

//Setting up the URM09 Ultrasonic Sensor 
#define MAX_RANG (520)//the max measurement value of the module is 520cm(a little bit longer
than effective max range)
#define ADC_SOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit
int sensityPin = A0; // select the input pin
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

float dist_t, sensity_t; // Set-up for the ultrasonic sensor

////////////////////////////////
////////// STAGE TWO ///////////
////////////////////////////////
void Stage2() {
  //Currently at F facing west

  // read the value from the sensor:
  sensity_t = analogRead(sensityPin);
  // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t // Set the block distance to be that measured from the ultrasonic sensor

  // If block at D (<60cm):
  if (BlDist <= 600) {

    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // F -> D
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // D -> F
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    //Now at F facing East

    Rotate90CW();//Turn 90 degrees clockwise
    delay(1000);

    //F -> start line
    InitialMovement();
    delay(1000);
    LineFollow(); //Now at start line (W-junc) facing south

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
    
    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At F facing north
    delay(1000);
    Rotate90AC(); //At F facing west
    delay(1000);
  }

  // Now at F facing west
  // read the value from the sensor to check if block at B:
  sensity_t = analogRead(sensityPin);
  // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t // Set the block distance to be that measured from the ultrasonic sensor

  //If block at B (<120cm):
  if (BLDist <=1200){
    

    // F -> D
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // D -> B
    InitialMovement();
    delay(1000);
    LineFollow(); 

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    // B -> D
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    LineFollow();
    delay(1000);

    // D -> F
    InitialMovement();
    delay(1000);
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    Rotate90CW();
    delay(1000);
    InitiaLMovement();
    delay(1000);
    LineFollow();
    delay(1000);

    //Currently at start line (W junction) facing south 
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

    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At f facing north
    delay(1000);
    Rotate90AC(); // At f facing west
  }

  Rotate180(); //At F facing east
  delay(1000);
  
  //Stage 2 ends with robot at F facing east 
}

// This is to test values for motor speeds only
void loop() {
  Stage2();
  exit(0); //Completely terminates the code

}






