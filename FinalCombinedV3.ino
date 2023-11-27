//------------------------------------------ARDUINO CODE TO PICK UP BOTTOM 2 BLOCKS-------------------------------------------------------------------------------------
// TODO Monday 27th - test out turning with new line sensors mount
// test out delays/ change any delay values
// change motor values
// DONE - Make sure the robot stays in the start box or a fixed duration - see idp markscheme
// Make sure LEDs light up when box is detected - see idp markscheme

// Import relevant libraries
#include <Adafruit_MotorShield.h>
#include "Arduino.h"
#include "Wire.h"

// relevant defines
#define HIGH_ACCURACY
#define LONG_RANGE
#define MAX_RANG (520)//Setting up the URM09 Ultrasonic Sensor 
#define ADC_SOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotorR = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorL = AFMS.getMotor(2);

// Define all relevant variables
int sensityPin = A0; // setting up the input pin for the Ultrasonic sensor
int LineFR = 9; //Far Right Line Sensor
int LineR = 8;
int LineL = 7;
int LineFL = 4; //Far Left Line Sensor
int StartButton = 6;
int LEDR = 3;
int LEDB = 10;
int LEDW = 2;
int Magnetsens = 11; // Hall effect sensor

// Motor variables - each motor requires different params to get working properly (and for both directions)
int MLF = 255;
int MLB = 255;
int MRF = 255;
int MRB = 255;

// Delay variables for turning
int delay90AC = 700;
int delay90CW = 700;
int delay180_1 = 700;
int delay180_2 = 700;


// Defining what all the relevant variables mean
// speed = speed
// WaDist = distance to the wall in front
// BlDist = distance to the block in front
// USsens = the output of the ultra sonic sensor
// TOFsens = output of the Time Of Flight sensor
float speed, WaDist, BlDist, USsens, TOFsens;
float dist_t, sensity_t; //Set up for the ultrasonic sensor

void setup() {

  Serial.begin(9600);
  AFMS.begin(); 
  Wire.begin();
  pinMode(StartButton, INPUT);
  pinMode(LineFL, INPUT);
  pinMode(LineL, INPUT);
  pinMode(LineR, INPUT);
  pinMode(LineFR, INPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDB, OUTPUT); 
  pinMode(LEDW, OUTPUT); 
  pinMode(Magnetsens, INPUT);
 
}

// Function to check if button has been pushed
bool checkButtonPress(){
 int val = digitalRead(StartButton); // read input value
 if (val == HIGH) { // check if the input is HIGH
 return true;
 }
 else {
 return false;
  }
}

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
    if (digitalRead(LineR) == LOW && digitalRead(LineL) == LOW) {
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(MRF);
      myMotorL->setSpeed(MLF);
    }

    // drifted to the right, so turns a bit to the left to correct
    if (digitalRead(LineR) == LOW && digitalRead(LineL) == HIGH) {
      myMotorR->run(FORWARD);
      myMotorL->run(BACKWARD);
      myMotorR->setSpeed(MRF);
      myMotorL->setSpeed(MLF);
    }

    // drifted to the left, so turns a bit to the right to correct
    if (digitalRead(LineR) == HIGH && digitalRead(LineL) == LOW) {
      myMotorR->run(BACKWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(MRF);
      myMotorL->setSpeed(MLF);
    }

    // stop when junction hit:
    // 1. both inner line sensors HIGH AND
    // 2. either the outer left or outer right HIGH
    if (digitalRead(LineFL) == HIGH || digitalRead(LineFR) == HIGH) {
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

  //delay as per markscheme to wait at least 5 seconds
  delay(6000);

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

  //delay as per markscheme to wait at least 5 seconds
  delay(6000);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////// STAGE ONE ////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Stage1() {
  // Drive forward with line following corrections until junction reached.
  GoToWJunc();
  delay(1000);

  // read the value from the sensor:
  sensity_t = analogRead(sensityPin);
  // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t; // Set the block distance to be that measured from the ultrasonic sensor

  //Currently at W junc facing north
  // If block at F (<60cm):
  if (BlDist <= 60) {

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

    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    //delay as per markscheme to wait 5 seconds
    delay(5000);

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

    GoToWJunc(); // At W junction facing north

  }

  // read the value from the sensor:
  sensity_t = analogRead(sensityPin);
  // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t; // Set the block distance to be that measured from the ultrasonic sensor

  //If block at E (<120cm):
  if (BlDist <=120){
    
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // Start line -> F
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // F ->E
    InitialMovement();
    delay(1000);
    LineFollow(); 

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // E -> F
    LineFollow();
    delay(1000);

    InitialMovement();
    delay(1000);

    // F -> Start line
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    //delay 5 seconds as per markscheme
    delay(5000);

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

    GoToWJunc(); // At W junction facing north
  }

  // Now go to F and face west
  InitialMovement();
  delay(1000);
  LineFollow();

  delay(1000);
  
  Rotate90AC(); //Finish facing west at F
  //Stage 1 ends with robot at F facing west
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////// STAGE TWO ////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Stage2() {
  //Currently at F facing west

  // read the value from the sensor:
  sensity_t = analogRead(sensityPin);
  // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t; // Set the block distance to be that measured from the ultrasonic sensor

  // If block at D (<60cm):
  if (BlDist <= 60) {

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

    //delay 5 seconds as per markscheme
    delay(5000);

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

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
  BlDist = dist_t; // Set the block distance to be that measured from the ultrasonic sensor

  //If block at B (<120cm):
  if (BlDist <=120){
    

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
    InitialMovement();
    delay(1000);
    LineFollow();
    delay(1000);

    //Currently at start line (W junction) facing south 
    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    //delay 5 seconds as per markscheme
    delay(5000);

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////// STAGE THREE //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Stage3() {
  //Currently at F facing east

  // read the value from the sensor:
  sensity_t = analogRead(sensityPin);
  // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t; // Set the block distance to be that measured from the ultrasonic sensor

  // If block at H (<60cm):
  if (BlDist <= 60) {

    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // F -> H
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // H -> F
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    //Now at F facing west

    Rotate90AC();//Turn 90 degrees anticlockwise
    delay(1000);

    //F -> start line
    InitialMovement();
    delay(1000);
    LineFollow(); //Now at start line (W-junc) facing south

    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    //delay 5 seconds as per markscheme
    delay(5000);

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

    GoToWJunc(); // At W junction facing north
    
    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At F facing north
    delay(1000);
    Rotate90CW(); //At F facing east
    delay(1000);
  }

  // Now at F facing east
  // read the value from the sensor to check if block at J:
  sensity_t = analogRead(sensityPin);
  // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t; // Set the block distance to be that measured from the ultrasonic sensor

  //If block at J (<120cm):
  if (BlDist <=120){
    

    // F -> H
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // H -> J
    InitialMovement();
    delay(1000);
    LineFollow(); 

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    // J -> H
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    LineFollow();
    delay(1000);

    // H -> F
    InitialMovement();
    delay(1000);
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    Rotate90AC();
    delay(1000);
    InitialMovement();
    LineFollow();

    //Currently at start line (W junction) facing south 
    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    //delay 5 seconds as per markscheme
    delay(5000);

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

    GoToWJunc(); // At W junction facing north

    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At f facing north
    delay(1000);
    Rotate90CW(); // At f facing east
  }

  Rotate90AC(); //At F facing north
  delay(1000);

  // F to E
  InitialMovement();
  delay(1000);
  LineFollow(); 

  delay(1000);
  Rotate90AC();
  //Stage 3 ends with robot at E facing west 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////// STAGE FOUR ///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Stage4() {
  //Currently at E facing west

  // read the value from the sensor:
  sensity_t = analogRead(sensityPin);
  // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t; // Set the block distance to be that measured from the ultrasonic sensor

  // If block at C (<60cm):
  if (BlDist <= 60) {

    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // E -> C
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // C -> E
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    //Now at E facing East

    Rotate90CW();//Turn 90 degrees clockwise
    delay(1000);

    //E to F
    InitialMovement();
    delay(1000);
    LineFollow(); //Now at start line (W-junc) facing south
    delay(1000);

    //F -> start line
    InitialMovement();
    delay(1000);
    LineFollow(); //Now at start line (W-junc) facing south

    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    //delay 5 seconds as per markscheme
    delay(5000);

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

    GoToWJunc(); // At W junction facing north
    
    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At F facing north
    delay(1000);

    // F to E
    InitialMovement();
    delay(1000);
    LineFollow(); //At E facing north
    delay(1000);

    Rotate90AC(); //At E facing west
    delay(1000);
  }

  // Now at E facing west
  // read the value from the sensor to check if block at A:
  sensity_t = analogRead(sensityPin);
  // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t; // Set the block distance to be that measured from the ultrasonic sensor

  //If block at A (<120cm):
  if (BlDist <=120){
    

    // E -> C
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // C -> A
    InitialMovement();
    delay(1000);
    LineFollow(); 

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    // A -> C
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    LineFollow();
    delay(1000);

    // C -> E
    InitialMovement();
    delay(1000);
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    Rotate90CW();
    delay(1000);

    //E to F
    InitialMovement();
    delay(1000);
    LineFollow();
    delay(1000);

    //F to start line (W junc)
    InitialMovement();
    delay(1000);
    LineFollow();
    delay(1000);

    //Currently at start line (W junction) facing south 
    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    //delay 5 seconds as per markscheme
    delay(5000);

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

    GoToWJunc(); // At W junction facing north

    // Wjunc to F
    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At f facing north

    //F to E
    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At E facing north

    delay(1000);
    Rotate90AC(); // At E facing west
  }

  Rotate180(); //At E facing east
  delay(1000);
  
  //Stage 4 ends with robot at E facing east 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////// STAGE FIVE ///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Stage5() {
  //Currently at E facing east

  // read the value from the sensor:
  sensity_t = analogRead(sensityPin);
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t; // Set the block distance to be that measured from the ultrasonic sensor

  // If block at G (<60cm):
  if (BlDist <= 60) {

    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // E -> G
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // G -> E
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    //Now at E facing west

    Rotate90AC();//Turn 90 degrees anticlockwise
    delay(1000);

    //E to F
    InitialMovement();
    delay(1000);
    LineFollow(); 
    delay(1000);

    //F -> start line
    InitialMovement();
    delay(1000);
    LineFollow(); //Now at start line (W-junc) facing south

    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    //delay 5 seconds as per markscheme
    delay(5000);

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

    GoToWJunc(); // At W junction facing north
    
    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At F facing north
    
    //F to E
    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At E facing north

    delay(1000);
    Rotate90CW(); //At E facing east
    delay(1000);
  }

  // Now at E facing east
  // read the value from the sensor to check if block at I:
  sensity_t = analogRead(sensityPin);
  // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  BlDist = dist_t; // Set the block distance to be that measured from the ultrasonic sensor

  //If block at I (<120cm):
  if (BlDist <=120){
    

    // E -> G
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    // G -> I
    InitialMovement();
    delay(1000);
    LineFollow(); 

    // Collected block, so turn back
    Rotate180();
    delay(1000);
    
    // I -> G
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    LineFollow();
    delay(1000);

    // G -> E
    InitialMovement();
    delay(1000);
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    Rotate90AC();
    
    // E to F
    delay(1000);
    InitialMovement();
    LineFollow();

    // F to W
    delay(1000);
    InitialMovement();
    LineFollow();

    //Currently at start line (W junction) facing south 
    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    //delay 5 seconds as per markscheme
    delay(5000);

    if (digitalRead(Magnetsens) == HIGH) {
      GoToRed();
    }
    else {
      GoToGreen();
    }

    GoToWJunc(); // At W junction facing north

    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At F facing north

    //E to F
    delay(1000);
    InitialMovement();
    delay(1000);
    LineFollow(); //At E facing north

    delay(1000);
    Rotate90CW(); // At E facing east
  }
  //Stage 5 ends with robot at E facing west 
}




void loop() {
  
  bool buttonpush;
  bool pushed;
  //while loop checks whether button has been pushed until it has been pushed, sets 'pushed' to true if pushed

  while (buttonpush != true) {
    buttonpush = checkButtonPress();
    if (buttonpush == true){
      pushed = true;
      break;
    }
  } 

  // pushed indicates button has been pushed after reset/power on - run main code inside while loop
  while (pushed == true) {
    Serial.println("Button has been pushed, run main function");
    
    // Start stage 1
    Stage1();

    // Start stage 2
    Stage2();

    // Start stage 3
    Stage3();

    // Start stage 4
    Stage4();

    // Start stage 5
    Stage5();

  }

//  exit(0); //Completely terminates the code

}