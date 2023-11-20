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
int LineFR = 4; //Far Right Line Sensor
int LineR = 7;
int LineL = 8;
int LineFL = 12; //Far Left Line Sensor
int StartButton = 1;
int LEDG = 1;
int LEDB = 1;
int LEDW = 1;
int Magnetsens = 1; // Hall effect sensor

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
    myMotorR->setSpeed(180);
    myMotorL->setSpeed(255);

    
    if (digitalRead(LineR) == HIGH || digitalRead(LineL) == HIGH) { // this statement checks to see if a line has been reached - then stops it and ends while loop. Right or left sensor checked
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);
      break;
    }                                             // robot should now be at the white zone junction.
  }
}

// Rotate 90 degrees anticlockwise
void Rotate90AC() {
  
  myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(255);
  myMotorL->setSpeed(255);
  delay(700);

  while(1){
    myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
    myMotorL->run(BACKWARD);
    myMotorR->setSpeed(255);
    myMotorL->setSpeed(255);

    if (digitalRead(LineFR)==HIGH || digitalRead(LineFL)==HIGH){
      myMotorR->run(RELEASE);
      myMotorL->run(RELEASE);
      break;
    }
  }
}

void Rotate90CW() {
  
  myMotorR->run(BACKWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(255);
  myMotorL->setSpeed(255);
  delay(700);

  while(1){
    myMotorR->run(BACKWARD);       // rotating the robot in place, slowly, as to not lose the cube
    myMotorL->run(FORWARD);
    myMotorR->setSpeed(255);
    myMotorL->setSpeed(255);

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
  myMotorR->setSpeed(255);
  myMotorL->setSpeed(255);
  delay(1500);

  /*while(1){
    myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
    myMotorL->run(BACKWARD);
    myMotorR->setSpeed(255);
    myMotorL->setSpeed(255);

    if (digitalRead(LineFR)==HIGH || digitalRead(LineFL)==HIGH){
      myMotorR->run(RELEASE);
      myMotorL->run(RELEASE);
      break;
    }
  }

  while(1){
    myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
    myMotorL->run(BACKWARD);
    myMotorR->setSpeed(255);
    myMotorL->setSpeed(255);

    if (digitalRead(LineFR)==HIGH || digitalRead(LineFL)==HIGH){
      myMotorR->run(RELEASE);
      myMotorL->run(RELEASE);
      break;
    }
  } */
}

// Go to white junction to center of the START, pointing south
void GoToCentW() {
  myMotorR->run(FORWARD);
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(225);
  myMotorL->setSpeed(255);
  delay(1000);
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of white area
}

// Follow the line
void LineFollow() { 
  // 4 line sensors next to each other
  while (1) {

    if (digitalRead(LineR) == LOW && digitalRead(LineL) == LOW) { // goes forward
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(225);
      myMotorL->setSpeed(255);
    }
    if (digitalRead(LineR) == LOW && digitalRead(LineL) == HIGH) { // robot drifts to the right so now correct so it turns a bit to the left
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(225);
      myMotorL->setSpeed(225);
    }
    if (digitalRead(LineR) == HIGH && digitalRead(LineL) == LOW) { // robot drifts to the left so now correct so it turns a bit to the right
      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);
      myMotorR->setSpeed(195);
      myMotorL->setSpeed(255);
    }
    if ((digitalRead(LineL) == HIGH && digitalRead(LineR) == HIGH) || ((digitalRead(LineR) == HIGH || digitalRead(LineL) == HIGH) && (digitalRead(LineFL) == HIGH || digitalRead(LineFR) == HIGH))) { // this statement causes the robot to stop when a junction is hit
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
  myMotorR->setSpeed(225);
  myMotorL->setSpeed(255);    
  delay(300);
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);
}

// Go to red from the centre of START when facing south
void GoToRed() { 
  //Take back to Red
      myMotorR->run(FORWARD);       // rotating the robot in place, slowly, as to not lose the cube
      myMotorL->run(BACKWARD);
      myMotorR->setSpeed(255);
      myMotorL->setSpeed(255);
      delay(890);                  // this delay will need to be worked out until roughly 90* is achieved
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);

      delay(1000);

      myMotorR->run(FORWARD);
      myMotorL->run(FORWARD);           // travel forwards to red area
      myMotorR->setSpeed(225);
      myMotorL->setSpeed(255);
      delay(4000);                           // change delay to work out best distance
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);                                      // robot now in center of red area  ------ hopefully...

      delay(1000);

      myMotorR->run(BACKWARD);
      myMotorL->run(BACKWARD);
      myMotorR->setSpeed(225);
      myMotorL->setSpeed(255);
      delay(4000);                           // change delay to work out best distance
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);                                      // robot now in center of white area  ------ hopefully...

      delay(1000);

      myMotorR->run(FORWARD);       // rotating the robot in place
      myMotorL->run(BACKWARD);
      myMotorR->setSpeed(255);
      myMotorL->setSpeed(255);
      delay(890);                  // this delay will need to be worked out until roughly 90* is achieved
      myMotorR->setSpeed(0);
      myMotorL->setSpeed(0);

      delay(1000);
}

//Go to green from the centre of START when facing south
void GoToGreen() {
  myMotorR->run(BACKWARD);       // rotating the robot in place, slowly, as to not lose the cube
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(255);
  myMotorL->setSpeed(255);
  delay(890);                  // this delay will need to be worked out until roughly 90* is achieved
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);    

  delay(1000);

  myMotorR->run(FORWARD);       // travel forwards to green
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(225);
  myMotorL->setSpeed(255);
  delay(4000);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of green area  ------ hopefully...

  delay(1000);

  myMotorR->run(BACKWARD);
  myMotorL->run(BACKWARD);
  myMotorR->setSpeed(225);
  myMotorL->setSpeed(255);
  delay(4000);                           // change delay to work out best distance
  myMotorR->setSpeed(0);
  myMotorL->setSpeed(0);                                      // robot now in center of white area  ------ hopefully...

  delay(1000);

  myMotorR->run(BACKWARD);       // roating the robot in place
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(255);
  myMotorL->setSpeed(255);
  delay(890);                  // this delay will need to be worked out until roughly 90* is achieved
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
  if (BlDist <= 60) {

    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    LineFollow();  
                                                                                  
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    Rotate180();
    delay(1000);
    
    InitialMovement();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    LineFollow();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    GoToCentW();
    delay(1000); // TO BE DELETED LATER - FOR TESTING
    delay(1000); // TO BE DELETED LATER - FOR TESTING

    if (digitalRead(Magnetsens) == HIGH) {
      GoToGreen();
    }    

    else {
      GoToRed();
    }

    delay(1000); // TO BE DELETED LATER - FOR TESTING
    GoToWJunc();

  } //---------------------------------------  robot should have done first block, ready to restart from white zone node --------------------------------------

  delay(1000);
  InitialMovement();
  LineFollow();                                             
  
  delay(1000);
  //robot now at node F facing north

  // Turn on TOF sensor to see if there is Block ahead.
  TOFsensor.start();
  delay(100);
  BlDist = TOFsensor.getDistance();
  //if block is within 60cm it must be at next junction
  if (BlDist <= 60) {

    InitialMovement();
    LineFollow(); 

    delay(1000);                                                                                
 
    Rotate180();

    delay(1000);

    InitialMovement();
    LineFollow();                                                          // robot is now back at junction F

    delay(1000);

    InitialMovement();
    LineFollow();                                                          // robot is now back at white zone junction

    delay(1000);
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
    LineFollow();                                                          // robot is now back at junction F facing north

    InitialMovement();
    LineFollow();                       // robot is now back at junction E facing north


}                              //------------------------------------------------------------------------------------Stage One Complete-------------------------------------------------------------------------

