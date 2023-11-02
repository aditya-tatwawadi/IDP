

#include <Servo.h>

// create servo object to control a servo
Servo myservo;  

// variable to store the servo position
int pos = 0;    

void setup() {
   // attaches the servo on pin 9 to the servo object
  myservo.attach(9); 
}

void loop() {
  // goes from 0 degrees to 180 degrees
  for (pos = 0; pos <= 180; pos += 1) { 
    // in steps of 1 degree

    // tell servo to go to position in variable 'pos'
    myservo.write(pos);   
    // waits 5ms for the servo to reach the position           
    delay(5);                       
  }
  // goes from 180 degrees to 0 degrees
  for (pos = 180; pos >= 0; pos -= 1) { 
     // tell servo to go to position in variable 'pos'
    myservo.write(pos);             
    // waits 5ms for the servo to reach the position
    delay(5);                       
  }
}
