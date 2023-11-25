/*
Continually checks for button push event after reset/power on. If button pushed, while loop in void runs indefinitely. 
*/

int StartButton = 3; // Connect push button to input pin 3
void setup() {
 Serial.begin(9600);
 pinMode(StartButton, INPUT); // declare pushbutton as input
}

// function to check if button has been pushed
bool checkButtonPress(){
 int val = digitalRead(StartButton); // read input value
 if (val == HIGH) { // check if the input is HIGH
 return true;
 }
 else {
 return false;
  }
}

void loop(){
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
    //INSERT REAL CODE HERE - OTHERWISE, USE THE BELOW AS A TEST TO SEE IF FUNCTION IS WORKING
   Serial.println("Button has been pushed, run main function");
  }
  
}
