#define PULSE 3 // defining pulse pin 
#define DIR 13 // defining direction pin 
#define RIGHT 0 // defining directions
#define LEFT 1 // defining directions 
uint32_t stepcount; // long integer to count the steps moved 
uint32_t steprate; // long integer to set the steprate of each revolution 
uint32_t dwant; // long integer distance you want to move in mm
uint32_t DTRAVEL; // long integer distance traveled in one step in mm
uint32_t NSTEP; // long integer number of steps we want to move 

void setup() { // Setting up the values 
  pinMode(PULSE, OUTPUT); // setting Pulse pin to return the output 
  pinMode(DIR, OUTPUT); // setting DIR pin to return the output 
  stepcount = 0; // setting stepcount as 0 for the origin 
  steprate = 200; // setting steprate for the movement 
  Serial.begin(9600); // start serial communication
}

void moveMotor(int dwant, bool dir) { // making a function named moveMotor to move motor 
  digitalWrite(DIR, dir); // setting the value of DIR as dir for determining direction
  NSTEP = (dwant * steprate) / 8; // calculating number of steps we need to move to reach the distance we want 
  stepcount = 0; // reset step count
  while (stepcount < NSTEP) { // if condition loop to reach the number of steps we want to move 
    digitalWrite(PULSE, HIGH); // Setting pulse as high when we want to move 
    digitalWrite(PULSE, LOW); // Setting pulse as Low to stop when we reached the limit
    delay(2); // stopping for 2 milliseconds 
    stepcount++; // increase the step count as it moved one step
  }
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("MOVE")) {
      int dwant = command.substring(5).toInt();
      bool dir = command.substring(4, 5).toInt();
      moveMotor(dwant, dir);
    }
  }
}
