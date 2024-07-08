#define PULSE 3 // defing pulse pin 
#define DIR 5 // defining direction pin 
#define RIGHT 0 // definign directions
#define LEFT 1 // defining durections 
uint32_t stepcount; // long integer to count the steps moved 
uint32_t steprate; // long integer to set the steprate of each revolution 
uint32_t dwant; // long integer distance you want to move in mm
uint32_t DTRAVEL; // long integer distance tarvel in one step in mm
uint32_t NSTEP; // long integer number of steps we want to move 

void setup() { // Setting uo the values 
  pinMode(PULSE, OUTPUT); // setting Pulse pin to return the output 
  pinMode(DIR, OUTPUT); // detting DIR pin to return the output 
  stepcount = 0; // setting stepcount as 0 for the origin 
  steprate = 200; // setting steprate for the movement 
}

void moveMotor(int dwant, bool dir) { //makinga function named moveMoter to move moter 
  digitalWrite(DIR,dir); // setting the value of DIR as dir for determinig  direction
    NSTEP = (dwant*steprate)/8; // calculating number of dteps we need to move to reach the distance we want 
  if(stepcount < NSTEP){ // if condition loop to reach the number of steps we want to move 
    digitalWrite(PULSE, HIGH); //Setting pulse as high when we want to move 
    digitalWrite(PULSE, LOW); //Setting pulse as LOw to stop when we reached the limit
    delay(2); // stopping for 2 milisecond 
    stepcount++; //increase the step count as it moved one step
  }
}

void loop() { 
  moveMotor(100,LEFT); // calling the loop to move in the direction and the distance we want to move 
}
