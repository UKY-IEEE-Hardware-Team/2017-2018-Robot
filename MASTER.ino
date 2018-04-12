// MASTER FILE
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <Servo.h>

#define FRONT_LEFT  0
#define LEFT_RIGHT  1
#define LEFT_LEFT   2
#define BACK_RIGHT  3
#define BACK_LEFT   4
#define RIGHT_RIGHT 5
#define RIGHT_LEFT  6
#define FRONT_RIGHT 7

const int stepsPerRevolution = 2048;  
//The pin connections need to be 4 pins connected
// to Motor Driver In1, In2, In3, In4  and then the pins entered
// here in the sequence 1-3-2-4 for proper sequencing
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11); 

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
const int switchStop = 34;
const int switchStart = 35;

const int irPin = 2;          // Pin for Solar Panel

const int trigPin[8] = {22, 24, 26, 28, 30, 32, 34, 36}; // Pins for Ultrasonic Trigger
const int echoPin[8] = {23, 25, 27, 29, 31, 33, 35, 37}; // Pins for Ultrasoinc Echo

double orientation = 0;

int path = 0;
int bit = 2;

int state = 0;
double direction = 0;

Servo backleft;
Servo backright;
Servo frontright;
Servo frontleft;

void setup() {
  // put your setup code here, to run once:
  pinMode(switchStop, INPUT_PULLUP);
  pinMode(switchStart, INPUT_PULLUP);

  // Attach servos for driving
  backright.attach(46);
  backleft.attach(25);  
  frontright.attach(45);
  frontleft.attach(19);

}


void readSwitch()
{
  if (!(digitalRead(switchStop)))
    state = 0; //Do nothing
  else if (!(digitalRead(switchStart)))
    state = 1;
  else
    state = 0; //Do nothing
}


double readSensorDistance(int sensorNumber) // return inches from ultrasonic
{
  const long inchThreshold = 75.0;  // Maximum inch reading
  const long betweenThresh = 3.0;   // Maximum difference from last reading 
  long duration;                    // Used for distance measurement
  long distanceInch;                // Distance in Inches
  long distanceAve;                 // Average distance in Inches
  // int prevInch;                  // Previous reading

  for (int i = 0; i < 4; i++) {
    digitalWrite(trigPin[sensorNumber], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin[sensorNumber], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin[sensorNumber], LOW);
    duration = pulseIn(echoPin[sensorNumber], HIGH);
    distanceInch = duration*0.0133/2;

    if (distanceInch < inchThreshold /*&& abs(distanceInch - prevInch) < betweenThresh */){ // Average of 4, if amount exceeds thresholds, ignore it before it gets averaged.
      if (i == 3){
        distanceAve = distanceAve + distanceInch;
        distanceAve = distanceAve / 4; 
        Serial.print("Distance: ");
        Serial.print(distanceAve);
        Serial.println(" inch");
      } else {
        distanceAve = distanceAve + distanceInch;
      }
      // prevInch = distanceInch;
    } else {
      Serial.println("Ignored Reading");
      i = i - 1;
    }
    delay(60);
  }
  return distanceAve;
}

// Code from Robot_turning for drive functions

void forward(){ 
  frontleft.write(0);
  frontright.write(180);
  backright.write(180);
  backleft.write(0);
  }

  
void backwards(){
  backleft.write(180);
  frontleft.write(180);
  frontright.write(0);
  backright.write(0);
  }

 void right(){
  backleft.write(180);
  frontleft.write(0);
  frontright.write(0);
  backright.write(180);
 
 }
  void left(){    
  backleft.write(0);
  frontleft.write(180);
  frontright.write(180);
  backright.write(0);
 }

 void clockwise(){
  backleft.write(0);
  frontleft.write(0);
  frontright.write(0);
  backright.write(0);
  }

 void counterclockwise(){
  backleft.write(180); 
  frontleft.write(180);
  frontright.write(180);
  backright.write(180);
 }

 void backrightstrafe(){
  backright.write(0);
  frontleft.write(180);
  frontright.write(90);
  backleft.write(90);
 }
 
 void backleftstrafe(){
  backright.write(90);
  frontleft.write(90);
  frontright.write(0);
  backleft.write(180);
 }
 
 void frontrightstrafe(){
  backright.write(180);
  frontleft.write(0);
  frontright.write(90);
  backleft.write(90);
 }
 
 void frontleftstrafe(){
  backright.write(90);
  frontleft.write(90);
  frontright.write(180);
  backleft.write(0);
 }

 void stopMoving(){
  backright.write(90);
  frontleft.write(90);
  frontright.write(90);
  backleft.write(90);
 }


void drive(int direction1)
{
  switch(direction1) //Uses functions from Robot_turning
  {
    case(0):
    forward();
    break;
    case(1):
    frontrightstrafe();
    break;
    case(2):
    right();
    break;
    case(3):
    backrightstrafe();
    break;
    case(4):
    backwards();
    break;
    case(5):
    backleftstrafe();
    break;
    case(6):
    left();
    break;
    case(7):
    frontleftstrafe();
    break;
    case(8):
    clockwise();
    break;
    case(9):
    counterclockwise();
    break;
    case(10):
    stopMoving();
    break;
  }
}

int switchState()
{
lcd.setCursor(0, 0);
  if (!(digitalRead(switchStop)))
    lcd.print("STOP");
  else if (!(digitalRead(switchStart)))
    lcd.print("START");
  else
    lcd.print("STANDBUYNOW");
  delay(100);
  lcd.clear();
}

int setDirection(int path) {
  if(bit == 2){
    if(path & 0b100){
      direction = 90.0;
    }else{
      direction = -90.0; 
    }    
  } 
  else if(bit==1) {
    if(path & 0b010){
      direction = 90.0;
    } else {
      direction = -90.0;
    }
  }
  else if(bit == 0){
    if(path & 0b001){
      direction = 90.0;
    } else {
      direction = -90.0;
    }
  }
}

void goToPosition(int path)
{
  int directionToGo = setDirection(path);
  if (directionToGo == 90.0) // If button needed to push is the top one
  {
    drive(6); // Go to the left

    bool closeEnough = false;
    while (!closeEnough)
    {
      double distanceToWall = readSensorDistance(LEFT_RIGHT); // Determine the distance to the wall
      if (distanceToWall <= 5.0) // If the distance to wall is less than the distance we want to stop at
      {
        closeEnough = true; 
      }
    }
    drive(10); // Stop 
  }
  else if (directionToGo == -90.0) // If button needed to push is the bottom one
  {
    drive(2); // Go to the right

    bool closeEnough = false;
    while (!closeEnough)
    {
      double distanceToWall = readSensorDistance(RIGHT_RIGHT); // Determine the distance to the wall
      if (distanceToWall <= 5.0) // If the distance to wall is less than the distance we want to stop at
      {
        closeEnough = true; 
      }
    }
    drive(10); // Stop 
  }
}

void moveToMiddle()
{
    int directionToGo = setDirection(path);

  if (directionToGo == 90.0) // If button pushed is the top one
  {
    drive(2); // Go to the right

    bool closeEnough = false;
    while (!closeEnough)
    {
      double distanceToMiddle = readSensorDistance(RIGHT_RIGHT); // Determine the distance to the middle
      if (distanceToMiddle <= 0.1) // If the distance to the middle is less than the distance we want to stop at
      {
        closeEnough = true; 
      }
    }
    drive(10); // Stop 
  }
  else if (directionToGo == -90.0) // If button pushed is the bottom one
  {
    drive(6); // Go to the left

    bool closeEnough = false;
    while (!closeEnough)
    {
      double distanceToMiddle = readSensorDistance(LEFT_RIGHT); // Determine the distance to the middle
      if (distanceToMiddle <= 0.1) // If the distance to the middle is less than the distance we want to stop at
      {
        closeEnough = true; 
      }
    }
    drive(10); // Stop 
  }
}

void moveDownRamp()
{
  drive(0); // Go forward

    bool closeEnough = false;
    while (!closeEnough)
    {
      double distanceToDownRamp = readSensorDistance(FRONT_RIGHT); // Determine the distance to down off the ramp
      if (distanceToDownRamp <= 0.1) // If the distance to the down off the ramp is less than the distance we want to stop at
      {
        closeEnough = true; 
      }
    }
    drive(10); // Stop 
}

int readSolarPanel()
{
  const int irPin = 2;          // Pin for Solar Panel
  const int threshold = 25;     // Vertical threshold for IR signal
  const int horzThresh = 15;    // I have no idea what number this was supposed to be. It should be a number between the two possible edgeDistances.
  const int sampleLength = 500; // Takes 500 samples       
  bool vals[sampleLength];      // Array for input
  int edgeList[10];             // Starting time of each of the bits
  int pulseCount = 0;           // Number of rising edges detected. This number won't count the first one.
  bool bitVal;                  // The value of the bit read.
  int outVal = 0;

  unsigned long startTime = millis();
  const unsigned long timeWait = 200; // Changed according to rules, MAY BREAK CODE

  // Wait for IR reading to exceed threshold or time to exceed time threshold
  while (analogRead(irPin) <= threshold && millis() - startTime <= timeWait);
  
  if (millis() - startTime > timeWait) // If wait is too long, time out and return -1
  {
    return -1;
  }

  // Read IR data, threshold, and put into array
  for (int i = 0; i < sampleLength; i++)
  {
    vals[i] = analogRead(irPin) > threshold;
  }

  // Pulse Counter
  pulseCount = 0;
  // Iterate over IR data array. Start at index 1, not index 0 to ignore first rising edge.
  for (int i = 1; i < sampleLength; i++)
  {
    if (vals[i] > vals[i - 1]) // Detect a rising edge
    {
      // Register rising edge as a pulse
      pulseCount++;
      // Store sample number of pulse into edgeList
      edgeList[pulseCount] = i;
    }
  }

  if (pulseCount != 9) // Return -1 if wrong number of pulses detected
  {
    return -1;
  }
  for (int k = 1; k < 6; k++) // If any of the first five of eight bits read as one, it is in the waiting state
  {
    if (edgeList[k + 1] - edgeList[k] >= horzThresh) 
    {
      return -2;
    }
  }

  // Iterate over the 3 bits in edgeList
  for (int k = 6; k < 9; k++)
  {
    // Measure number of samples between rising edges, threshold, and store bit value
    bitVal = edgeList[k + 1] - edgeList[k] >= horzThresh;
    // Write bit value to outVal int
    bitWrite(outVal, (2 - k) + 6, bitVal);
  }
  return outVal;
}

void stepperMotorRotate()
{
  myStepper.setSpeed(15);
  myStepper.step(stepsPerRevolution*5); // For full points, rotate between 4.75 and 5.24 times
}

void displayPath(){}
void drive(){}
void ramButton(){}
void readPathValue(){}
void goDownAndRightOrUpAndRight(){}
void moveRight(){}
void rotateToCorrectOrientation(){}
void alignWithWedge(){}
void wedgeServo(){}
void moveTowardsWedge(){}
void backOffWedge(){}
void moveInwards(){}
void alignWithCenter(){}
void rotateServoCounterClockwise(){}
void driveForwardALittleBit(){}
void backOff(){}
void rotateNinetyDegrees(){}
void driveTowardsTheBooty(){}
void lowerTheBootyMachine(){}
void pullOnTheBooty(){}
void inverseRotate90Degrees(){}
void driveBackUpRamp(){}
void readLastPartOfPath(){}
void moveInThatDirection(){}
void enjoyYourBooty(){}

void loop() {

int tempInt = 0; //Temporary int for readSensorDistance REMOVE LATER!!!

  switch(state){
    case(0):
      readSwitch(); //read the switch for what to do initially
      break;
     case(1):
      readSolarPanel(); //Read the panel to get the path to follow
      break;
     case(2):
      displayPath();
      break;
     case(3):
      goToPosition(path); // Go either north or south based on path
      break;
     case(4):
      readSensorDistance(tempInt); //Get the values from each sensor and readjust motion path to line up with the button
      break;
     case(5):
      drive(); //Drive towards the button (as determined by case 4)
      break;
     case(6):
      ramButton(); //When aligned with the button, drive forward to activate it
      break;
     case(7):
      moveToMiddle(); //Move robot back towards the middle of the boat
      break;
     case(8):
      readSensorDistance(tempInt); // get values and readjust motion path to get to the middle
      break;
     case(9):
      moveDownRamp();  //move down the ramp
      break;
     case(10):
      readSensorDistance(tempInt); //Get our location in the map
      break;
     case(11):
      readPathValue(); // either up or down
      break;
     case(12):
      goDownAndRightOrUpAndRight();
      break;
     case(13):
      readSensorDistance(tempInt); //Keep a constant distance from the wall
      break;
     case(14):
      moveRight(); //Move right until we are aligned with the wedge
      break;
     case(15):
      rotateToCorrectOrientation(); //Rotate so the wedge thing is oriented right (based on path)
      break;
     case(16):
      alignWithWedge(); //Based on sensor readings
      break;
     case(17):
      wedgeServo();
      break;
     case(18):
      moveTowardsWedge();
      break;
     case(19):
      backOffWedge();
      break;
     case(20):
      rotateToCorrectOrientation();
      break;
     case(21):
      readSensorDistance(tempInt); //Keep constant distance with wall
      break;
     case(22): 
      moveRight();  //Along wall until we almost hit wall in front of us
      break;
     case(23):
      moveInwards(); //Depends on the path variable
      break;
     case(24):
      alignWithCenter(); //Align with center so we can do stepper motor things
      break;
     case(25):
      rotateServoCounterClockwise();
      break;
     case(26):
      driveForwardALittleBit(); // Ensure contact with wall
      break;
     case(27):
      stepperMotorRotate();
      break;
     case(28):
      backOff();
      break;
     case(29):
      rotateNinetyDegrees();
      break;
     case(30):
      driveTowardsTheBooty();
      break;
     case(31):
      lowerTheBootyMachine(); //BootyServoLeft
      break;
     case(32):
      pullOnTheBooty(); //BootyServoRight
      break;
     case(33):
      inverseRotate90Degrees();
      break;
     case(34):
      driveBackUpRamp();
      break;
     case(35):
      readSensorDistance(tempInt); // Know where on the boat we are
      break;
     case(36):
      readLastPartOfPath();
      break;
     case(37):
      moveInThatDirection();
      break;
     case(38):
      enjoyYourBooty();
      break;  
     
      
            
  }
  
}

