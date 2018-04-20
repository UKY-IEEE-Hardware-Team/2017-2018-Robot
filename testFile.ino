#include <Servo.h>
#include "math.h"


const int  FRONT_LEFT  = 0;
const int  LEFT_RIGHT  = 1;
const int  LEFT_LEFT   = 2;
const int  BACK_RIGHT  = 3;
const int  BACK_LEFT   = 4;
const int  RIGHT_RIGHT = 5;
const int  RIGHT_LEFT  = 6;
const int  FRONT_RIGHT = 7;

static int locationA;
static int locationB;
static int locationC;

static int state;
const int irPin = 2;          // Pin for Solar Panel

const int trigPin[8] = {22, 24, 26, 28, 30, 32, 34, 36}; // Pins for Ultrasonic Trigger
const int echoPin[8] = {23, 25, 27, 29, 31, 33, 35, 37}; // Pins for Ultrasoinc Echo
const int switchStop =  0;    // Pin for Switch Stop
const int switchStart = 1;   // Pin for Switch Start

int Ledpin=13;
int secdelay=2000;
int degreeIn;

Servo backleft;
Servo backright;
Servo frontright;
Servo frontleft; //Need to do frontLeft.attach(pin #) for all servos





void rotate(int direction, int millis){
    if(direction == 0){
        drive(8);
        delay(millis);
        drive(10);
    } else {
        drive(9);
        delay(millis);
        drive(10);
    }
}

//TODO Integrate the read sensors thing with the way we are actually getting it
double distanceFromMiddle(int sides){
  double firstFirst;
  double firstSecond;
  double secondFirst;
  double secondSecond;
    if(sides == 0){   //Front/Back Side
        firstFirst = readSensorDistance(FRONT_LEFT);
        secondSecond = readSensorDistance(BACK_RIGHT);
        firstSecond = readSensorDistance(FRONT_RIGHT);
        secondFirst = readSensorDistance(BACK_LEFT);

        //Rotate until the sensors are roughly perpindicular
        while((firstFirst-firstSecond) > 1 || (firstFirst-firstSecond) < -1){
            if(firstFirst < firstSecond){
                rotate(1, 10);
                firstFirst = readSensorDistance(FRONT_LEFT);
                secondSecond = readSensorDistance(BACK_RIGHT);
                firstSecond = readSensorDistance(FRONT_RIGHT);
                secondFirst = readSensorDistance(BACK_LEFT);
            } else {
                rotate(0, 10);
                firstFirst = readSensorDistance(FRONT_LEFT);
                secondSecond = readSensorDistance(BACK_RIGHT);
                firstSecond = readSensorDistance(FRONT_RIGHT);
                secondFirst = readSensorDistance(BACK_LEFT);
            }
        }
        return firstFirst - secondFirst; //If 0 or close, in middle; > 0 then the back side is closer than the front so we should drive forward; < 0 back is farther, so drive backwards
    } else { //Right/Left Side
        firstFirst = readSensorDistance(LEFT_LEFT);
        secondSecond = readSensorDistance(RIGHT_RIGHT);
        firstSecond = readSensorDistance(LEFT_RIGHT);
        secondFirst = readSensorDistance(RIGHT_LEFT);


        while((firstFirst-firstSecond) > 1 || (firstFirst-firstSecond) < -1){
            if(firstSecond > firstFirst){
                rotate(1, 10);
                firstFirst = readSensorDistance(LEFT_LEFT);
                secondSecond = readSensorDistance(RIGHT_RIGHT);
                firstSecond = readSensorDistance(LEFT_RIGHT);
                secondFirst = readSensorDistance(RIGHT_LEFT);
            } else {
                rotate(0, 10);
                firstFirst = readSensorDistance(LEFT_LEFT);
                secondSecond = readSensorDistance(RIGHT_RIGHT);
                firstSecond = readSensorDistance(LEFT_RIGHT);
                secondFirst = readSensorDistance(RIGHT_LEFT);
            }
        }
        return firstFirst-secondFirst;
    }
}

void moveToMiddle(){ //on the ship
  while(distanceFromMiddle(0) > 2){
    if(locationA == 0){ // We need to go south
      drive(4);
      delay(20);
      drive(10);
    } else {
      drive(0);
      delay(20);
      drive(10);
    }
  }
  drive(10);
}

void moveRightAlongWall(){

  double firstDistance;
  double secondDistance;
  if(locationB == 1){
    firstDistance = readSensorDistance(RIGHT_LEFT);
    delay(10);
    secondDistance = readSensorDistance(RIGHT_RIGHT);
  } else {
    firstDistance = readSensorDistance(LEFT_RIGHT);
    delay(10);
    secondDistance = readSensorDistance(RIGHT_RIGHT);
  }

    //Rotate until robot is parallel with wall
    while(firstDistance-secondDistance > 0.1){
        if(locationB == 0)
            if(secondDistance < firstDistance){
                rotate(0,10);
            } else {
                rotate(1,10);
            }
        else
            if(secondDistance < firstDistance){
                rotate(1,10);
            } else {
                rotate(0,10);
            }
    }
    double frontDistance;
    if(locationB == 1){
    frontDistance = readSensorDistance(FRONT_RIGHT);
    delay(10);
    firstDistance = readSensorDistance(RIGHT_LEFT);
    delay(10);
    secondDistance = readSensorDistance(RIGHT_RIGHT);
  } else {
    frontDistance = readSensorDistance(FRONT_LEFT);
    delay(10);
    firstDistance = readSensorDistance(LEFT_RIGHT);
    delay(10);
    secondDistance = readSensorDistance(LEFT_LEFT);
  }

    while(frontDistance > 4 && (firstDistance >3 && firstDistance < 5) && (secondDistance > 3 && secondDistance < 5)){
        if(locationB == 1){
        drive(4);
        delay(10);
        frontDistance = readSensorDistance(FRONT_RIGHT);
        delay(10);
        firstDistance = readSensorDistance(RIGHT_LEFT);
        delay(10);
        secondDistance = readSensorDistance(RIGHT_RIGHT);
      } else {
        drive(0);
        delay(10);
        frontDistance = readSensorDistance(FRONT_LEFT);
        delay(10);
        firstDistance = readSensorDistance(LEFT_RIGHT);
        delay(10);
        secondDistance = readSensorDistance(LEFT_LEFT);
      }
    }

    drive(10);

    if(frontDistance <=4){
        //increment main case counter;

    }
}

void moveInwards(){ //at the flag
    while(distanceFromMiddle(1) >= 1){
        if(locationB == 0){
          drive(2);
        } else {
          drive(6);
        }
        delay(10);
    }
    drive(10); //turns it off
}



void rotateToPerpindicular(int side) {
    //readSensorDistance();
    int first,second;
    if(side == 0){ //front
      first = 0;
      second = 7;
    }
    if(side == 1){ //left
      first = 2;
      second = 1;
    }
    if(side == 2){ //back
      first = 4;
      second = 3;
    }
    if(side == 3){ //right
      first = 6;
      second = 5;
    }

    double firstDistance = readSensorDistance(first);
    delay(10);
    double secondDistance = readSensorDistance(second);



    while(firstDistance-secondDistance > 0.1){
        if(secondDistance > firstDistance){
          rotate(0,10);
        } else {
          rotate(1,10);
        }

        firstDistance = readSensorDistance(first);
        delay(10);
        secondDistance = readSensorDistance(second);
    }
}
void alignWithCenter(){ //prior to raising flag
    //Figure out clockwise or counter clockwise and do so (back of the robot should be pointed at the "wheel")
    //Line up perpindicular.
    rotateToPerpindicular(2); //Should be back I think
    //Confirm that we are in middle left-to-right
    //drive(4);
    double backDistance = readSensorDistance(BACK_LEFT);
    while(backDistance > 2){
      drive(4);
      backDistance = readSensorDistance(BACK_LEFT);
    }
    drive(10);
  }


void servosetup(){
  backright.attach(4);
  backleft.attach(7);
  frontright.attach(5);
  frontleft.attach(6);
}


void degreeToMovement(int degree){
  if ((degree >= 0) && (degree <= 90)){
  frontleft.write(-2*degree + 180);
  frontright.write(0);
  backleft.write(180);
  backright.write(-1*pow(10, -16)*sqrt(degree) + 2*degree + 5*pow(10,(-13)));
  }
  if ((degree > 90) && (degree <= 180)){
  frontleft.write(0);
  frontright.write(-1*pow(10, -16)*sqrt(degree) + 2*degree - 180);
  backleft.write(-2*degree + 360);
  backright.write(180);
  }
  if ((degree > 180) && (degree <= 270)){
  frontleft.write(-1*pow(10, -16)*sqrt(degree) + 2*degree - 360);
  frontright.write(180);
  backleft.write(0);
  backright.write(-2*degree + 540);
  }
  if ((degree > 270) && (degree <= 360)){
  frontleft.write(180);
  frontright.write(-2*degree + 720);
  backleft.write(-1*pow(10, -16)*sqrt(degree) + 2*degree - 540);
  backright.write(0);
  }
}

void forward(){
  backleft.write(180);
  frontleft.write(180);
  frontright.write(0);
  backright.write(0);
  }


void backwards(){
  backleft.write(0);
  frontleft.write(0);
  frontright.write(180);
  backright.write(180);
  }

 void right(){
  backleft.write(60);
  frontleft.write(120);
  frontright.write(60);
  backright.write(120);

 }
  void left(){
  backleft.write(120);
  frontleft.write(60);
  frontright.write(60);
  backright.write(120);
 }

 void clockwise(){
  backleft.write(180);
  frontleft.write(0);
  frontright.write(180);
  backright.write(0);
  }

 void counterclockwise(){
  backleft.write(180);
  frontleft.write(0);
  frontright.write(180);
  backright.write(0);
 }

 void backleftstrafe(){
  backright.write(180);
  frontleft.write(0);
  frontright.write(90);
  backleft.write(90);
 }

 void backrightstrafe(){
  frontright.write(180);
  backleft.write(0);
  frontleft.write(100);
  backright.write(95);
 }

 void frontrightstrafe(){
  backright.write(0);
  frontleft.write(180);
  frontright.write(90);
  backleft.write(90);
  }

 void frontleftstrafe(){
  frontright.write(0);
  backleft.write(180);
  frontleft.write(100);
  backright.write(95);
 }

 void stopmoving(){
   frontright.write(90);
   backleft.write(90);
   frontleft.write(90);
   backright.write(90);
 }


void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    servosetup();
    sensorSetup();
}

//#include <Arduino.h>
#include <Stepper.h>
const int stepsPerRevolution = 2048;
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
// Servo backleft;
// Servo backright;
// Servo frontright;
// Servo frontleft;

// int locationB;
// int locationA;
// int locationC;
// int state;

//
// void servoSetup(){
//   backright.attach(46);
//   backleft.attach(25);
//   frontright.attach(45);
//   frontleft.attach(19);
// }








void drive(int direction)
{
  switch(direction) //Uses functions from Robot_turning
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
    stopmoving();
    break;
  }
}

void setDirection(int path) {



  locationA = path&0b001;
  locationB = (path&0b010)>>1;
  locationC = (path & 0b100)>>2;

  // if(bit == 2){
  //   if(path & 0b100){
  //     direction = 90.0;
  //   }else{
  //     direction = -90.0;
  //   }
  // }
  // else if(bit==1) {
  //   if(path & 0b010){
  //     direction = 90.0;
  //   } else {
  //     direction = -90.0;
  //   }
  // }
  // else if(bit == 0){
  //   if(path & 0b001){
  //     direction = 90.0;
  //   } else {
  //     direction = -90.0;
  //   }
  // }
}

void goToPosition(int path)
{

  //int directionToGo = setDirection(path);

  if (locationA == 0) // If button needed to push is the top one
  {
    drive(6); // Go to the left

    bool closeEnough = false;
    while (!closeEnough)
    {
      double distanceToWall = readSensorDistance(LEFT_RIGHT); // Determine the distance to the wall
      if (distanceToWall <= 4.0) // If the distance to wall is less than the distance we want to stop at
      {
        closeEnough = true;
      }
    }
    drive(10); // Stop
    ramButton();
  }
  else if (locationA == 1) // If button needed to push is the bottom one
  {
    drive(2); // Go to the right

    bool closeEnough = false;
    while (!closeEnough)
    {
      double distanceToWall = readSensorDistance(RIGHT_RIGHT); // Determine the distance to the wall
      if (distanceToWall <= 4.0) // If the distance to wall is less than the distance we want to stop at
      {
        closeEnough = true;
      }
    }
    drive(10); // Stop
    ramButton();
  } else {
    Serial.println("ERROR WRONG goToPosition");
  }
  state = 3;
}
//void ramButton(int directionToGo)
void ramButton()
{
  if (locationA == 0) // If button needed to push is the top one
  {
    double distanceToWallLeft_right = readSensorDistance(LEFT_RIGHT); // Determine the distance to the wall
    double distanceToWallLeft_left = readSensorDistance(LEFT_LEFT); // Determine the distance to the wall
    bool closeEnough = false;

    while (!closeEnough)
    {
      if (distanceToWallLeft_right - distanceToWallLeft_left > 1) // If we are crooked
      {
        drive(9); //CounterClockwise

      }
      else if (distanceToWallLeft_right - distanceToWallLeft_left < -1) // If we are crooked
      {
        drive(8); //Clockwise
      }
      else
      {
        closeEnough = true;
      }
    }

    drive(6); // Go to the left
    delay(1000);
    drive(10); // Stop
  }
  else if (locationA == 1) // If button needed to push is the bottom one
  {
    double distanceToWallRight_right = readSensorDistance(RIGHT_RIGHT); // Determine the distance to the wall
    double distanceToWallRight_left = readSensorDistance(RIGHT_LEFT); // Determine the distance to the wall
    bool closeEnough = false;

    while (!closeEnough)
    {
      if (distanceToWallRight_right - distanceToWallRight_left > 1) // If we are crooked
      {
        drive(8); //Clockwise
      }
      else if (distanceToWallRight_right - distanceToWallRight_left < -1) // If we are crooked
      {
        drive(9); //CounterClockwise
      }
      else
      {
        closeEnough = true;
      }
    }

    drive(2); // Go to the right
    delay(1000);
    drive(10); // Stop
  }
}

//#include "common.h"

void sensorSetup(){
  pinMode(switchStop, INPUT_PULLUP);  // Initialize Switch Stop
  pinMode(switchStart, INPUT_PULLUP); // Initialize Switch Start
  for (int i = 0; i < 8; i++) {
    pinMode(trigPin[i], OUTPUT);      // Initialize 8 Trig Pins
    pinMode(echoPin[i], INPUT);       // Initialize 8 Echo Pins
  }
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


void stepperMotorRotate()
{
  myStepper.setSpeed(15);
  myStepper.step(stepsPerRevolution*5); // For full points, rotate between 4.75 and 5.24 times
  state = 20;
}



void printFunctions(){
  Serial.println("3: goToPosition");
  Serial.println("4: ramButton");
  Serial.println("5: distasnceFromMiddle");
  Serial.println("6: moveToMiddle");
  Serial.println("7: moveRightAlongWall");
  Serial.println("8: moveInwards");
  Serial.println("9: rotateToPerpindicular");
  Serial.println("10: alignWithCenter");
  Serial.println("11: drive");
}



void interface(){
  Serial.println("Enter your function number (-1 to print functions out)");
  while(!Serial.available());
  int x = Serial.parseInt();
  int y, z;
  while(Serial.available()) Serial.read();
  if(x == -1){
    printFunctions();
  } else {
    switch (x){
      // case(0):
      //  readSwitch();
      //  Serial.print("Switch state is: ");
      //  Serial.println(state);
      //  break;
      // case(1):
      //  Serial.print("Value: ");
      //  Serial.println(readSolarPanel());
      //  break;
      // case(2):
      //  writePathLCD(path);
      //  break;
      case(3):
        Serial.print("0 for up or 1 for down");
        while(!Serial.available());
        y = Serial.parseInt();
        if(y==0||y==1){
          locationA = y;
          goToPosition(y);
        }
        break;
      case(4):
        ramButton();
        break;
      case(5):
        Serial.print("0 for front/back, 1 for left/right");
        while(!Serial.available());
        y = Serial.parseInt();
        if(y==0||y==1){
          Serial.print("Distance from middle: ");Serial.println(distanceFromMiddle(y));
        }
        break;
      case(6):
        Serial.println("starting to move to middle");
        moveToMiddle();
        Serial.println("finished moving to middle");
        break;
      case(7):
        Serial.print("0 for top or 1 for bottom");
        while(!Serial.available());
        y = Serial.parseInt();
        if(y==0||y==1){
          locationB = y;
          Serial.println("Starting to move along wall");
          moveRightAlongWall();
          Serial.println("Finishing moving");
        }
        break;
      case(8):
        Serial.println("starting to move inwards towards flag");
        moveInwards();
        Serial.println("finished moving inwards towards flag");
        break;
      case(9):
        Serial.print("0 for front, 1 for left, 2 for back, 3 for right");
        while(!Serial.available());
        y = Serial.parseInt();
        if(y>=0&&y<=3){
          Serial.println("starting to rotate");
          rotateToPerpindicular(y);
          Serial.println("finished rotating");
        }
        break;
      case(10):
        Serial.println("aligning with center");
        alignWithCenter();
        Serial.println("finished aligning with center");
        break;
      case(11):
        Serial.print("Enter a number between 0 and 10");
        while(!Serial.available());
        y = Serial.parseInt();
        if(y>=0 && y <=10){
          drive(y);
          delay(2000);
          drive(10);
        }
        break;
      


    }
  }
}

void loop() {
  interface();
    // drive(2);
    // pinMode(4,OUTPUT);
    // pinMode(5,OUTPUT);
    // pinMode(6,OUTPUT);
    // pinMode(7,OUTPUT);
    // delay(1000);
    // pinMode(4,INPUT);
    // pinMode(5,INPUT);
    // pinMode(6,INPUT);
    // pinMode(7,INPUT);
    // // digitalWrite(4,LOW);
    // // digitalWrite(5,LOW);
    // // digitalWrite(6,LOW);
    // // digitalWrite(7,LOW);
    // //drive(10);
    // delay(1000);
    // put your main code here, to run repeatedly:
}
