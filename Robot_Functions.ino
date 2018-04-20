#include <Stepper.h>
#include <Servo.h>
const int stepsPerRevolution = 2048; 
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11); 
Servo backleft;
Servo backright;
Servo frontright;
Servo frontleft;
void setup() {
  // put your setup code here, to run once:
backright.attach(46);
  backleft.attach(25);  
  frontright.attach(45);
  frontleft.attach(19);
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
      if (distanceToWall <= 4.0) // If the distance to wall is less than the distance we want to stop at
      {
        closeEnough = true; 
      }
    }
    drive(10); // Stop 
    ramButton(directionToGo);
  }
  else if (directionToGo == -90.0) // If button needed to push is the bottom one
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
    ramButton(directionToGo);
  }
  state = 3;
}

void ramButton(int directionToGo)
{
  if (directionToGo == 90.0) // If button needed to push is the top one
  {
    bool closeEnough = false;

    while (!closeEnough)
    {
      double distanceToWallLeft_right = readSensorDistance(LEFT_RIGHT); // Determine the distance to the wall
      double distanceToWallLeft_left = readSensorDistance(LEFT_LEFT); // Determine the distance to the wall
    
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
  else if (directionToGo == -90.0) // If button needed to push is the bottom one
  {
    bool closeEnough = false;

    while (!closeEnough)
    {
      double distanceToWallRight_right = readSensorDistance(RIGHT_RIGHT); // Determine the distance to the wall
      double distanceToWallRight_left = readSensorDistance(RIGHT_LEFT); // Determine the distance to the wall
    
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
void stepperMotorRotate()
{
  myStepper.setSpeed(15);
  myStepper.step(stepsPerRevolution*5); // For full points, rotate between 4.75 and 5.24 times
  state = 20;
}
void moveDownRamp()
{
    bool closeEnough = false;

    while (!closeEnough)
    {
      double distanceToWallBack_right = readSensorDistance(BACK_RIGHT); // Determine the distance to the wall
      double distanceToWallBack_left = readSensorDistance(BACK_LEFT); // Determine the distance to the wall
    
      if (distanceToWallBack_right - distanceToWallBack_left > 1) // If we are crooked
      {
        drive(8); //Clockwise
      }
      else if (distanceToWallBack_right - distanceToWallBack_left < -1) // If we are crooked
      {
        drive(9); //CounterClockwise
      }
      else
      {
        closeEnough = true; 
      }
    }
  
    drive(0); // Go forward
    delay(10000); // Delay ten seconds. Random time I picked.

    while (readSensorDistance(RIGHT_RIGHT) > 30) // While we are not reading the right wall
    {
       drive(0); // Keep going forward
       delay(1000);
    }
    
    drive(10); // Stop 
}
void loop() {
  // put your main code here, to run repeatedly:

}
