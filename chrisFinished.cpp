#include <Arduino.h>
#include <Stepper.h>
#include <Servo.h>
#include "chris.h"
#include "robot_turning.h"

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

void stepperMotorRotate()
{
  myStepper.setSpeed(15);
  myStepper.step(stepsPerRevolution*5); // For full points, rotate between 4.75 and 5.24 times
  state = 20;
}
