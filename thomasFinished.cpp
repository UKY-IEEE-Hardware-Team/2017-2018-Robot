#include "common.h"
#include "chris.h"




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
