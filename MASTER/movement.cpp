// MASTER FILE
#include <LiquidCrystal.h>

#define FRONT_LEFT 0
#define LEFT_RIGHT 1
#define LEFT_LEFT 2
#define BACK_RIGHT 3
#define BACK_LEFT 4
#define RIGHT_RIGHT 5
#define RIGHT_LEFT 6
#define FRONT_RIGHT 7



        
void rotate(int direction, int millis){
    if(direction == CLOCKWISE){
        clockwise();
        delay(millis);
        stop();
    } else {
        counterClockwise();
        delay(millis);
        stop();
    }
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
    default:
    stop();
    break;
  }
}


void moveRightAlongWall(){
    readSensors();
    double firstDistance = readSensors(RIGHT_LEFT);
    delay(10);
    double secondDistance = readSensors(RIGHT_RIGHT);
    
    //Rotate until robot is parallel with wall
    while(firstDistance-secondDistance > 0.1){
        if(path == 0)
            if(asin((firstDistance-secondDistance)/12) > 0){
                rotate(CLOCKWISE,10);
            } else {
                rotate(COUNTER_CLOCKWISE,10);
            } 
        else 
            if(asin((firstDistance-secondDistance)/12) > 0){
                rotate(COUNTER_CLOCKWISE,10);
            } else {
                rotate(CLOCKWISE,10);
            } 
    }
    
    double frontDistance = readSensors(FRONT_RIGHT);
    delay(10);
    firstDistance = readSensors(RIGHT_LEFT);
    delay(10);
    secondDistance = readSensors(RIGHT_RIGHT);

    while(frontDistance > 4 && (firstDistance >3 && firstDistance < 5) && (secondDistance > 3 && secondDistance < 5)){
        drive(0);
        delay(10);
        frontDistance = readSensors(FRONT_RIGHT);
        delay(10);
        firstDistance = readSensors(RIGHT_LEFT);
        delay(10);
        secondDistance = readSensors(RIGHT_RIGHT);
    }
    
    drive(999);

    if(frontDistance <=4){
        //increment main case counter;
        
    }
}




