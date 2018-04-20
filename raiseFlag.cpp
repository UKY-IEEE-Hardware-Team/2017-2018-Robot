#include <Servo.h>

#define FORWARD 0
#define BACKWARD 4
#define STOP 10

Servo flagServo;  // initializes the servo flagServo

void raiseFlag()  // task to be called by the state machiene to raise the flag
{
  flagServo.attach(FLAG_SERVO); // attaches flagServo to the FLAG_SERVO pin
  flagServo.write(93);   // rotates flagServo counterclockwise to its designated position

  drive(FORWARD);
  delay(250);
  drive(STOP);

  stepperMotorRotate();

  drive(BACKWARD);
  delay(50);
  drive(STOP);
}
