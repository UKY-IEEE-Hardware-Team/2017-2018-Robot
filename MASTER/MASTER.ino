// MASTER FILE
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
const int switchStop = 34;
const int switchStart = 35;

double orientation = 0;



int path = 0;
int bit = 2;

class Sensor{
  void read();
};

class UltraSonic extends Sensor{

  UltraSonicSensorName sensor;
  void read();
};

class Sharp extends Sensor{
  SharpSensorName sensor;
  void read();
};


int state = 0;
float direction = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(switchStop, INPUT_PULLUP);
  pinMode(switchStart, INPUT_PULLUP);
}

float sensorReadings[12];
Sensor sensors[12]; //Make this class later



void readSensors(){
  for(int x = 0; x < 12; x++){
    sensorReadings[x] = sensors[x].read();
  }
}

int setDirection() {
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






void loop() {

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
      readSensors(); //Get the values from each sensor and readjust motion path to line up with the button
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
      readSensors(); // get values and readjust motion path to get to the middle
      break;
     case(9):
      moveRight();  //move down the ramp
      break;
     case(10):
      readSensors(); //Get our location in the map
      break;
     case(11):
      readPathValue(); // either up or down
      break;
     case(12):
      goDownAndRightOrUpAndRight();
      break;
     case(13):
      readSensors(); //Keep a constant distance from the wall
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
      readSensors(); //Keep constant distance with wall
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
      readSensors(); // Know where on the boat we are
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
  int switchState = switchState();

  int path = readSolarPanel();

  writePathLCD(path);

  //Based on first number of path, drive north or south
  double direction1;
  double speed1;
  drive(direction1, speed1);

  double distanceToWall = readSensorDistance();

  
  
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

int readSolarPanel()
{
  
}

void writePathLCD(int path)
{
  
}

void drive(double Direction, double Speed)
{

}

double readSensorDistance()
{
  
}

