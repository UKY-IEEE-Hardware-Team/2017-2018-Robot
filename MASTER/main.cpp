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



LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
const int switchStop = 34;
const int switchStart = 35;

double orientation = 0;

double distanceFromMiddle(int sides){
    if(sides == 0){   //Front/Back Side
        double firstFirst = readSensors(FRONT_LEFT);
        double secondSecond = readSensors(BACK_RIGHT);
        double firstSecond = readSensors(FRONT_RIGHT);
        double secondFirst = readSensors(BACK_LEFT);
        
        //Rotate until the sensors are roughly perpindicular  
        while((firstFirst-firstSecond) > 1 || (firstFirst-firstSecond) < -1){
            if(asin(firstFirst-firstSecond)/12) > 0){
                rotate(CLOCKWISE, time);
                double firstFirst = readSensors(FRONT_LEFT);
                double secondSecond = readSensors(BACK_RIGHT);
                double firstSecond = readSensors(FRONT_RIGHT);
                double secondFirst = readSensors(BACK_LEFT);
            } else {
                rotate(CLOCKWISE, time);
                double firstFirst = readSensors(FRONT_LEFT);
                double secondSecond = readSensors(BACK_RIGHT);
                double firstSecond = readSensors(FRONT_RIGHT);
                double secondFirst = readSensors(BACK_LEFT);
            }
        }
        return firstFirst - secondFirst; //If 0 or close, in middle; > 0 then the back side is closer than the front so we should drive forward; < 0 back is farther, so drive backwards
    } else { //Right/Left Side
        double firstFirst = readSensors(LEFT_LEFT);
        double secondSecond = readSensors(RIGHT_RIGHT);
        double firstSecond = readSensors(LEFT_RIGHT);
        double secondFirst = readSensors(RIGHT_LEFT);
        
          
        while((firstFirst-firstSecond) > 1 || (firstFirst-firstSecond) < -1){
            if(atanf((firstFirst-firstSecond)/12) > 0){
                rotate(COUNTER_CLOCKWISE, 10);
                double firstFirst = readSensors(LEFT_LEFT);
                double secondSecond = readSensors(RIGHT_RIGHT);
                double firstSecond = readSensors(LEFT_RIGHT);
                double secondFirst = readSensors(RIGHT_LEFT);
            } else {
                rotate(COUNTER_CLOCKWISE, 10);
                double firstFirst = readSensors(LEFT_LEFT);
                double secondSecond = readSensors(RIGHT_RIGHT);
                double firstSecond = readSensors(LEFT_RIGHT);
                double secondFirst = readSensors(RIGHT_LEFT);
            }
        }
        return firstFirst-secondFirst;
    }
}
        
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

void readSwitch()
{
  if (!(digitalRead(switchStop)))
    state = 0; //Do nothing
  else if (!(digitalRead(switchStart)))
    state = 1;
  else
    state = 0; //Do nothing
}



void readSensors(){
  for(int x = 0; x < 12; x++){
    sensorReadings[x] = sensors[x].read();
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



void writePathLCD(int path)
{
  
}

void drive(double Direction, double Speed)
{

}

double readSensorDistance()
{
  
}



