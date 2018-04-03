// MASTER FILE
#include <LiquidCrystal.h>

#define FRONT_LEFT  0
#define LEFT_RIGHT  1
#define LEFT_LEFT   2
#define BACK_RIGHT  3
#define BACK_LEFT   4
#define RIGHT_RIGHT 5
#define RIGHT_LEFT  6
#define FRONT_RIGHT 7 

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // Initialize LCD

const int switchStop =  0;    // Pin for Switch Stop
const int switchStart = 1;   // Pin for Switch Start

const int irPin = 2;          // Pin for Solar Panel

const int trigPin[8] = {22, 24, 26, 28, 30, 32, 34, 36}; // Pins for Ultrasonic Trigger
const int echoPin[8] = {23, 25, 27, 29, 31, 33, 35, 37}; // Pins for Ultrasoinc Echo

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
  pinMode(switchStop, INPUT_PULLUP);  // Initialize Switch Stop
  pinMode(switchStart, INPUT_PULLUP); // Initialize Switch Start
  lcd.begin(16, 2);                   // Initialize LCD
  for (int i = 0; i < 8; i++) {
    pinMode(trigPin[i], OUTPUT);      // Initialize 8 Trig Pins
    pinMode(echoPin[i], INPUT);       // Initialize 8 Echo Pins
  }
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




int readSolarPanel()
{
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
  lcd.clear();
  lcd.home();
  if (!(digitalRead(switchStop)))
    lcd.print("STOP");
  else if (!(digitalRead(switchStart)))
    lcd.print("START");
  else
    lcd.print("STANDBUYNOW");
  delay(100);
}

void writePathLCD(int path)
{
  lcd.clear();
  lcd.home();
  if (path == -1){
    lcd.print("Scanning");
  } else if (path == -2) {
    lcd.print("Waiting");
  } else {
    lcd.print("Path: ");
    lcd.print(path);
  }
}

void drive(double Direction, double Speed)
{

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



