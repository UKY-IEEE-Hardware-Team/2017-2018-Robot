// MASTER FILE
#include <LiquidCrystal.h>
#include <Servo.h>

#define FRONT_LEFT  0
#define LEFT_RIGHT  1
#define LEFT_LEFT   2
#define BACK_RIGHT  3
#define BACK_LEFT   4
#define RIGHT_RIGHT 5
#define RIGHT_LEFT  6
#define FRONT_RIGHT 7 

LiquidCrystal lcd(8, 9, 0, 1, 2, 3);  // Initialize LCD

const int switchStop =  0;    // Pin for Switch Stop
const int switchStart = 1;   // Pin for Switch Start

const int irPin = A2;          // Pin for Solar Panel

const int trigPin[8] = {22, 24, 26, 28, 30, 32, 34, 36}; // Pins for Ultrasonic Trigger
const int echoPin[8] = {23, 25, 27, 29, 31, 33, 35, 37}; // Pins for Ultrasoinc Echo

Servo wedger;
int state = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(switchStop, INPUT_PULLUP);  // Initialize Switch Stop
  pinMode(switchStart, INPUT_PULLUP); // Initialize Switch Start
  Serial.begin(250000);
  lcd.begin(16, 2);                   // Initialize LCD
  wedger.attach(35);
  for (int i = 0; i < 8; i++) {
    pinMode(trigPin[i], OUTPUT);      // Initialize 8 Trig Pins
    pinMode(echoPin[i], INPUT);       // Initialize 8 Echo Pins
  }
}

int path = -1;
void loop() {
  //if (path == -1 || path == -2){
    path = readSolarPanel(calibSolar());
    Serial.println(path);
  //}
}

float sensorReadings[12];

void readSwitch()
{
  if (!(digitalRead(switchStop)))
    state = 0; //Do nothing
  else if (!(digitalRead(switchStart)))
    state = 1;
  else
    state = 0; //Do nothing
}

int calibSolar() {
  unsigned long startTime = millis();
  unsigned long timeWait = 500;
  int maxRead = 0;
  int minRead = 1023;
  int solarRead = 0;
  
  while (millis() - startTime <= timeWait) {
    solarRead = analogRead(irPin);
    maxRead = max(maxRead, solarRead);
    minRead = min(minRead, solarRead);
  }
  Serial.print("Threshold ");
  Serial.println((maxRead + minRead) / 2);
  return ((maxRead + minRead) / 2);
}


int readSolarPanel(int threshold)
{
  //int threshold = 25;     // Vertical threshold for IR signal
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
  while (analogRead(irPin) <= threshold && millis() - startTime <= timeWait) {
    //Serial.print("IR Pin Reading: ");
    //Serial.println(analogRead(irPin));
  }
  
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
    lcd.print("Path:   ");
    lcd.print(path + 1);
    lcd.print(".");
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

double readSharpDistance(int pinNumber)
{
  double distanceHistory[5];
  double distanceAverage;
  double distanceChartedTwo = 0;
  double reading;
  int index;
  double lookUpTableTwo[23] = {  600,   524,   418,   341,   290,   243,   216,   192,   168,   156,    136,    115,    107,     91,     83,     75,     66,     62,     54,     50,     46,     42,     38}; 

  for(int i = 0; i < 5; i++) {
    reading = analogRead(pinNumber);
    index = 0;
    while(lookUpTableTwo[index] > reading && index < 22) {
      index = index + 1;
    }
    if (index == 22) {
      distanceChartedTwo = 949.2 * pow(reading, -1.01);
    } else if (index == 0) {
      distanceChartedTwo = 0.9;
    } else {
      distanceChartedTwo = ((float(index) - 1)/ 2) + 1 + .5 * (lookUpTableTwo[index - 1] - reading) / ((lookUpTableTwo[index - 1] - lookUpTableTwo[index]));
    }
    
    for(int i = 4; i > 0; i--) {
      distanceHistory[i] = distanceHistory[i-1];
    }
    distanceHistory[0] = distanceChartedTwo;
    delay( 100 );
  }
  for(int i = 0; i < 5; i++) {
    distanceAverage = distanceAverage + distanceHistory[i];
  }
  distanceAverage = distanceAverage / 5.0;
  
  Serial.print(distanceChartedTwo);
  Serial.print("\t");
  Serial.println(distanceAverage);
  return distanceAverage;
}
/*
void goDownAndRightOrUpAndRight(int path) {
  int sensorClose = -1;
  double sensorDiff = 0;
  double read0 = 0;
  double read7 = 0;
  double read1 = 0;
  double read2 = 0;
  // Drive off ramp
  drive(0);
  delay(800); // Drive 0.8 seconds before checking sensors
  drive(10);
  read0 = readSensorDistance(0);
  delay(60);
  read7 = readSensorDistance(7);
  delay(140);
  drive(0);
  while ((read0 < 15) || (read7 < 15) {
    read0 = readSensorDistance(0);
    delay(60);
    read7 = readSensorDistance(7);
    delay(60);
  }
  drive(10);
  while ((read0 < 12) || (read7 < 12) {
    drive(0)
    delay(60);
    drive(10);
    read0 = readSensorDistance(0);
    delay(60);
    read7 = readSensorDistance(7);
  }
  if (bitRead(path, 6)) {
    // Turn clockwise twice, go left
    // Clockwise 1
    drive(8)
    delay(500);
    read0 = readSensorDistance(0);
    delay(60);
    read7 = readSensorDistance(7);
    while(abs(read7 - read0) < 2) {
      delay(60);
      read0 = readSensorDistance(0);
      delay(60);
      read7 = readSensorDistance(7);
    }

    // Clockwise 2
    drive(8)
    delay(500);
    read1 = readSensorDistance(1);
    delay(60);
    read2 = readSensorDistance(2);
    while(abs(read1 - read2) < 2) {
      delay(60);
      read1 = readSensorDistance(1);
      delay(60);
      read2 = readSensorDistance(2);
    }

    // Make sure perpendicular
    read1 = readSensorDistance(1);
    delay(60);
    read2 = readSensorDistance(2);
    sensorDiff = read1 - read2;
    while(abs(sensorDiff) > 1) {
      if(sensorDiff > 1) {
        // Turn counter clockwise
        drive(9);
        delay(60); 
      } else if (sensorDiff < -1) {
        // Turn clockwise
        drive(8);
        delay(60);
      }
      read1 = readSensorDistance(1);
      read2 = readSensorDistance(2);
      sensorDiff = read1 - read2;
    }

    // Go 6 inches away from the wall Left
    drive(6);
    while(read1 < 6 || read2 < 6) {
      read1 = readSensorDistance(1);
      delay(60);
      read2 = readSensorDistance(2);
      delay(60);
    }
    drive(10); 
  } else {
    // Robot is in the correct orientation after leaving the ramp, just go left
    // Make sure perpendicular
    read1 = readSensorDistance(1);
    delay(60);
    read2 = readSensorDistance(2);
    sensorDiff = read1 - read2;
    while(abs(sensorDiff) > 1) {
      if(sensorDiff > 1) {
        // Turn counter clockwise
        drive(9);
        delay(60); 
      } else if (sensorDiff < -1) {
        // Turn clockwise
        drive(8);
        delay(60);
      }
      read1 = readSensorDistance(1);
      read2 = readSensorDistance(2);
      sensorDiff = read1 - read2;
    }
    
    // Go 6 inches away from the wall Left
    drive(6);
    while(read1 < 6 || read2 < 6) {
      read1 = readSensorDistance(1);
      delay(60);
      read2 = readSensorDistance(2);
      delay(60);
    }
    drive(10); 
  }
}
*/
/*
void wedgeAndOff() {
  // This should move wedger to disengaged position (Should already be here)
  wedger.write(900);
  // Align to perpendicular using back and front ultras
  double read0 = 0;
  double read7 = 0;
  double read3 = 0;
  double read4 = 0;
  bool perpendicular = false;
  read0 = readSensorDistance(0);
  read3 = readSensorDistance(3);
  delay(60);
  read7 = readSensorDistance(7);
  read4 = readSensorDistance(4);
  if(abs(read0 - read1) > 1 && abs(read3 - read4) > 1) {
    perpendicular = false;
  } else {
    perpendicular = true;
  }
  while(!perpendicular) {
    // Turn if the robot is biased
    if(read0 > read7 && read4 > read3) {
      // Needs to turn clockwise a bit
      drive(8);
      delay(50);
      drive(10);
    } else if(read0 < read7 && read4 < read3) {
      // Needs to turn counterclockwise a bit
      drive(9);
      delay(50);
      drive(10);
    } else {
      // The robot reads opposite biases on each side
      perpendicular = true;
    }

    // Check perpendicularity
    read0 = readSensorDistance(0);
    read3 = readSensorDistance(3);
    delay(60);
    read7 = readSensorDistance(7);
    read4 = readSensorDistance(4);
    if(abs(read0 - read1) < 1 || abs(read3 - read4) < 1) {
      perpendicular = true;
    }
  }

  // Engage Wedge motor
  wedger.write(1700);

  // Run into Wedge
  drive(6);
  delay(400);
  drive(10);

  // Back off Wedge
  drive(2);
  delay(200);
  drive(10);

  // Disengage Wedge
  wedger.write(900);
}
*/
