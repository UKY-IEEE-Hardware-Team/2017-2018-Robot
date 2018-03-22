#include <LiquidCrystal.h>

const int irPin = A8;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

int IR_Read(int irPin = A8, int threshold = 20, int horzThresh = 15);

void setup()
{
  Serial.begin(250000);
  lcd.begin(16, 2);
  int i = 0;
  int reading = -1;
  while(reading == -1 || reading == -2)
  {
    //i++;
    reading = IR_Read(irPin);
    Serial.println(reading);
    Serial.println("Attempt");
    //Serial.println(i);
    if (reading == -1)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Scanning");
    } 
    else if (reading == -2) 
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Waiting");
    }
  }
  Serial.println("Reading Success");
  Serial.println(reading);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(reading);
}

void loop()
{
  
}

int IR_Read(int irPin, int threshold, int horzThresh)
{
  //const int threshold = 25; // Vertical threshold for IR signal
  const int sampleLength = 500; // Takes 500 samples       
  bool vals[sampleLength]; // Array for input
  int firstPulse;
  int edgeList[10];   // Starting time of the three relevant bits and the end burst 
  int pulseCount = 0;
  //int horzThresh = 15; // I have no idea what number this was supposed to be. It should be a number between the two possible edgeDistances.
  bool bitVal;
  int outVal = 0;
  
  unsigned long startTime = millis();
  const unsigned long timeWait = 1000;

  // Wait for IR reading to exceed threshold or time to exceed time threshold
  while(analogRead(irPin) <= threshold && millis() - startTime <= timeWait);
  // If wait is too long, time out and return -1
  if (millis() - startTime > timeWait)
  {
    return -1;
  }
  // Read IR data, threshold, and put into aray
  for (int i = 0; i < sampleLength; i++)
  {
    //Serial.println(analogRead(irPin));
    vals[i] = analogRead(irPin) > threshold;
  }
  // Pulse Counter
  pulseCount = 0;
  // Iterate over IR data array. Start at index 1, not index 0 to ignore first rising edge.
  for (int i = 1; i < sampleLength; i++)
  {
    // Detect a rising edge
    //Serial.println(i);
    if (vals[i] > vals[i - 1])
    {
      // Register rising edge as a pulse
      //Serial.println(i);
      pulseCount++;
      // Store sample number of pulse into edgeList
      // Since first pulse is pulse #6, subtract 6 to get to index 0 in edgeList
      edgeList[pulseCount] = i;
      //Serial.println("ledge");
    }
  }
  // Return -1 if wrong number of pulses detected
  if (pulseCount != 9)
  {
    //Serial.println("WOW");
    return -1;
  }
  for (int k = 1; k < 6; k++)
  {
    if (edgeList[k+1] - edgeList[k] >= horzThresh)
    {
      return -2;
    }
  }
  // Iterate over the 3 bits in edgeList
  for (int k = 6; k < 9; k++)
  {
    
    // Measure number of samples between rising edges, threshold, and store bit value
    bitVal = edgeList[k + 1] - edgeList[k] >= horzThresh;
    // Write bit value tou outVal int
    //Serial.println(bitVal);
    bitWrite(outVal, (2-k) + 6, bitVal);
  }
  //Serial.println("OWO");
  return outVal;
}
