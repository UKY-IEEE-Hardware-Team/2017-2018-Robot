// MASTER FILE
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
const int switchStop = 34;
const int switchStart = 35;

void setup() {
  // put your setup code here, to run once:
  pinMode(switchStop, INPUT_PULLUP);
  pinMode(switchStart, INPUT_PULLUP);
}

void loop() {
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

