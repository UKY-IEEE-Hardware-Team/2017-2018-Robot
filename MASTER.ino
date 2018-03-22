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
