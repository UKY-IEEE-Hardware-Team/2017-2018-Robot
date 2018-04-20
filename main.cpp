#include <Arduino.h>
#include "common.h"
#include "chris.h"
#include "robot_turning.h"
#include "thomas.h"

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    servosetup();
    sensorSetup();
}

void printFunctions(){
  Serial.println("3: goToPosition");
  Serial.println("4: ramButton");
  Serial.println("5: distasnceFromMiddle");
  Serial.println("6: moveToMiddle");
  Serial.println("7: moveRightAlongWall");
  Serial.println("8: moveInwards");
  Serial.println("9: rotateToPerpindicular");
  Serial.println("10: alignWithCenter");
  Serial.println("11: drive");
}



void interface(){
  Serial.println("Enter your function number (-1 to print functions out)");
  while(!Serial.available());
  int x = Serial.parseInt();
  int y, z;
  while(Serial.available()) Serial.read();
  if(x == -1){
    printFunctions();
  } else {
    switch (x){
      // case(0):
			// 	readSwitch();
			// 	Serial.print("Switch state is: ");
			// 	Serial.println(state);
			// 	break;
			// case(1):
			// 	Serial.print("Value: ");
			// 	Serial.println(readSolarPanel());
			// 	break;
      // case(2):
  		// 	writePathLCD(path);
  		// 	break;
      case(3):
        Serial.print("0 for up or 1 for down");
        while(!Serial.available());
        y = Serial.parseInt();
        if(y==0||y==1){
          locationA = y;
          goToPosition(y);
        }
        break;
      case(4):
        ramButton();
        break;
      case(5):
        Serial.print("0 for front/back, 1 for left/right");
        while(!Serial.available());
        y = Serial.parseInt();
        if(y==0||y==1){
          Serial.print("Distance from middle: ");Serial.println(distanceFromMiddle(y));
        }
        break;
      case(6):
        Serial.println("starting to move to middle");
        moveToMiddle();
        Serial.println("finished moving to middle");
        break;
      case(7):
        Serial.print("0 for top or 1 for bottom");
        while(!Serial.available());
        y = Serial.parseInt();
        if(y==0||y==1){
          locationB = y;
          Serial.println("Starting to move along wall");
          moveRightAlongWall();
          Serial.println("Finishing moving");
        }
        break;
      case(8):
        Serial.println("starting to move inwards towards flag");
        moveInwards();
        Serial.println("finished moving inwards towards flag");
        break;
      case(9):
        Serial.print("0 for front, 1 for left, 2 for back, 3 for right");
        while(!Serial.available());
        y = Serial.parseInt();
        if(y>=0&&y<=3){
          Serial.println("starting to rotate");
          rotateToPerpindicular(y);
          Serial.println("finished rotating");
        }
        break;
      case(10):
        Serial.println("aligning with center");
        alignWithCenter();
        Serial.println("finished aligning with center");
        break;
      case(11):
        Serial.print("Enter a number between 0 and 10");
        while(!Serial.available());
        y = Serial.parseInt();
        if(y>=0 && y <=10){
          drive(y);
          delay(2000);
          drive(10);
        }
        break;


    }
  }
}

void loop() {
  interface();
    // drive(2);
    // pinMode(4,OUTPUT);
    // pinMode(5,OUTPUT);
    // pinMode(6,OUTPUT);
    // pinMode(7,OUTPUT);
    // delay(1000);
    // pinMode(4,INPUT);
    // pinMode(5,INPUT);
    // pinMode(6,INPUT);
    // pinMode(7,INPUT);
    // // digitalWrite(4,LOW);
    // // digitalWrite(5,LOW);
    // // digitalWrite(6,LOW);
    // // digitalWrite(7,LOW);
    // //drive(10);
    // delay(1000);
    // put your main code here, to run repeatedly:
}
