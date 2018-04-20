#ifndef COMMON_H
#define COMMON_H
#include <Arduino.h>

const int  FRONT_LEFT  = 0;
const int  LEFT_RIGHT  = 1;
const int  LEFT_LEFT   = 2;
const int  BACK_RIGHT  = 3;
const int  BACK_LEFT   = 4;
const int  RIGHT_RIGHT = 5;
const int  RIGHT_LEFT  = 6;
const int  FRONT_RIGHT = 7;

static int locationA;
static int locationB;
static int locationC;

static int state;
const int irPin = 2;          // Pin for Solar Panel

const int trigPin[8] = {22, 24, 26, 28, 30, 32, 34, 36}; // Pins for Ultrasonic Trigger
const int echoPin[8] = {23, 25, 27, 29, 31, 33, 35, 37}; // Pins for Ultrasoinc Echo
const int switchStop =  0;    // Pin for Switch Stop
const int switchStart = 1;   // Pin for Switch Start



double readSensorDistance(int sensorNumber);
void sensorSetup();




#endif
