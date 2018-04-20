#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

  // #define  0   //
  // #define  1   //
  #define DRIVE_SERVO_FL 2
  #define DRIVE_SERVO_FR 3
  #define DRIVE_SERVO_BL 4
  #define DRIVE_SERVO_BR 5
  #define LIFT_SERVO 6
  #define PLACEHOLDER_SERVO 7
/*  Need to be moved
#define WEDGE_SERVO 6
  #define FLAG_SERVO 7
  #define LCD_P3 8
  #define LCD_P2 9
  #define LCD_P1 10
  #define LCD_P0 11
  #define LCD_EN 13
  #define LCD_RS 12
  */
  #define STEPPER_P0 14
  #define STEPPER_P1 15
  #define STEPPER_P2 16
  #define STEPPER_P3 17
  #define ULTRASONIC_0_TRIG 18
  #define ULTRASONIC_0_ECHO 19
  #define ULTRASONIC_1_TRIG 20
  #define ULTRASONIC_1_ECHO 21
  #define ULTRASONIC_2_TRIG 22
  #define ULTRASONIC_2_ECHO 23
  #define ULTRASONIC_3_TRIG 24
  #define ULTRASONIC_3_ECHO 25
  #define ULTRASONIC_4_TRIG 26
  #define ULTRASONIC_4_ECHO 27
  #define ULTRASONIC_5_TRIG 28
  #define ULTRASONIC_5_ECHO 29
  #define ULTRASONIC_6_TRIG 30
  #define ULTRASONIC_6_ECHO 31
  #define ULTRASONIC_7_TRIG 32
  #define ULTRASONIC_7_ECHO 33
  // #define  34  //
  // #define  35  //
  // #define  36  //
  // #define  37  //
  // #define  38  //
  // #define  39  //
  // #define  40  //
  // #define  41  //
  // #define  42  //
  // #define  43  //
  // #define  44  //
  // #define  45  //
  // #define  46  //
  // #define  47  //
  // #define  48  //
  // #define  49  //
  // #define  50  //
  // #define  51  //
  // #define  52  //
  // #define  53  //
  #define SOLAR_CELL A0
  #define SHARP_0 A1
  #define SHARP_1 A2
  #define SHARP_2 A3
  #define SHARP_3 A4
  // #define  A5  //
  // #define  A6  //
  // #define  A7  //
  // #define  A8  //
  // #define  A9  //
  // #define  A10 //
  // #define  A11 //
  // #define  A12 //
  // #define  A13 //
  // #define  A14 //
  // #define  A15 //

  const int trigPin[8] = {18, 20, 22, 24, 26, 28, 30, 32};  // Pins for Ultrasonic Trigger
  const int echoPin[8] = {19. 21, 23, 25, 27, 29, 31, 33};  // Pins for Ultrasoinc Echo

#endif
