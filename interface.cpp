void printFunctions(){
	Serial.println("0:  readSwitch ");
	Serial.println("1:  readSolarPanel ");
	Serial.println("2:  writePathLCD ");
	Serial.println("3:  goToPosition ");
	Serial.println("4:  readSensors ");
	Serial.println("5:  ramButton ");
	Serial.println("6:  moveToMiddle ");
	Serial.println("7:  moveDownTheRamp ");
	Serial.println("8:  goDownAndRightOrUpAndRight");
	Serial.println("9:  rotateToCorrectOrientation ");
	Serial.println("10: alignWithWedge ");
	Serial.println("11: wedgeServo ");
	Serial.println("12: moveTowardsWedge");
	Serial.println("13: backOffWedge ");
	Serial.println("14: moveInwards ");
	Serial.println("15: alignWithCenter ");
	Serial.println("16: rotateServoCounterClockwise ");
	Serial.println("17: driveForwardALittleBit ");
	Serial.println("18: stepperMotorRotate ");
	Serial.println("19: backoff ");
	Serial.println("20: rotateNinetyDegrees");
	Serial.println("21: driveTowardsTheBooty");
	Serial.println("22: lowerTheBootyMachine ");
	Serial.println("23: pullUpTheBooty ");
	Serial.println("24: inverseRotatNinetyDegrees");
	Serial.println("25: driveBackUpRamp");
	Serial.println("26: moveTowardsC");
	Serial.println("27: ramButton");
	Serial.println("28: shutOff ");
}


void interface(){
	Serial.println("Enter the number corresponding to the function. Enter -1 to have the list of functions and their number printed out");
	while(!Serial.available());
	int x = Serial.parseInt();
	if(x == -1) {
		printFunctions();
	} else {
		switch(x) {
			case(0):
				readSwitch();
				Serial.print("Switch state is: ");
				Serial.pritnln(state);
				break;
			case(1):
				Serial.print("Value: ");
				Serial.println(readSolarPanel());
				break;
			case(2):
				writePathLCD(path);
				break;
			case(3):
				Serial.println("Starting function");
				goToPosition(path);
				Serial.println("Function finished");
				break;
			case(4):
				readSensors();
				//Probably add a for loop or something to print the values out
				break;
			case(5):
				ramButton(); //May be absorbed elsewhere
				break;
			case(6):
				Serial.println("Starting function");
				moveToMiddle();
				Serial.println("Function finished");
				break;
			case(7):
				Serial.println("Starting function");
				moveDownTheRamp();
				Serial.println("Function finished");
				break;
			case(8):
				Serial.println("Starting  function");
				
				goDownAndRightOrUpAndRight();
				Serial.println("Function finished");
				break;
			case(9):
				Serial.println("Starting function");
				rotateToCorrectOrientation();
				Serial.println("Function finished");
				break;
			case(10):
				Serial.println("Starting function");
				alignWithWedge();
				Serial.println("Function finished");
				break;

			case(11):
				Serial.println("Starting function");
				wedgeServo();
				Serial.println("Function finished");
				break;

			case(12):
				Serial.println("Starting function");
				moveTowardsWedge();
				Serial.println("Function finished");
				break;
			case(13):
				Serial.println("Starting function");
				backOffWedge();
				Serial.println("Function finished");
				break;
			case(14):
				Serial.println("Starting function");
				moveInwards();
				Serial.println("Function finished");
				break;
			case(15):
				Serial.println("Starting function");
				alignWithCenter();
				Serial.println("Function finished");
				break;
			case(16):
				Serial.println("Starting function");
				rotateServoCounterClockwise();
				Serial.println("Function finished");
				break;
			case(17):
				Serial.println("Starting function");
				driveForwardALittleBit();
				Serial.println("Function finished");
				break;
			case(18):
				Serial.println("Starting function");
				stepperMotorRotate();
				Serial.println("Function finished");
				break;
			case(19):
				Serial.println("Starting function");
				backoff();
				Serial.println("Function finished");
				break;
			case(20):
				Serial.println("Starting function");
				rotateNinetyDegrees();
				Serial.println("Function finished");
				break;
			case(21):
				Serial.println("Starting function");
				driveTowardsTheBooty();
				Serial.println("Function finished");
				break;
			case(22):
				Serial.println("Starting function");
				lowerTheBootyMachine();
				Serial.println("Function finished");
				break;
			case(23):
				Serial.println("Starting function");
				pullUpTheBooty();
				Serial.println("Function finished");
				break;
			case(24):
				Serial.println("Starting function");
				inverseRotateNintyDegrees();
				Serial.println("Function finished");
				break;
			case(25):
				Serial.println("Starting function");
				driveBackUpRamp();
				Serial.println("Function finished");
				break;
			case(26):
				Serial.println("Starting function");
				moveTowardsC();
				Serial.println("Function finished");
				break;
			case(27):
				Serial.println("Starting function");
				ramButton();
				Serial.println("Function finished");
				break;
			case(28):
				Serial.println("Shutting stuff of");
				shutoff();
				Serial.println("Shut off");
				break;
			default:
				Serial.println("you stupid");
				break;
		}
	}
}
