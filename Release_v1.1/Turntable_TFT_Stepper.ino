/*
Name:    Turntable_TFT_Stepper.ino
Created: 4/20/2017 12:21:05 PM
Author:  jimsanderson
*/

//#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>

#include <DCC_Decoder.h>

#include <eeprom.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include <Adafruit_GFX.h>// Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
//#include <TFT_Extension.h>
#include <TouchScreen.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSerifItalic12pt7b.h>
#include <Fonts/FreeSerifItalic18pt7b.h>
#include <Fonts/FreeSerifItalic24pt7b.h>
#include <Fonts/FreeSerifItalic9pt7b.h>

//    >>>>    START     ------------------------------   TFT Setup    ------------------------------
#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

////2.8inch_280-5
//#define TS_MINX 180
//#define TS_MINY 170
//#define TS_MAXX 920
//#define TS_MAXY 930
//#define MINPRESSURE 10
//#define MAXPRESSURE 1000

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4

// Assign human-readable names to some common 16-bit color values:
#define BLACK       0x0000      /*   0,   0,   0 */
#define NAVY        0x000F      /*   0,   0, 128 */
#define DARKGREEN   0x03E0      /*   0, 128,   0 */
#define DARKCYAN    0x03EF      /*   0, 128, 128 */
#define MAROON      0x7800      /* 128,   0,   0 */
#define PURPLE      0x780F      /* 128,   0, 128 */
#define OLIVE       0x7BE0      /* 128, 128,   0 */
#define LIGHTGREY   0xC618      /* 192, 192, 192 */
#define DARKGREY    0x7BEF      /* 128, 128, 128 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define CYAN        0x07FF      /*   0, 255, 255 */
#define RED         0xF800      /* 255,   0,   0 */
#define MAGENTA     0xF81F      /* 255,   0, 255 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */
#define ORANGE      0xFD20      /* 255, 165,   0 */
#define GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define PINK        0xF81F
#define GREY		0x8410

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
//    >>>>    FINISH    ------------------------------   TFT Setup    ------------------------------

//    >>>>    START     --------------------------   Define DCC Control   --------------------------
#define kDCC_INTERRUPT  0 // Define DCC commands to Arduino
typedef struct { int address; }
DCCAccessoryAddress;  // Address to respond to
DCCAccessoryAddress gAddresses[7]; // Allows 7 DCC addresses: [XX] = number of addresses you need (including 0).
								   //    >>>>    FINISH    --------------------------   Define DCC Control   --------------------------

								   //    >>>>    START     ------------------------   Track Step Definitions   ------------------------

								   //Define Corresponding Head and Tail   UNO
const int GetTrackTail[6] = { 4, 5, 6, 1, 2, 3 }; // Array for Tail Tracks
int PositionTrack[7] = { 0, 0, 0, 0, 0, 0, 0 };   // Save EEPROM addresses to array - index 0 = calibration position
int PositionTrackDummy[7] = { 0, 560, 800, 1040, 2160, 2400, 2640 };
int storeTargetTrack = 0;             // Store Target Track position
int storeStartTrack = 0;

String str1, str2, str3, str4, str5, str6;  // used as String builders within voids

											//  Calibration Array
int sensorVal = digitalRead(3);         // enable Hall-effect Sensor on Pin 3
int arrayCalibrate[] = { 0, 0, 0, 0, 0 };   // Array to pass in calibration run results
boolean isTrackCalibration = false;

// Programme Track Positions
int currentStepPosition = 0;  // current step number
int storeProgTracks[] = { 0, 0, 0, 0, 0, 0 };
boolean chkOverwrite = false;

// Debug Variables
boolean isDebugMode = false;			// Set debug to console default is off
										// Parameters for Stepper
boolean isReleased = false;				// isReleased tries to make sure the motor is not continuously released
long stepperLastMoveTime = 0;
int mainDiff = 0;
const int MOTOR_OVERSHOOT = 10;			// the amount of overshoot/ lash correction when approaching from CCW
int overshootDestination = -1;
const int releaseTimeout_ms = 2000;		//reduced to 2 seconds for now
const int  totalSteps = 200 * 16; //number of steps for a full rotation
								  //    >>>>    FINISH    ------------------------   Track Step Definitions   ------------------------

								  //    >>>>    START     --------------------   Parameters for turntable move    --------------------

boolean programmingMode = false;
boolean tableTargetHead = false;
int tableTargetTrack = 0;
int tableTargetPosition = 0;
boolean newTargetLocation = false;
boolean inMotionToNewTarget = false;
boolean isTurntableHead = true;
boolean isTurntableHeadChanged = true;
int currentTrack = 1;
int newTrack = 1;
int distanceToGo = 0;
const int displayRotateDelay = 5;   // This is the minimum delay in ms between steps of the stepper motor
boolean displayRotating = false;    // Is the "Display Rotate" function enabled?
boolean displayRotatingCW = true;   // In Display Rotate mode, are we rot
									//    >>>>    FINISH    --------------------   Parameters for turntable move    --------------------

									//    >>>>    START     -------------------------------   TFT Menu   -------------------------------
int radius = 75;
int xCentre = tft.height() / 2;
int yCentre = (tft.width() / 2);
int turntableOffset = 25;
int topOld1x, topOld2x, botOld1x, botOld2x, topOld1y, topOld2y, botOld1y, botOld2y;
int topOld2, botOld1, topOld1, botOld2;
int butX, butY;

int sleep = 0; // sleep counter
int sleepTimer = 30000; // sleep after 30 secs
boolean pause = true;
int startAngle = 270;
int menuPage = 0;

//    >>>>    FINISH    -------------------------------   TFT Menu   -------------------------------

//    >>>>    START     ----------------------   Adafruit MotorShield Setup   ----------------------

Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers
Adafruit_StepperMotor *mystepper = AFMStop.getStepper(200, 2);   //Connect stepper with 200 steps per revolution (1.8 degree) to the M3, M4 terminals (blue,yellow,green,red)
																 //you can change these to SINGLE, DOUBLE, INTERLEAVE or MICROSTEP! wrapper for the motor!(3200 Microsteps / revolution)
void forwardstep2() { mystepper->onestep(BACKWARD, MICROSTEP); }
void backwardstep2() { mystepper->onestep(FORWARD, MICROSTEP); }
void release2() { mystepper->release(); }
AccelStepper stepper = AccelStepper(forwardstep2, backwardstep2); // wrap the stepper in an AccelStepper object
																  //int forwardstep2() { return dummyStepper(1, 25); } //75 real
																  //int backwardstep2() { return dummyStepper(-1, 25); } //75 real

																  //    <<<<    FINISH    ----------------------   Adafruit MotorShield Setup   ----------------------

																  //    >>>>    START     --------------------------------   SETUP    --------------------------------
void setup()
{
	Serial.begin(9600);

	Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());
	initialiseDCC();
	tft.reset();
	tft.begin(0x9325);

	tft.setRotation(3);
	tft.fillScreen(BLACK);

	pinMode(13, OUTPUT);

	tft.setCursor(5, 15);
	tft.setTextColor(WHITE);
	tft.setTextSize(1);
	tft.setFont(&FreeSerifItalic12pt7b);
	tft.setCursor(30, yCentre);
	tft.println("DCC Controlled Turntable");
	tft.setFont();
	delay(1000);
	tft.fillScreen(BLACK);
	MenuTabs(1);
	drawTurntable(radius, GREY);
	//drawTracks(false);
	delay(3000);
}
//    >>>>    FINISH    --------------------------------   SETUP    --------------------------------

//    >>>>    START     ---------------------------------   LOOP   ---------------------------------
void loop()
{
	sleepTimer++;
	sleepTFT();
	//delay(3000);

	//digitalWrite(13, HIGH);
	//TSPoint p = ts.getPoint();
	//digitalWrite(13, LOW);

	//drawTracks(false);

	while (pause)
	{
		//		drawTurntable(radius, GREY);

		for (int i = 0; i < 90; i++)
		{
			//tft.fillCircle(xCentre, yCentre, radius - 5, BLACK);
			delay(10);
			//drawPointAroundCircle(radius - 10, i);
			calcTurntablePoints((i - 1), false);
			calcTurntablePoints(i, true);
			delay(100);
		}



		tft.fillScreen(BLACK); // turn off screen
		pause = false; //kill loop
	}
}
//    >>>>    FINISH    ---------------------------------   LOOP   ---------------------------------

//    >>>>    START     --------------------------    EEPROM Commands     --------------------------

void EEPROMWritelong(int address, int value)
{
	if (isTrackCalibration)
		address = 0;
	else
		address = address * 2;

	int val1 = value / 100;
	int val2 = value % 100;

	EEPROM.write(address, val1);
	EEPROM.write(address + 1, val2);
}

void arrayWritelong(int address, int value)
{
	if (isTrackCalibration) {
		address = 0;
	}
	int val1 = value / 100;
	int val2 = value % 100;
	int valueJoin = (val1 * 100) + val2;
	PositionTrack[address] = valueJoin;
}

int EEPROMReadlong(int address)
{
	if (isTrackCalibration)
		address = 0;
	else
		address = address * 2;

	return (EEPROM.read(address) * 100) + EEPROM.read(address + 1);
}

void readArrayEEPROM()
{
	int returnArray = 0;
	for (int i = 0; i < 7; i++)
	{
		returnArray = EEPROMReadlong(i);
		PositionTrack[i] = returnArray;
	}
}

void clearEEPROM()
{
	int c = 0;
	for (int i = 0; i < 7; i++)
	{
		if (EEPROMReadlong(i) != 0)
		{
			c = i * 2;
			EEPROM.write(c, 0);
			EEPROM.write(c + 1, 0);
		}
	}
}

//    <<<<    FINISH    --------------------------    EEPROM Commands     --------------------------

//    >>>>    START     -------------------------    DCC Decoder Setup     -------------------------

void initialiseDCC()
{
	DCC.SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket_Handler, true);
	ConfigureDecoder();
	DCC.SetupDecoder(0x00, 0x00, kDCC_INTERRUPT);
}

void ConfigureDecoder()
{ //Put all the decoder #'s you need here.	Remember to change
  //DCCAccessoryAddress gAddresses[XX];(above) where XX = number of addresses you need.
	gAddresses[0].address = 200;
	gAddresses[1].address = 201;
	gAddresses[2].address = 202;
	gAddresses[3].address = 203;
	gAddresses[4].address = 204;
	gAddresses[5].address = 205;
	gAddresses[6].address = 206;
	gAddresses[7].address = 207;
}

void BasicAccDecoderPacket_Handler(int address, boolean activate, byte data)	// Basic accessory packet handler
{
	// Convert NMRA packet address format to human address
	address -= 1;
	address *= 4;
	address += 1;
	address += (data & 0x06) >> 1;

	boolean enable = (data & 0x01) ? 1 : 0;

	for (int i = 0; i < (int)(sizeof(gAddresses) / sizeof(gAddresses[0])); i++)
	{
		if (address == gAddresses[i].address)
		{
			tableTargetHead = enable;
			tableTargetPosition = i;
			//New packet and we have a new target location, set the flag
			newTargetLocation = true;
			doStepperMove();
		}
	}
}

void autoDCCMode()
{
	int addr = 0;
	DCC.loop();
	// Bump to next address to test
	if (++addr >= (int)(sizeof(gAddresses) / sizeof(gAddresses[0]))) { addr = 0; }

	stepperTimer();
}

//    <<<<    FINISH    -------------------------    DCC Decoder Setup     -------------------------

void drawTurntable(int radius, int colour)
{
	tft.fillCircle(xCentre, yCentre + turntableOffset, radius, colour);
	tft.fillCircle(xCentre, yCentre + turntableOffset, radius - 4, BLACK);
	drawTracks(true,true);
}

void calcTurntablePoints(int angle, boolean show)
{
	angle = angle - 90;
	int top1, top2, bot1, bot2;
	int radTurn = radius - 7;

	int dispCol = GREY;	int dispCol1 = GREEN;	int dispCol2 = RED;

	if (show != true)
	{
		dispCol = BLACK; dispCol1 = BLACK; 	dispCol2 = BLACK;
	}

	if (angle > 270 || angle < -90) angle = 270;

	top1 = ((360 - 3) + angle);
	top2 = ((3 + angle) + 180);
	bot1 = (3 + angle);
	bot2 = (((360 - 3) + angle) - 180);

	drawTrackLine(top1, top2, radTurn, radTurn, 0, dispCol);
	drawTrackLine(top1, top2, radTurn, radTurn, -1, dispCol);
	drawTrackLine(bot1, bot2, radTurn, radTurn, 0, dispCol);
	drawTrackLine(bot1, bot2, radTurn, radTurn, 1, dispCol);
	drawTrackLine(top1, bot1, radTurn, radTurn, 0, dispCol);
	drawTrackLine(top2, bot2, radTurn, radTurn, 0, dispCol);

	drawMarker((360 + angle), radTurn - 4, 2, dispCol1);
	drawMarker((360 + angle) - 180, radTurn - 4, 2, dispCol2);
}

void drawTrackLine(int p1, int p2, int radius1, int radius2, int offset, int colour)
{
	int aX = xCentre + radius1 * cos(p1 * PI / 180);
	int aY = yCentre + turntableOffset + radius1 * sin(p1 * PI / 180);
	int bX = xCentre + radius2 * cos(p2 * PI / 180);
	int bY = yCentre + turntableOffset + radius2 * sin(p2 * PI / 180);

	butX =  xCentre + radius1 * cos(p1 * PI / 180);
	butY = yCentre +  radius1 * sin(p1 * PI / 180);

	tft.drawLine(aX, aY + offset, bX, bY + offset, colour);

}

void drawMarker(int p1, int radius, int size, int colour)
{
	int aX = xCentre + radius * cos(p1 * PI / 180);
	int aY = yCentre + turntableOffset + radius * sin(p1 * PI / 180);
	tft.fillCircle(aX, aY, size, colour);
}

void drawTracks(boolean isTrackCalibration, boolean isButton)
{
	int i = 1;
	int lengthTrack = 30;
	int offset = 3;
	
	if (isTrackCalibration == true) i = 0;

	for (i; i < sizeof(PositionTrackDummy) / sizeof(PositionTrackDummy[0]); i++)
	{
		Serial.println(sizeof(PositionTrackDummy)/ sizeof(PositionTrackDummy[0]));
		Serial.println(i);
		int angle = convertStepDeg(PositionTrackDummy[i], false);

		int p1 = (360 - angle);
		drawTrackLine(p1, p1, radius, radius + lengthTrack, -3, GREY);
		drawTrackLine(p1, p1, radius, radius + lengthTrack, +3, GREY);

		if (isButton == true)
		{
			createButtons(butX, butY, GREENYELLOW, "", false);
		}

	}
}

float convertStepDeg(float unit, boolean toSteps)
{
	float step = (float(totalSteps) / 360)*unit;
	float deg = 90 - (unit / float(totalSteps) * 360);
	Serial.println(deg);

		if (toSteps == true)
		return step;
	else
		return deg;	
}

//    >>>>    START     ----------------------------   Menu Functions   ----------------------------	
void MenuTabs(int tabSelected)
{
	int tabs = 5;
	int tabHeight = 20;
	int tabPad = 3;
	int sidePadding = 14;
	int menuBorder = 6;

	int tabX;
	int tabY = menuBorder;

	int lastTabX = 0;
	int tabWidth = (tft.width() - ((sidePadding * 2) + (tabPad * (tabs - 1))))/tabs;

	for (int i = 0; i < tabs; i++)
	{
		if (i == 0)
			tabX = sidePadding;
		else
			tabX = lastTabX + tabWidth + tabPad;
		if (tabSelected == i)
			tft.fillRect(tabX, tabY, tabWidth, tabHeight, LIGHTGREY);
		else
			tft.fillRect(tabX, tabY, tabWidth, tabHeight, DARKGREY);

		lastTabX = tabX;
	}

	tft.fillRoundRect(menuBorder, menuBorder+tabHeight, tft.width() - (menuBorder*2), tft.height() - tabHeight- (menuBorder*2), 3, LIGHTGREY);
	tft.fillRoundRect(menuBorder+tabPad, menuBorder + tabHeight + tabPad, tft.width() - ((menuBorder + tabPad)*2),  tft.height() - tabHeight-((menuBorder + tabPad) * 2),3,BLACK);

}

void MainMenu()
{

}

void CalibrateMenu()
{

}

void ManualMove()
{}

void sleepTFT()
{

	if (sleep == sleepTimer)
	{
		tft.fillScreen(BLACK);
	}
	else sleep++;
}

void determineButtonLocation()
{
	//

}

void createButtons(int posX, int posY, int colour, String buttonText, boolean isActive)
{
	int defHeight = 30;
	int defWidth = 30;
	int radCorner = 4;

	if (isActive == true) colour = ORANGE;

	tft.fillRoundRect(posX, posY - 20, defHeight, defWidth, radCorner, colour);
	tft.drawRoundRect(posX - 2, posY - 2, defHeight - 2, defWidth - 2, radCorner, BLACK);
	tft.setCursor(posX + (defHeight / 2 - 5), posY + (defWidth / 2));
	tft.setFont(&FreeSansBold9pt7b);
	tft.print(buttonText);
	tft.setFont();

}

void touchButton()
{
}

//    >>>>    FINISH    ----------------------------   Menu Functions   ----------------------------

//    <<<<    START     ----------------------    Manually Move Turntable     ----------------------

//    <<<<    FINISH    ----------------------    Manually Move Turntable     ----------------------

//    >>>>    START     -----------------------    Check Track Positions     -----------------------

//    <<<<    FINISH    -----------------------    Check Track Positions     -----------------------

//    >>>>    START     ---------------    Select And Programme Turntable Tracks     ---------------

//    <<<<    FINISH    ---------------    Select And Programme Turntable Tracks     ---------------

//    >>>>    START     --------------------------    Calibrate Bridge    --------------------------

//    <<<<    FINISH    --------------------------    Calibrate Bridge    --------------------------

//    <<<<    START     ---------------------------    Stepper Voids     ---------------------------
void doStepperMove()
{
	stepper.run();	// Run the Stepper Motor
	boolean isInMotion = (abs(stepper.distanceToGo()) > 0);
	boolean newTargetSet = false;

	// If there is a new target location, set the target
	if (newTargetLocation)
	{
		SetStepperTargetLocation();
		//          displayOutput("Moving to ", ""));
		newTargetSet = true;
	}

	if ((currentStepPosition % totalSteps) == 0)
	{
		//setCurrentPosition seems to always reset the position to 0, ignoring the parameter
		str1 = String(F("Current location: "));
		//				str2 = String(stepper.currentPosition());
		str2 = String(currentStepPosition);
		str3 = String(F(" % STEPCOUNT.	Why here?"));
		//displayOutput(true, str1 + str2, str3);
	}

	if (mainDiff < 0) { displayRotatingCW = false; }
	else if (mainDiff > 0) { displayRotatingCW = true; }

}
//    ----------------------------------------------------------------------------------------------
//
// Subroutine: SetStepperTargetLocation()
//	Takes the global variables: tableTargetHeadOrTail, and tableTargetPosition, and sets the stepper
//	object moveTo() target position in steps-	inserts values back into "doStepperMove()"
//
//    ----------------------------------------------------------------------------------------------

void SetStepperTargetLocation()
{
	int newTargetLoc = -1;
	if (tableTargetHead)
	{	//use head location variable	
		newTargetLoc = PositionTrack[tableTargetPosition];
		inMotionToNewTarget = true;
	}
	else
	{	//use tail location variable	
		newTargetLoc = PositionTrack[tableTargetPosition];
		inMotionToNewTarget = true;
	}

	if (newTargetLoc > 0)
	{
		//int currentLoc = stepper.currentPosition();
		int currentLoc = currentStepPosition;
		int mainDiff = newTargetLoc - currentLoc;

		if (mainDiff > (totalSteps / 2))
			mainDiff = mainDiff - totalSteps;
		else
			mainDiff = mainDiff + totalSteps;
	}

	if (mainDiff < 0)
	{
		mainDiff -= MOTOR_OVERSHOOT;
		overshootDestination = MOTOR_OVERSHOOT;
	}
	//			stepper.move(mainDiff);
	//programmingMode = false;
	newTargetLocation = false;
}
//    ----------------------------------------------------------------------------------------------
//	Stepper Timer sub routine this runs from the main loop. It also supports the release function.
//    ----------------------------------------------------------------------------------------------

void stepperTimer()
{
	int currentLoc = 0;

	// Run the Stepper Motor 
	stepper.run();
	boolean isInMotion = (abs(stepper.distanceToGo()) > 0);

	//Check if we have any distance to move for release() timeout.	Can check the
	// buffered var isInMotion because we also check the other variables.
	if (isInMotion || programmingMode)
	{
		//We still have some distance to move, so reset the release timeout
		stepperLastMoveTime = millis();
		isReleased = false;
	}
	else
	{
		if (!isReleased)
		{
			if (overshootDestination > 0)
			{
				stepper.move(overshootDestination);
				overshootDestination = -1;
			}

			if (((millis() - stepperLastMoveTime) >= releaseTimeout_ms))
			{
				//If isReleased, don't release again.
				isReleased = true;
				int currentLoc = stepper.currentPosition();	// Resets the position to the actual positive number it should be
				currentLoc = currentLoc % totalSteps;

				if (currentLoc < 0)
					currentLoc += totalSteps;

				stepper.setCurrentPosition(currentLoc);
				stepper.moveTo(currentLoc);

				//String(F("	Actual Current Position: "));
				//str2 = String(stepper.currentPosition());	// shows the position value corrected.

				//Set the servo brake
				//	brakeservo.write(servoBrake);
				//	delay(750);

				//release the motor
				//                  release2();
				// str1 = String(F("	Brake Set & Motor Released "));

			}
		}
	}
}

void calcLeastSteps(int trA, int trB)
{
	int currentLoc = PositionTrack[trA];
	int newTargetLoc = PositionTrack[trB];
	int getMotorStepCount = (totalSteps / 2);

	if (newTargetLoc > 0)
	{
		int currentLoc = stepper.currentPosition();
		mainDiff = newTargetLoc - currentLoc;
		if (mainDiff > getMotorStepCount) { mainDiff = mainDiff - totalSteps; }
		else if (mainDiff < -getMotorStepCount) { mainDiff = mainDiff + totalSteps; }

		if (mainDiff < 0)
		{
			mainDiff -= MOTOR_OVERSHOOT;
			overshootDestination = MOTOR_OVERSHOOT;
		}

		if (mainDiff < 0) { displayRotatingCW = false; }
		else if (mainDiff > 0) { displayRotatingCW = true; }

		String textMove;
		if (displayRotatingCW) { textMove = "CW"; }
		else { textMove = "CCW"; }

		stepper.move(mainDiff);
	}
}
//    <<<<    FINISH    ---------------------------    Stepper Voids     ---------------------------/*
Name:    Turntable_TFT_Stepper.ino
Created: 4/20/2017 12:21:05 PM
Author:  jimsanderson
*/

//#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>

#include <DCC_Decoder.h>

#include <eeprom.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include <Adafruit_GFX.h>// Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
//#include <TFT_Extension.h>
#include <TouchScreen.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSerifItalic12pt7b.h>
#include <Fonts/FreeSerifItalic18pt7b.h>
#include <Fonts/FreeSerifItalic24pt7b.h>
#include <Fonts/FreeSerifItalic9pt7b.h>

//    >>>>    START     ------------------------------   TFT Setup    ------------------------------
#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

////2.8inch_280-5
//#define TS_MINX 180
//#define TS_MINY 170
//#define TS_MAXX 920
//#define TS_MAXY 930
//#define MINPRESSURE 10
//#define MAXPRESSURE 1000

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4

// Assign human-readable names to some common 16-bit color values:
#define BLACK       0x0000      /*   0,   0,   0 */
#define NAVY        0x000F      /*   0,   0, 128 */
#define DARKGREEN   0x03E0      /*   0, 128,   0 */
#define DARKCYAN    0x03EF      /*   0, 128, 128 */
#define MAROON      0x7800      /* 128,   0,   0 */
#define PURPLE      0x780F      /* 128,   0, 128 */
#define OLIVE       0x7BE0      /* 128, 128,   0 */
#define LIGHTGREY   0xC618      /* 192, 192, 192 */
#define DARKGREY    0x7BEF      /* 128, 128, 128 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define CYAN        0x07FF      /*   0, 255, 255 */
#define RED         0xF800      /* 255,   0,   0 */
#define MAGENTA     0xF81F      /* 255,   0, 255 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */
#define ORANGE      0xFD20      /* 255, 165,   0 */
#define GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define PINK        0xF81F
#define GREY		0x8410

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
//    >>>>    FINISH    ------------------------------   TFT Setup    ------------------------------

//    >>>>    START     --------------------------   Define DCC Control   --------------------------
#define kDCC_INTERRUPT  0 // Define DCC commands to Arduino
typedef struct { int address; }
DCCAccessoryAddress;  // Address to respond to
DCCAccessoryAddress gAddresses[7]; // Allows 7 DCC addresses: [XX] = number of addresses you need (including 0).
								   //    >>>>    FINISH    --------------------------   Define DCC Control   --------------------------

								   //    >>>>    START     ------------------------   Track Step Definitions   ------------------------

								   //Define Corresponding Head and Tail   UNO
const int GetTrackTail[6] = { 4, 5, 6, 1, 2, 3 }; // Array for Tail Tracks
int PositionTrack[7] = { 0, 0, 0, 0, 0, 0, 0 };   // Save EEPROM addresses to array - index 0 = calibration position
int PositionTrackDummy[7] = { 0, 710, 800, 890, 2310, 2400, 2490 };
int storeTargetTrack = 0;             // Store Target Track position
int storeStartTrack = 0;

String str1, str2, str3, str4, str5, str6;  // used as String builders within voids

											//  Calibration Array
int sensorVal = digitalRead(3);         // enable Hall-effect Sensor on Pin 3
int arrayCalibrate[] = { 0, 0, 0, 0, 0 };   // Array to pass in calibration run results
boolean isTrackCalibration = false;

// Programme Track Positions
int currentStepPosition = 0;  // current step number
int storeProgTracks[] = { 0, 0, 0, 0, 0, 0 };
boolean chkOverwrite = false;

// Debug Variables
boolean isDebugMode = false;			// Set debug to console default is off
										// Parameters for Stepper
boolean isReleased = false;				// isReleased tries to make sure the motor is not continuously released
long stepperLastMoveTime = 0;
int mainDiff = 0;
const int MOTOR_OVERSHOOT = 10;			// the amount of overshoot/ lash correction when approaching from CCW
int overshootDestination = -1;
const int releaseTimeout_ms = 2000;		//reduced to 2 seconds for now
const int  totalSteps = 200 * 16; //number of steps for a full rotation
								  //    >>>>    FINISH    ------------------------   Track Step Definitions   ------------------------

								  //    >>>>    START     --------------------   Parameters for turntable move    --------------------

boolean programmingMode = false;
boolean tableTargetHead = false;
int tableTargetTrack = 0;
int tableTargetPosition = 0;
boolean newTargetLocation = false;
boolean inMotionToNewTarget = false;
boolean isTurntableHead = true;
boolean isTurntableHeadChanged = true;
int currentTrack = 1;
int newTrack = 1;
int distanceToGo = 0;
const int displayRotateDelay = 5;   // This is the minimum delay in ms between steps of the stepper motor
boolean displayRotating = false;    // Is the "Display Rotate" function enabled?
boolean displayRotatingCW = true;   // In Display Rotate mode, are we rot
									//    >>>>    FINISH    --------------------   Parameters for turntable move    --------------------

									//    >>>>    START     -------------------------------   TFT Menu   -------------------------------
int radius = 75;
int xCentre = tft.height() / 2;
int yCentre = tft.width() / 2;

int topOld1x, topOld2x, botOld1x, botOld2x, topOld1y, topOld2y, botOld1y, botOld2y;
int topOld2, botOld1, topOld1, botOld2;

int sleep = 0; // sleep counter
int sleepTimer = 30000; // sleep after 30 secs
boolean pause = true;

int menuPage = 0;

//    >>>>    FINISH    -------------------------------   TFT Menu   -------------------------------

//    >>>>    START     ----------------------   Adafruit MotorShield Setup   ----------------------

Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers
Adafruit_StepperMotor *mystepper = AFMStop.getStepper(200, 2);   //Connect stepper with 200 steps per revolution (1.8 degree) to the M3, M4 terminals (blue,yellow,green,red)
																 //you can change these to SINGLE, DOUBLE, INTERLEAVE or MICROSTEP! wrapper for the motor!(3200 Microsteps / revolution)
void forwardstep2() { mystepper->onestep(BACKWARD, MICROSTEP); }
void backwardstep2() { mystepper->onestep(FORWARD, MICROSTEP); }
void release2() { mystepper->release(); }
AccelStepper stepper = AccelStepper(forwardstep2, backwardstep2); // wrap the stepper in an AccelStepper object
																  //int forwardstep2() { return dummyStepper(1, 25); } //75 real
																  //int backwardstep2() { return dummyStepper(-1, 25); } //75 real

																  //    <<<<    FINISH    ----------------------   Adafruit MotorShield Setup   ----------------------

																  //    >>>>    START     --------------------------------   SETUP    --------------------------------
void setup()
{
	Serial.begin(9600);

	Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());
	initialiseDCC();
	tft.reset();
	tft.begin(0x9325);

	tft.setRotation(3);
	tft.fillScreen(BLACK);

	pinMode(13, OUTPUT);

	tft.setCursor(5, 15);
	tft.setTextColor(WHITE);
	tft.setTextSize(1);
	tft.setFont(&FreeSerifItalic12pt7b);
	tft.setCursor(30, yCentre);
	tft.println("DCC Controlled Turntable");
	tft.setFont();
	delay(1000);
	tft.fillScreen(BLACK);
	//MenuTabs();

	delay(3000);
}
//    >>>>    FINISH    --------------------------------   SETUP    --------------------------------

//    >>>>    START     ---------------------------------   LOOP   ---------------------------------
void loop()
{
	//delay(3000);

	//digitalWrite(13, HIGH);
	//TSPoint p = ts.getPoint();
	//digitalWrite(13, LOW);

	while (pause)
	{
		//		drawTurntable(radius, GREY);

		for (int i = 0; i < 90; i++)
		{
			//tft.fillCircle(xCentre, yCentre, radius - 5, BLACK);
			delay(10);
			//drawPointAroundCircle(radius - 10, i);
			calcTurntablePoints((i - 1), false);
			calcTurntablePoints(i, true);
			delay(100);
		}

		tft.fillScreen(BLACK); // turn off screen
		pause = false; //kill loop
	}
}
//    >>>>    FINISH    ---------------------------------   LOOP   ---------------------------------

//    >>>>    START     --------------------------    EEPROM Commands     --------------------------

void EEPROMWritelong(int address, int value)
{
	if (isTrackCalibration)
		address = 0;
	else
		address = address * 2;

	int val1 = value / 100;
	int val2 = value % 100;

	EEPROM.write(address, val1);
	EEPROM.write(address + 1, val2);
}

void arrayWritelong(int address, int value)
{
	if (isTrackCalibration) {
		address = 0;
	}
	int val1 = value / 100;
	int val2 = value % 100;
	int valueJoin = (val1 * 100) + val2;
	PositionTrack[address] = valueJoin;
}

int EEPROMReadlong(int address)
{
	if (isTrackCalibration)
		address = 0;
	else
		address = address * 2;

	return (EEPROM.read(address) * 100) + EEPROM.read(address + 1);
}

void readArrayEEPROM()
{
	int returnArray = 0;
	for (int i = 0; i < 7; i++)
	{
		returnArray = EEPROMReadlong(i);
		PositionTrack[i] = returnArray;
	}
}

void clearEEPROM()
{
	int c = 0;
	for (int i = 0; i < 7; i++)
	{
		if (EEPROMReadlong(i) != 0)
		{
			c = i * 2;
			EEPROM.write(c, 0);
			EEPROM.write(c + 1, 0);
		}
	}
}

//    <<<<    FINISH    --------------------------    EEPROM Commands     --------------------------

//    >>>>    START     -------------------------    DCC Decoder Setup     -------------------------

void initialiseDCC()
{
	DCC.SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket_Handler, true);
	ConfigureDecoder();
	DCC.SetupDecoder(0x00, 0x00, kDCC_INTERRUPT);
}

void ConfigureDecoder()
{ //Put all the decoder #'s you need here.	Remember to change
  //DCCAccessoryAddress gAddresses[XX];(above) where XX = number of addresses you need.
	gAddresses[0].address = 200;
	gAddresses[1].address = 201;
	gAddresses[2].address = 202;
	gAddresses[3].address = 203;
	gAddresses[4].address = 204;
	gAddresses[5].address = 205;
	gAddresses[6].address = 206;
	gAddresses[7].address = 207;
}

void BasicAccDecoderPacket_Handler(int address, boolean activate, byte data)	// Basic accessory packet handler
{
	// Convert NMRA packet address format to human address
	address -= 1;
	address *= 4;
	address += 1;
	address += (data & 0x06) >> 1;

	boolean enable = (data & 0x01) ? 1 : 0;

	for (int i = 0; i < (int)(sizeof(gAddresses) / sizeof(gAddresses[0])); i++)
	{
		if (address == gAddresses[i].address)
		{
			tableTargetHead = enable;
			tableTargetPosition = i;
			//New packet and we have a new target location, set the flag
			newTargetLocation = true;
			doStepperMove();
		}
	}
}

void autoDCCMode()
{
	int addr = 0;
	DCC.loop();
	// Bump to next address to test
	if (++addr >= (int)(sizeof(gAddresses) / sizeof(gAddresses[0]))) { addr = 0; }

	stepperTimer();
}

//    <<<<    FINISH    -------------------------    DCC Decoder Setup     -------------------------

void drawTurntable(int radius, int colour)
{
	tft.fillCircle(xCentre, yCentre, radius, colour);
	tft.fillCircle(xCentre, yCentre, radius - 4, BLACK);
}

void calcTurntablePoints(int angle, boolean show)
{

	int top1, top2, bot1, bot2;
	int radTurn = radius - 7;

	int dispCol = GREY;
	int dispCol1 = GREEN;
	int dispCol2 = RED;

	if (show != true)
	{
		dispCol = BLACK; dispCol1 = BLACK; 	dispCol2 = BLACK;
	}

	if (angle > 360 || angle < 0) angle = 0;

	top1 = ((360 - 3) + angle);
	top2 = ((3 + angle) + 180);
	bot1 = (3 + angle);
	bot2 = (((360 - 3) + angle) - 180);

	drawTrackLine(top1, top2, radTurn, radTurn, 0, dispCol);
	drawTrackLine(top1, top2, radTurn, radTurn, -1, dispCol);
	drawTrackLine(bot1, bot2, radTurn, radTurn, 0, dispCol);
	drawTrackLine(top1, top2, radTurn, radTurn, 1, dispCol);
	drawTrackLine(top1, bot1, radTurn, radTurn, 0, dispCol);
	drawTrackLine(top2, bot2, radTurn, radTurn, 1, dispCol);

	drawMarker(angle, radTurn - 4, 2, dispCol1);
	drawMarker((360 + angle) - 180, radTurn - 4, 2, dispCol2);
}

void drawTrackLine(int p1, int p2, int radius1, int radius2, int offset, int colour)
{
	int aX = xCentre + radius1 * cos(p1 * PI / 180);
	int aY = yCentre + radius1 * sin(p1 * PI / 180);
	int bX = xCentre + radius2 * cos(p2 * PI / 180);
	int bY = yCentre + radius2 * sin(p2 * PI / 180);
	tft.drawLine(aX, aY + offset, bX, bY + offset, colour); // top line

}

void drawMarker(int p1, int radius, int size, int colour)
{
	int aX = xCentre + radius * cos(p1 * PI / 180);
	int aY = yCentre + radius * sin(p1 * PI / 180);
	tft.fillCircle(aX, aY, size, colour);
}

void drawTracks(boolean isTrackCalibration)
{
	int i = 1;
	int lengthTrack = 10;
	int offset = 3;

	if (isTrackCalibration == true) i = 0;

	for (i; i < sizeof(PositionTrack); i++)
	{
		int trackAngle = convertStepDeg(PositionTrack[i], false);
		int p1 = ((360 - 3) + trackAngle);
		int p2 = ((360 + 3) + trackAngle);

		drawTrackLine(p1, p2, radius, radius + lengthTrack, 0, GREY);
		drawTrackLine(p1, p2, radius, radius + lengthTrack, 1, GREY);

	}
}

int convertStepDeg(int unit, boolean ToSteps)
{

	if (ToSteps == true)
		(totalSteps / 360)*unit;
	else
		(unit / 360) * totalSteps;

}

//    >>>>    START     ----------------------------   Menu Functions   ----------------------------	
void MenuTabs(int tabSelected)
{
	int tabs = 5;
	int tabHeight = 30;
	int tabPad = 3;
	int sidePadding = 14;
	int menuBorder = 6;

	int tabX;
	int tabY = tft.height() - menuBorder - tabHeight;
	int lastTabX = 0;
	int tabWidth = tft.width - ((sidePadding * 2) + (tabPad * (tabs - 1)));

	for (int i = 0; i < tabs; i++)
	{
		if (i == 0)
			tabX = sidePadding;
		else
			tabX = lastTabX + tabWidth + tabPad;
		if (tabSelected == i)
			tft.drawRect(tabX, tabY, tabWidth, tabHeight, LIGHTGREY);
		else
			tft.drawRect(tabX, tabY, tabWidth, tabHeight, DARKGREY);

		lastTabX = tabX;
	}

	tft.fillRoundRect(menuBorder, menuBorder, tft.width() - menuBorder, tft.height() - tabHeight, 3, LIGHTGREY);
	tft.fillRoundRect(menuBorder + tabPad, menuBorder + tabPad, tft.width() - menuBorder - tabPad, tft.height() - menuBorder - tabPad, 3, BLACK);


}

void MainMenu()
{

}

void CalibrateMenu()
{

}

void ManualMove()
{}

void sleepTFT()
{

	if (sleep == sleepTimer)
	{
		tft.fillScreen(BLACK);
	}
	else sleep++;
}

void determineButtonLocation()
{
	//

}

void createButtons(int posX, int posY, int colour, String buttonText, boolean isActive)
{
	int defHeight = 30;
	int defWidth = 30;
	int radCorner = 4;

	if (isActive == true) colour = ORANGE;

	tft.fillRoundRect(posX, posY - 20, defHeight, defWidth, radCorner, colour);
	tft.drawRoundRect(posX - 2, posY - 2, defHeight - 2, defWidth - 2, radCorner, BLACK);
	tft.setCursor(posX + (defHeight / 2 - 5), posY + (defWidth / 2));
	tft.setFont(&FreeSansBold9pt7b);
	tft.print(buttonText);
	tft.setFont();

}

void touchButton()
{
}

//    >>>>    FINISH    ----------------------------   Menu Functions   ----------------------------

//    <<<<    START     ----------------------    Manually Move Turntable     ----------------------

//    <<<<    FINISH    ----------------------    Manually Move Turntable     ----------------------

//    >>>>    START     -----------------------    Check Track Positions     -----------------------

//    <<<<    FINISH    -----------------------    Check Track Positions     -----------------------

//    >>>>    START     ---------------    Select And Programme Turntable Tracks     ---------------

//    <<<<    FINISH    ---------------    Select And Programme Turntable Tracks     ---------------

//    >>>>    START     --------------------------    Calibrate Bridge    --------------------------

//    <<<<    FINISH    --------------------------    Calibrate Bridge    --------------------------

//    <<<<    START     ---------------------------    Stepper Voids     ---------------------------
void doStepperMove()
{
	stepper.run();	// Run the Stepper Motor
	boolean isInMotion = (abs(stepper.distanceToGo()) > 0);
	boolean newTargetSet = false;

	// If there is a new target location, set the target
	if (newTargetLocation)
	{
		SetStepperTargetLocation();
		//          displayOutput("Moving to ", ""));
		newTargetSet = true;
	}

	if ((currentStepPosition % totalSteps) == 0)
	{
		//setCurrentPosition seems to always reset the position to 0, ignoring the parameter
		str1 = String(F("Current location: "));
		//				str2 = String(stepper.currentPosition());
		str2 = String(currentStepPosition);
		str3 = String(F(" % STEPCOUNT.	Why here?"));
		//displayOutput(true, str1 + str2, str3);
	}

	if (mainDiff < 0) { displayRotatingCW = false; }
	else if (mainDiff > 0) { displayRotatingCW = true; }

}
//    ----------------------------------------------------------------------------------------------
//
// Subroutine: SetStepperTargetLocation()
//	Takes the global variables: tableTargetHeadOrTail, and tableTargetPosition, and sets the stepper
//	object moveTo() target position in steps-	inserts values back into "doStepperMove()"
//
//    ----------------------------------------------------------------------------------------------

void SetStepperTargetLocation()
{
	int newTargetLoc = -1;
	if (tableTargetHead)
	{	//use head location variable	
		newTargetLoc = PositionTrack[tableTargetPosition];
		inMotionToNewTarget = true;
	}
	else
	{	//use tail location variable	
		newTargetLoc = PositionTrack[tableTargetPosition];
		inMotionToNewTarget = true;
	}

	if (newTargetLoc > 0)
	{
		//int currentLoc = stepper.currentPosition();
		int currentLoc = currentStepPosition;
		int mainDiff = newTargetLoc - currentLoc;

		if (mainDiff > (totalSteps / 2))
			mainDiff = mainDiff - totalSteps;
		else
			mainDiff = mainDiff + totalSteps;
	}

	if (mainDiff < 0)
	{
		mainDiff -= MOTOR_OVERSHOOT;
		overshootDestination = MOTOR_OVERSHOOT;
	}
	//			stepper.move(mainDiff);
	//programmingMode = false;
	newTargetLocation = false;
}
//    ----------------------------------------------------------------------------------------------
//	Stepper Timer sub routine this runs from the main loop. It also supports the release function.
//    ----------------------------------------------------------------------------------------------

void stepperTimer()
{
	int currentLoc = 0;

	// Run the Stepper Motor 
	stepper.run();
	boolean isInMotion = (abs(stepper.distanceToGo()) > 0);

	//Check if we have any distance to move for release() timeout.	Can check the
	// buffered var isInMotion because we also check the other variables.
	if (isInMotion || programmingMode)
	{
		//We still have some distance to move, so reset the release timeout
		stepperLastMoveTime = millis();
		isReleased = false;
	}
	else
	{
		if (!isReleased)
		{
			if (overshootDestination > 0)
			{
				stepper.move(overshootDestination);
				overshootDestination = -1;
			}

			if (((millis() - stepperLastMoveTime) >= releaseTimeout_ms))
			{
				//If isReleased, don't release again.
				isReleased = true;
				int currentLoc = stepper.currentPosition();	// Resets the position to the actual positive number it should be
				currentLoc = currentLoc % totalSteps;

				if (currentLoc < 0)
					currentLoc += totalSteps;

				stepper.setCurrentPosition(currentLoc);
				stepper.moveTo(currentLoc);

				//String(F("	Actual Current Position: "));
				//str2 = String(stepper.currentPosition());	// shows the position value corrected.

				//Set the servo brake
				//	brakeservo.write(servoBrake);
				//	delay(750);

				//release the motor
				//                  release2();
				// str1 = String(F("	Brake Set & Motor Released "));

			}
		}
	}
}

void calcLeastSteps(int trA, int trB)
{
	int currentLoc = PositionTrack[trA];
	int newTargetLoc = PositionTrack[trB];
	int getMotorStepCount = (totalSteps / 2);

	if (newTargetLoc > 0)
	{
		int currentLoc = stepper.currentPosition();
		mainDiff = newTargetLoc - currentLoc;
		if (mainDiff > getMotorStepCount) { mainDiff = mainDiff - totalSteps; }
		else if (mainDiff < -getMotorStepCount) { mainDiff = mainDiff + totalSteps; }

		if (mainDiff < 0)
		{
			mainDiff -= MOTOR_OVERSHOOT;
			overshootDestination = MOTOR_OVERSHOOT;
		}

		if (mainDiff < 0) { displayRotatingCW = false; }
		else if (mainDiff > 0) { displayRotatingCW = true; }

		String textMove;
		if (displayRotatingCW) { textMove = "CW"; }
		else { textMove = "CCW"; }

		stepper.move(mainDiff);
	}
}
//    <<<<    FINISH    ---------------------------    Stepper Voids     ---------------------------