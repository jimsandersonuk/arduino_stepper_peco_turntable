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
#include <TouchScreen.h>

//    >>>>    START     ------------------------------   TFT Setup    ------------------------------

#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSerifItalic12pt7b.h>
#include <Fonts/FreeSerifItalic18pt7b.h>
#include <Fonts/FreeSerifItalic24pt7b.h>
#include <Fonts/FreeSerifItalic9pt7b.h>


#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

//2.8inch_280-5
#define TS_MINX 180
#define TS_MINY 170
#define TS_MAXX 920
#define TS_MAXY 930
#define MINPRESSURE 10
#define MAXPRESSURE 1000

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
const int  MOTOR_STEP_COUNT = 200 * 16; //number of steps for a full rotation

										//    >>>>    FINISH    ------------------------   Track Step Definitions   ------------------------

										//    >>>>    START     --------------------   Parameters for turntable move    --------------------

int radius = 75;
int xCentre = tft.height() / 2;
int yCentre = tft.width() / 2;
int topOld1x, topOld2x, botOld1x, botOld2x, topOld1y, topOld2y, botOld1y, botOld2y;
int topOld2, botOld1, topOld1, botOld2;

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

//Do display rotation
const int displayRotateDelay = 5;   // This is the minimum delay in ms between steps of the stepper motor
boolean displayRotating = false;    // Is the "Display Rotate" function enabled?
boolean displayRotatingCW = true;   // In Display Rotate mode, are we rot

									//    >>>>    FINISH    --------------------   Parameters for turntable move    --------------------

									//    >>>>    START     ---------------------------    Adafruit Setup    ---------------------------

Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers
Adafruit_StepperMotor *mystepper = AFMStop.getStepper(200, 2);   //Connect stepper with 200 steps per revolution (1.8 degree) to the M3, M4 terminals (blue,yellow,green,red)
																 //you can change these to SINGLE, DOUBLE, INTERLEAVE or MICROSTEP! wrapper for the motor!(3200 Microsteps / revolution)
void forwardstep2() { mystepper->onestep(BACKWARD, MICROSTEP); }
void backwardstep2() { mystepper->onestep(FORWARD, MICROSTEP); }
void release2() { mystepper->release(); }
AccelStepper stepper = AccelStepper(forwardstep2, backwardstep2); // wrap the stepper in an AccelStepper object
																  //int forwardstep2() { return dummyStepper(1, 25); } //75 real
																  //int backwardstep2() { return dummyStepper(-1, 25); } //75 real

																  //    <<<<    FINISH    ---------------------------    Adafruit Setup    ---------------------------

boolean pause = true;

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

	//pause = false;
	//drawTurntable(radius, GREY);
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
		drawTurntable(radius, GREY);

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

void determineButtonLocation()
{
	//

}

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

	int top1x, top2x, bot1x, bot2x, top1y, top2y, bot1y, bot2y, headCircleX, headCircleY, tailCircleX, tailCircleY;
	int top1, top2, bot1, bot2;
	int radTurn = radius - 7;


	int dispCol = GREY;
	int dispCol1 = GREEN;
	int dispCol2 = RED;

	if (show != true)
	{
		dispCol = BLACK;
		dispCol1 = BLACK;s
		dispCol2 = BLACK;
	}

	if (angle > 360 || angle < 0) angle = 0;

	top1 = ((360 - 3) + angle);
	top2 = ((3 + angle) + 180);
	bot1 = (3 + angle);
	bot2 = (((360 - 3) + angle) - 180);

	top1x = xCentre + radTurn * cos(top1 * PI / 180);
	top1y = yCentre + radTurn * sin(top1 * PI / 180);
	top2x = xCentre + radTurn * cos(top2 * PI / 180);
	top2y = yCentre + radTurn * sin(top2 * PI / 180);
	bot1x = xCentre + radTurn * cos(bot1 * PI / 180);
	bot1y = yCentre + radTurn * sin(bot1 * PI / 180);
	bot2x = xCentre + radTurn * cos(bot2 * PI / 180);
	bot2y = yCentre + radTurn * sin(bot2 * PI / 180);

	headCircleX = xCentre + (radTurn - 4) * cos(angle * PI / 180);
	headCircleY = yCentre + (radTurn - 4) * sin(angle * PI / 180);
	tailCircleX = xCentre + (radTurn - 4) * cos(((360 + angle) - 180) * PI / 180);
	tailCircleY = yCentre + (radTurn - 4) * sin(((360 + angle) - 180) * PI / 180);

	tft.drawLine(top1x, top1y, top2x, top2y, dispCol); // top line
	tft.drawLine(top1x - 1, top1y - 1, top2x - 1, top2y - 1, dispCol); // top line
	tft.drawLine(bot1x, bot1y, bot2x, bot2y, dispCol); // bot line
	tft.drawLine(bot1x + 1, bot1y + 1, bot2x + 1, bot2y + 1, dispCol); // bot line
	tft.drawLine(top2x, top2y, bot2x, bot2y, dispCol); //left line
	tft.drawLine(top1x, top1y, bot1x, bot1y, dispCol); //right line

	tft.fillCircle(headCircleX, headCircleY, 2, dispCol1);
	tft.fillCircle(tailCircleX, tailCircleY, 2, dispCol2);
}

//
//void calcTurntablePoints(int angle, int colour)
//{
//	int top1x, top2x, bot1x, bot2x, top1y, top2y, bot1y, bot2y, headCircle, tailCircle;
//	int top1, top2, bot1, bot2;
//	int radTurn = radius - 7;
//
//	tft.drawLine(topOld1x, topOld1y, topOld2x, topOld2y, BLACK); // top line
//	tft.drawLine(topOld1x - 1, topOld1y - 1, topOld2x - 1, topOld2y - 1, BLACK); // top line
//	tft.drawLine(botOld1x, botOld1y, botOld2x, botOld2y, BLACK); // bot line
//	tft.drawLine(botOld1x + 1, botOld1y + 1, botOld2x + 1, botOld2y + 1, BLACK); // bot line
//	tft.drawLine(topOld2x, topOld2y, botOld2x, botOld2y, BLACK); //left line
//	tft.drawLine(topOld1x, topOld1y, botOld1x, botOld1y, BLACK); //right line
//
//	
//	top1 = ((360 - 3) + angle);
//	top2 = ((3 + angle) + 180);
//	bot1 = (3 + angle);
//	bot2 = (((360 - 3) + angle) - 180);
//
//	top1x = xCentre + radTurn * cos(top1 * PI / 180);
//	top1y = yCentre + radTurn * sin(top1 * PI / 180);
//	top2x = xCentre + radTurn * cos(top2 * PI / 180);
//	top2y = yCentre + radTurn * sin(top2 * PI / 180);
//	bot1x = xCentre + radTurn * cos(bot1 * PI / 180);
//	bot1y = yCentre + radTurn * sin(bot1 * PI / 180);
//	bot2x = xCentre + radTurn * cos(bot2 * PI / 180);
//	bot2y = yCentre + radTurn * sin(bot2 * PI / 180);
//	headCircle = xCentre + radTurn-3 * sin(angle * PI / 180);
//	tailCircle = xCentre + radTurn-3 * sin(angle * PI / 180);
//
//	tft.drawLine(top1x, top1y, top2x, top2y, GREY); // top line
//	tft.drawLine(top1x - 1, top1y - 1, top2x - 1, top2y - 1, GREY); // top line
//	tft.drawLine(bot1x, bot1y, bot2x, bot2y, GREY); // bot line
//	tft.drawLine(bot1x + 1, bot1y + 1, bot2x + 1, bot2y + 1, GREY); // bot line
//	tft.drawLine(top2x, top2y, bot2x, bot2y, RED); //left line
//
//
//	topOld1x = top1x; topOld2x = top2x; botOld1x = bot1x; botOld2x = bot2x; topOld1y = top1y; topOld2y = top2y; botOld1y = bot1y; botOld2y = bot2y;
//	topOld1 = top1; topOld2 = top2; botOld1 = bot1; botOld2 = bot2;
//
//
//	//for (int i = 0; i < 2; i++)
//	//{
//	//	top1x = xCentre + (radTurn - i) * cos(top1 * PI / 180);
//	//	top1y = yCentre + (radTurn - i) * sin(top1 * PI / 180);
//	//	bot1x = xCentre + (radTurn - i) * cos(bot1 * PI / 180);
//	//	bot1y = yCentre + (radTurn - i) * sin(bot1 * PI / 180);
//
//	//	tft.drawLine(top1x, top1y, bot1x, bot1y, GREEN); //right line
//	//}
//
//
//}

//    >>>>    START     ----------------------------   Menu Functions   ----------------------------	
void MenuTabs()
{

	int startX = 10;
	int startY = 5;
	int tabPadding = 5;
	int tabWidth = (tft.width() / 5) - tabPadding * 2;
	int tabHeight = 30;
	int tabPad = 0;
	int tabX2 = 5;

	for (int i = 0; i < 5; i++)
	{
		if (i > 0)
			tabPad = tabPadding;

		int tabX = tabX2 + tabPad;


		Serial.println(String(tabX) + "   " + String(tabX2) + "   " + String(tabWidth * i));

		tft.fillRoundRect(tabX, startY, tabWidth, tabHeight, 3, LIGHTGREY);

		tabX2 = tabX + tabWidth;
	}
	tft.fillRoundRect(5, 25, tft.width() - 5, tft.height() - 20, 3, LIGHTGREY);
	tft.fillRoundRect(7, 27, tft.width() - 9, tft.height() - 24, 3, BLACK);

}


void MainMenu()
{


}

void CalibrateMenu()
{}

void ManualMove()
{}

void sleepTFT()
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


	if ((currentStepPosition % MOTOR_STEP_COUNT) == 0)
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

		if (mainDiff > (MOTOR_STEP_COUNT / 2))
			mainDiff = mainDiff - MOTOR_STEP_COUNT;
		else
			mainDiff = mainDiff + MOTOR_STEP_COUNT;
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
				currentLoc = currentLoc % MOTOR_STEP_COUNT;

				if (currentLoc < 0)
					currentLoc += MOTOR_STEP_COUNT;

				stepper.setCurrentPosition(currentLoc);
				//stepper.moveTo(currentLoc);				
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
	int getMotorStepCount = (MOTOR_STEP_COUNT / 2);

	if (newTargetLoc > 0)
	{
		//		int currentLoc = stepper.currentPosition();
		mainDiff = newTargetLoc - currentLoc;
		if (mainDiff > getMotorStepCount) { mainDiff = mainDiff - MOTOR_STEP_COUNT; }
		else if (mainDiff < -getMotorStepCount) { mainDiff = mainDiff + MOTOR_STEP_COUNT; }

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