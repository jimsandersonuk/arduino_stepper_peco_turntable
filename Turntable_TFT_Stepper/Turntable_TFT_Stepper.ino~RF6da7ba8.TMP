/*
Name:    Turntable_TFT_Stepper.ino
Created: 4/20/2017 12:21:05 PM
Author:  jimsanderson
*/

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include <DCC_Decoder.h>

#include <EEPROM.h>
#include <Wire.h>
//#include <SPI.h>
#include <SoftwareSerial.h>

#include <Adafruit_GFX.h>// Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
//#include <TFT_Extension.h>
#include <TouchScreen.h>

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSerifItalic12pt7b.h>

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
#define LIGHTGREY   0xC618      /* 192, 192, 192 */
#define DARKGREY    0x7BEF      /* 128, 128, 128 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define RED         0xF800      /* 255,   0,   0 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */
#define ORANGE      0xFD20      /* 255, 165,   0 */
#define GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define GREY		0x8410

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
byte screenRotation = 3;
//    >>>>    FINISH    ------------------------------   TFT Setup    ------------------------------

//    >>>>    START     --------------------------   Define DCC Control   --------------------------
#define kDCC_INTERRUPT  0 // Define DCC commands to Arduino
typedef struct { int address; }
DCCAccessoryAddress;  // Address to respond to
DCCAccessoryAddress gAddresses[7]; // Allows 7 DCC addresses: [XX] = number of addresses you need (including 0).

//    >>>>    FINISH    --------------------------   Define DCC Control   --------------------------

//    >>>>    START     ------------------------   Track Step Definitions   ------------------------

//Define Corresponding Head and Tail   UNO

const byte GetTrackTail[6] = { 4, 5, 6, 1, 2, 3 }; // Array for Tail Tracks
int PositionTrack[7] = { 0, 0, 0, 0, 0, 0, 0 };   // Save EEPROM addresses to array - index 0 = calibration position
int PositionTrackDummy[7] = { 0, 560, 800, 1040, 2160, 2400, 2640 };
byte storeTargetTrack = 0;             // Store Target Track position
byte storeStartTrack = 0;

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
byte currentTrack = 1;
byte newTrack = 1;
byte distanceToGo = 0;
const int displayRotateDelay = 5;   // This is the minimum delay in ms between steps of the stepper motor
boolean displayRotating = false;    // Is the "Display Rotate" function enabled?
boolean displayRotatingCW = true;   // In Display Rotate mode, are we rot
long stepperLastMoveTime = 0;
int mainDiff = 0;
const byte MOTOR_OVERSHOOT = 10;			// the amount of overshoot/ lash correction when approaching from CCW
byte overshootDestination = -1;
const int releaseTimeout_ms = 2000;		//reduced to 2 seconds for now
const int  totalSteps = 200 * 16;		//number of steps for a full rotation

//    >>>>    FINISH    --------------------   Parameters for turntable move    --------------------

//    >>>>    START     -------------------------------   TFT Menu   -------------------------------
byte radiusTurntable = 75;
byte xCentre = tft.height() / 2;
byte yCentre = (tft.width() / 2);
byte turntableOffset = 25;

int sleep = 0; // sleep counter
boolean pause = true;
const int sleepTimer = 30000; // sleep after 30 secs
const int startAngle = abs(360 / screenRotation);

byte menuPage = 0;

const byte buttonArraySize = 16;
unsigned int buttonArray[buttonArraySize][4];
String buttonTextArray[buttonArraySize] = { "AutoDCC", "Manual", "Calibrate", "Program", "C", "1", "2", "3", "4", "5", "6", "<< + 10", "< + 1", "10 + >>", "1 + >" };

const byte tabs = 4;
//String menuTabText[4] = { "AutoDCC", "Manual", "Calibrate", "Program" };
const byte tabHeight = 20;
const byte tabPad = 3;
const byte sidePadding = 14;
const byte menuBorder = 6;

const byte lengthTrack = 30;
const byte buttonHeight = 30;
const byte buttonWidth = 30;
const byte radiusButCorner = 4;
const int butColour = GREENYELLOW;
const int butActiveColour = ORANGE;
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

	//Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());
	initialiseDCC();
	tft.reset();
	tft.begin(0x9325);

	tft.setRotation(screenRotation);
	tft.fillScreen(BLACK);

	pinMode(13, OUTPUT);

	tft.setCursor(5, 15);
	tft.setTextColor(WHITE);
	tft.setTextSize(1);
	tft.setFont(&FreeSerifItalic12pt7b);
	tft.setCursor(30, yCentre);
	tft.println(F("DCC Controlled Turntable"));
	tft.setFont();
	delay(1000);
	tft.fillScreen(BLACK);
	MenuTabs(0);
	drawTurntable(radiusTurntable, GREY, 0);
	//drawTracks(false);
	delay(3000);
}
//    >>>>    FINISH    --------------------------------   SETUP    --------------------------------

//    >>>>    START     ---------------------------------   LOOP   ---------------------------------
void loop()
{
	sleep++;
	sleepTFT();
	//delay(3000);

	//digitalWrite(13, HIGH);
	//TSPoint p = ts.getPoint();
	//digitalWrite(13, LOW);

	//drawTracks(false);

	while (pause)
	{
		//		drawTurntable(radius, GREY);

		for (byte i = 0; i < 90; i++)
		{
			delay(10);
			drawTurntableBridge((i - 1), false);
			drawTurntableBridge(i, true);
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
	for (byte i = 0; i < 7; i++)
	{
		returnArray = EEPROMReadlong(i);
		PositionTrack[i] = returnArray;
	}
}

void clearEEPROM()
{
	int c = 0;
	for (byte i = 0; i < 7; i++)
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

	for (byte i = 0; i < (int)(sizeof(gAddresses) / sizeof(gAddresses[0])); i++)
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


void drawAll( int turntableOffset, boolean calibrationMode)
{
	drawTurntable( radiusTurntable, GREY,turntableOffset);
	drawTurntableBridge(270, false);
	drawTracks(calibrationMode);
}

void drawTurntable(int radius, byte colour, byte turnOffset)
{
	tft.fillCircle(xCentre, yCentre + turnOffset, radius, colour);
	tft.fillCircle(xCentre, yCentre + turnOffset, radius - 4, BLACK);
	drawTracks(true);
}

void drawTurntableBridge(int angle, boolean show)
{
	angle = angle - 90;
	int top1, top2, bot1, bot2;
	byte radiusBridge = radiusTurntable - 7;
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

	drawTrackLine(top1, top2, radiusBridge, radiusBridge, 0, dispCol);
	drawTrackLine(top1, top2, radiusBridge, radiusBridge, 1, dispCol);
	drawTrackLine(bot1, bot2, radiusBridge, radiusBridge, 0, dispCol);
	drawTrackLine(bot1, bot2, radiusBridge, radiusBridge, 1, dispCol);
	drawTrackLine(top1, bot1, radiusBridge, radiusBridge, 0, dispCol);
	drawTrackLine(top2, bot2, radiusBridge, radiusBridge, 0, dispCol);

	drawMarker((360 + angle), radiusBridge - 4, 2, dispCol1);
	drawMarker((360 + angle) - 180, radiusBridge - 4, 2, dispCol2);
}

void drawTrackLine(int p1, int p2, byte radius1, byte radius2, byte offset, int colour)
{
	int aX = findX(xCentre, radius1, p1);
	int aY = findY(yCentre + turntableOffset, radius1, p1);
	int bX = findX(xCentre, radius2, p2);
	int bY = findY(yCentre + turntableOffset, radius2, p2);
	tft.drawLine(aX, aY + offset, bX, bY + offset, colour);
}

void drawMarker(int p1, int radius1, int size, int colour)
{
	int aX = findX(xCentre, radius1, p1);
	int aY = findY(yCentre + turntableOffset, radius1, p1);
	tft.fillCircle(aX, aY, size, colour);
}

void drawTracks(boolean isTrackCalibration)
{
	byte i = 1;
	byte offset = 3;

	String buttonText = String(i);

	if (isTrackCalibration == true)
	{
		i = 0;
		buttonText = "C";
	}

	for (i; i < sizeof(PositionTrackDummy) / sizeof(PositionTrackDummy[0]); i++)
	{
		int angle = convertStepDeg(PositionTrackDummy[i], false);

		int p1 = (360 - angle);
		drawTrackLine(p1, p1, radiusTurntable, radiusTurntable + lengthTrack, -offset, GREY);
		drawTrackLine(p1, p1, radiusTurntable, radiusTurntable + lengthTrack, +offset, GREY);
		createTrackButtons(i, buttonText, false);
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
void MenuTabs(byte tabSelected)
{
	byte tabX;
	byte tabY = menuBorder;

	byte lastTabX = 0;
	int tabWidth = (tft.width() - ((sidePadding * 2) + (tabPad * (tabs - 1)))) / tabs;

	for (byte i = 0; i < tabs; i++)
	{
		if (i == 0)
			tabX = sidePadding;
		else
			tabX = lastTabX + tabWidth + tabPad;
		if (tabSelected == i)
			tft.fillRect(tabX, tabY, tabWidth, tabHeight, LIGHTGREY);			
		else
			tft.fillRect(tabX, tabY, tabWidth, tabHeight, DARKGREY);
				
/*
		if (buttonTextArray[i].length > tabWidth - 2)
		{
		}
*/
		writeButtonArray(buttonTextArray[i], tabX, tabY, tabHeight, tabWidth);

		tft.setFont(&FreeSans9pt7b);
		tft.setCursor(tabX + 2, tabY + 3);
		tft.print(buttonTextArray[i]);
		lastTabX = tabX;
		tft.setFont();
	}

	// Menu Frame
	tft.fillRoundRect(menuBorder, menuBorder + tabHeight, tft.width() - (menuBorder * 2), tft.height() - tabHeight - (menuBorder * 2), 3, LIGHTGREY);
	tft.fillRoundRect(menuBorder + tabPad, menuBorder + tabHeight + tabPad, tft.width() - ((menuBorder + tabPad) * 2), tft.height() - tabHeight - ((menuBorder + tabPad) * 2), 3, BLACK);

}

void AutoDccMenu()
{
	menuPage = 0;
	MenuTabs(menuPage);
	drawAll(0, false);
}

void ManualMove()
{
	menuPage = 1;
	MenuTabs(menuPage);
	drawAll(0, false);
}

void CalibrateMenu()
{
	menuPage = 2;
	MenuTabs(menuPage);
	drawAll(turntableOffset, true);
}

void Programming()
{
	menuPage = 3;
	MenuTabs(menuPage);
	drawAll(turntableOffset, true);
}

void sleepTFT()
{

	if (sleep >= sleepTimer)
	{
		tft.fillScreen(BLACK);
	}
	else sleep++;
}

void createTrackButtons(byte button, String buttonText, boolean isActive)
{
	int degPos = convertStepDeg(PositionTrackDummy[button], false);
	int pX =  readButtonArray(buttonText, 'pX');
	int pY =  readButtonArray(buttonText, 'pY');
	byte buttonColour = 0;

	if (pX == 0 || pY == 0)
	{
		pX = findX(xCentre, radiusTurntable + lengthTrack, degPos);
		pY = findY(yCentre, radiusTurntable + lengthTrack, degPos);
		writeButtonArray(buttonText, pX, pY, buttonHeight, buttonWidth);
	}


	byte offsetX = -(buttonWidth / 2);
	byte offsetY = -(buttonHeight / 2);

	if (quadrant(getDegreeCoordinates(pX, pY)) == 0) { offsetY = -buttonHeight; }

	if (isActive == true) buttonColour = butActiveColour;

	tft.fillRoundRect(pX - offsetX, pY - offsetY, buttonHeight, buttonWidth, radiusButCorner, buttonColour);
	tft.drawRoundRect(pX - offsetX - 2, pY - offsetY - 2, buttonHeight - 2, buttonWidth - 2, radiusButCorner, BLACK);
	tft.setCursor(pX + (buttonHeight / 3), pY + (buttonWidth / 3));
	tft.setFont(&FreeSansBold9pt7b);
	tft.setTextColor(BLACK);
	tft.print(buttonText);
	tft.setFont();
}

int touchButton(double tX, double tY)
{	




}

int getDegreeCoordinates(int pX, int pY)
{
	int deg = atan2((pX - xCentre), (pY - yCentre))*(180 / PI);

	if (deg < 0)
		deg = 360 + deg;

	return deg;
}

byte quadrant(int deg)
{
	return byte(deg / 45);
}

int findX(int cX, byte circleRad, int angle)
{
	return cX + circleRad * cos(angle * PI / 180);
}

int findY(int cY, byte circleRad, int angle)
{
	return cY + circleRad * cos(angle * PI / 180);
}

void writeButtonArray(String buttonText, int pX, int pY, byte pH, byte pW)
{
	byte arrayPos = -1;
	
	for (byte i = 0; i < sizeof(buttonTextArray) / sizeof(buttonTextArray[0]); i++)
	{
		if (buttonTextArray[i] == buttonText)
		{
			arrayPos = i;
		}
	}
	
	buttonArray[arrayPos][0] = pX; // rectangle x point
	buttonArray[arrayPos][1] = pY; // rectangle y point
	buttonArray[arrayPos][2] = pH; // rectangle height
	buttonArray[arrayPos][3] = pW; // rectangle width

}

int readButtonArray(String buttonText, char returnValue)
{
	byte arrayPos = -1;

	for (byte i = 0; i < sizeof(buttonTextArray) / sizeof(buttonTextArray[0]); i++)
	{
		if (buttonTextArray[i] == buttonText)
		{
			arrayPos = i;
		}
	}

	switch (returnValue)
	{
	case 'pX':
		return buttonArray[arrayPos][0];
		break;
	case 'pY':
		return buttonArray[arrayPos][1];
		break;
	case 'pH':
		return buttonArray[arrayPos][2];
		break;
	case 'pW':
		return buttonArray[arrayPos][3];
		break;
	case 'pos':
		return arrayPos;
		break;
	default:
		break;
	}
	return arrayPos;
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
	if (newTargetLocation == true)
	{
		SetStepperTargetLocation();
		//          displayOutput("Moving to ", ""));
		newTargetSet = true;
	}

	if ((currentStepPosition % totalSteps) == 0)
	{
		//setCurrentPosition seems to always reset the position to 0, ignoring the parameter
		//str1 = String(F("Current location: "));
		//				str2 = String(stepper.currentPosition());
		//str2 = String(currentStepPosition);
		//str3 = String(F(" % STEPCOUNT.	Why here?"));
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
	byte newTargetLoc = -1;
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
				stepper.currentPosition();	// Resets the position to the actual positive number it should be
				currentLoc = stepper.currentPosition() % totalSteps;

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

void calcLeastSteps(byte trA, byte trB)
{
	byte currentLoc = PositionTrack[trA];
	byte newTargetLoc = PositionTrack[trB];
	int getMotorStepCount = (totalSteps / 2);

	if (newTargetLoc > 0)
	{
		currentLoc = stepper.currentPosition();
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
