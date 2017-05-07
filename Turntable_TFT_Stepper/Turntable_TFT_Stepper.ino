/*
Name:    Turntable_TFT_Stepper.ino
Created: 4/20/2017 12:21:05 PM
Author:  jimsanderson
*/

#define DEBUG

#ifdef DEBUG
#include <SoftwareSerial.h>
#endif

#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
	
#include <Adafruit_GFX.h>// Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <stdint.h>	
#include <TouchScreen.h>
//#include <SPI.h>
//#include <TFT_Extension.h>

#include <Fonts/FreeSansBold9pt7b.h>

#include <DCC_Decoder.h>
#include <EEPROM.h>

#define LCDROTATION 3
//    >>>>    START     ------------------------------   TFT Setup    ------------------------------

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

#define TS_MINX 165
#define TS_MINY 160
#define TS_MAXX 900
#define TS_MAXY 910

#define MINPRESSURE 50
#define MAXPRESSURE 1000

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

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
int screenRotation = LCDROTATION;

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
//int storeTargetTrack = 0;             // Store Target Track position
//int storeStartTrack = 0;
int selectedTracks[2] = { 0,0 };

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
boolean DCCMode = false;
boolean maunalMode = false;
boolean programmingMode = false;
boolean calibrationMode = false;
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
long stepperLastMoveTime = 0;
int mainDiff = 0;
const int MOTOR_OVERSHOOT = 10;			// the amount of overshoot/ lash correction when approaching from CCW
int overshootDestination = -1;
const int releaseTimeout_ms = 2000;		//reduced to 2 seconds for now
const int  totalSteps = 200 * 16;		//number of steps for a full rotation

//    >>>>    FINISH    --------------------   Parameters for turntable move    --------------------

//    >>>>    START     -------------------------------   TFT Menu   -------------------------------
int radiusTurntable = 65;

int turntableOffset = 25;

int refactorX = 0 ;
int refactorY = 0 ;

int sleep = 0; // sleep counter
boolean asleep = false;
boolean pause = true;
const int sleepTimer = 60000; // sleep after 30 secs
const int startAngle = abs(360 / screenRotation);

int menuPage = 0;

const int buttonArraySize = 17;
int buttonArrayX[buttonArraySize];
int buttonArrayY[buttonArraySize];
int buttonArrayT[buttonArraySize];
String buttonTextArray[buttonArraySize] = { "AutoDCC", "Manual", "Calibrate", "Program", "C", "1", "2", "3", "4", "5", "6", "<<", "<", ">>", ">","Save" };

const int tabs = 4;
int currentTab = 0;
int tabWidth = 0;
const int tabHeight = 20;
const int tabPad = 3;
const int sidePadding = 14;
const int menuBorder = 6;

int xCentre = tft.height() / 2;
int yCentre = (((tft.width() + (menuBorder + tabPad) * 2)) / 2);

const int lengthTrack = 30;
const int buttonHeight = 30;
const int buttonWidth = 30;
const int radiusButCorner = 4;
const int butColour = GREENYELLOW;
const int butActiveColour = ORANGE;

//    >>>>    FINISH    -------------------------------   TFT Menu   -------------------------------

//    >>>>    START     ----------------------   Adafruit MotorShield Setup   ----------------------

Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers
Adafruit_StepperMotor *mystepper = AFMStop.getStepper(200, 2);   //Connect stepper with 200 steps per revolution (1.8 degree) to the M3, M4 terminals (blue,yellow,green,red)
																 //you can change these to SINGLE, DOUBLE, INTERLEAVE or MICROSTEP! wrapper for the motor!(3200 Microsteps / revolution)
void release2() { mystepper->release(); }

//#ifdef DEBUG
	//int forwardstep2() { return dummyStepper(1, 25); } //75 real
	//int backwardstep2() { return dummyStepper(-1, 25); } //75 real

//#elif
	void forwardstep2() { mystepper->onestep(FORWARD, MICROSTEP); }
	void backwardstep2() { mystepper->onestep(BACKWARD, MICROSTEP); }

//#endif // DEBUG

AccelStepper stepper = AccelStepper(forwardstep2, backwardstep2); // wrap the stepper in an AccelStepper object

//    <<<<    FINISH    ----------------------   Adafruit MotorShield Setup   ----------------------

//    >>>>    START     --------------------------------   SETUP    --------------------------------
void setup()
{

#ifdef DEBUG
	Serial.begin(115200);
	//Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());
#endif

	//pixelFactorQuadrants();	// work out screen rotation and then apply ratation factoring to TouchScreen results
	initialiseDCC();	//Start up DCC reading
	tft.reset();
	tft.begin(0x9325);

	tft.setRotation(screenRotation);
	tft.fillScreen(BLACK);

	pinMode(13, OUTPUT);

	tft.setCursor(5, 15);
	tft.setTextColor(WHITE);
	tft.setFont(&FreeSansBold9pt7b);
	tft.setCursor(30, yCentre);
	tft.println("DCC Controlled Turntable");
	tft.setFont();
	delay(1000);
	tft.fillScreen(BLACK);
	MenuTabs(0);
	drawMenuFrame();
	//delay(3000);
}
//    >>>>    FINISH    --------------------------------   SETUP    --------------------------------

//    >>>>    START     ---------------------------------   LOOP   ---------------------------------
void loop()
{
	//sleep++;
	//sleepTFT();

	digitalWrite(13, HIGH);
	TSPoint p = ts.getPoint();
	digitalWrite(13, LOW);

	// if sharing pins, you'll need to fix the directions of the touchscreen pins
	//pinMode(XP, OUTPUT);
	pinMode(XM, OUTPUT);
	pinMode(YP, OUTPUT);
	//pinMode(YM, OUTPUT);

	if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
	{
		translateLCD(p.x, p.y);
		int selectedButton = touchButton(refactorX, refactorY);
//#ifdef DEBUG
//	Serial.println("Selected Button = " + String(selectedButton));
//#endif

	//tft.fillScreen(BLACK);
	MenuTabs(selectedButton);
	//drawAll(0, false);
	decideButtonAction(selectedButton);
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

//    >>>>    START     ---------------------------   Draw Turntable     ---------------------------

void drawAll( int turntableOffset, boolean calibrationMode)
{
	tft.fillRoundRect(menuBorder + tabPad, menuBorder + tabHeight + tabPad, tft.width() - ((menuBorder + tabPad) * 2), tft.height() - tabHeight - ((menuBorder + tabPad) * 2), 3, BLACK);
	drawTurntable( radiusTurntable, GREY,turntableOffset);
	drawTurntableBridge(0, true);
	drawTracks(calibrationMode);
}

void drawTurntable(int radius, int colour, int turnOffset)
{
	tft.fillCircle(xCentre, yCentre + turnOffset, radius, colour);
	tft.fillCircle(xCentre, yCentre + turnOffset, radius - 4, BLACK);
}

void drawTurntableBridge(int angle, boolean show)
{
	int adjAngle = angle - (360 - startAngle);
	int top1, top2, bot1, bot2;
	int radiusBridge = radiusTurntable - 7;
	int dispCol = GREY;	int dispCol1 = GREEN;	int dispCol2 = RED;

	if (show != true)
	{
		dispCol = BLACK; dispCol1 = BLACK; 	dispCol2 = BLACK;
	}

	if (adjAngle > startAngle || adjAngle < 360 - startAngle) adjAngle = startAngle;

	top1 = ((360 - 3) + adjAngle);
	top2 = ((3 + adjAngle) + 180);
	bot1 = (3 + adjAngle);
	bot2 = (((360 - 3) + adjAngle) - 180);

	drawTrackLine(top1, top2, radiusBridge, radiusBridge, 0, dispCol);
	drawTrackLine(top1, top2, radiusBridge, radiusBridge, 1, dispCol);
	drawTrackLine(bot1, bot2, radiusBridge, radiusBridge, 0, dispCol);
	drawTrackLine(bot1, bot2, radiusBridge, radiusBridge, 1, dispCol);
	drawTrackLine(top1, bot1, radiusBridge, radiusBridge, 0, dispCol);
	drawTrackLine(top2, bot2, radiusBridge, radiusBridge, 0, dispCol);

	drawMarker((360 + adjAngle), radiusBridge - 4, 2, dispCol1);
	drawMarker((360 + adjAngle) - 180, radiusBridge - 4, 2, dispCol2);
}

void drawTrackLine(int p1, int p2, int radius1, int radius2, int offset, int colour)
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
	int i = 1;
	int offset = 3;

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


//    >>>>    FINISH    ---------------------------   Draw Turntable     ---------------------------

//    >>>>    START     ----------------------------   Menu Functions   ----------------------------	
void MenuTabs(int tabSelected)
{
	int tabX;
	int tabY = menuBorder;

	int lastTabX = 0;
	tabWidth = (tft.width() - ((sidePadding * 2) + (tabPad * (tabs - 1)))) / tabs;
	tft.setTextColor(BLACK);

	for (int i = 0; i < tabs; i++)
	{
		if (i == 0)
			tabX = sidePadding;
		else
			tabX = lastTabX + tabWidth + tabPad;

		if (tabSelected == i)
		{
			tft.fillRect(tabX, tabY, tabWidth, tabHeight, LIGHTGREY);
			//tft.setFont(&RobotoBold5pt7b);
		}
		else
		{
			tft.fillRect(tabX, tabY, tabWidth, tabHeight, DARKGREY);
			//tft.setFont(&Roboto5pt7b);
		}

		writeButtonArray(buttonTextArray[i], tabX, tabY, 1);
		tft.setCursor(tabX + 8, tabY + 6);
		tft.print(buttonTextArray[i]);
		lastTabX = tabX;
		tft.setFont();
		currentTab = tabSelected;
	}
}

void drawMenuFrame()
{	// Menu Frame
	tft.fillRoundRect(menuBorder, menuBorder + tabHeight, tft.width() - (menuBorder * 2), tft.height() - tabHeight - (menuBorder * 2), 3, LIGHTGREY);
	tft.fillRoundRect(menuBorder + tabPad, menuBorder + tabHeight + tabPad, tft.width() - ((menuBorder + tabPad) * 2), tft.height() - tabHeight - ((menuBorder + tabPad) * 2), 3, BLACK);
}

void decideButtonAction(int buttonPress)
{
	if (buttonPress < 0) return;

	if (buttonPress < tabs)
	{
		DCCMode = false;
		maunalMode = false;
		programmingMode = false;
		calibrationMode = false;

		switch (buttonPress)
		{
		case 0:
			DCCMode = true;
			AutoDccMenu();
			break;
		case 1:
			maunalMode = true;
			ManualMove();
			break;
		case 2:
			calibrationMode = true;
			CalibrateMenu();
			break;
		case 3:
			programmingMode = true;
			Programming();
			break;
		}
	}
	if (buttonPress >= tabs)
	{
		switch (menuPage)
		{
		case 0:
			break;
		case 1:
			manualMoveTurntable(buttonPress);
			break;
		case 2:
			calibrateTurntable(buttonPress);
			break;
		case 3:
			programTurntable(buttonPress);
			break;

		}
	}
}

void AutoDccMenu()
{
	menuPage = 0;
	MenuTabs(menuPage);
	drawAll(0, false);
	autoDCCTurntable();
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
		asleep = true;
		tft.fillScreen(BLACK);
	}
	else sleep++;
}

//    >>>>    FINISH    ----------------------------   Menu Functions   ----------------------------

//    >>>>    START     ---------------------------   Calculate Points   ---------------------------

int getDegreeCoordinates(int pX, int pY)
{
	int deg = atan2((pX - xCentre), (pY - yCentre))*(180 / PI);

	if (deg < 0)
		deg = 360 + deg;

	return deg;
}

int quadrant(int deg)
{
	return int(deg / 45);
}

int findX(int cX, int circleRad, int angle)
{
	return cX + circleRad * cos(angle * PI / 180);
}

int findY(int cY, int circleRad, int angle)
{
	return cY + circleRad * sin(angle * PI / 180);
}

float convertStepDeg(float unit, boolean toSteps)
{
	float step = (float(totalSteps) / 360)*unit;
	float deg = 90 - (unit / float(totalSteps) * 360);
	//Serial.println("Degrees = "+ String(deg));

	if (toSteps == true)
		return step;
	else
		return deg;
}

//    >>>>    FINISH    ---------------------------   Calculate Points   ---------------------------

//    >>>>    START     ----------------------   Translate Lcd Touchscreen    ----------------------

void translateLCD(int tX, int tY)
{

	float rMinX, rMaxX, rMinY, rMaxY;

	if (tX < TS_MINX) rMinX = tX;
	else rMinX = TS_MINX;

	if (tX > TS_MAXX) rMaxX = tX;
	else rMaxX = TS_MAXX;

	if (tY < TS_MINY) rMinY = tY;
	else rMinY = TS_MINY;

	if (tY > TS_MAXY) rMaxY = tY;
	else rMaxY = TS_MAXY;

	switch (LCDROTATION)
	{
	case 3:

		float refY = 1-(((float)tX - (float)rMinX) / ((float)rMaxX - (float)rMinX)); //* tft.width();
		float refX = 1-(((float)tY - (float)rMinY) / ((float)rMaxY - (float)rMinY)); // *tft.height();
		refactorX = refX * tft.width();
		refactorY = refY * tft.height();

//#ifdef DEBUG
//		Serial.println("-----------------------");
////		Serial.println();
////		Serial.println("pX = " + String(p.x));
//		Serial.println("tX = " + String(tX));
//		Serial.println("tY = " + String(tY));
//		Serial.println("rMinX = " + String(rMinX));
//		Serial.println("refY = " + String(refY));
//		Serial.println("refX = " + String(refX));
//		Serial.println("corrX = " + String(refactorX));
//		Serial.println("corrY = " + String(refactorY));
//		//Serial.println("-----------------------");
//#endif
		break;
	}
}

//    >>>>    FINISH    ----------------------   Translate Lcd Touchscreen    ----------------------

//    >>>>    START     -------------------------   Button Calculations    -------------------------

int touchButton(int tX, int tY)
{
	int pH;
	int pW;
	int buttonPressed = -1;
	int arraySize = sizeof(buttonTextArray) / sizeof(buttonTextArray[0]);
	/*
	// touchscreen to Pixels based upon current screen rotation
	float cX = (((float)tX - (float)TS_MINX) / ((float)TS_MAXX - (float)TS_MINX)); //* tft.width();
	float cY = (((float)tY - (float)TS_MINY) / ((float)TS_MAXY - (float)TS_MINY)); // *tft.height();
	*/

	for (int i = 0; i < arraySize; i++)
	{
		int pX = buttonArrayX[i];
		int pY = buttonArrayY[i];
		int bt = buttonArrayT[i];
//
//#ifdef DEBUG
//		Serial.println("-----------------------------------");
//		Serial.println("pX = " + String(i));
//		Serial.println("pX = " + String(tX));
//		Serial.println("pY = " + String(tY));
//		Serial.println("-----------------------------------");
//#endif // DEBUG		
		switch (bt)
		{
		case 1:
			pH = tabHeight;
			pW = tabWidth;
			break;
		case 2:
			pH = buttonHeight;
			pW = buttonWidth;
		}

		if (pX == 0 || pY == 0)
		{
			i = arraySize+1;
			break;
		}

		if (tX>pX && tX<pX + pW && tY > pY && tY < pY + pH)
		{
			buttonPressed = i;
			i = arraySize+1;
			break;
		}
	}

	return buttonPressed;
}

void createTrackButtons(int button, String buttonText, boolean isActive)
{
	int degPos = convertStepDeg(PositionTrackDummy[button], false);
	int pX = readButtonArray(buttonText, 0);
	int pY = readButtonArray(buttonText, 1);
	int buttonColour = 0;

	if (pX == 0 || pY == 0)
	{
		pX = findX(xCentre, radiusTurntable + lengthTrack, degPos);
		pY = findY(yCentre, radiusTurntable + lengthTrack, degPos);
		writeButtonArray(buttonText, pX, pY, 2);
	}

	int offsetX = -(buttonWidth / 2);
	int offsetY = -(buttonHeight / 2);

	if (quadrant(getDegreeCoordinates(pX, pY)) == 0) { offsetY = -buttonHeight; }

	if (isActive == true) buttonColour = butActiveColour;

	tft.fillRoundRect(pX - offsetX, pY - offsetY, buttonHeight, buttonWidth, radiusButCorner, buttonColour);
	tft.drawRoundRect(pX - offsetX - 2, pY - offsetY - 2, buttonHeight - 2, buttonWidth - 2, radiusButCorner, BLACK);
	tft.setCursor(pX + (buttonHeight / 3), pY + (buttonWidth / 3));
	//tft.setFont(&TurnButtonsRobotoBold10pt7b);
	tft.setTextColor(BLACK);
	tft.print(buttonText);
	tft.setFont();
}

void writeButtonArray(String buttonText, int pX, int pY, int bt)
{
	int arrayPos = -1;

	for (int i = 0; i < sizeof(buttonTextArray) / sizeof(buttonTextArray[0]); i++)
	{
		if (buttonTextArray[i] == buttonText)
		{
			arrayPos = i;
			i = sizeof(buttonTextArray) / sizeof(buttonTextArray[0]);
		}
	}

	buttonArrayX[arrayPos] = pX; // rectangle x point
	buttonArrayY[arrayPos] = pY; // rectangle y point
	buttonArrayT[arrayPos] = bt; // rectangle height
								 //buttonArray[arrayPos][2] = pH; // rectangle height
								 //buttonArray[arrayPos][3] = pW; // rectangle width

#ifdef DEBUG

	//Serial.println("Tab: " + String(arrayPos) + " X: " + String(buttonArrayX[arrayPos]) + " Y: " + String(buttonArrayY[arrayPos]) + " H: " + String(buttonArrayT[arrayPos]));

#endif // DEBUG
}

int readButtonArray(String buttonText, int returnValue)
{
	int arrayPos = -1;

	for (int i = 0; i < sizeof(buttonTextArray) / sizeof(buttonTextArray[0]); i++)
	{
		if (buttonTextArray[i] == buttonText)
		{
			arrayPos = i;
		}
	}

	switch (returnValue)
	{
	case 0: //px
		return buttonArrayX[arrayPos];
		break;
	case 1: //py
		return buttonArrayY[arrayPos];
		break;
	case 2: //buttonType
		return buttonArrayT[arrayPos];
		break;
		/*case 'pH':
		return buttonArray[arrayPos][2];
		break;
		case 'pW':
		return buttonArray[arrayPos][3];
		break;*/
	case 3://array position
		return arrayPos;
		break;
	default:
		break;
	}
	return arrayPos;
}

//    >>>>    FINISH    -------------------------   Button Calculations    -------------------------

//    >>>>    START     ----------------------------   Auto DCC Mode    ----------------------------

void autoDCCTurntable()
{

}

//    >>>>    FINISH    ----------------------------   Auto DCC Mode    ----------------------------

//    <<<<    START     ----------------------    Manually Move Turntable     ----------------------

void manualMoveTurntable(int buttonPress)
{
	//do {
	//	switch (buttonPress)
	//	{
	//	case 5: // track 1	
	//		break;
	//	case 6: // track 2	
	//		break;
	//	case 7: // track 3	
	//		break;
	//	case 8:	// track 4
	//		break;
	//	case 9: // track 5	
	//		break;
	//	case 10: // track 6	
	//		break;
	//	}
	//} while (menuPage = 1);
}


//    <<<<    FINISH    ----------------------    Manually Move Turntable     ----------------------

//    >>>>    START     --------------------------    Calibrate Bridge    --------------------------

void calibrateTurntable(int buttonPress)
{

	switch (buttonPress)
	{
	case 4: // position 1
		break;
	}
}
//    <<<<    FINISH    --------------------------    Calibrate Bridge    --------------------------

//    >>>>    START     ---------------    Select And Programme Turntable Tracks     ---------------

void programTurntable(int buttonPress)
{

	switch (buttonPress)
	{
	case 5: // position 1
		break;
	case 6: // position 2
		break;
	case 7: // position 3
		break;
	case 8: // position 4
		break;
	case 9: // position 5
		break;
	case 10: // position 6
		break;
	case 11: // move << -10
		break;
	case 12: // position < -1
		break;
	case 13: // move >> +10
		break;
	case 14: // move > + 1
		break;
	case 15: // Save position 
		break;
	}
}

//    <<<<    FINISH    ---------------    Select And Programme Turntable Tracks     ---------------

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

void calcLeastSteps(int trA, int trB)
{
	int currentLoc = PositionTrack[trA];
	int newTargetLoc = PositionTrack[trB];
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
