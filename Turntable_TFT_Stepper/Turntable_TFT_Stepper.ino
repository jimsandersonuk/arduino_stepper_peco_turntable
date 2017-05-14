/*
Name:    Turntable_TFT_Stepper.ino
Created: 4/20/2017 12:21:05 PM
Author:  jimsanderson
*/

#define DEBUG
#define TESTING

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
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/webdings12pt7b.h>

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

//	#define LCD_CS A3
//	#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4

// Assign human-readable names to some common 16-bit color values:
#define BLACK       0x0000      /*   0,   0,   0 */
#define LIGHTGREY   0xC618      /* 192, 192, 192 */
#define DARKGREY    0x7BEF      /* 128, 128, 128 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define RED         0xF800      /* 255,   0,   0 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */
#define ORANGE      0xFD20      /* 255, 165,   0 */
#define GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define GREY        0x8410
//#define MIDGREEN    0x6C80      /*  50, 230,   70*/
#define MIDGREEN    0xA64E	/**/


TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Adafruit_TFTLCD tft(YP, XM, LCD_WR, LCD_RD, LCD_RESET);

//    >>>>    FINISH    ------------------------------   TFT Setup    ------------------------------

//Define Corresponding Head and Tail

const int GetTrackTail[6] = { 4, 5, 6, 1, 2, 3 }; // Array for Tail Tracks

#ifdef DEBUG 
int PositionTrack[7] = { 0, 560, 800, 1040, 2160, 2400, 2640 }; //{ 0, 0, 0, 0, 0, 0, 0 };   // Save EEPROM addresses to array - index 0 = calibration position
#else
int PositionTrack[7] = { 0, 0, 0, 0, 0, 0, 0 };
#endif // DEBUG 

int selectedTracks[2] = { 0, 0 };

//  Calibration Array
int sensorVal = digitalRead(3);         // enable Hall-effect Sensor on Pin 3
int arrayCalibrate[5] = { 0, 0, 0, 0, 0 };   // Array to pass in calibration run results

											 // Programme Track Positions
int currentStepPosition = 0;  // current step number
int storeProgTracks[6] = { 0, 0, 0, 0, 0, 0 };
boolean chkOverwrite = false;
// isReleased tries to make sure the motor is not continuously released

//    >>>>    FINISH    ------------------------   Track Step Definitions   ------------------------

//    >>>>    START     --------------------   Parameters for turntable move    --------------------

boolean newTargetLocation = false;
boolean inMotion = false;
boolean isTurntableHead = true;
boolean isTurntableHeadChanged = true;
boolean isReleased = false;
boolean displayRotatingCW = false;
int currentFunction = 0; // "AutoDCC", "Manual", "Calibrate", "Program"
const int displayRotateDelay = 5;   // This is the minimum delay in ms between steps of the stepper motor
boolean displayRotating = false;    // Is the "Display Rotate" function enabled?

long stepperLastMoveTime = 0;

int mainDiff = 0;
int distanceToGo = 0;

const int MOTOR_OVERSHOOT = 10;			// the amount of overshoot/ lash correction when approaching from CCW
int overshootDestination = -1;
const int releaseTimeout_ms = 2000;		//reduced to 2 seconds for now
const int  totalSteps = 200 * 16;		//number of steps for a full rotation

										//    >>>>    FINISH    --------------------   Parameters for turntable move    --------------------

										//    >>>>    START     -------------------------------   TFT Menu   -------------------------------

int menuPage = 0;
int storeKeyPress;
const int turntableParameters[5] = { 60, 25, 25, 5, 8 };	//radiusTurntable, turntableOffset, lengthTrack, trackWidth, inner radius

int refactorX = 0;
int refactorY = 0;

const int buttonArraySize = 16;
int buttonArrayX[buttonArraySize];
int buttonArrayY[buttonArraySize];
int buttonArrayT[buttonArraySize] = { 1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2 };
String buttonTextArray[buttonArraySize] = { "AutoDCC", "Manual", "Calibrate", "Program", "C", "1", "2", "3", "4", "5", "6", "<<", "<", ">>", ">","Save" };

const int tabParameters[5] = { 4, 20, 3, 14, 6 }; //tabs,tabHeight,tabPad,sidePadding,menuBorder
int tabWidth;


const int buttonParameters[3] = { 30, 30, 4 };//, GREENYELLOW, ORANGE}; //buttonHeight, buttonWidth, radiusButCorner, butColour, butActiveColour
const unsigned int buttonColour = LIGHTGREY; //GREENYELLOW;
const unsigned int buttonActiveColour = ORANGE;


int xCentre = (tft.height() / 2);
int yCentre = ((tft.width() + turntableParameters[1] +  tabParameters[4] + tabParameters[1] + tabParameters[2]) / 2);
int turnTablePos = yCentre;

//    >>>>    FINISH    -------------------------------   TFT Menu   -------------------------------


//    >>>>    START     --------------------------   Define DCC Control   --------------------------

#define kDCC_INTERRUPT  0 // Define DCC commands to Arduino
typedef struct { int address; }
DCCAccessoryAddress;  // Address to respond to
DCCAccessoryAddress gAddresses[7]; // Allows 7 DCC addresses: [XX] = number of addresses you need (including 0).

								   //    >>>>    FINISH    --------------------------   Define DCC Control   --------------------------

								   //    >>>>    START     ----------------------   Adafruit MotorShield Setup   ----------------------

Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers
Adafruit_StepperMotor *mystepper = AFMStop.getStepper(200, 2);   //Connect stepper with 200 steps per revolution (1.8 degree) to the M3, M4 terminals (blue,yellow,green,red)
																 //you can change these to SINGLE, DOUBLE, INTERLEAVE or MICROSTEP! wrapper for the motor!(3200 Microsteps / revolution)
void release2() { mystepper->release(); }

#ifdef TESTING
//int forwardstep2() { return dummyStepper(1, 25); }; //75 real
//int backwardstep2() { return dummyStepper(-1, 25); }; //75 real
#else
//void forwardstep2() { mystepper->onestep(FORWARD, MICROSTEP); }
//void backwardstep2() { mystepper->onestep(BACKWARD, MICROSTEP); }
void release2() { mystepper->release(); }
AccelStepper stepper = AccelStepper(forwardstep2, backwardstep2); // wrap the stepper in an AccelStepper object
#endif // TESTING

																  //    <<<<    FINISH    ----------------------   Adafruit MotorShield Setup   ----------------------

																  //    >>>>    START     --------------------------------   SETUP    --------------------------------
void setup()
{
	//initialiseDCC();	//Start up DCC reading
	tft.reset();
	tft.begin(0x9325);
	tft.setRotation(LCDROTATION);
	tft.fillScreen(BLACK);
	pinMode(13, OUTPUT);
	startup();
}

void startup()
{
	MenuTabs(0);
	drawMenuFrame();
	drawTurntable();
	createTrackButtons();
	createFunctionbuttons();
	printArrayToSerial();
	delay(3000);
}
//    >>>>    FINISH    --------------------------------   SETUP    --------------------------------

//    >>>>    START     ---------------------------------   LOOP   ---------------------------------
void loop()
{
	digitalWrite(13, HIGH);
	TSPoint p = ts.getPoint();
	digitalWrite(13, LOW);

	// if sharing pins, you'll need to fix the directions of the touchscreen pins 
	//pinMode(XP, OUTPUT); 
	pinMode(XM, OUTPUT);
	pinMode(YP, OUTPUT);
	//pinMode(YM, OUTPUT); 

	
	//printAllFontCharacters();

	if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
	{
		translateLCD(p.x, p.y);
		int selectedButton = touchButton(refactorX, refactorY);

		if (selectedButton != storeKeyPress)
		{
#ifdef DEBUG
			Serial.println(selectedButton);
#endif
			decideButtonAction(selectedButton);
			storeKeyPress = selectedButton;
		}
	}

}

//    >>>>    FINISH    ---------------------------------   LOOP   ---------------------------------

//    >>>>    START     --------------------------    EEPROM Commands     --------------------------
/*
void EEPROMWritelong(int address, int value)
{
if (currentFunction == 2)
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
if (currentFunction == 2) {
address = 0;
}
int val1 = value / 100;
int val2 = value % 100;
int valueJoin = (val1 * 100) + val2;
PositionTrack[address] = valueJoin;
}

int EEPROMReadlong(int address)
{
if (currentFunction == 2)
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
isTurntableHead = enable;
selectedTracks[1] = PositionTrack[i];
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
*/
//    >>>>    START     ---------------------------   Draw Turntable     ---------------------------

void drawAll(boolean calibrationMode)
{
	drawTurntable();
	drawTurntableBridge(0, true);
	drawTracks(calibrationMode);
	drawTracks(true);	

#ifdef DEBUG
	printArrayToSerial();
#endif // DEBUG

	drawButtons(3,false);
}

void drawTurntable()
{
	tft.fillCircle(xCentre, turnTablePos, turntableParameters[0], GREY);
	tft.fillCircle(xCentre, turnTablePos, turntableParameters[0] - 4, BLACK);
}

void drawTurntableBridge(int angle, boolean show)
{
	int dispCol = GREY;	int dispCol1 = GREEN;	int dispCol2 = RED;

	if (show != true)
	{
		dispCol = BLACK; dispCol1 = BLACK; 	dispCol2 = BLACK;
	}

	int adjAngle = correctAngle(angle);

	int top1 = ((360 - turntableParameters[3]) + adjAngle);
	int top2 = ((turntableParameters[3] + adjAngle) + 180);
	int bot1 = (turntableParameters[3] + adjAngle);
	int bot2 = (((360 - turntableParameters[3]) + adjAngle) - 180);

	int innerTurnRad = turntableParameters[0] - turntableParameters[4];

	drawTrackLine(top1, top2, innerTurnRad, innerTurnRad, 0, dispCol);
	drawTrackLine(bot1, bot2, innerTurnRad, innerTurnRad, 0, dispCol);
	drawTrackLine(top1, bot1, innerTurnRad, innerTurnRad, 0, dispCol);
	drawTrackLine(top2, bot2, innerTurnRad, innerTurnRad, 0, dispCol);

	drawMarker(adjAngle, innerTurnRad - 4, 2, dispCol1);
	drawMarker(adjAngle - 180, innerTurnRad - 4, 2, dispCol2);
}

void drawTrackLine(int p1, int p2, int radius1, int radius2, int offset, int colour)
{
	int aX = findX(xCentre, radius1, p1);
	int aY = findY(turnTablePos, radius1, p1);
	int bX = findX(xCentre, radius2, p2);
	int bY = findY(turnTablePos, radius2, p2);
	tft.drawLine(aX, aY + offset, bX, bY + offset, colour);
}

void drawMarker(int p1, int radius1, int size, int colour)
{
	int aX = findX(xCentre, radius1, p1);
	int aY = findY(turnTablePos, radius1, p1);
	tft.fillCircle(aX, aY, size, colour);
}

void drawTracks(boolean isTrackCalibration)
{
	int t = 1;

	if (currentFunction == 2)
	{
		t = 0;
	}
	int trackArray = sizeof(PositionTrack) / sizeof(PositionTrack[0]);

	for (int i = t; i < trackArray; i++)
	{
		int adjAngle = correctAngle(convertStepDeg(PositionTrack[i], false));
		int innerTrackRad = turntableParameters[0];
		int outerTrackRad = turntableParameters[0] + turntableParameters[2];
		drawTrackLine(adjAngle, adjAngle, innerTrackRad, outerTrackRad, turntableParameters[3] - 1, GREY);
		drawTrackLine(adjAngle, adjAngle, innerTrackRad, outerTrackRad, -(turntableParameters[3] - 1), GREY);
		//drawButtonTracks(i, buttonTextArray[i + tabParameters[0]], adjAngle, outerTrackRad + (buttonParameters[0] / 2), false);
	}
}
//    >>>>    FINISH    ---------------------------   Draw Turntable     ---------------------------

//    >>>>    START     ----------------------------   Menu Functions   ----------------------------	
void MenuTabs(int tabSelected)
{

	int tabX;
	int tabY = tabParameters[4];

	int lastTabX = 0;
	tabWidth = (tft.width() - ((tabParameters[3] * 2) + (tabParameters[2] * (tabParameters[0] - 1)))) / tabParameters[0];
	tft.setTextColor(BLACK);

	for (int i = 0; i < tabParameters[0]; i++)
	{
		if (i == 0)
			tabX = tabParameters[3];
		else
			tabX = lastTabX + tabWidth + tabParameters[2];
		if (tabSelected == i)
		{
			tft.fillRect(tabX, tabY, tabWidth, tabParameters[1], LIGHTGREY);
			//tft.setFont(&RobotoBold5pt7b);
		}
		else
		{
			tft.fillRect(tabX, tabY, tabWidth, tabParameters[1], DARKGREY);
			//tft.setFont(&Roboto5pt7b);
		}

		writeButtonArray(buttonTextArray[i], tabX, tabY, 1);
		tft.setCursor(tabX + 2, tabY + 3);
		tft.print(buttonTextArray[i]);
		lastTabX = tabX;
		tft.setFont();
	}
}

void drawMenuFrame()
{
	tft.fillRoundRect(tabParameters[4], tabParameters[4] + tabParameters[1], tft.width() - (tabParameters[4] * 2), tft.height() - tabParameters[1] - (tabParameters[4] * 2), 3, LIGHTGREY);
	tft.fillRoundRect(tabParameters[4] + tabParameters[2], tabParameters[4] + tabParameters[1] + tabParameters[2], tft.width() - ((tabParameters[4] + tabParameters[2]) * 2), tft.height() - tabParameters[1] - ((tabParameters[4] + tabParameters[2]) * 2), 3, BLACK);
}

void decideButtonAction(int buttonPress)
{
	if (buttonPress < 0) return;

	if (buttonPress < tabParameters[0])
	{
		currentFunction = 0;
		MenuTabs(buttonPress);
		switch (buttonPress)
		{
		case 0:
			break;
		case 1:
			currentFunction = 1;
			drawAll(false);
			break;
		case 2:
			currentFunction = 2;
			break;
		case 3:
			currentFunction = 3;
			break;
		}
	}
	if (buttonPress >= tabParameters[0])
	{
		switch (menuPage)
		{
		case 0:
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;

		}
	}
}
/*
void sleepTFT()
{

if (sleep >= sleepTimer)
{
asleep = true;
tft.fillScreen(BLACK);
}
else sleep++;
}
*/

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
	float deg = (unit / float(totalSteps) * 360);
	//Serial.println(deg);

	if (toSteps == true)
		return step;
	else
		return deg;
}

int correctAngle(int angle)
{
	int screenZero = 360 * ((float)LCDROTATION / 4);
	Serial.println(screenZero);
	int adjAngle = screenZero + (angle % 360);
	Serial.println(adjAngle);
	if (adjAngle >= 360)
	{
		adjAngle = adjAngle - 360;
	}
	Serial.println(adjAngle);
	return adjAngle;
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

		float refY = 1 - (((float)tX - (float)rMinX) / ((float)rMaxX - (float)rMinX)); //* tft.width();
		float refX = 1 - (((float)tY - (float)rMinY) / ((float)rMaxY - (float)rMinY)); // *tft.height();
		refactorX = refX * tft.width();
		refactorY = refY * tft.height();
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

	for (int i = 0; i < arraySize; i++)
	{
		int pX = buttonArrayX[i];
		int pY = buttonArrayY[i];
		int bt = buttonArrayT[i];

		switch (bt)
		{
		case 1:
			pH = tabParameters[1];
			pW = tabWidth;
			break;
		case 2:
			pH = buttonParameters[0];
			pW = buttonParameters[1];
		case 3:
			pH = buttonParameters[0];
			pW = buttonParameters[1];
			}

		if (bt == 0)
		{
			i = arraySize;
		}

		if (tX > pX && tX<pX + pW && tY > pY && tY < pY + pH)
		{
			buttonPressed = i;
			i = arraySize;
		}
	}

	return buttonPressed;
}

void createTrackButtons()
{
	int trackArray = sizeof(PositionTrack) / sizeof(PositionTrack[0]);

	for (int i = 0; i < trackArray; i++)
	{	

		int degPos = correctAngle(convertStepDeg(PositionTrack[i], false));
		int buttonRadius = turntableParameters[0] + turntableParameters[2] + (buttonParameters[0] / 2);
		int pX = 0;
		int pY = 0;

		if (i == 0)
		{
			pX = findX(xCentre, turntableParameters[0] + buttonParameters[2] + tabParameters[4], degPos) - (buttonParameters[1] / 2);
			pY = findY(yCentre, turntableParameters[0] + buttonParameters[2] + tabParameters[4]*2, degPos) - (buttonParameters[0] / 2);
		}
		else 
		{
			pX = findX(xCentre, buttonRadius, degPos) - (buttonParameters[1] / 2);
			pY = findY(yCentre, buttonRadius, degPos) - (buttonParameters[0] / 2);
		}

		String buttonText = buttonTextArray[i + tabParameters[0]];
		writeButtonArray(buttonText, pX, pY, 2);

#ifdef DEBUG
		tft.drawCircle(pX, pY, 3, ORANGE);
#endif //DEBUG
	}
}

void createFunctionbuttons()
{

	int spacer1 = 40; int spacer2 = 50;
#ifdef DEBUG
	tft.drawCircle(xCentre, 40, 2, RED);
	tft.drawCircle(spacer1, 40, 3, ORANGE);
	tft.drawCircle((spacer1 + spacer2), 40, 3, ORANGE);
	tft.drawCircle(tft.width() - spacer1 - buttonParameters[0], 40, 3, ORANGE);
	tft.drawCircle(tft.width() -  (spacer1 + spacer2)-buttonParameters[0], 40, 3, ORANGE);
	tft.drawCircle(tft.width() - buttonParameters[0] - tabParameters[4]*2, tft.height() - buttonParameters[0] - tabParameters[4]*2, 3, ORANGE);
#endif //DEBUG

	writeButtonArray("<<", spacer1, 40, 3);
	writeButtonArray("<", spacer1 + spacer2, 40, 3);
	writeButtonArray(">>", tft.width() - spacer1 - buttonParameters[0], 40, 3);
	writeButtonArray(">", tft.width() - (spacer1 + spacer2) - buttonParameters[0], 40, 3);
	writeButtonArray("Save", tft.width() - buttonParameters[0] - tabParameters[4]*2, tft.height() - buttonParameters[0] - tabParameters[4]*2, 3);
}

void drawButtons(int butttonPage, boolean reset)
{
	int butColour = buttonColour;
	int minArray;
	int maxArray;
	int cursorX = 6;
	int cursorY = 17;

	/*
	String buttonTextArray[buttonArraySize] = { "AutoDCC", "Manual", "Calibrate", "Program", "C", "1", "2", "3", "4", "5", "6", "<<", "<", ">>", ">","Save" };
	int manualArray[] = { 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 };
	int calibrateArray[] = { 4 };
	int programArray[] = { 4, 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14, 15 };
*/

	switch (butttonPage)
	{
	case 1:
		minArray = 5;
		maxArray = 14;
		break;
	case 2:
		minArray = 4;
		maxArray = 4;
		break;
	case 3:
		minArray = 4;
		maxArray = 15;
		break;
	}

	for (int i = minArray; i < maxArray + 1; i++)
	{
		int pX = readButtonArray(buttonTextArray[i], 1);
		int pY = readButtonArray(buttonTextArray[i], 2);
		int bT = readButtonArray(buttonTextArray[i], 3);
		String buttonText = buttonTextArray[i];

#ifdef DEBUG
		Serial.println("X: " + String(pX) + " Y: " + String(pY) + " T: " + String(bT) + " N: " + String(buttonText));
#endif //DEBUG

		butColour = LIGHTGREY;

		if (bT == 3)
		{
			butColour = GREENYELLOW;
		}

		if (buttonTextArray[i] == "C")
		{
			butColour = YELLOW;
		}

		if (buttonTextArray[i] == "<<" || buttonTextArray[i] == ">>")
		{
			cursorX = 1;
			cursorY = 14;
		}

		if (buttonTextArray[i] == "<" || buttonTextArray[i] == ">")
		{
			cursorX = 7;
			cursorY = 14;
			butColour = MIDGREEN;
		}
		if (buttonTextArray[i] == "Save")
		{
			cursorX = 1;
			cursorY = 14;
			butColour = ORANGE;

		}

		if (reset == true) butColour = BLACK;

		tft.fillRoundRect(pX, pY, buttonParameters[0], buttonParameters[1], buttonParameters[2], butColour);

		tft.setCursor(pX + cursorX, pY + cursorY);		
		tft.setTextColor(BLACK);

		/*
		String buttonTextArray[buttonArraySize] = { "AutoDCC", "Manual", "Calibrate", "Program", "C", "1", "2", "3", "4", "5", "6", "<<", "<", ">>", ">","Save" };
		int manualArray[] = { 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 };
		int calibrateArray[] = { 4 };
		int programArray[] = { 4, 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14, 15 };
		*/
		tft.fillRoundRect(pX, pY, buttonParameters[0], buttonParameters[1], buttonParameters[2], butColour);
		tft.setCursor(pX + cursorX, pY + cursorY);
		tft.setTextColor(BLACK);

		switch (i)
		{
		case 15: //"Save"
			//buttonText = "<";
			//tft.setFont();
			//tft.setTextSize(2);
			//tft.print((char)250);
			//buttonText = "<";
			tft.setFont(&FreeSansBold9pt7b);
			tft.print("OK");

			
			break;
		default:
			tft.setFont(&FreeSansBold12pt7b);
			tft.print(buttonText);
			break;
		}
		tft.setFont();
	}
}

void writeButtonArray(String buttonText, int pX, int pY, int bt)
{
	int arrayPos = -1;
	int arraySize = sizeof(buttonTextArray) / sizeof(buttonTextArray[0]);

	for (int i = 0; i < arraySize; i++)
	{
		if (buttonTextArray[i] == buttonText)
		{
			arrayPos = i;
			i = arraySize;
		}
	}

	buttonArrayX[arrayPos] = pX; // rectangle x point
	buttonArrayY[arrayPos] = pY; // rectangle y point
	buttonArrayT[arrayPos] = bt; // rectangle height

#ifdef DEBUG

 //Serial.println("Tab: " + String(arrayPos) + " X: " + String(buttonArrayX[arrayPos]) + " Y: " + String(buttonArrayY[arrayPos]) + " H: " + String(buttonArrayT[arrayPos]));

#endif // DEBUG
}

int readButtonArray(String buttonText, int returnValue)
{
	int arrayPos = -1;
	int arraySize = sizeof(buttonTextArray) / sizeof(buttonTextArray[0]);

	for (int i = 0; i < arraySize; i++)

	{
		if (buttonTextArray[i] == buttonText) arrayPos = i;
	}

	switch (returnValue)
	{
	case 1:
		return buttonArrayX[arrayPos];
		break;
	case 2:
		return buttonArrayY[arrayPos];
		break;
	case 3:
		return buttonArrayT[arrayPos];
		break;
	default:
		break;
	}
	return arrayPos;
}

//    >>>>    FINISH    -------------------------   Button Calculations    -------------------------

//    >>>>    START     ----------------------------   Auto DCC Mode    ----------------------------
//    >>>>    FINISH    ----------------------------   Auto DCC Mode    ----------------------------

//    <<<<    START     ----------------------    Manually Move Turntable     ----------------------
//    <<<<    FINISH    ----------------------    Manually Move Turntable     ----------------------

//    <<<<    START     ---------------    Select And Programme Turntable Tracks     ---------------
//    <<<<    FINISH    ---------------    Select And Programme Turntable Tracks     ---------------

//    <<<<    START     ---------------------------    Stepper Voids     ---------------------------
/*
#ifdef TESTING

int dummyStepper(int dummySteps, int delaySteps)
{
int dummyStepPosition = 0;

if (delaySteps == 0) { delaySteps = 75; }
//		if (isRotatingCW){dummyStepPosition = currentStepPosition +1;}
//		else (dummyStepPosition = currentStepPosition -1;)

if (dummySteps > 0) { dummyStepPosition = currentStepPosition + 1; }
else { dummyStepPosition = currentStepPosition - 1; }

if (dummyStepPosition > totalSteps) { dummyStepPosition = 0; }
if (dummyStepPosition < 0) { dummyStepPosition = totalSteps; }

delay(delaySteps);

distanceToGo = selectedTracks[1] - currentStepPosition;

return dummyStepPosition;
}

#endif // TESTING

int calcLeastSteps()
{
int calcSteps = 0;
#ifdef TESTING
int getStepsDistance = selectedTracks[1] - currentStepPosition;
#else
int getStepsDistance = selectedTracks[1] - stepper.currentPosition();
#endif // TESTING

if (getStepsDistance > (totalSteps / 2))
calcSteps = getStepsDistance - totalSteps;
else if (getStepsDistance < -(totalSteps / 2))
calcSteps = getStepsDistance + totalSteps;
return calcSteps;
}

void doStepperMove()
{

#ifdef TESTING
boolean isInMotion = (abs(distanceToGo) > 0);
#else
stepper.run();	// Run the Stepper Motor
boolean isInMotion = (abs(stepper.distanceToGo()) > 0);

#endif // TESTING

boolean newTargetSet = false;

// If there is a new target location, set the target
if (newTargetLocation)
{
SetStepperTargetLocation();
newTargetSet = true;
}

if (isInMotion)
{
if ((!isInMotion) && (!newTargetSet))
{
//str1 = String(F("Not Moving!	DtG: "));
//str1 = String(stepper.distanceToGo());
//String(F(" TP: "));
//str2 = String(stepper.targetPosition());
//String(F(" CP: "));
//str3 = String(stepper.currentPosition());
//String(F(" S: "));
//str4 = String(stepper.speed());

}
//release the brake
//	brakeservo.write(servoRelease);
//	delay(5);
//	inMotionToNewTarget = isInMotion;
}
else
{
//				if ((stepper.currentPosition() % totalSteps) == 0)
if ((currentStepPosition % totalSteps) == 0)
{
//setCurrentPosition seems to always reset the position to 0, ignoring the parameter
//str1 = String(F("Current location: "));
////				str2 = String(stepper.currentPosition());
//str2 = String(currentStepPosition);
//str3 = String(F(" % STEPCOUNT.	Why here?"));
//displayOutput(true, str1 + str2, str3);
}
}

if (mainDiff < 0) { displayRotatingCW = false; }
else if (mainDiff > 0) { displayRotatingCW = true; }

}

void SetStepperTargetLocation()
{
int newTargetLoc = -1;

//if (tableTargetHead)
//{	//use head location variable
//	{
//		newTargetLoc = PositionTrack[tableTargetPosition];
//		inMotionToNewTarget = true;
//	}
//}
//else
//{	//use tail location variable
//	{
//		newTargetLoc = GetTrackTail[tableTargetPosition];
//		inMotionToNewTarget = true;
//	}
//}

if (newTargetLoc > 0)
{
//int currentLoc = stepper.currentPosition();
int currentLoc = currentStepPosition;
int mainDiff = newTargetLoc - currentLoc;
if (mainDiff > (totalSteps / 2))
{
mainDiff = mainDiff - totalSteps;
}
else if (mainDiff < (-totalSteps / 2))
{
mainDiff = mainDiff + totalSteps;
}

if (mainDiff < 0)
{
mainDiff -= MOTOR_OVERSHOOT;
overshootDestination = MOTOR_OVERSHOOT;
}
#ifdef TESTING
dummyStepper(mainDiff, 0);
#else
stepper.move(mainDiff);
#endif // TESTING

}
//programmingMode = false;
newTargetLocation = false;
}

//    ----------------------------------------------------------------------------------------------
//
//	Stepper Timer sub routine this runs from the main loop. It also supports the release function.
//
//    ----------------------------------------------------------------------------------------------

void stepperTimer()
{
int currentLoc = 0;

// Run the Stepper Motor //
//			stepper.run();
//		boolean isInMotion = (abs(stepper.distanceToGo()) > 0);

boolean isInMotion = (abs(distanceToGo) > 0);

//Check if we have any distance to move for release() timeout.	Can check the
// buffered var isInMotion because we also check the other variables.
if (isInMotion || currentFunction == 3)
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
#ifdef TESTING
dummyStepper(overshootDestination, 0);
#else
stepper.move(overshootDestination);
#endif // TESTING

overshootDestination = -1;
}

if (((millis() - stepperLastMoveTime) >= releaseTimeout_ms))
{
//If isReleased, don't release again.
isReleased = true;
//str1 = String(F("Relative Current Position: "));
//str2 = String(currentStepPosition);	//shows position the table thinks it is at (how it got here)
//displayOutput(true, str1 + str2, "");

currentLoc = currentStepPosition;	// Resets the position to the actual positive number it should be

//str1 = String(stepper.currentPosition());	//shows position the table thinks it is at (how it got here)
//int currentLoc = stepper.currentPosition();	// Resets the position to the actual positive number it should be

currentLoc = currentLoc % totalSteps;

if (currentLoc < 0) { currentLoc += totalSteps; }

#ifdef TESTING
dummyStepper(currentLoc, 0);
#else
stepper.setCurrentPosition(currentLoc);
stepper.moveTo(currentLoc);
//String(F("	Actual Current Position: "));
//str2 = String(stepper.currentPosition());	// shows the position value corrected.
#endif // TESTING

//Set the servo brake
//brakeservo.write(servoBrake);
//delay(750);

//release the motor
//release2();
//str1 = String(F("	Brake Set & Motor Released "));

}
}
}
}

//    <<<<    FINISH    ---------------------------    Stepper Voids     ---------------------------


//    <<<<    FINISH    ---------------------------    Stepper Voids     ---------------------------
*/

//    >>>>    START     -----------------------   Test Screen Calibration    -----------------------
#ifdef DEBUG
void drawTestLinesbyDegrees()
{

	int rotateColourArray[] = { 0x001F, 0x07E0, 0x07FF, 0xF800, 0xF81F, 0xFFE0, 0xFFFF, 0xFD20, 0xAFE5, 0xF81F }; // BLUE, GREEN, CYAN, RED, MAGENTA, YELLOW, WHITE, ORANGE, GREENYELLOW, PINK

	for (int i = 0; i < 8; i++)
	{
		int angle = i * 45;
		//drawTrackLine(angle, angle, turntableParameters[0], turntableParameters[0] + turntableParameters[2], 0, rotateColourArray[i]);
		drawTrackLine(correctAngle(angle), correctAngle(angle), turntableParameters[0], turntableParameters[0] + turntableParameters[2], 0, rotateColourArray[i]);
	}
}
#endif //DEBUG

void printArrayToSerial()
{
	int arraySize = sizeof(buttonTextArray) / sizeof(buttonTextArray[0]);

	for (int i = 0; i < arraySize; i++)

	{
		Serial.println("Name : " + String(buttonTextArray[i]) + " | X " + String(buttonArrayX[i]) + " | Y " + String(buttonArrayY[i]) + " | T " + String(buttonArrayT[i]));
	}


}

void printAllFontCharacters()
{
	tft.fillScreen(BLACK);
	tft.setFont();
	tft.setTextWrap(true);
	//tft.setFont(&Symbols12pt7b);
	//tft.setFont(&FreeSansBold9pt7b);
	tft.setTextSize(2);
	for (int i = 0; i < 256; i++)
	{
		tft.setTextColor(WHITE);

		//if (i % 6 == 0)
		//{
		//	tft.println(String(i) + " " + String((char)i)) + " ";
		//}
		//else
		//{
		//	tft.print(String(i) + " " + String((char)i) + " ");
		//}

		tft.print(String(i) + " " + String((char)i) + " ");
		Serial.print(String(i) + " " + String((char)i) + " ");
		delay(250);

		//tft.setTextColor(BLACK);
		//tft.println((char)i);
		if (i%50 == 0)
		{
			delay(1000);
			tft.fillScreen(BLACK);
			tft.setCursor(5, 5);
		}
	}
	tft.fillScreen(BLACK);
}

//    >>>>    FINISH    -----------------------   Test Screen Calibration    -----------------------

