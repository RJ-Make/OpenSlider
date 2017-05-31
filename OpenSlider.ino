

//Original code writen by Thomas Sanladerer under CCBYSA (Creative Commons Attribution Share Alike) 
//Commercial use is allowed, you must attribute the creator, you may remix this work and the remixed work should be made available under this license.
//More information https ://www.youmagine.com/designs/diy-arduino-based-motorized-dslr-camera-slider-with-lcd-screen#information
//
//Original code modified by RJ_Make for use with Adafruit's latest 2.8 TFT V2 Shield and other additions on 04/05/2017
//scroll routine thanks to Andrew Wendt

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "TimerOne.h"
#include <Encoder.h>
#include <stdint.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>
#include <Fonts/FreeSans9pt7b.h> // Hardware-specific library
#include <Fonts/FreeSans12pt7b.h> // Hardware-specific library
#include <Fonts/FreeSans18pt7b.h> // Hardware-specific library
#include <Fonts/FreeSans24pt7b.h> // Hardware-specific library
#include <Fonts/FreeMono24pt7b.h> // Hardware-specific library
#include <Fonts/FreeMono18pt7b.h> // Hardware-specific library
#include <Fonts/FreeMonoBold18pt7b.h> // Hardware-specific library
#include <Fonts/FreeMonoBold24pt7b.h> // Hardware-specific library

// Assign human-readable names to some common 16-bit color values
// Modify these to match the colorspace of your LCD

// Color definitions for ILI9341
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


#define DIR_PIN 30
#define STEP_PIN 33
#define STEP_MTR_ENABLE_PIN 31
#define BUTTON_PIN 19
#define ENC0_PIN 21
#define ENC1_PIN 20
#define BATT_PIN 15

//Hall effect Sensor Sunkee A3144
//You should be able to use the same code functions below for a standard
//end switch. At rest, state is High, and Pull Low for tripped
#define HALL_SENS_PIN 18

#define STEPSMM 320 //160  // 160 = 1/16 Micro Steps and 320 = 1/32 Micro Steps

#define DEBOUNCE 300
#define HALL_DEBOUNCE 3500
#define MINRUNTIME 10000
#define MINPRESSURE 20
#define MAXPRESSURE 50

//number of battery voltage points
#define LOOKUP 20

long day = 86400000; // 86400000 milliseconds in a day
long hour = 3600000; // 3600000 milliseconds in an hour
long minute = 60000; // 60000 milliseconds in a minute
long second = 1000; // 1000 milliseconds in a second

Encoder myEnc(ENC1_PIN, ENC0_PIN);
long oldPosition = 0;

volatile long numruns = 0;
volatile long period = 10000;
volatile long lastTriggered = 0;
volatile long lastHall_Triggered = 0;

long nextBattMillis = 0;
long runtime = 0;
long offset = 0;
long sspeed = 0;
long decimals = 0;

byte oldDays = 0;
byte oldHours = 0;
byte oldMinutes = 0;
byte oldSeconds = 0;

boolean button = false;
boolean srunning = false;
boolean needsInit = false;
boolean sdir = 0;
boolean oldsdir = !sdir;
boolean enabled = false;

boolean drawnStatusBlank = 1;
boolean drawnStatus = srunning;

boolean hall_trigger = false;
uint8_t driver_enable_default = LOW; // Is your stepper driver enabled HIGH or LOW (All mine are LOW)
float battery_voltage_high = 9;
float battery_voltage_low = 6;

//Splash Screen
String splash_Msg = "Welcome to Open Slider.... an Open Source Project ...";
const int width = 18;

//Hall Sensor Detected On Startup
String sesnor_Msg = "Please Move Gantry Away From End Stop  ";

float rawVolt;
float volt;

// This is calibration data for the raw touch data to the screen coordinates for the V2 Shield
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

#define STMPE_CS 8
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);
#define TFT_CS 10
#define TFT_DC 9
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

void catchButton() {
	if (lastTriggered + DEBOUNCE < millis()) {
		button = true;
		lastTriggered = millis();
	}
}

void catchHall() {
	if (lastHall_Triggered + HALL_DEBOUNCE < millis()) {
		hall_trigger = true;
		lastHall_Triggered = millis();
	}
}

void setup(void){
Serial.begin(9600);

  pinMode(STEP_PIN, OUTPUT);
    Serial.println("SET STEP_PIN OUTPUT");
  pinMode(DIR_PIN, OUTPUT);
    Serial.println("SET DIR_PIN OUTPUT");
  pinMode(STEP_MTR_ENABLE_PIN, OUTPUT);
	Serial.println("SET MOTOR ENABLE PIN OUTPUT");
  pinMode(HALL_SENS_PIN, INPUT);
	Serial.println("SET HALL PIN INPUT");

  
  tft.begin();
  if (!ts.begin()) { 
    Serial.println("Unable to start touchscreen.");
  } 
  else { 
    Serial.println("Touchscreen started."); 
  }

  Timer1.initialize(period);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  Timer1.stop();

  //Setup the Encoder Interrupt
  digitalWrite(BUTTON_PIN, HIGH);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), catchButton, FALLING);
  //Setup the Hall Sensor Interrupt
  digitalWrite(HALL_SENS_PIN, HIGH);
  attachInterrupt(digitalPinToInterrupt(HALL_SENS_PIN), catchHall, FALLING);
  //Disable drivers ENABLE pin to conserve power
  digitalWrite(STEP_MTR_ENABLE_PIN, !driver_enable_default);

   // origin = left,top landscape (USB left upper)
  tft.setRotation(1); 
  tft.setTextWrap(false); // Don't wrap text to next line
  
 //Check Gantry Position
  if (CheckPosition()) {
	  screen_Hall_Sensor_Startup();
  }
  else {
     screen_splash();
  }

  //Get the inital screen information displayed
  updateLCDTime(true);
  updateLCDStatus();
  //Initialize battery information
  rawVolt = ((float)analogRead(BATT_PIN) + analogRead(BATT_PIN) + analogRead(BATT_PIN) + analogRead(BATT_PIN) + analogRead(BATT_PIN)) / 5;
  volt = toVolt(rawVolt);
}

void callback()
{
	numruns++;
	digitalWrite(STEP_PIN, digitalRead(STEP_PIN) ^ 1);
}

void loop() {

	// Check to see if anything is going on

	long newPosition = myEnc.read();

	if ((newPosition < 0) && (newPosition < offset)) {
		offset = newPosition;
	}
	newPosition = newPosition - offset;

	if (newPosition != oldPosition) {
		oldPosition = newPosition;
		updateRuntime();
		setPeriod();
	}

	if (button) {
		if ((sspeed == 0) && (decimals == 0)) {
			srunning = 0;
		}
		else {
			srunning = !srunning;
			digitalWrite(STEP_MTR_ENABLE_PIN, driver_enable_default);
			Serial.println("Enable Pin LOW");
			setPeriod();
		}
		updateLCDStatus();
		button = false;
	}

	if (hall_trigger) {
		Serial.println("Hall Triggered");
		DecelerationSwith(!sdir);
		updateLCDStatus();
		hall_trigger = false;
	}

	// Retrieve a point  
	TS_Point p = ts.getPoint();
	// Scale using the calibration #'s
	// and rotate coordinate system
	p.x = map(p.x, TS_MINY, TS_MAXY, 0, tft.height());
	p.y = map(p.y, TS_MINX, TS_MAXX, 0, tft.width());
	int y = tft.height() - p.x;
	int x = p.y;
	//Serial.println(p.y); // Used to check the y axis
	if (ts.touched()) {
		if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { // Used for some light debouncing

			//Used for 'calibrating' the touch pressure params
			//Serial.print("Z-Pressure Is: ");
			//Serial.println(p.z);

			// See if there's any  touch data for us
			if (!ts.bufferEmpty())
			{
				if (p.y < 170) {
					Serial.println("left Touch");
					sdir = 0;
				}
				else {
				Serial.println("right Touch");
					sdir = 1;
				}
				updateLCDStatus();
				digitalWrite(DIR_PIN, sdir);
			}
		}
	}
		//Lets check and update the battery information
       UpdateBattery();
}

	void UpdateBattery() {
		if ((long)(millis() - nextBattMillis) >= 0) {
			//update Battery status
			nextBattMillis += 10000;
			rawVolt = ((float)analogRead(BATT_PIN)) * 0.02 + rawVolt * 0.98;
			volt = toVolt(rawVolt);
			tft.setTextColor(WHITE);

			if (volt < battery_voltage_high) {
				tft.setTextColor(GREEN);
			}

			if (volt < (battery_voltage_high - 3.5)) {
				tft.setTextColor(YELLOW);
			}

			if (volt < battery_voltage_low) {
				tft.setTextColor(RED);
			}

			tft.fillRect(0, 110, 150, 22, BLACK);
			tft.setTextSize(1);
			tft.setCursor(0, 130);
			tft.setFont(&FreeSans12pt7b);
			tft.print(mapFloat(volt, battery_voltage_low, battery_voltage_high, 0, 100));   tft.print("% ");
			tft.print(volt);   tft.print("V ");
			Serial.print(volt); Serial.print("; "); Serial.println(millis());

		}
	}

	void screen_splash() {
		tft.fillScreen(RED);
		tft.fillScreen(WHITE);
		tft.fillScreen(BLUE);
		tft.setTextColor(WHITE, BLACK); // White on black
		tft.setTextSize(5); // large letters for the splash
			for (int offset = 0; offset < splash_Msg.length(); offset++){
				// Construct the string to display for this iteration
				String t = "";
				for (int i = 0; i < width; i++)
					t += splash_Msg.charAt((offset + i) % splash_Msg.length());

				// Print  the string for this iteration
				tft.setCursor(0, tft.height() / 2 - 10);  // display will be halfway down screen
				tft.print(t);

				// Short delay so the text doesn't move too fast
				delay(50);
			}

		//set back to black
		tft.fillScreen(BLACK);
	}

	void screen_Hall_Sensor_Startup() {
		tft.fillScreen(RED);
		tft.fillScreen(WHITE);
		tft.fillScreen(BLUE);
		tft.setTextColor(WHITE, BLACK); // White on black
		tft.setTextSize(5); // large letters for the splash
		for (int offset = 0; offset < sesnor_Msg.length(); offset++) {
			// Construct the string to display for this iteration
			String t = "";
			for (int i = 0; i < width; i++)
				t += sesnor_Msg.charAt((offset + i) % sesnor_Msg.length());

			// Print  the string for this iteration
			tft.setCursor(0, tft.height() / 2 - 10);  // display will be halfway down screen
			tft.print(t);

			// Short delay so the text doesn't move too fast
			delay(100);
		}

		//set back to black
		tft.fillScreen(BLACK);
	}

	void updateLCDStatus() {
		if (sdir != oldsdir) {
			if (!sdir) {

				tft.fillTriangle(
					320, 200, // peak
					280, 240, // bottom left
					320, 240, // bottom right
					BLACK);


				tft.fillTriangle(
					0, 200, // peak
					0, 240, // bottom left
					40, 240, // bottom right
					tft.color565(255, 0, 50));

			}

			else {
				tft.fillTriangle(
					0, 200, // peak
					0, 240, // bottom left
					40, 240, // bottom right
					BLACK);

				tft.fillTriangle(
					320, 200, // peak
					280, 240, // bottom left
					320, 240, // bottom right
					tft.color565(255, 0, 50));
			}

			oldsdir = sdir;
		}

		if (runtime < MINRUNTIME) {
			if (!drawnStatusBlank) {

				tft.fillRect(75, 200, 170, 35, BLACK);
				srunning = 0;
				drawnStatus = 0;
				drawnStatusBlank = 1;
			}
			return;

		}
		else if (drawnStatusBlank || (drawnStatus != srunning)) {

			tft.fillRect(75, 200, 170, 35, BLACK);
			tft.setTextSize(1);
			tft.setCursor(0, 0);
			tft.setFont(&FreeSans18pt7b);

			if (srunning) {

				tft.setCursor(75, 230);
				tft.setTextColor(GREEN);
				tft.print("RUNNING");
			}

			else {

				tft.setCursor(100, 230);
				tft.setTextColor(BLUE);
				tft.print("READY");

			}
			drawnStatus = srunning;
			drawnStatusBlank = 0;
		}
	}


void updateRuntime() {
	if (oldPosition != 0) {
		runtime = 1000 * oldPosition * oldPosition / 4;
	}
	if (needsInit) {
		updateLCDTime(true);
	}
	else {
		updateLCDTime(false);
	}

	updateLCDStatus();
}

void updateLCDTime(boolean firstrun) {

	int days = runtime / day;                                //number of days
	int hours = (runtime % day) / hour;                       //the remainder from days division (in milliseconds) divided by hours, this gives the full hours
	int minutes = ((runtime % day) % hour) / minute;         //and so on...
	int seconds = (((runtime % day) % hour) % minute) / second;

	tft.setCursor(0, 20);
	tft.setTextSize(4);
	tft.setFont();

	if (oldPosition == 0) {

		tft.setCursor(42, 70);
		tft.setFont(&FreeMonoBold24pt7b);
		tft.setTextSize(2);
		tft.fillRect(0, 5, 320, 28, BLACK); //clear time
		tft.fillRect(0, 54, 320, 35, BLACK); // clear speed

		tft.setTextColor(RED);
		tft.print("HALT");
		srunning = false;

		tft.setTextSize(1);
		tft.setFont(&FreeMono18pt7b);
		tft.setCursor(0, 100);
		tft.setTextColor(WHITE);
		//   tft.print("0 mm/s");

		sspeed = 0;
		decimals = 0;
		needsInit = true;
	}

	else {

		tft.setTextColor(WHITE);

		if (firstrun) {
			tft.fillRect(42, 24, 230, 30, BLACK); //clear HALT/STOP
		}

		tft.setFont(&FreeMonoBold18pt7b);
		tft.setTextSize(1);

		//DAYS
		if ((days != oldDays) || firstrun) {
			tft.setCursor(0, 30);

			tft.fillRect(0, 5, 42, 28, BLACK);
			if (days < 10) {
				tft.print("0");
			}
			tft.print(days);
			oldDays = days;
		}

		//HOURS
		if (firstrun) {

			tft.print("d");
		}


		tft.setCursor(80, 30);

		if ((hours != oldHours) || firstrun) {
			tft.fillRect(80, 5, 42, 28, BLACK);
			if (hours < 10) {
				tft.print("0");
			}
			tft.print(hours);
			oldHours = hours;
		}


		//MINUTES
		if (firstrun) {
			tft.print("h");
		}
		tft.setCursor(160, 30);

		if ((minutes != oldMinutes) || firstrun) {
			tft.fillRect(160, 5, 42, 28, BLACK);
			if (minutes < 10) {
				tft.print("0");
			}
			tft.print(minutes);
			oldMinutes = minutes;
		}


		//SECONDS
		if (firstrun) {

			tft.print("m");
		}
		tft.setCursor(240, 30);

		if ((seconds != oldSeconds) || firstrun) {
			tft.fillRect(240, 5, 42, 28, BLACK);
			if (seconds < 10) {
				tft.print("0");
			}
			tft.print(seconds);
			oldSeconds = seconds;
		}

		if (firstrun) {

			tft.print("s");

			needsInit = false;
		}

		if ((oldPosition + offset) != myEnc.read()) {
			return;
		}

		tft.fillRect(0, 54, 320, 35, BLACK);
		tft.setCursor(0, 80);
		tft.setTextSize(1);
		tft.setFont(&FreeMono18pt7b);
		tft.setTextColor(WHITE);

		sspeed = (1000000) / (runtime);

		if (runtime < (MINRUNTIME * 2)) {
			tft.setTextColor(YELLOW);
		}
		if (runtime < MINRUNTIME) {

			tft.setTextColor(RED);
		}

		tft.print(sspeed);
		tft.print(".");
		decimals = ((1000000000) / (runtime)) - (((1000000) / (runtime)) * 1000);

		if (decimals < 100) {
			if (decimals < 10) {
				tft.print("0");
			}
			tft.print("0");
		}

		tft.print(decimals);
		tft.print(" mm/s");
	}
}


void setPeriod() {
	if ((runtime < MINRUNTIME) | !srunning) {
		Timer1.stop();
		  // Disable Driver ENABLE pin to save power
		if (enabled)
		{
			digitalWrite(STEP_MTR_ENABLE_PIN, !driver_enable_default);
			enabled = false;
		}
	}

	else {
		float ssspeed = 1000000 / ((float)runtime);
		float sps = ssspeed * STEPSMM;
		float pperiod = 500000 / sps;
		Timer1.setPeriod(pperiod);
		enabled = true;
		Serial.println(pperiod);
	}
}

float toVolt(float rawADC){
	return mapFloat(rawADC, 0, 1023, 0, 10.8); //empirical calibration
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float lookup(float inval, short lut[][2], short clamp) {
	float out;
	byte i;

	for (i = 1; i<LOOKUP; i++)
	{
		if (lut[i][0] > inval)
		{
			return lut[i - 1][1] + (inval - lut[i - 1][0]) * (lut[i][1] - lut[i - 1][1]) / (lut[i][0] - lut[i - 1][0]);
		}
	}

	if (i == LOOKUP) {
		return clamp;
	}
}

void DecelerationSwith(boolean bdir) {

		float ssspeed = 1000000 / ((float)runtime);
		float sps = ssspeed * STEPSMM;
		float pperiod = 500000 / sps;
		float Deceleratepperiod = pperiod;
		enabled = true;

		Serial.println(Deceleratepperiod);
		//Extreamly Simple Deceleration loop
		for (int i = 1; i <= 60; i++) {
			Deceleratepperiod = Deceleratepperiod + 25;
			Timer1.setPeriod(Deceleratepperiod);
			Serial.println(Deceleratepperiod);
		   delay(10);
		}

		// lets reverse now
		digitalWrite(DIR_PIN, bdir);
		
		// Let get back to our speed
		Serial.println(Deceleratepperiod);
		for (int i = 1; i <= 60; i++) {
			Deceleratepperiod = Deceleratepperiod - 25;
			Timer1.setPeriod(Deceleratepperiod);
			Serial.println(Deceleratepperiod);
			delay(10);
		}
		 sdir = bdir;
	}

boolean CheckPosition() {
	long results;
	boolean HallSenseActive;
	results = digitalRead(HALL_SENS_PIN);
	
	//Serial.print("Digital Hall Sense Reading: ");
	//Serial.println(results);

	if (results < 1) {
		HallSenseActive = true;
	}
	else {
		HallSenseActive = false;
	}
	return HallSenseActive;
}
