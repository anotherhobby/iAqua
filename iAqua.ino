// iAqua Aquarium Controller w/ iPhone-like Interface
// Written by Dan Cunningham, aka AnotherHobby @ plantedtank.net
// Much code was swiped, modified, and integrated or otherwise inspired from other public works
// All code is public domain, feel free to use, abuse, edit, and share
// Written for Arduino Mega 2560

// CURRENT VERSION 1.0.4
// - the smart startup routine has been reworked and fixed by robsworld78 at The Planted Tank forums!
// - added a first run routine that will zero out all EEPROM data that is needed by iAqua
// - see the warning immediately below:
// - ////////////////////////////////////////////////////////////////////////////////////////////
// - // WARNING!! IF YOU HAVE CONFIGURED iAqua BEFORE v 1.0.4 AND DO NOT WANT TO LOOSE YOUR DATA
// - // YOU MUST COMMENT OUT THE FISRT LINE OF THE startup() ROUTINE THAT CALLS firstRunSetup().
// - // IF YOU DO NOT, IT WILL ZERO ALL OF YOUR SETTINGS!!!!
// - /////////////////////////////////////////////////////////////////////////////////////////////
//
// VERSION 1.0.3
// - fixed smartstartup routine bugs
// - fixed dosing display when resevoir goes negative due to forgettig to hit fill
// - added robsworld78 at The Planted Tank's PWM smart startup lighting code
// - fixed issue when temp turned red, and then turned feeding time red
//
// VERSION:  1.0.2
// - more accurate math for how doses were calcualted on home screen
// - updated power schedules to update home screen immediately
// - day name was off using new RTC and Time library
//
// VERSION:  1.0.1
// - created smart startup routine
// - fixed math bug with dosing pump speed saving to eeprom
// - changed PWM pins for dosing pumps to make room for RGBW PWM pins
// - changed from previous RTC library to RTClib.h
//
// VERSION:  1.0
// - initial version

// CODE ORDER:
// variables & setup
// main loop
// routines to draw every screen
// code to capture all touches
// all other routines

// EEPROM locations (saved settings)
// SPACE // DESCRIPTION
// 0 // last feeding data saved (1 for yes)
// 1 // last feeding minute
// 2 // last feeding hour
// 3 // last feeding day
// 4 // last feeding month
// 5 // last feeding year
// 6 // feeding settings saved (1 for yes)
// 7 // feeding minutes setting
// 8 // feeding  light 1 (0 off, 1 on)
// 9 // feeding pwr light 2 (0 off, 1 on)
// 10 // feeding pwr filter (0 off, 1 on)
// 11 // feeding pwr circ (0 off, 1 on)
// 12 // feeding pwr heat (0 off, 1 on)
// 13 // feeding pwr co2 (0 off, 1 on)
// 14 // feeding pwr aux 1 (0 off, 1 on)
// 15 // feeding pwr aux 2 (0 off, 1 on)
// 16 // heater settings saved (1 for yes)
// 17 // heater off temp
// 18 // heater on temp
// 19 // heater alarm temp
// 20 // dosing settings saved (1 for yes)
// 21 // dose in mL
// 22 // dosing reseviors capacity in mL
// 23 // pump 1 sec/ml
// 24 // pump 2 sec/ml
// 25 // dosing volume saved (1 for yes)
// 26 // 
// 27 // 
// 28 // screen:  return home
// 29 // screen:  autodim level
// 30 // screen:  autodim seconds
// 31 // screen:  brightness if no dim
// 32 // pump 1 remaining volume high byte ^10th
// 33 // pump 1 remaining volume low byte
// 34 // pump 2 remaining volume high byte ^10th
// 35 // pump 2 remaining volume low byte
// 100-129 // power scheudle
// 220-243 // light ramp schedule
// 300-317 // dosing scheudle

#include <Wire.h> // needed by tons of stuff
#include <EEPROM.h>  // used to store and retrieve settings from memory
#include <UTFT.h>  // used to interface with the TFT display
#include <UTouch.h>  // used to interface with the touch controller on the TFT display
#include <tinyFAT.h> // used to acess the SD card
#include <UTFT_tinyFAT.h>  // used to read .raw images from the SD card
//#include <DS1307new.h>  // library to talk to the RTC chip
#include <RTClib.h>

#include <Time.h> // allows conversion to UNIX time for easier date/time math
#include <TimeAlarms.h>  // used to power schedules
#include <IRremote.h>  // used to send IR commands to the light, LED must be on pin 9
#include <OneWire.h> // network library to communicate with the DallasTemperature sensor, 
#include <DallasTemperature.h>  // library for the Temp sensor itself

// libraries I was using at one time, but not any more. Leaving them here in case I need them again
//#include<stdlib.h>
//#include <avr/pgmspace.h>

// Declare which fonts we will be using
extern uint8_t Sinclair_S[];
extern uint8_t arial_bold[];

// define relay pins
// all analog to save on digital pins
int pwrLight1Pin = A0;
int pwrLight2Pin = A1;
int pwrFilterPin = A2;
int pwrCircPin = A3;
int pwrHeatPin = A4;
int pwrCO2Pin = A5;
int pwrAux2Pin = A6;
int pwrAux1Pin = A7;

int alarmPin = 2; //(was 2)
int screenBrightPin = 8; // pwm pin for the LCD backlight (was 4)
int lightSensorPin = A8;  // analog pin for the ambient light sensor
int pressureSensorPin = A9;  // analog pin for the CO2 pressure sensor

byte backLight = 255;  // startup brightness to 100%
boolean backlightTouch = true; // initial setting of true to allow the screen to stay bright after boot

//define pump pins
int dosingPump1 = 10;
int dosingPump2 = 11;

// screen settings corresponding to eeprom values 28-31
byte screenRetHome, screenDimLevel, screenDimSec, screenBrightMem;

// Pins for temperature sensor
#define ONE_WIRE_BUS_W 47         //water sensor on pin 47

// for time
RTC_DS1307 RTC;
// for time calcuation, we need to know the current time zone offset
//int UTC_Offset=-5;


// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWireW(ONE_WIRE_BUS_W);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensorW(&oneWireW);      //water sensor

UTFT myGLCD(SSD1289,38,39,40,41); // start up an instance of the TFT screen
UTouch myTouch(46,45,44,43,42);  // start up an instance of for touch
//UTouch  myTouch(6, 5, 4, 3, 2);

UTFT_tinyFAT myFiles(&myGLCD);  // start up an instance to read images from the SD card

// create an IR instance
IRsend irsend;

byte dispScreen=1;    // screens are listed below                       
// 1-home, 2-feeding, , 3-power, 4-extras, 5-lights
// 6-clock, 7-feeding sched, 8-schedule, 9-heater
// 10-dosing, 11-pwer schedule, 12-schedule item

byte scheduleItem; // track which item is being scheduled

boolean updateTime=true;  // keep track of when to udpate the clock

byte currentLightMode=0;  //0=high sun, 1=mid sun, 2=low sun, 3=moon, 4=transition, 5=unknown
byte lightEditing=0;  // track if we are actively editing lights

// the next 5 variables will be set at the start of a lighting fade
int fadeDurationSeconds = 0; 
unsigned long fadeStartingSeconds = 0;
unsigned long fadeTimeLeft;
boolean fadeInProgress = false;
byte fadeFromMode = 0;  //0=high sun, 1=mid sun, 2=low sun, 3=moon
byte fadeToMode = 0;  //0=high sun, 1=mid sun, 2=low sun, 3=moon

struct RGBW // for storing light intensity values
{
  int Red;
  int Green;
  int Blue;
  int White;
};
typedef struct RGBW LightColor;

LightColor currentColor = {
  0,0,0,0}; // The current color of the light (used for fading)
LightColor lastColor = {
  0,0,0,0};   // The previous color of the light (used for fading)
LightColor targetColor = {
  0,0,0,0}; // The target color of the light (used for fading)

LightColor lightHighSun = {
  0,0,0,0}; // store the RGBW values for the CS+ M1 button
LightColor lightMidSun = {
  0,0,0,0}; // store the RGBW values for the CS+ M2 button 
LightColor lightLowSun = {
  0,0,0,0}; // store the RGBW values for the CS+ M3 button 
LightColor lightMoon = {
  0,0,0,0}; // store the RGBW values for the CS+ M4 button 

// selected lights mode buttons for lights screen
char *lightModeS[] = {
  "5hsunS.raw","5msunS.raw","5lsunS.raw","5moonS.raw"};
// off lights mode buttons for lights screen
char *lightModeF[] = {
  "5hmsunF.raw","5lsunF.raw","5moonF.raw"};
// neutral lights mode buttons for lights screen
char *lightMode[] = {
  "5hsun.raw","5msun.raw","5lsun.raw","5moon.raw"};

// lights adjustment buttons for lights screen (RGBW up and down) 
char *lightWhite[] = {
  "5Wup.raw", "5Wdown.raw"};
char *lightRed[] = {
  "5Rup.raw", "5Rdown.raw"};
char *lightGreen[] = {
  "5Gup.raw", "5Gdown.raw"};
char *lightBlue[] = {
  "5Bup.raw", "5Bdown.raw"};
char *lightGray[] = {
  "5Fup.raw", "5Fdown.raw"}; // disabled button

// editing buttons for the lights screen, disabled and enabled
char *lightEdit[] = {
  "5editF.raw", "5editN.raw"};
char *lightSave[] = {
  "5saveF.raw", "5saveN.raw"};
char *lightResync[] = {
  "5resynF.raw", "5resynN.raw"};
char *lightCancel[] = {
  "5canF.raw", "5canN.raw"};

// large power buttons for the power screen and the feeding configuration screen, off and on
char *pwrLightIcon[] = {
  "3light_F.raw","3light_N.raw"};
char *pwrFilterIcon[] = {
  "3filt_F.raw","3filt_N.raw"};
char *pwrCircIcon[] = {
  "3circ_F.raw","3circ_N.raw"};
char *pwrHeatIcon[] = {
  "3heat_F.raw","3heat_N.raw"};
char *pwrCO2Icon[] = {
  "3co2_F.raw","3co2_N.raw"};
char *pwrAux1Icon[] = {
  "3aux1_F.raw","3aux1_N.raw"};
char *pwrAux2Icon[] = {
  "3aux2_F.raw","3aux2_N.raw"};

// on off power dot under each power button on the power screen and feeding config screen
char *pwrDot[] = {
  "3dotR.raw","3dotG.raw"};

// small power icons for the home screen, off and on
char *pwrLightIconS[] = {
  "1lightF.raw","1lightN.raw"};
char *pwrFilterIconS[] = {
  "1filtF.raw","1filtN.raw"};
char *pwrCircIconS[] = {
  "1circF.raw","1circN.raw"};
char *pwrHeatIconS[] = {
  "1heatF.raw","1heatN.raw"};
char *pwrCO2IconS[] = {
  "1co2F.raw","1co2N.raw"};
char *pwrAux1IconS[] = {
  "1aux1F.raw","1aux1N.raw"};
char *pwrAux2IconS[] = {
  "1aux2F.raw","1aux2N.raw"};

// small light mode icons for home screen
char *lightModeSm[] = {
  "1hsun.raw","1msun.raw","1lsun.raw","1moon.raw"};

// 24 pixel up and down arrow buttons used on several screens
char *arrowButton[] = {
  "24whUp.raw", "24whDn.raw"};

// enabled or not enabled small check boxes for the power schedule screen
char *schedActive[] = {
  "11dis.raw","11enab.raw"};
// enabled or not enabled large check boxes for the power item schedule screen
char *schedActiveB[] = {
  "11disB.raw","11enabB.raw"};

// days and month character strings for displaing at the top of the screen
char *Day[] = {
  "","Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
char *Mon[] = {
  "","Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};

// used for time
struct RTC_T
{  
  int tHour;
  int tMinute;
  int tSecond;
  int tDow;
  int tDay;
  int tMonth;
  int tYear;
} 
tmpRTC, prevRTC, saveRTC;

// used for storing power states of relays
struct PWR
{  
  byte pwrLight1;
  byte pwrLight2;
  byte pwrFilter;
  byte pwrCirc;
  byte pwrHeat;
  byte pwrCO2;
  byte pwrAux1;
  byte pwrAux2;
} 
feedPower, preFeedPower, globalPower;

// holds the schedule for power relays and light ramping
struct PWRSCHED
{  
  byte active;
  byte onHour;
  byte onMinute;
  byte offHour;
  byte offMinute;
} 
schedLights1, schedLights2, schedCirc, schedCo2, schedAux1, schedAux2, ramp1, ramp2, ramp3, ramp4, ramp5, ramp6;

// holds the schedulin for 2 dosing pumps
struct PUMPSCHED
{  
  byte onHour;
  byte onMinute;
  byte Sunday;
  byte Monday;
  byte Tuesday;
  byte Wednesday;
  byte Thursday;
  byte Friday;
  byte Saturday;
} 
pump1, pump2;


int x, y; //touch coordinates

boolean feedingActive=false; // track if feeding is currently active
byte feedingMins=0; // stores how long the feeding should be

boolean heaterWarning=false; // keeps track if there is an active overheating issue
boolean heaterWarningCleared=true; // keeps track if we clear the warning, impacts the icon on the home screen
byte heatOffTemp, heatOnTemp, coldWarnTemp;


// various millis to keep track of
unsigned long prevMillisTouch = 0; // track time between touches
unsigned long prevMillis5sec = 0; // track 5 seconds for refreshing clock and temp
unsigned long feedingMillis = 0; // track how long we've been feeding
unsigned long millisDim = 0; // used for brightness adjustment
unsigned long millisHome = 0; // used for returning home after configured time

float tempC = 0;  // water temperature
char tempstring[7];  // water temperature as a string
DeviceAddress waterSensor;

int freeRam ()
{
  // Returns available SRAM
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// if you have a Current Satellite Plus, this is true
// if you are controlling your lights directly with PWM, this is false
boolean lightCSP = true;
int maxRGBW = 100;

// define RGBW PWM pins
int ledRedPin = 6;
int ledBluePin = 5;
int ledGreenPin = 4;
int ledWhitePin = 3;

// REMOTE CONTROL CODES FOR CURRENT SATELLITE PLUS
const unsigned long POWER = 0x20DF02FD;
const unsigned long FULLORANGE = 0x20DF3AC5;
const unsigned long FULLLIGHTBLUE = 0x20DFBA45;
const unsigned long FULLPURPLE = 0x20DF827D;
const unsigned long FULLWHITE = 0x20DF1AE5;
const unsigned long FULLYELLOW = 0x20DF9A65;
const unsigned long FULLBLUE = 0x20DFA25D;
const unsigned long REDUP = 0x20DF2AD5;
const unsigned long REDDOWN = 0x20DF0AF5;
const unsigned long GREENUP = 0x20DFAA55;
const unsigned long GREENDOWN = 0x20DF8A75;
const unsigned long BLUEUP = 0x20DF926D;
const unsigned long BLUEDOWN = 0x20DFB24D;
const unsigned long WHITEUP = 0x20DF12ED;
const unsigned long WHITEDOWN = 0x20DF32CD;
const unsigned long M1 = 0x20DF38C7;
const unsigned long M2 = 0x20DFB847;
const unsigned long M3 = 0x20DF7887;
const unsigned long M4 = 0x20DFF807;
const unsigned long MOONLIGHT = 0x20DF18E7;
const unsigned long MOONDARK = 0x20DF9867;
const unsigned long MOONCLOUDS = 0x20DF58A7;
const unsigned long SUNRISE = 0x20DFD827;
const unsigned long CLOUDS1 = 0x20DF28D7;
const unsigned long CLOUDS2 = 0x20DFA857;
const unsigned long CLOUDS3 = 0x20DF6897;
const unsigned long CLOUDS4 = 0x20DFE817;
const unsigned long CLOUDS5 = 0x20DFC837;
const unsigned long STORM1 = 0x20DF08F7;
const unsigned long STORM2 = 0x20DF8877;
const unsigned long STORM3 = 0x20DF48B7;

/*
// REMOTE CONTROL CODES FOR ECOXOTIC E-SERIES
const unsigned long POWER = 0x20DF02FD;
const unsigned long FULLORANGE = 0x20DF3AC5; // NOT UPDATED YET
const unsigned long FULLLIGHTBLUE = 0x20DFBA45; // NOT UPDATED YET
const unsigned long FULLPURPLE = 0x20DF827D; // NOT UPDATED YET
const unsigned long FULLWHITE = 0x20DF1AE5; // NOT UPDATED YET
const unsigned long FULLYELLOW = 0x20DF9A65; // NOT UPDATED YET
const unsigned long FULLBLUE = 0x20DFA25D; // NOT UPDATED YET
const unsigned long REDUP = 0x20DF0AF5;
const unsigned long REDDOWN = 0x20DF38C7;
const unsigned long GREENUP = 0x20DF8A75;
const unsigned long GREENDOWN = 0x20DFB847;
const unsigned long BLUEUP = 0x20DFB24D;
const unsigned long BLUEDOWN = 0x20DF7887;
const unsigned long WHITEUP = 0x20DF32CD;
const unsigned long WHITEDOWN = 0x20DFF807;
const unsigned long M1 = 0x20DF58A7;
const unsigned long M2 = 0x20DF9867;
const unsigned long M3 = 0x20DF18E7;
const unsigned long M4 = 0x20DFD827;
const unsigned long MOONLIGHT = 0x20DF18E7; // NOT UPDATED YET
const unsigned long MOONDARK = 0x20DF9867; // NOT UPDATED YET
const unsigned long MOONCLOUDS = 0x20DF58A7; // NOT UPDATED YET
const unsigned long SUNRISE = 0x20DFD827; // NOT UPDATED YET
const unsigned long CLOUDS1 = 0x20DF28D7; // NOT UPDATED YET
const unsigned long CLOUDS2 = 0x20DFA857; // NOT UPDATED YET
const unsigned long CLOUDS3 = 0x20DF6897; // NOT UPDATED YET
const unsigned long CLOUDS4 = 0x20DFE817; // NOT UPDATED YET
const unsigned long CLOUDS5 = 0x20DFC837; // NOT UPDATED YET
const unsigned long STORM1 = 0x20DF08F7; // NOT UPDATED YET
const unsigned long STORM2 = 0x20DF8877; // NOT UPDATED YET
const unsigned long STORM3 = 0x20DF48B7; // NOT UPDATED YET
*/

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

//              END GLOBAL VARIABLES                      //

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


void setup()
{

  // set default values for the first ever run, you can comment this out after the first run if you want
  firstRunSetup();

  // initiate the screen and touch 
  myGLCD.InitLCD(PORTRAIT);
  myTouch.InitTouch(PORTRAIT);
  myTouch.setPrecision(PREC_MEDIUM);

  // init SD card
  file.setSSpin(53);
  file.initFAT(SPISPEED_VERYHIGH);

  Wire.begin();
  RTC.begin();

  delay(250);

  Serial.begin(9600);

  if (! RTC.isrunning()) 
  {
    // If no RTC is installed, alert on serial
    Serial.println("RTC is NOT running!\n");  // Store this string in PROGMEM
  }
  else Serial.println("RTC is running!\n");  // Store this string in PROGMEM


  //RTC.getTime();
  setSyncProvider(syncProvider);     // reference our syncProvider function instead of RTC_DS1307::get()

  // set all pin modes for output and inputs
  pinMode(pwrLight1Pin, OUTPUT);
  pinMode(pwrLight2Pin, OUTPUT);
  pinMode(pwrFilterPin, OUTPUT);
  pinMode(pwrCircPin, OUTPUT);
  pinMode(pwrHeatPin, OUTPUT);
  pinMode(pwrCO2Pin, OUTPUT);
  pinMode(pwrAux2Pin, OUTPUT);
  pinMode(pwrAux1Pin, OUTPUT);
  pinMode(screenBrightPin, OUTPUT);
  pinMode(lightSensorPin, INPUT);
  pinMode(pressureSensorPin, INPUT);
  pinMode(alarmPin, OUTPUT);

  pinMode(dosingPump1, OUTPUT);
  pinMode(dosingPump2, OUTPUT);

  // used for PWM lighting control
  pinMode(ledRedPin, OUTPUT);
  pinMode(ledBluePin, OUTPUT);
  pinMode(ledGreenPin, OUTPUT);
  pinMode(ledWhitePin, OUTPUT);

  // PWM lights can go up to 255, so we up this value if we are using PWM
  if (lightCSP==false) int maxRGBW = 255; 

  sensorW.begin();           //start up temperature library
  sensorW.getAddress(waterSensor, 0);   //get addresses of temperature sensor

  // get screen settings from eeprom
  screenRetHome=EEPROM.read(28);
  screenDimLevel=EEPROM.read(29);
  screenDimSec=EEPROM.read(30);
  screenBrightMem=EEPROM.read(31);

  // get heat warning temps
  heatOffTemp=EEPROM.read(17);
  heatOnTemp=EEPROM.read(18);
  coldWarnTemp=EEPROM.read(19);

  analogWrite(screenBrightPin, screenBrightMem); // turn up screen brightness right away

  // get feeding settings from EEPROM
  feedingMins=EEPROM.read(7);
  feedPower.pwrLight1=EEPROM.read(8);
  feedPower.pwrLight2=EEPROM.read(9);
  feedPower.pwrFilter=EEPROM.read(10);
  feedPower.pwrCirc=EEPROM.read(11);
  feedPower.pwrHeat=EEPROM.read(12);
  feedPower.pwrCO2=EEPROM.read(13);
  feedPower.pwrAux1=EEPROM.read(14);
  feedPower.pwrAux2=EEPROM.read(15);

  /*
  // turn on/off power relays, you can just change between On and Off to change these
   AlarmPwrLight1_On();
   AlarmPwrLight2_On();
   AlarmPwrFilter_On();
   AlarmPwrCirc_On();
   AlarmPwrHeat_On();
   AlarmPwrCO2_On();
   AlarmPwrAux1_Off();
   AlarmPwrAux2_Off();
   */
  // read in power schedule
  readPowerSchedule();


  // read in light ramping schedule
  readRampSchedule();

  // read in the dosing schedule
  readDosingSchedule();

  // synchronize the lights and read in the saved settings
  resyncLights();

  // clear the screen
  myGLCD.clrScr();

  // boot up logo
  myFiles.loadBitmap(26, 80, 188, 72, "iAqua.raw");
  myFiles.loadBitmap(53, 188, 134, 25, "Copy.raw");
  delay(1000);

  // create all alarams
  updateAlarms();

  myGLCD.clrScr();

  updateTimeDate();

  // display home screen
  screenHome();  

  // start up all power and set lighting mode according to schedudule
  smartStartup();  

  // Print available SRAM for debugging, comment out if you want
  //Serial.print("\nReady...\n");
  Serial.print("SRAM: ");
  Serial.print(freeRam());
  Serial.print("\n");

  millisDim=millis(); // update millis to keep screen bright
  Serial.print("millisDim: ");
  Serial.print(millisDim);
  Serial.print("\n");

  /*
  Serial.print("lightHighSun RGBW: ");
   Serial.print(lightHighSun.Red);
   Serial.print(", ");
   Serial.print(lightHighSun.Green);
   Serial.print(", ");
   Serial.print(lightHighSun.Blue);
   Serial.print(", ");
   Serial.print(lightHighSun.White);
   Serial.print("\n");
   
   Serial.print("lightMidSun RGBW: ");
   Serial.print(lightMidSun.Red);
   Serial.print(", ");
   Serial.print(lightMidSun.Green);
   Serial.print(", ");
   Serial.print(lightMidSun.Blue);
   Serial.print(", ");
   Serial.print(lightMidSun.White);
   Serial.print("\n");
   
   Serial.print("lightLowSun RGBW: ");
   Serial.print(lightLowSun.Red);
   Serial.print(", ");
   Serial.print(lightLowSun.Green);
   Serial.print(", ");
   Serial.print(lightLowSun.Blue);
   Serial.print(", ");
   Serial.print(lightLowSun.White);
   Serial.print("\n");
   
   Serial.print("lightMoon RGBW: ");
   Serial.print(lightMoon.Red);
   Serial.print(", ");
   Serial.print(lightMoon.Green);
   Serial.print(", ");
   Serial.print(lightMoon.Blue);
   Serial.print(", ");
   Serial.print(lightMoon.White);
   Serial.print("\n");  
   */

}

time_t syncProvider()
{
  //this does the same thing as RTC_DS1307::get()
  return RTC.now().unixtime();
}

void loop()
{

  unsigned long currentMillis = millis(); // get current millis

    // if the feeding cycle is active, we keep an eye on it and stop it when the configured minutes have passed
  if (feedingActive==true)
  {
    millisDim=millis(); // keep the screen bright duing feeding
    unsigned long feedingTotalMillis = feedingMins; // calculate duration based on saved config
    feedingTotalMillis=(feedingTotalMillis*1000*60);
    if ((currentMillis-feedingMillis)>feedingTotalMillis) // keep looping until feeding is done
    {
      feedingStop();
    }
  }

  // check for touch events
  if (myTouch.dataAvailable())  {
    if (currentMillis - prevMillisTouch > 500) // make sure it's been .5 sec between touches
    {
      // set backlight touch if not already set and turn up brightness
      if (backlightTouch==false) // backlight touch is for adjusting brightness after touch
      {
        backLight=255;
        analogWrite(screenBrightPin, backLight);
        backlightTouch=true;
      }

      prevMillisTouch=currentMillis; // reset the touch timer
      millisDim=currentMillis; // reset screen dim timer
      millisHome=currentMillis; // reset return home timer

      Serial.print("Updating millisHome to: ");
      Serial.print(millisHome);
      Serial.print("\n");

      processMyTouch();
    }
  }

  // update time, temp, and feeding every 5 seconds
  if (currentMillis - prevMillis5sec > 5000) 
  {
    prevMillis5sec=millis();
    updateTimeDate();
    if (dispScreen==1) checkTemp();
    if ((dispScreen==1)||(dispScreen==2)) checkFeeding();
  }

  // this is only used for testing, so you can enter codes in the serial window
  // and get output during code testing
  if (Serial.available() > 0) {
    delay(5); //Wait for transmission to finish
    TestCodes(SerialReadInt());  // Go serial input code and process
  }

  // adjust brightness automatically unless touch event
  if (screenDimSec!=0) // if set to 0, we won't dim
  {
    if (screenDimLevel!=0) // if set to 0, we won't dim
    {
      if (backlightTouch==true)
      {
        unsigned long pastMillis=(currentMillis - millisDim);
        if (pastMillis > (1000*screenDimSec)) backlightTouch=false;
      }
      else 
      {
        autoBrightness();
      }
    }
  }

  // if we aren't on the home or feeding screen, we return after preset time of no interaction
  if (screenRetHome != 0) // if set to 0, we don't return home
  {
    if ((dispScreen!=1)&&(dispScreen!=2))
    {
      unsigned long pastMillis=(currentMillis - millisHome);
      if (pastMillis > (60000*screenRetHome)) 
      {
        if (dispScreen==9) updateAlarms(); // this will rebuild all of the schedules
        screenHome();
      }
    }
  }

  // Get the time in seconds (since 1970)
  unsigned long rightNow = now();

  // check on the fading of the lights  
  if (fadeInProgress==true)
  { 
    if(rightNow > fadeStartingSeconds + fadeDurationSeconds)   // if we have just finished the fade...
    {
      fadeInProgress = false;
      currentLightMode=fadeToMode;  // If a color fade has been completed, set the final mode

        if (lightCSP==true) // for the CSP, we send a hard code to lock in the setting and avoid drift
      {
        // send IR command for correct lighting mode
        if (currentLightMode == 0) irsend.sendNEC(M1,32);
        else if (currentLightMode == 1) irsend.sendNEC(M2,32);
        else if (currentLightMode == 2) irsend.sendNEC(M3,32);
        else if (currentLightMode == 3) irsend.sendNEC(M4,32);
      }
      if (dispScreen==1) screenHome(); // redraw the home screen if we are home
    }
    else   // if the fade is still running
    {
      // If there is a fade in progress, check if there are any IR commands that should be sent
      checkLightFade(rightNow - fadeStartingSeconds, fadeDurationSeconds);
    }
  }
  Alarm.delay(5); // must use Alarm delay to use the TimeAlarms library

}

void screenHome()  // draw main home screen showing overview info
{

  if (dispScreen!=1)   myGLCD.clrScr(); // clear if not home

  dispScreen=1;  // set screen so we can know what screen was touched later

  myGLCD.setColor(0,0,0);
  myGLCD.fillRect(0,32,239,254); // clear the screen between the header and the dock

  // draw dock, home icon, and header
  myFiles.loadBitmap(0, 254, 240, 66, "dock.raw");
  myFiles.loadBitmap(2, 2, 30, 30, "1home.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(34, 81, 255);
  myGLCD.print("HOME        ", 36, 12);

  // draw lines to divide screen
  myGLCD.setColor(130, 130, 130);
  myGLCD.drawLine(40, 31, 239, 31); // under header  
  myGLCD.drawLine(0, 110, 239, 110); // across screen below temp
  myGLCD.drawLine(56, 110, 56, 237); // 1st cutting into 4ths
  myGLCD.drawLine(105, 110, 105, 237); // 2nd cutting into 4ths
  myGLCD.drawLine(168, 110, 168, 237); // 3rd cutting into 4ths

  myGLCD.drawLine(0, 237, 239, 237); // across above dock

  // draw water temp logo
  if (heaterWarningCleared==true) // if no warning is active, or has been acknowledged
  {
    myFiles.loadBitmap(50, 44, 60, 51, "1therm.raw");
  }
  else if (heaterWarningCleared==false) // if warning hasn't been acknowledged
  {
    myFiles.loadBitmap(50, 44, 60, 51, "1thermR.raw");
  }

  // check temp and draw to screen
  checkTemp();

  // load °F image
  myFiles.loadBitmap(202, 66, 14, 12, "f.raw");

  // display feeding info
  checkFeeding();

  myFiles.loadBitmap(5, 172, 46, 46, "1feed.raw");

  // display lighting info
  checkLighting();

  // get remainding doses
  checkDosing();

  // draw power status of outputs
  myFiles.loadBitmap(178, 121, 24, 24, pwrLightIconS[globalPower.pwrLight1]);
  myFiles.loadBitmap(206, 121, 24, 24, pwrLightIconS[globalPower.pwrLight2]);
  myFiles.loadBitmap(178, 149, 24, 24, pwrFilterIconS[globalPower.pwrFilter]);
  myFiles.loadBitmap(206, 149, 24, 24, pwrCircIconS[globalPower.pwrCirc]);
  myFiles.loadBitmap(178, 177, 24, 24, pwrHeatIconS[globalPower.pwrHeat]);
  myFiles.loadBitmap(206, 177, 24, 24, pwrCO2IconS[globalPower.pwrCO2]);
  myFiles.loadBitmap(178, 205, 24, 24, pwrAux1IconS[globalPower.pwrAux1]);
  myFiles.loadBitmap(206, 205, 24, 24, pwrAux2IconS[globalPower.pwrAux2]);

}

void screenFeeding()  // start the feeding cycle and draw the feeding screen
{

  myGLCD.clrScr();
  dispScreen=2;

  updateTimeDate();

  // draw header and footer
  myGLCD.setColor(130, 130, 130);
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 307, 239, 307); // at footer
  myFiles.loadBitmap(2, 2, 30, 30, "2feed.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(0, 184, 19);
  myGLCD.print("FEEDING      ", 36, 12);
  myFiles.loadBitmap(107, 294, 26, 26, "foothome.raw");

  myGLCD.setFont(arial_bold);
  myGLCD.setColor(0, 184, 19);
  myGLCD.print("TIME REMAINING", CENTER, 60);
  myGLCD.setColor(240, 240, 255);

  unsigned long nowMillis = millis();
  unsigned long feedingTotalMillis = feedingMins;
  feedingTotalMillis=(feedingTotalMillis*1000*60);
  int feedingMinsLeft=((feedingTotalMillis-(nowMillis-feedingMillis))/1000)/60;
  char char3[3];
  itoa(feedingMinsLeft, char3, 10);
  myGLCD.print(char3, 40, 80);
  myGLCD.print("MINUTES", 88, 80);

  // buttons to stop and restart feeding
  myFiles.loadBitmap(67, 223, 48, 48, "2stop.raw");
  myFiles.loadBitmap(124, 223, 48, 48, "2restart.raw");

  // picture of fish eating
  myFiles.loadBitmap(74, 110, 92, 92, "2feeding.raw");

  // start feeding cycle if not already active
  if (feedingActive==false)
  {
    // capture current power status so we can return to it
    preFeedPower.pwrLight1=globalPower.pwrLight1;
    preFeedPower.pwrLight2=globalPower.pwrLight2;
    preFeedPower.pwrFilter=globalPower.pwrFilter;
    preFeedPower.pwrCirc=globalPower.pwrCirc;
    preFeedPower.pwrHeat=globalPower.pwrHeat;
    // there is a catch to make sure CO2 isn't turned back on if the schedule turns it off during feeding
    preFeedPower.pwrCO2=globalPower.pwrCO2; 
    preFeedPower.pwrAux1=globalPower.pwrAux1;
    preFeedPower.pwrAux2=globalPower.pwrAux2;

    // fire power relays as configured
    if (feedPower.pwrLight1==1) AlarmPwrLight1_On();
    else if (feedPower.pwrLight1==0) AlarmPwrLight1_Off();
    if (feedPower.pwrLight2==1) AlarmPwrLight2_On();
    else if (feedPower.pwrLight2==0) AlarmPwrLight2_Off();
    if (feedPower.pwrFilter==1) AlarmPwrFilter_On();
    else if (feedPower.pwrFilter==0) AlarmPwrFilter_Off();
    if (feedPower.pwrCirc==1) AlarmPwrCirc_On();
    else if (feedPower.pwrCirc==0) AlarmPwrCirc_Off();
    if (feedPower.pwrHeat==1) AlarmPwrHeat_On();
    else if (feedPower.pwrHeat==0) AlarmPwrHeat_Off();
    if (feedPower.pwrCO2==1) AlarmPwrCO2_On();
    else if (feedPower.pwrCO2==0) AlarmPwrCO2_Off();
    if (feedPower.pwrAux1==1) AlarmPwrAux1_On();
    else if (feedPower.pwrAux1==0) AlarmPwrAux1_Off();
    if (feedPower.pwrAux2==1) AlarmPwrAux2_On();
    else if (feedPower.pwrAux2==1) AlarmPwrAux2_Off();

    feedingActive=true;
    feedingMillis = millis(); // mark when feeding started

    // save feeding time to EEPROM
    //RTC.getTime();    
    RTC.now().unixtime();

    EEPROM.write(0, 1); // 0 // last feeding data saved (0 for no, 1 for yes)
    //EEPROM.write(1, RTC.minute); // 1 // last feeding minute
    //EEPROM.write(2, RTC.hour);    // 2 // last feeding hour
    //EEPROM.write(3, RTC.day);    // 3 // last feeding day
    //EEPROM.write(4, RTC.month);    // 4 // last feeding month
    //EEPROM.write(5, (RTC.year-2000));    // 5 // last feeding year
    EEPROM.write(1, minute()); // 1 // last feeding minute
    EEPROM.write(2, hour());    // 2 // last feeding hour
    EEPROM.write(3, day());    // 3 // last feeding day
    EEPROM.write(4, month());    // 4 // last feeding month
    EEPROM.write(5, (year()-2000));    // 5 // last feeding year

  }   
}

void screenPower()  // draw the screen to turn power outputs on/off
{

  dispScreen=3;

  myGLCD.clrScr();

  // draw footer
  myGLCD.setColor(130, 130, 130);
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "foothome.raw");
  updateTimeDate();

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "3power.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(222, 8, 51);
  myGLCD.print("POWER      ", 36, 12);

  myGLCD.setColor(255, 255, 255);
  myGLCD.print("MASTER", CENTER, 52);

  // all on and all off buttons
  myFiles.loadBitmap(73, 77, 40, 40, "3allon.raw");
  myFiles.loadBitmap(128, 77, 40, 40, "3alloff.raw");

  // load all power icons and power dots
  myFiles.loadBitmap(15, 139, 48, 48, pwrLightIcon[globalPower.pwrLight1]);
  myFiles.loadBitmap(34, 192, 10, 11, pwrDot[globalPower.pwrLight1]);
  myFiles.loadBitmap(69, 139, 48, 48, pwrLightIcon[globalPower.pwrLight2]);
  myFiles.loadBitmap(88, 192, 10, 11, pwrDot[globalPower.pwrLight2]);
  myFiles.loadBitmap(124, 139, 48, 48, pwrFilterIcon[globalPower.pwrFilter]);
  myFiles.loadBitmap(143, 192, 10, 11, pwrDot[globalPower.pwrFilter]);
  myFiles.loadBitmap(178, 139, 48, 48, pwrCircIcon[globalPower.pwrCirc] );
  myFiles.loadBitmap(197, 192, 10, 11, pwrDot[globalPower.pwrCirc]);
  myFiles.loadBitmap(15, 212, 48, 48, pwrHeatIcon[globalPower.pwrHeat] );
  myFiles.loadBitmap(34, 264, 10, 11, pwrDot[globalPower.pwrHeat]);
  myFiles.loadBitmap(69, 212, 48, 48, pwrCO2Icon[globalPower.pwrCO2] );
  myFiles.loadBitmap(88, 264, 10, 11, pwrDot[globalPower.pwrCO2]);
  myFiles.loadBitmap(124, 212, 48, 48, pwrAux1Icon[globalPower.pwrAux1] );
  myFiles.loadBitmap(143, 264, 10, 11, pwrDot[globalPower.pwrAux1]);
  myFiles.loadBitmap(178, 212, 48, 48, pwrAux2Icon[globalPower.pwrAux2] );
  myFiles.loadBitmap(197, 264, 10, 11, pwrDot[globalPower.pwrAux2]);

}

void screenSettings()  // draw the screen that has all of the extra settings apps
{
  dispScreen=4;

  myGLCD.clrScr();

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "4extras.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(255, 77, 0);
  myGLCD.print("SETTINGS  ", 36, 12);

  myGLCD.setColor(130, 130, 130);
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer

  myFiles.loadBitmap(107, 294, 26, 26, "foothome.raw");  // footer home button

  // draw app icon buttons
  if (globalPower.pwrLight1==1) myFiles.loadBitmap(10, 50, 48, 63, "4lights.raw");
  else if (globalPower.pwrLight1==0) myFiles.loadBitmap(10, 50, 48, 63, "4lightsF.raw");
  myFiles.loadBitmap(67, 50, 48, 63, "4clock.raw");
  myFiles.loadBitmap(124, 50, 48, 63, "4feeding.raw");
  myFiles.loadBitmap(181, 50, 48, 63, "4sched.raw");
  myFiles.loadBitmap(10, 118, 48, 63, "4heater.raw");
  myFiles.loadBitmap(67, 118, 48, 63, "4dosing.raw");
  myFiles.loadBitmap(124, 118, 48, 63, "4screen.raw");

}

void screenLights() // draw the screen for configuring the lights
{

  dispScreen=5; 
  char char3[3]; // used for converting numbers to char
  char char3t[3];

  myGLCD.clrScr();

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "5lights.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(185, 55, 255);
  myGLCD.print("LIGHTS MODES", 36, 12);

  myGLCD.setColor(130, 130, 130);
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer

  myGLCD.drawLine(0, 94, 239, 94); // vertical center line
  myGLCD.drawLine(0, 168, 239, 168); // vertical center line

  myFiles.loadBitmap(107, 294, 26, 26, "footextr.raw"); // footer button

  // draw buttons based on current mode (either selected or not selected)
  if (currentLightMode==0)// high sun
  {
    myFiles.loadBitmap(10, 39, 48, 48, lightModeS[0]);
    myFiles.loadBitmap(67, 39, 48, 48, lightMode[1]);
    myFiles.loadBitmap(124, 39, 48, 48, lightMode[2]);
    myFiles.loadBitmap(181, 39, 48, 48, lightMode[3]);
    myFiles.loadBitmap(10, 101, 48, 63, lightEdit[1]);
    myFiles.loadBitmap(181, 101, 48, 63, lightResync[1]);    

    // get RGBW for high sun
    currentColor.Red=EEPROM.read(200);
    currentColor.Green=EEPROM.read(201);
    currentColor.Blue=EEPROM.read(202);
    currentColor.White=EEPROM.read(203);

  }  
  else if (currentLightMode==1) // mid sun
  {
    myFiles.loadBitmap(10, 39, 48, 48, lightMode[0]);
    myFiles.loadBitmap(67, 39, 48, 48, lightModeS[1]);
    myFiles.loadBitmap(124, 39, 48, 48, lightMode[2]);
    myFiles.loadBitmap(181, 39, 48, 48, lightMode[3]);
    myFiles.loadBitmap(10, 101, 48, 63, lightEdit[1]);
    myFiles.loadBitmap(181, 101, 48, 63, lightResync[1]);    

    // get RGBW for mid sun
    currentColor.Red=EEPROM.read(204);
    currentColor.Green=EEPROM.read(205);
    currentColor.Blue=EEPROM.read(206);
    currentColor.White=EEPROM.read(207);
  }
  else if (currentLightMode==2) // low sun
  {
    myFiles.loadBitmap(10, 39, 48, 48, lightMode[0]);
    myFiles.loadBitmap(67, 39, 48, 48, lightMode[1]);
    myFiles.loadBitmap(124, 39, 48, 48, lightModeS[2]);
    myFiles.loadBitmap(181, 39, 48, 48, lightMode[3]);
    myFiles.loadBitmap(10, 101, 48, 63, lightEdit[1]);
    myFiles.loadBitmap(181, 101, 48, 63, lightResync[1]);    

    // get RGBW for low sun
    currentColor.Red=EEPROM.read(208);
    currentColor.Green=EEPROM.read(209);
    currentColor.Blue=EEPROM.read(210);
    currentColor.White=EEPROM.read(211);
  }
  else if (currentLightMode==3) // moon
  {
    myFiles.loadBitmap(10, 39, 48, 48, lightMode[0]);
    myFiles.loadBitmap(67, 39, 48, 48, lightMode[1]);
    myFiles.loadBitmap(124, 39, 48, 48, lightMode[2]);
    myFiles.loadBitmap(181, 39, 48, 48, lightModeS[3]);
    myFiles.loadBitmap(10, 101, 48, 63, lightEdit[1]);
    myFiles.loadBitmap(181, 101, 48, 63, lightResync[1]);

    // get RGBW for moon
    currentColor.Red=EEPROM.read(212);
    currentColor.Green=EEPROM.read(213);
    currentColor.Blue=EEPROM.read(214);
    currentColor.White=EEPROM.read(215);
  }
  else if ((currentLightMode==4)||(currentLightMode==5)) // lights in transition or unknown
  {
    myFiles.loadBitmap(10, 39, 48, 48, lightMode[0]);
    myFiles.loadBitmap(67, 39, 48, 48, lightMode[1]);
    myFiles.loadBitmap(124, 39, 48, 48, lightMode[2]);
    myFiles.loadBitmap(181, 39, 48, 48, lightMode[3]);
    myFiles.loadBitmap(10, 101, 48, 63, lightEdit[0]);
    myFiles.loadBitmap(181, 101, 48, 63, lightResync[1]);
  }  

  // draw the rest of the buttons disabled until the edit button is pressed
  myFiles.loadBitmap(67, 101, 48, 63, lightSave[0]);
  myFiles.loadBitmap(124, 101, 48, 63, lightCancel[0]);

  myFiles.loadBitmap(10, 175, 48, 48, lightGray[0]);
  myFiles.loadBitmap(67, 175, 48, 48, lightGray[0]);
  myFiles.loadBitmap(124, 175, 48, 48, lightGray[0]);
  myFiles.loadBitmap(181, 175, 48, 48, lightGray[0]);

  myFiles.loadBitmap(10, 241, 48, 48, lightGray[1]);
  myFiles.loadBitmap(67, 241, 48, 48, lightGray[1]);
  myFiles.loadBitmap(124, 241, 48, 48, lightGray[1]);
  myFiles.loadBitmap(181, 241, 48, 48, lightGray[1]);

  myGLCD.setFont(Sinclair_S);
  myGLCD.setColor(255, 255, 255);

  // draw the RGBW values to the screen

  itoa(currentColor.White, char3, 10);
  if (currentColor.White>=0 && currentColor.White<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 26, 228);

  itoa(currentColor.Red, char3, 10);
  if (currentColor.Red>=0 && currentColor.Red<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 83, 228);

  itoa(currentColor.Green, char3, 10);
  if (currentColor.Green>=0 && currentColor.Green<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 140, 228);

  itoa(currentColor.Blue, char3, 10);
  if (currentColor.Blue>=0 && currentColor.Blue<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 197, 228);

}

void screenClock()  // draw the screen for setting the date/time
{

  dispScreen=6; 

  myGLCD.clrScr();

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "6clock.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(47, 168, 208);
  myGLCD.print("CLOCK     ", 36, 12);

  myGLCD.print("TIME", CENTER, 46);
  myGLCD.print("DATE", CENTER, 167);

  myGLCD.setColor(130, 130, 130);
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 155, 239, 155); // center line

  myGLCD.setFont(Sinclair_S);
  myGLCD.setColor(255, 255, 255);

  // draw clock buttons and text
  myGLCD.print("24H", 12, 72);
  myFiles.loadBitmap(12, 89, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(12, 119, 24, 24, arrowButton[1]);
  myGLCD.print("M", 91, 72);
  myFiles.loadBitmap(83, 89, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(83, 119, 24, 24, arrowButton[1]);
  myGLCD.print("S", 172, 72);
  myFiles.loadBitmap(164, 89, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(164, 119, 24, 24, arrowButton[1]);

  // draw date buttons and text
  myGLCD.print("M", 20, 194);
  myFiles.loadBitmap(12, 211, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(12, 241, 24, 24, arrowButton[1]);
  myGLCD.print("D", 91, 194);
  myFiles.loadBitmap(83, 211, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(83, 241, 24, 24, arrowButton[1]);
  myGLCD.print("Y", 172, 194);
  myFiles.loadBitmap(164, 211, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(164, 241, 24, 24, arrowButton[1]);

  myGLCD.setFont(arial_bold);
  myGLCD.setColor(255, 77, 0);

  // get the current date and time to display
  /*
  saveRTC.tHour=RTC.hour;
   saveRTC.tMinute=(RTC.minute+1); // skip to the minute ahead to make it faster to set the time
   saveRTC.tSecond=0; // always have 0 seconds
   saveRTC.tDay=RTC.day;
   saveRTC.tMonth=RTC.month;
   saveRTC.tYear=(RTC.year-2000);
   */
  saveRTC.tHour=hour();
  saveRTC.tMinute=(minute()+1); // skip to the minute ahead to make it faster to set the time
  saveRTC.tSecond=0; // always have 0 seconds
  saveRTC.tDay=day();
  saveRTC.tMonth=month();
  saveRTC.tYear=(year()-2000);

  char char3[3];
  char char3t[3];

  // draw the date and time to the screen

  itoa(saveRTC.tHour, char3, 10);
  if (saveRTC.tHour>=0 && saveRTC.tHour<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 45, 108);

  itoa(saveRTC.tMinute, char3, 10);
  if (saveRTC.tMinute>=0 && saveRTC.tMinute<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 123, 108);

  myGLCD.print("00", 201, 108);

  itoa(saveRTC.tMonth, char3, 10);
  if (saveRTC.tMonth>=0 && saveRTC.tMonth<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 45, 230);

  itoa(saveRTC.tDay, char3, 10);
  if (saveRTC.tDay>=0 && saveRTC.tDay<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 123, 230);

  itoa(saveRTC.tYear, char3, 10);
  if (saveRTC.tYear>=0 && saveRTC.tYear<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 201, 230);

  // buttons to save or cancel
  myFiles.loadBitmap(12, 286, 84, 26, "6cancel.raw");
  myFiles.loadBitmap(144, 286, 84, 26, "6set.raw");

}

void screenFeedSettings() // screen for setting the minutes and power settings for feeding
{

  dispScreen=7; 

  myGLCD.clrScr();

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "2feed.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(0, 184, 19);
  myGLCD.print("FEED CONFIG", 36, 12);

  myGLCD.print("MINUTES", 24, 65);

  // buttons for changing minutes
  myFiles.loadBitmap(158, 46, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(158, 76, 24, 24, arrowButton[1]);

  myGLCD.setColor(255, 77, 0);

  // display current setting for minutes
  char char3[3];
  itoa(feedingMins, char3, 10);
  myGLCD.print(char3, 190, 65);

  myGLCD.setColor(222, 8, 51);
  myGLCD.print("POWER SETUP", CENTER, 121);

  myGLCD.setColor(130, 130, 130);

  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 129, 32, 129); // LEFT OF POWER
  myGLCD.drawLine(208, 129, 239, 129); // Right of power
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "footextr.raw");

  // Feeding settings stored in EEPROM
  // 6 // feeding settings saved (0 for no, 1 for yes)
  // 7 // feeding minutes setting
  // 8 // feeding  light 1 (0 off, 1 on)
  // 9 // feeding pwr light 2 (0 off, 1 on)
  // 10 // feeding pwr filter (0 off, 1 on)
  // 11 // feeding pwr circ (0 off, 1 on)
  // 12 // feeding pwr heat (0 off, 1 on)
  // 13 // feeding pwr co2 (0 off, 1 on)
  // 14 // feeding pwr aux 1 (0 off, 1 on)
  // 15 // feeding pwr aux 2 (0 off, 1 on)

  // load power icons based on saved settings
  myFiles.loadBitmap(15, 151, 48, 48, pwrLightIcon[feedPower.pwrLight1]);
  myFiles.loadBitmap(34, 204, 10, 11, pwrDot[feedPower.pwrLight1]);
  myFiles.loadBitmap(69, 151, 48, 48, pwrLightIcon[feedPower.pwrLight2]);
  myFiles.loadBitmap(88, 204, 10, 11, pwrDot[feedPower.pwrLight2]);
  myFiles.loadBitmap(124, 151, 48, 48, pwrFilterIcon[feedPower.pwrFilter]);
  myFiles.loadBitmap(143, 204, 10, 11, pwrDot[feedPower.pwrFilter]);
  myFiles.loadBitmap(178, 151, 48, 48, pwrCircIcon[feedPower.pwrCirc] );
  myFiles.loadBitmap(197, 204, 10, 11, pwrDot[feedPower.pwrCirc]);
  myFiles.loadBitmap(15, 224, 48, 48, pwrHeatIcon[feedPower.pwrHeat] );
  myFiles.loadBitmap(34, 276, 10, 11, pwrDot[feedPower.pwrHeat]);
  myFiles.loadBitmap(69, 224, 48, 48, pwrCO2Icon[feedPower.pwrCO2] );
  myFiles.loadBitmap(88, 276, 10, 11, pwrDot[feedPower.pwrCO2]);
  myFiles.loadBitmap(124, 224, 48, 48, pwrAux1Icon[feedPower.pwrAux1] );
  myFiles.loadBitmap(143, 276, 10, 11, pwrDot[feedPower.pwrAux1]);
  myFiles.loadBitmap(178, 224, 48, 48, pwrAux2Icon[feedPower.pwrAux2] );
  myFiles.loadBitmap(197, 276, 10, 11, pwrDot[feedPower.pwrAux2]);

}

void screenHeater() // screen for choosing temp settings for the heater
{

  dispScreen=8; 
  char char3[3]; // used to convernt numbers to char

  myGLCD.clrScr();

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "8heat.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(34, 81, 255);
  myGLCD.print("HEATER SETUP", 36, 12);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(40, 31, 239, 31); // under header

  // lines to divide screen
  myGLCD.drawLine(0, 131, 239, 131); 
  myGLCD.drawLine(0, 212, 239, 212);

  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "footextr.raw");

  // off icon
  myFiles.loadBitmap(17, 67, 48, 48, "8off.raw");

  myGLCD.setColor(222, 8, 51);
  myGLCD.print("OFF", 88, 84);

  // buttons for adjusting off temp
  myFiles.loadBitmap(162, 64, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(162, 94, 24, 24, arrowButton[1]);

  // get the setting for heat off and print it
  int eeprom=(EEPROM.read(17)); // 17 // heater off temp
  Serial.print(eeprom);
  itoa(eeprom, char3, 10);
  myGLCD.setColor(255, 77, 0);
  myGLCD.print(char3, 195, 84);

  // draw the on icon
  myFiles.loadBitmap(17, 150, 48, 48, "8on.raw");
  myGLCD.setColor(222, 8, 51);
  myGLCD.print("ON", 96, 164);

  // buttons for adjusting the on temp
  myFiles.loadBitmap(162, 145, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(162, 175, 24, 24, arrowButton[1]);

  // get the setting for heat on and print it
  eeprom=(EEPROM.read(18));
  Serial.print(eeprom);
  itoa(eeprom, char3, 10); // 18 // heater on temp
  myGLCD.setColor(255, 77, 0);
  myGLCD.print(char3, 195, 164);

  // draw cold warning temp
  myFiles.loadBitmap(17, 235, 48, 48, "8warn.raw");
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("LOW", 86, 235);
  myGLCD.print("WARN", 78, 255);

  // heat warning adjustment buttons
  myFiles.loadBitmap(162, 224, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(162, 254, 24, 24, arrowButton[1]);

  // get the setting for cold warning and print it
  eeprom=(EEPROM.read(19));
  Serial.print(eeprom);
  itoa(eeprom, char3, 10); // 19 // heater alarm temp
  myGLCD.setColor(255, 77, 0);
  myGLCD.print(char3, 195, 245);

}

void screenSchedule() // screen with icons to allow setting schedules
{
  dispScreen=9; 
  myGLCD.clrScr();

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "9sched.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(238, 0, 145);
  myGLCD.print("SCHEDULE", 36, 12);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "footextr.raw");

  // draw schedule icons
  myFiles.loadBitmap(10, 50, 48, 63, "9power.raw");
  myFiles.loadBitmap(67, 50, 48, 63, "4dosing.raw");
  myFiles.loadBitmap(124, 50, 48, 63, "4lights.raw");

}

void screenDosing()  // screen to configure the 2 dosing pumps
{

  dispScreen=10; 
  char char3[3]; // used to convernt numbers to char
  char char4[4]; // used to convernt numbers to char
  char char5[5];

  myGLCD.clrScr();

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "10dose.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(138, 93, 35);
  myGLCD.print("DOSING SETUP", 36, 12);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(40, 31, 239, 31); // under header

  // 3 horizontal separater lines
  myGLCD.drawLine(0, 102, 239, 102);
  myGLCD.drawLine(0, 170, 239, 170);
  myGLCD.drawLine(0, 238, 239, 238);

  //// DOSE AMOUNT
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("DOSE", 4, 53);
  myGLCD.print("AMOUNT", 4, 74);

  // buttons to adjust dose amount
  myFiles.loadBitmap(145, 42, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(145, 72, 24, 24, arrowButton[1]);

  // read dose amount setting and print it
  byte doseAmt=EEPROM.read(21); // 21 // dose in mL
  itoa(doseAmt, char3, 10);
  myGLCD.setColor(255, 77, 0);
  myGLCD.print(char3, 191, 59);

  //// RESEVOIR CAPACITY AND FILL
  myFiles.loadBitmap(4, 112, 48, 48, "10fill.raw");
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("FULL", 62, 112);
  myGLCD.setFont(Sinclair_S);
  myGLCD.print("VOL EACH", 67, 132);

  // read res capacity and do math
  int doseCap=EEPROM.read(22); // 22 // dosing reseviors capacity in mL;
  Serial.print("doseCap: ");
  Serial.print(doseCap);
  Serial.print("\n");
  doseCap=doseCap*10; // saved value is ^10
  Serial.print(doseCap);
  Serial.print("\n");
  int doses=(doseCap/doseAmt); // number of doses is the capacity divided by dose size
  itoa(doses, char3, 10);

  myGLCD.setColor(255, 77, 0);
  myGLCD.print("DOSES:", 67, 152);
  myGLCD.print(char3, 115, 152);

  // buttons for adjusting the resevoir capacity
  myFiles.loadBitmap(145, 110, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(145, 140, 24, 24, arrowButton[1]);

  // print capacity to screen
  itoa(doseCap, char3, 10);
  myGLCD.setColor(255, 77, 0);
  myGLCD.setFont(arial_bold);
  myGLCD.print(char3, 191, 127);

  //// PUMP 1 test button and calibration
  myFiles.loadBitmap(4, 181, 48, 48, "10test.raw");
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("1", 62, 191);
  myGLCD.setFont(Sinclair_S);
  myGLCD.print("PUMP RATE", 67, 181);
  myGLCD.print("MACROS", 67, 211);

  // buttons to adjust pump time
  myFiles.loadBitmap(145, 178, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(145, 208, 24, 24, arrowButton[1]);

  // read in pump 1 time and display it in seconds
  int pumpTime=EEPROM.read(23);  // 23 // pump 1 sec/ml
  pumpTime=pumpTime*10;
  itoa(pumpTime, char5, 10);
  //sprintf(char5, "%f", pumpTime); // convert float to char
  myGLCD.setColor(255, 77, 0);
  myGLCD.setFont(arial_bold);
  int px=187;
  // shift x coord to center text if needed
  if (pumpTime>999) px=175;
  myGLCD.print(char5, px, 197);  // 175 if 4 chars

  //// PUMP 2 test button and calibration
  myFiles.loadBitmap(4, 249, 48, 48, "10test.raw");
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("2", 62, 259);
  myGLCD.setFont(Sinclair_S);
  myGLCD.print("PUMP RATE", 66, 249);
  myGLCD.print("MICROS", 66, 279);

  // buttons to adjust pump time
  myFiles.loadBitmap(145, 245, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(145, 275, 24, 24, arrowButton[1]);

  // read in pump 2 time and display it in seconds
  pumpTime=EEPROM.read(24);  // 24 // pump 2 sec/ml
  pumpTime=pumpTime*10;
  itoa(pumpTime, char5, 10);
  //sprintf(char5, "%f", pumpTime); // convert float to char
  myGLCD.setColor(255, 77, 0);
  myGLCD.setFont(arial_bold);
  px=187;
  // shift x coord to center text if needed
  if (pumpTime>999) px=175;
  myGLCD.print(char5, px, 264);  // 175 if 4 chars

  // units of measurement labels
  myGLCD.setFont(Sinclair_S);
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("mL", 195, 83);
  myGLCD.print("mL", 195, 151);
  myGLCD.print("ms/mL", 187, 221);
  myGLCD.print("ms/mL", 187, 288);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "footextr.raw");

}

void screenPwrSchedule() // this is the screen to show the power schedule
{
  dispScreen=11; 
  myGLCD.clrScr();

  char char3[3];
  char char3t[3];

  // draw header 
  myFiles.loadBitmap(2, 2, 30, 30, "9sched.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(238, 0, 145);
  myGLCD.print("SCHEDULE", 36, 12);

  myGLCD.setColor(222, 8, 51);
  myGLCD.print("POWER", CENTER, 50);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "footschd.raw");

  // grid for schedule
  myGLCD.drawLine(0, 100, 239, 100);
  myGLCD.drawLine(0, 126, 239, 126);
  myGLCD.drawLine(0, 152, 239, 152);
  myGLCD.drawLine(0, 178, 239, 178);
  myGLCD.drawLine(0, 204, 239, 204);
  myGLCD.drawLine(0, 230, 239, 230);
  myGLCD.drawLine(0, 256, 239, 256);

  // write power schedule column headers
  myGLCD.setFont(Sinclair_S);
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("ON", 99, 90);
  myGLCD.print("OFF", 144, 90);
  myGLCD.print("ACTIVE", 183, 90);

  ///////////////////////////////// Lights 1 schedule
  if (schedLights1.active==1) myGLCD.setColor(255, 255, 255);
  else myGLCD.setColor(100, 100, 100);
  myFiles.loadBitmap(198, 104, 18, 18,  schedActive[schedLights1.active]);

  myGLCD.print("LIGHT 1", 8, 109);

  // convert light 1 on hour to char
  itoa(schedLights1.onHour, char3, 10); 
  if (schedLights1.onHour>=0 && schedLights1.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 87, 109);         
  myGLCD.print(":", 103, 109);         

  // convert light 1 on min to char
  itoa(schedLights1.onMinute, char3, 10); 
  if (schedLights1.onMinute>=0 && schedLights1.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 111, 109);

  // convert light 1 off hour to char
  itoa(schedLights1.offHour, char3, 10); 
  if (schedLights1.offHour>=0 && schedLights1.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 136, 109);         
  myGLCD.print(":", 152, 109);         

  // convert light 1 on min to char
  itoa(schedLights1.offMinute, char3, 10); 
  if (schedLights1.offMinute>=0 && schedLights1.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 160, 109);

  ///////////////////////////////// Lights 2 schedule
  if (schedLights2.active==1) myGLCD.setColor(255, 255, 255);
  else myGLCD.setColor(100, 100, 100);
  myFiles.loadBitmap(198, 130, 18, 18,  schedActive[schedLights2.active]);

  myGLCD.print("LIGHT 2", 8, 135);

  // convert on hour to char
  itoa(schedLights2.onHour, char3, 10); 
  if (schedLights2.onHour>=0 && schedLights2.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 87, 135);         
  myGLCD.print(":", 103, 135);         

  // convert on min to char
  itoa(schedLights2.onMinute, char3, 10); 
  if (schedLights2.onMinute>=0 && schedLights2.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 111, 135);

  // convert off hour to char
  itoa(schedLights2.offHour, char3, 10); 
  if (schedLights2.offHour>=0 && schedLights2.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 136, 135);         
  myGLCD.print(":", 152, 135);         

  // convert off min to char
  itoa(schedLights2.offMinute, char3, 10); 
  if (schedLights2.offMinute>=0 && schedLights2.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 160, 135);

  ///////////////////////////////// schedCirc schedule
  if (schedCirc.active==1) myGLCD.setColor(255, 255, 255);
  else myGLCD.setColor(100, 100, 100);
  myFiles.loadBitmap(198, 156, 18, 18, schedActive[schedCirc.active]);

  myGLCD.print("CIRC", 8, 160);

  // convert on hour to char
  itoa(schedCirc.onHour, char3, 10); 
  if (schedCirc.onHour>=0 && schedCirc.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 87, 160);         
  myGLCD.print(":", 103, 160);         

  // convert on min to char
  itoa(schedCirc.onMinute, char3, 10); 
  if (schedCirc.onMinute>=0 && schedCirc.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 111, 160);

  // convert off hour to char
  itoa(schedCirc.offHour, char3, 10); 
  if (schedCirc.offHour>=0 && schedCirc.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 136, 160);         
  myGLCD.print(":", 152, 160);         

  // convert off min to char
  itoa(schedCirc.offMinute, char3, 10); 
  if (schedCirc.offMinute>=0 && schedCirc.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 160, 160);

  ///////////////////////////////// schedCo2 schedule
  if (schedCo2.active==1) myGLCD.setColor(255, 255, 255);
  else myGLCD.setColor(100, 100, 100);
  myFiles.loadBitmap(198, 182, 18, 18, schedActive[schedCo2.active]);
  myGLCD.print("CO2", 8, 186);

  // convert on hour to char
  itoa(schedCo2.onHour, char3, 10); 
  if (schedCo2.onHour>=0 && schedCo2.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 87, 186);         
  myGLCD.print(":", 103, 186);         

  // convert on min to char
  itoa(schedCo2.onMinute, char3, 10); 
  if (schedCo2.onMinute>=0 && schedCo2.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 111, 186);

  // convert off hour to char
  itoa(schedCo2.offHour, char3, 10); 
  if (schedCo2.offHour>=0 && schedCo2.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 136, 186);         
  myGLCD.print(":", 152, 186);         

  // convert off min to char
  itoa(schedCo2.offMinute, char3, 10); 
  if (schedCo2.offMinute>=0 && schedCo2.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 160, 186);

  ///////////////////////////////// schedAux1 schedule
  if (schedAux1.active==1) myGLCD.setColor(255, 255, 255);
  else myGLCD.setColor(100, 100, 100);
  myFiles.loadBitmap(198, 208, 18, 18, schedActive[schedAux1.active]);

  myGLCD.print("AUX 1", 8, 212);

  // convert on hour to char
  itoa(schedAux1.onHour, char3, 10); 
  if (schedAux1.onHour>=0 && schedAux1.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 87, 212);         
  myGLCD.print(":", 103, 212);         

  // convert on min to char
  itoa(schedAux1.onMinute, char3, 10); 
  if (schedAux1.onMinute>=0 && schedAux1.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 111, 212);

  // convert off hour to char
  itoa(schedAux1.offHour, char3, 10); 
  if (schedAux1.offHour>=0 && schedAux1.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 136, 212);         
  myGLCD.print(":", 152, 212);         

  // convert off min to char
  itoa(schedAux1.offMinute, char3, 10); 
  if (schedAux1.offMinute>=0 && schedAux1.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 160, 212);

  ///////////////////////////////// schedAux2 schedule
  if (schedAux2.active==1) myGLCD.setColor(255, 255, 255);
  else myGLCD.setColor(100, 100, 100);
  myFiles.loadBitmap(198, 234, 18, 18, schedActive[schedAux2.active]);

  myGLCD.print("AUX 2", 8, 238);

  // convert on hour to char
  itoa(schedAux2.onHour, char3, 10); 
  if (schedAux2.onHour>=0 && schedAux2.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 87, 238);         
  myGLCD.print(":", 103, 238);         

  // convert on min to char
  itoa(schedAux2.onMinute, char3, 10); 
  if (schedAux2.onMinute>=0 && schedAux2.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 111, 238);

  // convert off hour to char
  itoa(schedAux2.offHour, char3, 10); 
  if (schedAux2.offHour>=0 && schedAux2.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 136, 238);         
  myGLCD.print(":", 152, 238);         

  // convert off min to char
  itoa(schedAux2.offMinute, char3, 10); 
  if (schedAux2.offMinute>=0 && schedAux2.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 160, 238);

}


void screenPwrScheduleItem(int itemNo) // screen for editing the schedule of an individual power relay
{

  myGLCD.clrScr();
  dispScreen=12; 

  char char3[3];
  char char3t[3];

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "9sched.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(238, 0, 145);
  myGLCD.print("SCHEDULE", 36, 12);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "footschd.raw");

  myGLCD.setColor(222, 8, 51);
  myGLCD.print("POWER:", 8, 50);
  myGLCD.setColor(255, 255, 255);

  // print on/off/active and buttons
  myGLCD.print("ON", 8, 113);
  myGLCD.print("OFF", 8, 194);
  myGLCD.print("ACTIVE", 8, 256);

  myGLCD.setFont(Sinclair_S);

  // buttons to edit the hour ON
  myGLCD.print("H", 91, 77);
  myFiles.loadBitmap(83, 94, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(83, 124, 24, 24, arrowButton[1]);

  // buttons to edit the minute ON
  myGLCD.print("M", 172, 77);
  myFiles.loadBitmap(164, 94, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(164, 124, 24, 24, arrowButton[1]);

  // buttons to edit the hour off
  myGLCD.print("H", 91, 158);
  myFiles.loadBitmap(83, 175, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(83, 205, 24, 24, arrowButton[1]);

  // buttons to edit the minute off
  myGLCD.print("M", 172, 158);
  myFiles.loadBitmap(164, 175, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(164, 205, 24, 24, arrowButton[1]);

  int isActive, onHour, onMin, offHour, offMin; // vars for schedule
  scheduleItem=itemNo; // track which item's scheudle we are editing
  myGLCD.setFont(arial_bold);

  // set variables based on the item we are scheduling

  if (itemNo==1) // light 1
  {
    myGLCD.print("LIGHT 1", 108, 50);
    isActive=schedLights1.active;
    onHour=schedLights1.onHour;
    onMin=schedLights1.onMinute;
    offHour=schedLights1.offHour;
    offMin=schedLights1.offMinute;
  }
  else if (itemNo==2) // light 1
  {
    myGLCD.print("LIGHT 2", 108, 50);
    isActive=schedLights2.active;
    onHour=schedLights2.onHour;
    onMin=schedLights2.onMinute;
    offHour=schedLights2.offHour;
    offMin=schedLights2.offMinute;
  }
  else if (itemNo==3) // light 1
  {
    myGLCD.print("CIRC", 108, 50);
    isActive=schedCirc.active;
    onHour=schedCirc.onHour;
    onMin=schedCirc.onMinute;
    offHour=schedCirc.offHour;
    offMin=schedCirc.offMinute;
  }
  else if (itemNo==4) // light 1
  {
    myGLCD.print("CO2", 108, 50);
    isActive=schedCo2.active;
    onHour=schedCo2.onHour;
    onMin=schedCo2.onMinute;
    offHour=schedCo2.offHour;
    offMin=schedCo2.offMinute;
  }
  else if (itemNo==5) // light 1
  {
    myGLCD.print("AUX 1", 108, 50);
    isActive=schedAux1.active;
    onHour=schedAux1.onHour;
    onMin=schedAux1.onMinute;
    offHour=schedAux1.offHour;
    offMin=schedAux1.offMinute;
  }
  else if (itemNo==6) // light 1
  {
    myGLCD.print("AUX 2", 108, 50);
    isActive=schedAux2.active;
    onHour=schedAux2.onHour;
    onMin=schedAux2.onMinute;
    offHour=schedAux2.offHour;
    offMin=schedAux2.offMinute;
  }

  // now draw the schedule to the screen

  myGLCD.setFont(arial_bold);
  myGLCD.setColor(255, 77, 0);

  itoa(onHour, char3, 10);
  if (onHour>=0 && onHour<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 123, 113);

  itoa(onMin, char3, 10);
  if (onMin>=0 && onMin<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 201, 113);

  itoa(offHour, char3, 10);
  if (offHour>=0 && offHour<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 123, 194);

  itoa(offMin, char3, 10);
  if (offMin>=0 && offMin<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 201, 194);

  myFiles.loadBitmap(161, 249, 30, 30, schedActiveB[isActive]);


}


void screenResyncLights() // this screen is displayed quickly as the lights are resynced
{

  myGLCD.clrScr();

  myFiles.loadBitmap(2, 2, 30, 30, "5lights.raw");

  myGLCD.setFont(arial_bold);
  myGLCD.setColor(185, 55, 255);
  myGLCD.print("LIGHTS MODES", 36, 12);

  myGLCD.setColor(255, 255, 255);
  myGLCD.print("RESYNCING", CENTER, 132);
  myGLCD.print("LIGHTS...", CENTER, 168);

  myGLCD.setColor(130, 130, 130);
  myGLCD.drawLine(40, 31, 239, 31); // under header

}


void screenLightRamps() // this is the screen to schedule light ramping
{
  dispScreen=13; 
  myGLCD.clrScr();

  char char3[3];
  char char3t[3];

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "9sched.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(238, 0, 145);
  myGLCD.print("SCHEDULE", 36, 12);

  myGLCD.setColor(185, 55, 255);
  myGLCD.print("LIGHT RAMP", CENTER, 50);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "footschd.raw");

  // grid for schedule
  myGLCD.drawLine(0, 100, 239, 100);
  myGLCD.drawLine(0, 126, 239, 126);
  myGLCD.drawLine(0, 152, 239, 152);
  myGLCD.drawLine(0, 178, 239, 178);
  myGLCD.drawLine(0, 204, 239, 204);
  myGLCD.drawLine(0, 230, 239, 230);
  myGLCD.drawLine(0, 256, 239, 256);

  // schedule headers
  myGLCD.setFont(Sinclair_S);
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("MODE", 24, 90);
  myGLCD.print("START", 100, 90);
  myGLCD.print("LENGTH", 176, 90);

  // ramp mode icons
  myFiles.loadBitmap(9, 104, 66, 19, "13ramp1.raw");
  myFiles.loadBitmap(9, 130, 66, 19, "13ramp2.raw");
  myFiles.loadBitmap(9, 156, 66, 19, "13ramp3.raw");
  myFiles.loadBitmap(9, 182, 66, 19, "13ramp4.raw");
  myFiles.loadBitmap(9, 208, 66, 19, "13ramp5.raw");
  myFiles.loadBitmap(9, 234, 66, 19, "13ramp6.raw");

  /////////////////////////////////// RAMP 1

  // convert on hour to char
  itoa(ramp1.onHour, char3, 10); 
  if (ramp1.onHour>=0 && ramp1.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 100, 109);         
  myGLCD.print(":", 116, 109);         

  // convert on min to char
  itoa(ramp1.onMinute, char3, 10); 
  if (ramp1.onMinute>=0 && ramp1.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 124, 109);

  // convert off hour to char
  itoa(ramp1.offHour, char3, 10); 
  if (ramp1.offHour>=0 && ramp1.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 180, 109);         
  myGLCD.print(":", 196, 109);         

  // convert off min to char
  itoa(ramp1.offMinute, char3, 10); 
  if (ramp1.offMinute>=0 && ramp1.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 204, 109); 

  ///////////////////////////////// RAMP 2 schedule

  // convert on hour to char
  itoa(ramp2.onHour, char3, 10); 
  if (ramp2.onHour>=0 && ramp2.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 100, 135);         
  myGLCD.print(":", 116, 135);         

  // convert on min to char
  itoa(ramp2.onMinute, char3, 10); 
  if (ramp2.onMinute>=0 && ramp2.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 124, 135);

  // convert off hour to char
  itoa(ramp2.offHour, char3, 10); 
  if (ramp2.offHour>=0 && ramp2.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 180, 135);         
  myGLCD.print(":", 196, 135);         

  // convert off min to char
  itoa(ramp2.offMinute, char3, 10); 
  if (ramp2.offMinute>=0 && ramp2.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 204, 135);

  ///////////////////////////////// RAMP 3 schedule

  // convert on hour to char
  itoa(ramp3.onHour, char3, 10); 
  if (ramp3.onHour>=0 && ramp3.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 100, 160);         
  myGLCD.print(":", 116, 160);         

  // convert on min to char
  itoa(ramp3.onMinute, char3, 10); 
  if (ramp3.onMinute>=0 && ramp3.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 124, 160);

  // convert off hour to char
  itoa(ramp3.offHour, char3, 10); 
  if (ramp3.offHour>=0 && ramp3.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 180, 160);         
  myGLCD.print(":", 196, 160);         

  // convert off min to char
  itoa(ramp3.offMinute, char3, 10); 
  if (ramp3.offMinute>=0 && ramp3.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 204, 160);

  ///////////////////////////////// RAMP 4 schedule

  // convert on hour to char
  itoa(ramp4.onHour, char3, 10); 
  if (ramp4.onHour>=0 && ramp4.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 100, 186);         
  myGLCD.print(":", 116, 186);         

  // convert on min to char
  itoa(ramp4.onMinute, char3, 10); 
  if (ramp4.onMinute>=0 && ramp4.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 124, 186);

  // convert off hour to char
  itoa(ramp4.offHour, char3, 10); 
  if (ramp4.offHour>=0 && ramp4.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 180, 186);         
  myGLCD.print(":", 196, 186);         

  // convert off min to char
  itoa(ramp4.offMinute, char3, 10); 
  if (ramp4.offMinute>=0 && ramp4.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 204, 186);

  ///////////////////////////////// RAMP 5 schedule

  // convert on hour to char
  itoa(ramp5.onHour, char3, 10); 
  if (ramp5.onHour>=0 && ramp5.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 100, 212);         
  myGLCD.print(":", 116, 212);         

  // convert on min to char
  itoa(ramp5.onMinute, char3, 10); 
  if (ramp5.onMinute>=0 && ramp5.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 124, 212);

  // convert off hour to char
  itoa(ramp5.offHour, char3, 10); 
  if (ramp5.offHour>=0 && ramp5.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 180, 212);         
  myGLCD.print(":", 196, 212);         

  // convert off min to char
  itoa(ramp5.offMinute, char3, 10); 
  if (ramp5.offMinute>=0 && ramp5.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 204, 212);

  ///////////////////////////////// RAMP 6 schedule

  // convert on hour to char
  itoa(ramp6.onHour, char3, 10); 
  if (ramp6.onHour>=0 && ramp6.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 100, 238);         
  myGLCD.print(":", 116, 238);         

  // convert on min to char
  itoa(ramp6.onMinute, char3, 10); 
  if (ramp6.onMinute>=0 && ramp6.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 124, 238);

  // convert off hour to char
  itoa(ramp6.offHour, char3, 10); 
  if (ramp6.offHour>=0 && ramp6.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 180, 238);         
  myGLCD.print(":", 196, 238);         

  // convert off min to char
  itoa(ramp6.offMinute, char3, 10); 
  if (ramp6.offMinute>=0 && ramp6.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 204, 238);

  //////////////////////////////////// print power information
  myGLCD.setColor(222, 8, 51);
  myGLCD.print("POWER ON", 28, 276);
  myGLCD.print("POWER OFF", 144, 276);
  myGLCD.setColor(255, 255, 255);
  // convert light 1 on hour to char
  itoa(schedLights1.onHour, char3, 10); 
  if (schedLights1.onHour>=0 && schedLights1.onHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 40, 288);         
  myGLCD.print(":", 56, 288);         

  // convert light 1 on min to char
  itoa(schedLights1.onMinute, char3, 10); 
  if (schedLights1.onMinute>=0 && schedLights1.onMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 64, 288);

  // convert light 1 off hour to char
  itoa(schedLights1.offHour, char3, 10); 
  if (schedLights1.offHour>=0 && schedLights1.offHour<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 160, 288);         
  myGLCD.print(":", 176, 288);         

  // convert light 1 off min to char
  itoa(schedLights1.offMinute, char3, 10); 
  if (schedLights1.offMinute>=0 && schedLights1.offMinute<=9) // add a zero if needed
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 184, 288);

}

void screenLightRampItem(byte rampNo)  // screen to edit the ramp schedule for an individual item
{
  dispScreen=14; 
  myGLCD.clrScr();

  char char3[3];
  char char3t[3];

  scheduleItem=rampNo; // keep track of which item we are editing

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "9sched.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(238, 0, 145);
  myGLCD.print("SCHEDULE", 36, 12);

  myGLCD.setColor(185, 55, 255);
  myGLCD.print("RAMP", 36, 34);

  itoa(rampNo, char3, 10); //convert rampNo to char to display
  myGLCD.print(char3, 116, 34);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "footschd.raw");

  myGLCD.drawLine(75, 182, 239, 182); // separator line


  myFiles.loadBitmap(15, 160, 29, 45, "14down.raw"); // down arrow

  myGLCD.setColor(255, 255, 255);

  myGLCD.print("START", 75, 84);
  myGLCD.print("LENGTH", 75, 192);

  myGLCD.setFont(Sinclair_S);

  // start hour up/down
  myGLCD.print("H", 83, 103);
  myFiles.loadBitmap(75, 120, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(75, 150, 24, 24, arrowButton[1]);

  // start min up/down
  myGLCD.print("M", 162, 103);
  myFiles.loadBitmap(154, 120, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(154, 150, 24, 24, arrowButton[1]);

  // length hour up/down
  myGLCD.print("H", 83, 214);
  myFiles.loadBitmap(75, 231, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(75, 261, 24, 24, arrowButton[1]);

  // length min up/down
  myGLCD.print("M", 162, 214);
  myFiles.loadBitmap(154, 231, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(154, 261, 24, 24, arrowButton[1]);

  int onHour, onMin, offHour, offMin; // vars for schedule

  // based on which ramp mode, get schedule and print mode and icons
  if (rampNo==1)
  {
    onHour=ramp1.onHour;
    onMin=ramp1.onMinute;
    offHour=ramp1.offHour; // duration/length... not tehcnically "off"
    offMin=ramp1.offMinute; // duration/length... not tehcnically "off"
    myGLCD.print("MOON TO LOW SUN", 40, 54);
    myFiles.loadBitmap(6, 101, 48, 48, lightMode[3]); // starting mode
    myFiles.loadBitmap(6, 212, 48, 48, lightMode[2]); // ending mode
  }
  else if (rampNo==2)
  {
    onHour=ramp2.onHour;
    onMin=ramp2.onMinute;
    offHour=ramp2.offHour; // duration/length... not tehcnically "off"
    offMin=ramp2.offMinute; // duration/length... not tehcnically "off"
    myGLCD.print("LOW SUN TO MID SUN", 40, 54);
    myFiles.loadBitmap(6, 101, 48, 48, lightMode[2]); // starting mode
    myFiles.loadBitmap(6, 212, 48, 48, lightMode[1]); // ending mode
  }
  else if (rampNo==3)
  {
    onHour=ramp3.onHour;
    onMin=ramp3.onMinute;
    offHour=ramp3.offHour; // duration/length... not tehcnically "off"
    offMin=ramp3.offMinute; // duration/length... not tehcnically "off"
    myGLCD.print("MID SUN TO HIGH SUN", 40, 54);
    myFiles.loadBitmap(6, 101, 48, 48, lightMode[1]); // starting mode
    myFiles.loadBitmap(6, 212, 48, 48, lightMode[0]); // ending mode
  }
  else if (rampNo==4)
  {
    onHour=ramp4.onHour;
    onMin=ramp4.onMinute;
    offHour=ramp4.offHour; // duration/length... not tehcnically "off"
    offMin=ramp4.offMinute; // duration/length... not tehcnically "off"
    myGLCD.print("HIGH SUN TO MID SUN", 40, 54);
    myFiles.loadBitmap(6, 101, 48, 48, lightMode[0]); // starting mode
    myFiles.loadBitmap(6, 212, 48, 48, lightMode[1]); // ending mode
  }
  else if (rampNo==5)
  {
    onHour=ramp5.onHour;
    onMin=ramp5.onMinute;
    offHour=ramp5.offHour; // duration/length... not tehcnically "off"
    offMin=ramp5.offMinute; // duration/length... not tehcnically "off"
    myGLCD.print("MID SUN TO LOW SUN", 40, 54);
    myFiles.loadBitmap(6, 101, 48, 48, lightMode[1]); // starting mode
    myFiles.loadBitmap(6, 212, 48, 48, lightMode[2]); // ending mode
  }
  else if (rampNo==6)
  {
    onHour=ramp6.onHour;
    onMin=ramp6.onMinute;
    offHour=ramp6.offHour; // duration/length... not tehcnically "off"
    offMin=ramp6.offMinute; // duration/length... not tehcnically "off"
    myGLCD.print("LOW SUN TO MOON", 40, 54);
    myFiles.loadBitmap(6, 101, 48, 48, lightMode[2]); // starting mode
    myFiles.loadBitmap(6, 212, 48, 48, lightMode[3]); // ending mode
  }

  // WRITE OUT ALL OF THE STORED TIMES
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(255, 77, 0);

  itoa(onHour, char3, 10);
  if (onHour>=0 && onHour<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 110, 139);

  itoa(onMin, char3, 10);
  if (onMin>=0 && onMin<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 189, 139);

  itoa(offHour, char3, 10);
  if (offHour>=0 && offHour<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 110, 250);

  itoa(offMin, char3, 10);
  if (offMin>=0 && offMin<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 189, 250);

}

void screenDosingSched(byte pumpNo) // screen for scheduling the dosing pumps, screen draws based on which pump
{

  // pump 1 is macro, 2 is micro

    dispScreen=15; 
  myGLCD.clrScr();

  char char3[3];
  char char3t[3];

  scheduleItem=pumpNo; // track which pump we are editing

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "9sched.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(238, 0, 145);
  myGLCD.print("SCHEDULE", 36, 12);

  myGLCD.setColor(138, 93, 35);
  myGLCD.print("DOSING", 36, 34);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(40, 31, 239, 31); // under header
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "footschd.raw");

  // macro and micro buttons to swap schedules
  myFiles.loadBitmap(12, 71, 46, 48, "15macros.raw");
  myFiles.loadBitmap(12, 120, 46, 48, "15micros.raw");

  myGLCD.drawLine(70, 70, 70, 170); // separator line

  myGLCD.setColor(255, 255, 255);

  // vars for scheduling
  byte onHour, onMin, daySun, dayMon, dayTue, dayWed, dayThu, dayFri, daySat;

  // based on which pump, get the schedule 
  if (pumpNo==1)
  {
    myGLCD.setColor(34, 81, 255);  // blue for macros
    myGLCD.print("MACROS", 85, 70);
    onHour=pump1.onHour;
    onMin=pump1.onMinute;
    daySun=pump1.Sunday;
    dayMon=pump1.Monday;
    dayTue=pump1.Tuesday;
    dayWed=pump1.Wednesday;
    dayThu=pump1.Thursday;
    dayFri=pump1.Friday;
    daySat=pump1.Saturday;
  }
  else if (pumpNo==2)
  {
    myGLCD.setColor(255, 77, 0);  // orange for micros
    myGLCD.print("MICROS", 85, 70);
    onHour=pump2.onHour;
    onMin=pump2.onMinute;
    daySun=pump2.Sunday;
    dayMon=pump2.Monday;
    dayTue=pump2.Tuesday;
    dayWed=pump2.Wednesday;
    dayThu=pump2.Thursday;
    dayFri=pump2.Friday;
    daySat=pump2.Saturday;
  }

  // convert schedule to char and print
  itoa(onHour, char3, 10);
  if (onHour>=0 && onHour<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 120, 129);

  itoa(onMin, char3, 10);
  if (onMin>=0 && onMin<=9) // add a zero
  {
    itoa(0, char3t, 10); //make char3t 0
    strcat(char3t, char3);
    strcpy (char3,char3t);
  }
  myGLCD.print(char3, 199, 129);

  // buttons to set time
  myGLCD.setColor(255, 255, 255);
  myGLCD.setFont(Sinclair_S);
  myGLCD.print("H", 93, 93);
  myFiles.loadBitmap(85, 110, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(85, 140, 24, 24, arrowButton[1]);
  myGLCD.print("M", 172, 93);
  myFiles.loadBitmap(164, 110, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(164, 140, 24, 24, arrowButton[1]);

  // days of the week check boxes
  myGLCD.print("SUN", 54, 192);
  myFiles.loadBitmap(51, 203, 30, 30, schedActiveB[daySun]); 
  myGLCD.print("MON", 108, 192);
  myFiles.loadBitmap(105, 203, 30, 30, schedActiveB[dayMon]); 
  myGLCD.print("TUE", 162, 192);
  myFiles.loadBitmap(159, 203, 30, 30, schedActiveB[dayTue]); 
  myGLCD.print("WED", 33, 239);
  myFiles.loadBitmap(30, 250, 30, 30, schedActiveB[dayWed]); 
  myGLCD.print("THU", 87, 239);
  myFiles.loadBitmap(84, 250, 30, 30, schedActiveB[dayThu]); 
  myGLCD.print("FRI", 135, 239);
  myFiles.loadBitmap(132, 250, 30, 30, schedActiveB[dayFri]); 
  myGLCD.print("SAT", 189, 239);
  myFiles.loadBitmap(186, 250, 30, 30, schedActiveB[daySat]); 

}  


void screenScreen() // this is the screen for setting brightness settings
{

  dispScreen=16; 
  char char3[3]; // used to convernt numbers to char
  char char4[4]; // used to convernt numbers to char

  myGLCD.clrScr();

  // draw header
  myFiles.loadBitmap(2, 2, 30, 30, "16screen.raw");
  myGLCD.setFont(arial_bold);
  myGLCD.setColor(255, 77, 0);
  myGLCD.print("SCREEN", 36, 12);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(40, 31, 239, 31); // under header

  // 3 horizontal separater lines
  myGLCD.drawLine(0, 102, 239, 102);
  myGLCD.drawLine(0, 170, 239, 170);
  myGLCD.drawLine(0, 238, 239, 238);

  // draw return home labels
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("RETURN", 4, 49);
  myGLCD.print("TO HOME", 4, 70);
  myGLCD.setFont(Sinclair_S);
  myGLCD.print("AFTER LAST TOUCH", 8, 90);

  // return to home up/down buttons
  myFiles.loadBitmap(145, 42, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(145, 72, 24, 24, arrowButton[1]);

  // draw return home setting
  myGLCD.setFont(arial_bold);
  itoa(screenRetHome, char3, 10);
  myGLCD.setColor(255, 77, 0);
  myGLCD.print(char3, 191, 59);

  // auto dim labels
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("AUTO-DIM", 4, 112);
  myGLCD.print("LEVEL", 4, 133);
  myGLCD.setFont(Sinclair_S);
  myGLCD.print("0 TO 5 (0=OFF)", 6, 154);

  // auto dim up/down
  myFiles.loadBitmap(145, 110, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(145, 140, 24, 24, arrowButton[1]);

  // auto dim setting
  itoa(screenDimLevel, char3, 10);
  myGLCD.setColor(255, 77, 0);
  myGLCD.setFont(arial_bold);
  myGLCD.print(char3, 191, 127);

  // auto dim seconds labels
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("AUTO-DIM", 4, 181);
  myGLCD.print("SECONDS", 4, 202);
  myGLCD.setFont(Sinclair_S);
  myGLCD.print("AFTER LAST TOUCH", 6, 223);

  // auto dim seconds up/down
  myFiles.loadBitmap(145, 178, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(145, 208, 24, 24, arrowButton[1]);

  // auto dim seconds setting
  itoa(screenDimSec, char4, 10);
  myGLCD.setColor(255, 77, 0);
  myGLCD.setFont(arial_bold);
  myGLCD.print(char4, 191, 197);

  // brightness labels
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("BRIGHT", 4, 249);
  myGLCD.setFont(Sinclair_S);
  myGLCD.print("IF NOT USING", 6, 270);
  myGLCD.print("AUTO-DIM", 6, 283);

  // brightness up/down
  myFiles.loadBitmap(145, 245, 24, 24, arrowButton[0]);
  myFiles.loadBitmap(145, 275, 24, 24, arrowButton[1]);

  // brightness setting
  itoa(screenBrightMem, char4, 10);
  myGLCD.setColor(255, 77, 0);
  myGLCD.setFont(arial_bold);
  myGLCD.print(char4, 187, 264);

  // labels under each setting
  myGLCD.setFont(Sinclair_S);
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("MIN", 195, 83);
  myGLCD.print("3 AVG", 187, 151);
  myGLCD.print("SEC", 195, 221);
  myGLCD.print("1-255", 187, 288);

  myGLCD.setColor(130, 130, 130);  
  myGLCD.drawLine(0, 307, 104, 307); // left footer
  myGLCD.drawLine(136, 307, 239, 307); // right footer
  myFiles.loadBitmap(107, 294, 26, 26, "footextr.raw");

}

void processMyTouch() // this is a huge block dedicated to processing all touch screen events
{
  //Serial.print("Touch Detected...\n");
  myTouch.read();
  x=myTouch.getX();
  y=myTouch.getY();

  // initiate char vars for converting numbers to char for display
  char char3[3];
  char char3t[3];
  char char4[4];
  char char5[5];

  // we evaluate touch based on which screen we are in

  switch (dispScreen) 

  {              

  case 1:  // home screen

    if ((x>=50)&&(x<=110)&&(y>=44)&&(y<=95)) // pressed the thermometer to clear a warning
    {
      heaterWarningCleared=true; // clear warning
      screenHome();
    }

    if ((x>=10)&&(x<=58)&&(y>=266)&&(y<=314)) // home dock icon
    {
      screenHome();
    }

    if ((x>=67)&&(x<=115)&&(y>=266)&&(y<=314)) // feeding dock icon
    {
      screenFeeding();
    }

    // coordinates of the power putton
    if ((x>=124)&&(x<=172)&&(y>=266)&&(y<=314)) // power dock icon
    {
      screenPower();
    }

    // coordinates of the exras button
    if ((x>=181)&&(x<=229)&&(y>=266)&&(y<=314)) // settings dock icon
    {
      screenSettings();
    }
    break;

  case 2:    // feeding screen

    if ((x>=67)&&(x<=115)&&(y>=223)&&(y<=271))  // stop button  
    {     
      feedingStop();
    }

    if ((x>=124)&&(x<=172)&&(y>=223)&&(y<=271))  // restart button  
    {     
      feedingActive=false;
      screenFeeding();
    }

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // home button  
    {     
      screenHome();
    }

    break;

  case 3:    // power screen

    if ((x>=69)&&(x<=118)&&(y>=77)&&(y<=125))        // all on  
    {
      // turn on all power outputs
      AlarmPwrLight1_On();
      AlarmPwrLight2_On();
      AlarmPwrFilter_On();
      AlarmPwrCirc_On();
      AlarmPwrHeat_On();
      AlarmPwrCO2_On();
      AlarmPwrAux1_On();
      AlarmPwrAux2_On();
      screenPower(); // redraw screen

    }
    else if ((x>=125)&&(x<=174)&&(y>=77)&&(y<=125))    // all off
    {
      // turn off all power outputs
      AlarmPwrLight1_Off();
      AlarmPwrLight2_Off();
      AlarmPwrFilter_Off();
      AlarmPwrCirc_Off();
      AlarmPwrHeat_Off();
      AlarmPwrCO2_Off();
      AlarmPwrAux1_Off();
      AlarmPwrAux2_Off();
      screenPower(); // redraw screen
    }

    else if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // home button  
    {     
      screenHome();
    }

    else if ((x>=15)&&(x<=64)&&(y>=139)&&(y<=187))    // Front lights power
    {
      //toggle power
      if (globalPower.pwrLight1==0) AlarmPwrLight1_On();
      else if (globalPower.pwrLight1==1) AlarmPwrLight1_Off();
      // draw icons
      myFiles.loadBitmap(15, 139, 48, 48, pwrLightIcon[globalPower.pwrLight1]);
      myFiles.loadBitmap(34, 192, 10, 11, pwrDot[globalPower.pwrLight1]);

    }
    else if ((x>=69)&&(x<=118)&&(y>=139)&&(y<=187))    // Back lights power
    {
      // toggle power
      if (globalPower.pwrLight2==0) AlarmPwrLight2_On();
      else if (globalPower.pwrLight2==1) AlarmPwrLight2_Off();
      // draw icons
      myFiles.loadBitmap(69, 139, 48, 48, pwrLightIcon[globalPower.pwrLight2]);
      myFiles.loadBitmap(88, 192, 10, 11, pwrDot[globalPower.pwrLight2]);
    }
    else if ((x>=124)&&(x<=173)&&(y>=139)&&(y<=187))    // Filter power
    {
      // toggle power
      if (globalPower.pwrFilter==0) AlarmPwrFilter_On();
      else if (globalPower.pwrFilter==1)AlarmPwrFilter_Off();
      // draw icons
      myFiles.loadBitmap(124, 139, 48, 48, pwrFilterIcon[globalPower.pwrFilter]);
      myFiles.loadBitmap(143, 192, 10, 11, pwrDot[globalPower.pwrFilter]);
    }
    else if ((x>=178)&&(x<=227)&&(y>=139)&&(y<=187))    // Circ power
    {
      // toggle power
      if (globalPower.pwrCirc==0) AlarmPwrCirc_On();
      else if (globalPower.pwrCirc==1) AlarmPwrCirc_Off();
      // draw icons
      myFiles.loadBitmap(178, 139, 48, 48, pwrCircIcon[globalPower.pwrCirc] );
      myFiles.loadBitmap(197, 192, 10, 11, pwrDot[globalPower.pwrCirc]);
    }

    else if ((x>=15)&&(x<=64)&&(y>=198)&&(y<=246))    // Heat power
    {
      // toggle power
      if (globalPower.pwrHeat==0) AlarmPwrHeat_On();
      else if (globalPower.pwrHeat==1)AlarmPwrHeat_Off();
      // draw icons
      myFiles.loadBitmap(15, 212, 48, 48, pwrHeatIcon[globalPower.pwrHeat] );
      myFiles.loadBitmap(34, 264, 10, 11, pwrDot[globalPower.pwrHeat]);
    }
    else if ((x>=69)&&(x<=118)&&(y>=198)&&(y<=246))    // CO2 power
    {
      // toggle power
      if (globalPower.pwrCO2==0) AlarmPwrCO2_On();
      else if (globalPower.pwrCO2==1) AlarmPwrCO2_Off();
      // draw icons
      myFiles.loadBitmap(69, 212, 48, 48, pwrCO2Icon[globalPower.pwrCO2] );
      myFiles.loadBitmap(88, 264, 10, 11, pwrDot[globalPower.pwrCO2]);
    }
    else if ((x>=124)&&(x<=173)&&(y>=198)&&(y<=246))    // aux 1 power
    {
      // toggle power
      if (globalPower.pwrAux1==0) AlarmPwrAux1_On();
      else if (globalPower.pwrAux1==1) AlarmPwrAux1_Off();
      // draw icons
      myFiles.loadBitmap(124, 212, 48, 48, pwrAux1Icon[globalPower.pwrAux1] );
      myFiles.loadBitmap(143, 264, 10, 11, pwrDot[globalPower.pwrAux1]);
    }
    else if ((x>=178)&&(x<=227)&&(y>=198)&&(y<=246))    // aux 2 power
    {
      // toggle power
      if (globalPower.pwrAux2==0) AlarmPwrAux2_On();
      else if (globalPower.pwrAux2==1) AlarmPwrAux2_Off();
      // draw icons  
      myFiles.loadBitmap(178, 212, 48, 48, pwrAux2Icon[globalPower.pwrAux2] );
      myFiles.loadBitmap(197, 264, 10, 11, pwrDot[globalPower.pwrAux2]);
    }
    break;

  case 4:    // settings screen

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // home button  
    {     
      screenHome();
    }

    if ((x>=10)&&(x<=58)&&(y>=50)&&(y<=113))
    {
      if (globalPower.pwrLight1==1) screenLights(); // only respond to the lights button if they are turned on
    }
    else if ((x>=67)&&(x<=115)&&(y>=50)&&(y<=113))
    {
      screenClock();
    }
    else if ((x>=124)&&(x<=172)&&(y>=50)&&(y<=113))
    {
      screenFeedSettings();
    }
    else if ((x>=181)&&(x<=229)&&(y>=50)&&(y<=113))
    {
      screenSchedule();
    }
    else if ((x>=10)&&(x<=58)&&(y>=118)&&(y<=181))
    {
      screenHeater();
    }
    else if ((x>=67)&&(x<=115)&&(y>=118)&&(y<=181))
    {
      screenDosing();
    }
    else if ((x>=124)&&(x<=172)&&(y>=118)&&(y<=181))
    {
      screenScreen();
    }

    break;

  case 5:    // lights screen

    if (lightEditing==0) // actions for buttons that are active when editing is not active
    {
      boolean updateLightValues=false; // track if we need to update the light values at the end due to mode change

        if ((x>=10)&&(x<=58)&&(y>=39)&&(y<=87))        //  full sun mode button
      {
        clearSelectedLight(1); // deselect the previously selected light
        currentLightMode=0;
        if (lightCSP==true) irsend.sendNEC(M1,32); // send IR to lights to change modes
        else if (lightCSP==false) // change PWM levels
        {
          analogWrite(ledRedPin, lightHighSun.Red);
          analogWrite(ledGreenPin, lightHighSun.Green);
          analogWrite(ledBluePin, lightHighSun.Blue);
          analogWrite(ledWhitePin, lightHighSun.White);
        }
        currentColor = lightHighSun; // update current color values
        myFiles.loadBitmap(10, 101, 48, 63, lightEdit[1]); // activate the edit button
        myFiles.loadBitmap(10, 39, 48, 48, lightModeS[currentLightMode]); // draw the button selected
        updateLightValues=true; // update the light values

      }
      else if ((x>=67)&&(x<=115)&&(y>=39)&&(y<=87))    //  mid sun mode button
      {
        clearSelectedLight(1); // deselect the previously selected light
        currentLightMode=1;
        if (lightCSP==true) irsend.sendNEC(M2,32); // send IR to lights to change modes
        else if (lightCSP==false) // change PWM levels
        {
          analogWrite(ledRedPin, lightMidSun.Red);
          analogWrite(ledGreenPin, lightMidSun.Green);
          analogWrite(ledBluePin, lightMidSun.Blue);
          analogWrite(ledWhitePin, lightMidSun.White);
        }
        currentColor = lightMidSun;
        myFiles.loadBitmap(10, 101, 48, 63, lightEdit[1]); // activate the edit button
        myFiles.loadBitmap(67, 39, 48, 48, lightModeS[currentLightMode]); // draw the button selected
        updateLightValues=true; // update the light values
      }
      else if ((x>=124)&&(x<=172)&&(y>=39)&&(y<=87))      //  low sun mode button
      {
        clearSelectedLight(1); // deselect the previously selected light
        currentLightMode=2;
        if (lightCSP==true) irsend.sendNEC(M3,32); // send IR to lights to change modes
        else if (lightCSP==false) // change PWM levels
        {
          analogWrite(ledRedPin, lightLowSun.Red);
          analogWrite(ledGreenPin, lightLowSun.Green);
          analogWrite(ledBluePin, lightLowSun.Blue);
          analogWrite(ledWhitePin, lightLowSun.White);
        }
        currentColor = lightLowSun;
        myFiles.loadBitmap(10, 101, 48, 63, lightEdit[1]); // activate the edit button
        myFiles.loadBitmap(124, 39, 48, 48, lightModeS[currentLightMode]); // draw the button selected
        updateLightValues=true; // update the light values
      }
      else if ((x>=181)&&(x<=229)&&(y>=39)&&(y<=87))     //  moon mode button
      {
        clearSelectedLight(1); // deselect the previously selected light
        currentLightMode=3;
        if (lightCSP==true) irsend.sendNEC(M4,32); // send IR to lights to change modes
        else if (lightCSP==false) // change PWM levels
        {
          analogWrite(ledRedPin, lightMoon.Red);
          analogWrite(ledGreenPin, lightMoon.Green);
          analogWrite(ledBluePin, lightMoon.Blue);
          analogWrite(ledWhitePin, lightMoon.White);
        }
        currentColor = lightMoon;
        myFiles.loadBitmap(10, 101, 48, 63, lightEdit[1]); // activate the edit button
        myFiles.loadBitmap(181, 39, 48, 48, lightModeS[currentLightMode]); // draw the button selected
        updateLightValues=true; // update the light values
      }
      else if ((x>=10)&&(x<=58)&&(y>=101)&&(y<=149))        //  edit button
      {
        clearSelectedLight(2); // turn off light modes that are inactive
        lightEditing=1; // activate editing mode
        myFiles.loadBitmap(181, 101, 48, 63, lightResync[0]); // disable resync
        myFiles.loadBitmap(10, 101, 48, 63, lightEdit[0]); // disable edit
        myFiles.loadBitmap(67, 101, 48, 63, lightSave[1]); // enable save
        myFiles.loadBitmap(124, 101, 48, 63, lightCancel[1]); // enable cancel

        // enable RGBW buttons
        myFiles.loadBitmap(10, 175, 48, 48, lightWhite[0]);
        myFiles.loadBitmap(67, 175, 48, 48, lightRed[0]);
        myFiles.loadBitmap(124, 175, 48, 48, lightGreen[0]);
        myFiles.loadBitmap(181, 175, 48, 48, lightBlue[0]);
        myFiles.loadBitmap(10, 241, 48, 48, lightWhite[1]);
        myFiles.loadBitmap(67, 241, 48, 48, lightRed[1]);
        myFiles.loadBitmap(124, 241, 48, 48, lightGreen[1]);
        myFiles.loadBitmap(181, 241, 48, 48, lightBlue[1]);

      }
      else if ((x>=181)&&(x<=229)&&(y>=101)&&(y<=149))    //  resync button
      {
        screenResyncLights(); // display that we are resyncing
        resyncLights(); // actually resync the lights
        screenLights(); // redraw the lights screen
      }

      // if modes were switched, we need to update the lighting values by the RGBW up/down buttons
      if (updateLightValues==1)
      {

        myGLCD.setFont(Sinclair_S);
        myGLCD.setColor(255, 255, 255);

        itoa(currentColor.White, char3, 10);
        if (currentColor.White>=0 && currentColor.White<=9) // add a zero
        {
          itoa(0, char3t, 10); //make char3t 0
          strcat(char3t, char3);
          strcpy (char3,char3t);
        }
        myGLCD.print(char3, 26, 228);

        itoa(currentColor.Red, char3, 10);
        if (currentColor.Red>=0 && currentColor.Red<=9) // add a zero
        {
          itoa(0, char3t, 10); //make char3t 0
          strcat(char3t, char3);
          strcpy (char3,char3t);
        }
        myGLCD.print(char3, 83, 228);

        itoa(currentColor.Green, char3, 10);
        if (currentColor.Green>=0 && currentColor.Green<=9) // add a zero
        {
          itoa(0, char3t, 10); //make char3t 0
          strcat(char3t, char3);
          strcpy (char3,char3t);
        }
        myGLCD.print(char3, 140, 228);

        itoa(currentColor.Blue, char3, 10);
        if (currentColor.Blue>=0 && currentColor.Blue<=9) // add a zero
        {
          itoa(0, char3t, 10); //make char3t 0
          strcat(char3t, char3);
          strcpy (char3,char3t);
        }
        myGLCD.print(char3, 197, 228);
      }

    }

    else if (lightEditing==1) // actions for buttons that are active when editing is active
    {
      if ((x>=67)&&(x<=115)&&(y>=101)&&(y<=149))    //  save button
      {

        screenResyncLights(); // show that we are working

        // setup eeprom rgbw variables
        byte eR, eG, eB, eW;


        // save current light mode
        if (currentLightMode==0)
        {
          eR=200; // eeprom location to save M1 red
          eG=201; // eeprom location to save M1 green
          eB=202; // eeprom location to save M1 blue
          eW=203; // eeprom location to save M1 white
          if (lightCSP==true) irsend.sendNEC(M1,32); // save values to position 1 on lights
          if (lightCSP==true) irsend.sendNEC(0xFFFFFFFF,32); // start sending footer to save
        }
        else if (currentLightMode==1)
        {
          eR=204; // eeprom location to save M2 red
          eG=205; // eeprom location to save M2 green
          eB=206; // eeprom location to save M2 blue
          eW=207; // eeprom location to save M2 white
          if (lightCSP==true) irsend.sendNEC(M2,32); // save values to position 2 on lights
          if (lightCSP==true) irsend.sendNEC(0xFFFFFFFF,32); //  start sending footer to save
        }
        else if (currentLightMode==2)
        {
          eR=208; // eeprom location to save M3 red
          eG=209; // eeprom location to save M3 green
          eB=210; // eeprom location to save M3 blue
          eW=211; // eeprom location to save M3 white
          if (lightCSP==true) irsend.sendNEC(M3,32); // save values to position 3 on lights
          if (lightCSP==true) irsend.sendNEC(0xFFFFFFFF,32); //  start sending footer to save
        }
        else if (currentLightMode==3)
        {
          eR=212; // eeprom location to save M4 red
          eG=213; // eeprom location to save M4 green
          eB=214; // eeprom location to save M4 blue
          eW=215; // eeprom location to save M4 white
          if (lightCSP==true) irsend.sendNEC(M4,32); // save values to position 4 on lights
          if (lightCSP==true) irsend.sendNEC(0xFFFFFFFF,32); //  start sending footer to save
        }

        if (lightCSP==true) 
        {
          // blast the footer for 3 seconds to save the memory position in the light
          unsigned long currentMillis = millis(); // get current millis
          unsigned long blastMillis = millis(); // get current millis
          while ((blastMillis-currentMillis)<3000)
          {
            irsend.sendNEC(0xFFFFFFFF,32); // blast footer to save
            blastMillis = millis(); // get current millis
          }
        }

        // save new values to eeprom locations
        EEPROM.write(eR,currentColor.Red);
        EEPROM.write(eG,currentColor.Green);
        EEPROM.write(eB,currentColor.Blue);
        EEPROM.write(eW,currentColor.White);

        lightEditing=0; // disable editing
        int nextMode=currentLightMode; // keep track of what mode we are in
        resyncLights(); // we resync the lights after saving
        currentLightMode=nextMode; // return to the saved mode 

          // switch back to the lighting mode you saved
        if (lightCSP==true) 
        {
          if (currentLightMode==0) irsend.sendNEC(M1,32); // activate the mode you just saved
          else if (currentLightMode==1) irsend.sendNEC(M2,32); // activate the mode you just saved
          else if (currentLightMode==2) irsend.sendNEC(M3,32); // activate the mode you just saved
          else if (currentLightMode==3) irsend.sendNEC(M4,32); // activate the mode you just saved
          //irsend.sendNEC(0xFFFFFFFF,32); // send footer
        }
        else if (lightCSP==false) //update PWM light
        {
          analogWrite(ledRedPin, currentColor.Red);
          analogWrite(ledGreenPin, currentColor.Green);
          analogWrite(ledBluePin, currentColor.Blue);
          analogWrite(ledWhitePin, currentColor.White);
        }
        screenLights(); // redraw the screen
      }

      else if ((x>=124)&&(x<=172)&&(y>=101)&&(y<=149))      //  cancel button
      {
        lightEditing=0;

        // switch back to current light mode
        if (currentLightMode==0) 
        {
          currentColor=lightHighSun;
          if (lightCSP==true) irsend.sendNEC(M1,32);
        }
        else if (currentLightMode==1) 
        {
          currentColor=lightMidSun;
          if (lightCSP==true) irsend.sendNEC(M2,32);
        }
        else if (currentLightMode==2) 
        {
          currentColor=lightLowSun;
          if (lightCSP==true) irsend.sendNEC(M3,32);
        }
        else if (currentLightMode==3) 
        {
          currentColor=lightMoon;
          if (lightCSP==true) irsend.sendNEC(M4,32);
        }
        //if (lightCSP==true) //irsend.sendNEC(0xFFFFFFFF,32); // send footer

        if (lightCSP==false) //update PWM light
        {
          analogWrite(ledRedPin, currentColor.Red);
          analogWrite(ledGreenPin, currentColor.Green);
          analogWrite(ledBluePin, currentColor.Blue);
          analogWrite(ledWhitePin, currentColor.White);
        }

        screenLights(); // redraw screen
      }

      else if ((x>=10)&&(x<=58)&&(y>=175)&&(y<=223))   // white up button
      {
        if (currentColor.White<maxRGBW) 
        {
          currentColor.White=(currentColor.White+1); // increment stored light value

          // update value on screen
          myGLCD.setFont(Sinclair_S);
          myGLCD.setColor(255, 255, 255);
          itoa(currentColor.White, char3, 10);
          if (currentColor.White>=0 && currentColor.White<=9) // add a zero
          {
            itoa(0, char3t, 10); //make char3t 0
            strcat(char3t, char3);
            strcpy (char3,char3t);
          }
          myGLCD.print(char3, 26, 228);

          if (lightCSP==true)
          {
            irsend.sendNEC(WHITEUP,32);  // send value to light
            //irsend.sendNEC(0xFFFFFFFF,32); // send footer
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledWhitePin, currentColor.White); //update PWM light
        }
      }
      else if ((x>=10)&&(x<=58)&&(y>=241)&&(y<=289))   // white down
      {
        if (currentColor.White>=1) 
        {
          currentColor.White=(currentColor.White-1); // increment stored light value
          itoa(currentColor.White, char3, 10);
          if (currentColor.White>=0 && currentColor.White<=9) // add a zero
          {
            itoa(0, char3t, 10); //make char3t 0
            strcat(char3t, char3);
            strcpy (char3,char3t);
          }
          myGLCD.print(char3, 26, 228);
          if (lightCSP==true)
          {
            irsend.sendNEC(WHITEDOWN,32);  // send value to light
            //irsend.sendNEC(0xFFFFFFFF,32); // send footer
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledWhitePin, currentColor.White); //update PWM light
        }
      }
      else if ((x>=67)&&(x<=115)&&(y>=175)&&(y<=223))    //  red up
      {
        if (currentColor.Red<maxRGBW) 
        {
          currentColor.Red=(currentColor.Red+1); // increment stored light value
          itoa(currentColor.Red, char3, 10);
          if (currentColor.Red>=0 && currentColor.Red<=9) // add a zero
          {
            itoa(0, char3t, 10); //make char3t 0
            strcat(char3t, char3);
            strcpy (char3,char3t);
          }
          myGLCD.print(char3, 83, 228);
          if (lightCSP==true)
          {
            irsend.sendNEC(REDUP,32);  // send value to light
            //irsend.sendNEC(0xFFFFFFFF,32); // send footer
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledRedPin, currentColor.Red); //update PWM light
        }
      }
      else if ((x>=67)&&(x<=115)&&(y>=241)&&(y<=289))    //  red down
      {
        if (currentColor.Red>=1) 
        {
          currentColor.Red=(currentColor.Red-1); // increment stored light value
          itoa(currentColor.Red, char3, 10);
          if (currentColor.Red>=0 && currentColor.Red<=9) // add a zero
          {
            itoa(0, char3t, 10); //make char3t 0
            strcat(char3t, char3);
            strcpy (char3,char3t);
          }
          myGLCD.print(char3, 83, 228);                
          if (lightCSP==true)
          {
            irsend.sendNEC(REDDOWN,32);  // send value to light
            //irsend.sendNEC(0xFFFFFFFF,32); // send footer
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledRedPin, currentColor.Red); //update PWM light
        }
      }
      else if ((x>=124)&&(x<=172)&&(y>=175)&&(y<=223))      //  green up
      {
        if (currentColor.Green<maxRGBW) 
        {
          currentColor.Green=(currentColor.Green+1); // increment stored light value
          itoa(currentColor.Green, char3, 10);
          if (currentColor.Green>=0 && currentColor.Green<=9) // add a zero
          {
            itoa(0, char3t, 10); //make char3t 0
            strcat(char3t, char3);
            strcpy (char3,char3t);
          }
          myGLCD.print(char3, 140, 228);
          if (lightCSP==true)
          {
            irsend.sendNEC(GREENUP,32);  // send value to light
            //irsend.sendNEC(0xFFFFFFFF,32); // send footer
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledGreenPin, currentColor.Green); //update PWM light
        }
      }
      else if ((x>=124)&&(x<=172)&&(y>=241)&&(y<=289))      //  green down
      {
        if (currentColor.Green>=1) 
        {
          currentColor.Green=(currentColor.Green-1); // increment stored light value
          itoa(currentColor.Green, char3, 10);
          if (currentColor.Green>=0 && currentColor.Green<=9) // add a zero
          {
            itoa(0, char3t, 10); //make char3t 0
            strcat(char3t, char3);
            strcpy (char3,char3t);
          }
          myGLCD.print(char3, 140, 228);                
          if (lightCSP==true)
          {
            irsend.sendNEC(GREENDOWN,32);  // send value to light
            //irsend.sendNEC(0xFFFFFFFF,32); // send footer
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledGreenPin, currentColor.Green); //update PWM light
        }
      }
      else if ((x>=181)&&(x<=229)&&(y>=175)&&(y<=223))     //  blue up
      {
        if (currentColor.Blue<maxRGBW) 
        {
          currentColor.Blue=(currentColor.Blue+1); // increment stored light value
          itoa(currentColor.Blue, char3, 10);
          if (currentColor.Blue>=0 && currentColor.Blue<=9) // add a zero
          {
            itoa(0, char3t, 10); //make char3t 0
            strcat(char3t, char3);
            strcpy (char3,char3t);
          }
          myGLCD.print(char3, 197, 228);
          if (lightCSP==true)
          {
            irsend.sendNEC(BLUEUP,32);  // send value to light
            //irsend.sendNEC(0xFFFFFFFF,32); // send footer
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledBluePin, currentColor.Blue); //update PWM light
        }
      }
      else if ((x>=181)&&(x<=229)&&(y>=241)&&(y<=289))     //  blue down
      {
        if (currentColor.Blue>=1) 
        {
          currentColor.Blue=(currentColor.Blue-1); // increment stored light value
          itoa(currentColor.Blue, char3, 10);
          if (currentColor.Blue>=0 && currentColor.Blue<=9) // add a zero
          {
            itoa(0, char3t, 10); //make char3t 0
            strcat(char3t, char3);
            strcpy (char3,char3t);
          }
          myGLCD.print(char3, 197, 228);
          if (lightCSP==true)
          {
            irsend.sendNEC(BLUEDOWN,32);  // send value to light
            //irsend.sendNEC(0xFFFFFFFF,32); // send footer
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledBluePin, currentColor.Blue); //update PWM light
        }
      }

    }

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // settings button 
    {     
      screenSettings();
    }

    break;

  case 6:    // clock screen for setting time

    itoa(0, char3t, 10); //make char3t 0

      myGLCD.setFont(arial_bold);
    myGLCD.setColor(255, 77, 0);

    if ((x>=12)&&(x<=96)&&(y>=286)&&(y<=312))  // cancel button  
    {     
      screenSettings();
    }
    else if ((x>=144)&&(x<=228)&&(y>=286)&&(y<=312))  // save button  
    {     
      SaveTime();
      screenSettings();
    }
    else if ((x>=12)&&(x<=36)&&(y>=89)&&(y<=113))  // hour up button
    {     
      saveRTC.tHour=(saveRTC.tHour+1);
      if (saveRTC.tHour>23) saveRTC.tHour=0;
      itoa(saveRTC.tHour, char3, 10);
      if (saveRTC.tHour>=0 && saveRTC.tHour<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 45, 108);
    }
    else if ((x>=12)&&(x<=36)&&(y>=119)&&(y<=143))  // hour down button  
    {     
      saveRTC.tHour=(saveRTC.tHour-1);
      if (saveRTC.tHour<0) saveRTC.tHour=23;
      itoa(saveRTC.tHour, char3, 10);
      if (saveRTC.tHour>=0 && saveRTC.tHour<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 45, 108);
    }
    else if ((x>=83)&&(x<=107)&&(y>=89)&&(y<=113))  // min up button  
    {     
      saveRTC.tMinute=(saveRTC.tMinute+1);
      if (saveRTC.tMinute>59) saveRTC.tMinute=0;
      itoa(saveRTC.tMinute, char3, 10);
      if (saveRTC.tMinute>=0 && saveRTC.tMinute<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 123, 108);

    }
    else if ((x>=83)&&(x<=107)&&(y>=119)&&(y<=143))  // min down button  
    {     
      saveRTC.tMinute=(saveRTC.tMinute-1);
      if (saveRTC.tMinute<0) saveRTC.tMinute=59;
      itoa(saveRTC.tMinute, char3, 10);
      if (saveRTC.tMinute>=0 && saveRTC.tMinute<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 123, 108);
    }
    else if ((x>=164)&&(x<=188)&&(y>=89)&&(y<=113))  // sec up button  
    {     
      saveRTC.tSecond=(saveRTC.tSecond+1);
      if (saveRTC.tSecond>59) saveRTC.tSecond=0;
      itoa(saveRTC.tSecond, char3, 10);
      if (saveRTC.tSecond>=0 && saveRTC.tSecond<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 201, 108);

    }
    else if ((x>=164)&&(x<=188)&&(y>=119)&&(y<=143))  // sec down button  
    {     
      saveRTC.tSecond=(saveRTC.tSecond-1);
      if (saveRTC.tSecond<0) saveRTC.tSecond=59;
      itoa(saveRTC.tSecond, char3, 10);
      if (saveRTC.tSecond>=0 && saveRTC.tSecond<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 201, 108);
    }

    else if ((x>=12)&&(x<=36)&&(y>=211)&&(y<=235))  // month up button  
    {     
      saveRTC.tMonth=(saveRTC.tMonth+1);
      if (saveRTC.tMonth>12) saveRTC.tMonth=1;
      itoa(saveRTC.tMonth, char3, 10);
      if (saveRTC.tMonth>=0 && saveRTC.tMonth<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 45, 230);

    }
    else if ((x>=12)&&(x<=36)&&(y>=241)&&(y<=265))  // month down button  
    {     
      saveRTC.tMonth=(saveRTC.tMonth-1);
      if (saveRTC.tMonth<1) saveRTC.tMonth=12;
      itoa(saveRTC.tMonth, char3, 10);
      if (saveRTC.tMonth>=0 && saveRTC.tMonth<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 45, 230);
    }
    else if ((x>=83)&&(x<=107)&&(y>=211)&&(y<=235))  // day up button  
    {     
      saveRTC.tDay=(saveRTC.tDay+1);
      if (saveRTC.tDay>31) saveRTC.tDay=1;
      itoa(saveRTC.tDay, char3, 10);
      if (saveRTC.tDay>=0 && saveRTC.tDay<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 123, 230);

    }
    else if ((x>=83)&&(x<=107)&&(y>=241)&&(y<=265))  // day down button  
    {     
      saveRTC.tDay=(saveRTC.tDay-1);
      if (saveRTC.tDay<1) saveRTC.tDay=31;
      itoa(saveRTC.tDay, char3, 10);
      if (saveRTC.tDay>=0 && saveRTC.tDay<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 123, 230);
    }
    else if ((x>=164)&&(x<=188)&&(y>=211)&&(y<=235))  // year up button  
    {     
      saveRTC.tYear=(saveRTC.tYear+1);
      if (saveRTC.tYear>99) saveRTC.tYear=0;
      itoa(saveRTC.tYear, char3, 10);
      if (saveRTC.tYear>=0 && saveRTC.tYear<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 201, 230);

    }
    else if ((x>=164)&&(x<=188)&&(y>=241)&&(y<=265))  // year down button  
    {     
      saveRTC.tYear=(saveRTC.tYear-1);
      if (saveRTC.tYear<0) saveRTC.tYear=99;
      itoa(saveRTC.tYear, char3, 10);
      if (saveRTC.tYear>=0 && saveRTC.tYear<=9) // add a zero
      {
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 201, 230);
    }

    break;

  case 7:    // feeding schedule screen

    if ((x>=158)&&(x<=182)&&(y>=46)&&(y<=70)) // minutes up  button 
    {
      feedingMins=feedingMins+1; 
      itoa(feedingMins, char3, 10);
      if (feedingMins>=0 && feedingMins<=9) // add a zero
      {
        char char3t[3];
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.setFont(arial_bold);
      myGLCD.setColor(255, 77, 0);
      myGLCD.print(char3, 190, 65);
    }
    else if ((x>=158)&&(x<=182)&&(y>=76)&&(y<=100))  // minutes down
    {
      if (feedingMins>1) // don't increment lower than 1 minute
      {
        feedingMins=feedingMins-1;   
        itoa(feedingMins, char3, 10);
        if (feedingMins>=0 && feedingMins<=9) // add a zero
        {
          itoa(0, char3t, 10); //make char3t 0
          strcat(char3t, char3);
          strcpy (char3,char3t);
        }
        myGLCD.setFont(arial_bold);
        myGLCD.setColor(255, 77, 0);
        myGLCD.print(char3, 190, 65);
      }
    }
    else if ((x>=15)&&(x<=63)&&(y>=151)&&(y<=199))  // pwrLight1Icon  
    {
      // toggle to opposite power setting
      if (feedPower.pwrLight1==0) feedPower.pwrLight1=1;
      else if (feedPower.pwrLight1==1) feedPower.pwrLight1=0;
      // save to memory
      EEPROM.write(8, feedPower.pwrLight1);
      // update icons
      myFiles.loadBitmap(15, 151, 48, 48, pwrLightIcon[feedPower.pwrLight1]);
      myFiles.loadBitmap(34, 204, 10, 11, pwrDot[feedPower.pwrLight1]);
    }
    else if ((x>=69)&&(x<=117)&&(y>=151)&&(y<=199))  // pwrLight2Icon  
    {
      // toggle to opposite power setting
      if (feedPower.pwrLight2==0) feedPower.pwrLight2=1;
      else if (feedPower.pwrLight2==1) feedPower.pwrLight2=0;
      // save to memory
      EEPROM.write(9, feedPower.pwrLight2);
      // update icons
      myFiles.loadBitmap(69, 151, 48, 48, pwrLightIcon[feedPower.pwrLight2]);
      myFiles.loadBitmap(88, 204, 10, 11, pwrDot[feedPower.pwrLight2]);
    }
    else if ((x>=124)&&(x<=172)&&(y>=151)&&(y<=199))  // pwrFilterIcon  
    {
      // toggle to opposite power setting
      if (feedPower.pwrFilter==0) feedPower.pwrFilter=1;
      else if (feedPower.pwrFilter==1) feedPower.pwrFilter=0;
      // save to memory
      EEPROM.write(10, feedPower.pwrFilter);
      // update icons
      myFiles.loadBitmap(124, 151, 48, 48, pwrFilterIcon[feedPower.pwrFilter]);
      myFiles.loadBitmap(143, 204, 10, 11, pwrDot[feedPower.pwrFilter]);
    }
    else if ((x>=178)&&(x<=226)&&(y>=151)&&(y<=199))  // pwrCircIcon  
    {
      // toggle to opposite power setting
      if (feedPower.pwrCirc==0) feedPower.pwrCirc=1;
      else if (feedPower.pwrCirc==1) feedPower.pwrCirc=0;
      // save to memory
      EEPROM.write(11, feedPower.pwrCirc);
      // update icons
      myFiles.loadBitmap(178, 151, 48, 48, pwrCircIcon[feedPower.pwrCirc] );
      myFiles.loadBitmap(197, 204, 10, 11, pwrDot[feedPower.pwrCirc]);
    }
    else if ((x>=15)&&(x<=63)&&(y>=224)&&(y<=272))  // pwrHeatIcon  
    {
      // toggle to opposite power setting
      if (feedPower.pwrHeat==0) feedPower.pwrHeat=1;
      else if (feedPower.pwrHeat==1) feedPower.pwrHeat=0;
      // save to memory
      EEPROM.write(12, feedPower.pwrHeat);
      // update icons
      myFiles.loadBitmap(15, 224, 48, 48, pwrHeatIcon[feedPower.pwrHeat] );
      myFiles.loadBitmap(34, 276, 10, 11, pwrDot[feedPower.pwrHeat]);
    }
    else if ((x>=69)&&(x<=117)&&(y>=224)&&(y<=272))  // pwrCO2Icon  
    {
      // toggle to opposite power setting
      if (feedPower.pwrCO2==0) feedPower.pwrCO2=1;
      else if (feedPower.pwrCO2==1) feedPower.pwrCO2=0;
      // save to memory
      EEPROM.write(13, feedPower.pwrCO2);
      // update icons
      myFiles.loadBitmap(69, 224, 48, 48, pwrCO2Icon[feedPower.pwrCO2] );
      myFiles.loadBitmap(88, 276, 10, 11, pwrDot[feedPower.pwrCO2]);
    }
    else if ((x>=124)&&(x<=172)&&(y>=224)&&(y<=272))  // pwrAux1Icon  
    {
      // toggle to opposite power setting
      if (feedPower.pwrAux1==0) feedPower.pwrAux1=1;
      else if (feedPower.pwrAux1==1) feedPower.pwrAux1=0;
      // save to memory
      EEPROM.write(14, feedPower.pwrAux1);
      // update icons
      myFiles.loadBitmap(124, 224, 48, 48, pwrAux1Icon[feedPower.pwrAux1] );
      myFiles.loadBitmap(143, 276, 10, 11, pwrDot[feedPower.pwrAux1]);
    }
    else if ((x>=178)&&(x<=226)&&(y>=224)&&(y<=272))  // pwrAux2Icon  
    {
      // toggle to opposite power setting
      if (feedPower.pwrAux2==0) feedPower.pwrAux2=1;
      else if (feedPower.pwrAux2==1) feedPower.pwrAux2=0;
      // save to memory
      EEPROM.write(15, feedPower.pwrAux2);
      // update icons
      myFiles.loadBitmap(178, 224, 48, 48, pwrAux2Icon[feedPower.pwrAux2] );
      myFiles.loadBitmap(197, 276, 10, 11, pwrDot[feedPower.pwrAux2]);
    }
    else if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // settings button  
    {
      // save minutes on exit
      EEPROM.write(6,1);
      EEPROM.write(7,feedingMins);
      // exit screen
      screenSettings();
    }

    break;

  case 8:    // heater settings screen

    myGLCD.setColor(255, 77, 0);
    myGLCD.setFont(arial_bold);

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // settings button  
    {     
      screenSettings();
    }
    else if ((x>=162)&&(x<=186)&&(y>=64)&&(y<=88))  // off up button  
    {     
      int theSetting=EEPROM.read(17); // read existing setting
      theSetting=theSetting+1; // increment
      itoa(theSetting, char3, 10);
      myGLCD.print(char3, 195, 84);
      EEPROM.write(17,theSetting); // save setting
      heatOffTemp=theSetting;
    }
    else if ((x>=162)&&(x<=186)&&(y>=94)&&(y<=118))  // off down button  
    {     
      int theSetting=EEPROM.read(17); // read existing setting
      theSetting=theSetting-1; // increment
      itoa(theSetting, char3, 10);
      myGLCD.print(char3, 195, 84);
      EEPROM.write(17,theSetting); // save setting
      heatOffTemp=theSetting;
    }
    else if ((x>=162)&&(x<=186)&&(y>=145)&&(y<=169))  // on up button  
    {     
      int theSetting=EEPROM.read(18); // read existing setting
      theSetting=theSetting+1; // increment
      itoa(theSetting, char3, 10);
      myGLCD.print(char3, 195, 164);
      EEPROM.write(18,theSetting); // save setting
      heatOnTemp=theSetting;
    }
    else if ((x>=162)&&(x<=186)&&(y>=175)&&(y<=199))  // on down button  
    {     
      int theSetting=EEPROM.read(18); // read existing setting
      theSetting=theSetting-1; // increment
      itoa(theSetting, char3, 10);
      myGLCD.print(char3, 195, 164);
      EEPROM.write(18,theSetting); // save setting
      heatOnTemp=theSetting;
    }
    else if ((x>=162)&&(x<=186)&&(y>=224)&&(y<=248))  // warn up button  
    {     
      int theSetting=EEPROM.read(19); // read existing setting
      theSetting=theSetting+1; // increment
      itoa(theSetting, char3, 10);
      myGLCD.print(char3, 195, 243);
      EEPROM.write(19,theSetting); // save setting
    }
    else if ((x>=162)&&(x<=186)&&(y>=254)&&(y<=278))  // warn down button  
    {     
      int theSetting=EEPROM.read(19); // read existing setting
      theSetting=theSetting-1; // increment
      itoa(theSetting, char3, 10);
      myGLCD.print(char3, 195, 243);
      EEPROM.write(19,theSetting); // save setting
    }

    break;

  case 9:    // schedule screen

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // home button  
    {     
      updateAlarms(); // this will rebuild all of the schedules
      screenSettings();
    }
    else if ((x>=10)&&(x<=58)&&(y>=50)&&(y<=113))
    {
      screenPwrSchedule();
    }
    else if ((x>=67)&&(x<=115)&&(y>=50)&&(y<=113))
    {
      screenDosingSched(1); // send 1 to start with Macro pump
    }
    else if ((x>=124)&&(x<=172)&&(y>=50)&&(y<=113))
    {
      screenLightRamps();
    }

    break;

  case 10:    // dosing configuration screen

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // settings button  
    {     
      screenSettings();
    }

    else if ((x>=145)&&(x<=169)&&(y>=42)&&(y<=66))   // dose up
    {     
      byte doseAmt=EEPROM.read(21); // 21 // read dose in mL
      doseAmt=doseAmt+1;  // increment up by one
      itoa(doseAmt, char3, 10);
      myGLCD.setColor(255, 77, 0);
      myGLCD.setFont(arial_bold);
      if (doseAmt>=0 && doseAmt<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 191, 59); // display dose size
      int doseCap=EEPROM.read(22); // 22 // read dosing reseviors capacity in ^10 mL;
      doseCap=doseCap*10; // multiply by 10 to get acutal mL
      int doses=(doseCap/doseAmt); // calcuate number of doses in a full resevoir
      itoa(doses, char3, 10); 
      if (doses>=0 && doses<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.setFont(Sinclair_S);
      myGLCD.print(char3, 115, 152);  // draw total doses to screen
      EEPROM.write(21,doseAmt);  // save to memory
    }
    else if ((x>=145)&&(x<=169)&&(y>=72)&&(y<=96))   // dose down
    {     
      byte doseAmt=EEPROM.read(21); // 21 // read dose in mL
      doseAmt=doseAmt-1;  // increment down by one
      itoa(doseAmt, char3, 10);
      if (doseAmt>=0 && doseAmt<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.setColor(255, 77, 0);
      myGLCD.setFont(arial_bold);
      myGLCD.print(char3, 191, 59);
      int doseCap=EEPROM.read(22); // 22 // read dosing reseviors capacity in ^10 mL;
      doseCap=doseCap*10; // multiply by 10 to get acutal mL
      int doses=(doseCap/doseAmt); // calcuate number of doses in a full resevoir
      itoa(doses, char3, 10);
      if (doses>=0 && doses<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }

      myGLCD.setFont(Sinclair_S);
      myGLCD.print(char3, 115, 152);  // draw total doses to screen
      EEPROM.write(21,doseAmt);  // save to memory
    }
    else if ((x>=145)&&(x<=169)&&(y>=110)&&(y<=134))   // resevoir volume up by 10
    {     
      int doseCap=EEPROM.read(22); // 22 // dosing reseviors capacity in mL;
      doseCap=(doseCap+1);  // increment up by 1
      EEPROM.write(22,doseCap); // save new value
      doseCap=(doseCap*10);  // convert to power of 10 to get actual mL
      itoa(doseCap, char4, 10);

      myGLCD.setColor(255, 77, 0);
      myGLCD.setFont(arial_bold);
      myGLCD.print(char4, 191, 127);

      byte doseAmt=EEPROM.read(21); // 21 // read dose in mL
      int doses=(doseCap/doseAmt); // calcuate number of doses in a full resevoir
      itoa(doses, char3, 10); 
      if (doses>=0 && doses<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.setFont(Sinclair_S);
      myGLCD.print(char3, 115, 152);  // draw total doses to screen
    }
    else if ((x>=145)&&(x<=169)&&(y>=140)&&(y<=164))   // resevoir volume down by 10
    {     
      int doseCap=EEPROM.read(22); // 22 // dosing reseviors capacity in mL;
      doseCap=(doseCap-1);  // increment up by 1
      EEPROM.write(22,doseCap); // save new value
      doseCap=(doseCap*10);  // convert to power of 10 to get actual mL
      itoa(doseCap, char4, 10);

      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(191,127,239,143);  // clear previous value

      myGLCD.setColor(255, 77, 0);
      myGLCD.setFont(arial_bold);
      myGLCD.print(char4, 191, 127);

      byte doseAmt=EEPROM.read(21); // 21 // read dose in mL
      int doses=(doseCap/doseAmt); // calcuate number of doses in a full resevoir
      itoa(doses, char3, 10); 
      if (doses>=0 && doses<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.setFont(Sinclair_S);
      myGLCD.print(char3, 115, 152);  // draw total doses to screen
    }
    else if ((x>=145)&&(x<=169)&&(y>=178)&&(y<=202))   // pump1 sec increased by .1 (technically 10 since it's 10th power)
    {     
      int pumpSpeed=EEPROM.read(23); // 22 // read saved motor ms/mL;
      pumpSpeed=(pumpSpeed+1);  // increment up by 1
      EEPROM.write(23,pumpSpeed); // save new value
      pumpSpeed=(pumpSpeed*10);  // convert to power of 10 to get actual ms
      itoa(pumpSpeed, char5, 10);
      int px=187;
      if (pumpSpeed>999) px=175;
      myGLCD.setColor(255, 77, 0);
      myGLCD.setFont(arial_bold);
      myGLCD.print(char5, px, 197);
    }
    else if ((x>=145)&&(x<=169)&&(y>=178)&&(y<=232))   // pump1 sec decreased by .1 (technically 10 since it's 10th power)
    {     
      int pumpSpeed=EEPROM.read(23); // 22 // read saved motor ms/mL;
      pumpSpeed=(pumpSpeed-1);  // increment up by 1
      EEPROM.write(23,pumpSpeed); // save new value
      pumpSpeed=(pumpSpeed*10);  // convert to power of 10 to get actual ms
      itoa(pumpSpeed, char5, 10);
      int px=187;
      if (pumpSpeed>999) px=175;
      myGLCD.setColor(0,0,0);
      myGLCD.fillRect(175,197,239,213); // clear previous value
      myGLCD.setColor(255, 77, 0);
      myGLCD.setFont(arial_bold);
      myGLCD.print(char5, px, 197);
    }
    else if ((x>=4)&&(x<=52)&&(y>=112)&&(y<=160))   // resevoir fill button
    {
      byte doseCap=EEPROM.read(22);  // read dosing resevoir capacity
      EEPROM.write(32,doseCap);  // reset the number of doses left, high byte
      EEPROM.write(33,0);  // reset the number of doses left, low byte
      EEPROM.write(34,doseCap); // reset the number of doses left, high byte
      EEPROM.write(35,0);  // reset the number of doses left, low byte
      byte doseAmt=EEPROM.read(21); // 21 // read dose in mL
      int doses=(doseCap/doseAmt); // calcuate number of doses in a full resevoir
      myGLCD.setColor(255, 77, 0);
      myGLCD.setFont(arial_bold);
      myGLCD.print("FULL", 62, 112);
    }   
    else if ((x>=145)&&(x<=169)&&(y>=245)&&(y<=269))   // pump 2 rate increase
    {     
      int pumpSpeed=EEPROM.read(24); // 22 // read saved motor ms/mL;
      pumpSpeed=(pumpSpeed+1);  // increment up by 1
      EEPROM.write(24,pumpSpeed); // save new value
      pumpSpeed=(pumpSpeed*10);  // convert to power of 10 to get actual ms
      itoa(pumpSpeed, char5, 10);
      int px=187;
      if (pumpSpeed>999) px=175;
      myGLCD.setColor(255, 77, 0);
      myGLCD.setFont(arial_bold);
      myGLCD.print(char5, px, 264);

    }
    else if ((x>=145)&&(x<=169)&&(y>=275)&&(y<=299))   // pump 2 rate decrease
    {     
      int pumpSpeed=EEPROM.read(24); // 22 // read saved motor ms/mL;
      pumpSpeed=(pumpSpeed-1);  // increment up by 1
      EEPROM.write(24,pumpSpeed); // save new value
      pumpSpeed=(pumpSpeed*10);  // convert to power of 10 to get actual ms
      itoa(pumpSpeed, char5, 10);
      int px=187;
      if (pumpSpeed>999) px=175;
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(175,264,239,280); // clear previous value
      myGLCD.setColor(255, 77, 0);
      myGLCD.setFont(arial_bold);
      myGLCD.print(char5, px, 264);
    }
    else if ((x>=4)&&(x<=52)&&(y>=181)&&(y<=229))   // pump1 test button
    {
      int pumpTime=EEPROM.read(23);  // 23 // pump 1 sec/ml
      int doseAmt=EEPROM.read(21); // 21 // dose in mL
      pumpTime=((pumpTime*10)*doseAmt); // multiply ms/mL by number of mL to pump
      analogWrite(dosingPump1, 255);
      delay(pumpTime); // set how long you want the motor to run... 1000 = aprox 1ml, test your own pump
      analogWrite(dosingPump1, 0);
    }   
    else if ((x>=4)&&(x<=52)&&(y>=249)&&(y<=297))   // pump2 test button
    {
      int pumpTime=EEPROM.read(24);  // 23 // pump 2 sec/ml
      int doseAmt=EEPROM.read(21); // 21 // dose in mL
      pumpTime=((pumpTime*10)*doseAmt); // multiply ms/mL by number of mL to pump
      analogWrite(dosingPump2, 255);
      delay(pumpTime); // set how long you want the motor to run... 1000 = aprox 1ml, test your own pump
      analogWrite(dosingPump2, 0);
    }   
    break;

  case 11:    // power schedule screen

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // schedule button  
    {     
      screenSchedule();
    }
    else if ((x>=0)&&(x<=185)&&(y>=100)&&(y<=125)) 
    {     
      screenPwrScheduleItem(1);
    }
    else if ((x>=0)&&(x<=185)&&(y>=126)&&(y<=151))
    {
      screenPwrScheduleItem(2);
    }
    else if ((x>=0)&&(x<=185)&&(y>=152)&&(y<=177))
    {
      screenPwrScheduleItem(3);
    }
    else if ((x>=0)&&(x<=185)&&(y>=178)&&(y<=203))
    {
      screenPwrScheduleItem(4);
    }
    else if ((x>=0)&&(x<=185)&&(y>=204)&&(y<=229))
    {
      screenPwrScheduleItem(5);
    }
    else if ((x>=0)&&(x<=185)&&(y>=230)&&(y<=255))
    {
      screenPwrScheduleItem(6);
    }

    break;

  case 12:    // power item schedule screen

    myGLCD.setFont(arial_bold);
    myGLCD.setColor(255, 77, 0);

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // schedule button  
    {     
      screenPwrSchedule();
    }
    else if ((x>=83)&&(x<=107)&&(y>=94)&&(y<=118)) // on hour up
    {
      int onHour;
      if (scheduleItem==1)
      {
        if (schedLights1.onHour==23) schedLights1.onHour=0;
        else schedLights1.onHour=schedLights1.onHour+1;
        onHour=schedLights1.onHour;
        EEPROM.write(101,onHour); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (schedLights2.onHour==23) schedLights2.onHour=0;
        else schedLights2.onHour=schedLights2.onHour+1;
        onHour=schedLights2.onHour;
        EEPROM.write(106,onHour); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (schedCirc.onHour==23) schedCirc.onHour=0;
        else schedCirc.onHour=schedCirc.onHour+1;
        onHour=schedCirc.onHour;
        EEPROM.write(111,onHour); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (schedCo2.onHour==23) schedCo2.onHour=0;
        else schedCo2.onHour=schedCo2.onHour+1;
        onHour=schedCo2.onHour;
        EEPROM.write(116,onHour); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (schedAux1.onHour==23) schedAux1.onHour=0;
        else schedAux1.onHour=schedAux1.onHour+1;
        onHour=schedAux1.onHour;
        EEPROM.write(121,onHour); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (schedAux2.onHour==23) schedAux2.onHour=0;
        else schedAux2.onHour=schedAux2.onHour+1;
        onHour=schedAux2.onHour;
        EEPROM.write(126,onHour); // save to memory
      }

      // print to screen
      itoa(onHour, char3, 10);
      if (onHour>=0 && onHour<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 123, 113);

    }
    else if ((x>=83)&&(x<=107)&&(y>=124)&&(y<=148)) // on hour down
    {
      int onHour;
      if (scheduleItem==1)
      {
        if (schedLights1.onHour==0) schedLights1.onHour=23;
        else schedLights1.onHour=schedLights1.onHour-1;
        onHour=schedLights1.onHour;
        EEPROM.write(101,onHour); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (schedLights2.onHour==0) schedLights2.onHour=23;
        else schedLights2.onHour=schedLights2.onHour-1;
        onHour=schedLights2.onHour;
        EEPROM.write(106,onHour); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (schedCirc.onHour==0) schedCirc.onHour=23;
        else schedCirc.onHour=schedCirc.onHour-1;
        onHour=schedCirc.onHour;
        EEPROM.write(111,onHour); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (schedCo2.onHour==0) schedCo2.onHour=23;
        else schedCo2.onHour=schedCo2.onHour-1;
        onHour=schedCo2.onHour;
        EEPROM.write(116,onHour); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (schedAux1.onHour==0) schedAux1.onHour=23;
        else schedAux1.onHour=schedAux1.onHour-1;
        onHour=schedAux1.onHour;
        EEPROM.write(121,onHour); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (schedAux2.onHour==0) schedAux2.onHour=23;
        else schedAux2.onHour=schedAux2.onHour-1;
        onHour=schedAux2.onHour;
        EEPROM.write(126,onHour); // save to memory
      }

      // print to screen
      itoa(onHour, char3, 10);
      if (onHour>=0 && onHour<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 123, 113);

    }
    else if ((x>=164)&&(x<=188)&&(y>=94)&&(y<=118)) // on min up
    {
      int onMinute;
      if (scheduleItem==1)
      {
        if (schedLights1.onMinute==59) schedLights1.onMinute=0;
        else schedLights1.onMinute=schedLights1.onMinute+1;
        onMinute=schedLights1.onMinute;
        EEPROM.write(102,onMinute); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (schedLights2.onMinute==59) schedLights2.onMinute=0;
        else schedLights2.onMinute=schedLights2.onMinute+1;
        onMinute=schedLights2.onMinute;
        EEPROM.write(107,onMinute); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (schedCirc.onMinute==59) schedCirc.onMinute=0;
        else schedCirc.onMinute=schedCirc.onMinute+1;
        onMinute=schedCirc.onMinute;
        EEPROM.write(112,onMinute); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (schedCo2.onMinute==59) schedCo2.onMinute=0;
        else schedCo2.onMinute=schedCo2.onMinute+1;
        onMinute=schedCo2.onMinute;
        EEPROM.write(117,onMinute); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (schedAux1.onMinute==59) schedAux1.onMinute=0;
        else schedAux1.onMinute=schedAux1.onMinute+1;
        onMinute=schedAux1.onMinute;
        EEPROM.write(122,onMinute); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (schedAux2.onMinute==59) schedAux2.onMinute=0;
        else schedAux2.onMinute=schedAux2.onMinute+1;
        onMinute=schedAux2.onMinute;
        EEPROM.write(127,onMinute); // save to memory
      }

      // print to screen
      itoa(onMinute, char3, 10);
      if (onMinute>=0 && onMinute<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 201, 113);

    }
    else if ((x>=164)&&(x<=188)&&(y>=124)&&(y<=148)) // on min down
    {
      int onMinute;
      if (scheduleItem==1)
      {
        if (schedLights1.onMinute==0) schedLights1.onMinute=59;
        else schedLights1.onMinute=schedLights1.onMinute-1;
        onMinute=schedLights1.onMinute;
        EEPROM.write(102,onMinute); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (schedLights2.onMinute==0) schedLights2.onMinute=59;
        else schedLights2.onMinute=schedLights2.onMinute-1;
        onMinute=schedLights2.onMinute;
        EEPROM.write(107,onMinute); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (schedCirc.onMinute==0) schedCirc.onMinute=59;
        else schedCirc.onMinute=schedCirc.onMinute-1;
        onMinute=schedCirc.onMinute;
        EEPROM.write(112,onMinute); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (schedCo2.onMinute==0) schedCo2.onMinute=59;
        else schedCo2.onMinute=schedCo2.onMinute-1;
        onMinute=schedCo2.onMinute;
        EEPROM.write(117,onMinute); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (schedAux1.onMinute==0) schedAux1.onMinute=59;
        else schedAux1.onMinute=schedAux1.onMinute-1;
        onMinute=schedAux1.onMinute;
        EEPROM.write(122,onMinute); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (schedAux2.onMinute==0) schedAux2.onMinute=59;
        else schedAux2.onMinute=schedAux2.onMinute-1;
        onMinute=schedAux2.onMinute;
        EEPROM.write(127,onMinute); // save to memory
      }

      // print to screen
      itoa(onMinute, char3, 10);
      if (onMinute>=0 && onMinute<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 201, 113);

    }
    else if ((x>=83)&&(x<=107)&&(y>=175)&&(y<=199)) // off hour up
    {
      int offHour;
      if (scheduleItem==1)
      {
        if (schedLights1.offHour==23) schedLights1.offHour=0;
        else schedLights1.offHour=schedLights1.offHour+1;
        offHour=schedLights1.offHour;
        EEPROM.write(103,offHour); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (schedLights2.offHour==23) schedLights2.offHour=0;
        else schedLights2.offHour=schedLights2.offHour+1;
        offHour=schedLights2.offHour;
        EEPROM.write(108,offHour); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (schedCirc.offHour==23) schedCirc.offHour=0;
        else schedCirc.offHour=schedCirc.offHour+1;
        offHour=schedCirc.offHour;
        EEPROM.write(113,offHour); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (schedCo2.offHour==23) schedCo2.offHour=0;
        else schedCo2.offHour=schedCo2.offHour+1;
        offHour=schedCo2.offHour;
        EEPROM.write(118,offHour); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (schedAux1.offHour==23) schedAux1.offHour=0;
        else schedAux1.offHour=schedAux1.offHour+1;
        offHour=schedAux1.offHour;
        EEPROM.write(123,offHour); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (schedAux2.offHour==23) schedAux2.offHour=0;
        else schedAux2.offHour=schedAux2.offHour+1;
        offHour=schedAux2.offHour;
        EEPROM.write(128,offHour); // save to memory
      }

      // print to screen
      itoa(offHour, char3, 10);
      if (offHour>=0 && offHour<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 123, 194);            
    }
    else if ((x>=83)&&(x<=107)&&(y>=205)&&(y<=229)) // off hour down
    {
      int offHour;
      if (scheduleItem==1)
      {
        if (schedLights1.offHour==0) schedLights1.offHour=23;
        else schedLights1.offHour=schedLights1.offHour-1;
        offHour=schedLights1.offHour;
        EEPROM.write(103,offHour); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (schedLights2.offHour==0) schedLights2.offHour=23;
        else schedLights2.offHour=schedLights2.offHour-1;
        offHour=schedLights2.offHour;
        EEPROM.write(108,offHour); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (schedCirc.offHour==0) schedCirc.offHour=23;
        else schedCirc.offHour=schedCirc.offHour-1;
        offHour=schedCirc.offHour;
        EEPROM.write(113,offHour); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (schedCo2.offHour==0) schedCo2.offHour=23;
        else schedCo2.offHour=schedCo2.offHour-1;
        offHour=schedCo2.offHour;
        EEPROM.write(118,offHour); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (schedAux1.offHour==0) schedAux1.offHour=23;
        else schedAux1.offHour=schedAux1.offHour-1;
        offHour=schedAux1.offHour;
        EEPROM.write(123,offHour); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (schedAux2.offHour==0) schedAux2.offHour=23;
        else schedAux2.offHour=schedAux2.offHour-1;
        offHour=schedAux2.offHour;
        EEPROM.write(128,offHour); // save to memory
      }

      // print to screen
      itoa(offHour, char3, 10);
      if (offHour>=0 && offHour<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 123, 194);               
    }
    else if ((x>=164)&&(x<=188)&&(y>=175)&&(y<=199)) // off min up
    {
      int offMinute;
      if (scheduleItem==1)
      {
        if (schedLights1.offMinute==59) schedLights1.offMinute=0;
        else schedLights1.offMinute=schedLights1.offMinute+1;
        offMinute=schedLights1.offMinute;
        EEPROM.write(104,offMinute); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (schedLights2.offMinute==59) schedLights2.offMinute=0;
        else schedLights2.offMinute=schedLights2.offMinute+1;
        offMinute=schedLights2.offMinute;
        EEPROM.write(109,offMinute); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (schedCirc.offMinute==59) schedCirc.offMinute=0;
        else schedCirc.offMinute=schedCirc.offMinute+1;
        offMinute=schedCirc.offMinute;
        EEPROM.write(114,offMinute); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (schedCo2.offMinute==59) schedCo2.offMinute=0;
        else schedCo2.offMinute=schedCo2.offMinute+1;
        offMinute=schedCo2.offMinute;
        EEPROM.write(119,offMinute); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (schedAux1.offMinute==59) schedAux1.offMinute=0;
        else schedAux1.offMinute=schedAux1.offMinute+1;
        offMinute=schedAux1.offMinute;
        EEPROM.write(124,offMinute); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (schedAux2.offMinute==59) schedAux2.offMinute=0;
        else schedAux2.offMinute=schedAux2.offMinute+1;
        offMinute=schedAux2.offMinute;
        EEPROM.write(129,offMinute); // save to memory
      }

      // print to screen
      itoa(offMinute, char3, 10);
      if (offMinute>=0 && offMinute<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 201, 194);

    }
    else if ((x>=164)&&(x<=188)&&(y>=205)&&(y<=229)) // off min down
    {
      int offMinute;
      if (scheduleItem==1)
      {
        if (schedLights1.offMinute==0) schedLights1.offMinute=59;
        else schedLights1.offMinute=schedLights1.offMinute-1;
        offMinute=schedLights1.offMinute;
        EEPROM.write(104,offMinute); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (schedLights2.offMinute==0) schedLights2.offMinute=59;
        else schedLights2.offMinute=schedLights2.offMinute-1;
        offMinute=schedLights2.offMinute;
        EEPROM.write(109,offMinute); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (schedCirc.offMinute==0) schedCirc.offMinute=59;
        else schedCirc.offMinute=schedCirc.offMinute-1;
        offMinute=schedCirc.offMinute;
        EEPROM.write(114,offMinute); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (schedCo2.offMinute==0) schedCo2.offMinute=59;
        else schedCo2.offMinute=schedCo2.offMinute-1;
        offMinute=schedCo2.offMinute;
        EEPROM.write(119,offMinute); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (schedAux1.offMinute==0) schedAux1.offMinute=59;
        else schedAux1.offMinute=schedAux1.offMinute-1;
        offMinute=schedAux1.offMinute;
        EEPROM.write(124,offMinute); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (schedAux2.offMinute==0) schedAux2.offMinute=59;
        else schedAux2.offMinute=schedAux2.offMinute-1;
        offMinute=schedAux2.offMinute;
        EEPROM.write(129,offMinute); // save to memory
      }

      // print to screen
      itoa(offMinute, char3, 10);
      if (offMinute>=0 && offMinute<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 201, 194);

    }
    else if ((x>=161)&&(x<=191)&&(y>=249)&&(y<=279)) // active button
    {
      int isActive;
      if (scheduleItem==1)
      {
        if (schedLights1.active==1) schedLights1.active=0;
        else schedLights1.active=1;
        isActive=schedLights1.active;
        EEPROM.write(100,isActive); // save to memory
      }
      if (scheduleItem==2)
      {
        if (schedLights2.active==1) schedLights2.active=0;
        else schedLights2.active=1;
        isActive=schedLights2.active;
        EEPROM.write(105,isActive); // save to memory
      }
      if (scheduleItem==3)
      {
        if (schedCirc.active==1) schedCirc.active=0;
        else schedCirc.active=1;
        isActive=schedCirc.active;
        EEPROM.write(110,isActive); // save to memory
      }
      if (scheduleItem==4)
      {
        if (schedCo2.active==1) schedCo2.active=0;
        else schedCo2.active=1;
        isActive=schedCo2.active;
        EEPROM.write(115,isActive); // save to memory
      }
      if (scheduleItem==5)
      {
        if (schedAux1.active==1) schedAux1.active=0;
        else schedAux1.active=1;
        isActive=schedAux1.active;
        EEPROM.write(120,isActive); // save to memory
      }
      if (scheduleItem==6)
      {
        if (schedAux2.active==1) schedAux2.active=0;
        else schedAux2.active=1;
        isActive=schedAux2.active;
        EEPROM.write(125,isActive); // save to memory
      }

      myFiles.loadBitmap(161, 249, 30, 30, schedActiveB[isActive]);
    }

    break;

  case 13:    // light ramp schedule screen

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // schedule button  
    {     
      screenSchedule();
    }
    else if ((x>=0)&&(x<=185)&&(y>=100)&&(y<=125)) 
    {     
      screenLightRampItem(1);
    }
    else if ((x>=0)&&(x<=185)&&(y>=126)&&(y<=151))
    {
      screenLightRampItem(2);
    }
    else if ((x>=0)&&(x<=185)&&(y>=152)&&(y<=177))
    {
      screenLightRampItem(3);
    }
    else if ((x>=0)&&(x<=185)&&(y>=178)&&(y<=203))
    {
      screenLightRampItem(4);
    }
    else if ((x>=0)&&(x<=185)&&(y>=204)&&(y<=229))
    {
      screenLightRampItem(5);
    }
    else if ((x>=0)&&(x<=185)&&(y>=230)&&(y<=255))
    {
      screenLightRampItem(6);
    }

    break;

  case 14:    // power item schedule screen

    myGLCD.setFont(arial_bold);
    myGLCD.setColor(255, 77, 0);

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // schedule button  
    {     
      screenLightRamps();
    }
    else if ((x>=75)&&(x<=99)&&(y>=120)&&(y<=144)) // on hour up
    {
      int onHour;
      if (scheduleItem==1)
      {
        if (ramp1.onHour==23) ramp1.onHour=0;
        else ramp1.onHour=ramp1.onHour+1;
        onHour=ramp1.onHour;
        EEPROM.write(220,onHour); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (ramp2.onHour==23) ramp2.onHour=0;
        else ramp2.onHour=ramp2.onHour+1;
        onHour=ramp2.onHour;
        EEPROM.write(224,onHour); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (ramp3.onHour==23) ramp3.onHour=0;
        else ramp3.onHour=ramp3.onHour+1;
        onHour=ramp3.onHour;
        EEPROM.write(228,onHour); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (ramp4.onHour==23) ramp4.onHour=0;
        else ramp4.onHour=ramp4.onHour+1;
        onHour=ramp4.onHour;
        EEPROM.write(232,onHour); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (ramp5.onHour==23) ramp5.onHour=0;
        else ramp5.onHour=ramp5.onHour+1;
        onHour=ramp5.onHour;
        EEPROM.write(236,onHour); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (ramp6.onHour==23) ramp6.onHour=0;
        else ramp6.onHour=ramp6.onHour+1;
        onHour=ramp6.onHour;
        EEPROM.write(240,onHour); // save to memory
      }

      // print to screen
      itoa(onHour, char3, 10);
      if (onHour>=0 && onHour<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 110, 139);

    }
    else if ((x>=75)&&(x<=99)&&(y>=150)&&(y<=174)) // on hour down
    {
      int onHour;
      if (scheduleItem==1)
      {
        if (ramp1.onHour==0) ramp1.onHour=23;
        else ramp1.onHour=ramp1.onHour-1;
        onHour=ramp1.onHour;
        EEPROM.write(220,onHour); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (ramp2.onHour==0) ramp2.onHour=23;
        else ramp2.onHour=ramp2.onHour-1;
        onHour=ramp2.onHour;
        EEPROM.write(224,onHour); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (ramp3.onHour==0) ramp3.onHour=23;
        else ramp3.onHour=ramp3.onHour-1;
        onHour=ramp3.onHour;
        EEPROM.write(228,onHour); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (ramp4.onHour==0) ramp4.onHour=23;
        else ramp4.onHour=ramp4.onHour-1;
        onHour=ramp4.onHour;
        EEPROM.write(232,onHour); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (ramp5.onHour==0) ramp5.onHour=23;
        else ramp5.onHour=ramp5.onHour-1;
        onHour=ramp5.onHour;
        EEPROM.write(236,onHour); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (ramp6.onHour==0) ramp6.onHour=23;
        else ramp6.onHour=ramp6.onHour-1;
        onHour=ramp6.onHour;
        EEPROM.write(240,onHour); // save to memory
      }

      // print to screen
      itoa(onHour, char3, 10);
      if (onHour>=0 && onHour<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 110, 139);

    }
    else if ((x>=154)&&(x<=178)&&(y>=120)&&(y<=144)) // on min up
    {
      int onMinute;
      if (scheduleItem==1)
      {
        if (ramp1.onMinute==59) ramp1.onMinute=0;
        else ramp1.onMinute=ramp1.onMinute+1;
        onMinute=ramp1.onMinute;
        EEPROM.write(221,onMinute); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (ramp2.onMinute==59) ramp2.onMinute=0;
        else ramp2.onMinute=ramp2.onMinute+1;
        onMinute=ramp2.onMinute;
        EEPROM.write(225,onMinute); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (ramp3.onMinute==59) ramp3.onMinute=0;
        else ramp3.onMinute=ramp3.onMinute+1;
        onMinute=ramp3.onMinute;
        EEPROM.write(229,onMinute); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (ramp4.onMinute==59) ramp4.onMinute=0;
        else ramp4.onMinute=ramp4.onMinute+1;
        onMinute=ramp4.onMinute;
        EEPROM.write(233,onMinute); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (ramp5.onMinute==59) ramp5.onMinute=0;
        else ramp5.onMinute=ramp5.onMinute+1;
        onMinute=ramp5.onMinute;
        EEPROM.write(237,onMinute); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (ramp6.onMinute==59) ramp6.onMinute=0;
        else ramp6.onMinute=ramp6.onMinute+1;
        onMinute=ramp6.onMinute;
        EEPROM.write(241,onMinute); // save to memory
      }

      // print to screen
      itoa(onMinute, char3, 10);
      if (onMinute>=0 && onMinute<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 189, 139);

    }
    else if ((x>=154)&&(x<=178)&&(y>=150)&&(y<=174)) // on min down
    {
      int onMinute;
      if (scheduleItem==1)
      {
        if (ramp1.onMinute==0) ramp1.onMinute=59;
        else ramp1.onMinute=ramp1.onMinute-1;
        onMinute=ramp1.onMinute;
        EEPROM.write(221,onMinute); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (ramp2.onMinute==0) ramp2.onMinute=59;
        else ramp2.onMinute=ramp2.onMinute-1;
        onMinute=ramp2.onMinute;
        EEPROM.write(225,onMinute); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (ramp3.onMinute==0) ramp3.onMinute=59;
        else ramp3.onMinute=ramp3.onMinute-1;
        onMinute=ramp3.onMinute;
        EEPROM.write(229,onMinute); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (ramp4.onMinute==0) ramp4.onMinute=59;
        else ramp4.onMinute=ramp4.onMinute-1;
        onMinute=ramp4.onMinute;
        EEPROM.write(233,onMinute); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (ramp5.onMinute==0) ramp5.onMinute=59;
        else ramp5.onMinute=ramp5.onMinute-1;
        onMinute=ramp5.onMinute;
        EEPROM.write(237,onMinute); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (ramp6.onMinute==0) ramp6.onMinute=59;
        else ramp6.onMinute=ramp6.onMinute-1;
        onMinute=ramp6.onMinute;
        EEPROM.write(241,onMinute); // save to memory
      }

      // print to screen
      itoa(onMinute, char3, 10);
      if (onMinute>=0 && onMinute<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 189, 139);

    }
    else if ((x>=75)&&(x<=99)&&(y>=231)&&(y<=255)) // off hour up
    {
      int offHour;
      if (scheduleItem==1)
      {
        if (ramp1.offHour==23) ramp1.offHour=0;
        else ramp1.offHour=ramp1.offHour+1;
        offHour=ramp1.offHour;
        EEPROM.write(222,offHour); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (ramp2.offHour==23) ramp2.offHour=0;
        else ramp2.offHour=ramp2.offHour+1;
        offHour=ramp2.offHour;
        EEPROM.write(226,offHour); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (ramp3.offHour==23) ramp3.offHour=0;
        else ramp3.offHour=ramp3.offHour+1;
        offHour=ramp3.offHour;
        EEPROM.write(230,offHour); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (ramp4.offHour==23) ramp4.offHour=0;
        else ramp4.offHour=ramp4.offHour+1;
        offHour=ramp4.offHour;
        EEPROM.write(234,offHour); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (ramp5.offHour==23) ramp5.offHour=0;
        else ramp5.offHour=ramp5.offHour+1;
        offHour=ramp5.offHour;
        EEPROM.write(238,offHour); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (ramp6.offHour==23) ramp6.offHour=0;
        else ramp6.offHour=ramp6.offHour+1;
        offHour=ramp6.offHour;
        EEPROM.write(242,offHour); // save to memory
      }

      // print to screen
      itoa(offHour, char3, 10);
      if (offHour>=0 && offHour<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 110, 250);
    }
    else if ((x>=75)&&(x<=99)&&(y>=261)&&(y<=285)) // off hour down
    {
      int offHour;
      if (scheduleItem==1)
      {
        if (ramp1.offHour==0) ramp1.offHour=23;
        else ramp1.offHour=ramp1.offHour-1;
        offHour=ramp1.offHour;
        EEPROM.write(222,offHour); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (ramp2.offHour==0) ramp2.offHour=23;
        else ramp2.offHour=ramp2.offHour-1;
        offHour=ramp2.offHour;
        EEPROM.write(226,offHour); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (ramp3.offHour==0) ramp3.offHour=23;
        else ramp3.offHour=ramp3.offHour-1;
        offHour=ramp3.offHour;
        EEPROM.write(230,offHour); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (ramp4.offHour==0) ramp4.offHour=23;
        else ramp4.offHour=ramp4.offHour-1;
        offHour=ramp4.offHour;
        EEPROM.write(234,offHour); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (ramp5.offHour==0) ramp5.offHour=23;
        else ramp5.offHour=ramp5.offHour-1;
        offHour=ramp5.offHour;
        EEPROM.write(238,offHour); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (ramp6.offHour==0) ramp6.offHour=23;
        else ramp6.offHour=ramp6.offHour-1;
        offHour=ramp6.offHour;
        EEPROM.write(242,offHour); // save to memory
      }

      // print to screen
      itoa(offHour, char3, 10);
      if (offHour>=0 && offHour<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 110, 250);
    }
    else if ((x>=154)&&(x<=178)&&(y>=231)&&(y<=255)) // off min up
    {
      int offMinute;
      if (scheduleItem==1)
      {
        if (ramp1.offMinute==59) ramp1.offMinute=0;
        else ramp1.offMinute=ramp1.offMinute+1;
        offMinute=ramp1.offMinute;
        EEPROM.write(223,offMinute); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (ramp2.offMinute==59) ramp2.offMinute=0;
        else ramp2.offMinute=ramp2.offMinute+1;
        offMinute=ramp2.offMinute;
        EEPROM.write(227,offMinute); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (ramp3.offMinute==59) ramp3.offMinute=0;
        else ramp3.offMinute=ramp3.offMinute+1;
        offMinute=ramp3.offMinute;
        EEPROM.write(231,offMinute); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (ramp4.offMinute==59) ramp4.offMinute=0;
        else ramp4.offMinute=ramp4.offMinute+1;
        offMinute=ramp4.offMinute;
        EEPROM.write(235,offMinute); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (ramp5.offMinute==59) ramp5.offMinute=0;
        else ramp5.offMinute=ramp5.offMinute+1;
        offMinute=ramp5.offMinute;
        EEPROM.write(239,offMinute); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (ramp6.offMinute==59) ramp6.offMinute=0;
        else ramp6.offMinute=ramp6.offMinute+1;
        offMinute=ramp6.offMinute;
        EEPROM.write(243,offMinute); // save to memory
      }

      // print to screen
      itoa(offMinute, char3, 10);
      if (offMinute>=0 && offMinute<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 189, 250);

    }
    else if ((x>=154)&&(x<=178)&&(y>=261)&&(y<=285)) // off min down
    {
      int offMinute;
      if (scheduleItem==1)
      {
        if (ramp1.offMinute==0) ramp1.offMinute=59;
        else ramp1.offMinute=ramp1.offMinute-1;
        offMinute=ramp1.offMinute;
        EEPROM.write(223,offMinute); // save to memory
      }
      else if (scheduleItem==2)
      {
        if (ramp2.offMinute==0) ramp2.offMinute=59;
        else ramp2.offMinute=ramp2.offMinute-1;
        offMinute=ramp2.offMinute;
        EEPROM.write(227,offMinute); // save to memory
      }
      else if (scheduleItem==3)
      {
        if (ramp3.offMinute==0) ramp3.offMinute=59;
        else ramp3.offMinute=ramp3.offMinute-1;
        offMinute=ramp3.offMinute;
        EEPROM.write(231,offMinute); // save to memory
      }
      else if (scheduleItem==4)
      {
        if (ramp4.offMinute==0) ramp4.offMinute=59;
        else ramp4.offMinute=ramp4.offMinute-1;
        offMinute=ramp4.offMinute;
        EEPROM.write(235,offMinute); // save to memory
      }
      else if (scheduleItem==5)
      {
        if (ramp5.offMinute==0) ramp5.offMinute=59;
        else ramp5.offMinute=ramp5.offMinute-1;
        offMinute=ramp5.offMinute;
        EEPROM.write(239,offMinute); // save to memory
      }
      else if (scheduleItem==6)
      {
        if (ramp6.offMinute==0) ramp6.offMinute=59;
        else ramp6.offMinute=ramp6.offMinute-1;
        offMinute=ramp6.offMinute;
        EEPROM.write(243,offMinute); // save to memory
      }

      // print to screen
      itoa(offMinute, char3, 10);
      if (offMinute>=0 && offMinute<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 189, 250);

    }
    break;

  case 15:  // macro and micro dosing schedule

    // scheduleItem 1 = macros
    // scheduleItem 2 = micros
    myGLCD.setFont(arial_bold);

    // set font color based on micros or macros
    if (scheduleItem==1) myGLCD.setColor(34, 81, 255);
    else myGLCD.setColor(255, 77, 0);

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // schedule button  
    {     
      screenSchedule();
    }
    else if ((x>=12)&&(x<=58)&&(y>=71)&&(y<=119))  // macros button  
    {     
      if (scheduleItem==2) screenDosingSched(1); // switch to macros schedule
    }
    else if ((x>=12)&&(x<=58)&&(y>=120)&&(y<=168))  // micros button  
    {     
      if (scheduleItem==1) screenDosingSched(2); // switch to micros schedule
    }
    else if ((x>=85)&&(x<=109)&&(y>=110)&&(y<=134)) // on hour up
    {
      int onHour;
      if (scheduleItem==1) // for Macros
      {
        if (pump1.onHour==23) pump1.onHour=0;
        else pump1.onHour=pump1.onHour+1;
        onHour=pump1.onHour;
        EEPROM.write(300,onHour); // save to memory
      }
      else if (scheduleItem==2) // for Micros
      {
        if (pump2.onHour==23) pump2.onHour=0;
        else pump2.onHour=pump2.onHour+1;
        onHour=pump2.onHour;
        EEPROM.write(309,onHour); // save to memory
      }
      // print to screen
      itoa(onHour, char3, 10);
      if (onHour>=0 && onHour<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 120, 129);

    }
    else if ((x>=85)&&(x<=109)&&(y>=140)&&(y<=164)) // on hour down
    {
      int onHour;
      if (scheduleItem==1) // for Macros
      {
        if (pump1.onHour==0) pump1.onHour=23;
        else pump1.onHour=pump1.onHour-1;
        onHour=pump1.onHour;
        EEPROM.write(300,onHour); // save to memory
      }
      else if (scheduleItem==2) // for Micros
      {
        if (pump2.onHour==0) pump2.onHour=23;
        else pump2.onHour=pump2.onHour-1;
        onHour=pump2.onHour;
        EEPROM.write(309,onHour); // save to memory
      }
      // print to screen
      itoa(onHour, char3, 10);
      if (onHour>=0 && onHour<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 120, 129);         
    } 

    else if ((x>=164)&&(x<=188)&&(y>=110)&&(y<=134)) // on min up
    {
      int onMinute;
      if (scheduleItem==1) // for Macros
      {
        if (pump1.onMinute==59) pump1.onMinute=0;
        else pump1.onMinute=pump1.onMinute+1;
        onMinute=pump1.onMinute;
        EEPROM.write(301,onMinute); // save to memory
      }
      else if (scheduleItem==2) // for Micros
      {
        if (pump2.onMinute==59) pump2.onMinute=0;
        else pump2.onMinute=pump2.onMinute+1;
        onMinute=pump2.onMinute;
        EEPROM.write(310,onMinute); // save to memory
      }
      // print to screen
      itoa(onMinute, char3, 10);
      if (onMinute>=0 && onMinute<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 199, 129);
    }
    else if ((x>=164)&&(x<=188)&&(y>=140)&&(y<=164)) // on min down
    {
      int onMinute;
      if (scheduleItem==1) // for Macros
      {
        if (pump1.onMinute==0) pump1.onMinute=59;
        else pump1.onMinute=pump1.onMinute-1;
        onMinute=pump1.onMinute;
        EEPROM.write(301,onMinute); // save to memory
      }
      else if (scheduleItem==2) // for Micros
      {
        if (pump2.onMinute==0) pump2.onMinute=59;
        else pump2.onMinute=pump2.onMinute-1;
        onMinute=pump2.onMinute;
        EEPROM.write(310,onMinute); // save to memory
      }
      // print to screen
      itoa(onMinute, char3, 10);
      if (onMinute>=0 && onMinute<=9) // add a zero
      {
        itoa(0, char3t, 10); //make char3t 0
        strcat(char3t, char3);
        strcpy (char3,char3t);
      }
      myGLCD.print(char3, 199, 129);
    }
    else if ((x>=51)&&(x<=81)&&(y>=203)&&(y<=233)) // dose on Sunday
    {
      if (scheduleItem==1) // for Macros
      {
        if (pump1.Sunday==1) pump1.Sunday=0;
        else pump1.Sunday=1;
        EEPROM.write(302,pump1.Sunday); // save to memory
        myFiles.loadBitmap(51, 203, 30, 30, schedActiveB[pump1.Sunday]); 
      }
      else if (scheduleItem==2) // for Micros
      {
        if (pump2.Sunday==1) pump2.Sunday=0;
        else pump2.Sunday=1;
        EEPROM.write(311,pump2.Sunday); // save to memory
        myFiles.loadBitmap(51, 203, 30, 30, schedActiveB[pump2.Sunday]); 
      }
    }
    else if ((x>=105)&&(x<=135)&&(y>=203)&&(y<=233)) // dose on Monday
    {
      if (scheduleItem==1) // for Macros
      {
        if (pump1.Monday==1) pump1.Monday=0;
        else pump1.Monday=1;
        EEPROM.write(303,pump1.Monday); // save to memory
        myFiles.loadBitmap(105, 203, 30, 30, schedActiveB[pump1.Monday]); 
      }
      else if (scheduleItem==2) // for Micros
      {
        if (pump2.Monday==1) pump2.Monday=0;
        else pump2.Monday=1;
        EEPROM.write(312,pump2.Monday); // save to memory
        myFiles.loadBitmap(105, 203, 30, 30, schedActiveB[pump2.Monday]); 
      }
    }
    else if ((x>=159)&&(x<=189)&&(y>=203)&&(y<=233)) // dose on Tuesday
    {
      if (scheduleItem==1) // for Macros
      {
        if (pump1.Tuesday==1) pump1.Tuesday=0;
        else pump1.Tuesday=1;
        EEPROM.write(304,pump1.Tuesday); // save to memory
        myFiles.loadBitmap(159, 203, 30, 30, schedActiveB[pump1.Tuesday]); 
      }
      else if (scheduleItem==2) // for Micros
      {
        if (pump2.Tuesday==1) pump2.Tuesday=0;
        else pump2.Tuesday=1;
        EEPROM.write(313,pump2.Tuesday); // save to memory
        myFiles.loadBitmap(159, 203, 30, 30, schedActiveB[pump2.Tuesday]); 
      }
    }
    else if ((x>=30)&&(x<=60)&&(y>=250)&&(y<=280)) // dose on Wednesday
    {
      if (scheduleItem==1) // for Macros
      {
        if (pump1.Wednesday==1) pump1.Wednesday=0;
        else pump1.Wednesday=1;
        EEPROM.write(305,pump1.Wednesday); // save to memory
        myFiles.loadBitmap(30, 250, 30, 30, schedActiveB[pump1.Wednesday]); 
      }
      else if (scheduleItem==2) // for Micros
      {
        if (pump2.Wednesday==1) pump2.Wednesday=0;
        else pump2.Wednesday=1;
        EEPROM.write(314,pump2.Wednesday); // save to memory
        myFiles.loadBitmap(30, 250, 30, 30, schedActiveB[pump2.Wednesday]); 
      }
    }
    else if ((x>=84)&&(x<=114)&&(y>=250)&&(y<=280)) // dose on Thursday
    {
      if (scheduleItem==1) // for Macros
      {
        if (pump1.Thursday==1) pump1.Thursday=0;
        else pump1.Thursday=1;
        EEPROM.write(306,pump1.Thursday); // save to memory
        myFiles.loadBitmap(84, 250, 30, 30, schedActiveB[pump1.Thursday]); 
      }
      else if (scheduleItem==2) // for Micros
      {
        if (pump2.Thursday==1) pump2.Thursday=0;
        else pump2.Thursday=1;
        EEPROM.write(315,pump2.Thursday); // save to memory
        myFiles.loadBitmap(84, 250, 30, 30, schedActiveB[pump2.Thursday]); 
      }
    }
    else if ((x>=132)&&(x<=162)&&(y>=250)&&(y<=280)) // dose on Friday
    {
      if (scheduleItem==1) // for Macros
      {
        if (pump1.Friday==1) pump1.Friday=0;
        else pump1.Friday=1;
        EEPROM.write(307,pump1.Friday); // save to memory
        myFiles.loadBitmap(132, 250, 30, 30, schedActiveB[pump1.Friday]); 
      }
      else if (scheduleItem==2) // for Micros
      {
        if (pump2.Friday==1) pump2.Friday=0;
        else pump2.Friday=1;
        EEPROM.write(316,pump2.Friday); // save to memory
        myFiles.loadBitmap(132, 250, 30, 30, schedActiveB[pump2.Friday]); 
      }
    }
    else if ((x>=186)&&(x<=216)&&(y>=250)&&(y<=280)) // dose on Saturday
    {
      if (scheduleItem==1) // for Macros
      {
        if (pump1.Saturday==1) pump1.Saturday=0;
        else pump1.Saturday=1;
        EEPROM.write(308,pump1.Saturday); // save to memory
        myFiles.loadBitmap(186, 250, 30, 30, schedActiveB[pump1.Saturday]); 
      }
      else if (scheduleItem==2); // for Micros
      {
        if (pump2.Saturday==1) pump2.Saturday=0;
        else pump2.Saturday=1;
        EEPROM.write(317,pump2.Saturday); // save to memory
        myFiles.loadBitmap(186, 250, 30, 30, schedActiveB[pump2.Saturday]); 
      }
    }
    break;

  case 16:  // screen brightness settings 

    myGLCD.setFont(arial_bold);

    // byte screenRetHome, screenDimLevel, screenDimSec, screenBrightMem;

    if ((x>=107)&&(x<=129)&&(y>=294)&&(y<=318))  // settings button  
    {     
      screenSettings();
    }  

    else if ((x>=145)&&(x<=169)&&(y>=42)&&(y<=66))   // return home min up
    {
      if (screenRetHome<255)
      {
        screenRetHome=screenRetHome+1;  // increment up by one
        itoa(screenRetHome, char3, 10);
        if (screenRetHome>=0 && screenRetHome<=9) // add a zero
        {
          myGLCD.setColor(0, 0, 0);
          myGLCD.fillRect(191,59,239,75); // clear previous value
        }
        myGLCD.setColor(255, 77, 0);
        myGLCD.print(char3, 191, 59); 
        EEPROM.write(28,screenRetHome);  // save to memory
      }
    }
    else if ((x>=145)&&(x<=169)&&(y>=72)&&(y<=96))    // return home min down
    {     
      if (screenRetHome>0)
      {
        screenRetHome=screenRetHome-1;  // increment down by one
        itoa(screenRetHome, char3, 10);
        myGLCD.setColor(255, 77, 0);
        if (screenRetHome>=0 && screenRetHome<=9) // add a zero
        {
          myGLCD.setColor(0, 0, 0);
          myGLCD.fillRect(191,59,239,75); // clear previous value
        }
        myGLCD.setColor(255, 77, 0);
        myGLCD.print(char3, 191, 59); 
        EEPROM.write(28,screenRetHome);  // save to memory
      }
    }
    else if ((x>=145)&&(x<=169)&&(y>=110)&&(y<=134))   // auto-dim level up by 1
    {
      if (screenDimLevel<5)
      {
        screenDimLevel=(screenDimLevel+1);  // increment up by 1
        EEPROM.write(29,screenDimLevel); // save new value
        itoa(screenDimLevel, char3, 10);
        myGLCD.setColor(255, 77, 0);
        myGLCD.print(char3, 191, 127);
      }
    }
    else if ((x>=145)&&(x<=169)&&(y>=140)&&(y<=164))   // auto-dim level down by 1
    {     
      if (screenDimLevel>0)
      {
        screenDimLevel=(screenDimLevel-1);  // increment up by 1
        EEPROM.write(29,screenDimLevel); // save new value
        itoa(screenDimLevel, char3, 10);
        myGLCD.setColor(255, 77, 0);
        myGLCD.print(char3, 191, 127);
      }
    }
    else if ((x>=145)&&(x<=169)&&(y>=178)&&(y<=202))   // dim seconds up by 1
    {     
      if (screenDimSec<255)
      {
        screenDimSec=(screenDimSec+1);  // increment up by 1
        EEPROM.write(30,screenDimSec); // save new value
        itoa(screenDimSec, char4, 10);
        myGLCD.setColor(255, 77, 0);
        myGLCD.print(char4, 191, 197);
      }
    }
    else if ((x>=145)&&(x<=169)&&(y>=178)&&(y<=232))   // dim seconds down by one
    {     
      if (screenDimSec>0)
      {
        screenDimSec=(screenDimSec-1);  // increment up by 1
        EEPROM.write(30,screenDimSec); // save new value
        itoa(screenDimSec, char4, 10);
        myGLCD.setColor(255, 77, 0);
        myGLCD.print(char4, 191, 197);
      }
    }
    else if ((x>=145)&&(x<=169)&&(y>=245)&&(y<=269))   // brightness up by 1
    {     
      if (screenBrightMem<255)
      {
        screenBrightMem=(screenBrightMem+1);  // increment up by 1
        EEPROM.write(31,screenBrightMem); // save new value
        itoa(screenBrightMem, char4, 10);
        myGLCD.setColor(255, 77, 0);
        myGLCD.print(char4, 187, 264);
      }
    }
    else if ((x>=145)&&(x<=169)&&(y>=275)&&(y<=299))   // brightness down by one
    {     
      if (screenBrightMem>2)
      {
        screenBrightMem=(screenBrightMem-1);  // increment up by 1
        EEPROM.write(31,screenBrightMem); // save new value
        itoa(screenBrightMem, char4, 10);
        myGLCD.setColor(255, 77, 0);
        myGLCD.print(char4, 187, 264);
      }
    }
    break;
  }
}



void checkTemp() // called from the main loop to update temperature reading
{    
  sensorW.requestTemperatures();  // get temp from sensor
  tempC = sensorW.getTempC(waterSensor);  //read water temperature
  tempC = ((tempC*1.8)+32);  //convert to F
  dtostrf(tempC, 4, 1, tempstring);  //convert to string
  int intTempC=tempC; // create an integer var of the temp
  int eepromTemp;

  if (heaterWarning==false) // with no warning active, check temp against temp warning value
  {
    if (intTempC>=heatOffTemp)  // turn off heater if over threshold
    {
      heaterWarning=true; // turn on warning
      heaterWarningCleared=false;  // warning is obviously not cleared yet
      AlarmPwrHeat_Off(); // turn off heater power

      // change to red temp graphics on home screen
      myFiles.loadBitmap(202, 66, 14, 12, "f_R.raw");
      myFiles.loadBitmap(50, 44, 60, 51, "1thermR.raw");

      // this shows the power relay status on the home screen if the heater is powered off
      myFiles.loadBitmap(178, 177, 24, 24, pwrHeatIconS[globalPower.pwrHeat]);
    }
  }
  else if (heaterWarning==true) // if there was a heat warning, we watch for a low temp to turn heat back on
  {
    if (intTempC<=heatOnTemp) // turn heater back on when temp drops below setting
    {
      heaterWarning=false; // turn off the warning, but do not mark it as cleared so the icon stays red 
      AlarmPwrHeat_On(); // turn heater back on

        // this shows the power relay status on the home screen if the heater is powered off
      myFiles.loadBitmap(178, 177, 24, 24, pwrHeatIconS[globalPower.pwrHeat]);
    }
  }

  if (tempC<=coldWarnTemp) // see if temp is under ALARM threshold
  {
    // NEED TO ACTIVATE AN ALARM TONE HERE
    //tone(alarmPin, frequency, 500);
    // http://arduino.cc/en/Tutorial/tone
  }

  // set temp color based on alarms on home screen
  if (heaterWarningCleared==true) myGLCD.setColor(240, 240, 255);  // if no warning is active, or it has been acknowledged
  else if (heaterWarningCleared==false) myGLCD.setColor(222, 8, 51);  // if warning is active and hasn't been acknowledged

  //draw temp to screen
  myGLCD.setFont(arial_bold);
  myGLCD.print(tempstring, 132, 64);

}

void checkFeeding() // called from the main loop to update feeding info on home screen
{

  myGLCD.setFont(arial_bold);
  myGLCD.setColor(255, 255, 255);

  char char3[3]; // used to convert int to char
  int fx; // x value used to center the hours/mins

  // if feeding isn't active, display the elapsed hours since last feeding
  if (feedingActive==false)
  {
    time_t feedHours=calcFeeding(); // calculate how many hours since we've fed the fish

    // print the feeding hours to screen
    itoa(feedHours, char3, 10);
    if (feedHours<10) fx=20; // shift x position to center
    else if (feedHours>9) fx=12; // shift x position to center
    myGLCD.print(char3, fx, 130);
    myGLCD.print("HR", 12, 146);
  }

  // if feeding is active, we display the amount of time left in the feeding cycle
  else if (feedingActive==true)
  {
    // update millis to track feeding time
    unsigned long nowMillis = millis();
    unsigned long feedingTotalMillis = feedingMins;

    // calculate how many minutes are left from millis to display on the screen
    feedingTotalMillis=(feedingTotalMillis*1000*60);
    int feedingMinsLeft=((feedingTotalMillis-(nowMillis-feedingMillis))/1000)/60;
    feedingMinsLeft=feedingMinsLeft+1; // make it 1 minute more, then use < sign for more intuitive display

    // convert to chars to write to screen
    itoa(feedingMinsLeft, char3, 10);
    if (feedingMinsLeft<10) fx=28;
    else if (feedingMinsLeft>9) fx=20;

    if (dispScreen==1) // write to home screen
    {
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(0,130,55,145); // clear previous value
      myGLCD.setColor(0, 184, 19);
      myGLCD.print("<", (fx-16), 130);
      myGLCD.print(char3, fx, 130);
      myGLCD.print("MIN", 4, 146);
    }
    else if (dispScreen==2)  // write to feedign screen
    {
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(40,80,87,96); // clear previous value
      myGLCD.setColor(255, 255, 255);
      myGLCD.print("<", 24, 80);
      myGLCD.print(char3, 40, 80);
      myGLCD.print("MINUTES", 88, 80);
    }
  }
}

void checkDosing() // updates dosing info on the home screen
{
  char char3[3];  // used to convert int to char

  // just to note, that since we need to store a number higher than 255 in EEPROM, we have a high and low byte
  // when retrieved, the high byte is a number that is converted to the 10th power
  // then the low byte is added to the high byte to get our stored value
  // this gives us values up to 2559, which would be over 2L of ferts, so we are set

  int vol1_H=EEPROM.read(32); // read the remaining volume for pump 1 (high byte ^10)
  int vol1_L=EEPROM.read(33); // read the remaining volume for pump 1 (low byte)
  int vol2_H=EEPROM.read(34); // read the remaining volume for pump 2 (high byte ^10)
  int vol2_L=EEPROM.read(35); // read the remaining volume for pump 2 (low byte)

  float vol1=((vol1_H*10)+vol1_L); // combine high and low byte to get volume
  float vol2=((vol2_H*10)+vol2_L); // combine high and low byte to get volume

  Serial.print("Vol1 H: ");
  Serial.print(vol1_H);
  Serial.print("\n");
  Serial.print("Vol1 L: ");
  Serial.print(vol1_L);
  Serial.print("\n");
  Serial.print("Vol1: ");
  Serial.print(vol1);
  Serial.print("\n");

  int doseAmt=EEPROM.read(21); // 21 // dose in mL
  int doseCap=EEPROM.read(22); // 22 // dosing reseviors capacity in mL^10;
  doseCap=doseCap*10;  // to power of 10
  int doses1=(vol1/doseAmt);  // cacluate how many Macro doeses are left
  int doses2=(vol2/doseAmt);  // cacluate how many Micro are left

    // here we prepare to draw the fill over the screen icons
  // set initial Y values of an emtpy resevoir
  int y1mac=212;
  int y1mic=212;

  // calculate percentage left and generate the y1 coordinate for drawing the fill levels
  if (doses1>0) y1mac=213-((vol1/doseCap)*50);  // for Macros
  if (doses2>0) y1mic=213-((vol2/doseCap)*50);  // for Micros

  // if the math above throws the pixels outside the tube, set it back to empty
  if ((y1mac > 212)||(y1mac < 163)) y1mac = 212;
  if ((y1mic > 212)||(y1mic < 163)) y1mic = 212;

  // clear previous data on screen with black
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(116,135,165,143); 

  // values to center remaining dose numerical values over tubes on screen
  int xcharMacro=116;
  int xcharMicro=142;
  if (doses1<=9) xcharMacro=120;
  if (doses1<=9) xcharMicro=146;

  // draw remaining dose numerical values
  myGLCD.setFont(Sinclair_S);
  myGLCD.setColor(255, 255, 255);
  itoa(doses1, char3, 10);
  myGLCD.print(char3, xcharMacro, 135);
  itoa(doses2, char3, 10);
  myGLCD.print(char3, xcharMicro, 135);

  // draw empty fert tubes
  myFiles.loadBitmap(112, 155, 23, 65, "1ferts.raw");
  myFiles.loadBitmap(138, 155, 23, 65, "1ferts.raw");

  //draw fert fill
  myGLCD.setColor(34, 81, 255);  // blue for macros
  myGLCD.fillRect(117,y1mac,129,213);
  myGLCD.setColor(255, 77, 0);  // orange for micros
  myGLCD.fillRect(143,y1mic,155,213);

}

void checkLighting()
{
  // display lighting info
  // clear old data

  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(57,111,104,236);

  if (currentLightMode==4) // active transition
  {
    myFiles.loadBitmap(67, 118, 28, 28, lightModeSm[fadeFromMode]);
    myFiles.loadBitmap(75, 151, 12, 20, "1arrow.raw");
    myFiles.loadBitmap(67, 174, 28, 28, lightModeSm[fadeToMode]);

    // Get the time in seconds (since 1970)
    unsigned long rightNow = now();

    // calculate how much time is left in the fade in minutes
    unsigned long timeLeft=((fadeStartingSeconds + fadeDurationSeconds)-rightNow)/60;

    // convert to char
    int minsLeft=timeLeft;
    char char4[4];
    itoa(minsLeft, char4, 10);

    // center the minutes left
    int xmin=0;
    if (minsLeft<10) xmin=77;
    if ((minsLeft>9)&&(minsLeft<100)) xmin=73;
    if (minsLeft>99) xmin=69;

    // clear old data
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRect(69,207,93,215);

    // print to screen
    myGLCD.setFont(Sinclair_S);
    myGLCD.setColor(255, 255, 255);
    myGLCD.print(char4, xmin, 207);
    myGLCD.print("MIN", 69, 219);
  }
  // if we are not fading, then print the current mode
  else if (currentLightMode==5) 
  {
    myFiles.loadBitmap(57, 140, 48, 48, "1quest.raw");
  }
  else
  {  
    myFiles.loadBitmap(57, 140, 48, 48, lightMode[currentLightMode]);
  }
}


void clearSelectedLight(int cmd) // this is called by the light mode editing screen and it touch events
{
  if (cmd==1) // switch off the selected icon of an individual light button
  {
    if (currentLightMode==0)
    {
      myFiles.loadBitmap(10, 39, 48, 48, lightMode[currentLightMode]);
    }
    else if (currentLightMode==1)
    {
      myFiles.loadBitmap(67, 39, 48, 48, lightMode[currentLightMode]);
    }
    else if (currentLightMode==2)
    {
      myFiles.loadBitmap(124, 39, 48, 48, lightMode[currentLightMode]);
    }
    else if (currentLightMode==3)
    {
      myFiles.loadBitmap(181, 39, 48, 48, lightMode[currentLightMode]);
    }
  }
  else if (cmd==2) // disable all light modes that aren't active
  {
    if (currentLightMode==0)
    {
      // disable all other mode buttons
      myFiles.loadBitmap(67, 39, 48, 48, lightModeF[0]);
      myFiles.loadBitmap(124, 39, 48, 48, lightModeF[1]);
      myFiles.loadBitmap(181, 39, 48, 48, lightModeF[2]);
    }
    else if (currentLightMode==1)
    {
      // disable all other mode buttons
      myFiles.loadBitmap(10, 39, 48, 48, lightModeF[0]);
      myFiles.loadBitmap(124, 39, 48, 48, lightModeF[1]);
      myFiles.loadBitmap(181, 39, 48, 48, lightModeF[2]);
    }
    else if (currentLightMode==2)
    {
      // disable all other mode buttons
      myFiles.loadBitmap(10, 39, 48, 48, lightModeF[0]);
      myFiles.loadBitmap(67, 39, 48, 48, lightModeF[0]);
      myFiles.loadBitmap(181, 39, 48, 48, lightModeF[2]);
    }
    else if (currentLightMode==3)
    {
      // disable all other mode buttons
      myFiles.loadBitmap(10, 39, 48, 48, lightModeF[0]);
      myFiles.loadBitmap(67, 39, 48, 48, lightModeF[0]);
      myFiles.loadBitmap(124, 39, 48, 48, lightModeF[1]);
    }
  }
}



void resyncLights() 
{

  // this routine forces the light into a factory preset mode and reads in the stored M1-M4 light modes
  if (lightCSP==true)
  {
    irsend.sendNEC(FULLYELLOW,32);
    currentColor.Red = 42;
    currentColor.Green = 42;
    currentColor.Blue = 42;
    currentColor.White = 42;
  }
  else if (lightCSP==false) 
  {
    analogWrite(ledRedPin, maxRGBW);
    analogWrite(ledGreenPin, maxRGBW);
    analogWrite(ledBluePin, maxRGBW);
    analogWrite(ledWhitePin, maxRGBW);
    currentColor.Red = maxRGBW;
    currentColor.Green = maxRGBW;
    currentColor.Blue = maxRGBW;
    currentColor.White = maxRGBW;
  }

  targetColor.Red = 0;
  targetColor.Green = 0;
  targetColor.Blue = 0;
  targetColor.White = 0;
  lastColor.Red = 0;
  lastColor.Blue = 0;
  lastColor.Green = 0;
  lastColor.White = 0;

  currentLightMode=5;

  // this code only runs the very first time you use the code, you can comment it out after first run

  /*
  if (EEPROM.read(199)!=1) 
   {
   EEPROM.write(199,1); // save to eeprom to mark that this has been done
   EEPROM.write(200,42);
   EEPROM.write(201,42);
   EEPROM.write(202,42);
   EEPROM.write(203,42);
   EEPROM.write(204,42);
   EEPROM.write(205,42);
   EEPROM.write(206,42);
   EEPROM.write(207,42);
   EEPROM.write(208,42);
   EEPROM.write(209,42);
   EEPROM.write(210,42);
   EEPROM.write(211,42);
   EEPROM.write(212,42);
   EEPROM.write(213,42);
   EEPROM.write(214,42);
   EEPROM.write(215,42);
   EEPROM.write(216,0);
   EEPROM.write(217,0);
   EEPROM.write(218,0);
   EEPROM.write(219,0);
   EEPROM.write(220,0);
   EEPROM.write(221,0);
   EEPROM.write(222,0);
   EEPROM.write(223,0);
   EEPROM.write(224,0);
   EEPROM.write(225,0);
   EEPROM.write(226,0);
   EEPROM.write(227,0);
   EEPROM.write(228,0);
   EEPROM.write(229,0);
   EEPROM.write(230,0);
   EEPROM.write(231,0);
   EEPROM.write(232,0);
   EEPROM.write(233,0);
   EEPROM.write(234,0);
   EEPROM.write(235,0);
   EEPROM.write(236,0);
   EEPROM.write(237,0);
   EEPROM.write(238,0);
   EEPROM.write(239,0);
   EEPROM.write(240,0);
   EEPROM.write(241,0);
   EEPROM.write(242,0);
   EEPROM.write(243,0);
   }
   */

  lightHighSun.Red=EEPROM.read(200);
  lightHighSun.Green=EEPROM.read(201);
  lightHighSun.Blue=EEPROM.read(202);
  lightHighSun.White=EEPROM.read(203);

  lightMidSun.Red=EEPROM.read(204);
  lightMidSun.Green=EEPROM.read(205);
  lightMidSun.Blue=EEPROM.read(206);
  lightMidSun.White=EEPROM.read(207);

  lightLowSun.Red=EEPROM.read(208);
  lightLowSun.Green=EEPROM.read(209);
  lightLowSun.Blue=EEPROM.read(210);
  lightLowSun.White=EEPROM.read(211);

  lightMoon.Red=EEPROM.read(212);
  lightMoon.Green=EEPROM.read(213);
  lightMoon.Blue=EEPROM.read(214);
  lightMoon.White=EEPROM.read(215);  

}


void feedingStop()
{

  // return all power outputs to previous settings
  if (preFeedPower.pwrLight1==1) AlarmPwrLight1_On();
  else if (preFeedPower.pwrLight1==0) AlarmPwrLight1_Off();
  if (preFeedPower.pwrLight2==1) AlarmPwrLight2_On();
  else if (preFeedPower.pwrLight2==0) AlarmPwrLight2_Off();
  if (preFeedPower.pwrFilter==1) AlarmPwrFilter_On();
  else if (preFeedPower.pwrFilter==0) AlarmPwrFilter_Off();
  if (preFeedPower.pwrCirc==1) AlarmPwrCirc_On();
  else if (preFeedPower.pwrCirc==0) AlarmPwrCirc_Off();
  if (preFeedPower.pwrHeat==1) AlarmPwrHeat_On();
  else if (preFeedPower.pwrHeat==0) AlarmPwrHeat_Off();
  if (preFeedPower.pwrCO2==1) AlarmPwrCO2_On();
  else if (preFeedPower.pwrCO2==0) AlarmPwrCO2_Off();
  if (preFeedPower.pwrAux1==1) AlarmPwrAux1_On();
  else if (preFeedPower.pwrAux1==0) AlarmPwrAux1_Off();
  if (preFeedPower.pwrAux2==1) AlarmPwrAux2_On();
  else if (preFeedPower.pwrAux2==1) AlarmPwrAux2_Off();

  feedingActive=false; // stop feeding cycle
  feedingMillis=0; // reset feeding millis
  screenHome(); // refresh the home screen
}


void SaveTime()
{        
  //RTC.stopClock();     
  saveRTC.tYear=(saveRTC.tYear+2000);
  //RTC.fillByYMD(saveRTC.tYear, saveRTC.tMonth, saveRTC.tDay);
  //RTC.fillByHMS(saveRTC.tHour, saveRTC.tMinute, saveRTC.tSecond);
  //RTC.setTime();
  //delay(10);
  RTC.adjust(DateTime(saveRTC.tYear, saveRTC.tMonth, saveRTC.tDay, saveRTC.tHour, saveRTC.tMinute, saveRTC.tSecond));
  //RTC.startClock();
  //delay(10);
  //RTC.now().unixtime(); 
  updateTimeDate();
}

void updateTimeDate()
{

  //RTC.getTime();
  RTC.now();

  // draw date and time
  myGLCD.setColor(240, 240, 255);
  myGLCD.setFont(Sinclair_S);
  /*
   if ((RTC.hour!=prevRTC.tHour) || (RTC.minute!=prevRTC.tMinute) || updateTime) {    //time
   prevRTC.tHour = RTC.hour;
   prevRTC.tMinute = RTC.minute;
   int hr_12;
   byte ampm;
   hr_12=prevRTC.tHour%12;
   if (hr_12==0) hr_12=12;
   if (RTC.hour<=11) ampm=0;
   else ampm=1;
   printTime(hr_12, RTC.minute, ampm, 180, 2);
   
   }*/

  if ((hour()!=prevRTC.tHour) || (minute()!=prevRTC.tMinute) || updateTime) {    //time
    prevRTC.tHour = hour();
    prevRTC.tMinute = minute();
    int hr_12;
    byte ampm;
    hr_12=prevRTC.tHour%12;
    if (hr_12==0) hr_12=12;
    if (hour()<=11) ampm=0;
    else ampm=1;
    printTime(hr_12, minute(), ampm, 180, 2);
  }
  /*
   if ((RTC.day!=prevRTC.tDay) || (RTC.month!=prevRTC.tMonth) || (RTC.year!=prevRTC.tYear)  || updateTime) {     //date
   prevRTC.tDay = RTC.day;
   prevRTC.tMonth = RTC.month;
   printDate(40, 2);      
   */
  if ((day()!=prevRTC.tDay) || (month()!=prevRTC.tMonth) || (year()!=prevRTC.tYear)  || updateTime) {     //date
    prevRTC.tDay = day();
    prevRTC.tMonth = month();
    printDate(40, 2);             
  }

}

void printTime(int thour, int tminute, byte ampm, int posx, int posy)
{
  char tmpTime[8], charT[3];

  tmpTime[0] = '\0';

  if (thour>=0 && thour<=9) {          //add space
    strcat(tmpTime, " ");
    itoa(thour, charT, 10);
    strcat(tmpTime, charT);
  }
  else 
    itoa(thour, tmpTime, 10);

  strcat(tmpTime, ":");  

  if (tminute>=0 && tminute<=9) {         //add 0
    strcat(tmpTime, "0");
    itoa(tminute, charT, 10);
    strcat(tmpTime, charT);
  }
  else {
    itoa(tminute, charT, 10);
    strcat(tmpTime, charT);
  }
  if (ampm==0) strcat(tmpTime, "am");
  else strcat(tmpTime, "pm");

  myGLCD.print(tmpTime, posx, posy);           // Display time  
}

void printDate(int x, int y) 
{
  char  chDate[25], tmpChar[5];

  strcat(chDate, "     ");
  chDate[0] = '\0';
  //strcat(chDate, Day[RTC.dow]);
  strcat(chDate, Day[weekday()]);
  strcat(chDate, ", ");
  //strcat(chDate, Mon[RTC.month]);
  strcat(chDate, Mon[month()]);
  strcat(chDate, " ");
  //  itoa(RTC.day, tmpChar, 10);
  itoa(day(), tmpChar, 10);
  strcat(chDate, tmpChar);
  // this line is for omitting year
  strcat(chDate, "  ");

  //strcat(chDate, " ");
  //itoa(RTC.year, tmpChar, 10);
  //strcat(chDate, tmpChar);
  //strcat(chDate, "   ");

  myGLCD.print(chDate, x, y);            //Display date 
}

time_t tmConvert_t(int YYYY, byte MM, byte DD, byte hh, byte mm, byte ss)
{
  tmElements_t tmSet;
  tmSet.Year = YYYY - 1970;
  tmSet.Month = MM;
  tmSet.Day = DD;
  tmSet.Hour = hh;
  tmSet.Minute = mm;
  tmSet.Second = ss;
  return makeTime(tmSet);         //convert to time_t
}

void autoBrightness()
{

  //Serial.print("Sensor: ");
  int photocellReading = analogRead(lightSensorPin); 

  // map the photo sensor value of 0-1023 directly to the PWM output of 0-255
  int brightnessLevel=map(photocellReading, 0, 1023, 0, 255);

  // apply a brightness bump or drop based on the user preference autodim setting
  int brightAdjust;
  if (screenDimLevel==1) brightAdjust=-20;
  else if (screenDimLevel==2) brightAdjust=-10;
  else if (screenDimLevel==3) brightAdjust=0;
  else if (screenDimLevel==4) brightAdjust=10;
  else if (screenDimLevel==5) brightAdjust=20;

  brightnessLevel=brightnessLevel+brightAdjust; // apply brightness adjustment
  if (brightnessLevel>255) brightnessLevel=255; // can't get brighter than 255
  if (brightnessLevel<2) brightnessLevel=1; // must be at least 1 or it's off

  int diff=brightnessLevel-backLight;
  if (diff < 0)
  {
    diff = ((diff - diff) - diff);
  }
  if (diff > 25) // we only adjsut brightness if it's more than a 10% change from where we are now
  {
    rampScreenBrightness(backLight, brightnessLevel);
  }
  //Serial.print(brightnessLevel);
  //Serial.print("\n");
}

// smoothly ramps the screen brightness from one level to another
void rampScreenBrightness(byte fromLevel, byte toLevel)
{
  if (fromLevel<toLevel)
  {
    while (fromLevel<toLevel) 
    {
      analogWrite(screenBrightPin, fromLevel);
      fromLevel++;
      delay(5);
    }
  }
  else if (fromLevel>toLevel)
  {
    while (fromLevel>toLevel) 
    {
      analogWrite(screenBrightPin, fromLevel);  
      fromLevel--;
      delay(5);
    }
  }
  backLight=toLevel;
}

time_t calcFeeding()
{
  // need to retrieve and calculate last feeding time

  if (EEPROM.read(0)==1)
  {
    //RTC.getTime();
    RTC.now();
    //  time_t currentStamp = tmConvert_t(RTC.year,RTC.month,RTC.day,RTC.hour,RTC.minute,RTC.second);
    time_t currentStamp = tmConvert_t(year(),month(),day(),hour(),minute(),second());

    // 0 // last feeding data saved (0 for no, 1 for yes)
    // 1 // last feeding minute
    // 2 // last feeding hour
    // 3 // last feeding day
    // 4 // last feeding month
    // 5 // last feeding year
    byte lastSec = 0;
    byte lastMin = EEPROM.read(1);
    byte lastHr = EEPROM.read(2);
    byte lastDay = EEPROM.read(3);
    byte lastMon = EEPROM.read(4);
    int lastYear = (EEPROM.read(5)+2000);

    time_t lastStamp = tmConvert_t(lastYear,lastMon,lastDay,lastHr,lastMin,lastSec);
    float elapsedSec = (currentStamp-lastStamp);
    int elapsedHr = elapsedSec/60/60;
    return elapsedHr;
  }
  else if (EEPROM.read(0)==0)
  {
    return 0;
  }
}

void readPowerSchedule()
{

  // load power schedule from eeprom
  schedLights1.active = EEPROM.read(100);
  schedLights1.onHour = EEPROM.read(101);
  schedLights1.onMinute = EEPROM.read(102);
  schedLights1.offHour = EEPROM.read(103);
  schedLights1.offMinute = EEPROM.read(104);
  schedLights2.active = EEPROM.read(105);
  schedLights2.onHour = EEPROM.read(106);
  schedLights2.onMinute = EEPROM.read(107);
  schedLights2.offHour = EEPROM.read(108);
  schedLights2.offMinute = EEPROM.read(109);
  schedCirc.active = EEPROM.read(110);
  schedCirc.onHour = EEPROM.read(111);
  schedCirc.onMinute = EEPROM.read(112);
  schedCirc.offHour = EEPROM.read(113);
  schedCirc.offMinute = EEPROM.read(114);
  schedCo2.active = EEPROM.read(115);
  schedCo2.onHour = EEPROM.read(116);
  schedCo2.onMinute = EEPROM.read(117);
  schedCo2.offHour = EEPROM.read(118);
  schedCo2.offMinute = EEPROM.read(119);
  schedAux1.active = EEPROM.read(120);
  schedAux1.onHour = EEPROM.read(121);
  schedAux1.onMinute = EEPROM.read(122);
  schedAux1.offHour = EEPROM.read(123);
  schedAux1.offMinute = EEPROM.read(124);
  schedAux2.active = EEPROM.read(125);
  schedAux2.onHour = EEPROM.read(126);
  schedAux2.onMinute = EEPROM.read(127);
  schedAux2.offHour = EEPROM.read(128);
  schedAux2.offMinute = EEPROM.read(129);

}

void readRampSchedule()
{
  // load lighting ramp schedule from eeprom
  ramp1.onHour = EEPROM.read(220);
  ramp1.onMinute = EEPROM.read(221);
  ramp1.offHour = EEPROM.read(222);
  ramp1.offMinute = EEPROM.read(223);

  ramp2.onHour = EEPROM.read(224);
  ramp2.onMinute = EEPROM.read(225);
  ramp2.offHour = EEPROM.read(226);
  ramp2.offMinute = EEPROM.read(227);

  ramp3.onHour = EEPROM.read(228);
  ramp3.onMinute = EEPROM.read(229);
  ramp3.offHour = EEPROM.read(230);
  ramp3.offMinute = EEPROM.read(231);

  ramp4.onHour = EEPROM.read(232);
  ramp4.onMinute = EEPROM.read(233);
  ramp4.offHour = EEPROM.read(234);
  ramp4.offMinute = EEPROM.read(235);

  ramp5.onHour = EEPROM.read(236);
  ramp5.onMinute = EEPROM.read(237);
  ramp5.offHour = EEPROM.read(238);
  ramp5.offMinute = EEPROM.read(239);

  ramp6.onHour = EEPROM.read(240);
  ramp6.onMinute = EEPROM.read(241);
  ramp6.offHour = EEPROM.read(242);
  ramp6.offMinute = EEPROM.read(243);
}

void readDosingSchedule()
{

  // load dosing schedule from eeprom
  pump1.onHour = EEPROM.read(300);
  pump1.onMinute = EEPROM.read(301);

  pump1.Sunday = EEPROM.read(302);
  pump1.Monday = EEPROM.read(303);
  pump1.Tuesday = EEPROM.read(304);
  pump1.Wednesday = EEPROM.read(305);
  pump1.Thursday = EEPROM.read(306);
  pump1.Friday = EEPROM.read(307);
  pump1.Saturday = EEPROM.read(308);

  pump2.onHour = EEPROM.read(309);
  pump2.onMinute = EEPROM.read(310);

  pump2.Sunday = EEPROM.read(311);
  pump2.Monday = EEPROM.read(312);
  pump2.Tuesday = EEPROM.read(313);
  pump2.Wednesday = EEPROM.read(314);
  pump2.Thursday = EEPROM.read(315);
  pump2.Friday = EEPROM.read(316);
  pump2.Saturday = EEPROM.read(317);

}  

// Giving some big credit here, my original version of the smart startup had several bugs with schedules wrapping past midnight.
// This new smart startup routine was reworked and fixed by robsworld78 at The Planted Tank forums
void smartStartup()
{  
  RTC.now();

  // get the current time in UNIX time, which is FAR easier for time calculations and comparisons
  // also must set UTC offset for mktime to calcuate stuff correct
  time_t currentStamp = tmConvert_t(year(),month(),day(),hour(),minute(),second());

  time_t midnight = tmConvert_t(year(),month(),day(),23,59,59); // current day 11:59:59pm
  time_t powerOn;
  time_t powerOff;
  time_t powerOffNextDay;

  //// POWER STARTUP:  LIGHTS 1
  // read in the power schedule and see if we are at a time when it should be on
  if (schedLights1.active == 1) 
  {  
    time_t powerOn = tmConvert_t(year(),month(),day(),schedLights1.onHour,schedLights1.onMinute,0); // get a unix time stamp for the power on
    time_t powerOff = tmConvert_t(year(),month(),day(),schedLights1.offHour,schedLights1.offMinute,0); // get a unix time stamp for the power off  
    time_t powerOffNextDay = tmConvert_t(year(),month(),day(),schedLights1.offHour,schedLights1.offMinute,0);   
    powerOffNextDay = powerOffNextDay+86400;

    if (powerOn==powerOff) AlarmPwrLight1_On();
    else if (powerOn>powerOff) 
    {
      if ((currentStamp>=powerOn) && (powerOffNextDay>=midnight)) AlarmPwrLight1_On();
      else if (((currentStamp<=powerOn) && (currentStamp<=powerOff) && (powerOffNextDay>=midnight))) AlarmPwrLight1_On();
      else AlarmPwrLight1_Off();
    }
    else if ((currentStamp>=powerOn) && (currentStamp<=powerOff)) AlarmPwrLight1_On();
    else  AlarmPwrLight1_Off();
  }

  //// POWER STARTUP:  LIGHTS 2
  // read in the power schedule and see if we are at a time when it should be on
  if (schedLights2.active == 1) 
  {  
    powerOn = tmConvert_t(year(),month(),day(),schedLights2.onHour,schedLights2.onMinute,0); // get a unix time stamp for the power on
    powerOff = tmConvert_t(year(),month(),day(),schedLights2.offHour,schedLights2.offMinute,0); // get a unix time stamp for the power off  
    powerOffNextDay = tmConvert_t(year(),month(),day(),schedLights2.offHour,schedLights2.offMinute,0); // get a unix time stamp for the power off  
    powerOffNextDay = powerOffNextDay+86400;

    if (powerOn==powerOff) AlarmPwrLight2_On();
    else if (powerOn>powerOff) 
    {
      if ((currentStamp>=powerOn) && (powerOffNextDay>=midnight)) AlarmPwrLight2_On();
      else if (((currentStamp<=powerOn) && (currentStamp<=powerOff) && (powerOffNextDay>=midnight))) AlarmPwrLight2_On();
      else AlarmPwrLight2_Off();
    }
    else if ((currentStamp>=powerOn) && (currentStamp<=powerOff)) AlarmPwrLight2_On();
    else AlarmPwrLight2_Off();
  }

  //// POWER STARTUP:  FILTER  
  AlarmPwrFilter_On(); // the filter is ALWAYS ON

  //// POWER STARTUP:  CIRCULATION PUMP
  // read in the power schedule and see if we are at a time when it should be on
  if (schedCirc.active == 1) 
  {
    powerOn = tmConvert_t(year(),month(),day(),schedCirc.onHour,schedCirc.onMinute,0); // get a unix time stamp for the power on
    powerOff = tmConvert_t(year(),month(),day(),schedCirc.offHour,schedCirc.offMinute,0); // get a unix time stamp for the power off  
    powerOffNextDay = tmConvert_t(year(),month(),day(),schedCirc.offHour,schedCirc.offMinute,0); // get a unix time stamp for the power off  
    powerOffNextDay=powerOffNextDay+86400;

    if (powerOn==powerOff) AlarmPwrCirc_On();
    else if (powerOn>powerOff) 
    {
      if ((currentStamp>=powerOn) && (powerOffNextDay>=midnight)) AlarmPwrCirc_On();
      else if (((currentStamp<=powerOn) && (currentStamp<=powerOff) && (powerOffNextDay>=midnight))) AlarmPwrCirc_On();
      else AlarmPwrCirc_Off();
    }
    else if ((currentStamp>=powerOn) && (currentStamp<=powerOff)) AlarmPwrCirc_On();
    else AlarmPwrCirc_Off();
  }    

  //// POWER STARTUP:  HEATER  
  AlarmPwrHeat_On(); // the heater is ALWAYS ON

  //// POWER STARTUP:  CO2
  // read in the power schedule and see if we are at a time when it should be on
  if (schedCo2.active == 1) 
  {
    powerOn = tmConvert_t(year(),month(),day(),schedCo2.onHour,schedCo2.onMinute,0); // get a unix time stamp for the power on
    powerOff = tmConvert_t(year(),month(),day(),schedCo2.offHour,schedCo2.offMinute,0); // get a unix time stamp for the power off  
    powerOffNextDay = tmConvert_t(year(),month(),day(),schedCo2.offHour,schedCo2.offMinute,0); // get a unix time stamp for the power off  
    powerOffNextDay=powerOffNextDay+86400;

    if (powerOn==powerOff) AlarmPwrCO2_On();
    else if (powerOn>powerOff) 
    {
      if ((currentStamp>=powerOn) && (powerOffNextDay>=midnight)) AlarmPwrCO2_On();
      else if (((currentStamp<=powerOn) && (currentStamp<=powerOff) && (powerOffNextDay>=midnight))) AlarmPwrCO2_On();
      else AlarmPwrCO2_Off();
    }
    else if ((currentStamp>=powerOn) && (currentStamp<=powerOff)) AlarmPwrCO2_On();
    else AlarmPwrCO2_Off();
  }

  //// POWER STARTUP:  AUX 1
  // read in the power schedule and see if we are at a time when it should be on
  if (schedAux1.active == 1)
  {
    powerOn = tmConvert_t(year(),month(),day(),schedAux1.onHour,schedAux1.onMinute,0); // get a unix time stamp for the power on
    powerOff = tmConvert_t(year(),month(),day(),schedAux1.offHour,schedAux1.offMinute,0); // get a unix time stamp for the power off  
    powerOffNextDay = tmConvert_t(year(),month(),day(),schedAux1.offHour,schedAux1.offMinute,0); // get a unix time stamp for the power off  
    powerOffNextDay=powerOffNextDay+86400;

    if (powerOn==powerOff) AlarmPwrAux1_On();
    else if (powerOn>powerOff) 
    {
      if ((currentStamp>=powerOn) && (powerOffNextDay>=midnight)) AlarmPwrAux1_On();
      else if (((currentStamp<=powerOn) && (currentStamp<=powerOff) && (powerOffNextDay>=midnight))) AlarmPwrAux1_On();
      else AlarmPwrAux1_Off();
    }
    else if ((currentStamp>=powerOn) && (currentStamp<=powerOff)) AlarmPwrAux1_On();
    else AlarmPwrAux1_Off();
  }

  //// POWER STARTUP:  AUX 2
  // read in the power schedule and see if we are at a time when it should be on
  if (schedAux2.active == 1)
  {
    powerOn = tmConvert_t(year(),month(),day(),schedAux2.onHour,schedAux2.onMinute,0); // get a unix time stamp for the power on
    powerOff = tmConvert_t(year(),month(),day(),schedAux2.offHour,schedAux2.offMinute,0); // get a unix time stamp for the power off  
    powerOffNextDay = tmConvert_t(year(),month(),day(),schedAux2.offHour,schedAux2.offMinute,0); // get a unix time stamp for the power off  
    powerOffNextDay=powerOffNextDay+86400;

    if (powerOn==powerOff) AlarmPwrAux2_On();
    else if (powerOn>powerOff) 
    {
      if ((currentStamp>=powerOn) && (powerOffNextDay>=midnight)) AlarmPwrAux2_On();
      else if (((currentStamp<=powerOn) && (currentStamp<=powerOff) && (powerOffNextDay>=midnight))) AlarmPwrAux2_On();
      else AlarmPwrAux2_Off();
    }
    else if ((currentStamp>=powerOn) && (currentStamp<=powerOff)) AlarmPwrAux2_On();
    else AlarmPwrAux2_Off();
  }

  // if the light power is on, we can set the correct lighting mode
  // lighting mode is restored to ramp with closest start time after current time
  if (globalPower.pwrLight1==1) // only bother if the lights are on
  {  
    time_t rampStart1 = tmConvert_t(year(),month(),day(),ramp1.onHour,ramp1.onMinute,0);
    time_t rampStart2 = tmConvert_t(year(),month(),day(),ramp2.onHour,ramp2.onMinute,0);
    time_t rampStart3 = tmConvert_t(year(),month(),day(),ramp3.onHour,ramp3.onMinute,0);
    time_t rampStart4 = tmConvert_t(year(),month(),day(),ramp4.onHour,ramp4.onMinute,0);
    time_t rampStart5 = tmConvert_t(year(),month(),day(),ramp5.onHour,ramp5.onMinute,0);
    time_t rampStart6 = tmConvert_t(year(),month(),day(),ramp6.onHour,ramp6.onMinute,0);

    //// Ramp 2 after midnight
    if ((rampStart2<rampStart1) && (currentStamp>rampStart1))
    {    
      irsend.sendNEC(M3,32); // flip to low sun  
      currentLightMode=2;
    }
    else if ((rampStart2<rampStart1) && (currentStamp<rampStart2))
    {      
      irsend.sendNEC(M3,32); // flip to low sun  
      currentLightMode=2;
    }    
    else if (((rampStart2<rampStart1) && (currentStamp>rampStart2) && (currentStamp<rampStart3)))
    {      
      irsend.sendNEC(M2,32); // flip to mid sun
      currentLightMode=1;
    } 
    else if (((rampStart2<rampStart1) && (currentStamp>rampStart3) && (currentStamp<rampStart4)))
    {      
      irsend.sendNEC(M1,32); // flip to high sun
      currentLightMode=0;
    } 
    else if (((rampStart2<rampStart1) && (currentStamp>rampStart4) && (currentStamp<rampStart5)))
    {      
      irsend.sendNEC(M2,32); // flip to mid sun
      currentLightMode=1;
    } 
    else if (((rampStart2<rampStart1) && (currentStamp>rampStart5) && (currentStamp<rampStart6)))
    {      
      irsend.sendNEC(M3,32); // flip to low sun  
      currentLightMode=2;
    }       

    //// Ramp 3 after midnight
    else if (((rampStart2>rampStart1) && (rampStart3<rampStart1) && (currentStamp>rampStart2)))
    {      
      irsend.sendNEC(M2,32); // flip to mid sun
      currentLightMode=1;
    }
    else if (((rampStart2>rampStart1) && (rampStart3<rampStart1) && (currentStamp<rampStart3)))
    {      
      irsend.sendNEC(M2,32); // flip to mid sun
      currentLightMode=1;
    }    
    else if ((((rampStart2>rampStart1) && (rampStart3<rampStart1) && (currentStamp>rampStart3) && (currentStamp<rampStart4))))
    {      
      irsend.sendNEC(M1,32); // flip to high sun
      currentLightMode=0;
    } 
    else if ((((rampStart2>rampStart1) && (rampStart3<rampStart1) && (currentStamp>rampStart4) && (currentStamp<rampStart5))))
    {      
      irsend.sendNEC(M2,32); // flip to mid sun
      currentLightMode=1;
    } 
    else if ((((rampStart2>rampStart1) && (rampStart3<rampStart1) && (currentStamp>rampStart5) && (currentStamp<rampStart6))))
    {      
      irsend.sendNEC(M3,32); // flip to low sun  
      currentLightMode=2;
    }      

    //// Ramp 4 after midnight
    else if ((((rampStart2>rampStart1) && (rampStart3>rampStart1) && (rampStart4<rampStart1) && (currentStamp>rampStart3))))
    {      
      irsend.sendNEC(M1,32); // flip to high sun
      currentLightMode=0;
    }
    else if ((((rampStart2>rampStart1) && (rampStart3>rampStart1) && (rampStart4<rampStart1) && (currentStamp<rampStart4))))
    {      
      irsend.sendNEC(M1,32); // flip to high sun
      currentLightMode=0;
    }    
    else if (((((rampStart2>rampStart1) && (rampStart3>rampStart1) && (rampStart4<rampStart1) && (currentStamp>rampStart4) && (currentStamp<rampStart5)))))
    {      
      irsend.sendNEC(M2,32); // flip to mid sun
      currentLightMode=1;
    } 
    else if (((((rampStart2>rampStart1) && (rampStart3>rampStart1) && (rampStart4<rampStart1) && (currentStamp>rampStart5) && (currentStamp<rampStart6)))))
    {      
      irsend.sendNEC(M3,32); // flip to low sun  
      currentLightMode=2;
    }  

    //// Ramp 5 after midnight
    else if (((((rampStart2>rampStart1) && (rampStart3>rampStart1) && (rampStart4>rampStart1) && (rampStart5<rampStart1) && (currentStamp>rampStart4)))))  ////winner
    {      
      irsend.sendNEC(M2,32); // flip to mid sun
      currentLightMode=1;
    }
    else if (((((rampStart2>rampStart1) && (rampStart3>rampStart1) && (rampStart4>rampStart1) && (rampStart5<rampStart1) && (currentStamp<rampStart5))))) /// after midnight
    {      
      irsend.sendNEC(M2,32); // flip to mid sun
      currentLightMode=1;
    }    
    else if ((((((rampStart2>rampStart1) && (rampStart3>rampStart1) && (rampStart4>rampStart1) && (rampStart5<rampStart1) && (currentStamp>rampStart5) && (currentStamp<rampStart6))))))
    {      
      irsend.sendNEC(M3,32); // flip to low sun  
      currentLightMode=2;
    }

    //// Ramp 6 after midnight  
    else if ((((((rampStart2>rampStart1) && (rampStart3>rampStart1) && (rampStart4>rampStart1) && (rampStart5>rampStart1) && (rampStart6<rampStart1) && (currentStamp>rampStart5))))))
    {      
      irsend.sendNEC(M3,32); // flip to low sun  
      currentLightMode=2;
    }
    else if ((((((rampStart2>rampStart1) && (rampStart3>rampStart1) && (rampStart4>rampStart1) && (rampStart5>rampStart1) && (rampStart6<rampStart1) && (currentStamp<rampStart6))))))
    {      
      irsend.sendNEC(M3,32); // flip to low sun  
      currentLightMode=2;
    }    
    else if (((((((rampStart2>rampStart1) && (rampStart3>rampStart1) && (rampStart4>rampStart1) && (rampStart5>rampStart1) && (rampStart6<rampStart1) && (currentStamp>rampStart6) && (currentStamp<rampStart1)))))))
    {      
      irsend.sendNEC(M4,32); // flip to moonlight
      currentLightMode=3;
    }   

    //// start of regular ramping during 24hr period       
    else if ((currentStamp>rampStart1) && (currentStamp<rampStart2)) // if we are after ramp 1
    {
      irsend.sendNEC(M3,32); // flip to low sun  
      currentLightMode=2;
    }  
    else if ((currentStamp>rampStart2) && (currentStamp<rampStart3)) // if we are not in the previous ramp, but are still before this one
    {
      irsend.sendNEC(M2,32); // flip to mid sun
      currentLightMode=1;
    }
    else if ((currentStamp>rampStart3) && (currentStamp<rampStart4)) // if we are not in the previous ramp, but are still before this one
    {
      irsend.sendNEC(M1,32); // flip to high sun
      currentLightMode=0;
    }
    else if ((currentStamp>rampStart4) && (currentStamp<rampStart5)) // if we are not in the previous ramp, but are still before this one
    {
      irsend.sendNEC(M2,32); // flip to mid sun
      currentLightMode=1;
    }
    else if ((currentStamp>rampStart5) && (currentStamp<rampStart6)) // if we are not in the previous ramp, but are still before this one
    {
      irsend.sendNEC(M3,32); // flip to low sun  
      currentLightMode=2;
    }
    else if ((currentStamp>rampStart6) && (currentStamp<rampStart1+86400)) // if we are not in the previous ramp, but are still before this one
    {
      irsend.sendNEC(M4,32); // flip to moonlight
      currentLightMode=3;
    }
    else if ((currentStamp<midnight) && (currentStamp<rampStart1)) // if we are not in the previous ramp, but are still before this one
    {
      irsend.sendNEC(M4,32); // flip to moonlight
      currentLightMode=3;
    }
  }

  if (lightCSP==false) setStartupPWMLighting(); // if not using IR lighting, then startup lights with PWM
}

void setStartupPWMLighting()
{
  if (currentLightMode==0)
  {
    analogWrite(ledRedPin, lightHighSun.Red);
    analogWrite(ledGreenPin, lightHighSun.Green);
    analogWrite(ledBluePin, lightHighSun.Blue);
    analogWrite(ledWhitePin, lightHighSun.White);
    //    analogWrite(ledVioletPin, lightHighSun.Violet);
    //    analogWrite(ledMoonPin, lightHighSun.Moon);
  }
  else if (currentLightMode==1)
  {
    analogWrite(ledRedPin, lightMidSun.Red);
    analogWrite(ledGreenPin, lightMidSun.Green);
    analogWrite(ledBluePin, lightMidSun.Blue);
    analogWrite(ledWhitePin, lightMidSun.White);
    //    analogWrite(ledVioletPin, lightMidSun.Violet);
    //    analogWrite(ledMoonPin, lightMidSun.Moon);
  }
  else if (currentLightMode==2)
  {
    analogWrite(ledRedPin, lightLowSun.Red);
    analogWrite(ledGreenPin, lightLowSun.Green);
    analogWrite(ledBluePin, lightLowSun.Blue);
    analogWrite(ledWhitePin, lightLowSun.White);
    //    analogWrite(ledVioletPin, lightLowSun.Violet);
    //    analogWrite(ledMoonPin, lightLowSun.Moon);
  }
  else if (currentLightMode==3)
  {
    analogWrite(ledRedPin, lightMoon.Red);
    analogWrite(ledGreenPin, lightMoon.Green);
    analogWrite(ledBluePin, lightMoon.Blue);
    analogWrite(ledWhitePin, lightMoon.White);
    //    analogWrite(ledVioletPin, lightMoon.Violet);
    //    analogWrite(ledMoonPin, lightMoon.Moon);
  }
  else 
  {
    analogWrite(ledRedPin, lightLowSun.Red);
    analogWrite(ledGreenPin, lightLowSun.Green);
    analogWrite(ledBluePin, lightLowSun.Blue);
    analogWrite(ledWhitePin, lightLowSun.White);
    //    analogWrite(ledVioletPin, lightLowSun.Violet);
    //    analogWrite(ledMoonPin, lightLowSun.Moon);
  }  
}


void updateAlarms()
{

  // clear, free, delete all possible alarms
  for (byte i=0; i<=30; i++) Alarm.free(i);

  //////// POWER ALARMS ////////
  if (schedLights1.active==1)
  {
    Alarm.alarmRepeat(schedLights1.onHour,schedLights1.onMinute,0,AlarmPwrLight1_On);
    Alarm.alarmRepeat(schedLights1.offHour,schedLights1.offMinute,0,AlarmPwrLight1_Off);
  }
  if (schedLights2.active==1)
  {
    Alarm.alarmRepeat(schedLights2.onHour,schedLights2.onMinute,0,AlarmPwrLight2_On);
    Alarm.alarmRepeat(schedLights2.offHour,schedLights2.offMinute,0,AlarmPwrLight2_Off);
  }
  if (schedCirc.active==1)
  {
    Alarm.alarmRepeat(schedCirc.onHour,schedCirc.onMinute,0,AlarmPwrCirc_On);
    Alarm.alarmRepeat(schedCirc.offHour,schedCirc.offMinute,0,AlarmPwrCirc_Off);
  }
  if (schedCo2.active==1)
  {
    Alarm.alarmRepeat(schedCo2.onHour,schedCo2.onMinute,0,AlarmPwrCO2_On);
    Alarm.alarmRepeat(schedCo2.offHour,schedCo2.offMinute,0,AlarmPwrCO2_Off);
  }
  if (schedAux1.active==1)
  {
    Alarm.alarmRepeat(schedAux1.onHour,schedAux1.onMinute,0,AlarmPwrAux1_On);
    Alarm.alarmRepeat(schedAux1.offHour,schedAux1.offMinute,0,AlarmPwrAux1_Off);
  }
  if (schedAux2.active==1)
  {
    Alarm.alarmRepeat(schedAux2.onHour,schedAux2.onMinute,0,AlarmPwrAux2_On);
    Alarm.alarmRepeat(schedAux2.offHour,schedAux2.offMinute,0,AlarmPwrAux2_Off);
  }

  //////// RAMP ALARMS ////////
  Alarm.alarmRepeat(ramp1.onHour,ramp1.onMinute,0,AlarmRamp1);
  Alarm.alarmRepeat(ramp2.onHour,ramp2.onMinute,0,AlarmRamp2);
  Alarm.alarmRepeat(ramp3.onHour,ramp3.onMinute,0,AlarmRamp3);
  Alarm.alarmRepeat(ramp4.onHour,ramp4.onMinute,0,AlarmRamp4);
  Alarm.alarmRepeat(ramp5.onHour,ramp5.onMinute,0,AlarmRamp5);
  Alarm.alarmRepeat(ramp6.onHour,ramp6.onMinute,0,AlarmRamp6);

  //////// DOSING ALARMS ////////
  if (pump1.Sunday==1)
  {
    Alarm.alarmRepeat(dowSunday, pump1.onHour,pump1.onMinute,0,AlarmMacros);
  }
  if (pump2.Sunday==1)
  {
    Alarm.alarmRepeat(dowSunday, pump2.onHour,pump2.onMinute,0,AlarmMicros);
  }
  if (pump1.Monday==1)
  {
    Alarm.alarmRepeat(dowMonday, pump1.onHour,pump1.onMinute,0,AlarmMacros);
  }
  if (pump2.Monday==1)
  {
    Alarm.alarmRepeat(dowMonday, pump2.onHour,pump2.onMinute,0,AlarmMicros);
  }
  if (pump1.Tuesday==1)
  {
    Alarm.alarmRepeat(dowTuesday, pump1.onHour,pump1.onMinute,0,AlarmMacros);
  }
  if (pump2.Tuesday==1)
  {
    Alarm.alarmRepeat(dowTuesday, pump2.onHour,pump2.onMinute,0,AlarmMicros);
  }
  if (pump1.Wednesday==1)
  {
    Alarm.alarmRepeat(dowWednesday, pump1.onHour,pump1.onMinute,0,AlarmMacros);
  }
  if (pump2.Wednesday==1)
  {
    Alarm.alarmRepeat(dowWednesday, pump2.onHour,pump2.onMinute,0,AlarmMicros);
  }
  if (pump1.Thursday==1)
  {
    Alarm.alarmRepeat(dowThursday, pump1.onHour,pump1.onMinute,0,AlarmMacros);
  }
  if (pump2.Thursday==1)
  {
    Alarm.alarmRepeat(dowThursday, pump2.onHour,pump2.onMinute,0,AlarmMicros);
  }
  if (pump1.Friday==1)
  {
    Alarm.alarmRepeat(dowFriday, pump1.onHour,pump1.onMinute,0,AlarmMacros);
  }
  if (pump2.Friday==1)
  {
    Alarm.alarmRepeat(dowFriday, pump2.onHour,pump2.onMinute,0,AlarmMicros);
  }
  if (pump1.Saturday==1)
  {
    Alarm.alarmRepeat(dowSaturday, pump1.onHour,pump1.onMinute,0,AlarmMacros);
  }
  if (pump2.Saturday==1)
  {
    Alarm.alarmRepeat(dowSaturday, pump2.onHour,pump2.onMinute,0,AlarmMicros);
  }
}

// alarm handlers
void AlarmPwrLight1_On() 
{
  digitalWrite(pwrLight1Pin, HIGH);
  globalPower.pwrLight1=1;
  if (dispScreen==1) myFiles.loadBitmap(178, 121, 24, 24, pwrLightIconS[globalPower.pwrLight1]); // update home screen
}

void AlarmPwrLight2_On()
{
  digitalWrite(pwrLight2Pin, HIGH);
  globalPower.pwrLight2=1;
  if (dispScreen==1) myFiles.loadBitmap(206, 121, 24, 24, pwrLightIconS[globalPower.pwrLight2]); // update home screen
}

void AlarmPwrCO2_On()
{
  digitalWrite(pwrCO2Pin, HIGH);
  globalPower.pwrCO2=1;
  if (dispScreen==1) myFiles.loadBitmap(206, 177, 24, 24, pwrCO2IconS[globalPower.pwrCO2]); // update home screen
}

void AlarmPwrCirc_On()
{
  digitalWrite(pwrCircPin, HIGH);
  globalPower.pwrCirc=1;
  if (dispScreen==1) myFiles.loadBitmap(206, 149, 24, 24, pwrCircIconS[globalPower.pwrCirc]); // update home screen
}

void AlarmPwrFilter_On()
{
  digitalWrite(pwrFilterPin, HIGH);
  globalPower.pwrFilter=1;
  if (dispScreen==1) myFiles.loadBitmap(178, 149, 24, 24, pwrFilterIconS[globalPower.pwrFilter]); // update home screen
}

void AlarmPwrHeat_On()
{
  digitalWrite(pwrHeatPin, HIGH);
  globalPower.pwrHeat=1;
  if (dispScreen==1) myFiles.loadBitmap(178, 177, 24, 24, pwrHeatIconS[globalPower.pwrHeat]); // update home screen
}

void AlarmPwrAux1_On()
{
  digitalWrite(pwrAux1Pin, HIGH);
  globalPower.pwrAux1=1;
  if (dispScreen==1) myFiles.loadBitmap(178, 205, 24, 24, pwrAux1IconS[globalPower.pwrAux1]); // update home screen
}

void AlarmPwrAux2_On()
{
  digitalWrite(pwrAux2Pin, HIGH);
  globalPower.pwrAux2=1;
  if (dispScreen==1) myFiles.loadBitmap(206, 205, 24, 24, pwrAux2IconS[globalPower.pwrAux2]); // update home screen
}

void AlarmPwrLight1_Off()
{
  digitalWrite(pwrLight1Pin, LOW);
  globalPower.pwrLight1=0;
  if (dispScreen==1) myFiles.loadBitmap(178, 121, 24, 24, pwrLightIconS[globalPower.pwrLight1]); // update home screen
}

void AlarmPwrLight2_Off()
{
  digitalWrite(pwrLight2Pin, LOW);
  globalPower.pwrLight2=0;
  if (dispScreen==1) myFiles.loadBitmap(206, 121, 24, 24, pwrLightIconS[globalPower.pwrLight2]); // update home screen
}

void AlarmPwrCO2_Off()
{
  digitalWrite(pwrCO2Pin, LOW);
  globalPower.pwrCO2=0;
  // if we happen to be feeding, and the CO2 off schedule kills the CO2, make sure CO2 doesn't turn back on 
  if (feedingActive==true) preFeedPower.pwrCO2=0; 
  if (dispScreen==1) myFiles.loadBitmap(206, 177, 24, 24, pwrCO2IconS[globalPower.pwrCO2]); // update home screen
}

void AlarmPwrCirc_Off()
{
  digitalWrite(pwrCircPin, LOW);
  globalPower.pwrCirc=0;
  if (dispScreen==1) myFiles.loadBitmap(206, 149, 24, 24, pwrCircIconS[globalPower.pwrCirc]); // update home screen
}

void AlarmPwrFilter_Off()
{
  digitalWrite(pwrFilterPin, LOW);
  globalPower.pwrFilter=0;
  if (dispScreen==1) myFiles.loadBitmap(178, 149, 24, 24, pwrFilterIconS[globalPower.pwrFilter]); // update home screen
}

void AlarmPwrHeat_Off()
{
  digitalWrite(pwrHeatPin, LOW);
  globalPower.pwrHeat=0;
  if (dispScreen==1) myFiles.loadBitmap(178, 177, 24, 24, pwrHeatIconS[globalPower.pwrHeat]); // update home screen
}

void AlarmPwrAux1_Off()
{
  digitalWrite(pwrAux1Pin, LOW);
  globalPower.pwrAux1=0;
  if (dispScreen==1) myFiles.loadBitmap(178, 205, 24, 24, pwrAux1IconS[globalPower.pwrAux1]); // update home screen
}

void AlarmPwrAux2_Off()
{
  digitalWrite(pwrAux2Pin, LOW);
  globalPower.pwrAux2=0;
  if (dispScreen==1) myFiles.loadBitmap(206, 205, 24, 24, pwrAux2IconS[globalPower.pwrAux2]); // update home screen
}

void AlarmRamp1()
{
  // AlarmPwrLight1_On(); // turn lights on no matter what, avoids any accidental schedule conflicts
  // AlarmPwrLight2_On(); // turn lights on no matter what, avoids any accidental schedule conflicts
  delay(1000); // delay for 1 second to make sure the lights have finished turning on

  fadeFromMode = 3;  //0=high sun, 1=mid sun, 2=low sun, 3=moon
  fadeToMode = 2;  //0=high sun, 1=mid sun, 2=low sun, 3=moon

  // last color is the starting point of the fade
  lastColor.Red = currentColor.Red;
  lastColor.Green = currentColor.Green;
  lastColor.Blue = currentColor.Blue;
  lastColor.White = currentColor.White;

  // target color is low sun
  targetColor.Red = lightLowSun.Red;
  targetColor.Green = lightLowSun.Green;
  targetColor.Blue = lightLowSun.Blue;
  targetColor.White = lightLowSun.White;

  // calculate how long to run the fade for
  int fadeHours = ramp1.offHour;
  int fadeMins = ramp1.offMinute;  
  fadeDurationSeconds = ((fadeHours*60*60)+(fadeMins*60));

  fadeStartingSeconds = now();
  fadeInProgress = true;
  currentLightMode=4;
  if (dispScreen==1) checkLighting();
}

void AlarmRamp2()
{
  fadeFromMode = 2;  //0=high sun, 1=mid sun, 2=low sun, 3=moon
  fadeToMode = 1;  //0=high sun, 1=mid sun, 2=low sun, 3=moon

  // last color is the starting point of the fade
  lastColor.Red = currentColor.Red;
  lastColor.Green = currentColor.Green;
  lastColor.Blue = currentColor.Blue;
  lastColor.White = currentColor.White;

  // target color is mid sun
  targetColor.Red = lightMidSun.Red;
  targetColor.Green = lightMidSun.Green;
  targetColor.Blue = lightMidSun.Blue;
  targetColor.White = lightMidSun.White;

  // calculate how long to run the fade for
  int fadeHours = ramp2.offHour;
  int fadeMins = ramp2.offMinute;  
  fadeDurationSeconds = ((fadeHours*60*60)+(fadeMins*60));

  fadeStartingSeconds = now();
  fadeInProgress = true;
  currentLightMode=4;
  if (dispScreen==1) checkLighting();
}

void AlarmRamp3()
{
  fadeFromMode = 1;  //0=high sun, 1=mid sun, 2=low sun, 3=moon
  fadeToMode = 0;  //0=high sun, 1=mid sun, 2=low sun, 3=moon

  // last color is the starting point of the fade
  lastColor.Red = currentColor.Red;
  lastColor.Green = currentColor.Green;
  lastColor.Blue = currentColor.Blue;
  lastColor.White = currentColor.White;

  // target color is high sun
  targetColor.Red = lightHighSun.Red;
  targetColor.Green = lightHighSun.Green;
  targetColor.Blue = lightHighSun.Blue;
  targetColor.White = lightHighSun.White;

  // calculate how long to run the fade for
  int fadeHours = ramp3.offHour;
  int fadeMins = ramp3.offMinute;  
  fadeDurationSeconds = ((fadeHours*60*60)+(fadeMins*60));

  fadeStartingSeconds = now();
  fadeInProgress = true;
  currentLightMode=4;
  if (dispScreen==1) checkLighting();
}

void AlarmRamp4()
{
  fadeFromMode = 0;  //0=high sun, 1=mid sun, 2=low sun, 3=moon
  fadeToMode = 1;  //0=high sun, 1=mid sun, 2=low sun, 3=moon

  // last color is the starting point of the fade
  lastColor.Red = currentColor.Red;
  lastColor.Green = currentColor.Green;
  lastColor.Blue = currentColor.Blue;
  lastColor.White = currentColor.White;

  // target color is mid sun
  targetColor.Red = lightMidSun.Red;
  targetColor.Green = lightMidSun.Green;
  targetColor.Blue = lightMidSun.Blue;
  targetColor.White = lightMidSun.White;

  // calculate how long to run the fade for
  int fadeHours = ramp4.offHour;
  int fadeMins = ramp4.offMinute;  
  fadeDurationSeconds = ((fadeHours*60*60)+(fadeMins*60));

  fadeStartingSeconds = now();
  fadeInProgress = true;
  currentLightMode=4;
  if (dispScreen==1) checkLighting();
}

void AlarmRamp5()
{
  fadeFromMode = 1;  //0=high sun, 1=mid sun, 2=low sun, 3=moon
  fadeToMode = 2;  //0=high sun, 1=mid sun, 2=low sun, 3=moon

  // last color is the starting point of the fade
  lastColor.Red = currentColor.Red;
  lastColor.Green = currentColor.Green;
  lastColor.Blue = currentColor.Blue;
  lastColor.White = currentColor.White;

  // target color is low sun
  targetColor.Red = lightLowSun.Red;
  targetColor.Green = lightLowSun.Green;
  targetColor.Blue = lightLowSun.Blue;
  targetColor.White = lightLowSun.White;

  // calculate how long to run the fade for
  int fadeHours = ramp5.offHour;
  int fadeMins = ramp5.offMinute;  
  fadeDurationSeconds = ((fadeHours*60*60)+(fadeMins*60));

  fadeStartingSeconds = now();
  fadeInProgress = true;
  currentLightMode=4;
  if (dispScreen==1) checkLighting();
}

void AlarmRamp6()
{
  fadeFromMode = 2;  //0=high sun, 1=mid sun, 2=low sun, 3=moon
  fadeToMode = 3;  //0=high sun, 1=mid sun, 2=low sun, 3=moon

  // last color is the starting point of the fade
  lastColor.Red = currentColor.Red;
  lastColor.Green = currentColor.Green;
  lastColor.Blue = currentColor.Blue;
  lastColor.White = currentColor.White;

  // target color is moonlight
  targetColor.Red = lightMoon.Red;
  targetColor.Green = lightMoon.Green;
  targetColor.Blue = lightMoon.Blue;
  targetColor.White = lightMoon.White;

  // calculate how long to run the fade for
  int fadeHours = ramp6.offHour;
  int fadeMins = ramp6.offMinute;  
  fadeDurationSeconds = ((fadeHours*60*60)+(fadeMins*60));

  fadeStartingSeconds = now();
  fadeInProgress = true;
  currentLightMode=4;
  if (dispScreen==1) checkLighting();
}

void AlarmMacros()
{
  int pumpTime=EEPROM.read(23);  // 23 // pump 1 ms/ml
  int doseAmt=EEPROM.read(21); // 21 // dose in mL
  pumpTime=((pumpTime*10)*doseAmt); // multiply ms/mL by number of mL to pump
  analogWrite(dosingPump1, 255);
  delay(pumpTime); // delay while the motor runs
  analogWrite(dosingPump1, 0);

  // update resevoir volume, storing as high/low bytes so we can store bigger numbers
  int vol_H=EEPROM.read(32); // read the remaining volume for pump 1 (high byte, ^10)
  int vol_L=EEPROM.read(33); // read the remaining volume for pump 1 (low byte)
  float vol=((vol_H*10)+vol_L); // combine high and low byte to get volume
  vol=(vol-doseAmt); // subtract off what we just dosed
  float floatVol=(vol/10); // divide by 10 to start converting to high byte
  vol_H=floatVol; // move float into int to remove the decimal (this is our high byte)
  EEPROM.write(32,vol_H); // write high byte to memory
  vol_L=(vol-(vol_H*10)); // subtract the high byte from the dosing volume to get the low byte
  EEPROM.write(33,vol_L); // write low byte to memory

  if (dispScreen==1) checkDosing(); // update home screen
}

void AlarmMicros()
{
  int pumpTime=EEPROM.read(24);  // 24 // pump 2 sec/ml
  int doseAmt=EEPROM.read(21); // 21 // dose in mL
  pumpTime=((pumpTime*10)*doseAmt); // multiply ms/mL by number of mL to pump
  analogWrite(dosingPump2, 255);
  delay(pumpTime); // delay while the motor runs
  analogWrite(dosingPump2, 0);

  // update resevoir volume, storing as high/low bytes so we can store bigger numbers
  int vol_H=EEPROM.read(34); // read the remaining volume for pump 1 (high byte, ^10)
  int vol_L=EEPROM.read(35); // read the remaining volume for pump 1 (low byte)
  float vol=((vol_H*10)+vol_L); // combine high and low byte to get volume
  vol=(vol-doseAmt); // subtract off what we just dosed
  float floatVol=(vol/10); // divide by 10 to start converting to high byte
  vol_H=floatVol; // move float into int to remove the decimal (this is our high byte)
  EEPROM.write(34,vol_H); // write high byte to memory
  vol_L=(vol-(vol_H*10)); // subtract the high byte from the dosing volume to get the low byte
  EEPROM.write(35,vol_L); // write low byte to memory

  if (dispScreen==1) checkDosing(); // update home screen
}


int SerialReadInt()
{
  // Reads first 3 bytes from serial monitor; anything more is tossed
  byte i;
  char inBytes[4];
  char * inBytesPtr = &inBytes[0];  // Pointer to first element

  for (i=0; i<3; i++)             // Only want first 3 bytes
    inBytes[i] = Serial.read();
  inBytes[i] =  '\0';             // Put NULL character at the end
  while (Serial.read() >= 0)      // If anything else is there, throw it away
    ; // do nothing      
  return atoi(inBytesPtr);        // Convert to decimal
}

void TestCodes (int cmd)
{
  // Handles commands sent in from the serial monitor
  if (cmd==1)
  {
    Serial.print("Sensor: ");
    int photocellReading = analogRead(lightSensorPin); 
    Serial.print(photocellReading);
    Serial.print("\n");
  }
  else if (cmd>1) analogWrite(screenBrightPin, cmd);

}

// Check if there should be any IR commands sent for the color fade, based on time elapsed
void checkLightFade(int secondsElapsed, int durationInSeconds)
{  

  // update fade timing info on home screen
  if (dispScreen==1)
  {
    // Get the time in seconds (since 1970)
    unsigned long rightNow = now();

    // calculate how much time is left in the fade in minutes
    unsigned long timeLeft=((fadeStartingSeconds + fadeDurationSeconds)-rightNow)/60;

    // we only update the display if it's a new value
    if (timeLeft!=fadeTimeLeft)
    {
      // update remaining time
      fadeTimeLeft=timeLeft;

      // convert to char
      int minsLeft=timeLeft;
      char char4[4];
      itoa(minsLeft, char4, 10);

      // center the minutes left
      int xmin;
      if (minsLeft<10) xmin=77;
      if ((minsLeft>9)&&(minsLeft<100)) xmin=73;
      if (minsLeft>99) xmin=69;

      // clear old data
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(69,207,93,215);

      // print to screen
      myGLCD.setFont(Sinclair_S);
      myGLCD.setColor(255, 255, 255);
      myGLCD.print(char4, xmin, 207);
      myGLCD.print("MIN", 69, 219);
    }
  }


  // RED /////////////////////////////////////////////////////////
  if(targetColor.Red != lastColor.Red)
  {
    // Get the change per second for the current color to the target color
    float RTick = (float)durationInSeconds / (float)(targetColor.Red - lastColor.Red);

    // Get the expected change for the time elapsed
    int RValue = round(secondsElapsed / RTick); 

    if(RTick > 0) // If the change is positive
    {
      // And if the current color does not match the expected color
      if(lastColor.Red + RValue > currentColor.Red) 
      {
        // And, for protection, the current color does not exceed the target currentColor.
        if(currentColor.Red < targetColor.Red)
        {
          // Increase the current color
          currentColor.Red++;
          if (lightCSP==true) 
          {
            Serial.print("red up\n");
            irsend.sendNEC(REDUP,32);
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledRedPin, currentColor.Red);
        }
      }
    }
    else // If the change is negative
    {
      // And if the current color does not match the expected color
      if((lastColor.Red + RValue) < currentColor.Red)
      {
        // And, for protection, the current color is not less than the target currentColor.
        if(currentColor.Red > targetColor.Red)
        {
          // Decrease the current color
          currentColor.Red--;
          if (lightCSP==true) 
          {
            Serial.print("red down\n");
            irsend.sendNEC(REDDOWN,32);
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledRedPin, currentColor.Red);
        }
      } 
    }
  }

  // GREEN /////////////////////////////////////////////////////////
  if(targetColor.Green != lastColor.Green)
  {
    // Get the change per second for the current color to the target color
    float GTick = (float)durationInSeconds / (float)(targetColor.Green - lastColor.Green);

    // Get the expected change for the time elapsed
    int GValue = round(secondsElapsed / GTick);

    if(GTick > 0) // If the change is positive
    {
      // And if the current color does not match the expected color
      if(lastColor.Green + GValue > currentColor.Green)
      {
        // And, for protection, the current color does not exceed the target currentColor.
        if(currentColor.Green < targetColor.Green)
        {
          // Increase the current color
          currentColor.Green++;
          if (lightCSP==true) 
          {
            Serial.print("green up\n");
            irsend.sendNEC(GREENUP,32);
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledGreenPin, currentColor.Green);
        }
      }
    }
    else // If the change is negative
    {
      // And if the current color does not match the expected color
      if((lastColor.Green + GValue) < currentColor.Green)
      {        
        // And, for protection, the current color is not less than the target currentColor.
        if(currentColor.Green > targetColor.Green)
        {
          // Decrease the current color
          currentColor.Green--;
          if (lightCSP==true) 
          {
            Serial.print("green down\n");
            irsend.sendNEC(GREENDOWN,32);
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledGreenPin, currentColor.Green);
        }
      } 
    }
  }

  // BLUE /////////////////////////////////////////////////////////
  if(targetColor.Blue != lastColor.Blue)
  {
    // Get the change per second for the current color to the target color
    float BTick = (float)durationInSeconds / (float)(targetColor.Blue - lastColor.Blue);

    // Get the expected change for the time elapsed
    int BValue = round(secondsElapsed / BTick);

    if(BTick > 0) // If the change is positive
    {
      // And if the current color does not match the expected color
      if(lastColor.Blue + BValue > currentColor.Blue)
      {
        // And, for protection, the current color does not exceed the target currentColor.
        if(currentColor.Blue < targetColor.Blue)
        {
          // Increase the current color
          currentColor.Blue++;
          if (lightCSP==true) 
          {
            Serial.print("blue up\n");
            irsend.sendNEC(BLUEUP,32);
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledBluePin, currentColor.Blue);
        }
      }
    }
    else // If the change is negative
    {
      // And if the current color does not match the expected color
      if((lastColor.Blue + BValue) < currentColor.Blue)
      {
        // And, for protection, the current color is not less than the target currentColor.
        if(currentColor.Blue > targetColor.Blue)
        {
          // Decrease the current color
          currentColor.Blue--;
          if (lightCSP==true) 
          {
            Serial.print("blue down\n");
            irsend.sendNEC(BLUEDOWN,32);
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledBluePin, currentColor.Blue);
        }
      }
    }
  }

  // WHITE /////////////////////////////////////////////////////////
  if(targetColor.White != lastColor.White)
  {
    // Get the change per second for the current color to the target color
    float WTick = (float)durationInSeconds / (float)(targetColor.White - lastColor.White);

    // Get the expected change for the time elapsed
    int WValue = round(secondsElapsed / WTick);

    if(WTick > 0) // If the change is positive
    {
      // And if the current color does not match the expected color
      if(lastColor.White + WValue > currentColor.White)
      {
        // And, for protection, the current color does not exceed the target currentColor.
        if(currentColor.White < targetColor.White)
        {
          // Increase the current color
          currentColor.White++;
          if (lightCSP==true) 
          {
            Serial.print("white up\n");
            irsend.sendNEC(WHITEUP,32);
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledWhitePin, currentColor.White);
        }
      }
    }
    else // If the change is negative
    {
      // And if the current color does not match the expected color
      if((lastColor.White + WValue) < currentColor.White)
      {
        // And, for protection, the current color is not less than the target currentColor.
        if(currentColor.White > targetColor.White)
        {
          // Decrease the current color
          currentColor.White--;
          if (lightCSP==true) 
          {
            Serial.print("white down\n");
            irsend.sendNEC(WHITEDOWN, 32);
            delay(333);
          }
          else if (lightCSP==false) analogWrite(ledWhitePin, currentColor.White);
        }
      } 
    }
  }
}

void firstRunSetup()
{

  // we need to track if this has EVER run, because we only want it to run once ever
  // iAqua was started 6/5/2014. That's when I created the project.
  // I'm going to save 65 into EEPROM bank 2014 as a check
  if (EEPROM.read(2014)!=65) {

    EEPROM.write(2014,65); // write to EEPROM so this never runs again

    // default screen settings
    EEPROM.write(28,2); 
    EEPROM.write(29,3);
    EEPROM.write(30,30);
    EEPROM.write(31,255);

    // set default feeding values on first launch
    EEPROM.write(1,minute());   // last feeding data
    EEPROM.write(2,hour());   // last feeding data
    EEPROM.write(3,day());   // last feeding data
    EEPROM.write(4,month());   // last feeding data
    EEPROM.write(5,(year()-2000));   // last feeding data

    EEPROM.write(6,1);   // 6 // feeding  settings saved
    EEPROM.write(7,10);  // 7 // feeding minutes setting
    EEPROM.write(8,1);   // 8 // feeding  light 1 (0 off, 1 on)
    EEPROM.write(9,1);   // 9 // feeding pwr light 2 (0 off, 1 on)
    EEPROM.write(10,0);  // 10 // feeding pwr filter (0 off, 1 on)
    EEPROM.write(11,0);  // 11 // feeding pwr circ (0 off, 1 on)
    EEPROM.write(12,0);  // 12 // feeding pwr heat (0 off, 1 on)
    EEPROM.write(13,0);  // 13 // feeding pwr co2 (0 off, 1 on)
    EEPROM.write(14,0);  // 14 // feeding pwr aux 1 (0 off, 1 on)
    EEPROM.write(15,0);  // 15 // feeding pwr aux 2 (0 off, 1 on)

    // feeding settings
    EEPROM.write(6,1);
    EEPROM.write(7,10); // default of 10 min

    // set default heater values on first launch

    EEPROM.write(16,1);  // 16 // heater settings saved (0 for no, 1 for yes)
    EEPROM.write(17,80); // 17 // heater off temp
    EEPROM.write(18,76); // 18 // heater on temp
    EEPROM.write(19,82); // 19 // heater alarm temp

    // set default dosing values on first launch

    EEPROM.write(20,1);   // 20 // dosing settings saved
    EEPROM.write(21,10);  // 21 // dose in mL
    EEPROM.write(22,6);  // 22 // dosing reseviors capacity in mL^10
    EEPROM.write(23,79);  // 23 // pump 1 ms^10/ml
    EEPROM.write(24,79);  // 24 // pump 2 ms^10/ml
    EEPROM.write(25,1);   // 25 // dosing volume saved (0 for no, 1 for yes)
    EEPROM.write(26,0);   // 26 // pump 1 remaining doses
    EEPROM.write(27,0);   // 27 // pump 2 remaining doses
    EEPROM.write(32,0);   // remaining volume for pump 1 (high byte ^10)
    EEPROM.write(33,0);   // remaining volume for pump 1 (low byte)
    EEPROM.write(34,0);   // remaining volume for pump 2 (high byte ^10)
    EEPROM.write(35,0);   // remaining volume for pump 2 (low byte)


    // power schedules, zero them all out
    EEPROM.write(100,0);   // 
    EEPROM.write(100,0);   //
    EEPROM.write(101,0);   //
    EEPROM.write(102,0);   //
    EEPROM.write(103,0);   //
    EEPROM.write(104,0);   //
    EEPROM.write(105,0);   //
    EEPROM.write(106,0);   //
    EEPROM.write(107,0);   //
    EEPROM.write(108,0);   //
    EEPROM.write(109,0);   //
    EEPROM.write(110,0);   //
    EEPROM.write(111,0);   //
    EEPROM.write(112,0);   //
    EEPROM.write(113,0);   //
    EEPROM.write(114,0);   //
    EEPROM.write(115,0);   //
    EEPROM.write(116,0);   //
    EEPROM.write(117,0);   //
    EEPROM.write(118,0);   //
    EEPROM.write(119,0);   //
    EEPROM.write(120,0);   //
    EEPROM.write(121,0);   //
    EEPROM.write(122,0);   //
    EEPROM.write(123,0);   //
    EEPROM.write(124,0);   //
    EEPROM.write(125,0);   //
    EEPROM.write(126,0);   //
    EEPROM.write(127,0);   //
    EEPROM.write(128,0);   //
    EEPROM.write(129,0);   //

    //light ramp schedules
    EEPROM.write(220,0);   //
    EEPROM.write(221,0);   //
    EEPROM.write(222,0);   //
    EEPROM.write(223,0);   //
    EEPROM.write(224,0);   //
    EEPROM.write(225,0);   //
    EEPROM.write(226,0);   //
    EEPROM.write(227,0);   //
    EEPROM.write(228,0);   //
    EEPROM.write(229,0);   //
    EEPROM.write(230,0);   //
    EEPROM.write(231,0);   //
    EEPROM.write(232,0);   //
    EEPROM.write(233,0);   //
    EEPROM.write(234,0);   //
    EEPROM.write(235,0);   //
    EEPROM.write(236,0);   //
    EEPROM.write(237,0);   //
    EEPROM.write(238,0);   //
    EEPROM.write(239,0);   //
    EEPROM.write(240,0);   //
    EEPROM.write(241,0);   //
    EEPROM.write(242,0);   //
    EEPROM.write(243,0);   //

    // dosing schedules
    EEPROM.write(300,0);   //
    EEPROM.write(301,0);   //
    EEPROM.write(302,0);   //
    EEPROM.write(303,0);   //
    EEPROM.write(304,0);   //
    EEPROM.write(305,0);   //
    EEPROM.write(306,0);   //
    EEPROM.write(307,0);   //
    EEPROM.write(308,0);   //
    EEPROM.write(309,0);   //
    EEPROM.write(310,0);   //
    EEPROM.write(311,0);   //
    EEPROM.write(312,0);   //
    EEPROM.write(313,0);   //
    EEPROM.write(314,0);   //
    EEPROM.write(315,0);   //
    EEPROM.write(316,0);   //
    EEPROM.write(317,0);   //

    // RGBW values
    EEPROM.write(200,0);   //
    EEPROM.write(201,0);   //
    EEPROM.write(202,0);   //
    EEPROM.write(203,0);   //
    EEPROM.write(204,0);   //
    EEPROM.write(205,0);   //
    EEPROM.write(206,0);   //
    EEPROM.write(207,0);   //
    EEPROM.write(208,0);   //
    EEPROM.write(209,0);   //
    EEPROM.write(210,0);   //
    EEPROM.write(211,0);   //
    EEPROM.write(212,0);   //
    EEPROM.write(213,0);   //
    EEPROM.write(214,0);   //
    EEPROM.write(215,0);   //

  }
}















