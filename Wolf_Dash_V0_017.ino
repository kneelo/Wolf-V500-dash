/* Lapduino lap timer and datalogging dash display for Wolf V500 ECU v0.017

This program displays engine operating information from a Wolf V500 ECU and lap time data on a 20x4 LCD screen.
All data is logged at the GPS refresh speed on a SD card.
A 16 WS2812 RGB LED string is used as a shift lit and to display warning alerts.

Hardware:
Arduino Mega 2560
Serial buffer increased from the standard 64 bytes to 256 bytes as sometimes the loop takes so long that the serial buffer was overflowing (do this by altering the HardwareSerial.h file)
20x4 I2C LCD display (connected on i2c bus)
16 LED WS2812 neopixel string (connected on digital pin 6)
TTL to RS232 serial adapter (connected on hardware serial 1)
Iblox NEO 6 GPS Module (connected on hardware serial 2) (modify gps module to communicate at 38400, refresh at 5hz and only send GGA and RMC NMEA sentences)
Adafuit SD card datalogger and rtc shield  (SD card connected on digital pins 10, 11, 12, 13 and rtc using i2c bus.  SD library also requires pin 53 set to output even if it not used)
Selector buttons on digital pins 2, 3, 4 & 5 (switched to gnd)
Brake connected on digital pin 7 (connected to the brake light +12V wire through a voltage divider to reduce the voltage to less than 5V)
*/

//--------------------------------------------------------Libraries-----------------------------------------------------

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPS++.h>
#include <ClickButton.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>


//-----------------------------------------------------Constants---------------------------------------------------------

#define brightness 30  //brightness of neopixel LED's

//---------------------------------------------------Objects-----------------------------------------------------

// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)

Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, 6, NEO_GRB + NEO_KHZ800);  //16 pixel strip, controlled from digital pin 6

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

TinyGPSPlus gps;

RTC_DS1307 rtc;  //Real time clock

// Initiate ClickButton objects

/*
ClickButton buttonObject(pin [,active [,CLICKBTN_PULLUP]]);
where:
buttonObject is your name for the button object in code.
pin is the pin connected to the button
active denotes an active LOW or HIGH button (default is LOW)
CLICKBTN_PULLUP turns on the internal pullup resistor. This is only possible with active low buttons.
*/

ClickButton button1(2, LOW, CLICKBTN_PULLUP);  //page scroll up
ClickButton button2(3, LOW, CLICKBTN_PULLUP);  //page scroll down
ClickButton button3(4, LOW, CLICKBTN_PULLUP);  //line scroll
ClickButton button4(5, LOW, CLICKBTN_PULLUP);  //enter

void colorWipe(uint32_t c, uint8_t wait)  // Define colorwipe function
{
  for (uint16_t i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void colorSolid(uint32_t c)  // Define colorsolid function
{
  for (uint16_t i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, c);
  }
  strip.show();
}

//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.

void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

//This function will return a 4 byte (32bit) long from the eeprom
//at the specified address to address + 3.

long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time)
{
  DateTime now = rtc.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

//------------------------------------------Global Variables--------------------------------------------------

int pagenumber = 0;
int pageupdate = 1;
int button1state = 0;
int button2state = 0;
int button3state = 0;
int button4state = 0;
int alarm = 0;

int rpm = 0;
int manipressg = 0;
int injduty = 0;
long fuelpress = 0;
int rel_fuelpress = 0;
int airtemp = 0;
int oilpress = 0;
int load = 0;
int throttle = 0;
int ignadv = 0;
int idlevlv = 0;
int engtemp = 0;
int boostvlv = 0;
int brakeson;

float gpsspeedmax = 0;
int rpmmax = 0;
float voltmin = 99.90;
int oilpressmin = 999;
int rel_fuelpressmin = 999;
int manipressgmax = -99;
int airtempmax = 0;

float volt = 0.00;
float afr = 0.00;
float PSIboost = 0.00;
float injms = 0.00;

long gpslat = 0;
long gpslon = 0;
long prevgpslat = 0;
long prevgpslon = 0;
float gpslatf;
float gpslonf;

float gpsspeed;
float gpscourse;
int gpssats;
long gpshdop;
float gpshdopf;
long unsigned int gpsfixage;

int sessionno;
int timerstatus = 0;
long long d;
long long startlat;
long long startlon;
long long pitlat;
long long pitlon;
long xdeg;
long ydeg;

int gpslatdeg;
long gpslatbil;
int gpslondeg;
long gpslonbil;

long dist;
long dist1;
long dist2;
float distf;
float dist1f;
float dist2f;

long dlat;
long dlon;
float factor;

long start1lat;
long start1lon;
long start2lat;
long start2lon;

long pit1lat;
long pit1lon;
long pit2lat;
long pit2lon;

long long  intersectlon = 0;
long long  intersectlat = 0;
long long  a = 0;
long long  b = 0;
long XAsumstart;
long XBsumstart;
long YAsumstart;
long YBsumstart;
long XAsumpit;
long XBsumpit;
long YAsumpit;
long YBsumpit;

int lineselect = 0;
int flashstate = 0;

float prevlapdist;
float lapdist;
long startcross;
long finishcross;
long prevlaptime;
float prevlaptimef;
long laptime;
float laptimef;
int lapnumber = 0;
long gpsprevMillis;
long gpsMillis;

long LCDpreviousMillis = 0;
int LCDinterval = 500;           // Interval at which to update LCD (milliseconds)

long wolf1previousMillis = 0;
long wolf1interval = 100;         // Interval at which to update wolf packet 0 data from ECU (milliseconds)
int wolf1update = 0;
byte wolf1request[4] = {0x52, 0x00, 0x00, 0x1f}; //String to request packet 0 data from wolf ECU

long wolf2previousMillis = 0;
long wolf2interval = 200;         // Interval at which to request new wolf packet 2 data from ECU (milliseconds)
int wolf2update = 0;
byte wolf2request[4] = {0x52, 0x00, 0x40, 0x1f}; //String to request packet 2 data from wolf ECU

byte wolfdata [64];  //array to store wolf data
byte readbyte = 0;
int wolfcounter = 0;
long wolfcount = 0;  //count of loop cycles since last time ecu data was updated

long currentMillis = 0;

long alarmmillis;
long alarmintervalmillis = 0;
long alarmflashinterval = 250;  //length of LED flashes for alarm
long alarmduration = 5000; //time alarm is continued to be displayed for after fault clears
int alarmflashstate = 0;

long loopcount;  //count of loop cycles run between logging events

String dataString = "";
File dataFile;

//-----------------------------------------------SETUP--------------------------------------------------------

void setup()
{

  //Serial.begin(38400);  // Open serial port for debugging from computer
  Serial1.begin(38400);  // Open serial port for comms with Wolf ECU
  Serial2.begin(38400);  //Open serial port for comms with GPS module

  pinMode(53, OUTPUT); //required to make the SD card libray work properly on the mega

  Wire.begin();

  lcd.begin(20, 4);        //initialize the lcd for 20 chars 4 lines

  rtc.begin();            //initialise real time clock

  //3 Quick  blinks of backlight
  for (int i = 0; i < 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }

  lcd.backlight();

  //write start text to LCD
  lcd.setCursor(1, 0);
  lcd.print("LAPDUINO WOLF V500");
  lcd.setCursor(2, 1);
  lcd.print("DATALOGGING DASH");
  delay(500);
  lcd.setCursor(7, 2);
  lcd.print("V0.017");
  delay(5000);

  //initialise neopixel strip and wipe a blue streak across it
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  colorWipe(strip.Color(0, 0, brightness), 25); // Blue wipe accross neopixels
  colorWipe(strip.Color(0, 0, 0), 25); // turn off neopixels


  //setup button timers (all in milliseconds / ms)
  button1.debounceTime   = 20;   // Debounce timer in ms
  button1.multiclickTime = 250;  // Time limit for multi clicks
  button1.longClickTime  = 1000; // Time until long clicks register
  button2.debounceTime   = 20;   // Debounce timer in ms
  button2.multiclickTime = 250;  // Time limit for multi clicks
  button2.longClickTime  = 1000; // Time until long clicks register
  button3.debounceTime   = 20;   // Debounce timer in ms
  button3.multiclickTime = 250;  // Time limit for multi clicks
  button3.longClickTime  = 1000; // Time until long clicks register
  button4.debounceTime   = 20;   // Debounce timer in ms
  button4.multiclickTime = 250;  // Time limit for multi clicks
  button4.longClickTime  = 1000; // Time until long clicks register

  //check if SD card is present

  if (!SD.begin(10, 11, 12, 13)) {
    //Serial.println("SD card initialization failed!");
    lcd.setCursor(4, 3);
    lcd.print("NO SD CARD!");
    delay(10000);
    return;
  }
  //Serial.println("SD card initialization complete");

  //retreive session number from eeprom, increment and write back to eeprom
  sessionno = EEPROMReadlong(32);
  sessionno = sessionno + 1;
  EEPROMWritelong(32, sessionno);

  //create new log file name for each new session using the session number
  String sessionname = "log";
  if (sessionno < 10)
  {
    sessionname += "000";
    sessionname += sessionno;
  }

  else if (sessionno < 100)
  {
    sessionname += "00";
    sessionname += sessionno;
  }

  else if (sessionno < 1000)
  {
    sessionname += "0";
    sessionname += sessionno;
  }

  else
  {
    sessionname += sessionno;
  }

  sessionname += ".csv";


  // set the time stamp for the log file
  SdFile::dateTimeCallback(dateTime);

  // open log file on the SD card
  dataFile = SD.open(sessionname.c_str(), FILE_WRITE);
  //Serial.println(sessionname.c_str());

  //read stored start line and pit lane entry line data from the eeprom and write to log file

  start1lat = EEPROMReadlong(0);
  start1lon = EEPROMReadlong(4);
  start2lat = EEPROMReadlong(8);
  start2lon = EEPROMReadlong(12);
  pit1lat = EEPROMReadlong(16);
  pit1lon = EEPROMReadlong(20);
  pit2lat = EEPROMReadlong(24);
  pit2lon = EEPROMReadlong(28);

  dataFile.print("start1lat: ");
  dataFile.println(start1lat);
  dataFile.print("start1lon: ");
  dataFile.println(start1lon);
  dataFile.print("start2lat: ");
  dataFile.println(start2lat);
  dataFile.print("start2lon: ");
  dataFile.println(start2lon);
  dataFile.print("pit1lat: ");
  dataFile.println(pit1lat);
  dataFile.print("pit1lon: ");
  dataFile.println(pit1lon);
  dataFile.print("pit2lat: ");
  dataFile.println(pit2lat);
  dataFile.print("pit2lon: ");
  dataFile.println(pit2lon);

  //calculate variables used to detect if start line or pit entry crossed
  XBsumstart = start1lon - start2lon;
  YBsumstart = start1lat - start2lat;
  XBsumpit = pit1lon - pit2lon;
  YBsumpit = pit1lat - pit2lat;

  pinMode(7, INPUT);  //set digital pin 7 as input pin for brake indication

  //retrieve RTC time and write to log file
  DateTime now = rtc.now();
  dataString = "";
  dataString += String(now.year(), DEC);
  dataString += "/";
  dataString += String(now.month(), DEC);
  dataString += "/";
  dataString += String(now.day(), DEC);
  dataString += " ";
  dataString += String(now.hour(), DEC);
  dataString += ":";
  dataString += String(now.minute(), DEC);
  dataString += ":";
  dataString += String(now.second(), DEC);
  dataFile.println(dataString);
  //Serial.println(dataString);

  //write column headers to log file
  dataString = "GPS time,arduino milliseconds,gps fix age,latitude,longitude,lapnumber,lap time,last lap time,lap distance,speed,GPD heading,GPS sats,GPS hdop,RPM,load,throttle,injector ms,injector duty,ignition timing,manifold pressure,boost PSI,air temp,engine temp,voltage,afr,idle valve,oil pressure,fuel pressure,manifold relative fuel pressure,brakes,alarm status,loop cycles count";
  dataFile.println(dataString);
  //Serial.println(dataString);

  lcd.clear();

}
//-----------------------------------------------END SETUP-------------------------------------------------------

//------------------------------------------------LOOP-----------------------------------------------------------
void loop()
{

  //--------------------------------------------Update GPS data---------------------------------------------

  while (Serial2.available() > 0)
    (gps.encode(Serial2.read()));

  if (gps.satellites.isUpdated() && (prevgpslat > 0 || prevgpslon > 0)) //updates when gga nmea message is received and previous data is not zero
  {
    //move last gps coordinates to previous position ready for new data
    prevgpslat = gpslat;
    prevgpslon = gpslon;

    gpsprevMillis = gpsMillis;
    gpsMillis = millis();

    gpslatf = gps.location.lat(); // Latitude in degrees float
    gpslonf = gps.location.lng(); // Longitude in degrees float
    gpslatdeg = gps.location.rawLat().deg;
    gpslatbil = gps.location.rawLat().billionths;
    gpslondeg = gps.location.rawLng().deg;
    gpslonbil = gps.location.rawLng().billionths;
    gpsfixage = gps.location.age(); //gps data age

    //make latitude in ten millionths of a degree
    gpslat = (gpslatdeg * 10000000L) + (gpslatbil * 0.01L);

    //make longitude in ten millionths of a degree
    gpslon = (gpslondeg * 10000000L) + (gpslonbil * 0.01L);

    //Serial.println("current coords");
    //Serial.print("gpslat: ");
    //Serial.println(gpslat);
    //Serial.print("gpslon: ");
    //Serial.println(gpslon);

    //Serial.println("previous coords");
    //Serial.print("prevgpslat: ");
    //Serial.println(prevgpslat);
    //Serial.print("prevgpslon: ");
    //Serial.println(prevgpslon);

    gpsspeed = gps.speed.kmph(); // Speed in kilometers per hour float
    gpscourse = gps.course.deg(); // Course in degrees float
    gpssats = gps.satellites.value(); // Number of satellites
    gpshdop = gps.hdop.value(); // Horizontal Dim. of Precision (100ths)

    if (gpsspeed > gpsspeedmax && gpsMillis > 30000) //check and update max speed value only from 30 seconds after start
    {
      gpsspeedmax = gpsspeed;
    }

    //----------------------Process GPS data------------------------------


    //TYPICAL LINE INTERSECTION CALCULATION FORMULA
    /*
     var XAsum = A.LngStart - A.LngEnd;
     var XBsum = B.LngStart - B.LngEnd;
     var YAsum = A.LatStart - A.LatEnd;
     var YBsum = B.LatStart - B.LatEnd;

     var LineDenominator = XAsum * YBsum - YAsum * XBsum;
     if(LineDenominator == 0.0)
     return false;

     var a = A.LngStart * A.LatEnd - A.LatStart * A.LngEnd;
     var b = B.LngStart * B.LatEnd - B.LatStart * B.LngEnd;
     var x = (a * XBsum - b * XAsum) / LineDenominator;
     var y = (a * YBsum - b * YAsum) / LineDenominator;
     */

    XAsumstart = prevgpslon - gpslon;
    YAsumstart = prevgpslat - gpslat;

    d = XAsumstart * YBsumstart - YAsumstart * XBsumstart;

    //-----------------------------------------check if current path and start line is parallel--------------------------------------

    if (d == 0) //the two paths are parallel so havent crossed start line
    {

      //Serial.println("d = 0 so lines parallel");

      if (timerstatus == 1)  //update lap details
      {
        xdeg = (gpslon - prevgpslon) * cos(radians(gpslatf));
        ydeg = gpslat - prevgpslat;
        dist = sqrt(sq(xdeg) + sq(ydeg)) * 11.132; //work out distance travelled since last GPS position (in mm)
        distf = dist * 0.001; //convert dist to a float in metres

        lapdist = lapdist + distf; //update total distance travelled this lap
        laptime = gpsMillis - startcross;  //update elasped lap time since crossed start line
      }
    }

    //----------------------------------else lines not parallel so find where they intersect--------------------------------------------------

    else
    {
      //Serial.println("d not 0 so line may cross");

      //calculate position where the paths would intersect
      a = ((long long)prevgpslon * (long long)gpslat - (long long)prevgpslat * (long long)gpslon);
      b = ((long long)start1lon * (long long)start2lat - (long long)start1lat * (long long)start2lon);

      startlon = (a * (long long)XBsumstart - b * (long long)XAsumstart) / (long long)d;  //intersection longitude
      startlat = (a * (long long)YBsumstart - b * (long long)YAsumstart) / (long long)d;  //intersection latitude

      //check if the intesection point is on the paths
      intersectlat = ((long long)prevgpslat - startlat ) * (startlat  - (long long)gpslat);
      intersectlon = ((long long)prevgpslon - startlon ) * (startlon  - (long long)gpslon);

      //---------------------------------------------if start line crossed---------------------------------------------------

      if (intersectlat >= 10 && intersectlon >= 10 && (gpsMillis - startcross) > 5000) //if intesectlat and intersectlong greater than 0 then paths coincided... have got a few false triggers so timer added and playing with increasing values to try and fix.
      {
        //Serial.println("start line crossed!");

        //work out distance from last gps point to start line
        xdeg = (startlon - prevgpslon) * cos(radians(gpslatf));
        ydeg = startlat - prevgpslat;
        dist1 = sqrt(sq(xdeg) + sq(ydeg)) * 11.132; //work out distance travelled since last GPS position (in mm)
        dist1f = (float)dist1 * 0.001; //convert dist to a float in metres

        //work out distance from start line to current gps point
        xdeg = (gpslon - startlon) * cos(radians(gpslatf));
        ydeg = gpslat - startlat;
        dist2 = sqrt(sq(xdeg) + sq(ydeg)) * 11.132; //work out distance travelled since last GPS position (in mm)
        dist2f = (float)dist2 * 0.001; //convert dist to a float in metres

        finishcross = gpsprevMillis + ((gpsMillis - gpsprevMillis) * (dist1 / (dist1 + dist2))); //calculate the time in millis the finish line was crossed

        if (timerstatus == 1)  //complete current lap
        {
          prevlapdist = lapdist + dist1f; //complete previous lap distance
          prevlaptime = finishcross - startcross;  //complete lap time as previous lap
          prevlaptimef = (float)prevlaptime * 0.001;
        }

        lapdist = dist2f; //start new lap dist
        startcross = finishcross;  //update the time the start line was crossed
        laptime = gpsMillis - startcross;  //update elasped lap time since crossed start line
        lapnumber = lapnumber + 1;   //update lap number

        //ensure timer is running if crossing time for first time
        timerstatus = 1;
      }

      //-------------------------------------------if start line not crossed---------------------------------------------------

      else
      {
        //Serial.println("start line not crossed");

        if (timerstatus == 1)  //update lap details
        {
          //Serial.println("calculating distance");

          //update lap distance
          xdeg = (gpslon - prevgpslon) * cos(radians(gpslatf));
          ydeg = gpslat - prevgpslat;
          dist = sqrt(sq(xdeg) + sq(ydeg)) * 11.132; //work out distance travelled since last GPS position (in mm)
          distf = dist * 0.001; //convert dist to a float in metres

          lapdist = lapdist + distf; //update total distance travelled this lap
          laptime = gpsMillis - startcross;  //update elasped lap time since crossed start line
        }
      }
    }

    //calculate float values for logging and display on the LCD
    laptimef = laptime * 0.001;
    gpshdopf = gpshdop * 0.01;

    //-------------------------------------check if pit entry crossed--------------------------------------------

    XAsumpit = prevgpslon - gpslon;
    YAsumpit = prevgpslat - gpslat;

    d = XAsumpit * YBsumpit - YAsumpit * XBsumpit;

    if ( d != 0)  //if the two paths arent parallel

    {
      a = (long long)prevgpslon * (long long)gpslat - (long long)prevgpslat * (long long)gpslon;
      b = (long long)pit1lon * (long long)pit2lat - (long long)pit1lat * (long long)pit2lon;

      pitlon = (a * (long long)XBsumpit - b * (long long)XAsumpit) / (long long)d;
      pitlat = (a * (long long)YBsumpit - b * (long long)YAsumpit) / (long long)d;

      intersectlat = ((long long)prevgpslat - pitlat ) * (pitlat  - (long long)gpslat);
      intersectlon = ((long long)prevgpslon - pitlon ) * (pitlon  - (long long)gpslon);

      if (intersectlat >= 10 && intersectlon >= 10) //paths intersect, pit entry has been crossed (values should be greater than or equal to zero but trying to stop false triggers)
      {
        //stop timer and reset lap time and distance
        timerstatus = 0;
        lapdist = 0;
        laptime = 0;

        // Serial.println("pit line crossed.............");
      }
    }

    //make sure lap dist is within range that will display correctly on LCD
    if (lapdist > 9999)
    {
      lapdist = 9999;
    }
    if (lapdist < 0)
    {
      lapdist = 0;
    }


    //-------------------------------------datalog to SD Card--------------------------------------------

    //create data string to write to SD card
    dataString = "";
    dataString += String(gps.date.year());
    dataString += "/";
    dataString += String(gps.date.month());
    dataString += "/";
    dataString += String(gps.date.day());
    dataString += " ";
    dataString += String(gps.time.hour());
    dataString += ":";
    dataString += String(gps.time.minute());
    dataString += ":";
    dataString += String(gps.time.second());
    dataString += ".";
    dataString += String(gps.time.centisecond());
    dataString += ",";
    dataString += String(gpsMillis);
    dataString += ",";
    dataString += String(gps.location.age());
    dataString += ",";
    dataString += String(gps.location.rawLat().negative ? "-" : "+");
    dataString += String(gps.location.rawLat().deg);
    dataString += ".";
    dataString += String(gps.location.rawLat().billionths);
    dataString += ",";
    dataString += String(gps.location.rawLng().negative ? "-" : "+");
    dataString += String(gps.location.rawLng().deg);
    dataString += ".";
    dataString += String(gps.location.rawLng().billionths);
    dataString += ",";
    dataString += String(lapnumber);
    dataString += ",";
    dataString += String(laptimef, 2);
    dataString += ",";
    dataString += String(prevlaptimef, 2);
    dataString += ",";
    dataString += String(lapdist, 1);
    dataString += ",";
    dataString += String(gpsspeed, 1);
    dataString += ",";
    dataString += String(gpscourse, 0);
    dataString += ",";
    dataString += String(gpssats);
    dataString += ",";
    dataString += String(gpshdopf, 1);
    dataString += ",";
    dataString += String(rpm);
    dataString += ",";
    dataString += String(load);
    dataString += ",";
    dataString += String(throttle);
    dataString += ",";
    dataString += String(injms, 2);
    dataString += ",";
    dataString += String(injduty);
    dataString += ",";
    dataString += String(ignadv);
    dataString += ",";
    dataString += String(manipressg);
    dataString += ",";
    dataString += String(PSIboost, 2);
    dataString += ",";
    dataString += String(airtemp);
    dataString += ",";
    dataString += String(engtemp);
    dataString += ",";
    dataString += String(volt, 2);
    dataString += ",";
    dataString += String(afr, 1);
    dataString += ",";
    dataString += String(idlevlv);
    dataString += ",";
    dataString += String(oilpress);
    dataString += ",";
    dataString += String(fuelpress);
    dataString += ",";
    dataString += String(rel_fuelpress);
    dataString += ",";
    dataString += String(brakeson);
    dataString += ",";
    dataString += String(alarm);
    dataString += ",";
    dataString += String(loopcount);

    dataFile.println(dataString);  //write data string to SD card

    //Serial.println(dataString);  // print data string to the serial port

    dataFile.flush();

    loopcount = 0;  //rest loop count
  }


  //---------------if previous gps data fields are empty fill them before trying to check for start or pit lines-----------------

  else if (gps.satellites.isUpdated() && prevgpslat == 0  && prevgpslon == 0)
  {
    //move last gps coordinates to previous position ready for new data
    prevgpslat = gpslat;
    prevgpslon = gpslon;

    gpslatdeg = gps.location.rawLat().deg;
    gpslatbil = gps.location.rawLat().billionths;
    gpslondeg = gps.location.rawLng().deg;
    gpslonbil = gps.location.rawLng().billionths;

    //make latitude in ten millionths of a degree
    gpslat = (gpslatdeg * 10000000L) + (gpslatbil * 0.01L);

    //make longitude in ten millionths of a degree
    gpslon = (gpslondeg * 10000000L) + (gpslonbil * 0.01L);
  }

  //---------------------------------Get data from wolf ecu---------------------------------------

  //empty serial buffer if not expecting data
  if (Serial1.available() > 0 && wolf1update == 0  && wolf2update == 0)  // there are bytes in the serial buffer to read
  {
    while (Serial1.available() > 0)  // every time a byte is read it is expunged from the serial buffer so keep reading the buffer until all the bytes have been read.
    {
      readbyte = Serial1.read(); // read serial data
    }
  }

  currentMillis = millis();


  //send request for packet 0 data
  if ((currentMillis - wolf1previousMillis > wolf1interval) && wolf2update == 0 && wolfcount > 50) //check if wolf 1 data update interval has expired
  {
    wolf1previousMillis = millis();  //save the last time the wolf was refreshed
    wolf1update = 1;  //indicates wolf1data array is incomplete
    Serial1.write(wolf1request, 4); //Send data to the Wolf ECU on serial1 to initiate wolf packet 0 data transfer
    //Serial.write(wolf1request, 4); //Send data to serial0 for debugging
    //Serial.print('\n'); //New line to serial0
  }


  //send request for packet 2 data
  if ((currentMillis - wolf2previousMillis > wolf2interval) && wolf1update == 0 && wolfcount > 50) //check if wolf 2 data update interval has expired
  {
    wolf2previousMillis = millis();  //save the last time the wolf was refreshed
    wolf2update = 1;  //indicates wolf1data array is incomplete
    Serial1.write(wolf2request, 4); //Send data to the Wolf ECU on serial1 to initiate wolf packet 0 data transfer
    //Serial.write(wolf2request, 4); //Send data to serial0 for debugging
    //Serial.print('\n'); //New line to serial0
  }


  // Read packet 0 response from ECU
  if (Serial1.available() > 31 && wolf1update == 1)  // wait until the full message has be received in the buffer
  {
    wolfcounter = 0;  //Reset received byte counter before new data received

    while (Serial1.available() > 0)  // every time a byte is read it is expunged from the serial buffer so keep reading the buffer until all the bytes have been read.
    {
      readbyte = Serial1.read();
      wolfdata[wolfcounter] = readbyte;  //Add the revieved byte to the array its stored in
      wolfcounter++;  //Add 1 to the counter so the next received byte goes into the next cell in the array
    }

    rpm = (wolfdata[0] * 256) + wolfdata[1];  //calculate RPM from hex data
    load = wolfdata[2];  //calculate load from hex data
    throttle = wolfdata[3] / 2.3923;  //calculate throttle position from hex data
    PSIboost = ((wolfdata[4] * 256.0) + wolfdata[5]) / 10.0;  //calculate psi boost from hex data
    injms = ((wolfdata[6] * 256.00) + wolfdata[7]) / 125.00;  //calculate injection duration from hex data
    injduty = wolfdata[10] / 2.52545;  //calculate injector duty from hex data
    ignadv = (wolfdata[11] / 2.845) - 11.243;  //calculate ignition advance from hex data
    manipressg = ((wolfdata[12] / 2.3935) * 2) - 100;  //calculate MAP pressure from hex data
    airtemp = wolfdata[14] - 64;  //calculate intake air temp from hex data
    engtemp = wolfdata[15] - 64;  //calculate coolant temp from hex data
    volt = wolfdata[16] / 10.0;  //calculate voltage from hex data
    afr = wolfdata[18] / 10.0;  //calculate wideband afr from hex data
    idlevlv = ((wolfdata[24] * 256) + wolfdata[25]) * 0.39216;  //calculate idle valve position from hex data

    //check if min/max values have been exceeded.  wait 30 seconds after to start.
    if (rpm > rpmmax && gpsMillis > 30000)
    {
      rpmmax = rpm;
    }

    if (volt < voltmin && gpsMillis > 30000)
    {
      voltmin = volt;
    }

    if (manipressg > manipressgmax && gpsMillis > 30000)
    {
      manipressgmax = manipressg;
    }

    if (airtemp > airtempmax && gpsMillis > 30000)
    {
      airtempmax = airtemp;
    }

    brakeson = digitalRead(7);  //update status of brakes

    wolf1update = 0;
    wolfcounter = 0;
    wolfcount = 0;

    /* log raw incoming packet 0 serial data to log file
        dataString = "Wolfdata1: ";
        for (int i = 0; i < 33; i++)
        {
          dataString += String(wolfdata[i], DEC);
        }
        dataFile.println(dataString);
        */
  }


  if (Serial1.available() > 31 && wolf2update == 1)  // wait until the full message has be received in the buffer
  {
    wolfcounter = 0;  //Reset received byte counter before new data received

    while (Serial1.available() > 0)  // every time a byte is read it is expunged from the serial buffer so keep reading the buffer until all the bytes have been read.
    {
      readbyte = Serial1.read();
      wolfdata[wolfcounter] = readbyte;  //Add the revieved byte to the array its stored in
      wolfcounter++;  //Add 1 to the counter so the next received byte goes into the next cell in the array
    }

    fuelpress = ((wolfdata[10] * 256.00) + wolfdata[11]);  //get the LS2 data from packet 2
    if (fuelpress > 2000)
    {
      fuelpress = fuelpress - 40959; //Negative output is offset 40959 so this corrects output to a range between -1023 to 1023
    }

    fuelpress = fuelpress * 1.010753;  //Convert to kPa  1034 kPa / 1023 (negative range not used in this case)
    rel_fuelpress = fuelpress - manipressg;   //Calculate the fuel pressure relative to manifold pressure
    oilpress = ((wolfdata[12] * 256.00) + wolfdata[13]);  //get the LS3 data from packet 2

    if (oilpress > 2000)
    {
      oilpress = oilpress - 40959; //Negative output is offset 40959 so this corrects output to a range between -1023 to 1023
    }

    oilpress = oilpress * 1.010753;  //Convert to kPa  1034 kPa / 1023 (negative range not used in this case)
    if (oilpress < oilpressmin && gpsMillis > 60000)
    {
      oilpressmin = oilpress;
    }

    if (rel_fuelpress < rel_fuelpressmin && gpsMillis > 60000)
    {
      rel_fuelpressmin = rel_fuelpress;
    }
    wolf2update = 0;
    wolfcounter = 0;
    wolfcount = 0;

    /* log raw incoming packet 2 serial data to log file
    dataString = "Wolfdata2: ";
    for (int i = 0; i < 33; i++)
    {
      dataString += String(wolfdata[i], DEC);
    }
    dataFile.println(dataString);
    */
  }

  //---------------------------Page up scroll button----------------------------------------

  button1.Update(); //update the state of button1 (page scroll up)

  if (button1.clicks == 1)
  {
    pagenumber = pagenumber - 1;
    pageupdate = 1;
  }

  //---------------------------Page down scroll button----------------------------------------


  button2.Update(); //update the state of button2 (page scroll down)

  if (button2.clicks == 1)
  {
    pagenumber = pagenumber + 1;
    pageupdate = 1;
  }

  if (pagenumber == 5) pagenumber = 0;
  if (pagenumber == -1) pagenumber = 4;

  //---------------------------Line scroll button----------------------------------------


  button3.Update(); //update the state of button3 (line scroll)

  if (button3.clicks == 1)  lineselect = lineselect + 1;

  if (lineselect == 3) lineselect = 0;

  button4.Update(); //update the state of button4 (enter)

  //-------------------------Enter button: set start line---------------------------------

  if (button4.clicks == 1 && pagenumber == 4 && lineselect == 0)
  {
    dlat = gpslat - prevgpslat;
    dlon = gpslon - prevgpslon;

    xdeg = dlon * cos(radians(gpslatf));
    ydeg = dlat;
    dist1 = sqrt(sq(xdeg) + sq(ydeg)) * 11.132; //work out distance travelled since last GPS position (in mm)
    factor = 10000 / dist1;  //work out factor required to get coordinates of start line 20m apart

    //work out start line coords
    start1lat = -dlon * factor + gpslat;
    start1lon = dlat * factor + gpslon;
    start2lat = dlon * factor + gpslat;
    start2lon = -dlat * factor + gpslon;

    //work out variables used to check if start line crossed
    XBsumstart = start1lon - start2lon;
    YBsumstart = start1lat - start2lat;

    //write coords to eeprom
    EEPROMWritelong(0, start1lat);
    EEPROMWritelong(4, start1lon);
    EEPROMWritelong(8, start2lat);
    EEPROMWritelong(12, start2lon);

    pagenumber = 1;
    pageupdate = 1;
  }

  //-------------------------Enter button: set pit entry---------------------------------

  if (button4.clicks == 1 && pagenumber == 4 && lineselect == 1)
  {
    dlat = gpslat - prevgpslat;
    dlon = gpslon - prevgpslon;

    xdeg = dlon * cos(radians(gpslatf));
    ydeg = dlat;
    dist1 = sqrt(sq(xdeg) + sq(ydeg)) * 11.132; //work out distance travelled since last GPS position (in mm)
    factor = 5000 / dist1;  //work out factor required to get coordinates of pit line 10m apart

    //work out pit line coords
    pit1lat = -dlon * factor + gpslat;
    pit1lon = dlat * factor + gpslon;
    pit2lat = dlon * factor + gpslat;
    pit2lon = -dlat * factor + gpslon;

    //work out variables used to check if pit line crossed
    XBsumpit = pit1lon - pit2lon;
    YBsumpit = pit1lat - pit2lat;

    //write coords to eeprom
    EEPROMWritelong(16, pit1lat);
    EEPROMWritelong(20, pit1lon);
    EEPROMWritelong(24, pit2lat);
    EEPROMWritelong(28, pit2lon);

    pagenumber = 1;
    pageupdate = 1;
  }

  //-------------------------Enter button: reset session number counter---------------------------------

  if (button4.clicks == 1 && pagenumber == 4 && lineselect == 2)
  {
    //reset the session number counter used for naming the log files
    sessionno = 0;
    EEPROMWritelong(32, sessionno);
    pagenumber = 1;
    pageupdate = 1;
  }

  //----------------------------------Refresh LCD display--------------------------------

  if (currentMillis - LCDpreviousMillis > LCDinterval) //check if LCD update interval has expired
  {
    LCDpreviousMillis = currentMillis;  //save the last time the LCD was refreshed

    //---------------------Print the fixed text to the LCD display------------------------

    if (pageupdate == 1)
    {

      if (pagenumber == 0)
      {

        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("kph");
        lcd.setCursor(11, 0);
        lcd.print("rpm");
        lcd.setCursor(0, 1);
        lcd.print("FUEL:");
        lcd.setCursor(8, 1);
        lcd.print("kPa");
        lcd.setCursor(0, 2);
        lcd.print("OIL:");
        lcd.setCursor(7, 2);
        lcd.print("kPa");
        lcd.setCursor(0, 3);
        lcd.print("MAP:");
        lcd.setCursor(7, 3);
        lcd.print("kPa");
        lcd.setCursor(11, 3);
        lcd.print("IAT:");
        lcd.setCursor(18, 3);
        lcd.print(char(223));
        lcd.setCursor(19, 3);
        lcd.print("C");
        lcd.setCursor(11, 2);
        lcd.print("ENG:");
        lcd.setCursor(18, 2);
        lcd.print(char(223));
        lcd.setCursor(19, 2);
        lcd.print("C");
        lcd.setCursor(19, 0);
        lcd.print("V");
        lcd.setCursor(12, 1);
        lcd.print("AFR:");
      }

      else if (pagenumber == 1)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("LAP#:");
        lcd.setCursor(10, 0);
        lcd.print("DIST:");
        lcd.setCursor(19, 0);
        lcd.print("m");
        lcd.setCursor(0, 1);
        lcd.print("SATS:");
        lcd.setCursor(10, 1);
        lcd.print("HDOP:");
        lcd.setCursor(1, 2);
        lcd.print("CURRENT LAP:");
        lcd.setCursor(19, 2);
        lcd.print("s");
        lcd.setCursor(4, 3);
        lcd.print("LAST LAP:");
        lcd.setCursor(19, 3);
        lcd.print("s");
      }

      else if (pagenumber == 2)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("INJ:");
        lcd.setCursor(8, 0);
        lcd.print("ms");
        lcd.setCursor(12, 0);
        lcd.print("THR:");
        lcd.setCursor(19, 0);
        lcd.print("%");
        lcd.setCursor(0, 1);
        lcd.print("IGN:");
        lcd.setCursor(7, 1);
        lcd.print(char(223));
        lcd.setCursor(8, 1);
        lcd.print("B");
        lcd.setCursor(11, 1);
        lcd.print("LOAD:");
        lcd.setCursor(19, 1);
        lcd.print("%");
        lcd.setCursor(0, 2);
        lcd.print("INDUTY:");
        lcd.setCursor(9, 2);
        lcd.print("%");
        lcd.setCursor(12, 2);
        lcd.print("IDLE:");
        lcd.setCursor(19, 2);
        lcd.print("%");
        lcd.setCursor(0, 3);
        lcd.print("BST:");
        lcd.setCursor(8, 3);
        lcd.print("psi");
        lcd.setCursor(12, 3);
        lcd.print("BSTV:");
        lcd.setCursor(19, 3);
        lcd.print("%");
      }

      else if (pagenumber == 3)
      {
        lcd.clear();
        lcd.setCursor(2, 0);
        lcd.print("SESSION MIN/MAX");
        lcd.setCursor(3, 1);
        lcd.print("kph");
        lcd.setCursor(11, 1);
        lcd.print("rpm");
        lcd.setCursor(19, 1);
        lcd.print("V");
        lcd.setCursor(0, 2);
        lcd.print("OIL:");
        lcd.setCursor(7, 2);
        lcd.print("kPa");
        lcd.setCursor(11, 2);
        lcd.print("FP:");
        lcd.setCursor(17, 2);
        lcd.print("kPa");
        lcd.setCursor(0, 3);
        lcd.print("MAP:");
        lcd.setCursor(7, 3);
        lcd.print("kPa");
        lcd.setCursor(11, 3);
        lcd.print("IAT:");
        lcd.setCursor(18, 3);
        lcd.print(char(223));
        lcd.setCursor(19, 3);
        lcd.print("C");
      }

      pageupdate = 0;
    }


    //---------------------------------LCD page 1 data - Main engine data-----------------------------------------

    if (pagenumber == 0) //if pageswitch equals 0 display main engine operating data
    {
      lcd.setCursor(3, 0);
      lcd.print("kph");
      lcd.setCursor(11, 0);
      lcd.print("rpm");
      lcd.setCursor(0, 1);
      lcd.print("FUEL:");
      lcd.setCursor(8, 1);
      lcd.print("kPa");
      lcd.setCursor(0, 2);
      lcd.print("OIL:");
      lcd.setCursor(7, 2);
      lcd.print("kPa");
      lcd.setCursor(0, 3);
      lcd.print("MAP:");
      lcd.setCursor(7, 3);
      lcd.print("kPa");
      lcd.setCursor(11, 3);
      lcd.print("IAT:");
      lcd.setCursor(18, 3);
      lcd.print(char(223));
      lcd.setCursor(19, 3);
      lcd.print("C");
      lcd.setCursor(11, 2);
      lcd.print("ENG:");
      lcd.setCursor(18, 2);
      lcd.print(char(223));
      lcd.setCursor(19, 2);
      lcd.print("C");
      lcd.setCursor(19, 0);
      lcd.print("V");
      lcd.setCursor(12, 1);
      lcd.print("AFR:");

      //Print speed value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(0, 0);
      lcd.print("   ");

      if (gpsspeed < 10)
      {
        lcd.setCursor(2, 0);
        lcd.print(gpsspeed, 0);
      }

      else if (gpsspeed < 100)
      {
        lcd.setCursor(1, 0);
        lcd.print(gpsspeed, 0);
      }

      else
      {
        lcd.setCursor(0, 0);
        lcd.print(gpsspeed, 0);
      }

      //Print rpm value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(6, 0);
      lcd.print("     ");

      if (rpm < 10)
      {
        lcd.setCursor(10, 0);
        lcd.print(rpm);
      }

      else if (rpm < 100)
      {
        lcd.setCursor(9, 0);
        lcd.print(rpm);
      }

      else if (rpm < 1000)
      {
        lcd.setCursor(8, 0);
        lcd.print(rpm);
      }

      else if (rpm < 10000)
      {
        lcd.setCursor(7, 0);
        lcd.print(rpm);
      }

      else
      {
        lcd.setCursor(6, 0);
        lcd.print(rpm);
      }

      //Print rel_fuelpress value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(5, 1);
      lcd.print("   ");

      if (rel_fuelpress < 10)
      {
        lcd.setCursor(7, 1);
        lcd.print(rel_fuelpress);
      }

      else if (rel_fuelpress < 100)
      {
        lcd.setCursor(6, 1);
        lcd.print(rel_fuelpress);
      }

      else
      {
        lcd.setCursor(5, 1);
        lcd.print(rel_fuelpress);
      }

      //Print oilpress value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(4, 2);
      lcd.print("   ");

      if (oilpress < 10)
      {
        lcd.setCursor(6, 2);
        lcd.print(oilpress);
      }

      else if (oilpress < 100)
      {
        lcd.setCursor(5, 2);
        lcd.print(oilpress);
      }

      else
      {
        lcd.setCursor(4, 2);
        lcd.print(oilpress);
      }

      //Print manipressg value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(4, 3);
      lcd.print("   ");

      if (manipressg < -99)
      {
        manipressg = -99;
      }

      if (manipressg < -9)
      {
        lcd.setCursor(4, 3);
        lcd.print(manipressg);
      }

      else if (manipressg < 0)
      {
        lcd.setCursor(5, 3);
        lcd.print(manipressg);
      }

      else if (manipressg < 10)
      {
        lcd.setCursor(6, 3);
        lcd.print(manipressg);
      }

      else if (manipressg < 100)
      {
        lcd.setCursor(5, 3);
        lcd.print(manipressg);
      }

      else
      {
        lcd.setCursor(4, 3);
        lcd.print(manipressg);
      }

      //Print airtemp value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(15, 3);
      lcd.print("   ");

      if (airtemp < 10)
      {
        lcd.setCursor(17, 3);
        lcd.print(airtemp);
      }

      else if (airtemp < 100)
      {
        lcd.setCursor(16, 3);
        lcd.print(airtemp);
      }

      else
      {
        lcd.setCursor(15, 3);
        lcd.print(airtemp);
      }

      //Print engtemp value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(15, 2);
      lcd.print("   ");

      if (engtemp < 10)
      {
        lcd.setCursor(17, 2);
        lcd.print(engtemp);
      }

      else if (engtemp < 100)
      {
        lcd.setCursor(16, 2);
        lcd.print(engtemp);
      }

      else
      {
        lcd.setCursor(15, 2);
        lcd.print(engtemp);
      }

      //Print volt value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(15, 0);
      lcd.print("    ");

      if (volt < 10)
      {
        lcd.setCursor(16, 0);
        lcd.print(volt, 1);
      }

      else
      {
        lcd.setCursor(15, 0);
        lcd.print(volt, 1);
      }

      //Print afr value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(16, 1);
      lcd.print("    ");

      if (afr < 10)
      {
        lcd.setCursor(17, 1);
        lcd.print(afr, 1);
      }

      else
      {
        lcd.setCursor(16, 1);
        lcd.print(afr, 1);
      }
    }

    //------------------------------LCD page 2 data- GPS and lap data---------------------------------

    else if (pagenumber == 1)  //if pageswitch equals 1 display GPS and lap data screen
    {
      lcd.setCursor(0, 0);
      lcd.print("LAP#:");
      lcd.setCursor(10, 0);
      lcd.print("DIST:");
      lcd.setCursor(19, 0);
      lcd.print("m");
      lcd.setCursor(0, 1);
      lcd.print("SATS:");
      lcd.setCursor(10, 1);
      lcd.print("HDOP:");
      lcd.setCursor(1, 2);
      lcd.print("CURRENT LAP:");
      lcd.setCursor(19, 2);
      lcd.print("s");
      lcd.setCursor(4, 3);
      lcd.print("LAST LAP:");
      lcd.setCursor(19, 3);
      lcd.print("s");

      //print lap number to LCD

      lcd.setCursor(5, 0);
      lcd.print(lapnumber);

      //print lap distance to LCD

      lcd.setCursor(15, 0);
      lcd.print("    ");

      if (lapdist < 10)
      {
        lcd.setCursor(18, 0);
        lcd.print(lapdist, 0);
      }

      else if (lapdist < 100)
      {
        lcd.setCursor(17, 0);
        lcd.print(lapdist, 0);
      }

      else if (lapdist < 1000)
      {
        lcd.setCursor(16, 0);
        lcd.print(lapdist, 0);
      }

      else
      {
        lcd.setCursor(15, 0);
        lcd.print(lapdist, 0);
      }

      //print number of sats to LCD

      lcd.setCursor(5, 1);
      lcd.print("  ");

      lcd.setCursor(5, 1);
      lcd.print(gpssats);

      //print HDOP to LCD

      lcd.setCursor(15, 1);
      lcd.print("     ");

      lcd.setCursor(15, 1);
      lcd.print(gpshdopf, 1);

      //print current lap time to LCD

      lcd.setCursor(13, 2);
      lcd.print("      ");

      if (laptimef < 10)
      {
        lcd.setCursor(16, 2);
        lcd.print(laptimef, 1);
      }

      else if (laptimef < 100)
      {
        lcd.setCursor(15, 2);
        lcd.print(laptimef, 1);
      }

      else if (laptimef < 1000)
      {
        lcd.setCursor(14, 2);
        lcd.print(laptimef);
      }

      else
      {
        lcd.setCursor(13, 2);
        lcd.print(laptimef);
      }

      //print last lap time to LCD

      lcd.setCursor(13, 3);
      lcd.print("      ");

      if (prevlaptimef < 10)
      {
        lcd.setCursor(16, 3);
        lcd.print(prevlaptimef, 1);
      }

      else if (prevlaptimef < 100)
      {
        lcd.setCursor(15, 3);
        lcd.print(prevlaptimef, 1);
      }

      else if (prevlaptimef < 1000)
      {
        lcd.setCursor(14, 3);
        lcd.print(prevlaptimef);
      }

      else
      {
        lcd.setCursor(13, 3);
        lcd.print(prevlaptimef);
      }
    }

    //---------------------------------LCD page 3 data - Alternative engine data-------------------------------

    else if (pagenumber == 2) //if pageswitch equals 2 display alternate data
    {
      lcd.setCursor(0, 0);
      lcd.print("INJ:");
      lcd.setCursor(8, 0);
      lcd.print("ms");
      lcd.setCursor(12, 0);
      lcd.print("THR:");
      lcd.setCursor(19, 0);
      lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print("IGN:");
      lcd.setCursor(7, 1);
      lcd.print(char(223));
      lcd.setCursor(8, 1);
      lcd.print("B");
      lcd.setCursor(11, 1);
      lcd.print("LOAD:");
      lcd.setCursor(19, 1);
      lcd.print("%");
      lcd.setCursor(0, 2);
      lcd.print("INDUTY:");
      lcd.setCursor(9, 2);
      lcd.print("%");

      lcd.setCursor(12, 2);
      lcd.print("IDLE:");
      lcd.setCursor(19, 2);
      lcd.print("%");
      lcd.setCursor(0, 3);
      lcd.print("BST:");
      lcd.setCursor(8, 3);
      lcd.print("psi");
      lcd.setCursor(12, 3);
      lcd.print("BSTV:");
      lcd.setCursor(19, 3);
      lcd.print("%");

      //Print fuel injection timing
      //Clear old value, work out correct text position to right justify and print to LCD

      lcd.setCursor(4, 0);
      lcd.print("    ");

      if (injms < 10)
      {
        lcd.setCursor(5, 0);
        lcd.print(injms, 1);
      }

      else
      {
        lcd.setCursor(4, 0);
        lcd.print(injms, 1);
      }

      //Print throttle position value
      //Clear old value, work out correct text position to right justify and print to LCD

      lcd.setCursor(16, 0);
      lcd.print("   ");

      if (throttle < 10)
      {
        lcd.setCursor(18, 0);
        lcd.print(throttle);
      }

      else if (throttle < 100)
      {
        lcd.setCursor(17, 0);
        lcd.print(throttle);
      }

      else
      {
        lcd.setCursor(16, 0);
        lcd.print(throttle);
      }

      //Print ignition timing value
      //Clear old value, work out correct text position to right justify and print to LCD

      lcd.setCursor(4, 1);
      lcd.print("   ");

      if (ignadv < -10)
      {
        lcd.setCursor(4, 1);
        lcd.print(ignadv);
      }

      else if (ignadv < 0)
      {
        lcd.setCursor(5, 1);
        lcd.print(ignadv);
      }

      else if (ignadv < 10)
      {
        lcd.setCursor(6, 1);
        lcd.print(ignadv);
      }

      else if (ignadv < 100)
      {
        lcd.setCursor(5, 1);
        lcd.print(ignadv);
      }

      else
      {
        lcd.setCursor(4, 1);
        lcd.print(ignadv);
      }

      //Print load value
      //Clear old value, work out correct text position to right justify and print to LCD

      lcd.setCursor(16, 1);
      lcd.print("   ");

      if (load < 10)
      {
        lcd.setCursor(18, 1);
        lcd.print(load);
      }

      else if (load < 100)
      {
        lcd.setCursor(17, 1);
        lcd.print(load);
      }

      else
      {
        lcd.setCursor(16, 1);
        lcd.print(load);
      }

      //Print injector duty value
      //Clear old value, work out correct text position to right justify and print to LCD

      lcd.setCursor(7, 2);
      lcd.print("  ");

      if (injduty < 10)
      {
        lcd.setCursor(8, 2);
        lcd.print(injduty);
      }

      else
      {
        lcd.setCursor(7, 2);
        lcd.print(injduty);
      }

      //Print idle valve value
      //Clear old value, work out correct text position to right justify and print to LCD

      lcd.setCursor(17, 2);
      lcd.print("  ");

      if (idlevlv < 10)
      {
        lcd.setCursor(18, 2);
        lcd.print(idlevlv);
      }

      else
      {
        lcd.setCursor(17, 2);
        lcd.print(idlevlv);
      }

      //Print psi boost value
      //Clear old value, work out correct text position to right justify and print to LCD

      lcd.setCursor(4, 3);
      lcd.print("  ");

      if (PSIboost < 10)
      {
        lcd.setCursor(5, 3);
        lcd.print(PSIboost, 1);
      }

      else
      {
        lcd.setCursor(4, 3);
        lcd.print(PSIboost, 1);
      }

      //Print boost valve value
      //Clear old value, work out correct text position to right justify and print to LCD

      lcd.setCursor(17, 3);
      lcd.print("  ");

      if (boostvlv < 10)
      {
        lcd.setCursor(18, 3);
        lcd.print(boostvlv);
      }

      else
      {
        lcd.setCursor(17, 3);
        lcd.print(boostvlv);
      }
    }

    //-----------------------------------------LCD page 4 data - min / max engine session values---------------------------------------------

    else if (pagenumber == 3) //if pageswitch equals 3 display session max values
    {
      lcd.setCursor(2, 0);
      lcd.print("SESSION MIN/MAX");
      lcd.setCursor(3, 1);
      lcd.print("kph");
      lcd.setCursor(11, 1);
      lcd.print("rpm");
      lcd.setCursor(19, 1);
      lcd.print("V");
      lcd.setCursor(0, 2);
      lcd.print("OIL:");
      lcd.setCursor(7, 2);
      lcd.print("kPa");
      lcd.setCursor(11, 2);
      lcd.print("FP:");
      lcd.setCursor(17, 2);
      lcd.print("kPa");
      lcd.setCursor(0, 3);
      lcd.print("MAP:");
      lcd.setCursor(7, 3);
      lcd.print("kPa");
      lcd.setCursor(11, 3);
      lcd.print("IAT:");
      lcd.setCursor(18, 3);
      lcd.print(char(223));
      lcd.setCursor(19, 3);
      lcd.print("C");

      //Print max speed value
      //Work out correct text position to right justify and print to LCD

      lcd.setCursor(0, 1);
      lcd.print("   ");

      if (gpsspeedmax < 10)
      {
        lcd.setCursor(2, 1);
        lcd.print(gpsspeedmax, 0);
      }

      else if (gpsspeedmax < 100)
      {
        lcd.setCursor(1, 1);
        lcd.print(gpsspeedmax, 0);
      }

      else
      {
        lcd.setCursor(0, 1);
        lcd.print(gpsspeedmax, 0);
      }

      //Print max rpm value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(6, 1);
      lcd.print("     ");

      if (rpmmax < 10)
      {
        lcd.setCursor(10, 1);
        lcd.print(rpmmax);
      }

      else if (rpmmax < 100)
      {
        lcd.setCursor(9, 1);
        lcd.print(rpmmax);
      }

      else if (rpmmax < 1000)
      {
        lcd.setCursor(8, 1);
        lcd.print(rpmmax);
      }

      else if (rpmmax < 10000)
      {
        lcd.setCursor(7, 1);
        lcd.print(rpmmax);
      }

      else
      {
        lcd.setCursor(6, 1);
        lcd.print(rpmmax);
      }

      //Print min volt value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(15, 1);
      lcd.print("    ");

      if (voltmin < 10)
      {
        lcd.setCursor(16, 1);
        lcd.print(voltmin, 1);
      }

      else
      {
        lcd.setCursor(15, 1);
        lcd.print(voltmin, 1);
      }

      //Print min oilpress value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(4, 2);
      lcd.print("   ");

      if (oilpressmin < 10)
      {
        lcd.setCursor(6, 2);
        lcd.print(oilpressmin);
      }

      else if (oilpressmin < 100)
      {
        lcd.setCursor(5, 2);
        lcd.print(oilpressmin);
      }

      else
      {
        lcd.setCursor(4, 2);
        lcd.print(oilpressmin);
      }

      //Print min relative fuel press value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(14, 2);
      lcd.print("   ");

      if (rel_fuelpressmin < 10)
      {
        lcd.setCursor(16, 2);
        lcd.print(rel_fuelpressmin);
      }

      else if (rel_fuelpressmin < 100)
      {
        lcd.setCursor(15, 2);
        lcd.print(rel_fuelpressmin);
      }

      else
      {
        lcd.setCursor(14, 2);
        lcd.print(rel_fuelpressmin);
      }

      //Print max manipressg value
      //Clear old value, work out correct text position to right justify and print to LCD

      lcd.setCursor(4, 3);
      lcd.print("   ");

      if (manipressgmax < -99)
      {
        manipressgmax = -99;
      }

      if (manipressgmax < -9)
      {
        lcd.setCursor(4, 3);
        lcd.print(manipressgmax);
      }

      else if (manipressgmax < 0)
      {
        lcd.setCursor(5, 3);
        lcd.print(manipressgmax);
      }

      else if (manipressgmax < 10)
      {
        lcd.setCursor(6, 3);
        lcd.print(manipressgmax);
      }

      else if (manipressgmax < 100)
      {
        lcd.setCursor(5, 3);
        lcd.print(manipressgmax);
      }

      else
      {
        lcd.setCursor(4, 3);
        lcd.print(manipressgmax);
      }

      //Print max air temp value
      //work out correct text position to right justify and print to LCD

      lcd.setCursor(15, 3);
      lcd.print("   ");

      if (airtempmax < 10)
      {
        lcd.setCursor(17, 3);
        lcd.print(airtempmax);
      }

      else if (airtempmax < 100)
      {
        lcd.setCursor(16, 3);
        lcd.print(airtempmax);
      }

      else
      {
        lcd.setCursor(15, 3);
        lcd.print(airtempmax);
      }
    }

    //-----------------------------------------LCD page 5 - Setup track start line and pit entry-------------------------------

    else if (pagenumber == 4)  //if pageswitch equals 4 display setup screen
    {
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("TRACK SETUP");

      if (lineselect == 0)  //if lineselect is 0 flash 'set pit entry' text
      {
        lcd.setCursor(0, 1);

        if (flashstate == 0)
        {
          lcd.print("SET START LINE");
          flashstate = 1;
        }

        else
        {
          lcd.print("               ");
          flashstate = 0;
        }

        lcd.setCursor(0, 2);
        lcd.print("SET PIT ENTRY");
        lcd.setCursor(0, 3);
        lcd.print("RESET SESSION COUNT");
      }

      else if (lineselect == 1)  //if lineselect is 1 flash 'set pit entry' text
      {
        lcd.setCursor(0, 2);

        if (flashstate == 0)
        {
          lcd.print("SET PIT ENTRY");
          flashstate = 1;
        }

        else
        {
          lcd.print("               ");
          flashstate = 0;
        }

        lcd.setCursor(0, 1);
        lcd.print("SET START LINE");
        lcd.setCursor(0, 3);
        lcd.print("RESET SESSION COUNT");
      }

      else if (lineselect == 2)  //if lineselect is 2 flash 'reset session count' text
      {
        lcd.setCursor(0, 3);

        if (flashstate == 0)
        {
          lcd.print("RESET SESSION COUNT");
          flashstate = 1;
        }

        else
        {
          lcd.print("                   ");
          flashstate = 0;
        }

        lcd.setCursor(0, 1);
        lcd.print("SET START LINE");
        lcd.setCursor(0, 2);
        lcd.print("SET PIT ENTRY");
      }
    }
  }

  //--------------------------Refresh shift light strip-----------------------------
  //Check if anything is over alarm limit and flash shift light strip red

  if (manipressg > 100 || engtemp > 100 || rel_fuelpress < 150 || airtemp > 100 || oilpress < 150 || volt < 12) //If alarm trigger points are exceeded set the alarm to active and reset the timer
  {
    alarm = 1;
    alarmmillis = currentMillis;
  }

  if (currentMillis - alarmmillis > alarmduration) //if alarmduration timer has expired turn off the alarm
  {
    alarm = 0;
  }

  //flash shift light bar on and off at alarmflashinterval if alarm active

  if ( alarm == 1 )
  {
    if (currentMillis - alarmintervalmillis > alarmflashinterval)  //check if alarm flash interval has expired
    {
      alarmintervalmillis = currentMillis; //reset timer for alarm flash
      if (alarmflashstate == 0)  //if LED's are off and turn them on
      {
        colorSolid(strip.Color(brightness, 0, 0));  // light up all LED's red
        alarmflashstate = 1;  //mark alarm flash state as on
      }

      else  //if LED's are on turn the off
      {
        colorSolid(strip.Color(0, 0, 0)); // turn off all LED's
        alarmflashstate = 0;  //mark alarm flash state as off
      }
    }
  }

  //if alarm not active display shift light (this will update at the same rate as the packet0 data from the wolf is refreshed)
  //brightness of the LED's can be altered by updating the LED brightness constant

  else
  {
    int switch1, switch2, switch3, switch4, switch5, switch6, switch7, switch8;
    switch1 = 4000;  //set the RPM the two outside LED's will go green
    switch2 = 4250;  //set the RPM the next two outside LED's will go green
    switch3 = 4500;  //set the RPM the next two outside LED's will go green
    switch4 = 4750;  //set the RPM the next two outside LED's will go yellow
    switch5 = 5000;  //set the RPM the next two outside LED's will go yellow
    switch6 = 5250;  //set the RPM the next two outside LED's will go yellow
    switch7 = 5500;  //set the RPM the next two outside LED's will go red
    switch8 = 5750;  //set the RPM the centre two LED's will go red

    if (rpm > switch1)
    {
      strip.setPixelColor(0, 0, brightness, 0);
      strip.setPixelColor(15, 0, brightness, 0);
    }

    else
    {
      strip.setPixelColor(0, 0, 0, 0);
      strip.setPixelColor(15, 0, 0, 0);
    }

    if (rpm > switch2)
    {
      strip.setPixelColor(1, 0, brightness, 0);
      strip.setPixelColor(14, 0, brightness, 0);
    }

    else
    {
      strip.setPixelColor(1, 0, 0, 0);
      strip.setPixelColor(14, 0, 0, 0);
    }

    if (rpm > switch3)
    {
      strip.setPixelColor(2, 0, brightness, 0);
      strip.setPixelColor(13, 0, brightness, 0);
    }

    else
    {
      strip.setPixelColor(2, 0, 0, 0);
      strip.setPixelColor(13, 0, 0, 0);
    }

    if (rpm > switch4)
    {
      strip.setPixelColor(3, brightness, brightness, 0);
      strip.setPixelColor(12, brightness, brightness, 0);
    }

    else
    {
      strip.setPixelColor(3, 0, 0, 0);
      strip.setPixelColor(12, 0, 0, 0);
    }

    if (rpm > switch5)
    {
      strip.setPixelColor(4, brightness, brightness, 0);
      strip.setPixelColor(11, brightness, brightness, 0);
    }

    else
    {
      strip.setPixelColor(4, 0, 0, 0);
      strip.setPixelColor(11, 0, 0, 0);
    }

    if (rpm > switch6)
    {
      strip.setPixelColor(5, brightness, brightness, 0);
      strip.setPixelColor(10, brightness, brightness, 0);
    }

    else
    {
      strip.setPixelColor(5, 0, 0, 0);
      strip.setPixelColor(10, 0, 0, 0);
    }

    if (rpm > switch7)
    {
      strip.setPixelColor(6, brightness, 0, 0);
      strip.setPixelColor(9, brightness, 0, 0);
    }

    else
    {
      strip.setPixelColor(6, 0, 0, 0);
      strip.setPixelColor(9, 0, 0, 0);
    }

    if (rpm > switch8)
    {
      strip.setPixelColor(7, brightness, 0, 0);
      strip.setPixelColor(8, brightness, 0, 0);
    }

    else
    {
      strip.setPixelColor(7, 0, 0, 0);
      strip.setPixelColor(8, 0, 0, 0);
    }

    strip.show();
  }

  loopcount++;
  wolfcount++;

}
//------------------------------------------------end loop--------------------------------------------------


