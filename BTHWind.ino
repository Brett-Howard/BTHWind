#include <Adafruit_NeoPixel_ZeroDMA.h>  //uses triple the needed RAM but allows interrupts and what not to work as they should
#include "anemometer.h"                 //Peet Bro's Anemometer
#include <SPI.h>                        //SPI Support
#include <Wire.h>                       //I2C Support
#include <SdFat.h>                      //SD Card FAT file system
#include <SDConfigFile.h>               //Read config files
#include <Adafruit_BNO055.h>            //Library for a 9DOF IMU that should have a magnetometer with lower angle noise
#include <Adafruit_BMP280.h>            //support for barometric pressure sensor
#include <Adafruit_GFX.h>               //Font support for 14-seg LEDs
#include <Adafruit_LEDBackpack.h>       //Support for 14-seg LEDs
#include <SparkFun_APDS9960.h>          //Sparkfun APDS9960 Ambient Light/Gesutre Sensor library (which actually works)
                                        //using a local copy to allow for shortening the gesutre delay
#include <NMEAGPS.h>                    //GPS Support
#include <GPSport.h>                    //GPS Support
#include <RH_RF95.h>                    //LoRa Radio support
#include <Timezone.h>                   //allows for conversion to local time
#include <QList.h>                      //Linked List Library (used for averaging in TrueHead menu item)


//I2C Address Information (just to make sure there are no collisions)
//BNO055 - 28h or 29h
//APDS9960 - 39h
//BMP280 - 77h
//LED Backpack - 70h

//#define debug                   //comment this out to not depend on USB uart.
//#define showDistFromHome        //show distance from home once per second (requires debug)
//#define showClockTime           //prints time to console every time it is fetched (requires debug)
//#define noisyDebug              //For those days when you need more information (this also requires debug to be on)
//#define showReceivedPackets     //show recived messages from the mast head unit as they come in
//#define showPacketLoss          //print packet lost messages
#define LoRaRadioPresent          //comment this line out to start using the unit with a wireless wind transducer
#define useBaroTemp               //Use the BMP280 barometric pressure temp sensor
//#define useBnoTemp              //Use the BNO055 temperature sensor

#define batteryLogInterval 600000   //every 10 minutes
#define radioTimeoutReset 5000      //reset radio reception variables if no message is received for this period.

#define informationScrollSpeed 150  //scroll speed of informational text (ie. Waiting for GPS fix or Waiting for Masthead unit)

//#define RGB
#define RGBW              //These defines will change the instantiation of the LED ring and the color table.

#define LED_RING_PIN 11
#define LED_RING_SIZE 24

#ifdef RGB
  #define OFF         0x000000
  #define BLACK       0x000000
  #define RED         0xFF0000
  #define DIM_RED     0x3F0000
  #define GREEN       0x00FF00
  #define DIM_GREEN   0x003F00
  #define BLUE        0x0000FF
  #define DIM_BLUE    0x00003F
  #define YELLOW      0xFFFF00
  #define DIM_YELLOW  0x3F3F00
  #define PURPLE      0xFF00FF
  #define DIM_PURPLE  0x3F003F
  #define WHITE       0xFFFFFF
  #define DIM_WHITE   0x3F3F3F  
#endif

#ifdef RGBW  //actually WGRB
  #define OFF           0x00000000
  #define BLACK         0x00000000
  #define GREEN         0x00FF0000
  #define DIM_GREEN     0x003F0000
  #define RED           0x0000FF00
  #define DIM_RED       0x00003F00
  #define BLUE          0x000000FF
  #define DIM_BLUE      0x0000003F
  #define YELLOW        0x00FFFF00
  #define DIM_YELLOW    0x003F3F00
  #define PURPLE        0x00FF00FF
  #define DIM_PURPLE    0x003F003F
  #define WHITE         ALL_WHITE  //0xFF000000
  #define DIM_WHITE     0x3F000000
  #define ALL_WHITE     0xFFFFFFFF
  #define DIM_ALL_WHITE 0x3F3F3F3F   
#endif

#define ANEMOMETER_SPEED_PIN 5
#define ANEMOMETER_DIR_PIN 6

#define RED_LED_PIN 13
#define GREEN_LED_PIN 8

#define SD_CHIP_SEL 4

#define GESTURE_INT A5

#define RFM95_CS 9
#define RFM95_RST 10
#define RFM95_INT 12
#define RF95_FREQ 915.0
#define RH_RF95_MAX_MESSAGE_LEN 10
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
#ifdef RGB
  Adafruit_NeoPixel_ZeroDMA strip(LED_RING_SIZE, LED_RING_PIN, NEO_GRB + NEO_KHZ800);
#endif
#ifdef RGBW
  Adafruit_NeoPixel_ZeroDMA strip(LED_RING_SIZE, LED_RING_PIN, NEO_RGBW + NEO_KHZ800);
#endif
uint32_t pixelBackground[LED_RING_SIZE];

SdFat sd;
SDConfigFile cfg;   //used for SD config file parsing
SdFile configFile;     //used for SD config file reading/writing
SdFile windStats;   //internal file for statistics
SdFile compCal;     //used for IMU calibration
SdFile gpsLog;      //used for the GPX log output
SdFile battFile;    //used to log battery voltage over time
SdFile logfile;     //used for sail stats logging

ArduinoOutStream cout(Serial);  //allows "cout <<" to go to the Serial port

Anemometer Peet(ANEMOMETER_SPEED_PIN, ANEMOMETER_DIR_PIN, 8, 125);
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
Adafruit_BMP280 baro;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
SparkFun_APDS9960 apds = SparkFun_APDS9960();
#ifdef LoRaRadioPresent
  RH_RF95 rf95(RFM95_CS, RFM95_INT);
#endif

static NMEAGPS  gps;
static gps_fix globalFix;

enum Mode { AppWind, WindStats, TrueWind, TrueHead, CompHead, COG, SOG, Baro, Temp, Heel, MastBatt }; 

int16_t bowOffset, variance;
uint8_t speedMAD;
uint8_t TrueHeadAvgDepth;
uint16_t windUpdateRate;
uint16_t directionFilter;
uint8_t heelAngle;
uint16_t menuDelay;
char tempUnits;
int8_t TimeZone;
uint16_t delayBetweenFixes;
int16_t baroRefAlt;
char trackName[20] = {0};
char filename[23];
bool GPXLogging;
time_t startTime, endTime;
int32_t homeLat, homeLon;
float homeStatRadius, homeGPSRadius;
uint32_t lastMessageTime = 0;
bool linkLost;

TimeChangeRule DSTrule, STrule;

Timezone localTZ(DSTrule,STrule);  //this creates a global placeholder that gets updated later once the SD config file is read.

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.
// NOTE: I didn't follow any of the above with my design and its never been an issue.  I also found that it works without adding a voltage buffer
// but in my final design I did include a level translator to bring the 3.3V I/O up to 5V logic levels.

void setup() {
  #ifdef debug
  	Serial.begin(115200);
	  while(!Serial);  //wait for USB port to be initialized
  #endif

  pinMode(LED_RING_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT );
  pinMode(GREEN_LED_PIN, OUTPUT );
  pinMode(RFM95_CS, OUTPUT);
  pinMode(SD_CHIP_SEL, OUTPUT);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(LED_RING_PIN, LOW);
  digitalWrite(RFM95_CS, HIGH);
  digitalWrite(SD_CHIP_SEL, HIGH);


  //setup radio pins
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);


  strip.begin();
  strip.clear();
  strip.show(); // Initialize all pixels to 'off'

  alpha4.begin(0x70);  // pass in the address for the alpha numeric display
  scrollString("BTHWind\0", 250);
  
///////////////////////////////////////Startup Barometric Sensor/////////////////////////////////////////////////////
  if (!baro.begin()) {
    #ifdef debug
      Serial.println(F("Failed to find barometric pressure sensor"));
    #endif
  }
  else {
    #ifdef debug
      Serial.println(F("Barometric pressure sensor initialized"));
    #endif
  }

///////////////////////////////////////Startup Ambient Light and Gesture Sensor///////////////////////////////////////
  if(apds.init()) {
    #ifdef debug
      Serial.println("Light Sensor Initialized");
    #endif
  }
  else {
    #ifdef debug
      Serial.println("Light sensor is borked.");
    #endif
  }
  if ( apds.enableLightSensor(false) ) {     //false means no interrupt is configured for this mode
    #ifdef debug  
      Serial.println(F("Light sensor is now running"));
    #endif
  } 
  else {
    #ifdef debug
      Serial.println(F("Something went wrong during light sensor init!"));
    #endif
  }
  if ( apds.enableGestureSensor(true) ) {
    #ifdef debug
      Serial.println(F("Gesture sensor is now running"));
    #endif
  } else {
    #ifdef debug
      Serial.println(F("Something went wrong during gesture sensor init!"));
    #endif
  }
  apds.setGestureGain(GGAIN_1X);      //seems odd but the lowest gain setting works best
  //apds.enableProximitySensor(false);
/////////////////////////////////////Setup GPS///////////////////////////////////////////////////////////////////////////

  if (gps.merging != NMEAGPS::EXPLICIT_MERGING)
    Serial.println( F("Warning: EXPLICIT_MERGING should be enabled for best results!") );

  gpsPort.begin( 9600 );

  gps.send_P( &gpsPort, F("PGCMD,33,0") );  //turn off antenna nuisance data
  gps.send_P( &gpsPort, F("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0") ); // 1 RMC & GGA  (need GGA for Altitude, Sat Count and Hdop)
  SdFile::dateTimeCallback(dateTime);  //register date time callback for file system

//////////////////////////////////////////////////////Setup LoRa Radio//////////////////////////////////////////////////////
  #ifdef LoRaRadioPresent
    
    // Ensure serial flash is not interfering with radio communication on SPI bus
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
 
    while (!rf95.init()) {
      Serial.println("LoRa radio init failed");
      while (1);
    }
    #ifdef debug
     Serial.println("LoRa radio init OK!");
    #endif

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("setFrequency failed");
      while (1);
    }
    #ifdef debug
      Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
    #endif
    
    //power is adjustible from 5 to 23dBm
    rf95.setTxPower(13);  //leaving at the default power because this is plugged in (mashead is at 5dBm)
    rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf128);
  #endif

  //wait for fix tries to reply to the mast head if it can thus waiting for fix must be after radio init.
  waitForFix();  //Sit here and wait until the GPS locks onto the network of sats.

///////////////////////////////////////////Initialize SD Card/////////////////////////////////////////////////////////
  if(initSD()) {
    //Serial.print(F("Card size: ")); Serial.print(sd.card()->cardSize() * 0.000512 + 0.5); Serial.println(" MiB");
    //removing card free because it takes a while on a 16GB card.
    //Serial.print(F("Card free: ")); Serial.print(sd.vol()->freeClusterCount() * .000512 * sd.vol()->blocksPerCluster()); Serial.println(" MiB");
  }
  
//////////////////////////////////////Startup Magnetometer and Accelerometer//////////////////////////////////////////
  if(!bno.begin())  //9DOF IMU
  {
    #ifdef debug
      Serial.println("BNO055 initialization failed");
    #endif
    failBlink();
  }
  #ifdef debug
    cout << "BNO055 IMU Initialized" << endl;
  #endif

  //These lines align the axes to the direction that I currently have my bread board sitting on the couch.  May need to chage when in the case.
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P2);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P2);
  
///////////////////////////////////////////////////////////////////////////////////////////////////////
//sd.remove("/!CONFIG/IMUCAL.CSV");           //uncomment this line to erase the IMU calibration data//
///////////////////////////////////////////////////////////////////////////////////////////////////////

  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  
  char tmp[13];

  //if there isn't a IMU calibration file print status while you calibrate
  if(!sd.exists("/!CONFIG/IMUCAL.CSV")) {
    displayString("IMU?");
    delay(3000);
    displayString("SGAM");
    delay(4000);
    while(system < 3 || gyro < 3 || accel < 3 || mag < 3) {
      bno.getCalibration(&system, &gyro, &accel, &mag);
      Serial.print("Sys:");
      Serial.print(system, DEC);
      Serial.print(" G:");
      Serial.print(gyro, DEC);
      Serial.print(" A:");
      Serial.print(accel, DEC);
      Serial.print(" M:");
      Serial.println(mag, DEC);
      sprintf(tmp, "%d%d%d%d", system, gyro, accel, mag);
      displayString(tmp);
    }
  }
  
  //now that the system is in calibration store that calibration out to the SD card
  adafruit_bno055_offsets_t offsets;

  bno.getSensorOffsets(offsets);
  
  #ifdef debug
    displaySensorOffsets(offsets); 
  #endif
  

  if(!sd.exists("/!CONFIG/IMUCAL.CSV")) {
    #ifdef debug
      cout << "Compass calibration file did not exist." << endl;
    #endif
    //write out a new calibration file
    if(compCal.open("/!CONFIG/IMUCAL.CSV", O_CREAT | O_WRITE | O_EXCL)) {
      cout << "New compass calibration file created." << endl;
      compCal.print(offsets.accel_offset_x); compCal.print(',');
      compCal.print(offsets.accel_offset_y); compCal.print(',');
      compCal.print(offsets.accel_offset_z); compCal.print(',');
      compCal.print(offsets.gyro_offset_x); compCal.print(',');
      compCal.print(offsets.gyro_offset_y); compCal.print(',');
      compCal.print(offsets.gyro_offset_z); compCal.print(',');
      compCal.print(offsets.mag_offset_x); compCal.print(',');
      compCal.print(offsets.mag_offset_y); compCal.print(',');
      compCal.print(offsets.mag_offset_z); compCal.print(',');
      compCal.print(offsets.accel_radius); compCal.print(',');
      compCal.print(offsets.mag_radius);
    }
    compCal.close(); 
  }

  //if there is calibration data read it from the file and apply it 
  else if(compCal.open("/!CONFIG/IMUCAL.CSV", O_READ))
  {
    #ifdef debug
      cout << "Reading input from IMU calibration file" << endl;
    #endif
    
    int i = 0;
    int16_t num;
    
    offsets.accel_offset_x = csvReadInt16(&compCal, &num, ',');
    offsets.accel_offset_y = csvReadInt16(&compCal, &num, ',');
    offsets.accel_offset_z = csvReadInt16(&compCal, &num, ',');
    offsets.gyro_offset_x = csvReadInt16(&compCal, &num, ',');
    offsets.gyro_offset_y = csvReadInt16(&compCal, &num, ',');
    offsets.gyro_offset_z = csvReadInt16(&compCal, &num, ',');
    offsets.mag_offset_x = csvReadInt16(&compCal, &num, ',');
    offsets.mag_offset_y = csvReadInt16(&compCal, &num, ',');
    offsets.mag_offset_z = csvReadInt16(&compCal, &num, ',');
    offsets.accel_radius = csvReadInt16(&compCal, &num, ',');
    offsets.mag_radius = csvReadInt16(&compCal, &num, ',');
    compCal.close();
    
    #ifdef debug
      displaySensorOffsets(offsets);
    #endif

    bno.setSensorOffsets(offsets);  //squirt the calibration into the IMU object
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Read Configuration from SD card and populate globals
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if(!readConfig()) {  //read configuration from SD card
    #ifdef debug
	    Serial.println(F("Failed to create and/or read configuration file from SD"));
	  #endif
	  failBlink();  //dies here (never returns)
  }
  #ifdef debug
	  Serial.println(F("SD card configuration read successfully"));
	#endif

  //now that DST and ST are populated update timezone with the time change rules from the SD card
  localTZ.setRules(DSTrule, STrule);
  
  //write update rate into the GPS  (has to be moved here after we've fetched the value from the config)
  sprintf(tmp, "PMTK220,%d\0", delayBetweenFixes);
  gps.send( &gpsPort, tmp ); //set fix update rate
  
  sd.remove("WINDSTAT.LOG");  //delete prior log file
  
//////////////Setup and configure the anemometer object and link up the necessary interrupts
  Peet.setBowOffset(bowOffset);
  Peet.setSpeedMAD(speedMAD);
  Peet.setDirectionFilter(directionFilter);
  
  #ifndef LoRaRadioPresent
    attachInterrupt(digitalPinToInterrupt(ANEMOMETER_SPEED_PIN), isrSpeed, FALLING);
    attachInterrupt(digitalPinToInterrupt(ANEMOMETER_DIR_PIN), isrDirection, FALLING);
  #endif
  //if there is a LoRa radio the radio data is polled on and given to the anemometer object when received in the main loop.  
  tcConfigure(1000);  //1 second timer 
  
  //setup inerrupt to handle the gesture sensor
  attachInterrupt(digitalPinToInterrupt(GESTURE_INT),isrGesture, FALLING);

  //Set LED ring brightness and animate the background into place
  uint16_t w;
  apds.readAmbientLight(w);
  strip.setBrightness(map(w,0,37889,5,255));  //set the brightness for the power up sequence so it doesn't change after powerup.
  animateRedGreenWipe(60);  //pretty startup animation

  //prime the pump for the GPS just to make sure the globals are usable on the first time through the loop.
  while(!gps.available())
    if(Serial1.available())
      gps.handle(Serial1.read());   //inject stuff into the GPS object until a valid fix is puked out
    
  globalFix = gps.read();       //snag the fix to prime the pump.
  startTime = getLocalTime();
}  //setup

void displayIntFloat(int, char);  //compiler wants this function and only this one listed here for some reason.
volatile bool gestureSensed = false;

void loop() {
  static Mode curMode = TrueWind;    //default state upon power up is True Wind
  static Mode prevMode;
  uint8_t gesture; 
  static uint16_t w,wndSpd,windMax = 0,boatMax = 0;
  static bool firstEntry = 1;
  static int curHeelAngle;
  static uint32_t menuTimer;
  static bool locked = false;
  static bool newSDData = false;
  static uint16_t battVoltage;
  static sensors_event_t compEvent;
  static uint8_t compTimer;
  static uint32_t battTimer = 0;
  static bool GPXLogStarted = false;
  static uint32_t speedAccum, boatSpeedAccum;
  static uint16_t AvWindDir;
  static bool tripStarted = false, logEntryMade = false, truncateLastEntry = false;
  static NeoGPS::Location_t home(homeLat, homeLon);  //create Location_t that represents slip locaiton 
  time_t local;

  //adjust wind ring brightness based on ambient light
  apds.readAmbientLight(w);
  strip.setBrightness(map(w,0,37889,5,255));
  strip.show();

  //fetch new event from IMU.  This is done here to minimize reads to the IMU from multiple helper functions.
  bno.getEvent(&compEvent);

  //Log battery voltage to a CSV file for graphing to understand how well the masthead unit's solar setup is working
  if(millis() > battTimer + batteryLogInterval && battVoltage != 999)
  {
    local = getLocalTime();

    char date1[22];
    battFile.open("/!CONFIG/BATTERY.CSV", O_WRITE | O_CREAT | O_APPEND);
    sprintf(date1, "%4d-%02d-%02d %02d:%02d", year(local), month(local), day(local), hour(local), minute(local));
    battFile.print(date1); battFile.print(F(", ")); battFile.println(battVoltage);
    battFile.close();
    #ifdef debug
      cout << "Printing Battery File " << date1 << ',' << battVoltage << endl;
    #endif
    battTimer = millis();
  }

  //Determine if we're over heeling
  if(millis() > compTimer + 50) {   //only read compass and heel information every 50mS (20Hz) because it doesn't update any faster than that
    curHeelAngle = getHeelAngle(compEvent);
    compTimer = millis();
  }
  if(curHeelAngle > heelAngle && heelAngle != 0 && curMode != Heel) {
    prevMode = curMode;
    firstEntry = true;   //this will ensure the display of "Reduce Heel" on entry of Heel state
    curMode = Heel;
  }
  else if(curMode == Heel && curHeelAngle < heelAngle) {
    curMode = prevMode;   //return to previous mode
    //firstEntry = true;    //tell the user what mode they used to be in (not sure if I want to keep this)
  }
  
  //use DIR_NEAR and DIR_FAR gestures to lock the menu
  if(gestureSensed || apds.isGestureAvailable()) {   //check the interrupt and poll just incase we missed it during an SD file I/O operation
    detachInterrupt(digitalPinToInterrupt(GESTURE_INT));  //detaching Interrupt here because I've seen an example that does something similar... 
                                                          //hopefully this possibly fixes the crashing when something is held in front of the gesture sensor.
    gestureSensed = false;
    gesture = apds.readGesture();
    if (gesture == DIR_NEAR || locked)
    {
      if(!locked) {
        scrollString("LOCKED\0", menuDelay);
        locked = 1;
      }
      if(locked && gesture == DIR_FAR) {
        scrollString("UNLOCKED\0", menuDelay);
        locked = 0;
      }
      else if(locked)
      {
        gesture = 0;  //make all gestures go away so they don't change menu status
      }
    }
    
  }

apds.clearProximityInt();
//reattach gesture interrupt that is detached inside the ISR
attachInterrupt(digitalPinToInterrupt(GESTURE_INT),isrGesture, FALLING);


//This switch is the state machine for all the menu items. 
switch(curMode)
{
  case AppWind:      //displays apparent wind speed and direction relative to the bow
      if(firstEntry) {
        scrollString("APPARENT WIND\0", menuDelay);
        firstEntry = false;
        restoreBackground();
      }
      ///////do AppWind
      if(linkLost) {
          showLinkLostMessage();
      }
      else if(wndSpd > 0) {
        if(millis() > menuTimer + windUpdateRate) {
          displayIntFloat(wndSpd,'\0');
          menuTimer = millis();
        }
      }
      else { displayString("CALM"); }
      #ifdef noisyDebug
        cout << "AWS: "  << wndSpd << " AWA: " << Peet.getDirection() << endl;
      #endif
      //////Transition state
      if(gesture == DIR_LEFT) { curMode = Baro; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_UP) { curMode = TrueHead; firstEntry = true; }
      else if(gesture == DIR_DOWN) { curMode = TrueWind; firstEntry = true; }
      break;

  case TrueWind:      //displays true wind speed and direction relative to the bow
      if(firstEntry) {
        scrollString("TRUE WIND\0", menuDelay);
        firstEntry = false;
      }
      //////////////do TrueWind
      uint16_t _SOG;
      uint16_t _AWS;
      uint16_t _AWA;
      uint16_t _TWS;

      if (globalFix.valid.speed) {
        _SOG = globalFix.speed()*100;
      }

      _AWA = Peet.getDirection();
      
      if(millis() > menuTimer + windUpdateRate) {
        #ifdef noisyDebug
          cout << "AWA: "  << _AWA << " AWS: " << wndSpd << " SOG: " << _SOG << endl;
        #endif
        
        if(linkLost) {
          showLinkLostMessage();
        }
        else if(wndSpd > 0) {
          displayIntFloat(getTWS(_AWA, wndSpd, _SOG), '\0');
          displayWindPixel(getTWA(_AWA, wndSpd, _SOG), WHITE);
        }
        else { displayString("CALM"); }

        menuTimer = millis();
      }   
      
      //////////////Transition State
      if(gesture == DIR_LEFT) { curMode = Baro; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_UP) { curMode = AppWind; firstEntry = true; }
      else if(gesture == DIR_DOWN) { curMode = WindStats; firstEntry = true; }
      break;

  case TrueHead:      //displays true wind compass heading 
      if(firstEntry) {
        scrollString("TRUE HEADING\0", menuDelay);
        firstEntry = false;
      }
      //////////////do TrueHead
      static QList<uint16_t> TWDAry;

      if (globalFix.valid.speed) {
        _SOG = globalFix.speed()*100;
      }

      _AWA = Peet.getDirection();
      
      /*_SOG = 001;
      _AWA = 40;
      wndSpd = 375;*/
      if(millis() > menuTimer + windUpdateRate) {
          cout << "AWA: "  << _AWA << " AWS: " << wndSpd << " SOG: " << _SOG << endl;
        
        if(linkLost) {
          showLinkLostMessage();
        }
        else if(wndSpd > 0) {
          #ifdef noisyDebug
            cout << "cog = " << getCOG(compEvent) << endl;
            cout << "twd = " << getTWD(_AWA,wndSpd,_SOG,getCOG(compEvent)) << endl;
          #endif
          //add a new TWD to the list
          TWDAry.push_back(getTWD(_AWA,wndSpd,_SOG,getCOG(compEvent)));        //TODO: Calculate this always so that you alwasy have a full average array (not just once you've entered the menu item.)
          //remove an element once we have enough values.
          if(TWDAry.size() > TrueHeadAvgDepth-1) {
            TWDAry.pop_front();
          }
          //Accumulate values
          uint16_t AvgTWD = 0;
          for(int i(0); i < TWDAry.size(); i++) {
            AvgTWD += TWDAry[i];
          }
          //calculate avarage
          AvgTWD /= TWDAry.size();
          
          //display average
          displayAngle(AvgTWD, 'T');
          displayWindPixel(getTWD(_AWA,wndSpd,_SOG,getCOG(compEvent)), WHITE);
        }
        else { displayString("CALM"); }

        menuTimer = millis();
      }   
      
      //////////////Transition State
      if(gesture == DIR_LEFT) { curMode = Baro; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_UP) { curMode = WindStats; firstEntry = true; }
      else if(gesture == DIR_DOWN) { curMode = AppWind; firstEntry = true; }
      break;  
  
  case WindStats:     //displays statistical information about the current trip
      char temp[5];
      if(firstEntry || newSDData) {
        #ifdef debug
          cout << "got new SD data" << endl;
        #endif
        scrollString("SAIL STATS\0", menuDelay);
        firstEntry = false;
        newSDData = false;
        
        displayString("WAIT");
        
        ProcessStatistics(speedAccum, boatSpeedAccum, AvWindDir);  //read from global WINDSTAT.LOG and results returned by reference
        
        menuTimer = millis();  //start the menu back at the top
      }
      /////////////do WindStats

      //show the values on the display
      if(millis() > menuTimer && millis() < menuTimer+2000) {   //this timing method is really annoying but its good to keep loop moving faster
        displayString("DATE");
      }
      else if(millis() >= menuTimer+2000 && millis() < menuTimer+6000) {
        sprintf(temp, "%02u%02u", month(startTime), day(startTime));
        displayString(temp);
      }
      else if(millis() >= menuTimer+6000 && millis() < menuTimer+8000) {
        displayString("STRT");
      }
      else if(millis() >= menuTimer+8000 && millis() < menuTimer+12000) {
        sprintf(temp, "%02u%02u", hour(startTime), minute(startTime));
        displayString(temp);
      }
      else if(millis() >= menuTimer+12000 && millis() < menuTimer+14000) {
        displayString("END ");
      }
      else if(millis() >= menuTimer+14000 && millis() < menuTimer+18000) {
        sprintf(temp, "%02u%02u", hour(endTime), minute(endTime));
        displayString(temp);
      }
      else if(millis() >= menuTimer+18000 && millis() < menuTimer+20000) {
        displayString("ASOG");
      }
      else if(millis() >= menuTimer+20000 && millis() < menuTimer+24000) {
        displayIntFloat(boatSpeedAccum, '\0');
      }
      else if(millis() >= menuTimer+24000 && millis() < menuTimer+26000) {
        displayString("MSOG");
      }
      else if(millis() >= menuTimer+26000 && millis() < menuTimer+30000) {
        displayIntFloat(boatMax, '\0');
      }
      else if(millis() >= menuTimer+30000 && millis() < menuTimer+32000) {  
        displayString("AVGW");
      }
      else if(millis() >= menuTimer+32000 && millis() < menuTimer+36000) {
        displayIntFloat(speedAccum, '\0');
      }
      else if(millis() >= menuTimer+36000 && millis() < menuTimer+38000) {
        displayString("MAXW");
      }
      else if(millis() >= menuTimer+38000 && millis() < menuTimer+42000) {
        displayIntFloat(windMax, '\0');
      }
      else if(millis() >= menuTimer+42000 && millis() < menuTimer+44000) {
        displayString("AvWD");
      }
      else if(millis() >= menuTimer+44000 && millis() < menuTimer+48000) {
        displayAngle(AvWindDir, '\0');
      }
      else if(millis() >= menuTimer+48000 && millis() < menuTimer+50000) {
        displayString("BARO");
      }
      else if(millis() >= menuTimer+50000 && millis() < menuTimer+54000) {
        displayIntFloat(getBaro(), '\0');
      }
      else if(millis() > menuTimer+54000)
        menuTimer = millis();    //restart the menu again

      ////////////Transition State
      if(gesture == DIR_LEFT) { curMode = Baro; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_UP) { curMode = TrueWind; firstEntry = true; }
      else if(gesture == DIR_DOWN) { curMode = TrueHead; firstEntry = true; }
      break;
  
  case CompHead:     //displays magnetometer based heading (compass)
      if(firstEntry) {
        scrollString("COMPASS HEADING\0", menuDelay);
        firstEntry = false;
      }
      /////////////do CompHead
      
      static int16_t heading;
      
      heading = getMagneticHeading(compEvent);

      if(heading < 0)
        displayString("CAL ");
      else {
        displayAngle(heading, 'M');
        displayWindPixel(heading, WHITE);
      }

      ///////////Transition State
      if(gesture == DIR_LEFT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = MastBatt; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = COG; firstEntry = true; }
      break; 
  
  
  case COG:    //displays GPS course over ground
      if(firstEntry) {
        scrollString("GPS COG\0", menuDelay);
        firstEntry = false;
      }
      //////////////////do COG
      if (globalFix.valid.speed) {
        displayAngle(uint16_t(globalFix.heading()), 'T');
      }
      //////////////////Transition State
      if(gesture == DIR_LEFT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = MastBatt; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = CompHead; firstEntry = true; }
      break; 
  
  
  case SOG:  //displays GPS speed over ground
      if(firstEntry) {
        scrollString("GPS SOG\0", menuDelay);
        firstEntry = false;
      }
      //////////////////do SOG
      if (globalFix.valid.speed) {
        displayIntFloat(globalFix.speed()*100, '\0');
      }

      //////////////////Transition State
      if(gesture == DIR_LEFT) { curMode = TrueWind; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = CompHead; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = SOG; firstEntry = true; }
      break;
  
  
  case Baro:    //displays current barometer
      if(firstEntry) {
        scrollString("BAROMETER\0", menuDelay);
        firstEntry = false;
      }
      ////////////////Do Baro
      if(millis() > menuTimer+1000) { 
        displayIntFloat(getBaro(), '\0');
        menuTimer = millis(); 
      } 
      ////////////////Transition State
      if(gesture == DIR_LEFT) { curMode = Temp; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = TrueWind; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = Baro; firstEntry = true; }
      break;
  
  
  case Temp:  //displays current temperature
      if(firstEntry) {
        scrollString("TEMPERATURE\0", menuDelay);
        firstEntry = false;
      }
      ///////////////Do Temp
      //update temp only once per second but in a non blocking way that doesn't slow down gesture response.
      if(millis() > menuTimer+1000) { 
        displayTemp(tempUnits); 
        menuTimer = millis(); 
      } 
      ///////////////Transition State
      if(gesture == DIR_LEFT) { curMode = MastBatt; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = Baro; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = Temp; firstEntry = true; }
      break;
  
  case MastBatt:  //displays mast head battery voltage
      if(firstEntry) {
        scrollString("MAST BATTERY\0", menuDelay);
        firstEntry = false;
      }
      ///////////////Do MastBatt
      if(linkLost) {
        showLinkLostMessage();
      }
      else {
        displayIntFloat(battVoltage, 'V');
      }
      ///////////////Transition State
      if(gesture == DIR_LEFT) { curMode = CompHead; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = Temp; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = MastBatt; firstEntry = true; }
      break;
  
  case Heel:  //Displays current heel angle
      if(firstEntry) {
        scrollString("Reduce Heel\0", menuDelay/2);
        firstEntry = false;
        menuTimer = millis();
      }
      if(millis() > menuTimer && millis() < menuTimer+5000) {
        displayAngle(curHeelAngle, '\0');
      }
      if(millis() > menuTimer+5000) {
        firstEntry = true;  //set so that upon returning you display menu heading
      }
      break;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////End of switch that handles menu items //////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  //Fetch all data from the GPS UART and feed it to the NeoGPS object
  //I tried to put this into a SerialEvent function but that seems to not work for me so I'll just leave this here.
  while (Serial1.available()) { gps.handle(Serial1.read()); }
      
  //update the global fix to be used in menu items and for GPX logging
  if(gps.available()) {
    globalFix = gps.read();  
    if( (homeGPSRadius <= 0) || globalFix.location.DistanceKm( home ) > homeGPSRadius ) {  //check that we're far enough from home to log GPS tracks
      if(!GPXLogStarted && GPXLogging) {
        startGPSFile();
        GPXLogStarted = true;
      }
      else if(GPXLogging) {
          WriteGPXLog();
      }
    }
  }

  //Collect statistics to be used in the SAIL STATS menu
  if(Peet.available()) {        //only true when new information has been received from the anemometer object
    
    wndSpd = Peet.getSpeed();   //wind speed should only be fetched once per loop when necessary because its an expensive operation (right here)
    
    //TODO: Remove this displayWindPixel() and put a copy in each mode so that its easier to decide what is displayed on the ring in each mode.  
    if(wndSpd > 0 && curMode != TrueWind && curMode != TrueHead && curMode != CompHead)  //if we have wind and aren't displaying true wind
        displayWindPixel(Peet.getDirection(), WHITE);
    else if(wndSpd == 0)      //if wind is calm
      restoreBackground();    //this should turn the pixel off in Apparent and True wind modes.

    static uint16_t i_log = 0;

    typedef struct sailStats {
      uint16_t speed;
      int16_t sinTWD;
      int16_t cosTWD;
      int16_t boatSpeed;
    };
    static sailStats statAry[60];   //size of this buffer is "about" how often the log file is updated (in seconds)

    uint32_t accumSpeed = 0;
    int32_t accumSinTWD, accumCosTWD;
    uint32_t accumBoatSpeed = 0;
    uint16_t _SOG_, _COG_;
    
    static uint32_t logTimer = 0;
    uint16_t elements = sizeof(statAry)/sizeof(statAry[0]);  //only done so you don't have to modify array size in two places

    if(millis() > logTimer+1000) {   //capture a new data point "about" once per second    
      static uint8_t bytesWritten = 0;
      
      #ifdef debug
        #ifdef showDistFromHome
          cout << "Distance from home: " << globalFix.location.DistanceKm( home ) << "Km\n";
        #endif
      #endif

      if( (homeStatRadius <= 0) || globalFix.location.DistanceKm( home ) > homeStatRadius ) {  
        if(!tripStarted)
        {
          #ifdef debug
            cout << "Trip started stat collection beginning\n";
          #endif
          startTime = getLocalTime();
          endTime = startTime;      //just make the start and stop times the same so it doesn't show 0000 if you go in to stats immediately
          tripStarted = true;
        }
        else if(tripStarted && logEntryMade && bytesWritten > 0) //if we're outside of the home stats radius and have already made a log entry clean it up.
        {
          cout << "Trip started again settting variable to truncate last entry on next logbook write\n";
          truncateLastEntry = true;
          logEntryMade = false;
        }
        if(globalFix.valid.speed) {
          _SOG_ = globalFix.speed() * 100;   //get GPS speed
          statAry[i_log].boatSpeed = _SOG_;
          if(_SOG_ > boatMax) { boatMax = _SOG_; }
        }

        _COG_ = getCOG(compEvent);
        
        //Add values into the temp stat array
        if(i_log < elements)
        {
          statAry[i_log].speed = getTWS(Peet.getDirection(), wndSpd, _SOG_);
          statAry[i_log].sinTWD = round(sin(degToRad(getTWD(Peet.getDirection(), wndSpd, _SOG_, _COG_)))*10000);  //10000 is to not need floats
          statAry[i_log].cosTWD = round(cos(degToRad(getTWD(Peet.getDirection(), wndSpd, _SOG_, _COG_)))*10000);
          
          i_log++;
        }

        //Once the temp array is full write data out to the SD card
        if(i_log == elements)
        {
          endTime = getLocalTime();
          
          //calculate average wind speed and direction components (direction components are needed to work out average wind direction for the trip)
          accumSpeed = accumSinTWD = accumCosTWD = accumBoatSpeed = 0;
          for(int i = 0; i < elements; i++) {
            accumSpeed += statAry[i].speed;
            accumSinTWD += statAry[i].sinTWD;
            accumCosTWD += statAry[i].cosTWD;
            accumBoatSpeed += statAry[i].boatSpeed;
          }
          if(elements)  //double check to protect from dividing by zero.
          {
            accumSpeed /= elements;
            accumSinTWD /= elements;
            accumCosTWD /= elements;
            accumBoatSpeed /= elements;
          }
       
          if(windStats.open("WINDSTAT.LOG", O_WRITE | O_CREAT | O_APPEND)  || windStats.isOpen()) { //if new create file or if already open continue.
              windStats.print(accumSpeed); windStats.print(',');
              windStats.print(accumSinTWD); windStats.print(',');
              windStats.print(accumCosTWD); windStats.print(',');
              windStats.println(accumBoatSpeed);
              blip(GREEN_LED_PIN,3,20);                    //green light blinks to inform stat file update
              newSDData = true;                            //flag to inform the sailStats state to re-parse the CSV file and update the displayed information.
              #ifdef debug
                cout << F("Avg Written. Time: ") << millis() << F(" Values: ") << accumSpeed << ',' << accumSinTWD << ',' << accumCosTWD << endl;
              #endif
          } //end of file I/O for sail stats
          i_log = 0;
        } //end of stats accumulation after temp array has become full
      } //end of home radius checking
      
      //If we're near home and have been on a trip that has ended more than a couple minutes ago write that down in the logbook.
      local = getLocalTime();
      
      if(tripStarted && !logEntryMade && local > endTime+120 && globalFix.location.DistanceKm( home ) <= homeStatRadius)
      {
        #ifdef debug
          cout << "Trip ended writing log entry\n";
        #endif 

        sd.mkdir("!LOG");  //make sure the !LOG directory exists on the SD card. 
        
        ProcessStatistics(speedAccum, boatSpeedAccum, AvWindDir);  //read from global WINDSTAT.LOG and results returned by reference
        
        char startStr[22], endStr[22];
        sprintf(startStr, "%4d-%02d-%02d %02d:%02d", year(startTime), month(startTime), day(startTime), hour(startTime), minute(startTime));
        sprintf(endStr, "%4d-%02d-%02d %02d:%02d", year(endTime), month(endTime), day(endTime), hour(endTime), minute(endTime));
 
        //create the file if it doesn't already exist.  Put field labels in first.
        if(!sd.exists("/!LOG/LOG.CSV")) {
          if(logfile.open("/!LOG/LOG.CSV", O_WRITE | O_CREAT)) {
            #ifdef debug
              cout << "printing headers\n";
            #endif
            logfile.print(F("Start")); logfile.print(',');
            logfile.print(F("End")); logfile.print(',');
            logfile.print(F("Avg Speed (kts)")); logfile.print(',');
            logfile.print(F("Avg Wind (kts)")); logfile.print(',');
            logfile.print(F("Max Wind (kts)")); logfile.print(',');
            logfile.print(F("Avg Wind Dir (degT)")); logfile.print(',');
            logfile.println(F("Baro (inHg)"));
          }
        }

        //O_AT_END used to make new entries appear at the end but moved back one record if truncation is needed.
        //O_APPEND actually ignores the seekSet command and always does all writes at the end of the file.
        if(logfile.open("/!LOG/LOG.CSV", O_WRITE | O_AT_END)  || logfile.isOpen()) {
          
          if(truncateLastEntry) {
            logfile.seekSet(logfile.fileSize() - bytesWritten);
            bytesWritten = 0;
            truncateLastEntry = false;
          }
          
          bytesWritten += logfile.print(startStr); 
          bytesWritten += logfile.print(',');                     
          bytesWritten += logfile.print(endStr); 
          bytesWritten += logfile.print(',');                       
          bytesWritten += logfile.print(float(boatSpeedAccum/100.0)); 
          bytesWritten += logfile.print(',');                           
          bytesWritten += logfile.print(float(speedAccum/100.0)); 
          bytesWritten += logfile.print(',');       
          bytesWritten += logfile.print(float(windMax/100.0));
          bytesWritten += logfile.print(',');
          bytesWritten += logfile.print(AvWindDir); 
          bytesWritten += logfile.print(',');                     
          bytesWritten += logfile.println(float(getBaro()/100.0));                          
          logfile.close();  //close the file to flush the cache to disk
          logEntryMade = true;
          
          #ifdef debug
            cout << "Log entry written " << unsigned(bytesWritten) << " bytes appended\n";
            cout << startStr << "," << endStr << "," << float(boatSpeedAccum/100.0) << ",";
            cout << float(speedAccum/100.0) << "," << float(windMax/100.0) << "," << AvWindDir << "," << float(getBaro()/100.0) << endl;
          #endif
        }

        //Figure out what to do if the trip ends and is then restarted....
      }
        
    logTimer = millis();  //take a datapoint so we know when to come back in here again
    }  //end of once per second
  } //if Peet.available();
    
  //update maximum wind speed if needed
  if(wndSpd > windMax) { windMax = wndSpd; }  //Only takes a single datapoint for a max but remember it comes from a moving average filter.

  //handle radio traffic and receive messages
  //Message format 2 bytes per field:
  //Field 1: (wind speed in knots * 100)
  //Field 2: Apparent Wind Direction (0-359) 0=bow (if offset is set properly)
  //Field 3: Battery voltage*100
  //Field 4: Free running 8-bit counter (for message loss detection) Each message should be sequential. 
  #ifdef LoRaRadioPresent
    if (rf95.available())
    {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      uint8_t data[RH_RF95_MAX_MESSAGE_LEN];

      if (rf95.recv(buf, &len))
      {
        linkLost = false;   //link is up because we're receiving messages.
        uint16_t spd;
        int16_t dir;
        uint8_t messageCount;

        if(stricmp((char*)buf,"McFly") == 0) {
          strcpy((char*)data, "HiBiff");   //first reply of HiBiff mostly just as a joke.  Mast head doesn't care what it gets back really.
          #ifdef debug
            cout << "Time = " << millis() << " Got McFly?...  Sending \"HiBiff\"" << endl;
          #endif
        }
        else {
          #ifdef debug
            //cout << "RSSI: " << rf95.lastRssi() << " SNR: " << rf95.lastSNR() << endl;
          #endif
          lastMessageTime = millis();
          strcpy((char*)data, "A");    //get ready to send back an "A" for ACK.
          
          //Grab the values from the recieved message
          memcpy(&spd, &buf, 2);
          memcpy(&dir, &buf[2], 2);
          memcpy(&battVoltage, &buf[4], 2);
          memcpy(&messageCount, &buf[6], 1);
          
          #ifdef debug
            #ifdef showPacketLoss
              static uint8_t lastMessage;
              static uint16_t packetsLost = 0;
              static uint32_t packetsReceived = 0;
              if(messageCount != uint8_t(lastMessage + 1)) {   //if the message count isn't sequential
                ++packetsLost;
                cout << "Packet lost!!!!!!" << endl << endl;
                cout << "Packet Loss: " << double(packetsLost) / double(packetsReceived) * 100.0 << "%" << endl;
                lastMessage = messageCount;
              }
              ++packetsReceived;
              lastMessage = messageCount;
            #endif
            #ifdef showReceivedPackets
              cout << "Time = " << millis() << " " << spd << " " << dir << " " << battVoltage << " "; Serial.println(messageCount, DEC);
            #endif          
          #endif
          
          //Pass the new radio data into the Anemometer object
          Peet.processWirelessData(spd, dir);
        }
        rf95.send(data, sizeof(data));  //transmit ACK response
        //rf95.waitPacketSent();
      }
      else {
        #ifdef debug
          cout << "Receive failed" << endl;
        #endif
      }
    }
  #endif
  
    //cout << F("Free Mem: ") << freeRam() << endl;
}  //loop

//////////////////////////////////////////////////////////Helper Functions//////////////////////////////////////////////////
float degToRad(float deg) { return (deg * PI / 180); }  //convert degrees to radians

float radToDeg(float rad) { return (rad * 180 / PI); }  //convert radians to degrees

float ctof(float c) { return c*1.8+32; }   //convert celcius to farhenheit

float ftoc(float f) { return f-32*0.555556; }    //convert farhenheit to celcius

//get Course Over Ground
//Returns magnetic heading if < 2knots.  Otherwise returns GPS heading.  Negative values indicate error.
//returned values are in degrees refereced to *TRUE* north (magnetic values have variance added to them.)
int16_t getCOG(sensors_event_t event) 
{
  uint16_t _SOG_;

  _SOG_ = globalFix.speed() * 100;  //get GPS speed

  //Get boat heading for TWS calculations
  //use gps heading if traveling over 2 knot otherwise use the compass heading
  if(_SOG_ > 200 && globalFix.valid.heading)
    return globalFix.heading();  
  else {
    static uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;

    //only use compass heading if IMU has a quality fix
    bno.getCalibration(&system, &gyro, &accel, &mag);
    if(mag > 2) {   //if the compass is within calibaration
      //variance added to compass heading becasue its magnetic referenced and we want wind true referenced.
      return int(round(event.orientation.x) + variance + 360) % 360;  
    }
    else {
      //not adding variance because GPS heading is true referenced
      if(globalFix.valid.heading)
      {
        return globalFix.heading();  //GPS heading may be inaccurate at low speeds but its the best we've got if we get here
      }
    }
  }
}

//get Magnetic Heading
//Asks the BNO object for a new event and returns the value.  Negative values mean unit is not in cal.
int16_t getMagneticHeading(sensors_event_t event)
{
  static uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  bno.getCalibration(&system, &gyro, &accel, &mag);

  if(mag < 2)
    return -1;
  else
  {
    return int16_t((round(event.orientation.x)) + 360) % int16_t(360);
  }
}

//get Heel Angle.
//returns the absolute value of the lateral heel angle.  The accelerometer is very easy to calibrate so its not checked.
uint8_t getHeelAngle(sensors_event_t event)
{
  return abs(event.orientation.y);
}

//get True Wind Speed
//returns true wind speed given Aparent Wind Angle, Apparent Wind Speed, and Speed Over Ground
uint16_t getTWS(uint16_t AWA, uint16_t AWS, int16_t SOG)    
{
  float _AWA = degToRad(AWA);
  float tanAlpha = sin(_AWA)/(float(AWS)/float(SOG)-cos(_AWA));
  float Alpha = atan(tanAlpha);
  if(SOG == 0) {
    return AWS;
  }
  else if(AWA == 0) {
    return(abs(round(AWS-SOG)));
  }
  else {
    return(abs(round(SOG*(sin(_AWA)/sin(Alpha)))));
  }
}

//get True Wind Angle
//returns True Wind Angle given Aparent Wind Angle, Apparent Wind Speed and Speed Over Ground
//This true wind angle is bow referenced
uint16_t getTWA(uint16_t AWA, uint16_t AWS, uint16_t SOG)
{
  float _AWA = degToRad(AWA);
  float tanAlpha = sin(_AWA)/(float(AWS)/float(SOG)-cos(_AWA));
  float Alpha = atan(tanAlpha);
  float tdiff = round(radToDeg(_AWA+Alpha));
  if(AWA == 0) {      //handle the singularity
    if(AWS >= SOG)    //if the wind is faster than we are moving 
      return(0);        //the wind is from dead ahead
    else              //otherwise
      return(180);      //the wind is from dead behind
  }
    
  return((uint16_t(tdiff)+360) % 360);  
}

//get True Wind Direction
//Returns True Wind Direction given Apparent Wind Angle, Apparent Wind Speed, Speed Over Ground, and Course Over Ground
//This is a compass heading based on the given COG
uint16_t getTWD(uint16_t AWA, uint16_t AWS, uint16_t SOG, uint16_t COG) { 
  return (COG + getTWA(AWA, AWS, SOG) + 360) % 360; 
}

//Reads the values in the WINDSTAT.LOG file on the SD card and returns the averages by reference.
//The function always just opens the file and calculates the averages you should check if new information is present before calling this.
void ProcessStatistics(uint32_t &speedAccum, uint32_t &boatSpeedAccum, uint16_t &AvWindDir)
{
  windStats.close();  //close the file that has been being logged to

  int32_t sinAccum, cosAccum;
  uint16_t i = 0, count = 0, l = 0;
  int16_t j = 0, k = 0;
  
  speedAccum = 0;
  sinAccum = 0;
  cosAccum = 0;
  boatSpeedAccum = 0;
  
  if(windStats.open("WINDSTAT.LOG", O_READ))
  { 
    while (windStats.available()) {
      csvReadUint16(&windStats, &i, ',');  //read off the speed
      //cout << "spd[" << count << "]:" << i;
      speedAccum += i;
      csvReadInt16(&windStats, &j, ',');  //read off the sin component of the wind direction
      //cout << " sin[" << count << "]:" << j;
      sinAccum += j;
      csvReadInt16(&windStats, &k, ',');   //read off the cos component of the wind direction
      //cout << " cos[" << count << "]:" << k << endl;
      cosAccum += k;
      csvReadUint16(&windStats, &l, ',');   //read off the boat speed 
      boatSpeedAccum += l;
      count++;
    }
    if(count)  //double check to make sure we don't divide by zero
    {
      speedAccum /= count;
      sinAccum /= count;
      cosAccum /= count;
      boatSpeedAccum /= count;
    }
    //cout << "spdAcc:" << speedAccum << " sinAcc:" << sinAccum << " cosAcc:" << cosAccum << endl;
    //cout << int(round(radToDeg(atan2(sinAccum, cosAccum))) + 360) % 360 << endl;
    AvWindDir = int(round(radToDeg(atan2(sinAccum, cosAccum))) + 360) % 360;

  }
  windStats.close();      //close the file to let the logger have it back
}

//A function that allows for finding out how much free memory is available.
extern "C" char *sbrk(int i); 
int freeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}
///////////////////////////////////////////////Compass/Accelerometer Helper Functions////////////////////////////////////////////////////
//Just a way to print the calibration information to the screen
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.println(calibData.accel_offset_z);

    Serial.print("Gyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.println(calibData.gyro_offset_z);

    Serial.print("Mag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.println(calibData.mag_offset_z); 

    Serial.print("Accel Radius: ");
    Serial.println(calibData.accel_radius);

    Serial.print("Mag Radius: ");
    Serial.println(calibData.mag_radius);
}

///////////////////////////////////////////////14 Segment Display Helper Functions///////////////////////////////////////////////////////
//displays a fixed string on the display
void displayString(char s[4]) {
  for(uint8_t i=0; i < 4; i++ ) {
    if(s[i+1] == '.')
      alpha4.writeDigitAscii(i, s[i], 1);
    else
      alpha4.writeDigitAscii(i, s[i], 0);
    alpha4.writeDisplay();
  }
}

//shows an angle and allows for a last char such as 'T' for true or 'M' for magnetic.  LastChar of null defaults to a degrees symbol.
void displayAngle(uint16_t val, char lastChar)
{
  char s[4];
  sprintf(s,"%i%i%i",val/100%10,val/10%10,val%10);
  alpha4.writeDigitAscii(0,s[0]);
  alpha4.writeDigitAscii(1,s[1]);
  alpha4.writeDigitAscii(2,s[2]);
  if(lastChar == '\0')
    alpha4.writeDigitRaw(3,0b000000011100011);  //degrees symbol
  else
    alpha4.writeDigitAscii(3,lastChar);
  alpha4.writeDisplay();
}

//Scrolls a string across the display and returns after scrolling has completed.
//This function can take consideratble time for longer strings and thus can be interrupted if a gesture is sensed via interrupt.
void scrollString(const char s[], uint16_t speed)
{
  uint8_t size = strlen(s) + 1;
  alpha4.clear();
  alpha4.writeDisplay();
  for(int i = 0; i < size-1; i++)
  {
    alpha4.writeDigitAscii(3, s[i]);
    if(i>0) alpha4.writeDigitAscii(2, s[i-1]);
    if(i>1) alpha4.writeDigitAscii(1, s[i-2]);
    if(i>2) alpha4.writeDigitAscii(0, s[i-3]);
    alpha4.writeDisplay();
    if(gestureSensed) return;  //this allows you to exit a mode while the menu scrolls
    delay(speed);
  }
  delay(speed*2);
}

void showLinkLostMessage ()
{
  //scrollString("WAITING FOR MASTHEAD UNIT\0", informationScrollSpeed);
  displayString("WAIT");  //scrolling the string blocks for too long so just keeping it simple.
}

//Allows displaying an integer value as the float that it represents.  Thus 123 will be shown as 1.23.  1234 will be shown as 12.34.
void displayIntFloat(int val, char lastChar = '\0')  //displays 2 digits of precision and expects the value as an int (float val * 100)
{
  if(lastChar == 'V' || lastChar == 'v')
  {
    val = val*10;
  }
  char s[5];
  sprintf(s,"%i%i%i%i",val/1000%10,val/100%10,val/10%10,val%10);
  if(lastChar == 'V' || lastChar == 'v')
    alpha4.writeDigitAscii(0,s[0], true);  //display the decimal point higher up if displaying voltage
  else
    alpha4.writeDigitAscii(0,s[0]);
  
  if(lastChar == 'V' || lastChar == 'v')
    alpha4.writeDigitAscii(1,s[1]);
  else
    alpha4.writeDigitAscii(1,s[1], true);   //display the decimal point to give 2 points of precision normally (if lastChar = \0)

  alpha4.writeDigitAscii(2,s[2]);
  if(lastChar != '\0') alpha4.writeDigitAscii(3,lastChar);
  else alpha4.writeDigitAscii(3,s[3]);
  alpha4.writeDisplay();  //finally update the display
}

//Returns the current barometric pressure as an integer (actual barometric pressure * 100)
int getBaro()
{
  if(baroRefAlt == -1) {
    //first wait until new GPS data available. 
    //This should probably be done less often but meh what if someone wants it to update regularly....?
    if(globalFix.valid.altitude)
    {
      float alt = globalFix.altitude();

      //constants slightly different because GPS altitude is in meters
      return round( baro.readPressure()*pow((1-(0.0065*alt)/(baro.readTemperature()+0.0065*alt+273.15)),-5.257)*0.0295301 );
    }
  }
  else
    return round( baro.readPressure()*pow((1-(0.0019812*baroRefAlt)/(baro.readTemperature()+0.0019812*baroRefAlt+273.15)),-5.257)*0.0295301 );
}

//Reads and displays the current temperature on the display.
void displayTemp(char units)
{
  if(units == 'c' || units == 'C')
    #ifdef useBaroTemp
      displayIntFloat(baro.readTemperature()*100, 'C');         //use baro pressure sensor for temp
    #endif
    #ifdef useBnoTemp
      displayIntFloat(bno.getTemp()*100, 'C');                //use IMU chip for temp
    #endif
  else if(units == 'f' || units == 'F')
    #ifdef useBaroTemp
      displayIntFloat(ctof(baro.readTemperature())*100, 'F');   //use baro pressure sensor for temp
    #endif
    #ifdef useBnoTemp
      displayIntFloat(ctof(bno.getTemp())*100,'F');           //use IMU chip for temp
    #endif
}


///////////////////////////////////////////////////////////Interrupt Handlers///////////////////////////////////////////////
#ifndef LoRaRadioPresent  //The anemometer interrupts are not needed if we don't have one connected
  void isrSpeed() {
    Peet.processSpeedTransition();
  }
  void isrDirection() {
    Peet.processDirTransition();
  }
#endif

void TC5_Handler (void) {   //This timer gets moved to the mast head in a wireless version of this project
    #ifndef LoRaRadioPresent  //The anemometer interrupts are not needed if we don't have one connected
      Peet.slowTimer();
    #endif

    //set link lost flag if no message has been received in the radioTimeoutReset period.
    if(millis() > lastMessageTime + radioTimeoutReset) {
      linkLost = true;
    }

    TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
  }

void isrGesture () {
  gestureSensed = true;
  detachInterrupt(digitalPinToInterrupt(GESTURE_INT));  //trying this because its in the example but it doesn't seem to make any difference.
}


/////////////////////////////////////////////////////LED Ring Handling Functions////////////////////////////////////////////

//Sweeps up red and green on the left and right sides of the display with a yellow pixel fore and aft
void animateRedGreenWipe(uint8_t wait) {
  //This is a background setting animation.
  uint8_t bottomLED;
  bottomLED = strip.numPixels()/2;  //calculate it here so it only has to be done once
  
  setBackgroundPixel(bottomLED, DIM_YELLOW);  
  strip.show();
  delay(wait);
  
  for(uint8_t i=1; i<bottomLED; i++) {
    setBackgroundPixel(bottomLED-i, DIM_GREEN);
    setBackgroundPixel(bottomLED+i, DIM_RED);
    strip.show();
    delay(wait);
  }
  setBackgroundPixel(0, DIM_YELLOW);
  strip.show();
} //animateRedGreenWipe

//Wraps a single color around the ring in a circle from the top.
void animateSolidColor(uint32_t c, uint8_t wait) {
  //This is a background setting animation.
  for(uint8_t i=0; i < strip.numPixels(); i++) {
    setBackgroundPixel(i, c);
    strip.show();
    delay(wait);
  }
} //animateSolidColor

//Writes a color into the background pixel array
void setBackgroundPixel (uint8_t pixel, uint32_t color) {
  //sets a pixel color in the strip and also updates the background buffer
  strip.setPixelColor(pixel, color);
  pixelBackground[pixel] = color;
} //setBackgroundPixel

//Writes the current background pixel array onto the LED ring (essentially clears things to the background)
void restoreBackground() {
  //This function will restore the pixel values to a blank background value
  for(uint8_t i=0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, pixelBackground[i]);
  }
} //restoreBackground

//This updates the LED ring with the lights corresponding to the passed angle
//Essentially it updates the led that represents that angle to the specificed color.
void displayWindPixel (uint16_t angle, uint32_t color)
{
  static float delta = 360.0 / strip.numPixels() / 2.0;  //static so its only calculated once
  int bin = round(angle/delta);
  int halfBin = trunc(bin/2);
  
  restoreBackground();

  if(halfBin == strip.numPixels()) strip.setPixelColor(0, color);
  else strip.setPixelColor(halfBin, color);
   
  if(bin%2) {
    if(halfBin+1 == strip.numPixels()) strip.setPixelColor(0, color);
    else strip.setPixelColor(halfBin+1, color);
  }
  strip.show();
}

//Just blinks an LED you can set the number of times and the duration of the blinks
//This only works for the LEDs on the GPIO pins directly on the Arduino board.
static void blip(int ledPin, int times, int dur) {
  pinMode(ledPin, OUTPUT);
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(dur);
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(dur);
  }
} //blip

//This is a way of saying we've died on the vine.  This function is meant to never return.
static void failBlink() {
  pinMode(RED_LED_PIN, OUTPUT );  //seems that I need this here because something is breaking this.
  #ifdef debug
    cout << "got to FailBlink" << endl;
  #endif
  displayString("FAIL");
  while(1) blip(RED_LED_PIN, 1, 75);
} //failBlink

///////////////////////////////////////////////////////SD Card Handling Functions////////////////////////////////////////////
static bool readConfig () {
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////sd.chdir("!CONFIG"); sd.vwd()->rmRfStar(); sd.chdir("/");      //using this will torch the entire config directory (IMU cal data too)
  //sd.remove("/!CONFIG/BTH_WIND.CFG");                            //this will delete the main config file and restore it to defaults
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (!cfg.begin("/!CONFIG/BTH_WIND.CFG", 100)) {
    sd.mkdir("!CONFIG");
    #ifdef debug
      Serial.println(F("Failed to open config file"));
      Serial.println(F("Creating Default Config File"));
    #endif
    cfg.end();  //close the cfg instance so as to use a normal one
    
    if (!configFile.open("/!CONFIG/BTH_WIND.CFG", O_CREAT | O_WRITE | O_EXCL)) { //create new file
      #ifdef debug
        Serial.println(F("Couldn't open new config file"));
      #endif
      failBlink();
      return 0;
    }
    else {
      configFile.print(F("#############################################################################################\n"));
      configFile.print(F("# BTHWindInstrument v0.1 Configuration by Brett Howard\n"));
      configFile.print(F("# BowOffset: Used to correct for the difference between anemometer north and your Bow\n"));
      configFile.print(F("# MagVariance: Magnetic variance between true and magnetic north (East = Negative)\n"));
      configFile.print(F("# HeelAngle: The angle at which you want to switch to displaying a digital heel angle (0 disables)\n"));
      configFile.print(F("# MenuScrollSpeed: The number of mS to delay each character when scrolling menu item titles\n"));
      configFile.print(F("# TempUnits: c for Celcius f for Fahrenheit\n"));
      configFile.print(F("# SpeedMAD: Speed Moving Average Depth (this smooths and averages the wind speed data)\n"));
      configFile.print(F("# TrueHeadAvgDepth: Number of samples to average for true wind heading menu item.  Samples are taken at WindUpdateRate\n"));
      configFile.print(F("# WindUpdateRate: Minimum delay between display updates for wind speed modes.\n"));
      configFile.print(F("# DirectionFilter: range (1-1000); lower = more filtering; 1000=no filtering\n"));
      configFile.print(F("#    Each wind direction delta is multiplied by DirectionFilter/1000\n"));
      configFile.print(F("# GPSUpdateRate: Time in mS Between GPS fix updates\n"));
      configFile.print(F("# BaroRefAlt: Barometer reference Altitude (in feet) put in 0 to report \"station pressure\"\n"));
      configFile.print(F("#    Setting the BaroRefAlt to -1 will tell the unit to use the GPS altitude for the calulation\n"));
      configFile.print(F("# GPXLogging: If true the unit will log your tracks in GPX format to the SD card\n"));
      configFile.print(F("# HomeLat: Latitude of your slip in integer format (multiply by 1e7)\n"));
      configFile.print(F("# HomeLon: Longitude of your slip in integer format (multiply by 1e7)\n"));
      configFile.print(F("# HomeStatRadius: Distance you must go before the statistics collection activates (0 disables)\n"));
      configFile.print(F("# HomeRadius: Distance you must go before the GPS tracking activates (0 disables)\n"));
      configFile.print(F("# TrackName: A name to be associated into your GPX log files\n"));
      configFile.print(F("# \n"));
      configFile.print(F("# Timezone Setup:\n"));
      configFile.print(F("# Provide a name and configure when each rule occurs\n"));
      configFile.print(F("# For week 1=Last 2=First 3=Second 4=Third 5=Fourth\n"));
      configFile.print(F("# For day of week 1=Sun 2=Mon 3=Tue 4=Wed 5=Thu 6=Fri 7=Sat\n"));
      configFile.print(F("# For month 1=Jan 2=Feb 3=Mar 4=Apr 5=May 6=Jun 7=Jul 8=Aug 9=Sep 10=Oct 11=Nov 12=Dec\n"));
      configFile.print(F("# For hour input the time that the adjustment is to take place\n"));
      configFile.print(F("# For offset input the time offset in minutes\n"));
      configFile.print(F("# If you wish to log only in UTC just setup a timezone with only zero offset values\n"));
      configFile.print(F("# \n"));
      configFile.print(F("#############################################################################################\n\n"));

      configFile.print(F("BowOffset=345\n"));
      configFile.print(F("MagVariance=15\n"));
      configFile.print(F("HeelAngle=20\n"));
      configFile.print(F("MenuScrollSpeed=80\n"));
      configFile.print(F("TempUnits=f\n"));
      configFile.print(F("SpeedMAD=5\n"));
      configFile.print(F("TrueHeadAvgDepth=30\n"));   //True wind Heading Average depth (number of samples to average taken once per WindUpdateRate)          
      configFile.print(F("WindUpdateRate=1000\n"));   //500 repaints the display at a 2Hz rate, 1000 is 1Hz
      configFile.print(F("DirectionFilter=250\n"));  //250 displays 1/4 of the actual delta on each update
      configFile.print(F("GPSUpdateRate=1000\n"));    //1Hz 
      configFile.print(F("BaroRefAlt=374\n"));         //374 feet is full pool elevation for Fern Ridge Reservoir, Eugene, OR
      configFile.print(F("GPXLogging=true\n"));
      configFile.print(F("HomeLat=441189070\n"));       //location of slip B32 at Richardson Park Marina
      configFile.print(F("HomeLon=-1233155660\n"));
      configFile.print(F("HomeStatRadius=350\n"));         //covers just about to the edge of the Eugene Yacht Club
      configFile.print(F("HomeGPSRadius=50\n"));           //set to be fairly small but big enough to thwart false positives.
      configFile.print(F("TrackName=Uncomfortably Level\n"));  //Boat name
      configFile.print(F("\n"));
      configFile.print(F("DSTName=PDT\n"));          //defaults to US Pacific
      configFile.print(F("DSTWeek=2\n"));            //first
      configFile.print(F("DSTDayOfWeek=1\n"));       //Sunday
      configFile.print(F("DSTMonth=3\n"));           //in March
      configFile.print(F("DSTHour=2\n"));            //at 2AM
      configFile.print(F("DSTOffset=-420\n"));       //subtract 7 hours
      configFile.print(F("\n"));
      configFile.print(F("STName=PST\n"));           //Pacific STD time
      configFile.print(F("STWeek=2\n"));             //first
      configFile.print(F("STDayOfWeek=1\n"));        //Sunday
      configFile.print(F("STMonth=11\n"));           //in November
      configFile.print(F("STHour=2\n"));             //at 2AM
      configFile.print(F("STOffset=-420\n"));        //subtract 8 hours (made wrong because it seems to not be working at the momnet)
      
      configFile.close();
      blip(GREEN_LED_PIN, 5, 200);
    }
    //open the new file to read in the vals
    if(!cfg.begin("/!CONFIG/BTH_WIND.CFG", 100)) { 
      Serial.println(F("Failed to open the newly created config")); 
      failBlink(); 
    }  
  }

  //Fetch Configuration Values
  while (cfg.readNextSetting())
  {
    if (cfg.nameIs("BowOffset")) { bowOffset = cfg.getIntValue(); }
    if (cfg.nameIs("MagVariance")) { variance = cfg.getIntValue(); }
    if (cfg.nameIs("HeelAngle")) { heelAngle = cfg.getIntValue(); }
    if (cfg.nameIs("TempUnits")) { strncpy(&tempUnits, cfg.copyValue(), 1); }
    if (cfg.nameIs("MenuScrollSpeed")) { menuDelay = cfg.getIntValue(); }
    if (cfg.nameIs("SpeedMAD")) { speedMAD = cfg.getIntValue(); }
    if (cfg.nameIs("TrueHeadAvgDepth")) { TrueHeadAvgDepth = cfg.getIntValue(); }
    if (cfg.nameIs("WindUpdateRate")) { windUpdateRate = cfg.getIntValue(); }
    if (cfg.nameIs("DirectionFilter")) { directionFilter = cfg.getIntValue(); }
    if (cfg.nameIs("GPSUpdateRate")) { delayBetweenFixes = cfg.getIntValue(); }
    if (cfg.nameIs("BaroRefAlt")) { baroRefAlt = cfg.getIntValue(); }
    if (cfg.nameIs("GPXLogging")) { GPXLogging = cfg.getBooleanValue(); }
    if (cfg.nameIs("HomeLat")) { homeLat = cfg.getIntValue(); }
    if (cfg.nameIs("HomeLon")) { homeLon = cfg.getIntValue(); }
    if (cfg.nameIs("HomeStatRadius")) { homeStatRadius = float(cfg.getIntValue()/1000.0); }
    if (cfg.nameIs("HomeGPSRadius")) { homeGPSRadius = float(cfg.getIntValue()/1000.0); }
    if (cfg.nameIs("TrackName")) { strcpy(trackName, cfg.copyValue()); }

    if (cfg.nameIs("DSTName")) { strcpy(DSTrule.abbrev, cfg.copyValue()); }
    if (cfg.nameIs("DSTWeek")) { DSTrule.week = uint8_t(cfg.getIntValue()); }
    if (cfg.nameIs("DSTDayOfWeek")) { DSTrule.dow = cfg.getIntValue(); }
    if (cfg.nameIs("DSTMonth")) { DSTrule.month = cfg.getIntValue(); }
    if (cfg.nameIs("DSTHour")) { DSTrule.hour = cfg.getIntValue(); }
    if (cfg.nameIs("DSTOffset")) { DSTrule.offset = cfg.getIntValue(); }
    
    if (cfg.nameIs("STName")) { strcpy(STrule.abbrev, cfg.copyValue()); }
    if (cfg.nameIs("STWeek")) { STrule.week = cfg.getIntValue(); }
    if (cfg.nameIs("STDayOfWeek")) { STrule.dow = cfg.getIntValue(); }
    if (cfg.nameIs("STMonth")) { STrule.month = cfg.getIntValue(); }
    if (cfg.nameIs("STHour")) { STrule.hour = cfg.getIntValue(); }
    if (cfg.nameIs("STOffset")) { STrule.offset = cfg.getIntValue(); }
  }

  cfg.end();  //clean up
  return true;
}  //readConfig

//This only initializes the SPI port to be able to talk to the SD card
bool initSD() {
  bool retval;
  //while (!gps.available());  //wait for a fix
  //globalFix = gps.read();
  #ifdef debug
    Serial.println( F("Initializing SD card...") );
  #endif
  
  // see if the card is present and can be initialized:
  if (!sd.begin(SD_CHIP_SEL)) {
    #ifdef debug
      Serial.println( F("SD card failed, or not present") );
    #endif
    failBlink();  //dies here (never returns)  
  }
  #ifdef debug
    Serial.println( F("SD card initialized.") );
    retval = true;
  #endif
  
  return retval;
}  //initSD

///////////////////////////////////////////////////////FAT File System Handling Functions//////////////////////////////////////


//This is the callback function for the SdFat file system library so that it can properly timestamp files it modifies and creates on the SD card
void dateTime(uint16_t* date, uint16_t* time) {
  time_t local;
  local = getLocalTime();

  *date = FAT_DATE(year(local), month(local), day(local));
  *time = FAT_TIME(hour(local), minute(local), second(local));
}

///////////////////////////////////////////////////////CSV File Handling Functions////////////////////////////////////////////


//these read functions should be cleaned up later they were just copied from one of the SdFat libraries and are more complex than probably necessary
int csvReadText(SdFile* file, char* str, size_t size, char delim) {
  char ch;
  int rtn;
  size_t n = 0;
  while (true) {
    // check for EOF
    if (!file->available()) {
      rtn = 0;
      break;
    }
    if (file->read(&ch, 1) != 1) {
      // read error
      rtn = -1;
      break;
    }
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    if (ch == delim || ch == '\n') {
      rtn = ch;
      break;
    }
    if ((n + 1) >= size) {
      // string too long
      rtn = -2;
      n--;
      break;
    }
    str[n++] = ch;
  }
  str[n] = '\0';
  return rtn;
}  //csvReadText

int csvReadUint32(SdFile* file, uint32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtoul(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while(isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadUint16(SdFile* file, uint16_t* num, char delim) {
  uint32_t tmp;
  int rtn = csvReadUint32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp > UINT_MAX) return -5;
  *num = tmp;
  return rtn;
}

int csvReadInt32(SdFile* file, int32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtol(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while(isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadInt16(SdFile* file, int16_t* num, char delim) {
  int32_t tmp;
  int rtn = csvReadInt32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp < INT_MIN || tmp > INT_MAX) return -5;
  *num = tmp;
  return tmp;
}

//////////////////////////////////////////////////////Timer Counter Configuration///////////////////////////////////////////////////
//configures the timer counter to allow for a slow tick to use if/when the wind gets so low the anemometer interrupts stop
void tcConfigure(int sampleRate)
{
  // Enable GCLK for TCC2 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset(); //reset TC5

  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT32;
  // Set TC5 mode as match frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  //set prescaler and enable TC5
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  delay(10);

  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  delay(10); //wait until TC5 is done syncing 
} 
//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  delay(10);
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}
//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  delay(10);
}

//////////////////////////////////////////////////////GPS helper Fucntions/////////////////////////////////////////////////////////////////////

//Returns a time_t (seconds since the epoch) of the current local time.
time_t getLocalTime()   //this function requires that you edit NeoTime.h to use the POSIX epoch or the time will be off by 30 years.
{
  time_t utc;

  if(globalFix.valid.time && globalFix.valid.date) {    //don't do the work if we don't have valid data
    utc = globalFix.dateTime;                           //sets utc to number of seconds since the epoch
    
    time_t local = localTZ.toLocal(globalFix.dateTime);              //returns time_t with current local time.
    
    #ifdef debug
      #ifdef showClockTime
        char timeStr[22];
        sprintf(timeStr, "%4d-%02d-%02d %02d:%02d", year(local), month(local), day(local), hour(local), minute(local));
        cout << "At the tone the time will be: " << timeStr << endl;
      #endif
    #endif

    return local;
  }
  else
    return false;
}

//this is an endless loop that fetches data from the GPS and returns when you have a valid location date and time.
//A status string is scrolled on the display to inform the user that we are waiting for the GPS to obtain a valid fix.
static void waitForFix()
{
  #ifdef debug
    Serial.print( F("Waiting for GPS fix...") );
  #endif

  uint16_t lastToggle = millis();

  #ifdef LoRaRadioPresent
    uint8_t data[3] = "A\0";
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
  #endif

  for (;;) {
    while (Serial1.available()) { gps.handle(Serial1.read()); }
    if (gps.available()) {
      globalFix = gps.read();
      if (globalFix.valid.location && globalFix.valid.date && globalFix.valid.time)
        break; // Got it!
    }

    //reply to mast head transmitter messages to send it (and keep it) in high speed transmission mode while we wait for the GPS Fix
    #ifdef LoRaRadioPresent
      if (rf95.available()) {
        rf95.recv(buf, &len);
        rf95.send(data, sizeof(data));  //transmit ACK response
        rf95.waitPacketSent();
      }
    #endif

    if ((uint16_t) millis() - lastToggle > 1000) {
      lastToggle += 1000;
      scrollString( "WAITING FOR GPS FIX\0", informationScrollSpeed );
      #ifdef debug
        Serial.write( '.' );
      #endif
    }
  }

  
  #ifdef debug
    Serial.println();
  #endif
  gps.overrun( false ); // we had to wait a while...

} // waitForFix

//Creates folder for the date and a GPX file for the time when the function was called.  
void startGPSFile()
{ 
  if(globalFix.valid.date && globalFix.valid.time)
  {
    char directory[9];
    time_t local;
    local = getLocalTime();

    sprintf(filename, "/%02d-%02d-%02d/%02d-%02d%s", year(local)%100, month(local), day(local), hour(local), minute(local), ".GPX");
    sprintf(directory, "/%02d-%02d-%02d", year(local)%100, month(local), day(local)); 
    
    sd.mkdir(directory);
    #ifdef debug
      Serial.print(F("File to be opened: "));
      Serial.println(filename);
    #endif
  }
  else {
    #ifdef debug
      Serial.println(F("Current fix has no valid date and/or time"));
    #endif
  }

  if(!gpsLog.open(filename, O_CREAT | O_WRITE | O_TRUNC)) {  //Delete the previous and start a new file if its been less than 1 min.
    #ifdef debug
      Serial.print( F("Couldn't create ") );
      Serial.println(filename);
    #endif
    failBlink();
  }

  #ifdef debug
    Serial.print( F("Writing to ") );
    Serial.println(filename);
  #endif
  
  gpsLog.print(F(
                  "<?xml version=\"1.0\" encoding=\"ISO-8859-1\" standalone=\"yes\"?>\r\n"
                  "<gpx version=\"1.1\" creator=\"BTHWindInstrument\" xmlns=\"http://www.topografix.com/GPX/1/1\" \r\n"
                  "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\r\n"
                  "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\r\n"
                  "<trk>\r\n\t<name>")); gpsLog.print(trackName); gpsLog.print(F("</name>\r\n\t<trkseg>\r\n"));  //heading of gpx file
  gpsLog.print(F("\t</trkseg>\r\n</trk>\r\n</gpx>\r\n"));
  gpsLog.close();
} // startgpsLog

//Snags the values from the GPS and or other locations and updates the GPX log with a new track segment
//this function assumes that startGPSFile() has been called previously
static void WriteGPXLog()
{ 
  // Log the fix information if we have a location and time
  if (globalFix.valid.location && globalFix.valid.time) {
    char date1[22];
    snprintf(date1, 21, "%4d-%02d-%02dT%02d:%02d:%02dZ", globalFix.dateTime.full_year(), globalFix.dateTime.month, 
            globalFix.dateTime.date, globalFix.dateTime.hours, globalFix.dateTime.minutes, globalFix.dateTime.seconds);
    
    while(!gpsLog.isOpen()) gpsLog.open(filename, O_WRITE);

    while(!gpsLog.seekSet(gpsLog.fileSize() - 28));  //seeks back to delete the closing tags from the end of the file
    
    //starts a new track point record
    gpsLog.print(F("\t\t<trkpt lat=\""));

    //write out each data element only if the fix is valid for that data element
    if (globalFix.valid.location) {
      gpsLog.print(globalFix.latitude(), 7);
      gpsLog.print(F("\" lon=\""));
      gpsLog.print(globalFix.longitude(), 7);
      gpsLog.print(F("\">\n"));
    }

    if (globalFix.valid.altitude) {
      gpsLog.print(F("\t\t\t<ele>"));
      gpsLog.print(globalFix.altitude(), 2);
      gpsLog.println(F("</ele>"));
    }

    if (globalFix.valid.time) {
      gpsLog.print(F("\t\t\t<time>"));
      gpsLog.print(date1);
      gpsLog.print(F("</time>\n"));
    }

    if (globalFix.valid.speed) {
      gpsLog.print(F("\t\t\t<speed>"));
      gpsLog.print(globalFix.speed()*0.514444);  //magic number is to convert knots to m/s);
      gpsLog.print(F("</speed>\n"));
    }

    if (globalFix.valid.hdop) {
      gpsLog.print(F("\t\t\t<hdop>"));
      gpsLog.print(float(globalFix.hdop)/float(1000), 3);
      gpsLog.print(F("</hdop>\n"));
    }

    if (globalFix.valid.heading) {
      gpsLog.print(F("\t\t\t<course>"));
      gpsLog.print(globalFix.heading());
      gpsLog.println(F("</course>"));
    }

    gpsLog.print(F("\t\t\t<sat>"));
    gpsLog.print(globalFix.satellites);
    gpsLog.println(F("</sat>"));

    gpsLog.print(F("\t\t</trkpt>\n"));

    //replace closing tags that were overwritten from the seekSet()
    gpsLog.print(F("\t</trkseg>\r\n</trk>\r\n</gpx>\r\n"));

    while(!gpsLog.close());  //closing the file flushes the cache and makes the file valid if you lose power without notice.
    blip(RED_LED_PIN, 1, 20);  //blink the LED to give status that a GPS point has been logged to SD file.
  }
}