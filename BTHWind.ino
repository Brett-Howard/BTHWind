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
#include "SparkFun_APDS9960.h"          //Sparkfun APDS9960 Ambient Light/Gesutre Sensor library (which actually works)
                                        //using a local copy to allow for shortening the gesutre delay
#include <NMEAGPS.h>                    //GPS Support
#include <GPSport.h>                    //GPS Support
#include <RH_RF95.h>                    //LoRa Radio support

//I2C Address Information (just to make sure there are no collisions)
//BNO055 - 28h or 29h
//APDS9960 - 39h
//BMP280 - 77h
//LED Backpack - 70h

#define debug             //comment this out to not depend on USB uart.
//#define noisyDebug        //For those days when you need more information (this also requires debug to be on)
#define LoRaRadioPresent  //comment this line out to start using the unit with a wireless wind transducer

#define batteryLogInterval 600000  //every 10 minutes

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
SDConfigFile cfg;
SdFile logfile;
SdFile windStats;
SdFile compCal;
SdFile gpsLog;
SdFile battFile;

ArduinoOutStream cout(Serial);

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

enum Mode { AppWind, WindStats, TrueWind, CompHead, COG, SOG, Baro, Temp, Heel, MastBatt }; 

int16_t bowOffset, variance;
uint8_t speedMAD;
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
uint8_t startHours = 0, startMinutes = 0, curHours = 0, curMinutes = 0;
int32_t homeLat, homeLon;
float homeStatRadius, homeGPSRadius;

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

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

  alpha4.begin(0x70);  // pass in the address
  scrollString("BTHWind\0", 175);
  
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

  waitForFix();  //Sit here and wait until the GPS locks onto the network of sats.

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
///////////////////////////////////////////Initialize SD Card/////////////////////////////////////////////////////////
  if(initSD()) {
    //Serial.print(F("Card size: ")); Serial.print(sd.card()->cardSize() * 0.000512 + 0.5); Serial.println(" MiB");
    //removing card free because it takes a while on a 16GB card.
    //Serial.print(F("Card free: ")); Serial.print(sd.vol()->freeClusterCount() * .000512 * sd.vol()->blocksPerCluster()); Serial.println(" MiB");
  }
  
//////////////////////////////////////Startup Magnetometer and Accelerometer//////////////////////////////////////////
  if(!bno.begin())
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
  
  //if there isn't a IMU calibration file print status while you calibrate
  if(!sd.exists("/!CONFIG/IMUCAL.CSV")) {
    displayString("IMU?");
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
  
  if(!readConfig()) {  //read configuration from SD card
    #ifdef debug
	    Serial.println(F("Failed to create and/or read configuration file from SD"));
	  #endif
	  failBlink();  //dies here (never returns)
  }
  #ifdef debug
	  Serial.println(F("SD card configuration read successfully"));
	#endif
  
  //write update rate into the GPS  (has to be moved here after we've fetched the value from the config)
  char tmp[13];
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
    tcConfigure(1000);  //1 second timer 
  #endif
    //if there is a LoRa radio the radio data is polled on and given to the anemometer object when received in the main loop.
    attachInterrupt(digitalPinToInterrupt(GESTURE_INT),isrGesture, FALLING);
  
  uint16_t w;
  apds.readAmbientLight(w);
  strip.setBrightness(map(w,0,37889,5,255));  //set the brightness for the power up sequence so it doesn't change after powerup.
  animateRedGreenWipe(60);  //pretty startup animation

  while(!gps.available())
    if(Serial1.available())
      gps.handle(Serial1.read());   //inject stuff into the GPS object until a valid fix is puked out
    
  globalFix = gps.read();       //snag the fix.
  //uint8_t startDay;  //I don't really need this so I'm just making a place holder that will go away.
  
  //getLocalTime(&startHours, &startDay);
  //startMinutes = globalFix.dateTime.minutes;  //figure out what time we started sailing
  
  //curHours = startHours;
  //curMinutes = startMinutes; //just make the start and stop times the same so it doesn't show 0000 if you go in to stats immediately
}

void displayIntFloat(int, char);  //compiler wants this function and only this one listed here for some reason.
volatile bool gestureSensed = false;

void loop() {
  static Mode curMode = TrueWind;
  static Mode prevMode;
  uint8_t gesture; 
  static uint16_t w,wndSpd,windMax = 0;
  static bool firstEntry = 1;
  static int curHeelAngle;
  static uint32_t tempTimer;
  static bool locked = false;
  static bool newSDData = false;
  static uint16_t battVoltage;
  static sensors_event_t compEvent;
  static uint8_t compTimer;
  static uint32_t battTimer;
  static bool GPXLogStarted = false;
  static uint32_t speedAccum, boatSpeedAccum;
  static int32_t sinAccum, cosAccum;
  static uint16_t AvWindDir;
  static bool tripStarted = false;

  //adjust wind ring brightness based on ambient light
  apds.readAmbientLight(w);
  strip.setBrightness(map(w,0,37889,5,255));
  strip.show();

  //Log battery voltage to a CSV file for graphing to understand how well the masthead unit's solar setup is working
  if(millis() > battTimer + batteryLogInterval)
  {
    uint8_t localHour;
    byte localDay;
    getLocalTime(&localHour, &localDay);
    char date1[22];
    battFile.open("/!CONFIG/BATTERY.CSV", O_WRITE | O_CREAT | O_APPEND);
    sprintf(date1, "%4d-%02d-%02d %02d:%02d", globalFix.dateTime.full_year(), globalFix.dateTime.month, localDay, localHour, globalFix.dateTime.minutes);
    battFile.print(date1); battFile.print(F(", ")); battFile.println(battVoltage);
    battFile.close();
    #ifdef debug
      cout << "Printing Battery File " << date1 << ',' << battVoltage << endl;
    #endif
    battTimer = millis();
  }

  //Determine if we're over heeling
  if(millis() > compTimer + 50) {   //only read compass and heel information every 50mS (20Hz) because it doesn't update any faster than that
    bno.getEvent(&compEvent);
    curHeelAngle = abs(compEvent.orientation.y);
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
  case AppWind:
      if(firstEntry) {
        scrollString("APPARENT WIND\0", menuDelay);
        firstEntry = false;
        restoreBackground();
      }
      ///////do AppWind
      if(wndSpd > 0) {
        if(millis() > tempTimer + windUpdateRate) {
          displayIntFloat(wndSpd,'\0');
          tempTimer = millis();
        }
      }
      else { displayString("CALM"); }
      #ifdef noisyDebug
        cout << "AWS: "  << wndSpd << " AWA: " << Peet.getDirection() << endl;
      #endif
      //////Transition state
      if(gesture == DIR_LEFT) { curMode = Baro; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_UP) { curMode = WindStats; firstEntry = true; }
      else if(gesture == DIR_DOWN) { curMode = TrueWind; firstEntry = true; }
      break;

      case TrueWind:
      if(firstEntry) {
        scrollString("TRUE WIND\0", menuDelay);
        firstEntry = false;
      }
      //////////////do TrueWind
      uint16_t _SOG;
      uint16_t _AWS;
      uint16_t _AWA;
      uint16_t _TWS;

      //if(gps.available())
        //globalFix = gps.read();    //this not needed because of the global read below
      if (globalFix.valid.speed) {
        _SOG = globalFix.speed()*100;
      }

      _AWA = Peet.getDirection();
      
      //_SOG = 700;     //remove this later (only for testing)
      //_AWA = 88;
      //_AWS = 284;
      if(millis() > tempTimer + windUpdateRate) {
        #ifdef noisyDebug
          cout << "AWA: "  << _AWA << " AWS: " << wndSpd << " SOG: " << _SOG << endl;
        #endif
        
        if(wndSpd > 0) {
          displayIntFloat(getTWS(_AWA, wndSpd, _SOG), '\0');
          displayWindPixel(getTWA(_AWA, wndSpd, _SOG), WHITE);
        }
        else { displayString("CALM"); }

        tempTimer = millis();
      }   
      
      //////////////Transition State
      if(gesture == DIR_LEFT) { curMode = Baro; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_UP) { curMode = AppWind; firstEntry = true; }
      else if(gesture == DIR_DOWN) { curMode = WindStats; firstEntry = true; }
      break;
  
  
  case WindStats:
      if(firstEntry || newSDData) {
        scrollString("SAIL STATS\0", menuDelay);
        firstEntry = false;
        newSDData = false;
        
        displayString("WAIT");
        
        windStats.close();  //close the file that has been being logged to
        //Reading the file in
        //only read the file in on first entry to the menu entry
        
        uint16_t i = 0, count = 0, l = 0;
        int16_t j = 0, k = 0;
        
        speedAccum = 0;
        sinAccum = 0;
        cosAccum = 0;
        boatSpeedAccum = 0;
        count = 0;
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
          speedAccum /= count;
          sinAccum /= count;
          cosAccum /= count;
          boatSpeedAccum /= count;
          //cout << "spdAcc:" << speedAccum << " sinAcc:" << sinAccum << " cosAcc:" << cosAccum << endl;
          //cout << int(round(radToDeg(atan2(sinAccum, cosAccum))) + 360) % 360 << endl;
          AvWindDir = int(round(radToDeg(atan2(sinAccum, cosAccum))) + 360) % 360;

        }
        windStats.close();
        tempTimer = millis();
      }
      /////////////do WindStats

      //show the values on the display
      if(millis() > tempTimer && millis() < tempTimer+1000) {   //this timing method is really annoying but its good to keep loop moving faster
        displayString("STRT");
      }
      else if(millis() > tempTimer+1000 && millis() < tempTimer+3000) {
        char temp[5];
        sprintf(temp, "%02u%02u", startHours, startMinutes);
        displayString(temp);
      }
      else if(millis() > tempTimer+3000 && millis() < tempTimer+4000) {
        displayString("END ");
      }
      else if(millis() > tempTimer+4000 && millis() < tempTimer+6000) {
        char temp[5];
        sprintf(temp, "%02u%02u", curHours, curMinutes);
        displayString(temp);
      }
      else if(millis() > tempTimer+6000 && millis() < tempTimer+7000) {
        displayString("ASOG");
      }
      else if(millis() > tempTimer+7000 && millis() < tempTimer+9000) {
        displayIntFloat(boatSpeedAccum, '\0');
      }
      else if(millis() > tempTimer+9000 && millis() < tempTimer+10000) {  
        displayString("AVG ");
      }
      else if(millis() > tempTimer+10000 && millis() < tempTimer+12000) {
        displayIntFloat(speedAccum, '\0');
      }
      else if(millis() > tempTimer+12000 && millis() < tempTimer+13000) {
        displayString("MAX ");
      }
      else if(millis() > tempTimer+13000 && millis() < tempTimer+15000) {
        displayIntFloat(windMax, '\0');
      }
      else if(millis() > tempTimer+15000 && millis() < tempTimer+16000) {
        displayString("AvWD");
      }
      else if(millis() > tempTimer+16000 && millis() < tempTimer+18000) {
        displayAngle(AvWindDir, '\0');
      }
      else if(millis() > tempTimer+18000)
        tempTimer = millis();

      ////////////Transition State
      if(gesture == DIR_LEFT) { curMode = Baro; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_UP) { curMode = TrueWind; firstEntry = true; }
      else if(gesture == DIR_DOWN) { curMode = AppWind; firstEntry = true; }
      break;
  
  case CompHead:
      if(firstEntry) {
        scrollString("COMPASS HEADING\0", menuDelay);
        firstEntry = false;
      }
      /////////////do CompHead
      
      static uint16_t heading;
      static uint8_t system, gyro, accel, mag;
      system = gyro = accel = mag = 0;

      bno.getCalibration(&system, &gyro, &accel, &mag);

      if(mag < 1)
        displayString("CAL ");
      else {
        //bno.getEvent(&compEvent);  //Don't need to get a new event because the heel detection logic does it on every loop
        
        heading = round(compEvent.orientation.x);  //round is more accurate than just letting it trunc shoot me I'm anal
        if(heading == 360) heading = 0;  //the round operation makes it possible for 360 to be reported for 359.5 to 359.99999
        displayAngle(heading, 'M');
      }

      ///////////Transition State
      if(gesture == DIR_LEFT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = MastBatt; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = COG; firstEntry = true; }
      break; 
  
  
  case COG:
      if(firstEntry) {
        scrollString("GPS COG\0", menuDelay);
        firstEntry = false;
      }
      //////////////////do COG
      //if(gps.available())
        //globalFix = gps.read();     //this read no longer needed because of the master one below
      if (globalFix.valid.speed) {
        displayAngle(uint16_t(globalFix.heading()), 'T');
      }
      //////////////////Transition State
      if(gesture == DIR_LEFT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = MastBatt; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = CompHead; firstEntry = true; }
      break; 
  
  
  case SOG:
      if(firstEntry) {
        scrollString("GPS SOG\0", menuDelay);
        firstEntry = false;
      }
      //////////////////do SOG
      //if(gps.available())
        //globalFix = gps.read();  //this read no longer needed because of the master one below
      if (globalFix.valid.speed) {
        displayIntFloat(globalFix.speed()*100, '\0');
      }

      //////////////////Transition State
      if(gesture == DIR_LEFT) { curMode = TrueWind; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = CompHead; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = SOG; firstEntry = true; }
      break;
  
  
  case Baro:
      if(firstEntry) {
        scrollString("BAROMETER\0", menuDelay);
        firstEntry = false;
      }
      ////////////////Do Baro
      if(millis() > tempTimer+1000) { 
        displayBaro();
        tempTimer = millis(); 
      } 
      ////////////////Transition State
      if(gesture == DIR_LEFT) { curMode = Temp; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = TrueWind; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = Baro; firstEntry = true; }
      break;
  
  
  case Temp:
      if(firstEntry) {
        scrollString("TEMPERATURE\0", menuDelay);
        firstEntry = false;
      }
      ///////////////Do Temp
      //update temp only once per second but in a non blocking way that doesn't slow down gesture response.
      if(millis() > tempTimer+1000) { 
        displayTemp(tempUnits); 
        tempTimer = millis(); 
      } 
      ///////////////Transition State
      if(gesture == DIR_LEFT) { curMode = MastBatt; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = Baro; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = Temp; firstEntry = true; }
      break;
  
  case MastBatt:
      if(firstEntry) {
        scrollString("MAST BATTERY\0", menuDelay);
        firstEntry = false;
      }
      ///////////////Do MastBatt
      displayIntFloat(battVoltage, 'V');
      ///////////////Transition State
      if(gesture == DIR_LEFT) { curMode = SOG; firstEntry = true; }
      else if(gesture == DIR_RIGHT) { curMode = Temp; firstEntry = true; }
      else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = MastBatt; firstEntry = true; }
      break;
  
  case Heel:
      if(firstEntry) {
        scrollString("Reduce Heel\0", menuDelay/2);
        firstEntry = false;
        tempTimer = millis();
      }
      if(millis() > tempTimer && millis() < tempTimer+5000) {
        displayAngle(curHeelAngle, '\0');
      }
      if(millis() > tempTimer+5000) {
        firstEntry = true;  //set so that upon returning you display menu heading
      }
      break;
  }
  
  
  //Fetch all data from the GPS UART and feed it to the NeoGPS object
  //I tried to put this into a SerialEvent function but that seems to not work for me so I'll just leave this here.
  while (Serial1.available()) { gps.handle(Serial1.read()); }
      
  //update the global fix to be used in menu items and for GPX logging
  if(gps.available()) {
    globalFix = gps.read();
    if(!GPXLogStarted && GPXLogging) {
      startLogFile();
      GPXLogStarted = true;
    }
    else if(GPXLogging) {
      static NeoGPS::Location_t home(homeLat, homeLon);  //create Location_t that represents slip locaiton 
      cout << "distance from home is " << globalFix.location.DistanceKm( home ) << "km\n";
      if( (homeGPSRadius <= 0) || globalFix.location.DistanceKm( home ) > homeGPSRadius ) {  
        cout << "logging gps\n";
        WriteGPXLog();
      }
    }
  }

  //Calculate statistics to be used in the SAIL STATS menu
  if(Peet.available()) {        //only true when new information has been received from the anemometer object
    
    wndSpd = Peet.getSpeed();   //wind speed should only be fetched once per loop when necessary because its an expensive operation (right here)
  
    if(wndSpd > 0 && curMode != TrueWind)  //if we have wind and aren't displaying true wind
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

    static NeoGPS::Location_t home(homeLat, homeLon);  //create Location_t that represents slip locaiton 

    cout << "time: " << millis() << "logtimer: " << logTimer << endl;
    if(millis() > logTimer+1000) {   //capture a new data point "about" once per second
      cout << "distance to home from stats func: " << globalFix.location.DistanceKm( home ) << endl;
      if( (homeStatRadius <= 0) || globalFix.location.DistanceKm( home ) > homeStatRadius ) {  
        cout << "logging stats\n";
        if(!tripStarted)
        {
          uint8_t startDay;  //I don't really need this so I'm just making a place holder that will go away.
          getLocalTime(&startHours, &startDay);
          startMinutes = globalFix.dateTime.minutes;  //figure out what time we started sailing
          tripStarted = true;
        }
        if(globalFix.valid.speed) {
          _SOG_ = globalFix.speed() * 100;   //get GPS speeed
          statAry[i_log].boatSpeed = _SOG_;
        }

        //Get boat heading for TWS calculations
        //use gps speed if traveling over 2 knot otherwise use the compass speed
        if(_SOG_ > 200 && globalFix.valid.heading)
          _COG_ = globalFix.heading();
        else {
          static uint8_t system, gyro, accel, mag;
          system = gyro = accel = mag = 0;

          //only use compass speed if IMU has a quality fix
          bno.getCalibration(&system, &gyro, &accel, &mag);
          if(mag > 0) {   //if the compass is within calibaration
            //variance added to compass heading becasue its magnetic referenced and we want wind true referenced.
            _COG_ = int(round(compEvent.orientation.x) + variance + 360) % 360;  //don't need to check for ==360 rounding errors here because the math works out the same
          }
          else {
            //not adding variance because GPS heading is true referenced
            if(globalFix.valid.heading)
            {
              _COG_ = globalFix.heading();  //GPS heading may be inaccurate at low speeds but its the best we've got if we get here
            }
          }
        }
        
        //Add values into the temp stat array
        if(i_log < elements)
        {
          cout << "logging into stat array\n";
          statAry[i_log].speed = getTWS(Peet.getDirection(), wndSpd, _SOG_);
          statAry[i_log].sinTWD = round(sin(degToRad(getTWD(Peet.getDirection(), wndSpd, _SOG_, _COG_)))*10000);  //10000 is to not need floats
          statAry[i_log].cosTWD = round(cos(degToRad(getTWD(Peet.getDirection(), wndSpd, _SOG_, _COG_)))*10000);
          
          i_log++;
        }

        //Once the temp array is full write data out to the SD card
        if(i_log == elements)
        {
          cout << "stat array full\n";
          //update trip end time
          uint8_t curDay;
          getLocalTime(&curHours, &curDay);
          curMinutes = globalFix.dateTime.minutes;
          
          //calculate average wind speed and direction components (direction components are needed to work out average wind direction for the trip)
          accumSpeed = accumSinTWD = accumCosTWD = accumBoatSpeed = 0;
          for(int i = 0; i < elements; i++) {
            accumSpeed += statAry[i].speed;
            accumSinTWD += statAry[i].sinTWD;
            accumCosTWD += statAry[i].cosTWD;
            accumBoatSpeed += statAry[i].boatSpeed;
          }
          accumSpeed /= elements;
          accumSinTWD /= elements;
          accumCosTWD /= elements;
          accumBoatSpeed /= elements;
        
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
        } //end of stats accumulation after temp array has become full
      } //end of home radius checking
        i_log = 0;
    } //end of once per second 
    logTimer = millis();
  }
    
  if(wndSpd > windMax) { windMax = wndSpd; }

  //handle radio traffic
  #ifdef LoRaRadioPresent 
    if (rf95.available())
    {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      uint8_t data[RH_RF95_MAX_MESSAGE_LEN];

      if (rf95.recv(buf, &len))
      {
        uint16_t spd;
        int16_t dir;
        uint8_t messageCount;

        if(stricmp((char*)buf,"McFly") == 0) {
          strcpy((char*)data, "HiBiff");
          #ifdef debug
            cout << "Got McFly?...  Sending \"HiBiff\"" << endl;
          #endif
        }
        else {
          #ifdef debug
            //cout << "RSSI: " << rf95.lastRssi() << " SNR: " << rf95.lastSNR() << endl;
          #endif
          strcpy((char*)data, "A");
          memcpy(&spd, &buf, 2);
          memcpy(&dir, &buf[2], 2);
          memcpy(&battVoltage, &buf[4], 2);
          memcpy(&messageCount, &buf[6], 1);
          #ifdef debug
            static uint8_t lastMessage;
            static uint16_t packetsLost = 0;
            static uint32_t packetsReceived = 0;
            if(messageCount != uint8_t(lastMessage + 1)) {
              ++packetsLost;
              cout << "Packet lost!!!!!!" << endl << endl;
              cout << "Packet Loss: " << double(packetsLost) / double(packetsReceived) * 100.0 << "%" << endl;
              lastMessage = messageCount;
            }
            ++packetsReceived;
            lastMessage = messageCount;
            cout << spd << " " << dir << " " << battVoltage << " "; Serial.println(messageCount, DEC);
          #endif
          Peet.processWirelessData(spd, dir);
        }
        rf95.send(data, sizeof(data));  //transmit response
        rf95.waitPacketSent();
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
float degToRad(float deg) { return (deg * PI / 180); }

float radToDeg(float rad) { return (rad * 180 / PI); }

float ctof(float c) { return c*1.8+32; }

float ftoc(float f) { return f-32*0.555556; }

//get True Wind Speed
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
uint16_t getTWA(uint16_t AWA, uint16_t AWS, uint16_t SOG)
{
  float _AWA = degToRad(AWA);
  float tanAlpha = sin(_AWA)/(float(AWS)/float(SOG)-cos(_AWA));
  float Alpha = atan(tanAlpha);
  float tdiff = round(radToDeg(_AWA+Alpha));
  if(AWA == 0) {
    if(AWS >= SOG)
      return(0);
    else
      return(180);
  }
  if(tdiff < 0)
    return(tdiff+180);
  else if(tdiff > 359)
    return(tdiff-180);
  else
    return(tdiff);
}

//get True Wind Direction
uint16_t getTWD(uint16_t AWA, uint16_t AWS, uint16_t SOG, uint16_t COG) { 
  return (COG + getTWA(AWA, AWS, SOG) + 360) % 360; 
}

extern "C" char *sbrk(int i);
 
int freeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}
///////////////////////////////////////////////Compass/Accelerometer Helper Functions////////////////////////////////////////////////////
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
void displayString(char s[4]) {
  for(uint8_t i=0; i < 4; i++ ) {
    if(s[i+1] == '.')
      alpha4.writeDigitAscii(i, s[i], 1);
    else
      alpha4.writeDigitAscii(i, s[i], 0);
    alpha4.writeDisplay();
  }
}

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

void displayIntFloat(int val, char lastChar = '\0')  //displays 2 digits of precision and expects the value as an int (float val * 100)
{
  if(lastChar == 'V' || lastChar == 'v')
  {
    val = val*10;
  }
  char s[5];
  sprintf(s,"%i%i%i%i",val/1000%10,val/100%10,val/10%10,val%10);
  if(lastChar == 'V' || lastChar == 'v')
    alpha4.writeDigitAscii(0,s[0], true);
  else
    alpha4.writeDigitAscii(0,s[0]);
  
  if(lastChar == 'V' || lastChar == 'v')
    alpha4.writeDigitAscii(1,s[1]);
  else
    alpha4.writeDigitAscii(1,s[1], true);

  alpha4.writeDigitAscii(2,s[2]);
  if(lastChar != '\0') alpha4.writeDigitAscii(3,lastChar);
  else alpha4.writeDigitAscii(3,s[3]);
  alpha4.writeDisplay();
}

void displayBaro()
{
  if(baroRefAlt == -1) {
    //first wait until new GPS data available. 
    //This should probably be done less often but meh what if someone wants it to update regularly....?
    if(globalFix.valid.altitude)
    {
      float alt = globalFix.altitude();

      //constants slightly different because GPS altitude is in meters
      displayIntFloat( round( baro.readPressure()*pow((1-(0.0065*alt)/(baro.readTemperature()+0.0065*alt+273.15)),-5.257)*0.0295301 ) );
    }
  }
  else
    displayIntFloat(round( baro.readPressure()*pow((1-(0.0019812*baroRefAlt)/(baro.readTemperature()+0.0019812*baroRefAlt+273.15)),-5.257)*0.0295301 ) );
}

void displayTemp(char units)
{
  if(units == 'c' || units == 'C')
    displayIntFloat(baro.readTemperature()*100, 'C');         //use baro pressure sensor for temp
    //displayIntFloat(bno.getTemp()*100, 'C');                //use IMU chip for temp
  else if(units == 'f' || units == 'F')
    displayIntFloat(ctof(baro.readTemperature())*100, 'F');   //use baro pressure sensor for temp
    //displayIntFloat(ctof(bno.getTemp())*100,'F');           //use IMU chip for temp
}
///////////////////////////////////////////////////////////Interrupt Handlers///////////////////////////////////////////////
#ifndef LoRaRadioPresent  //The anemometer interrupts are not needed if we don't have one connected
  void isrSpeed() {
    Peet.processSpeedTransition();
  }
  void isrDirection() {
    Peet.processDirTransition();
  }
  void TC5_Handler (void) {   //This timer gets moved to the mast head in a wireless version of this project
    Peet.slowTimer();
    TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
  }
#endif
void isrGesture () {
  gestureSensed = true;
  detachInterrupt(digitalPinToInterrupt(GESTURE_INT));  //trying this because its in the example but it doesn't seem to make any difference.
}
/////////////////////////////////////////////////////LED Ring Handling Functions////////////////////////////////////////////
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

void animateSolidColor(uint32_t c, uint8_t wait) {
  //This is a background setting animation.
  for(uint8_t i=0; i < strip.numPixels(); i++) {
    setBackgroundPixel(i, c);
    strip.show();
    delay(wait);
  }
} //animateSolidColor
void setBackgroundPixel (uint8_t pixel, uint32_t color) {
  //sets a pixel color in the strip and also updates the background buffer
  strip.setPixelColor(pixel, color);
  pixelBackground[pixel] = color;
} //setBackgroundPixel
void restoreBackground() {
  //This function will restore the pixel values to a blank background value
  for(uint8_t i=0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, pixelBackground[i]);
  }
} //restoreBackground

//This updates the LED ring with the lights corresponding to the passed angle
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
  cout << "got to FailBlink" << endl;
  displayString("FAIL");
  while(1) blip(RED_LED_PIN, 1, 75);
} //failBlink

///////////////////////////////////////////////////////SD Card Handling Functions////////////////////////////////////////////
static bool readConfig () {
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////sd.chdir("!CONFIG"); sd.vwd()->rmRfStar(); sd.chdir("/");      //using this will torch the entire config directory (IMU cal data too)
  sd.remove("/!CONFIG/BTH_WIND.CFG");                            //this will delete the main config file and restore it to defaults
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (!cfg.begin("/!CONFIG/BTH_WIND.CFG", 100)) {
    sd.mkdir("!CONFIG");
    #ifdef debug
      Serial.println(F("Failed to open config file"));
      Serial.println(F("Creating Default Config File"));
    #endif
    cfg.end();  //close the cfg instance so as to use a normal one
    
    if (!logfile.open("/!CONFIG/BTH_WIND.CFG", O_CREAT | O_WRITE | O_EXCL)) { //create new file
      #ifdef debug
        Serial.println(F("Couldn't open new config file"));
      #endif
      failBlink();
      return 0;
    }
    else {
      logfile.print(F("#############################################################################################\n"));
      logfile.print(F("# BTHWindInstrument v0.1 Configuration by Brett Howard\n"));
      logfile.print(F("# BowOffset: Used to correct for the difference between anemometer north and your Bow\n"));
      logfile.print(F("# MagVariance: Magnetic variance between true and magnetic north (East = Negative)\n"));
      logfile.print(F("# HeelAngle: The angle at which you want to switch to displaying a digital heel angle (0 disables)\n"));
      logfile.print(F("# MenuScrollSpeed: The number of mS to delay each character when scrolling menu item titles\n"));
      logfile.print(F("# TempUnits: c for Celcius f for Fahrenheit\n"));
      logfile.print(F("# SpeedMAD: Speed Moving Average Depth (this smooths and averages the wind speed data)\n"));
      logfile.print(F("# WindUpdateRate: Minimum delay between display updates for wind speed modes.\n"));
      logfile.print(F("# DirectionFilter: range (1-1000); lower = more filtering; 1000=no filtering\n"));
      logfile.print(F("#    Each wind direction delta is multiplied by DirectionFilter/1000\n"));
      logfile.print(F("# Timezone: Timezone needed to properly timestamp files and report sail start/stop times\n"));
      logfile.print(F("# GPSUpdateRate: Time in mS Between GPS fix updates\n"));
      logfile.print(F("# BaroRefAlt: Barometer reference Altitude (in feet) put in 0 to report \"station pressure\"\n"));
      logfile.print(F("#    Setting the BaroRefAlt to -1 will tell the unit to use the GPS altitude for the calulation\n"));
      logfile.print(F("# GPXLogging: If true the unit will log your tracks in GPX format to the SD card\n"));
      logfile.print(F("# HomeLat: Latitude of your slip in integer format\n"));
      logfile.print(F("# HomeLon: Longitude of your slip in integer format\n"));
      logfile.print(F("# HomeStatRadius: Distance you must go before the statistics collection activates (0 disables)\n"));
      logfile.print(F("# HomeRadius: Distance you must go before the GPS tracking activates (0 disables)\n"));
      logfile.print(F("# TrackName: A name to be associated into your GPX log files\n"));
      logfile.print(F("#############################################################################################\n\n"));

      logfile.print(F("BowOffset=337\n"));
      logfile.print(F("MagVariance=15\n"));     //might be possible to support -1 here and read mag var from RMC NMEA sentence
      logfile.print(F("HeelAngle=15\n"));
      logfile.print(F("MenuScrollSpeed=150\n"));
      logfile.print(F("TempUnits=f\n"));
      logfile.print(F("SpeedMAD=5\n"));          
      logfile.print(F("WindUpdateRate=1000\n"));   //500 repaints the display at a 2Hz rate
      logfile.print(F("DirectionFilter=250\n"));  //250 displays 1/4 of the actual delta on each update
      logfile.print(F("Timezone=-7\n"));
      logfile.print(F("GPSUpdateRate=1000\n"));
      logfile.print(F("BaroRefAlt=374\n"));         //374 feet is full pool elevation for Fern Ridge Reservoir, Eugene, OR
      logfile.print(F("GPXLogging=true\n"));
      logfile.print(F("HomeLat=441189070\n"));       //location of slip B32 at Richardson Park Marina
      logfile.print(F("HomeLon=-1233155660\n"));
      logfile.print(F("HomeStatRadius=350\n"));         //covers just about to the edge of the Eugene Yacht Club
      logfile.print(F("HomeGPSRadius=50\n"));           //set to be fairly small but big enough to thward false positives.
      logfile.print(F("TrackName=Uncomfortably Level\n"));  //Boat name
      logfile.close();
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
    if (cfg.nameIs("WindUpdateRate")) { windUpdateRate = cfg.getIntValue(); }
    if (cfg.nameIs("DirectionFilter")) { directionFilter = cfg.getIntValue(); }
    if (cfg.nameIs("Timezone")) { TimeZone = cfg.getIntValue(); }
    if (cfg.nameIs("GPSUpdateRate")) { delayBetweenFixes = cfg.getIntValue(); }
    if (cfg.nameIs("BaroRefAlt")) { baroRefAlt = cfg.getIntValue(); }
    if (cfg.nameIs("GPXLogging")) { GPXLogging = cfg.getBooleanValue(); }
    if (cfg.nameIs("HomeLat")) { homeLat = cfg.getIntValue(); }
    if (cfg.nameIs("HomeLon")) { homeLon = cfg.getIntValue(); }
    if (cfg.nameIs("HomeStatRadius")) { homeStatRadius = float(cfg.getIntValue()/1000.0); }
    if (cfg.nameIs("HomeGPSRadius")) { homeGPSRadius = float(cfg.getIntValue()/1000.0); }
    if (cfg.nameIs("TrackName")) { strcpy(trackName, cfg.copyValue()); }
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

//This is the callback function for the SdFat file system library so that it can properly timestamp files it modifies and creates
void dateTime(uint16_t* date, uint16_t* time) {
  byte localDay;
  uint8_t localHour;
  
  getLocalTime(&localHour, &localDay);
    
  *date = FAT_DATE(globalFix.dateTime.full_year(), globalFix.dateTime.month, localDay);
  *time = FAT_TIME(localHour, globalFix.dateTime.minutes, globalFix.dateTime.seconds);
}

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
//This returns the local hour and date based on the data from the GPS and the timeZone value set on the SD card config file.
void getLocalTime(uint8_t *localHour, byte *localDay)
{
  int localHourTemp;
  if(globalFix.valid.time && globalFix.valid.date) {    //don't do the work if we don't have valid data
    *localDay = globalFix.dateTime.date;
    localHourTemp = globalFix.dateTime.hours + TimeZone;
    
    if (localHourTemp > 23) { *localHour = localHourTemp - 24; *localDay += 1; }
    else if (localHourTemp < 0) { *localHour = localHourTemp + 24; *localDay -= 1; }
    else
      *localHour = localHourTemp;
  }
}

//this is an endless loop that fetches data from the GPS and returns when you have a valid location date and time.
static void waitForFix()
{
  #ifdef debug
    Serial.print( F("Waiting for GPS fix...") );
  #endif

  uint16_t lastToggle = millis();

  for (;;) {
    while (Serial1.available()) { gps.handle(Serial1.read()); }
    if (gps.available()) {
      globalFix = gps.read();
      if (globalFix.valid.location && globalFix.valid.date && globalFix.valid.time)
        break; // Got it!
    }

    // Slowly flash the LED until we get a fix
    if ((uint16_t) millis() - lastToggle > 1000) {
      lastToggle += 1000;
      scrollString( "WAITING FOR GPS FIX\0", 150 );
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
void startLogFile()
{ 
  uint8_t localHour;
  byte localDay;

  if(globalFix.valid.date && globalFix.valid.time)
  {
    char directory[9];
    
    getLocalTime(&localHour, &localDay); 
    
    //for some reason if these two sprintf's are swapped in order it zero's out localHour.  No idea why
    sprintf(filename, "/%02d-%02d-%02d/%02d-%02d%s", globalFix.dateTime.year, globalFix.dateTime.month, localDay, 
            localHour, globalFix.dateTime.minutes, ".GPX");
    sprintf(directory, "/%02d-%02d-%02d", globalFix.dateTime.year, globalFix.dateTime.month, localDay); 
    
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
static void WriteGPXLog()
{ 
  // Log the fix information if we have a location and time
  if (globalFix.valid.location && globalFix.valid.time) {
    char date1[22];
    snprintf(date1, 21, "%4d-%02d-%02dT%02d:%02d:%02dZ", globalFix.dateTime.full_year(), globalFix.dateTime.month, 
            globalFix.dateTime.date, globalFix.dateTime.hours, globalFix.dateTime.minutes, globalFix.dateTime.seconds);
    
    while(!gpsLog.isOpen()) gpsLog.open(filename, O_WRITE);

    while(!gpsLog.seekSet(gpsLog.fileSize() - 28));
    gpsLog.print(F("\t\t<trkpt lat=\""));

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
      gpsLog.print(globalFix.speed());
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

    //replace closing tags
    gpsLog.print(F("\t</trkseg>\r\n</trk>\r\n</gpx>\r\n"));

    while(!gpsLog.close());
    blip(RED_LED_PIN, 1, 20);
  }
}