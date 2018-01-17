#include <Adafruit_NeoPixel_ZeroDMA.h>  //uses triple the needed RAM but allows interrupts and what not to work as they should
#include "anemometer.h"                 //Peet Bro's Anemometer
#include <SPI.h>                        //SPI Support
#include <Wire.h>                       //I2C Support
#include <SdFat.h>                      //SD Card FAT file system
#include <SDConfigFile.h>               //Read config files
#include <LSM303.h>                     //Polulu library for LSM303 Compass/Accelerometer (which actually works)
#include <Adafruit_BNO055.h>            //Library for a 9DOF IMU that should have a magnetometer with lower angle noise
#include <Adafruit_BMP280.h>            //support for barometric pressure sensor
#include <Adafruit_GFX.h>               //Font support for 14-seg LEDs
#include <Adafruit_LEDBackpack.h>       //Support for 14-seg LEDs
#include <SparkFun_APDS9960.h>          //Sparkfun APDS9960 Ambient Light/Gesutre Sensor library (which actually works)
#include <NMEAGPS.h>                    //GPS Support
#include <GPSport.h>                    //GPS Support
#include <RH_RF95.h>                    //LoRa Radio support
#include <RHReliableDatagram.h>         //Retransmission and ACK/NAK protocol

//I2C Address Information (just to make sure there are no collisions)
//BNO055 - 28h or 29h
//LSM303 - 19h or 1Eh
//APDS9960 - 39h
//BMP280 - 77h

#define debug             //comment this out to not depend on USB uart.
//#define noisyDebug        //For those days when you need more information (this also requires debug to be on)
#define LoRaRadioPresent  //comment this line out to start using the unit with a wireless wind transducer

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

#ifdef RGBW  //actually WRGB
  #define OFF           0x00000000
  #define BLACK         0x00000000
  #define RED           0x00FF0000
  #define DIM_RED       0x003F0000
  #define GREEN         0x0000FF00
  #define DIM_GREEN     0x00003F00
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

#define chipSelect 4

#define RFM95_CS 9
#define RFM95_RST 10
#define RFM95_INT 12
#define RF95_FREQ 915.0
#define RH_RF95_MAX_MESSAGE_LEN 32
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

ArduinoOutStream cout(Serial);

Anemometer Peet(ANEMOMETER_SPEED_PIN, ANEMOMETER_DIR_PIN, 8, 125);
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
Adafruit_BMP280 baro;
LSM303 compass;
Adafruit_BNO055 comp = Adafruit_BNO055(55);
SparkFun_APDS9960 apds = SparkFun_APDS9960();
#ifdef LoRaRadioPresent
  RH_RF95 rf95(RFM95_CS, RFM95_INT);
  RHReliableDatagram manager(rf95, SERVER_ADDRESS);
#endif

static NMEAGPS  gps;
static gps_fix globalFix;

enum Mode { AppWind, WindStats, TrueWind, CompHead, COG, SOG, Baro, Temp, Heel }; 

int16_t bowOffset, declination;
uint8_t speedMAD; 
uint16_t directionFilter;
uint8_t heelAngle;
uint16_t menuDelay;
char tempUnits;
int8_t TimeZone;
uint16_t delayBetweenFixes;
int16_t baroRefAlt;

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

  //setup radio pins
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 

  strip.begin();
  strip.clear();
  strip.show(); // Initialize all pixels to 'off'

  alpha4.begin(0x70);  // pass in the address
  scrollString("BTHWind", sizeof("BTHWind"), 300);
  
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
//////////////////////////////////////Startup Magnetometer and Accelerometer//////////////////////////////////////////
  if(!compass.init()) {
    #ifdef debug
      Serial.println("Compass failed to initialize");
    #endif
  }
  else {
    #ifdef debug
      Serial.println("Compass initialized successfully");
    #endif
    compass.enableDefault();
    compass.writeMagReg(0x01, 0xE0);  //sets magnetic gain to minimum (in an effort to reduce compass angle noise)
    
    //Feed compass calibration data use sample sketch to get these values
    //compass.m_min = (LSM303::vector<int16_t>){-408, -633, -784};
    //compass.m_max = (LSM303::vector<int16_t>){+683, +496, +323};

    //These are the values I got when using minimum gain settings
    compass.m_min = (LSM303::vector<int16_t>){-80, -138, -167};
    compass.m_max = (LSM303::vector<int16_t>){+129, +102, +55};
  }

  /////////Begin of new Bosch 9DOF sensor that hopefully has less compass angle noise. 
  if(!comp.begin())
  {
    #ifdef debug
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    #endif
 //   failBlink();
  }

  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  //read in calibration from compCal file.

  while(system < 3 || gyro < 3  || accel < 3 || mag < 3)
    comp.getCalibration(&system, &gyro, &accel, &mag);

  
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
  if ( apds.enableLightSensor(false) ) {  //no idea why this says false I'm just following the example and it works.
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
  apds.setGestureGain(GGAIN_1X);
/////////////////////////////////////Setup GPS///////////////////////////////////////////////////////////////////////////

  if (gps.merging != NMEAGPS::EXPLICIT_MERGING)
    Serial.println( F("Warning: EXPLICIT_MERGING should be enabled for best results!") );

  gpsPort.begin( 9600 );

  gps.send_P( &gpsPort, F("PGCMD,33,0") );  //turn off antenna nuisance data
  if(baroRefAlt == -1)
    gps.send_P( &gpsPort, F("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0") ); // 1 RMC & GGA (because GGA contains Altitude)
  else
    gps.send_P( &gpsPort, F("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0") ); // 1 RMC only to reduce serial traffic and speed up the polling loop
  char tmp[13];
  sprintf(tmp, "PMTK220,%d\0", delayBetweenFixes);
  gps.send( &gpsPort, tmp ); //set fix update rate
  SdFile::dateTimeCallback(dateTime);  //register date time callback for file system

//////////////////////////////////////////////////////Setup LoRa Radio//////////////////////////////////////////////////////
  #ifdef LoRaRadioPresent
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    if(!manager.init())
    {
      #ifdef debug
        Serial.println(F("Lora Radio Initialization Failed"));
      #endif
      failBlink();
    }
    #ifdef debug
      Serial.println(F("LoRa radio initialized successfully"));
    #endif

    if (!rf95.setFrequency(RF95_FREQ)) {
      #ifdef debug
        Serial.println(F("setFrequency failed"));
      #endif
      failBlink();
    }
    #ifdef debug
      Serial.print(F("Set Freq to: ")); Serial.print(RF95_FREQ); Serial.println(F(" Mhz"));
    #endif

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(10, false);

    //Bw125Cr45Sf128 - Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range.
    //Bw500Cr45Sf128 - Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range.
    //Bw31_25Cr48Sf512 - Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range.
    //Bw125Cr48Sf4096 - Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range.
    rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf128);

    // You can optionally require this module to wait until Channel Activity
    // Detection shows no activity on the channel before transmitting by setting
    // the CAD timeout to non-zero:
    rf95.setCADTimeout(10000);

  #endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bowOffset = 0;
  if(initSD()) {
    Serial.print(F("Card size: ")); Serial.print(sd.card()->cardSize() * 0.000512 + 0.5); Serial.println(" MiB");
    Serial.print(F("Card free: ")); Serial.print(sd.vol()->freeClusterCount() * .000512 * sd.vol()->blocksPerCluster()); Serial.println(" MiB");
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

  sd.remove("WINDSTAT.LOG");  //delete prior log file
 
//////////////Setup and configure the anemometer object and link up the necessary interrupts
  Peet.setBowOffset(bowOffset);
  Peet.setSpeedMAD(speedMAD);
  Peet.setDirectionFilter(directionFilter);
  
  //#ifndef LoRaRadioPresent
    attachInterrupt(digitalPinToInterrupt(ANEMOMETER_SPEED_PIN), isrSpeed, FALLING);
    attachInterrupt(digitalPinToInterrupt(ANEMOMETER_DIR_PIN), isrDirection, FALLING);
    tcConfigure(1000);  //1 second timer 
  //#endif
  #ifdef LoRaRadioPresent
    attachInterrupt(digitalPinToInterrupt(RFM95_INT), isrRadio, FALLING);
  #endif
  
  
  uint16_t w;
  apds.readAmbientLight(w);
  strip.setBrightness(map(w,0,37889,5,255));
  animateRedGreenWipe(60);  //pretty startup animation  
}

void displayIntFloat(int val, char lastChar);

void loop() {
  static Mode curMode = AppWind;
  static Mode prevMode;
  uint8_t gesture; 
  static uint16_t w,wndSpd,windAvg,windMax = 0;
  static bool firstEntry = 1;
  static int curHeelAngle;
  static uint32_t tempTimer;
  static bool locked = false;
  int16_t curSpeed;
  static bool newSDData = false;

  //adjust wind ring brightness based on ambient light
  apds.readAmbientLight(w);
  strip.setBrightness(map(w,0,37889,5,255));
  strip.show();

  //Determine if we're over heeling
  curHeelAngle = abs(getHeelAngle());
  if(curHeelAngle > heelAngle && heelAngle != 0) {
    prevMode = curMode;
    curMode = Heel;
  }

  //use DIR_NEAR and DIR_FAR gestures to lock the menu
  if (apds.isGestureAvailable()) gesture = apds.readGesture();
  if (gesture == DIR_NEAR || locked)
  {
    if(!locked) {
      scrollString("LOCKED", sizeof("LOCKED"), menuDelay);
      locked = 1;
    }
    if(locked && gesture == DIR_FAR) {
      scrollString("UNLOCKED", sizeof("UNLOCKED"), menuDelay);
      locked = 0;
    }
    else if(locked)
    {
      gesture = 0;  //make all gestures go away so they don't change menu status
    }
  }

  switch(curMode)
  {
    case AppWind:
        if(firstEntry) {
          scrollString("APPARENT WIND", sizeof("APPARENT WIND"), menuDelay);
          firstEntry = false;
        }
        ///////do AppWind
        if(Peet.available()) { wndSpd = Peet.getSpeed(); }
        if(wndSpd > 0) { displayIntFloat(wndSpd,'\0'); }
        else { displayString("BEER"); }
        #ifdef noisyDebug
          cout << "AWS: "  << Peet.getSpeed() << " AWA: " << Peet.getDirection() << endl;
        #endif
        //////Transition state
        if(gesture == DIR_LEFT) { curMode = Baro; firstEntry = true; }
        else if(gesture == DIR_RIGHT) { curMode = CompHead; firstEntry = true; }
        else if(gesture == DIR_UP) { curMode = TrueWind; firstEntry = true; }
        else if(gesture == DIR_DOWN) { curMode = WindStats; firstEntry = true; }
        break;
    
    
    case WindStats:
        if(firstEntry || newSDData) {
          scrollString("WIND STATS", sizeof("WIND STATS"), menuDelay);
          firstEntry = false;
          newSDData = false;
          
          displayString("WAIT");
          windStats.close();  //close the file that has been being logged to
          //Reading the file in
          //only read the file in on first entry to the menu entry
          if(windStats.open("WINDSTAT.LOG", O_READ))
          {
            uint32_t speedAccum = 0;
            uint16_t i, j, count = 0;
            
            while (windStats.available()) {
              csvReadUint16(&windStats, &i, ',');
              speedAccum += i;
              csvReadUint16(&windStats, &j, ',');  //read the wind direction off and throw it away
              count++;
            }
            windAvg=speedAccum/count;
          }
          windStats.close();
        }
        /////////////do WindStats
          
        //show the values on the display
        if(millis() > tempTimer && millis() < tempTimer+1000) {   //this timing method is really annoying (should change the gestures to interrupt driven)
          displayString("AVG ");
        }
        if(millis() > tempTimer+1000 && millis() < tempTimer+2000) {
          displayIntFloat(windAvg, '\0');
        }
        if(millis() > tempTimer+2000 && millis() < tempTimer+3000) {
          displayString("MAX ");
        }
        if(millis() > tempTimer+3000 && millis() < tempTimer+4000) {
          displayIntFloat(windMax, '\0');
        }
        if(millis() > tempTimer+4000)
          tempTimer = millis();

        ////////////Transition State
        if(gesture == DIR_LEFT) { curMode = Baro; firstEntry = true; }
        else if(gesture == DIR_RIGHT) { curMode = CompHead; firstEntry = true; }
        else if(gesture == DIR_UP) { curMode = AppWind; firstEntry = true; }
        else if(gesture == DIR_DOWN) { curMode = TrueWind; firstEntry = true; }
        break;
    
    
    case TrueWind:
        if(firstEntry) {
          scrollString("TRUE WIND", sizeof("TRUE WIND"), menuDelay);
          firstEntry = false;
        }
        //////////////do TrueWind
        uint16_t _SOG;
        uint16_t _AWS;
        uint16_t _AWA;

        if(gps.available())
          globalFix = gps.read();
        if (globalFix.valid.speed) {
          _SOG = globalFix.speed()*100;
        }
        if(Peet.available()) { 
          _AWS = Peet.getSpeed();
          _AWA = Peet.getDirection();
        }
        
        _SOG = 700;
        //_AWA = 88;
        //_AWS = 284;
        #ifdef noisyDebug
          cout << "AWA: "  << _AWA << " AWS: " << _AWS << " SOG: " << _SOG << endl;
        #endif
        displayIntFloat(getTWS(_AWA, _AWS, _SOG), '\0');
        #ifdef noisyDebug
          cout << getTWS(_AWA, _AWS, _SOG) << endl;
        #endif
        displayWindPixel(getTWA(_AWA, _AWS, _SOG), WHITE);
        #ifdef noisyDebug
          cout << getTWA(_AWA, _AWS, _SOG) << endl;
        #endif
        //////////////Transition State
        if(gesture == DIR_LEFT) { curMode = Baro; firstEntry = true; }
        else if(gesture == DIR_RIGHT) { curMode = CompHead; firstEntry = true; }
        else if(gesture == DIR_UP) { curMode = WindStats; firstEntry = true; }
        else if(gesture == DIR_DOWN) { curMode = AppWind; firstEntry = true; }
        break;
    
    
    case CompHead:
        if(firstEntry) {
          scrollString("COMPASS HEADING", sizeof("COMPASS HEADING"), menuDelay);
          firstEntry = false;
        }
        /////////////do CompHead
        sensors_event_t event;
        comp.getEvent(&event);
        
        //update compass once per second in a non blocking way so as to make gesture reaction faster.
        if(millis() > tempTimer+1000) { 
          //uint16_t heading = round(event.orientation.x);
          uint16_t heading = int(compass.heading((LSM303::vector<int>){1, 0, 0}));
          heading += declination;
          heading = (heading + 360) % 360;
          displayAngle(heading); 
          tempTimer = millis();
        }
        ///////////Transition State
        if(gesture == DIR_LEFT) { curMode = AppWind; firstEntry = true; }
        else if(gesture == DIR_RIGHT) { curMode = SOG; firstEntry = true; }
        else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = COG; firstEntry = true; }
        break; 
    
    
    case COG:
        if(firstEntry) {
          scrollString("GPS COG", sizeof("GPS COG"), menuDelay);
          firstEntry = false;
        }
        //////////////////do COG
        if(gps.available())
          globalFix = gps.read();
        if (globalFix.valid.speed) {
          displayAngle(uint16_t(globalFix.heading()));
        }
        //////////////////Transition State
        if(gesture == DIR_LEFT) { curMode = AppWind; firstEntry = true; }
        else if(gesture == DIR_RIGHT) { curMode = SOG; firstEntry = true; }
        else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = CompHead; firstEntry = true; }
        break; 
    
    
    case SOG:
        if(firstEntry) {
          scrollString("GPS SOG", sizeof("GPS SOG"), menuDelay);
          firstEntry = false;
        }
        //////////////////do SOG
        if(gps.available())
          globalFix = gps.read();
        if (globalFix.valid.speed) {
          displayIntFloat(globalFix.speed()*100, '\0');
        }

        //////////////////Transition State
        if(gesture == DIR_LEFT) { curMode = CompHead; firstEntry = true; }
        else if(gesture == DIR_RIGHT) { curMode = Temp; firstEntry = true; }
        else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = SOG; firstEntry = true; }
        break;
    
    
    case Baro:
        if(firstEntry) {
          scrollString("BARO", sizeof("BARO"), menuDelay);
          firstEntry = false;
        }
        ////////////////Do Baro
        if(millis() > tempTimer+1000) { 
          displayBaro();
          tempTimer = millis(); 
        } 
        ////////////////Transition State
        if(gesture == DIR_LEFT) { curMode = Temp; firstEntry = true; }
        else if(gesture == DIR_RIGHT) { curMode = AppWind; firstEntry = true; }
        else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = Baro; firstEntry = true; }
        break;
    
    
    case Temp:
        if(firstEntry) {
          scrollString("TEMP", sizeof("TEMP"), menuDelay);
          firstEntry = false;
        }
        ///////////////Do Temp
        //update temp only once per second but in a non blocking way that doesn't slow down gesture response.
        if(millis() > tempTimer+1000) { 
          displayTemp(tempUnits); 
          tempTimer = millis(); 
        } 
        ///////////////Transition State
        if(gesture == DIR_LEFT) { curMode = SOG; firstEntry = true; }
        else if(gesture == DIR_RIGHT) { curMode = Baro; firstEntry = true; }
        else if(gesture == DIR_UP || gesture == DIR_DOWN) { curMode = Temp; firstEntry = true; }
        break;
    
    
    case Heel:
        scrollString("Reduce Heel", sizeof("Reduce Heel"), menuDelay);
        displayAngle(curHeelAngle);
        delay(500);
        firstEntry = true;  //set so that upon returning you display menu heading
        curMode = prevMode; //return to previous mode before heel warning
        break;
  }

  if(Peet.available()) { 
    curSpeed = Peet.getSpeed();   //need to fetch a new speed here just in case we're not in Apparent Wind mode.
  }

  if(curSpeed > 0 && curMode != TrueWind)
      displayWindPixel(Peet.getDirection(), WHITE);
  else if(curSpeed == 0)
    restoreBackground();

  static uint16_t i_log = 0;
  static uint16_t speedBuf[60];      //size of this buffer is about how often the log file is updated (in seconds)
  uint32_t accum = 0;
  static uint32_t logTimer = 0;
  uint16_t elements = sizeof(speedBuf)/sizeof(speedBuf[0]);
  
  //accumulate a pile of wind speeds then average them every so often and write that value out to the SD card
  if(millis() > logTimer+1000) {
    if(i_log < elements)
    {
      speedBuf[i_log] = curSpeed;
      i_log++;
    }
    if(i_log == elements)
    {
      accum = 0;
      for(int i = 0; i < elements; i++) {
        accum += speedBuf[i];
      }
      accum /= elements;
      if(windStats.open("WINDSTAT.LOG", O_WRITE | O_CREAT | O_APPEND)  || windStats.isOpen()) {
          windStats.print(accum); windStats.print(','); windStats.println(Peet.getDirection());
          blip(GREEN_LED_PIN,3,20);
          newSDData = true;
          #ifdef debug
            cout << F("SD card average wind log entry written. Timestamp: ") << millis() << F(" Value: ") << accum << endl;
          #endif
      }
      i_log = 0;
    }
    logTimer = millis();
  }
  
  if(curSpeed > windMax) { windMax = curSpeed; }
  
  //Fetch all data from the GPS UART and feed it to the NeoGPS object
  //I tried to put this into a SerialEvent function but that seems to not work for me so I'll just leave this here.
  while (Serial1.available()) { gps.handle(Serial1.read()); }

  //check for radio messages
  #ifdef LoRaRadioPresent
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf), from;
    uint8_t data[] = "ACK";

    if( manager.available() )
    {
      //wait for a message addressed to us from the mast head
      blip(RED_LED_PIN, 1, 20);
      if (manager.recvfromAck(buf, &len, &from))
      {
        Serial.print("got request from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);
        Serial.print("RSSI: "); Serial.println(rf95.lastRssi(),DEC);
        Serial.print("SNR: "); Serial.println(rf95.lastSNR(),DEC);
        // Send a reply back to the originator client
        if (!manager.sendtoWait(data, sizeof(data), from))
          Serial.println("sendtoWait failed");
        else
          Serial.println("Ack Sent");
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

uint16_t getTWS(uint16_t AWA, uint16_t AWS, int16_t SOG)
{
  //cout << "AWA: " << AWA << " AWS: " << AWS << " SOG: " << SOG << endl;
  float _AWA = degToRad(AWA);
  //cout << "degToRad AWA: " << _AWA << endl;
  float tanAlpha = sin(_AWA)/(float(AWS)/float(SOG)-cos(_AWA));
  //cout << "sinAWA: " << sin(_AWA) << endl;
  //cout << "cosAWA: " << cos(_AWA) << endl;
  //cout << "AWS/SOG: " << float(AWS)/float(SOG) << endl;
  //cout << "TanAlpha: " << tanAlpha << endl;
  float Alpha = atan(tanAlpha);
  //cout << "Alpha: " << Alpha << endl;
  if(AWA == 0) {
    //cout << "retVal: " << abs(round(AWS-SOG)) << endl;
    return(abs(round(AWS-SOG)));
  }
  else {
    //cout << "retVal: " << round(SOG*(sin(_AWA)/sin(Alpha))) << endl;
    return(abs(round(SOG*(sin(_AWA)/sin(Alpha)))));
  }
}

uint16_t getTWA(uint16_t AWA, uint16_t AWS, int16_t SOG)
{
  //cout << "AWA: " << AWA << " AWS: " << AWS << " SOG: " << SOG << endl;
  float _AWA = degToRad(AWA);
  //cout << "degToRad AWA: " << _AWA << endl;
  float tanAlpha = sin(_AWA)/(float(AWS)/float(SOG)-cos(_AWA));
  //cout << "TanAlpha: " << tanAlpha << endl;
  float Alpha = atan(tanAlpha);
  //cout << "Alpha: " << Alpha << endl;
  float tdiff = round(radToDeg(_AWA+Alpha));
  //cout << "Tdiff: " << tdiff << endl;
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

extern "C" char *sbrk(int i);
 
int freeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}
///////////////////////////////////////////////Compass/Accelerometer Helper Functions///////////////////////////////////////////////
int16_t getCompassHeading() {      //this is bogus crap 
  compass.read();
  float heading = atan2(compass.m.y,compass.m.x);
  if (heading < 0) heading += 2*PI;
  return int(radToDeg(heading));
}

int16_t getAngleCompensatedCompassHeading() {    //this is boguser crap
  compass.read();
  float xh, yh, pitch, roll, heading;
  pitch = atan2(compass.a.x, compass.a.z);
  roll = -1*(atan2(compass.a.y, compass.a.z));
  xh = (compass.m.x * cos(pitch)) + (-1 * compass.m.z * sin(pitch));
  yh = (compass.m.x * sin(roll) * sin(pitch)) + (compass.m.y * cos(roll)) - (-1 * compass.m.z * sin(roll) * cos(pitch));
  heading = atan2(yh,xh);
  if(heading < 0) heading += 2*PI;
  return int(radToDeg(heading));
}

int32_t getHeelAngle() {     //LSM303 version
  //negative is heel to port
  compass.read();
  return(radToDeg(atan2(compass.a.y,compass.a.z))); 
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

void displayAngle(uint16_t val)
{
  char s[4];
  sprintf(s,"%i%i%i",val/100%10,val/10%10,val%10);
  alpha4.writeDigitAscii(0,s[0]);
  alpha4.writeDigitAscii(1,s[1]);
  alpha4.writeDigitAscii(2,s[2]);
  alpha4.writeDigitRaw(3,0b000000011100011);
  alpha4.writeDisplay();
}

void scrollString(char *s, uint8_t size, uint16_t speed)
{
  alpha4.clear();
  alpha4.writeDisplay();
  for(int i = 0; i < size-1; i++)
  {
    alpha4.writeDigitAscii(3, s[i]);
    if(i>0) alpha4.writeDigitAscii(2, s[i-1]);
    if(i>1) alpha4.writeDigitAscii(1, s[i-2]);
    if(i>2) alpha4.writeDigitAscii(0, s[i-3]);
    alpha4.writeDisplay();
    delay(speed);
  }
}

void displayIntFloat(int val, char lastChar = '\0')  //displays 2 digits of precision and expects the value as an int (float val * 100)
{
  char s[5];
  sprintf(s,"%i%i%i%i",val/1000%10,val/100%10,val/10%10,val%10);
  alpha4.writeDigitAscii(0,s[0]);
  alpha4.writeDigitAscii(1,s[1], true);
  alpha4.writeDigitAscii(2,s[2]);
  if(lastChar != '\0') alpha4.writeDigitAscii(3,lastChar);
  else alpha4.writeDigitAscii(3,s[3]);
  alpha4.writeDisplay();
}

void displayBaro()
{
  if(baroRefAlt == 0) 
    displayIntFloat(round(baro.readPressure()*0.0295301));
  else if(baroRefAlt == -1) {
    //first wait until new GPS data available. 
    //This should probably be done less often but meh what if someone wants it to update regularly....?
    if(gps.available());
    {
      globalFix = gps.read();
      float alt = globalFix.altitude();
      
      //constants slightly different because GPS altitude is in meters
      displayIntFloat( round( baro.readPressure()*pow((1-(0.0065*alt)/(baro.readTemperature()+0.0065*alt+273.15)),-5.257)*0.0295301 ) );
    }
  }
  else
    displayIntFloat( round( baro.readPressure()*pow((1-(0.0019812*baroRefAlt)/(baro.readTemperature()+0.0019812*baroRefAlt+273.15)),-5.257)*0.0295301 ) );
}

void displayTemp(char units)
{
  if(units == 'c' || units == 'C')
    displayIntFloat(baro.readTemperature()*100, 'C');
  else if(units == 'f' || units == 'F')
    displayIntFloat(ctof(baro.readTemperature())*100, 'F');
}
///////////////////////////////////////////////////////////Interrupt Handlers///////////////////////////////////////////////
//#ifndef LoRaRadioPresent
  void isrSpeed() {
    Peet.processSpeedTransition();
  }
  void isrDirection() {
    Peet.processDirTransition();
  }
  void TC5_Handler (void) {
    Peet.slowTimer();
    TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
  }
//#endif
#ifdef LoRaRadioPresent
  void isrRadio()
  {
    uint16_t speed;
    int16_t direction;
    uint8_t message[RH_RF95_MAX_MESSAGE_LEN];


    //get message from mast head

    memcpy(&speed, &message[0], 2);
    memcpy(&direction, &message[2], 2);    //using memcpy rather than array assignment because message may not be memory aligned to 16bit boundaries. 
    
    Peet.ProcessWirelessData(speed, direction);
  }
#endif

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

static void blip(int ledPin, int times, int dur) {
  //Toggles a pin in the direction opposite that it currently sits a number of times with a delay of dur between them.
  //The reason for "opposite direction" is tha this allows one to "blink" a light that is on steady as well.
  //This function assumes that pin is already configured as an output.
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(dur);
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(dur);
  }
} //blip

static void failBlink() {
  while (true) {
    digitalWrite(RED_LED_PIN, HIGH);
    delay(75);
    digitalWrite(RED_LED_PIN, LOW);
    delay(75);
  }
} //failBlink

///////////////////////////////////////////////////////SD Card Handling Functions////////////////////////////////////////////
static bool readConfig () {
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //sd.chdir("!CONFIG"); sd.vwd()->rmRfStar(); sd.chdir("/");      //using this will torch the entire config directory (IMU cal data too)
  //sd.remove("/!CONFIG/BTH_WIND.CFG");                            //this will delete the main config file and restore it to defaults
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
      return 0;
    }
    else {
      logfile.print(F("#############################################################################################\n"));
      logfile.print(F("# BTHWindInstrument v0.1 Configuration by Brett Howard\n"));
      logfile.print(F("# BowOffset: Used to correct for the difference between anemometer North and your Bow\n"));
      logfile.print(F("# MagVariance: Set to zero to give magnetic readings or adjusted if you want true.\n"));
      logfile.print(F("# HeelAngle: The angle at which you want to switch to displaying a digital heel angle (0 disables)\n"));
      logfile.print(F("# MenuScrollSpeed: The number of mS to delay each character when scrolling menu item titles\n"));
      logfile.print(F("# TempUnits: c for Celcius f for Fahrenheit\n"));
      logfile.print(F("# SpeedMAD: Speed Moving Average Depth\n"));
      logfile.print(F("# DirectionFilter: range(.001 to 1)*1000; lower = more filtering; 1000=no filtering\n"));
      logfile.print(F("# Timezone: Timezone needed to properly timestamp files\n"));
      logfile.print(F("# GPSUpdateRate: Time in mS Between GPS fix updates\n"));
      logfile.print(F("# BaroRefAlt: Barometer reference Altitude (in feet) put in 0 to report \"station pressure\"\n"));
      logfile.print(F("#    Setting the BaroRefAlt to -1 will tell the unit to use the GPS altitude for the calulation\n"));
      logfile.print(F("#############################################################################################\n\n"));

      logfile.print(F("BowOffset=0\n"));
      logfile.print(F("MagVariance=15\n"));
      logfile.print(F("HeelAngle=12\n"));
      logfile.print(F("MenuScrollSpeed=150\n"));
      logfile.print(F("TempUnits=f\n"));
      logfile.print(F("SpeedMAD=1\n"));
      logfile.print(F("DirectionFilter=1000\n"));
      logfile.print(F("Timezone=-8\n"));
      logfile.print(F("GPSUpdateRate=1000\n"));
      logfile.print(F("BaroRefAlt=374"));         //374 feet is full pool elevation for Fern Ridge Reservoir, Eugene, OR
      logfile.close();
      blip(GREEN_LED_PIN, 5, 200);
    }
    if(!cfg.begin("/!CONFIG/BTH_WIND.CFG", 100)) { Serial.println(F("Failed to open the newly created config")); }  //open the new file to read in the vals
  }

  //Fetch Configuration Values
  while (cfg.readNextSetting())
  {
    if (cfg.nameIs("BowOffset")) { bowOffset = cfg.getIntValue(); }
    if (cfg.nameIs("MagVariance")) { declination = cfg.getIntValue(); }
    if (cfg.nameIs("HeelAngle")) { heelAngle = cfg.getIntValue(); }
    if (cfg.nameIs("TempUnits")) { strncpy(&tempUnits, cfg.copyValue(), 1); }
    if (cfg.nameIs("MenuScrollSpeed")) { menuDelay = cfg.getIntValue(); }
    if (cfg.nameIs("SpeedMAD")) { speedMAD = cfg.getIntValue(); }
    if (cfg.nameIs("DirectionFilter")) { directionFilter = cfg.getIntValue(); }
    if (cfg.nameIs("Timezone")) { TimeZone = cfg.getIntValue(); }
    if (cfg.nameIs("GPSUpdateRate")) { delayBetweenFixes = cfg.getIntValue(); }
    if (cfg.nameIs("BaroRefAlt")) { baroRefAlt = cfg.getIntValue(); }
  }
  cfg.end();  //clean up
  return true;
}  //readConfig


bool initSD() {
  bool retval;
  //while (!gps.available());  //wait for a fix
  //globalFix = gps.read();
  #ifdef debug
    Serial.println( F("Initializing SD card...") );
  #endif
  
  // see if the card is present and can be initialized:
  if (!sd.begin(chipSelect)) {
    #ifdef debug
      Serial.println( F("  SD card failed, or not present") );
    #endif
    failBlink();  //dies here (never returns)  
  }
  #ifdef debug
    Serial.println( F("SD card initialized.") );
    retval = true;
  #endif
  return retval;
}  //initSD

void dateTime(uint16_t* date, uint16_t* time) {
  byte localDay;
  int localHour;
  
  getLocalTime(localHour, localDay);
    
  *date = FAT_DATE(globalFix.dateTime.full_year(), globalFix.dateTime.month, localDay);
  *time = FAT_TIME(localHour, globalFix.dateTime.minutes, globalFix.dateTime.seconds);
  
}

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

//////////////////////////////////////////////////////Timer Counter Configuration///////////////////////////////////////////////////
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
void getLocalTime(int &localHour, byte &localDay)
{
  if(!gps.available()) {         //normally I'd wait to make sure there was a valid fix but I don't want to wait for GPS to use the gauge
      globalFix = gps.read();
  }

  if(globalFix.valid.time && globalFix.valid.date) {    //don't do the work if we don't have valid data
    localDay = globalFix.dateTime.date;
    localHour = globalFix.dateTime.hours + TimeZone;
    
    if (localHour > 23) { localHour -= 24; localDay += 1; }
    else if (localHour < 0) { localHour += 24; localDay -= 1; }
  }
}