#ifndef Anemometer_h
#define Anemometer_h
#include <Arduino.h>
#include <stdint.h>
#include <QList.h>

class Anemometer
{
	public:
		Anemometer(uint8_t speedPin, uint8_t dirPin, uint8_t speedMAD = 1, uint8_t directionMAD = 1);
		~Anemometer();
		
		uint16_t getSpeed();
		uint16_t getDirection();  //returns relative + for starboard - for port
		
		void setBowOffset(uint16_t angle);
		uint16_t getBowOffset();
		void setSpeedMAD(uint8_t speedMAD);
		void setDirectionFilter(uint16_t directionFilter);
		uint8_t getSpeedMAD();
		uint16_t getDirectionFilter();
		bool available();
		
		//these functions need to be connected to interrupts externally
		void processSpeedTransition();
		void processDirTransition();
		void ProcessWirelessData(uint16_t speed, int16_t direction);
		//The slow timer interrupt is needed to allow wind speed to go to zero
		//The other interrupt handlers never get called if the anemometer truely stops.
		void slowTimer();  //1Hz works well
	
	private:
		void processNewData();
		bool checkDirDev(long knots, int dev);
		bool checkSpeedDev(long knots, int dev);
		void addSpeedToList(uint16_t knots);

		bool newDataAvail;
		
		const unsigned long DEBOUNCE = 60000ul;
		const unsigned long TIMEOUT = 1600000ul;
	
		uint8_t _speedPin;
		uint8_t _dirPin;
		uint16_t _bowOffset;
	
		uint8_t _movingAverageCount;

		unsigned long prevSpeedPulse;
		unsigned long speedPulseWidth;
		unsigned long dirPulseTime;
		unsigned long dirPulseWidth;
		volatile unsigned long speedPulse = 0ul;    // Time capture of speed pulse
		volatile unsigned long dirPulse = 0ul;      // Time capture of direction pulse
		volatile unsigned long speedTime = 0ul;     // Time between speed pulses (microseconds)
		volatile unsigned long directionTime = 0ul; // Time between direction pulses (microseconds)
		volatile boolean newData = false;           // New speed pulse received
		volatile unsigned long lastUpdate = 0ul;    // Time of last serial output

		// Knots is actually stored as (Knots * 100). Deviations below should match these units.
		const int BAND_0 = 10 * 100;
		const int BAND_1 = 80 * 100;

		const int SPEED_DEV_LIMIT_0 = 5 * 100;  // Deviation from last measurement to be valid. Band_0: 0 to 10 knots
		const int SPEED_DEV_LIMIT_1 = 10 * 100; // Deviation from last measurement to be valid. Band_1: 10 to 80 knots
		const int SPEED_DEV_LIMIT_2 = 30 * 100; // Deviation from last measurement to be valid. Band_2: 80+ knots

		const int DIR_DEV_LIMIT_0 = 25; // Deviation from last measurement to be valid. Band_0: 0 to 10 knots
		const int DIR_DEV_LIMIT_1 = 18; // Deviation from last measurement to be valid. Band_1: 10 to 80 knots
		const int DIR_DEV_LIMIT_2 = 10; // Deviation from last measurement to be valid. Band_2: 80+ knots
		
		QList<uint16_t> speedList;  //may consider using a CircularBuffer to not need to do the new allocation.
		uint8_t _speedMAD;
		int16_t _windDirection;
		float _directionFilter;
};

#endif