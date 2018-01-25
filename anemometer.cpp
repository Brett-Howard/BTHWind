#include "anemometer.h"

Anemometer::Anemometer(uint8_t speedPin, uint8_t dirPin, uint8_t speedMAD, uint8_t directionFilter) {
	_speedPin = speedPin;
	_dirPin = dirPin;
	_movingAverageCount = 3;
	_speedMAD = speedMAD;
	_directionFilter = directionFilter;
	pinMode(_speedPin, INPUT);
	pinMode(_dirPin, INPUT);
}
Anemometer::~Anemometer() {
	speedList.clear();
}

uint16_t Anemometer::getSpeed() {
	uint32_t temp = 0;
	
    for(int i=0; i<speedList.size();i++) {
        temp += speedList[i];
    }
    newDataAvail = false;		//only new speed clears this value as its the more expensive operation.
	return(temp/speedList.size());
}

uint16_t Anemometer::getDirection() {
	return(_windDirection);
}

void Anemometer::addSpeedToList(uint16_t knots) {
	if( speedList.size() >= _speedMAD )
	{
		for(int i = speedList.size(); i >= _speedMAD; i--)
			speedList.pop_front();
	}
	speedList.push_back(knots);
}

void Anemometer::processNewData() {
	uint32_t dirPulse_, speedPulse_;
    uint32_t speedTime_;
    uint32_t directionTime_;
    int16_t windDirection = 0l, rps = 0l, knots = 0l;

    static int16_t prevKnots = 0;
    static int16_t prevDir = 0;
    int dev = 0;

    // Get snapshot of data into local variables. Note: an interrupt could trigger here disable them for safety
    noInterrupts();
    dirPulse_ = dirPulse;
    speedPulse_ = speedPulse;
    speedTime_ = speedTime;
    directionTime_ = directionTime;
    interrupts();					//TODO: benchmark things and if you can guarantee that thigns get done with high winds this step isn't needed

    // Make speed zero, if the pulse delay is too long (drop sails its time for a beer)
    if (micros() - speedPulse_ > TIMEOUT) {
        speedTime_ = 0ul;
		knots = 0;
	}

    // The following converts revolutions per 100 seconds (rps) to knots x 100
    // This calculation follows the Peet Bros. piecemeal calibration data
    if (speedTime_ > 0)
    {
        rps = 100000000 / speedTime_; //revolutions per 100s

        if (rps < 323)
        {
            knots = (rps * rps * -11) / 11507 + (293 * rps) / 115 - 12;
        }
        else if (rps < 5436)
        {
            knots = (rps * rps / 2) / 11507 + (220 * rps) / 115 + 96;
        }
        else
        {
            knots = (rps * rps * 11) / 11507 - (957 * rps) / 115 + 28664;
        }
		
		if(knots < 0) { knots = 0; }   //make sure knots is never negative
        
		dev = (int)knots - prevKnots;

        // Only update output if in deviation limit
        if (checkSpeedDev(knots, dev))
        {
            //insert into linked list
			addSpeedToList(knots);
			//Serial.print("Wind Speed: "); Serial.println(knots/100.0);

            if (directionTime_ > speedTime_)
            {
                windDirection = 999; // For debugging only
            }
            else
            {
                // Calculate direction from captured pulse times
                //windDirection = (((directionTime_ * 360) / speedTime_) + _bowOffset) % 360;
				windDirection = (((directionTime_ * 360) / speedTime_) + _bowOffset) % 360;
				
				// Find deviation from previous value
                dev = (int)windDirection - prevDir;

                // Check deviation is in range
                if (checkDirDev(knots, dev))
                {
					int delta = ((int)windDirection - _windDirection);
                    if (delta < -180)
                    {
                        delta = delta + 360; // Take the shortest path when filtering
                    }
                    else if (delta > +180)
                    {
                        delta = delta - 360;
                    }
                    // Perform filtering to smooth the direction output
                    ///////////////////Serial.print("dir fil: "); Serial.println(_directionFilter);
					_windDirection = (_windDirection + (int)(round(_directionFilter * delta))) % 360;
                    if (_windDirection < 0)
                        _windDirection += 360;
					//Serial.print("Wind Direction: "); Serial.println(_windDirection);
                }
                prevDir = windDirection;
            }
        }
        prevKnots = knots; // Update, even if outside deviation limit, cause it might be valid!?
    }
}

bool Anemometer::checkDirDev(long knots, int dev)
{
    if (knots < BAND_0)
    {
        if ((abs(dev) < DIR_DEV_LIMIT_0) || (abs(dev) > 360 - DIR_DEV_LIMIT_0))
            return true;
    }
    else if (knots < BAND_1)
    {
        if ((abs(dev) < DIR_DEV_LIMIT_1) || (abs(dev) > 360 - DIR_DEV_LIMIT_1))
            return true;
    }
    else
    {
        if ((abs(dev) < DIR_DEV_LIMIT_2) || (abs(dev) > 360 - DIR_DEV_LIMIT_2))
            return true;
    }
    return false;
}

bool Anemometer::checkSpeedDev(long knots, int dev)
{
    if (knots < BAND_0)
    {
        if (abs(dev) < SPEED_DEV_LIMIT_0)
            return true;
    }
    else if (knots < BAND_1)
    {
        if (abs(dev) < SPEED_DEV_LIMIT_1)
            return true;
    }
    else
    {
        if (abs(dev) < SPEED_DEV_LIMIT_2)
            return true;
    }
    return false;
}

void Anemometer::setBowOffset(uint16_t angle) {
	_bowOffset = angle;
}

uint16_t Anemometer::getBowOffset() {
	return _bowOffset;
}

void Anemometer::setSpeedMAD(uint8_t speedMAD) { _speedMAD = speedMAD; }
void Anemometer::setDirectionFilter(uint16_t directionFilter) {
	_directionFilter = float(directionFilter)/1000.0; 
}
uint8_t Anemometer::getSpeedMAD() { return _speedMAD; }
uint16_t Anemometer::getDirectionFilter() { return _directionFilter*1000; }
bool Anemometer::available() { return newDataAvail; }

void Anemometer::processSpeedTransition() {
	if (((micros() - speedPulse) > DEBOUNCE) && (digitalRead(_speedPin) == LOW))
    {
        // Work out time difference between last pulse and now
        speedTime = micros() - speedPulse;
        
        // Direction pulse should have occured after the last speed pulse
        if (dirPulse - speedPulse >= 0)
            directionTime = dirPulse - speedPulse;

        processNewData();
		newDataAvail = true;
        speedPulse = micros(); // Capture time of the new speed pulse
    }
}
void Anemometer::processDirTransition() {
	if (((micros() - dirPulse) > DEBOUNCE) && (digitalRead(_dirPin) == LOW))
    {
        dirPulse = micros(); // Capture time of direction pulse
	}
}

void Anemometer::processWirelessData(uint16_t speed, int16_t direction) {
    addSpeedToList(speed);
    //add in bow offset (because mast head doesn't know about it)
    //perform wind direction filter and apply bow offset
    int delta = (int(direction+_bowOffset) - _windDirection);
    if (delta < -180)
    {
        delta = delta + 360; // Take the shortest path when filtering
    }
    else if (delta > +180)
    {
        delta = delta - 360;
    }
    // Perform filtering to smooth the direction output
    _windDirection = (_windDirection + (int)(round(_directionFilter * delta))) % 360;
    if (_windDirection < 0)
        _windDirection += 360;
    
    newDataAvail = true;
}

void Anemometer::slowTimer() {
	if(micros() - speedPulse > TIMEOUT) {
		addSpeedToList(0);
		newDataAvail = true;
	}
}