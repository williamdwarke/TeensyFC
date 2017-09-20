#ifndef _FlightControl_h
#define _FlightControl_h

#include <TeensyFCTypes.h>

#define SEC_PER_US          0.000001

extern bool calibrated, armed;

//Timing
extern unsigned long logStartTime;
extern unsigned long accTime, magTime, printTime, ratePIDTime, ahrsTime;
extern double rateDT, ahrsDT;
extern int loopCount, gyroCount, accCount, magCount;

extern VECTOR gyro, acc, mag;
extern VECTOR rateSetpoints, attSetpoints;
extern VECTOR maxRates, maxAtts;
extern VECTOR ratePIDOutputs;

extern bool newIMUData;
extern fcModeVal fcMode;

int initFlightControl();
void flightControlLoop();

#endif