#ifndef _RC_H_
#define _RC_H_

#include <TeensyFCTypes.h>

#define NUM_RC_CHANNELS 8

extern rcTypeVal rcType;

static const uint8_t rcDeadband = 2;

extern int rcFailCount;
extern bool rcDetected;
extern bool newRCCommand;
extern bool rcFailsafe;
extern uint16_t rawRCCommands[NUM_RC_CHANNELS];

void initRC();
void parseIBUS();
void checkRCConnection();

#endif