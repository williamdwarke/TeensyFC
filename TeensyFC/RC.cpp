#include <Arduino.h>
#include <FlySkyIBus.h>

#include "Config.h"
#include "RC.h"

int rcFailCount = 0;
bool rcDetected = false;
bool newRCCommand = false;
bool rcFailsafe = false;

uint16_t rawRCCommands[NUM_RC_CHANNELS];
uint16_t previousRawRCCommands[NUM_RC_CHANNELS];

rcTypeVal rcType;

void initRC() {
    IBus.begin(IBUS_SERIAL);
}

void parseIBUS() {
    IBus.loop();

    //Always check for failsafe condition
    bool failedRCFrame = true;

    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        rawRCCommands[i] = IBus.readChannel(i);

        //Frame failed if all channels are exactly the same
        if (rawRCCommands[i] != previousRawRCCommands[i]) {
            failedRCFrame = false;
        }

        previousRawRCCommands[i] = rawRCCommands[i];
    }

    if (failedRCFrame) {
        rcFailCount++;

        //10 failed frames = failsafe
        if (rcFailCount >= 1000) {
            rcFailsafe = true;
        }
    } else {
        //Reset RC fail counter
        rcFailCount = 0;
        newRCCommand = true;
    }
}

void checkRCConnection() {
    //IBUS is all we need for now
    parseIBUS();
    if (newRCCommand) {
        rcType = IBUS;
        rcDetected = true;

        //Reset failsafe
        rcFailCount = 0;
        rcFailsafe = false;
    }
}