#include "FlightControl.h"
#include "Logging.h"
#include "RC.h"

//Todo: Log status messages to another file with timestamps

bool initialized = false;

void setup() {
    Serial.begin(115200);
    if (initFlightControl() == 0) {
        initialized = true;
    }
}

void loop() {
    if (!initialized) {
        Serial.println("Initialization failed! Looping...");
        delay(5000);
        return;
    }

    flightControlLoop();
}

//IBUS event
void serialEvent1() {
    parseIBUS();
}