#include <SdFat.h>
#include <SerialMessage.h>

File file;

char logFilename[256];
char logBuf[256];

int getLogFilename() {
    //Really hacky way of ensuring new filename
    unsigned long startTime = micros();
    int i = 1;
    do {
        snprintf(logFilename, 256, "flightLog%d.csv", i++);

        //5 second timeout
        if (micros() - startTime > 5000000) {
            return failMessage("Log filename timeout!");
        }
    } while (sdEx.exists(logFilename));

    snprintf(printMsgBuf, 256, "Created log file %s", logFilename);
    printMessage(printMsgBuf);
    return 0;
}

int loggingInit() {
    if (!sdExInitialized) {
        if (!sdEx.begin()) {
            return failMessage("SdFatSdioEX begin() failed");
        }
        sdEx.chvol();

        sdExInitialized = true;
    }

    return 0;
}

int startLog() {
    getLogFilename();

    //Open log file
    if (!file.open(logFilename, O_RDWR | O_CREAT)) {
        snprintf(errMsgBuf, 256, "Failed to open %s", logFilename);
        return failMessage(errMsgBuf);
    }
    file.truncate(0);

    printMessage("Started log");

    return 0;
}

int writeLog(const char *logMsg) {
    size_t nb = strlen(logMsg);
    if (nb == (size_t)file.write(logMsg, nb)) {
        return 0;
    } else {
        return failMessage("Log write failed");
    }
}

void stopLog() {
    file.close();

    printMessage("Stopped log");
}