#include <EEPROM.h>
#include <PID.h>
#include <SerialMessage.h>
#include <SPI.h>
#include <TeensyESC.h>
#include <TeensyINI.h>
#include <TeensyMPU9255.h>

#include "AHRS.h"
#include "Config.h"
#include "Logging.h"
#include "Motors.h"
#include "RC.h"

//Note: Teensy 3.6 has 4096 bytes of EEPROM

#include "FlightControl.h"

TeensyINI flightConfig;

RCCOMMAND rcCommands;
TeensyMPU9255 imu(IMU_SS_PIN, true, true, true);

PID xRatePID(0.9, 0.0005, 0.0035);
PID yRatePID(0.9, 0.0005, 0.0035);
PID zRatePID(1.0, 0.0005, 0);

PID xAttPID(8.0, 0, 0.00035);
PID yAttPID(8.0, 0, 0.00035);

//Initialize to attitude mode
fcModeVal fcMode = ATT_MODE;

bool calibrated = false, armed = false;

//Timing
elapsedMicros currentTime;
unsigned long logStartTime;
unsigned long accTime, magTime, logTime, printTime, ratePIDTime, ahrsTime, attTime, rcFSStartTime;
double rateDT, ahrsDT, attDT;
int loopCount = 0, gyroCount = 0, accCount = 0, magCount = 0, ahrsCount = 0;

VECTOR gyro, acc, mag;
VECTOR rateSetpoints, attSetpoints;
VECTOR maxRates, maxAtts;
VECTOR ratePIDOutputs, attPIDOutputs;

bool newIMUData = false;
bool newAccData = false;
bool failsafeLandInitiated = false;

double initialAccMagnitude = 0;

void initTimers() {
    accTime = magTime = logTime = printTime = ahrsTime = ratePIDTime = currentTime;
}

//Put all IMU axes in the same frame
void setMARGVariables() {
    gyro.x = imu.filteredGyro.y;
    gyro.y = imu.filteredGyro.x;
    gyro.z = -1.0*imu.filteredGyro.z;

    acc.x = -1.0 * imu.scaledAcc.y;
    acc.y = -1.0 * imu.scaledAcc.x;
    acc.z = imu.scaledAcc.z;

    mag.x = imu.scaledMag.x;
    mag.y = imu.scaledMag.y;
    mag.z = imu.scaledMag.z;
}

//Deadband centered at 1500
int applyRCDeadband(int rcCommand) {
    return (abs(1500 - rcCommand) < rcDeadband) ? 1500 : rcCommand;
}

void resetPIDs() {
    ratePIDOutputs.x = ratePIDOutputs.y = ratePIDOutputs.z = 0;
    xRatePID.reset();
    yRatePID.reset();
    zRatePID.reset();

    xAttPID.reset();
    yAttPID.reset();
}

void failsafeLand() {
    if (!failsafeLandInitiated) {
        failsafeLandInitiated = true;
        rcFSStartTime = currentTime;
    }

    fcMode = ATT_MODE;
    rcCommands.throttle = RCFS_THROTTLE;
    rcCommands.roll = rcCommands.pitch = 0;

    //After one second, check for landing
    if (currentTime - rcFSStartTime > 1000) {
        setMARGVariables();
        if (abs(getVectorMagnitude(acc.x, acc.y, acc.z) - initialAccMagnitude) < 0.2) {
            //Disarm
            rcCommands.aux2 = 1000;
        }
    }
}

void getAccMagCalibration() {

    bool calWaitMessage = false;

    if (flightConfig.acc_calibrated) {
        imu.accOffset.x = flightConfig.acc_offset_x;
        imu.accOffset.y = flightConfig.acc_offset_y;
        imu.accOffset.z = flightConfig.acc_offset_z;

        imu.accScale.x = flightConfig.acc_scale_x;
        imu.accScale.y = flightConfig.acc_scale_y;
        imu.accScale.z = flightConfig.acc_scale_z;
    } else {
        imu.calibrateAcc(10);
        flightConfig.acc_scale_x = imu.accScale.x;
        flightConfig.acc_scale_y = imu.accScale.y;
        flightConfig.acc_scale_z = imu.accScale.z;

        flightConfig.acc_offset_x = imu.accOffset.x;
        flightConfig.acc_offset_y = imu.accOffset.y;
        flightConfig.acc_offset_z = imu.accOffset.z;

        flightConfig.acc_calibrated = true;
        flightConfig.writeConfig();

        Serial.println("Saved Acc calibration to flash.\n");

        calWaitMessage = true;
    }

    if (flightConfig.mag_calibrated) {
        imu.magOffset.x = flightConfig.mag_offset_x;
        imu.magOffset.y = flightConfig.mag_offset_y;
        imu.magOffset.z = flightConfig.mag_offset_z;
    } else {
        Serial.println("Send any message to begin Mag calibration.");
        while (!Serial.available());
        do {
            Serial.read();
        } while (Serial.available());

        imu.calibrateMag();
        flightConfig.mag_offset_x = imu.magOffset.x;
        flightConfig.mag_offset_y = imu.magOffset.y;
        flightConfig.mag_offset_z = imu.magOffset.z;

        flightConfig.mag_calibrated = true;
        flightConfig.writeConfig();

        SerialUSB.println("Saved Mag calibration to flash.");

        calWaitMessage = true;
    }

    if (calWaitMessage) {
        Serial.println("Calibration complete.\nSet the IMU still on a flat surface for gyro calibration.");
        Serial.println("Send any message to continue.");
        while (!Serial.available());
        do {
            Serial.read();
        } while (Serial.available());
    }

    snprintf(printMsgBuf, PRINT_MAX_BUF, "AccOffset: %f, %f, %f\n", imu.accOffset.x, imu.accOffset.y, imu.accOffset.z);
    printMessage(printMsgBuf);

    snprintf(printMsgBuf, PRINT_MAX_BUF, "AccScale: %f, %f, %f\n", imu.accScale.x, imu.accScale.y, imu.accScale.z);
    printMessage(printMsgBuf);

    snprintf(printMsgBuf, PRINT_MAX_BUF, "MagOffset: %f, %f, %f\n", imu.magOffset.x, imu.magOffset.y, imu.magOffset.z);
    printMessage(printMsgBuf);
}

void setInitialAccMagnitude() {
    const int accSampleSize = 1000;
    int accSamples = 0;
    VECTOR accSampleAverage;
    accSampleAverage.x = accSampleAverage.y = accSampleAverage.z = 0;
    do {
        if (imu.dataReady()) {
            imu.readAcc();
            setMARGVariables();

            accSampleAverage.x += acc.x;
            accSampleAverage.y += acc.y;
            accSampleAverage.z += acc.z;

            accSamples++;
            delay(1);
        }
    } while (accSamples < accSampleSize);

    accSampleAverage.x /= accSampleSize;
    accSampleAverage.y /= accSampleSize;
    accSampleAverage.z /= accSampleSize;

    initialAccMagnitude = getVectorMagnitude(accSampleAverage.x, accSampleAverage.y, accSampleAverage.z);
}

int initFlightControl() {
    initRC();

    pinMode(IMU_INT_PIN, INPUT);
    pinMode(IMU_SS_PIN, OUTPUT);
    digitalWriteFast(IMU_SS_PIN, 1);
    SPI.begin();

    initMotors();

    //Todo: Get INI configuration
    if (flightConfig.init() < 0) {
        return failMessage("Failed to load INI config.");
    }

    loggingInit();

    if (imu.init(GFSR_1000DPS, AFSR_2G, LPF_250HZ) < 0) {
        return failMessage("IMU initialization failed.");
    }
    imu.enableInterruptPin(IMU_INT_PIN);
    getAccMagCalibration();
    imu.calibrateGyro();

    //Rate limits
    maxRates.x = RATE_LIMIT_DPS;
    maxRates.y = RATE_LIMIT_DPS;
    maxRates.z = RATE_LIMIT_DPS;

    xRatePID.setOutputLimits(-1 * pwmMax, pwmMax);
    yRatePID.setOutputLimits(-1 * pwmMax, pwmMax);
    zRatePID.setOutputLimits(-1 * pwmMax, pwmMax);

    xRatePID.setILimit(1000);
    yRatePID.setILimit(1000);
    zRatePID.setILimit(1000);

    //Attitude limits
    maxAtts.x = ATT_LIMIT_DEG;
    maxAtts.y = ATT_LIMIT_DEG;

    xAttPID.setOutputLimits(-1 * maxRates.x, maxRates.x);
    yAttPID.setOutputLimits(-1 * maxRates.y, maxRates.y);

    //Wait for RC connection
    printMessage("Waiting for RC connection...");
    do {
        checkRCConnection();
        delay(500);
    } while (!rcDetected);
    printMessage("IBUS RC detected.");

    printMessage("Initialization complete.");

    initTimers();
    return 0;
}

void flightControlLoop() {
    if (newRCCommand) {
        newRCCommand = false;

        rcCommands.throttle = rawRCCommands[2];
        rcCommands.roll =   applyRCDeadband(rawRCCommands[0]);
        rcCommands.pitch =  applyRCDeadband(map(rawRCCommands[1], 1000, 2000, 2000, 1000));
        rcCommands.yaw =    applyRCDeadband(rawRCCommands[3]);

        rcCommands.aux1 = rawRCCommands[4];
        rcCommands.aux2 = rawRCCommands[5];
        rcCommands.aux3 = rawRCCommands[6];
        rcCommands.aux4 = rawRCCommands[7];

        if (!armed && rcCommands.aux2 >= 1750 && rcCommands.throttle < 1025) {
            armed = true;
            startMotors();
            startLog();
            logStartTime = currentTime;

            snprintf(logBuf, LOG_BUF_SIZE, "Time, gyro.x, gyro.y, gyro.z, rateSetpoints.x, rateSetpoints.y, rateSetpoints.z\n");
            writeLog(logBuf);
        } else if (armed && rcCommands.aux2 < 1750) {
            armed = false;
            stopMotors();
            resetPIDs();
            stopLog();
        }

        if (rcCommands.aux1 >= MODE_ATT_LOWER && rcCommands.aux1 <= MODE_ATT_UPPER) {

            if (fcMode != ATT_MODE) {
                //printMessage("Attitude mode enabled");
                //Notify via LED/buzzer
            }

            fcMode = ATT_MODE;

            //Attitude Mode command scaling
            attSetpoints.x = constrain(map(rcCommands.roll,    1000, 2000, -1 * maxAtts.x, maxAtts.x), -1 * maxAtts.x, maxAtts.x);
            attSetpoints.y = constrain(map(rcCommands.pitch,   1000, 2000, -1 * maxAtts.y, maxAtts.y), -1 * maxAtts.y, maxAtts.y);
            rateSetpoints.z = constrain(map(rcCommands.yaw,    1000, 2000, -1 * maxRates.z, maxRates.z), -1 * maxRates.z, maxRates.z);
        } else {

            if (fcMode != RATE_MODE) {
                //printMessage("Rate mode enabled");
                //Notify via LED/buzzer
            }

            fcMode = RATE_MODE;

            //Rate mode command scaling
            rateSetpoints.x = constrain(map(rcCommands.roll,    1000, 2000, -1 * maxRates.x, maxRates.x), -1 * maxRates.x, maxRates.x);
            rateSetpoints.y = constrain(map(rcCommands.pitch,   1000, 2000, -1 * maxRates.y, maxRates.y), -1 * maxRates.y, maxRates.y);
            rateSetpoints.z = constrain(map(rcCommands.yaw,     1000, 2000, -1 * maxRates.z, maxRates.z), -1 * maxRates.z, maxRates.z);
        }
    } else if (rcFailsafe) {
        failsafeLand();
    }

    if (imu.dataReady()) {
        imu.readGyro();
        imu.filterGyro();
        gyroCount++;

        if (currentTime - accTime >= 800) {
            imu.readAcc();
            //imu.filterAcc();
            accTime = currentTime;
            accCount++;

            newAccData = true;
        }

        if (armed) {
            //Just copy the gyro over for rate control...
            gyro.x = imu.filteredGyro.y;
            gyro.y = imu.filteredGyro.x;
            gyro.z = -1.0*imu.filteredGyro.z;

            //Update rate PIDs
            rateDT = (double)(currentTime - ratePIDTime) * SEC_PER_US;
            ratePIDOutputs.x = xRatePID.calculate(rateSetpoints.x, gyro.x, rateDT);
            ratePIDOutputs.y = yRatePID.calculate(rateSetpoints.y, gyro.y, rateDT);
            ratePIDOutputs.z = zRatePID.calculate(rateSetpoints.z, gyro.z, rateDT);
            ratePIDTime = currentTime;
            updateMotorCommands(rcCommands.throttle, ratePIDOutputs.x, ratePIDOutputs.y, ratePIDOutputs.z);
        }

        newIMUData = true;
    } 

    //Mag = 100Hz
    if (currentTime - magTime >= 10000) {
        imu.readMag();
        magTime = currentTime;
        magCount++;

        newIMUData = true;
    }

    if (newIMUData && (currentTime - ahrsTime) >= 1000) {

        setMARGVariables();

        //Update attitude estimate
        ahrsDT = (double)(currentTime - ahrsTime) * SEC_PER_US;
        //MadgwickAHRSupdate(degToRad(gyro.x), degToRad(gyro.y), degToRad(gyro.z), acc.x, acc.y, acc.z, mag.x, mag.y, mag.z, ahrsDT);
        MadgwickAHRSupdateIMU(degToRad(gyro.x), degToRad(gyro.y), degToRad(gyro.z), acc.x, acc.y, acc.z, ahrsDT);
        ahrsTime = currentTime;
        ahrsCount++;

        //Update attitude PIDs if attitude mode
        if (fcMode == ATT_MODE) {

            //Get Euler angles
            quaternionToEuler();

            attDT = (double)(currentTime - attTime) * SEC_PER_US;
            attPIDOutputs.x = xAttPID.calculate(attSetpoints.x, degAtt.x, attDT);
            attPIDOutputs.y = yAttPID.calculate(attSetpoints.y, degAtt.y, attDT);
            attTime = currentTime;

            rateSetpoints.x = attPIDOutputs.x;
            rateSetpoints.y = attPIDOutputs.y;
        }

        newIMUData = false;
    }

    //100Hz Log
    if (armed && currentTime - logTime >= (1000000 / 10)) {

        //Logging
        snprintf(logBuf, LOG_BUF_SIZE, "%f, %f, %f, %f, %f, %f, %f\n", (currentTime - logStartTime) * SEC_PER_US,
                    gyro.x, gyro.y, gyro.z, rateSetpoints.x, rateSetpoints.y, rateSetpoints.z);
        writeLog(logBuf);

        logTime = currentTime;
    }

    if (currentTime - printTime >= 1000000) {

        /*
        //Get and print Euler angles
        quaternionToEuler();

        snprintf(printMsgBuf, PRINT_MAX_BUF, "%0.2f, %0.2f, %0.2f", degAtt.x, degAtt.y, degAtt.z);
        Serial.println(printMsgBuf);
        */

        /*
        snprintf(printMsgBuf, PRINT_MAX_BUF, "%0.2f, %0.2f", attSetpoints.x, attSetpoints.y);
        Serial.println(printMsgBuf);

        snprintf(printMsgBuf, PRINT_MAX_BUF, "%0.2f, %0.2f", attPIDOutputs.x, attPIDOutputs.y);
        Serial.println(printMsgBuf);

        snprintf(printMsgBuf, PRINT_MAX_BUF, "%0.2f, %0.2f", ratePIDOutputs.x, ratePIDOutputs.y);
        Serial.println(printMsgBuf);

        Serial.println("");

        Serial.println("Gyro count: " + String(gyroCount));
        Serial.println("Acc count: " + String(accCount));
        Serial.println("Mag count: " + String(magCount));
        Serial.println("AHRS count: " + String(ahrsCount));
        Serial.println("Loop count: " + String(loopCount));
        */

        gyroCount = 0;
        accCount = 0;
        magCount = 0;
        ahrsCount = 0;
        loopCount = 0;

        printTime = currentTime;
    }

    loopCount++;
}