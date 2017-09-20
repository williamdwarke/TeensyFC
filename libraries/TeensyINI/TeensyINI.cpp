/*
 * TeensyINI.cpp
 *
 *  Created on: May 29, 2015
 *      Author: wwarke
 */
#include <SdFat.h>
#include <SerialMessage.h>

#include "IniFile.h"
#include "TeensyINI.h"

File iniFileHandle, bakFileHandle;
IniFile ini(iniFilename);

TeensyINI::TeensyINI() {
    esc_cal_flag = false;
    acc_calibrated = false;
    mag_calibrated = false;

    acc_offset_x = acc_offset_y = acc_offset_z = 0;
    acc_scale_x = acc_scale_y = acc_scale_z = 0;
    mag_offset_x = mag_offset_y = mag_offset_z = 0;
}

/* loadDefaults() sets all of the local configuration values to their default values. */
void TeensyINI::loadDefaults() {
    esc_cal_flag = false;
    acc_calibrated = false;
    mag_calibrated = false;

    acc_offset_x = acc_offset_y = acc_offset_z = 0;
    acc_scale_x = acc_scale_y = acc_scale_z = 0;
    mag_offset_x = mag_offset_y = mag_offset_z = 0;
}

/* init() loads the LOG configuration from the INI file.  */
int TeensyINI::init() {

    if (!sdExInitialized) {
        if (!sdEx.begin()) {
            return failMessage("SdFatSdioEX begin() failed");
        }
        sdEx.chvol();

        sdExInitialized = true;
    }


    //If .TeensyINI file does not exist, create it
    if (!sdEx.exists(iniFilename)) {
        if (createFile() < 0) {
            snprintf(errMsgBuf, PRINT_MAX_BUF, "Failed to create INI file %s.", iniFilename);
            return failMessage(errMsgBuf);
        } else {
            snprintf(printMsgBuf, PRINT_MAX_BUF, "INI file %s created.\n", iniFilename);
            printMessage(printMsgBuf);
        }
    } else {
        //Open TeensyINI file for reading values
        readConfig();
    }

    return 0;
}

/* createFile() creates and initializes the INI configuration file. */
int TeensyINI::createFile() {
    //Load default configuration values
    loadDefaults();

    printMessage("Loaded default INI values");

    //Open and close INI file to inittialize it
    if (!iniFileHandle.open(iniFilename, FILE_WRITE)) {
        snprintf(errMsgBuf, PRINT_MAX_BUF, "failed to create INI file %s.", iniFilename);
        return failMessage(errMsgBuf);
    }
    iniFileHandle.close();

    //Store the newly populated config class to the INI
    return writeConfig();
}

/* readConfig() saves the INI config to program memory. */
void TeensyINI::readConfig() {
    const size_t bufferLen = 128;
    char buffer[bufferLen];

    if (!ini.open()) {
        snprintf(errMsgBuf, PRINT_MAX_BUF, "Failed to open INI file %s", iniFilename);
        failMessage(errMsgBuf);
        return;
    }

    //Calibration - booleans
    ini.getValue(iniSections[0], "esc_cal_flag", buffer, bufferLen, &esc_cal_flag);
    ini.getValue(iniSections[0], "acc_calibrated", buffer, bufferLen, &acc_calibrated);
    ini.getValue(iniSections[0], "mag_calibrated", buffer, bufferLen, &mag_calibrated);

    //Calibration - acc/mag values
    ini.getValue(iniSections[0], "acc_offset_x", buffer, bufferLen, &acc_offset_x);
    ini.getValue(iniSections[0], "acc_offset_y", buffer, bufferLen, &acc_offset_y);
    ini.getValue(iniSections[0], "acc_offset_z", buffer, bufferLen, &acc_offset_z);

    ini.getValue(iniSections[0], "acc_scale_x", buffer, bufferLen, &acc_scale_x);
    ini.getValue(iniSections[0], "acc_scale_y", buffer, bufferLen, &acc_scale_y);
    ini.getValue(iniSections[0], "acc_scale_z", buffer, bufferLen, &acc_scale_z);

    ini.getValue(iniSections[0], "mag_offset_x", buffer, bufferLen, &mag_offset_x);
    ini.getValue(iniSections[0], "mag_offset_y", buffer, bufferLen, &mag_offset_y);
    ini.getValue(iniSections[0], "mag_offset_z", buffer, bufferLen, &mag_offset_z);

    ini.close();
}

/* writeConfig() stores the current program configuration to the INI file. */
int TeensyINI::writeConfig() {
    int bufIndex = 0;
    size_t nb;
    const int bufSize = 32768;
    char printBuf[bufSize], tmpBuf[bufSize];

    //Make a backup of the current TeensyINI file for safekeeping
    if (sdEx.exists(iniFilename)) {
        //Read entire TeensyINI file into a temporary buffer
        if (!iniFileHandle.open(iniFilename, FILE_READ)) {
            snprintf(errMsgBuf, PRINT_MAX_BUF, "failed to open INI file %s.", iniFilename);
            return failMessage(errMsgBuf);
        }

        while (iniFileHandle.available()) {
            tmpBuf[bufIndex++] = iniFileHandle.read();
        }
        tmpBuf[bufIndex] = '\0';
        iniFileHandle.close();

        //Write the TeensyINI contents to the backup file
        //Remove existing backup if present
        if (sdEx.exists(bakFilename)) {
            sdEx.remove(bakFilename); 
        }

        bakFileHandle.open(bakFilename, FILE_WRITE);
        if (!bakFileHandle) {
            snprintf(errMsgBuf, PRINT_MAX_BUF, "failed to open backup INI file %s.", bakFilename);
            failMessage(errMsgBuf);
        } else {
            nb = strlen(tmpBuf);
            if (nb != (size_t)bakFileHandle.write(tmpBuf, nb)) {
                bakFileHandle.close();
                return failMessage("Backup INI write failed");
            }
            bakFileHandle.close();
        }

        //Remove the original TeensyINI file
        sdEx.remove(iniFilename); 
    }

    bufIndex = 0;
    iniFileHandle.open(iniFilename, FILE_WRITE);

    //Calibration Section
    snprintf(&printBuf[bufIndex], bufSize-bufIndex, "[%s]\n", iniSections[0]);
    bufIndex = strlen(printBuf);

    //Calibration - booleans
    snprintf(&printBuf[bufIndex], bufSize-bufIndex, "esc_cal_flag=%s\n", esc_cal_flag ? "true" : "false");
    bufIndex = strlen(printBuf);
    snprintf(&printBuf[bufIndex], bufSize-bufIndex, "acc_calibrated=%s\n", acc_calibrated ? "true" : "false");
    bufIndex = strlen(printBuf);
    snprintf(&printBuf[bufIndex], bufSize-bufIndex, "mag_calibrated=%s\n", mag_calibrated ? "true" : "false");
    bufIndex = strlen(printBuf);

    //Calibration - acc/mag values
    snprintf(&printBuf[bufIndex], bufSize-bufIndex, "acc_offset_x=%0.16f\nacc_offset_y=%0.16f\nacc_offset_z=%0.16f\n", acc_offset_x, acc_offset_y, acc_offset_z);
    bufIndex = strlen(printBuf);
    snprintf(&printBuf[bufIndex], bufSize-bufIndex, "acc_scale_x=%0.16f\nacc_scale_y=%0.16f\nacc_scale_z=%0.16f\n", acc_scale_x, acc_scale_y, acc_scale_z);
    bufIndex = strlen(printBuf);
    snprintf(&printBuf[bufIndex], bufSize-bufIndex, "mag_offset_x=%0.16f\nmag_offset_y=%0.16f\nmag_offset_z=%0.16f\n", mag_offset_x, mag_offset_y, mag_offset_z);
    bufIndex = strlen(printBuf);

    printBuf[bufIndex] = '\0';
    nb = strlen(printBuf);
    if (nb != (size_t)iniFileHandle.write(printBuf, nb)) {
        iniFileHandle.close();
        return failMessage("TeensyINI write failed");
    }

    iniFileHandle.close();
    return 0;
}

/*
void TeensyINI::printConfig(bool printSerial) {
    int bufIndex = 0;
    const int bufSize = 32768;
    char printBuf[bufSize], tmpBuf[bufSize];
}
*/