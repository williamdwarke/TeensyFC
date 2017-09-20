#include <Arduino.h>
#include <Wire.h>
#include <float.h>

#include <SerialMessage.h>

#include "util/SPIDevice.h"
#include "util/AccCalibration.h"
#include "MPU9255RegisterMap.h"

#include "TeensyMPU9255.h"

/*
 * Todo:
 * -Implement mag access via I2C master.
 * -Implement self-test functionality.
 * -Implement temperature readings.
 * -Implement disabling of internal gyro offset settings.
 */
TeensyMPU9255::TeensyMPU9255(int setSSPin, bool setGyroEnabled, bool setAccEnabled, bool setMagEnabled) {
    ssPin = setSSPin;
    gyroEnabled = setGyroEnabled;
    accEnabled = setAccEnabled;
    magEnabled = setMagEnabled;

    //Initialize member variables to default
    gyroFSR = GFSR_250DPS;
    accFSR = AFSR_2G;
    lpf = LPF_250HZ;

    //Disable interrupt pin by default
    useInterruptPin = false;
    interruptPin = 0;

    //Zero out accelerometer offsets and scales
    accOffset.x = 0;
    accOffset.y = 0;
    accOffset.z = 0;

    accScale.x = 0;
    accScale.y = 0;
    accScale.z = 0;

    magOffset.x = 0;
    magOffset.y = 0;
    magOffset.z = 0;

    //Zero moving-average filters
    gyroMAFSamples = 0;
    for (int i = 0; i < GYRO_MAF_WIDTH; i++) {
        gyroMAF[i].x = gyroMAF[i].y = gyroMAF[i].z = 0;
    }

    accMAFSamples = 0;
    for (int i = 0; i < ACC_MAF_WIDTH; i++) {
        accMAF[i].x = accMAF[i].y = accMAF[i].z = 0;
    }
}

int8_t TeensyMPU9255::reset() {
    //Set the H_RESET bit
    writeSPIReg(ssPin, MPU9255_PWR_MGMT_1, 0x80);

    //Delay until reset has completed
    unsigned long resetTime = millis();
    unsigned long readTime = resetTime;
    uint8_t regVal = 0x00;
    do {
        //1 second reset timeout
        if (readTime - resetTime > 1000) {
            return failMessage("MPU9255 reset timed out.");
        }

        delay(100);
        readTime = millis();

        //Reset is complete when MSB is reset
        regVal = readSPIReg(ssPin, MPU9255_PWR_MGMT_1);
    } while (regVal & 0x80);

    return 0;
}

/* Applies only to gyro and acc if mag is disabled. */
bool TeensyMPU9255::dataReady() {
    return useInterruptPin ? digitalReadFast(interruptPin) : (readSPIReg(ssPin, MPU9255_INT_STATUS) & 0x01);
}

void TeensyMPU9255::setLPF(uint8_t setLPF) {
    writeSPIReg(ssPin, MPU9255_CONFIG, setLPF);
}

void TeensyMPU9255::enableInterruptPin(int pin) {
    useInterruptPin = true;
    interruptPin = pin;

    //Active high, push-pull, latching, clear on read
    uint8_t intPinCfg = readSPIReg(ssPin, MPU9255_INT_PIN_CFG);
    writeSPIReg(ssPin, MPU9255_INT_PIN_CFG, intPinCfg | 0x30);

    //Enable data ready interrupt
    writeSPIReg(ssPin, MPU9255_INT_ENABLE, 0x01);
}

void TeensyMPU9255::setGyroFSR(uint8_t value) {
    uint8_t regVal = readSPIReg(ssPin, MPU9255_GYRO_CONFIG);

    //Clear the FSR bits [4:3] and then set them
    regVal &= ~(0x11 << 3);
    regVal |= value << 3;

    writeSPIReg(ssPin, MPU9255_GYRO_CONFIG, regVal);
}

void TeensyMPU9255::setAccFSR(uint8_t value) {
    uint8_t regVal = readSPIReg(ssPin, MPU9255_GYRO_CONFIG);

    //Clear the FSR bits [4:3] and then set them
    regVal &= ~(0x11 << 3);
    regVal |= value << 3;

    writeSPIReg(ssPin, MPU9255_GYRO_CONFIG, regVal);
}

void TeensyMPU9255::setSampleRateDivider(uint8_t div) {
    writeSPIReg(ssPin, MPU9255_SMPLRT_DIV, div);
}

void TeensyMPU9255::printRegister(const char *regName, uint8_t regVal) {
    snprintf(printMsgBuf, PRINT_MAX_BUF, "%s: 0x%02X", regName, regVal);
    printMessage(printMsgBuf);
}

int8_t TeensyMPU9255::init(gyroFSRVal setGyroFSRValue, accFSRVal setAccFSRValue, lpfVal setLPFValue) {

    gyroFSR = setGyroFSRValue;
    accFSR = setAccFSRValue;
    lpf = setLPFValue;

    //Calculate gyro, acc, and mag sensitivities
    switch (gyroFSR) {
        case GFSR_250DPS:
            gyroSensitivity = 500.0 / 65536.0;
            break;
        case GFSR_500DPS:
            gyroSensitivity = 1000.0 / 65536.0;
            break;
        case GFSR_1000DPS:
            gyroSensitivity = 2000.0 / 65536.0;
            break;
        case GFSR_2000DPS:
            gyroSensitivity = 4000.0 / 65536.0;
            break;
        default:
            printMessage("Warning: Invalid gyroFSR. Setting to 250DPS.");
            gyroSensitivity = 500.0 / 65536.0;
    }

    switch (accFSR) {
        case AFSR_2G:
            accSensitivity = 4.0 / 65536.0;
            break;
        case AFSR_4G:
            accSensitivity = 8.0 / 65536.0;
            break;
        case AFSR_8G:
            accSensitivity = 16.0 / 65536.0;
            break;
        case AFSR_16G:
            accSensitivity = 32.0 / 65536.0;
            break;
        default:
            printMessage("Warning: Invalid accFSR. Setting to 2G.");
            accSensitivity = 4.0 / 65536.0;
    }

    //Only 1 mag sensitivity
    magSensitivity = 9824.0 / 65536.0;

    //Reset the chip (and all register values)
    if (reset() < 0) {
        return -1;
    }

    //Initialize configuration registers
    uint8_t mpu9255Config[3][2] = {
        //Set clock source to auto
        {MPU9255_PWR_MGMT_1, 0x01},

        //Enable SPI interface and mag I2C master
        {MPU9255_USER_CTRL, 0x30},

        //Turn off FIFO
        {MPU9255_FIFO_EN, 0x00}
    };

    for (uint8_t i = 0; i < 3; i++) {
        writeSPIReg(ssPin, mpu9255Config[i][0], mpu9255Config[i][1]);
    }

    //Check WhoAmI
    uint8_t whoAmI = readSPIReg(ssPin, MPU9255_WHOAMI);
    if (whoAmI != MPU9255_WHOAMI_VAL) {
        snprintf(printMsgBuf, PRINT_MAX_BUF, "MPU9255 WHOAMI validation failed! (0x%02X != 0x%02X)", whoAmI, MPU9255_WHOAMI_VAL);
        return failMessage(printMsgBuf);
    }

    //No sample rate division (1:1)
    setSampleRateDivider(0);

    if (gyroEnabled) {
        writeSPIReg(ssPin, MPU9255_GYRO_CONFIG, 0x00);
        setGyroFSR(gyroFSR);
    } else {
        //Set PWR_MGMT_2 [0:2] to disable gyro
        writeSPIReg(ssPin, MPU9255_PWR_MGMT_2, 0x07);
    }

    if (accEnabled) {
        setAccFSR(accFSR);
    } else {
        //Set PWR_MGMT_2 [3:5] to disable acc
        uint8_t tmp = readSPIReg(ssPin, MPU9255_PWR_MGMT_2);
        writeSPIReg(ssPin, MPU9255_PWR_MGMT_2, tmp | 0x38);
    }

    /*
     *  Set the requested Gyro/Acc LPF cutoff frequency.
     *  Note: This also configures the Gyro ODR
     *  via page 13 of the register map document.
     */
    setLPF(lpf);

    if (magEnabled) {

        uint8_t ak8963Config[7][2] = {
            //Enable I2C and configure single-byte write
            {MPU9255_I2C_MST_CTRL, 0x0D},
            {MPU9255_I2C_SLV0_ADDR, AK8963_I2C_ADDR},
            {MPU9255_I2C_SLV0_CTRL, 0x81},

            //Reset
            {MPU9255_I2C_SLV0_REG, AK8963_CNTL2},
            {MPU9255_I2C_SLV0_DO, 0x01},
            //{MPU9255_I2C_SLV0_CTRL, 0x81},

            //Configure mag for fast (100Hz) mode
            {MPU9255_I2C_SLV0_REG, AK8963_CNTL1},
            {MPU9255_I2C_SLV0_DO, 0x16}
            //{MPU9255_I2C_SLV0_CTRL, 0x81},
        };

        for (uint8_t i = 0; i < 7; i++) {
            writeSPIReg(ssPin, ak8963Config[i][0], ak8963Config[i][1]);

            //Account for internal I2C delay
            if (i == 4) {
                //mag reset
                delay(100);
            } else {
                delay(1);
            }
        }

        //Check the mag device ID
        writeSPIReg(ssPin, MPU9255_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
        writeSPIReg(ssPin, MPU9255_I2C_SLV0_CTRL, 0x81);
        writeSPIReg(ssPin, MPU9255_I2C_SLV0_REG, AK8963_WIA);
        delay(100);
        uint8_t magDevID = readSPIReg(ssPin, MPU9255_EXT_SENS_DATA_00);
        if (magDevID != AK8963_DEVICE_ID) {
            snprintf(errMsgBuf, PRINT_MAX_BUF, "AK8963 device ID mismatch! (0x%02X != 0x%02X)", magDevID, AK8963_DEVICE_ID);
            return failMessage(errMsgBuf);
        }

        //Get mag sensitivity adjustment values
        writeSPIReg(ssPin, MPU9255_I2C_SLV0_REG, AK8963_ASAX);
        writeSPIReg(ssPin, MPU9255_I2C_SLV0_CTRL, 0x83);
        delay(100);
        uint8_t magAdjBuf[3];
        readSPIRegs(ssPin, MPU9255_EXT_SENS_DATA_00, 3, magAdjBuf);

        //Convert to signed floating-point adjustment values
        magAdj.x = (double)(((int8_t)magAdjBuf[0] - 128) / 256.0) + 1.0;
        magAdj.y = (double)(((int8_t)magAdjBuf[1] - 128) / 256.0) + 1.0;
        magAdj.z = (double)(((int8_t)magAdjBuf[2] - 128) / 256.0) + 1.0;

        //snprintf(printMsgBuf, PRINT_MAX_BUF, "Mag sens adj: 0x%02X, 0x%02X, 0x%02X", magAdjBuf[0], magAdjBuf[1], magAdjBuf[2]);
        //printMessage(printMsgBuf);

        //Configure for 7-byte continuous read of mag data registers
        //writeSPIReg(ssPin, MPU9255_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
        writeSPIReg(ssPin, MPU9255_I2C_SLV0_CTRL, 0x87);
        writeSPIReg(ssPin, MPU9255_I2C_SLV0_REG, AK8963_HXL);
    }

    return 0;
}

void TeensyMPU9255::readAccGyro(bool scale) {
    uint8_t rawDataBuf[14];
    readSPIRegs(ssPin, MPU9255_ACCEL_XOUT_H, 14, &rawDataBuf[0]);

    rawAcc.x = (int16_t)(((int16_t)rawDataBuf[0] << 8) | rawDataBuf[1]); 
    rawAcc.y = (int16_t)(((int16_t)rawDataBuf[2] << 8) | rawDataBuf[3]);
    rawAcc.z = (int16_t)(((int16_t)rawDataBuf[4] << 8) | rawDataBuf[5]);

    rawGyro.x = (int16_t)(((int16_t)rawDataBuf[8] << 8) | rawDataBuf[9]);
    rawGyro.y = (int16_t)(((int16_t)rawDataBuf[10] << 8) | rawDataBuf[11]);
    rawGyro.z = (int16_t)(((int16_t)rawDataBuf[12] << 8) | rawDataBuf[13]);
}

void TeensyMPU9255::readGyro(bool scale) {
    uint8_t rawDataBuf[6];
    readSPIRegs(ssPin, MPU9255_GYRO_XOUT_H, 6, &rawDataBuf[0], true);

    //Read the raw data into signed, 16-bit values
    rawGyro.x = (int16_t)(((int16_t)rawDataBuf[0] << 8) | rawDataBuf[1]);
    rawGyro.y = (int16_t)(((int16_t)rawDataBuf[2] << 8) | rawDataBuf[3]);
    rawGyro.z = (int16_t)(((int16_t)rawDataBuf[4] << 8) | rawDataBuf[5]);

    if (scale) {
        //Scale all values to double precision
        scaledGyro.x = ((double)rawGyro.x * gyroSensitivity) - gyroBias.x;
        scaledGyro.y = ((double)rawGyro.y * gyroSensitivity) - gyroBias.y;
        scaledGyro.z = ((double)rawGyro.z * gyroSensitivity) - gyroBias.z;
    }
}

void TeensyMPU9255::filterGyro() {
    //Shift samples through the filter
    VECTOR sum;
    sum.x = sum.y = sum.z = 0;
    for (int i = GYRO_MAF_WIDTH - 1; i > 0; i--) {
        //Calculate the sums while we're at it
        sum.x += gyroMAF[i-1].x;
        sum.y += gyroMAF[i-1].y;
        sum.z += gyroMAF[i-1].z;

        //Shift each sample to the right by one
        gyroMAF[i].x = gyroMAF[i-1].x;
        gyroMAF[i].y = gyroMAF[i-1].y;
        gyroMAF[i].z = gyroMAF[i-1].z;
    }

    //Shift in newest sample
    gyroMAF[0].x = scaledGyro.x;
    gyroMAF[0].y = scaledGyro.y;
    gyroMAF[0].z = scaledGyro.z;

    //Add it to the sum
    sum.x += scaledGyro.x;
    sum.y += scaledGyro.y;
    sum.z += scaledGyro.z;

    //Average only gyroMAFSamples samples if filter queue isn't full
    if (gyroMAFSamples < GYRO_MAF_WIDTH) {
        gyroMAFSamples++;
    }

    //Average
    filteredGyro.x = (sum.x / (double)gyroMAFSamples);
    filteredGyro.y = (sum.y / (double)gyroMAFSamples);
    filteredGyro.z = (sum.z / (double)gyroMAFSamples);
}

void TeensyMPU9255::readAcc(bool scale) {
    //Read the six raw data registers sequentially into data array
    uint8_t rawDataBuf[6];
    readSPIRegs(ssPin, MPU9255_ACCEL_XOUT_H, 6, &rawDataBuf[0], true);

    //Convert the MSB and LSB into a signed 16-bit value
    rawAcc.x = (uint16_t)(((uint16_t)rawDataBuf[0] << 8) | rawDataBuf[1]); 
    rawAcc.y = (uint16_t)(((uint16_t)rawDataBuf[2] << 8) | rawDataBuf[3]);
    rawAcc.z = (uint16_t)(((uint16_t)rawDataBuf[4] << 8) | rawDataBuf[5]);

    if (scale) {
        //Scale all values to double precision
        scaledAcc.x = ((double)rawAcc.x - accOffset.x) * accScale.x;
        scaledAcc.y = ((double)rawAcc.y - accOffset.y) * accScale.y;
        scaledAcc.z = ((double)rawAcc.z - accOffset.z) * accScale.z;
    }
}

void TeensyMPU9255::filterAcc() {
    //Shift samples through the filter
    VECTOR sum;
    sum.x = sum.y = sum.z = 0;
    for (int i = ACC_MAF_WIDTH - 1; i > 0; i--) {
        //Calculate the sums while we're at it
        sum.x += accMAF[i-1].x;
        sum.y += accMAF[i-1].y;
        sum.z += accMAF[i-1].z;

        //Shift each sample to the right by one
        accMAF[i].x = accMAF[i-1].x;
        accMAF[i].y = accMAF[i-1].y;
        accMAF[i].z = accMAF[i-1].z;
    }

    //Shift in newest sample
    accMAF[0].x = scaledAcc.x;
    accMAF[0].y = scaledAcc.y;
    accMAF[0].z = scaledAcc.z;

    //Add it to the sum
    sum.x += scaledAcc.x;
    sum.y += scaledAcc.y;
    sum.z += scaledAcc.z;

    //Average only mafSamples samples if filter queue isn't full
    if (accMAFSamples < GYRO_MAF_WIDTH) {
        accMAFSamples++;
    }

    //Average
    filteredAcc.x = (sum.x / (double)accMAFSamples);
    filteredAcc.y = (sum.y / (double)accMAFSamples);
    filteredAcc.z = (sum.z / (double)accMAFSamples);
}

void TeensyMPU9255::readMag(bool scale) {

    uint8_t rawDataBuf[7];

    readSPIRegs(ssPin, MPU9255_EXT_SENS_DATA_00, 7, &rawDataBuf[0], false);

    //Ignore magnetic overflow
    if (!(rawDataBuf[6] & 0x08)) {
        rawMag.x = (int16_t)(((int16_t)rawDataBuf[1] << 8) | rawDataBuf[0]);
        rawMag.y = (int16_t)(((int16_t)rawDataBuf[3] << 8) | rawDataBuf[2]);
        rawMag.z = (int16_t)(((int16_t)rawDataBuf[5] << 8) | rawDataBuf[4]);

        //Scale all values to double precision
        scaledMag.x = ((double)rawMag.x * magAdj.x * magSensitivity) - magOffset.x;
        scaledMag.y = ((double)rawMag.y * magAdj.y * magSensitivity) - magOffset.y;
        scaledMag.z = ((double)rawMag.z * magAdj.z * magSensitivity) - magOffset.z;
    }
}

void TeensyMPU9255::calibrateGyro(int sampleCount) {

    //FSR must be 250 DPS for calibration
    setGyroFSR(GFSR_250DPS);

    //Todo: wait until it's sitting still via acc

    long xSum = 0, ySum = 0, zSum = 0;
    for (int i = 0; i < sampleCount; i++) {
        //Read gyro sample without scaling
        do {
            delay(1);
        } while (!dataReady());
        readGyro(false);
        xSum += rawGyro.x;
        ySum += rawGyro.y;
        zSum += rawGyro.z;
    }

    //Compute average offsets
    int16_t xOffset = round((double)xSum / (double)sampleCount);
    int16_t yOffset = round((double)ySum / (double)sampleCount);
    int16_t zOffset = round((double)zSum / (double)sampleCount);

    /*
    //Write unsigned offset bytes to gyro offset registers
    writeSPIReg(ssPin, XG_OFFSET_H, (xOffset >> 8));
    writeSPIReg(ssPin, XG_OFFSET_L, (xOffset & 0x00FF));
    writeSPIReg(ssPin, YG_OFFSET_H, (yOffset >> 8));
    writeSPIReg(ssPin, YG_OFFSET_L, (yOffset & 0x00FF));
    writeSPIReg(ssPin, ZG_OFFSET_H, (zOffset >> 8));
    writeSPIReg(ssPin, ZG_OFFSET_L, (zOffset & 0x00FF));
    */

    gyroBias.x = (double)xOffset * (500.0 / 65536.0);
    gyroBias.y = (double)yOffset * (500.0 / 65536.0);
    gyroBias.z = (double)zOffset * (500.0 / 65536.0);

    //printMessage("Gyro calibration complete.");

    //printRegister("XG_OFFSET_H", readSPIReg(ssPin, XG_OFFSET_H));
    //printRegister("XG_OFFSET_L", readSPIReg(ssPin, XG_OFFSET_L));

    //Return FSR to original value
    setGyroFSR(gyroFSR);
}

void TeensyMPU9255::calibrateAcc(int sampleCount) {
    printMessage("\nAccelerometer Calibration:\n");
    snprintf(printMsgBuf, PRINT_MAX_BUF, "Samples: %d\nSend a single message to take a sample.\nMake sure the accelerometer is not moving while each sample is being taken.", sampleCount);
    printMessage(printMsgBuf);

    //Calibration code
    //initialize
    samp_capacity = sampleCount;
    n_samp = 0;
    data = (long*)malloc(samp_capacity * 3 * sizeof(long));
    reset_calibration_matrices();

    //initialize beta to something reasonable
    beta[0] = beta[1] = beta[2] = 512.0;
    beta[3] = beta[4] = beta[5] = 0.0095;

    for (int sampleNum = 0; sampleNum < sampleCount; sampleNum++) {
        while (!SerialUSB.available());
        //Clear out input buffer
        do {
            SerialUSB.read();
        } while (SerialUSB.available());

        snprintf(printMsgBuf, PRINT_MAX_BUF, "\nTaking sample %d...\n", sampleNum+1);
        printMessage(printMsgBuf);

        delay(200);
        take_sample(data + 3 * (n_samp % samp_capacity));
        n_samp++;

        printMessage("Done!");
    }

    SerialUSB.println("Performing calibration calculations...");
    calibrate_model();

    //XYZ offsets and scales are stored in betas
    accOffset.x = beta[0];
    accOffset.y = beta[1];
    accOffset.z = beta[2];

    accScale.x = beta[3];
    accScale.y = beta[4];
    accScale.z = beta[5];

    //Print samples for 5 seconds for verification
    unsigned long startTime = millis();
    do {
        delay(1000);
        readAcc();
        scaledAcc.x = ((double)rawAcc.x - accOffset.x) * accScale.x;
        scaledAcc.y = ((double)rawAcc.y - accOffset.y) * accScale.y;
        scaledAcc.z = ((double)rawAcc.z - accOffset.z) * accScale.z;
        SerialUSB.println("Acc: " + String(scaledAcc.x) + ", " + String(scaledAcc.y) + ", " + String(scaledAcc.z) + " g");
    } while(millis() - startTime < 5000);
}

void TeensyMPU9255::calibrateMag(int sampleCount) {
    printMessage("Magnetometer Calibration:\n");
    printMessage("Wave the IMU in a 3-dimensional figure-8 pattern for several seconds.");
    for (int i = 3; i > 0; i--) {
        snprintf(printMsgBuf, PRINT_MAX_BUF, "%d", i);
        printMessage(printMsgBuf);
        delay(1000);
    }
    printMessage("Go!");

    //Read one sample for baseline readings
    readMag();

    VECTOR magMin, magMax;
    magMin = magMax = scaledMag;
    
    int magSamples = 0;
    do {
        readMag();

        magMin.x = min(magMin.x, scaledMag.x);
        magMin.y = min(magMin.y, scaledMag.y);
        magMin.z = min(magMin.z, scaledMag.z);

        magMax.x = max(magMax.x, scaledMag.x);
        magMax.y = max(magMax.y, scaledMag.y);
        magMax.z = max(magMax.z, scaledMag.z);

        magSamples++;

        delay(10);
    } while (magSamples < sampleCount);

    snprintf(printMsgBuf, PRINT_MAX_BUF, "Collected %d samples.", magSamples);
    snprintf(printMsgBuf, PRINT_MAX_BUF, "Calibration Complete.\nMin values: %f, %f, %f.\nMax values: %f, %f, %f\n", magMin.x, magMin.y, magMin.z, magMax.x, magMax.y, magMax.z);
    printMessage(printMsgBuf);

    magOffset.x = (magMin.x + magMax.x) / 2.0;
    magOffset.y = (magMin.y + magMax.y) / 2.0;
    magOffset.z = (magMin.z + magMax.z) / 2.0;

    snprintf(printMsgBuf, PRINT_MAX_BUF, "Averages: %f, %f, %f\n", magOffset.x, magOffset.y, magOffset.z);
    printMessage(printMsgBuf);
}