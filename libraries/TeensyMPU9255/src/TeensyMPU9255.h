#ifndef TeensyMPU9255_h
#define TeensyMPU9255_h

#include <TeensyFCTypes.h>

//gyro MAF sample width
#define GYRO_MAF_WIDTH 32
#define ACC_MAF_WIDTH 16

/* Full scale range settings */
enum gyroFSRVal {
    GFSR_250DPS = 0,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accFSRVal {
    AFSR_2G = 0,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

/* Lowpass filter configuration */
enum lpfVal {
    LPF_250HZ = 0,
    LPF_184HZ,
    LPF_92HZ,
    LPF_41HZ,
    LPF_20HZ,
    LPF_10HZ,
    LPF_5HZ,
    LPF_3600HZ
};

/* x, y, z data structs */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} RAWDATA;

class TeensyMPU9255 {
private:

    bool gyroEnabled, accEnabled, magEnabled;
    uint8_t gyroFSR, accFSR, magFSR, lpf;
    double gyroSensitivity, accSensitivity, magSensitivity;
    bool useInterruptPin;
    int interruptPin;
    int ssPin;
    int gyroMAFSamples, accMAFSamples;
    
    int8_t reset();

    void setLPF(uint8_t setLPF);
    void setGyroFSR(uint8_t value);
    void setAccFSR(uint8_t value);
    void setSampleRateDivider(uint8_t div);

public:

    RAWDATA rawGyro, rawAcc, rawMag;
    VECTOR scaledGyro, scaledAcc, scaledMag;
    VECTOR gyroBias, accOffset, accScale, magAdj, magOffset;
    VECTOR gyroMAF[GYRO_MAF_WIDTH], accMAF[ACC_MAF_WIDTH];
    VECTOR filteredGyro, filteredAcc;

	TeensyMPU9255(int setSSPin, bool setGyroEnabled = true, bool setAccEnabled = true, bool setMagEnabled = true);

	int8_t init(gyroFSRVal setGyroFSRValue = GFSR_250DPS, accFSRVal setAccFSRValue = AFSR_2G, lpfVal setLPFValue = LPF_250HZ);
    void calibrateGyro(int sampleCount = 1280);
    void calibrateAcc(int sampleCount = 10);
    void calibrateMag(int sampleCount = 1280);
    bool dataReady();
    bool magDataReady();
    
    void printRegister(const char *regName, uint8_t regVal);

    void enableInterruptPin(int pin);
    void setMagBypass(bool enable);
    
    void readGyro(bool scale = true);
    void filterGyro();
    void filterAcc();

    void readAcc(bool scale = true);
    void readAccGyro(bool scale = true);
    void readMag(bool scale = true);
};

#endif