#include <SPI.h>

SPISettings MPU9250Slow(1000000, MSBFIRST, SPI_MODE3);
SPISettings MPU9250Fast(20000000, MSBFIRST, SPI_MODE3); 

uint8_t readSPIReg(uint8_t ss, uint8_t reg, bool fastClock) {
    SPI.beginTransaction(fastClock ? MPU9250Fast : MPU9250Slow);
    digitalWriteFast(ss, 0);
    SPI.transfer(reg | 0x80);
    uint8_t value = SPI.transfer(0x00);
    digitalWriteFast(ss, 1);
    SPI.endTransaction();

    return value;
}

void readSPIRegs(uint8_t ss, uint8_t reg, uint8_t count, uint8_t dest[], bool fastClock) {
    SPI.beginTransaction(fastClock ? MPU9250Fast : MPU9250Slow);
    digitalWriteFast(ss, 0);
    SPI.transfer(reg | 0x80);

    for (uint8_t i = 0; i < count; i++) {
        dest[i] = SPI.transfer(0x00);
    }

    digitalWriteFast(ss, 1);
    SPI.endTransaction();
}

void writeSPIReg(uint8_t ss, uint8_t reg, uint8_t value, bool fastClock) {
    SPI.beginTransaction(fastClock ? MPU9250Fast : MPU9250Slow);
    digitalWriteFast(ss, 0);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWriteFast(ss, 1);
    SPI.endTransaction();
}