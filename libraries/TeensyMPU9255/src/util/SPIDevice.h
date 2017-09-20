#ifndef SPIDevice_h
#define SPIDevice_h

    //Internal register functions
    void writeSPIReg(uint8_t ss, uint8_t reg, uint8_t value, bool fastClock = false);
    uint8_t readSPIReg(uint8_t ss, uint8_t reg, bool fastClock = false);
    void readSPIRegs(uint8_t ss, uint8_t startReg, uint8_t count, uint8_t dest[], bool fastClock = false);

#endif