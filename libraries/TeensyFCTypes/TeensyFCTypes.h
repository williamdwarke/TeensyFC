#ifndef _TEENSY_FC_TYPES_H_
#define _TEENSY_FC_TYPES_H_

typedef struct {
    double x;
    double y;
    double z;
} VECTOR;

enum fcModeVal {
    RATE_MODE = 0,
    ATT_MODE
};

enum rcTypeVal {
    IBUS = 0,
    SBUS
};

typedef struct {
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint16_t aux4;
} RCCOMMAND;

#endif