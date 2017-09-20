#ifndef _CONFIG_H_
#define _CONFIG_H_

//Pinout
#define IMU_SS_PIN  10
#define IMU_INT_PIN 9
#define MOTOR_1_PIN 23
#define MOTOR_2_PIN 2
#define MOTOR_3_PIN 22
#define MOTOR_4_PIN 3

#define IBUS_SERIAL Serial1

#define RATE_LIMIT_DPS  720
#define ATT_LIMIT_DEG   30

//Modes
#define MODE_ATT_LOWER  1000
#define MODE_ATT_UPPER  1500

#define MADGWICK_BETA   0.75

#define RCFS_THROTTLE   1200

#endif