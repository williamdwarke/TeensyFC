#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <TeensyESC.h>

static const int pwmMin = 1000, pwmIdle = 1050, pwmMax = 2000;

extern TeensyESC motor1, motor2, motor3, motor4;
extern int motorCommands[4];

void initMotors();
void updateMotors();
void updateMotorCommands(uint16_t throttle, double x, double y, double z);
void startMotors();
void stopMotors();

#endif