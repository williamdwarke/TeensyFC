#include <Arduino.h>

#include "Config.h"
#include "Motors.h"

TeensyESC motor1, motor2, motor3, motor4;

//Todo: dynamic mix table
int motorMixTable[4][3] = {
    {-1, -1, -1},
    {-1,  1,  1},
    { 1, -1,  1},
    { 1,  1, -1}
};
int motorCommands[4];

void initMotors() {
    motor1.attach(MOTOR_1_PIN);
    motor2.attach(MOTOR_2_PIN);
    motor3.attach(MOTOR_3_PIN);
    motor4.attach(MOTOR_4_PIN);

    //Todo: Motor Cal

    motor1.writeMicroseconds(pwmMin);
    motor2.writeMicroseconds(pwmMin);
    motor3.writeMicroseconds(pwmMin);
    motor4.writeMicroseconds(pwmMin);
}

void updateMotors() {
    motor1.writeMicroseconds(motorCommands[0]);
    motor2.writeMicroseconds(motorCommands[1]);
    motor3.writeMicroseconds(motorCommands[2]);
    motor4.writeMicroseconds(motorCommands[3]);
}

void updateMotorCommands(uint16_t throttle, double x, double y, double z) {
    throttle = constrain(throttle, pwmIdle, pwmMax - 150);
    for (int i = 0; i < 4; i++) {
        motorCommands[i] = constrain(throttle + motorMixTable[i][0] * x + motorMixTable[i][1] * y + motorMixTable[i][2] * z, pwmIdle, pwmMax);
    }
    updateMotors();
}

void startMotors() {
    for (int i = 0; i < 4; i++) {
        motorCommands[i] = pwmIdle;
    }
    updateMotors();
}

void stopMotors() {
    for (int i = 0; i < 4; i++) {
        motorCommands[i] = pwmMin;
    }
    updateMotors();
}