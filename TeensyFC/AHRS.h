#ifndef _AHRS_H_
#define _AHRS_H_

//Declination in degrees in Shalimar, FL
#define DECLINATION 3.183333

double radToDeg(double rad);
double degToRad(double deg);
double invSqrt(double x);
double getVectorMagnitude(double x, double y, double z);
void eulerToQuaternion();
void quaternionToEuler();
void normalizeQuaternion();
void initializeQuaternion(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);

void MadgwickAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz, double dt);
void MadgwickAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az, double dt);

extern double q0, q1, q2, q3;
extern VECTOR degAtt, radAtt;
//extern double radRoll, radPitch, radYaw;
//extern double degRoll, degPitch, degYaw;

#endif