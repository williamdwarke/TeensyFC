/*
 * ini.hpp
 *
 *  Created on: May 29, 2015
 *      Author: wwarke
 */

#ifndef TEENSY_INI_H_
#define TEENSY_INI_H_

/* Constants */
static const char *iniFilename 	= "flightConfig.ini";
static const char *bakFilename  = "flightConfig.bak";

static const char *iniSections[2] = {"calibration", "pid"};

/* Settings storage class */
class TeensyINI {
public:
    //Calibration
    bool esc_cal_flag;
    bool acc_calibrated;
    bool mag_calibrated;

    double acc_offset_x, acc_offset_y, acc_offset_z;
    double acc_scale_x, acc_scale_y, acc_scale_z;
    double mag_offset_x, mag_offset_y, mag_offset_z;

	TeensyINI();
    int init();
    void loadFile();
    void saveFile();
	void loadDefaults();
	int createFile();
	void readConfig();
	int writeConfig();
    void printConfig(bool printSerial = false);
};

#endif /* TEENSY_INI_H_ */

/*
SerialUSB.println("Acc offsets: " + String(imu.accOffset.x, 10) + ", " + String(imu.accOffset.y, 10) + ", " + String(imu.accOffset.z, 10));
SerialUSB.println("Acc scales: " + String(imu.accScale.x, 10) + ", " + String(imu.accScale.y, 10) + ", " + String(imu.accScale.z, 10));
SerialUSB.println("Mag offsets: " + String(imu.magOffset.x, 10) + ", " + String(imu.magOffset.y, 10) + ", " + String(imu.magOffset.z, 10));
*/