/*
 * calibration.h
 *
 *  Created on: 11.01.2021
 *      Author: symon
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}ThreeXYZ;

ThreeXYZ gyro_calibration();
ThreeXYZ acc_calibration();
#endif /* CALIBRATION_H_ */
