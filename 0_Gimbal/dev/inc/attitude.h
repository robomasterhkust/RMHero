#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_

#include "adis16470.h"

#define ATT_W_ACCEL     0.4f
#define ATT_W_GYRO      0.2f
#define GYRO_BIAS_MAX   0.05f

#define ATTITUDE_UPDATE_FREQ 1000U
#define ATTITUDE_USE_ADIS16470_TIMESTAMP

uint8_t attitude_imu_init(PIMUStruct pIMU);
uint8_t attitude_update(PIMUStruct pIMU);
void attitude_update_timestamp(uint32_t timestamp);

#endif
