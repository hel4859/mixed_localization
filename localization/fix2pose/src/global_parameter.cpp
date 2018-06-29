#include "../include/global_parameter.h"
double IMU_W_ERR = 0.0009; //rad velocity of IMU, unit: rad/s
double MAPFRAME_OBS_ERR = 0.3; //gps error, unit: m
double REAR_WHEEL_ENCODER_ERR = 0.01; // unit: m/m
double IMU_YAW_ERR = 0.122; //imu orientation, unit: rad
double OBS_YAW_ERR = 0.05; //from gps, unit: rad
double SCALE = cos(31.03*M_PI/180.0);
