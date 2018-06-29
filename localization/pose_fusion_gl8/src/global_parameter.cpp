#include "../include/global_parameter.h"
#include <cmath>
double IMU_W_ERR = 0.0009; //rad velocity of IMU, unit: rad/s
double MAPFRAME_OBS_ERR_FIX = 0.008; //gps error, fix type: 4, unit: m
double MAPFRAME_OBS_ERR_FLOAT = 1.0; //gps error, fix type: 5, unit: m
double MAPFRAME_OBS_ERR_SINGLE = 3.0; //gps error, fix type: 1, unit: m
double REAR_WHEEL_ENCODER_ERR = 0.01; // unit: m/m
double OBS_YAW_ERR_FIX = 0.2 * M_PI / 180.0; //from gps, unit: rad
double SCALE = cos(31.03 * M_PI / 180.0);
