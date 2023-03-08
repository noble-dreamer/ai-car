/*
 * @Author: lzx
 * @Date: 2023-01-05
 * @Description: IMU的互补滤波算法得到欧拉角
 * @Vision: V1.0
 */
#ifndef _IMUFILTER_H
#define _IMUFILTER_H
#include "zf_common_headfile.h"

typedef struct{
    float acc_x;

    float acc_y;

    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
} imu_param_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;

/// @brief 陀螺仪值
typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_param_t;



void gyro_offset_init(void);

float fast_sqrt_InvSqrt(float x);

float fast_sqrt(float x);
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx,float my,float mz);

void IMU_getValues();

void IMU_getEulerianAngles(void);

#endif