/*
 * @Author: copy
 * @Date: 2023-01-19
 * @Description: �˲����㷨(����)
 * @Vision: V1.0
 */
#ifndef _BASICFILTER_H
#define _BASICFILTER_H
#include "zf_common_headfile.h"

#define position_time_resolution 0.001  //���ʱ��
#define filter_length 8          //�����С 
#define filter_length_X 4        //�����С
#define filter_length_Y 4        //�����С

//���·���ÿһ����������ֻ࣬�ǲ�ͬ����Ϲ��ɲ�ͬ���˲���
float meanMovingWindow(float current_value, uint8_t window_size);
float meanMovingWindow_X(float current_value, uint8_t window_size);
float meanMovingWindow_Y(float current_value, uint8_t window_size);

float hullMovingWindow(float current_value);
float hullMovingWindow_X(float current_value);
float hullMovingWindow_Y(float current_value);

float highPassFIRFilter(float current_value);
float lowPassFIRFilter(float current_value);

float kalmanFilter_X(float prolonged_credibility, float temporary_credibility);
float kalmanFilter_Y(float prolonged_credibility, float temporary_credibility);
float kalmanFilterHomogeneous(float prolonged_credibility, float temporary_credibility);
float kalman_gyro_enc_combine(float gyro_speed, float enc_speed);
float kalman_single_object(float input);

#define F_COUNT 1024
#define MM 20

float *LMS_Filter(int itr, const float *xn, const float *dn, double mu, int length);

#endif