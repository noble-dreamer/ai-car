/*
 * @Author: lzx
 * @Date: 2023-02-04 18:00:07
 * @Description: ��������
 * @Vision: V1.0
 */
#ifndef _SOLVEHELPER_H
#define _SOLVEHELPER_H

#include "zf_common_headfile.h"
/* ������ʽ */
float FirstOrderLeastSquare(uint8_t *X_array, uint8_t *Y_array, uint8_t point_num, float *coefficient);
float Polyfit2(const float *x, const float *y, int n, float *coefficient);
void SecondOrderLeastSquare(uint8_t *X_array, uint8_t *Y_array, uint8_t point_num, float *coefficient);
void solve_quadratic_equation(float *coefficient, float *untie, uint8_t *root_count);
/* ��Ҫ���� */
uint8_t solveMatrixVertices(uint8_t enSlope);

#endif