/*
 * @Author: lzx
 * @Date: 2023-01-17 
 * @Description: ͼ�����㷨,����canny,�����д�����Ϣͨ��usart���
 * @Vision: V1.0
 */
#ifndef _IMG_PROCESS_H
#define _IMG_PROCESS_H
#include "zf_common_headfile.h"
#define KERNEL_SIZE 3
#define Sigma 0.8
#define object_num_limit 18
#define OBJNUM 18           //ʵ��Ŀ������,����������ص�ԭ��ʱ��Ŀ������+1
#define target_size 20

#define MAX_GRAY_LEVEL 256

#define edges_difference_degree 5

#define white_points_limitation object_num_limit * 100
#define Center MT9V03X_W *MT9V03X_H / 2 - MT9V03X_W / 2

#define local_step 8
#define local_dimensions 12

/*�ⶨ��Ҫ�ķ�ʽ�и�˹ģ��,���,sobal�˲�,candy��Ե��ȡ*/
void generateGaussianKernel(float *kernel, int size, float sigma);
void gaussianBlur(uint8 (*image)[MT9V03X_W], uint8 height, uint8 width, int size, float sigma);
/*���ٸ�˹ģ��*/
void fastGaussianBlur(uint8 (*image)[MT9V03X_W], uint8 height, uint8 width);

uint8 otsuThreshold(uint8 (*image)[MT9V03X_W], uint8 height, uint8 width);
void sobelEdgeDetection(uint8 (*image)[MT9V03X_W], float *a_array,uint8 width, uint8 height);
void nonMaximalSuppression(uint8 (*image)[MT9V03X_W], float *a_array,uint8 width, uint8 height);
void doubleThresholding(uint8 (*image)[MT9V03X_W], uint8 (*temp)[MT9V03X_W],uint8 width, uint8 height, int lowThreshold, int highThreshold);
void cannyEdgeDetection(uint8 (*image)[MT9V03X_W], uint8 width, uint8 height, float sigma);

/* �˱�Ե��ⷨ�����ϰ취,��ȡ�������������, */
uint8_t UpEdgeExtraction(uint8_t *r_array);
uint8_t DownEdgeExtraction(uint8_t *r_array);
uint8_t LeftEdgeExtraction(uint8_t *r_array);
uint8_t RightEdgeExtraction(uint8_t *r_array);

int8_t adjustEdgeLimit(uint16_t *check_point, uint8_t *X_index, uint8_t *Y_index, uint8_t up_limit, uint8_t down_limit, uint8_t Advance_amount);
uint8_t clusterInitialization(uint8_t (*edge_point_coord)[white_points_limitation], float min_distance, float max_distance, uint8_t *theoretical_detection_targets_count);
uint8_t kmeans(uint8_t *target_num, uint8_t (*edge_point_coord)[white_points_limitation], uint8_t size_restriction);
void absolute2RelativeCoordinates(uint8_t target_num);

uint8_t A4checktarget(uint8_t *r_array);

#endif