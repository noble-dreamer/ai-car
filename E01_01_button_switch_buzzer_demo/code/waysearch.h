/*
 * @Author: lzx
 * @Date: 2023-01-17 18:00:07
 * @Description: ·�������㷨
 * @Vision: V1.0
 */
#ifndef _WAYSEARCH_H
#define _WAYSEARCH_H


#include "zf_common_headfile.h"

#define STATIC_ARRAY_SIZE 24
extern Coord coord;
extern int globalTour[STATIC_ARRAY_SIZE][2];
// N : number of city point
// bestDistanceLength : best distance length
// return : 1 is success
// ���ھֲ���Ϣ�ظ��µ���Ⱥ�㷨��is better than before
int Ant_findBestPath(int N, float* bestDistanceLength);

//��һ����ϸ���㷨Ϊģ���˻��㷨���²�Ч��������Ⱥ,���ս�������city_result��
void annealingDeterminesPath();


#endif