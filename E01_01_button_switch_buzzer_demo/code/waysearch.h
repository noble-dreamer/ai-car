/*
 * @Author: lzx
 * @Date: 2023-01-17 18:00:07
 * @Description: 路径搜索算法
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
// 基于局部信息素更新的蚁群算法，is better than before
int Ant_findBestPath(int N, float* bestDistanceLength);

//另一种详细的算法为模拟退火算法，猜测效果不如蚁群,最终结果存放在city_result中
void annealingDeterminesPath();


#endif