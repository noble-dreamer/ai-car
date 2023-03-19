/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库移植用空工程

StateFlags state_flags = {0};
euler_param_t eulerAngle;
Coord coord = {0};
Stance stance = {0};
uint8_t target_lable[2][12] = {{0}, {0}};//用于表示类别

CarBodySize car_body_size = {0};
LocalImageSize local_image_size = {0};

Motor_info left_front = {0};
Motor_info right_front = {0};
Motor_info left_back = {0};
Motor_info right_back = {0};

uint16_t city_result[object_num_limit] = {0}; //这里也是存放每张图片，用来索引
float target_coord[2][object_num_limit] = {{0}, {0}}; //城市坐标，记录实际距离
/*用于存放全局的移动,这次去哪,下次去哪*/
int globalTour[STATIC_ARRAY_SIZE][2];

int8_t current_obj_index = -1;  //用于location，表示当前进程
uint8_t object_num = 0;         //用于location,表示总数
float compensate_radian = 0;    //用于location
uint8_t A4_select = 0;
uint8_t Carry = 0;

/*
    车体速度的基变量。
*/

float straight_speed_x = 0;
float straight_speed_y = 0;
float target_straight_speed_x = 0;
float target_straight_speed_y = 0;

float rotation_speed = 0;
float target_rotation_speed = 0;
float carry_speed = 0;
float mProport = 0;

int8_t rotation_direction = 0;

float deviation_radian, pan_direction = 5000, scale_factor = 0;
float test_car_x = 0, test_car_y = 0;
float growthRateMultiplier = -1; //增长比例乘数
int8_t counterclockwiseDetection = 0;

/*
    图像变换的基变量
*/
//[上下左右][双向搜索][索引]
#if MT9V03X_H > MT9V03X_W
uint16_t Egde[4][2][MT9V03X_H / 2];
#else
/*
//可以描绘出整个半边,[上下左右][两个侧向][所有半长]
  来源为image_one_dimension[]的index
*/
uint16_t Egde[4][2][MT9V03X_W / 2];
#endif

uint16_t A4_center = Center;
uint16_t demarcate_center;
uint16_t edge_size_limit;
// uint16 image_one_dimension[MT9V03X_H * MT9V03X_W];
// uint16 (*image_two_dimension)[MT9V03X_W] = (uint16 (*)[MT9V03X_W]) image_one_dimension;此方法可以将二维数组和一维数组联系起来
uint8_t image_one_dimension[MT9V03X_IMAGE_SIZE] = {0};
BinaryThreshold binary_threshold = {0};
Equation equation = {0};







/*
这里放置所有基变量,包括location,motor等
*/
extern struct motorcontrol MotorControl;
extern struct motorcontrol *thisMotorControl;
extern struct location car_location;
extern struct location *thiscar_location;

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化

    // 此处编写用户代码 例如外设初始化代码等
	test_encoder_get();
    //test_imgprocess();
    // 此处编写用户代码 例如外设初始化代码等
    //while(1)
    //{
    //    // 此处编写需要循环执行的代码
    //    
    //    // 此处编写需要循环执行的代码
    //}
	return 0;
}




