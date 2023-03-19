#ifndef _SELF_MAIN_H
#define _SELF_MAIN_H

#include "zf_common_headfile.h"
#define delta_T    0.1f  //计算间隔这个跟position_time_resolution是一样的，所以后面location的时候就不需要在乘以position_time_resolution
#define object_num_limit 18
#define FLT_MIN_SELF 0.000001
/* 系数：升序 */
#define fit_rder  1 /* 拟合阶数 */

/*我认为这个是有所必要的，应为全局的一个判断标志是需要在main.c中进行修改的*/
typedef struct
{
    bool start_up_flag;
    uint8_t receive_coordinate_information; //开始接收目标位置信息
    uint8_t run_status_flag;                //直行标志
    int8_t speed_control_flag;              //调速标志,1为加速，-1为减速
    uint8_t imufilterdriftinit;             //imu零飘初始化标志
    uint8_t integralDriftClearanceFlag;     //零漂积分清除标志
    uint8_t radian_adjustment_flag;         //弧度调整标志
    bool allowRecognitionFlag;              //允许Art开始识别标志
    uint8_t carry_flag;                     //捕捉标志
    uint8_t position_adjust_flag;           //微调标志
    uint8_t body_rotation_flag;             //旋转标志
    uint8_t missing_target_flag;            //丢失目标标志
    bool identify_complete_flag;            //识别完成标志
    uint8_t mandatory_parked;               //识别出框强制停车
    bool enRadianCorrection_flag;           //允许角度修正
    uint8_t stop_flag;                      //停车标志
    // GrabState grab_state_flag;           //抓取状态判断
    bool wait_recapping_flag;               //释放舵机，等待重新捕捉
} StateFlags;                               /* grab_state_flag\carry_flag 可合并 */
//在全局中定义更方便控制
extern StateFlags state_flags;

typedef struct
{
    /* data */
    float current_coord_x;
    float current_coord_y;
    float before_self_coord_x;
    float before_self_coord_y;
    float target_coord_x;
    float target_coord_y;
    float ahead_coord_x;
    float ahead_coord_y;
    float boundary_location_x;
    float boundary_location_y;
    float expected_adjustment_coord_x;
    float expected_adjustment_coord_y;
    float before_position_coord_x;
    float before_position_coord_y;
    float carry_distance;
}Coord;

typedef struct 
{
    float radian_present_error;
    float radian_before_error;
    float radian_sum_error;
    float bodywork_Kp;
    float bodywork_Ki;
    float bodywork_Kd;
} radian_PID;

typedef struct
{
    float car_yaw_radian;         /*用脖子想想一下，车子转向实际上只用到了yaw角度*/
    float before_self_yaw_radian;
    //float target_relative_radian; /* 目标位置与车前行方向弧度差，目标相对车更靠近x轴为负值 */
    //float toBeCorrectedSlope;     /* 边界修正角度 */
                                  /* 是不是要修正代码采取不转向,保持车身姿态的方法*/
    float target_point_radian;    /* 旋转目标角度 */

    float position_adjust_radian;
    radian_PID radian_pid;
} Stance;

typedef struct
{
    float Half_length;
    float Half_width;
    float wheel_distance;
    double Encoder2Velocity;
} CarBodySize;

typedef struct
{
    uint8_t field_view_height;
    uint8_t field_view_width;
} LocalImageSize; /* 截取处理视场范围 */


/* PID参数 */
typedef struct
{
    float Kp, Ki, Kd;
    int32_t set_value;     //set_value = left_front.encode_value_goal/ car_body_size.Encoder2Velocity;
    float set_error_value; //用于改进set_encode_data,也就是改进速度
    int32_t present_value; //计划输出量-当前检测量
    float present_error, before_error, b_before_error;
} PID;

/* 信息存储 */
typedef struct
{
    int32_t encode_value_goal;
    uint8_t dir;
    int32_t set_encode_data;
    int32_t get_encode_data;

    float   wheel_speed;
    float   target_wheel_speed;
    
    PID pid;
} Motor_info;

/* 四个轮子 */
typedef enum
{
    Motor_left_front,
    Motor_right_front,
    Motor_left_back,
    Motor_right_back
} Motor;

/* 编码器 */
typedef enum
{
    Encoder_left_front,  /* C11\C27 */
    Encoder_right_front, /* C9\C10 */
    Encoder_left_back,   /* C4\C5 */
    Encoder_right_back   /* C6\C7 */
} Encoder;

typedef struct 
{
    float pitch;    //俯仰角
    float roll;     //偏航角
    float yaw;       //翻滚角
} euler_param_t;


typedef struct
{
    lpuart_handle_t comm_lpuartHandle;
    lpuart_transfer_t comm_transfer;
    uint16 comm_URecv_Index;
    uint8 comm_URecv[256];
    uint8 comm_buff;
} Comm;

typedef struct
{
    uint8_t H_canny_th;
    uint8_t L_canny_th;
    uint8_t Local_th;
    uint8_t response_value;
    uint8_t Binary_th;
} BinaryThreshold;

/// @brief 
typedef struct
{
    float root_coord[4][2]; /* 交点坐标 左上 右上 左下 右下*/
    float coefficient_X_up[fit_rder + 1];
    float coefficient_X_down[fit_rder + 1];
    float coefficient_Y_left[fit_rder + 1];
    float coefficient_Y_right[fit_rder + 1];
} Equation;





#endif