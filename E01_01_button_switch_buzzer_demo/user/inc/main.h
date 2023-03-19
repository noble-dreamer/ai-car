#ifndef _SELF_MAIN_H
#define _SELF_MAIN_H

#include "zf_common_headfile.h"
#define delta_T    0.1f  //�����������position_time_resolution��һ���ģ����Ժ���location��ʱ��Ͳ���Ҫ�ڳ���position_time_resolution
#define object_num_limit 18
#define FLT_MIN_SELF 0.000001
/* ϵ�������� */
#define fit_rder  1 /* ��Ͻ��� */

/*����Ϊ�����������Ҫ�ģ�ӦΪȫ�ֵ�һ���жϱ�־����Ҫ��main.c�н����޸ĵ�*/
typedef struct
{
    bool start_up_flag;
    uint8_t receive_coordinate_information; //��ʼ����Ŀ��λ����Ϣ
    uint8_t run_status_flag;                //ֱ�б�־
    int8_t speed_control_flag;              //���ٱ�־,1Ϊ���٣�-1Ϊ����
    uint8_t imufilterdriftinit;             //imu��Ʈ��ʼ����־
    uint8_t integralDriftClearanceFlag;     //��Ư���������־
    uint8_t radian_adjustment_flag;         //���ȵ�����־
    bool allowRecognitionFlag;              //����Art��ʼʶ���־
    uint8_t carry_flag;                     //��׽��־
    uint8_t position_adjust_flag;           //΢����־
    uint8_t body_rotation_flag;             //��ת��־
    uint8_t missing_target_flag;            //��ʧĿ���־
    bool identify_complete_flag;            //ʶ����ɱ�־
    uint8_t mandatory_parked;               //ʶ�����ǿ��ͣ��
    bool enRadianCorrection_flag;           //����Ƕ�����
    uint8_t stop_flag;                      //ͣ����־
    // GrabState grab_state_flag;           //ץȡ״̬�ж�
    bool wait_recapping_flag;               //�ͷŶ�����ȴ����²�׽
} StateFlags;                               /* grab_state_flag\carry_flag �ɺϲ� */
//��ȫ���ж�����������
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
    float car_yaw_radian;         /*�ò�������һ�£�����ת��ʵ����ֻ�õ���yaw�Ƕ�*/
    float before_self_yaw_radian;
    //float target_relative_radian; /* Ŀ��λ���복ǰ�з��򻡶ȲĿ����Գ�������x��Ϊ��ֵ */
    //float toBeCorrectedSlope;     /* �߽������Ƕ� */
                                  /* �ǲ���Ҫ���������ȡ��ת��,���ֳ�����̬�ķ���*/
    float target_point_radian;    /* ��תĿ��Ƕ� */

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
} LocalImageSize; /* ��ȡ�����ӳ���Χ */


/* PID���� */
typedef struct
{
    float Kp, Ki, Kd;
    int32_t set_value;     //set_value = left_front.encode_value_goal/ car_body_size.Encoder2Velocity;
    float set_error_value; //���ڸĽ�set_encode_data,Ҳ���ǸĽ��ٶ�
    int32_t present_value; //�ƻ������-��ǰ�����
    float present_error, before_error, b_before_error;
} PID;

/* ��Ϣ�洢 */
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

/* �ĸ����� */
typedef enum
{
    Motor_left_front,
    Motor_right_front,
    Motor_left_back,
    Motor_right_back
} Motor;

/* ������ */
typedef enum
{
    Encoder_left_front,  /* C11\C27 */
    Encoder_right_front, /* C9\C10 */
    Encoder_left_back,   /* C4\C5 */
    Encoder_right_back   /* C6\C7 */
} Encoder;

typedef struct 
{
    float pitch;    //������
    float roll;     //ƫ����
    float yaw;       //������
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
    float root_coord[4][2]; /* �������� ���� ���� ���� ����*/
    float coefficient_X_up[fit_rder + 1];
    float coefficient_X_down[fit_rder + 1];
    float coefficient_Y_left[fit_rder + 1];
    float coefficient_Y_right[fit_rder + 1];
} Equation;





#endif