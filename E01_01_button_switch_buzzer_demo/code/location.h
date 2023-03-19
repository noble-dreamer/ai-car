/*
 * @Author: lzx
 * @Date: 2023-01-17 
 * @Description: ���嶨λ�㷨
 * @Vision: V1.0
 */
#ifndef _LOCATION_H
#define _LOCATION_H
#include "zf_common_headfile.h"

//���سߴ糵cm

#define Venue_Length 700
#define Venue_Width 500
#define block_size 20 //ÿһ����Ĵ�С
#define Gravity_Acc 9.81
#define M_PI       3.14159f
#define AheadDist 20 //ǰ������
#define XEndPoint 22.5  //X,Y�Ľ���λ�õ�
#define YEndPoint -32.5 //X,Y�Ľ���λ�õ�

struct location
{
    /* data */
    StateFlags *state_flags;
    Coord *coord;
    Stance *stance;
    euler_param_t *eulerAngle;
    Motor_info *left_front ;
    Motor_info *right_front;
    Motor_info *left_back  ;
    Motor_info *right_back ;
    CarBodySize *car_body_size;
    imu_param_t *imu_data;

    float (*target_coord)[object_num_limit];
    uint8_t (*target_lable)[12];
    uint16_t (*city_result);
    uint8_t *object_num;
    int8_t *current_obj_index;
    float *compensate_radian;
    bool *target_update;
    uint8_t *A4_select;
    uint8_t *Carry;

    void (*location_Constructor)(struct location *this);
    void (*location_Destructor)(struct location *this);
    float(*encoder_to_radian)(struct location *this);
    float(*location_solve_inverse_movement_rotation_speed)(struct location *this);//motor�����Ѿ�����
    float(*location_solve_gyro_rotation_speed)(struct location *this);
    float(*gyroMixFilter)(struct location *this);
    float*(*accMixFilter)(struct location *this);
    void (*regularRadians)(float *radian);
    void (*radiansGet)(struct location *this);
    void (*positioningSystemInit)(struct location *this, uint8_t num);
    void (*coordChange)(struct location *this);
    void (*nextRecentTargetDiscrimination)(struct location *this);
    uint8_t (*currentStageCompleteJudgment)(struct location *this);
    void (*positioningSystemChange)(struct location *this);
    
};

void location_Constructor(struct location *this);
void location_Destructor(struct location *this);
float location_solve_inverse_movement_rotation_speed(struct location *this);
float location_solve_gyro_rotation_speed(struct location *this);
float gyroMixFilter(struct location *this);
float *accMixFilter(struct location *this);
void regularRadians(float *radian);
void radiansGet(struct location *this);
void positioningSystemInit(struct location *this, uint8_t num);
void coordChange(struct location *this);
void nextRecentTargetDiscrimination(struct location *this);
uint8_t currentStageCompleteJudgment(struct location *this);
void positioningSystemChange(struct location *this);



void location_Init(struct location *this);
#endif