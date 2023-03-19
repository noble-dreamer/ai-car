/*
 * @Author: lzx
 * @Date: 2023-01-17 
 * @Description: 电机控制算法，包括编码器
 * @Vision: V1.0
 */
//计量单位统一使用cm
//速度表示为cm/s

#ifndef _MOTOR_H
#define _MOTOR_H
#include "zf_common_headfile.h"
#define Origin_Radian               M_PI/2

#define ENCODER_1                   (QTIMER1_ENCOEDER1)             //left_back
#define ENCODER_1_A                 (QTIMER1_ENCOEDER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCOEDER1_CH2_C1)

#define ENCODER_2                   (QTIMER1_ENCOEDER2)             //right_back
#define ENCODER_2_A                 (QTIMER1_ENCOEDER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCOEDER2_CH2_C24)

#define ENCODER_3                   (QTIMER2_ENCOEDER1)             //right_front
#define ENCODER_3_A                 (QTIMER2_ENCOEDER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCOEDER1_CH2_C25)

#define ENCODER_4                   (QTIMER3_ENCOEDER2)             //left_front
#define ENCODER_4_A                 (QTIMER3_ENCOEDER2_CH1_B18)
#define ENCODER_4_B                 (QTIMER3_ENCOEDER2_CH2_B19)

#define Motor_left_back_pwm PWM2_MODULE3_CHA_D2
#define Motor_left_back_dir D0
#define Motor_right_back_pwm PWM2_MODULE3_CHB_D3
#define Motor_right_back_dir D1
#define Motor_right_front_pwm PWM1_MODULE1_CHB_D15
#define Motor_right_front_dir D13
#define Motor_left_front_pwm PWM1_MODULE1_CHA_D14
#define Motor_left_front_dir D12

#define IncreaseOrder 5
#define DecreaseOrder 3

#define encoder2wheel_speed_K   1000

#define wheel_diameter          6                   //轮子直径

#define wheel_circumference     wheel_diameter*M_PI


struct motorcontrol
{
    /* data */
    StateFlags *state_flags;
    Coord *coord;
    Stance *stance;
    Motor_info *left_front ;
    Motor_info *right_front;
    Motor_info *left_back  ;
    Motor_info *right_back ;
    CarBodySize *car_body_size;

    float (*target_coord)[object_num_limit];

    float *straight_speed_x;
    float *straight_speed_y;
    float *target_straight_speed_x;
    float *target_straight_speed_y;
    float *rotation_speed;
    float *target_rotation_speed;

    float *carry_speed;
    float *mProport;

    //int8_t *rotation_direction;
    //float *deviation_radian;
    //float *pan_direction;
    //float *scale_factor;
    //float *test_car_x;
    //float *test_car_y;
    //float *growthRateMultiplier;
    //int8_t *counterclockwiseDetection;

    void (*motorcontrol_Constructor)(struct motorcontrol *this);
    void (*EncoderGet)(struct motorcontrol *this);
    void (*PIDParameterInit)(struct motorcontrol *this);

    void (*signalMotorControl)(struct motorcontrol *this, Motor motor, uint8_t dir, uint16_t encode_value_goal); //所有步骤都需要这个来改变电机的PWM
    //void (*carRunControl)(struct motorcontrol *this, float radian /* -pi~pi */, uint16_t speed);
    void (*encode_value_goal2pwm)(struct motorcontrol *this);
	void (*multiplier)(struct motorcontrol *this, float proport);    
	void (*motorIncrementalPID)(struct motorcontrol *this);
    void (*inverse_target_wheel_speed)(struct motorcontrol *this);
	void (*inversewheel_speed)(struct motorcontrol *this);
    void (*wheel_speed2rotation_speed)(struct motorcontrol *this);
    void (*wheel_speed2x_y_speed)(struct motorcontrol *this);
	void (*encoder2wheel_speed)(struct motorcontrol *this, Motor motor);
	void (*target_wheel_speed2encoder)(struct motorcontrol *this, Motor motor);
    void (*radian_PID_init)(struct motorcontrol *this);
    void (*radianBodyworkPID)(struct motorcontrol *this);
	void (*carspeedcontrol_radian)(struct motorcontrol *this);	
};

void motorcontrol_Constructor(struct motorcontrol *this);
void encoderInit();
void motorInit();
void motorcontrol_Init(struct motorcontrol *this);

void signalMotorControl(struct motorcontrol *this, Motor motor, uint8_t dir, uint16_t encode_value_goal);
void encode_value_goal2pwm(struct motorcontrol *this);
void multiplier(struct motorcontrol *this, float proport);

void motorIncrementalPID(struct motorcontrol *this);
void inverse_target_wheel_speed(struct motorcontrol *this);
void inversewheel_speed(struct motorcontrol *this);
void wheel_speed2rotation_speed(struct motorcontrol *this);
void wheel_speed2x_y_speed(struct motorcontrol *this);
void encoder2wheel_speed(struct motorcontrol *this, Motor motor);
void target_wheel_speed2encoder(struct motorcontrol *this, Motor motor);
void radian_PID_init(struct motorcontrol *this);
void radianBodywordPID(struct motorcontrol *this);

void carspeedcontrol_radian(struct motorcontrol *this);

#endif