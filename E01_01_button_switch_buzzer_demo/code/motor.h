/*
 * @Author: lzx
 * @Date: 2023-01-17 
 * @Description: 电机控制算法，包括编码器
 * @Vision: V1.0
 */
#ifndef _MOTOR_H
#define _MOTOR_H


#define ENCODER_1                   (QTIMER1_ENCOEDER1)
#define ENCODER_1_A                 (QTIMER1_ENCOEDER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCOEDER1_CH2_C1)

#define ENCODER_2                   (QTIMER1_ENCOEDER2)
#define ENCODER_2_A                 (QTIMER1_ENCOEDER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCOEDER2_CH2_C24)

#define ENCODER_3                   (QTIMER2_ENCOEDER1)
#define ENCODER_3_A                 (QTIMER2_ENCOEDER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCOEDER1_CH2_C25)

#define ENCODER_4                   (QTIMER3_ENCOEDER2)
#define ENCODER_4_A                 (QTIMER3_ENCOEDER2_CH1_B18)
#define ENCODER_4_B                 (QTIMER3_ENCOEDER2_CH2_B19)

#define Motor_left_front_pwm PWM1_MODULE2_CHA_D16
#define Motor_left_front_dir D17
#define Motor_right_front_pwm PWM1_MODULE1_CHA_D14
#define Motor_right_front_dir D15
#define Motor_left_back_pwm PWM1_MODULE3_CHA_D0
#define Motor_left_back_dir D1
#define Motor_right_back_pwm PWM1_MODULE0_CHA_D12
#define Motor_right_back_dir D13

#define IncreaseOrder 5
#define DecreaseOrder 3

#endif