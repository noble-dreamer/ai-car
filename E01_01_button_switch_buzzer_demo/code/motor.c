#include "motor.h"
/*
    ��ľ���ƶ�����:
    ��ǰ��ʻ    lf +, rf + , lb + , rb+ 
    ������ʻ    lf +, rf - , lb - , rb+
    ������ʻ    lf -, rf + , lb + , rb-
    ˳ʱ����ת  lf +, rf - , lb + , rb-  abs(v) = w*(this->car_body_size.half+this->car_body_size.half)
    ��ʱ����ת  lf -, rf + , lb - , rb+

    �������복��ת��,�ͳ����ٶ�֮�����һ��������ϵ
    PID ʹ�õ����ٶȿ���
    �趨ʱ��ʹ�õı�������ֵ,ע��ת��

*/

//����ʱ����ʱ����ͬʱ����,���Ի���ƫ��,���Ը���������
//Ȼ�������㳵ģ�Ƕ�,����������ģ��̬����ʼ�ճ�����ǰ
//deta_T 0.05s
extern StateFlags state_flags;
extern Coord coord;
extern Stance stance;
extern euler_param_t eulerAngle;
extern CarBodySize car_body_size;
extern imu_param_t imu_data;

extern Motor_info left_front;
extern Motor_info right_front;
extern Motor_info left_back;
extern Motor_info right_back;
extern float target_straight_speed_x;
extern float target_straight_speed_y;
/*
Ŀ���ַ
*/
extern float target_coord[2][object_num_limit];
extern uint8_t target_lable[2][12];
extern uint16_t city_result[object_num_limit];
extern uint8_t object_num;
extern int8_t current_obj_index;
extern float compensate_radian;
extern uint8_t A4_select;
extern uint8_t Carry;
extern bool target_update;

/*
    �����ٶȵĻ�������
*/
extern float straight_speed_x;
extern float straight_speed_y;
extern float rotation_speed;
extern float target_rotation_speed;
extern float carry_speed;
extern float mProport;
//extern uint8_t increase_adjust_count;
//extern uint8_t decrease_adjust_count;
extern int8_t rotation_direction;
extern float deviation_radian;
extern float pan_direction;
extern float scale_factor;
extern float test_car_x; 
extern float test_car_y;
extern float growthRateMultiplier; //������������
extern int8_t counterclockwiseDetection;



struct motorcontrol MotorControl;
struct motorcontrol *thisMotorControl = &MotorControl;



void motorcontrol_Constructor(struct motorcontrol *this)
{
    if (this!=0)
    {
        this->state_flags = &state_flags;
        this->coord = &coord;
        this->stance = &stance;
        this->left_front = &left_front ;
        this->right_front= &right_front;
        this->left_back  = &left_back  ;
        this->right_back = &right_back ;
        this->car_body_size = &car_body_size;

        this->target_coord = target_coord;

        this->straight_speed_x = &straight_speed_x;
        this->straight_speed_y = &straight_speed_y;
        this->target_straight_speed_x = &target_straight_speed_x;
        this->target_straight_speed_y = &target_straight_speed_y;
        this->rotation_speed = &rotation_speed;
        this->target_rotation_speed = &target_rotation_speed;

        this->carry_speed = &carry_speed;
        this->mProport = &mProport;

        //this->pan_direction = &pan_direction;
        //this->scale_factor = &scale_factor;
        //this->test_car_x = &test_car_x;
        //this->test_car_y = &test_car_y;
        //this->growthRateMultiplier = &growthRateMultiplier;
        //this->counterclockwiseDetection = &counterclockwiseDetection;

    }
    else
    {

    }
}

/**
 * @description: ���PID������ʼ��
 * @param {*}
 * @return {*}
 */
void PIDParameterInit(struct motorcontrol *this)
{

        this->left_front->pid.Kp = 1.8;
        this->left_front->pid.Ki = 0.65;
        this->left_front->pid.Kd = 0.5;
        this->right_front->pid.Kp = 1.8;
        this->right_front->pid.Ki = 0.7;
        this->right_front->pid.Kd = 0.5;
        this->left_back->pid.Kp = 1.8;
        this->left_back->pid.Ki = 0.8;
        this->left_back->pid.Kd = 0.5;
        this->right_back->pid.Kp = 1.8;
        this->right_back->pid.Ki = 0.8;
        this->right_back->pid.Kd = 0.5;
}

/**
 * @description: ��������ʼ��
 * @param {*}
 * @return {*}
 */
void encoderInit()
{
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                     // ��ʼ��������ģ�������� �������������ģʽ
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                     // ��ʼ��������ģ�������� �������������ģʽ
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B);                     // ��ʼ��������ģ�������� �������������ģʽ
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B);                     // ��ʼ��������ģ�������� �������������ģʽ
}
/**
 * @description: �����ʼ��
 * @param {*}
 * @return {*}
 */
void motorInit()
{
    gpio_init(Motor_left_front_dir, GPO, 0, GPO_PUSH_PULL); /*ԭ����������GPI_PULL_UP���������������������������Ч*/
    pwm_init(Motor_left_front_pwm, 17000, 0);
    gpio_init(Motor_right_front_dir, GPO, 0, GPO_PUSH_PULL);
    pwm_init(Motor_right_front_pwm, 17000, 0);
    gpio_init(Motor_left_back_dir, GPO, 0, GPO_PUSH_PULL);
    pwm_init(Motor_left_back_pwm, 17000, 0);
    gpio_init(Motor_right_back_dir, GPO, 0, GPO_PUSH_PULL);
    pwm_init(Motor_right_back_pwm, 17000, 0);
}

/**
 * @description: ������ֵ��ȡ��ת��Ϊ1sʱ��ı�����ֵ��
 * @param {*}
 * @return {*}
 */
void EncoderGet(struct motorcontrol *this)
{
    if (this != 0)
    {
        /* code */
    this->left_front->get_encode_data = (int16_t)(encoder_get_count(ENCODER_4) / delta_T);
    this->left_front->pid.present_value = this->left_front->get_encode_data;
    encoder_clear_count(ENCODER_4);                                             // ��ձ���������

    this->right_front->get_encode_data = 0 - (int16_t)(encoder_get_count(ENCODER_3) / delta_T);
    this->right_front->pid.present_value = this->right_front->get_encode_data;
    encoder_clear_count(ENCODER_3);                                             // ��ձ���������

    this->left_back->get_encode_data = (int16_t)(encoder_get_count(ENCODER_1)/ delta_T);
    this->left_back->pid.present_value = this->left_back->get_encode_data;
    encoder_clear_count(ENCODER_1);                                             // ��ձ���������

    this->right_back->get_encode_data = 0 - (int16_t)(encoder_get_count(ENCODER_2) / delta_T);
    this->right_back->pid.present_value = this->right_back->get_encode_data;
    encoder_clear_count(ENCODER_2);                                             // ��ձ���������
    static int vibration_count = 0;                                             //�𶯼���
   }

    else
    {
        ;
    }

}



/**
 * @description: ��������Ƹ����趨ֵ�ͷ�������ٶ�
 * @param {Motor} motor
 * @param {uint8_t} dir       +Ϊ������ǰת,-Ϊ�������ת
 * @param {uint16_t} encode_value_goal
 * @return {*}
 */
void signalMotorControl(struct motorcontrol *this, Motor motor, uint8_t dir, uint16_t encode_value_goal)
{
    if (this != 0)
    {
        /* code */
        encode_value_goal = encode_value_goal > 40000 ? 40000 : encode_value_goal;
        if (encode_value_goal == 40000)
        {
            switch (motor)
            {
            case Motor_left_front:
                this->left_front->encode_value_goal = encode_value_goal;
                break;
            case Motor_right_front:
                this->right_front->encode_value_goal = encode_value_goal;
                break;
            case Motor_left_back:
                this->left_back->encode_value_goal = encode_value_goal;
                break;
            case Motor_right_back:
                this->right_back->encode_value_goal = encode_value_goal;
                break;
            default:
                break;
            }
        }
            switch (motor)
            {
            case Motor_left_front:
                gpio_set_level(Motor_left_front_dir, dir);             //���Զ���չ,���÷���
                pwm_set_duty(Motor_left_front_pwm, encode_value_goal); //������ֵתpwm
                this->left_front->dir = dir;
                break;
            case Motor_right_front:
                gpio_set_level(Motor_right_front_dir, dir);
                pwm_set_duty(Motor_right_front_pwm, encode_value_goal);
                this->right_front->dir = dir;
                break;
            case Motor_left_back:
                gpio_set_level(Motor_left_back_dir, dir);
                pwm_set_duty(Motor_left_back_pwm, encode_value_goal);
                this->left_back->dir = dir;
                break;
            case Motor_right_back:
                gpio_set_level(Motor_right_back_dir, dir);
                pwm_set_duty(Motor_right_back_pwm, encode_value_goal);
                this->right_back->dir = dir;
                break;
            default:
                break;
            }
    }
    else
    {
        ;
    }
}


//
///**
// * @description: �����̬���ݡ�Ŀ��λ�ã����·������.
// * @param {float} radian
// * @param {uint16_t} speed
// * @return {*}
// */
//void carRunControl(struct motorcontrol *this, float radian /* -pi~pi */, uint16_t speed)
//{
//    if (this != 0)
//    {
//        this->state_flags->speed_control_flag = 0;
//        int16_t Vx, Vy;
//        Vy = (int16_t)(speed * sin(radian));
//        Vx = (int16_t)(speed * cos(radian));
//        int16_t Vlf = Vy + Vx,
//        Vrf = Vy - Vx,
//        Vlb = Vrf,
//        Vrb = Vlf;
//
//        /*
//        �ĸ����ӵ��ٶ����� 
//        */
//        this->left_front->encode_value_goal = Vlf;
//        this->right_front->encode_value_goal = Vrf;
//        this->left_back->encode_value_goal = Vlb;
//        this->right_back->encode_value_goal = Vrb;
//
//        this->left_front->encode_value_goal = Vlf;
//        this->right_front->encode_value_goal = Vrf;
//        this->left_back->encode_value_goal = Vlb;
//        this->right_back->encode_value_goal = Vrb;
//        /*��ʼ���΢���Ƕ�ʧĿ�����*/
//        /*
//        if (!this->state_flags->position_adjust_flag && !this->state_flags->missing_target_flag && this->state_flags->start_up_flag)
//        {
//            *this->increase_adjust_count = IncreaseOrder;
//            *this->decrease_adjust_count = DecreaseOrder;
//            this->state_flags->speed_control_flag = 1;
//            *this->mProport = 2.5f;
//            this->multiplier(this,1 / *(this->mProport));
//            pit_ms_init(PIT_CH2, 100);
//        }
//        */
//        if (Vlf < 0)
//        {
//            this->signalMotorControl(this,Motor_left_front, 0, 0 - left_front.encode_value_goal);
//        }
//        else
//        {
//            this->signalMotorControl(this,Motor_left_front, 1, left_front.encode_value_goal);
//        }
//        if (Vrf < 0)
//        {
//            this->signalMotorControl(this,Motor_right_front, 0, 0 - right_front.encode_value_goal);
//        }
//        else
//        {
//            this->signalMotorControl(this,Motor_right_front, 1, right_front.encode_value_goal);
//        }
//        if (Vlb < 0)
//        {
//            this->signalMotorControl(this,Motor_left_back, 0, 0 - left_back.encode_value_goal);
//        }
//        else
//        {
//            this->signalMotorControl(this,Motor_left_back, 1, left_back.encode_value_goal);
//        }
//        if (Vrb < 0)
//        {
//            this->signalMotorControl(this,Motor_right_back, 0, 0 - right_back.encode_value_goal);
//        }
//        else
//        {
//            this->signalMotorControl(this,Motor_right_back, 1, right_back.encode_value_goal);
//        }
//        this->state_flags->run_status_flag =1;
//        this->state_flags->stop_flag = 0;
//    }
//    else
//    {
//        ;
//    }
//    
//}
//���һ����������ת��ΪPWM�Ĺ���
void encode_value_goal2pwm(struct motorcontrol *this)
{
    if (this->left_front->target_wheel_speed > 0)
    {
        this->signalMotorControl(this,Motor_left_front, 1, this->left_front->encode_value_goal);
    }
    else
    {
        this->signalMotorControl(this,Motor_left_front, 0, this->left_front->encode_value_goal);
    }

    if (this->right_front->target_wheel_speed > 0)
    {
        this->signalMotorControl(this,Motor_right_front, 1, this->right_front->encode_value_goal);
    }
    else
    {
        this->signalMotorControl(this,Motor_right_front, 0, this->right_front->encode_value_goal);
    }

    if (this->left_back->target_wheel_speed > 0)
    {
        this->signalMotorControl(this,Motor_left_back, 1, this->left_back->encode_value_goal);
    }
    else
    {
        this->signalMotorControl(this,Motor_left_back, 0, this->left_back->encode_value_goal);
    }

    if (this->right_back->target_wheel_speed > 0)
    {
        this->signalMotorControl(this,Motor_right_back,1, this->right_back->encode_value_goal);
    }
    else
    {
        this->signalMotorControl(this,Motor_right_back,0, this->right_back->encode_value_goal);
    }

}
/**
 * @description: ��������,���PID����
 * @param {*}
 * @return {*}
 */
void multiplier(struct motorcontrol *this, float proport)
{
        this->left_front->encode_value_goal  *= proport;
        this->right_front->encode_value_goal *= proport;
        this->left_back->encode_value_goal   *= proport;
        this->right_back->encode_value_goal  *= proport;

        this->left_front->encode_value_goal  *= proport;
        this->right_front->encode_value_goal *= proport;
        this->left_back->encode_value_goal   *= proport;
        this->right_back->encode_value_goal  *= proport;
}

/**
 * @description: ���PID����,�����ٶȻ�.�ҵ�һ�������ٶ�
 * @param {*}
 * @return {*}
 */
void motorIncrementalPID(struct motorcontrol *this)
{
    float deta1,deta2,deta3,deta4;
    /* �������� */
    this->left_front->pid.b_before_error = this->left_front->pid.before_error;
    this->left_front->pid.before_error = this->left_front->pid.present_error;
    this->left_front->pid.present_error = this->left_front->pid.set_value - this->left_front->pid.present_value;
    
    this->right_front->pid.b_before_error = this->right_front->pid.before_error;
    this->right_front->pid.before_error = this->right_front->pid.present_error;
    this->right_front->pid.present_error = this->right_front->pid.set_value - this->right_front->pid.present_value;

    this->left_back->pid.b_before_error = this->left_back->pid.before_error;
    this->left_back->pid.before_error = this->left_back->pid.present_error;
    this->left_back->pid.present_error = this->left_back->pid.set_value - this->left_back->pid.present_value;

    this->right_back->pid.b_before_error = this->right_back->pid.before_error;
    this->right_back->pid.before_error = this->right_back->pid.present_error;
    this->right_back->pid.present_error = this->right_back->pid.set_value - this->right_back->pid.present_value;

    deta1 = (this->left_front->pid.present_error - 2 * this->left_front->pid.before_error + this->left_front->pid.b_before_error) / delta_T;
    deta2 = (this->right_front->pid.present_error - 2 * this->right_front->pid.before_error + this->right_front->pid.b_before_error) / delta_T;
    deta3 = (this->left_back->pid.present_error - 2 * this->left_back->pid.before_error + this->left_back->pid.b_before_error) / delta_T;
    deta4 = (this->right_back->pid.present_error - 2 * this->right_back->pid.before_error + this->right_back->pid.b_before_error) / delta_T;

    /* ������� */
    this->left_front->pid.set_error_value = this->left_front->pid.Kp *(this->left_front->pid.present_error - this->left_front->pid.before_error) + this->left_front->pid.Ki * this->left_front->pid.present_error + this->left_front->pid.Kd * deta1;
    this->right_front->pid.set_error_value = this->right_front->pid.Kp * (this->right_front->pid.present_error - this->right_front->pid.before_error) + this->right_front->pid.Ki * this->right_front->pid.present_error + this->right_front->pid.Kd * deta2;
    this->left_back->pid.set_error_value = this->left_back->pid.Kp * (this->left_back->pid.present_error - this->left_back->pid.before_error) + this->left_back->pid.Ki * this->left_back->pid.present_error + this->left_back->pid.Kd * deta3;
    this->right_back->pid.set_error_value = this->right_back->pid.Kp * (this->right_back->pid.present_error - this->right_back->pid.before_error) + this->right_back->pid.Ki * this->right_back->pid.present_error + this->right_back->pid.Kd * deta4;
  
}

//abs(v) = w*(this->car_body_size.half+this->car_body_size.half) ��ǰתΪ��
/*����������ٶ�Ϊrad/s, �����Ϊcm/s��cm*/
/*�����Ŀ�공���ٶ�*/
void inverse_target_wheel_speed(struct motorcontrol *this)
{
    this->left_front->target_wheel_speed = (*(this->target_straight_speed_y) - *(this->target_straight_speed_x) + *(this->target_rotation_speed) * (this->car_body_size->Half_length + this->car_body_size->Half_width))/wheel_circumference;
    this->right_front->target_wheel_speed =(*(this->target_straight_speed_y)+ * (this->target_straight_speed_x) - *(this->target_rotation_speed) * (this->car_body_size->Half_length + this->car_body_size->Half_width))/wheel_circumference;
    this->left_back->target_wheel_speed =  (*(this->target_straight_speed_y) - *(this->target_straight_speed_x) + *(this->target_rotation_speed) * (this->car_body_size->Half_length + this->car_body_size->Half_width))/wheel_circumference;
    this->right_back->target_wheel_speed = ((*this->target_straight_speed_y) + *(this->target_straight_speed_x) + *(this->target_rotation_speed) * (this->car_body_size->Half_length + this->car_body_size->Half_width))/wheel_circumference;
}

//abs(v) = w*(this->car_body_size.half+this->car_body_size.half)  ��ǰתΪ��
/*����������ٶ�Ϊrad/s, �����Ϊcm/s��cm*/
/*����������ٶ�*/
void inversewheel_speed(struct motorcontrol *this)
{
    this->left_front->wheel_speed = (*(this->straight_speed_y) - *(this->straight_speed_x) + *(this->rotation_speed) * (this->car_body_size->Half_length + this->car_body_size->Half_width))/wheel_circumference;
    this->right_front->wheel_speed = (*(this->straight_speed_y)+ *(this->straight_speed_x) - *(this->rotation_speed) * (this->car_body_size->Half_length + this->car_body_size->Half_width))/wheel_circumference;
    this->left_back->wheel_speed =  (*(this->straight_speed_y) - *(this->straight_speed_x) + *(this->rotation_speed) * (this->car_body_size->Half_length + this->car_body_size->Half_width))/wheel_circumference;
    this->right_back->wheel_speed = ((*this->straight_speed_y) + *(this->straight_speed_x) + *(this->rotation_speed) * (this->car_body_size->Half_length + this->car_body_size->Half_width))/wheel_circumference;
}



/**
 * @description: �����ٶ�ת���ٶ� rad/s ˳ʱ��Ϊ��,��ʱ��Ϊ��
 * @param {*}
 * @return {*}
 */
void wheel_speed2rotation_speed(struct motorcontrol *this)
{
    //float omega = (this->left_front->wheel_speed - this->right_front->wheel_speed - this->left_back->wheel_speed + this->right_back->wheel_speed) / (4 * this->car_body_size->wheel_distance);
    // w = (lf - rf -lb + rb) / 4(w+l)
    //*(this->rotation_speed) = omega * (2 * (this->car_body_size->Half_length + this->car_body_size->Half_width)) / (this->car_body_size->Half_length * this->car_body_size->Half_length + this->car_body_size->Half_width * this->car_body_size->Half_width);
    *(this->rotation_speed) = (this->left_front->wheel_speed - this->right_front->wheel_speed - this->left_back->wheel_speed + this->right_back->wheel_speed) / (4 * (this->car_body_size->Half_length + this->car_body_size->Half_width));
}
/**
 * @description: �����ٶ� r per second ת�����ٶ� cm per second,����Ϊ������
 * @param {*}
 * @return {*}
 */
void wheel_speed2x_y_speed(struct motorcontrol *this)
{
    /* vy = lf_wheel_speed +rf_wheel_speed  + lb _wheel_speed + rb_wheel_speed,����Ϊ��,����Ϊ�� */
    *(this->straight_speed_y) = (this->left_front->wheel_speed + this->left_back->wheel_speed + this->right_front->wheel_speed + this->right_back->wheel_speed) * wheel_circumference * fast_sqrt_InvSqrt(2);
    /* vx = rf_wheel_speed -lf_wheel_speed  + rb _wheel_speed - lb_wheel_speed ����Ϊ��,����Ϊ�� */
    *(this->straight_speed_x) = (-this->left_front->wheel_speed + this->right_front->wheel_speed - this->left_back->wheel_speed + this->right_back->wheel_speed) *wheel_circumference * fast_sqrt_InvSqrt(2);
}

/**
 * @description: ������ r per second ת�����ٶ� r per second
 * @param {*}
 * @return {*}
 */
void encoder2wheel_speed(struct motorcontrol *this, Motor motor)
{
    
    switch (motor)
    {
        case Motor_left_front:
        this->left_front->wheel_speed = this->left_front->get_encode_data * encoder2wheel_speed_K;
            break;
        case Motor_right_front:
        this->right_front->wheel_speed = this->right_front->get_encode_data * encoder2wheel_speed_K;
            break;
        case Motor_left_back:
        this->left_back->wheel_speed = this->left_back->get_encode_data * encoder2wheel_speed_K;
            break;
        case Motor_right_back:
        this->right_back->wheel_speed = this->right_back->get_encode_data * encoder2wheel_speed_K;
            break;
        default:
            break;
    }

}

/**
 * @description: ������ r per second ת�����ٶ� r per second
 * @param {*}
 * @return {*}
 */
void target_wheel_speed2encoder(struct motorcontrol *this, Motor motor)
{
    switch (motor)
    {
        case Motor_left_front:
        this->left_front->encode_value_goal = this->left_front->target_wheel_speed / encoder2wheel_speed_K;
            break;
        case Motor_right_front:
       this->right_front->encode_value_goal = this->right_front->target_wheel_speed / encoder2wheel_speed_K;
            break;
        case Motor_left_back:
       this->left_back->encode_value_goal =   this->left_back->target_wheel_speed / encoder2wheel_speed_K;
            break;
        case Motor_right_back:
       this->right_back->encode_value_goal =  this->right_back->target_wheel_speed / encoder2wheel_speed_K;
            break;
        default:
            break;
    }

}

void radian_PID_init(struct motorcontrol *this)
{
    this->stance->radian_pid.radian_present_error = 0;
    this->stance->radian_pid.radian_before_error  = 0;
    this->stance->radian_pid.radian_sum_error     = 0;
    this->stance->radian_pid.bodywork_Kd          = 60;
    this->stance->radian_pid.bodywork_Ki          = 0.001;
    this->stance->radian_pid.bodywork_Kp          = 15;
}

/**
 * @description: ����Ƕ�PID,������ǵ�ǰ�ǶȺ�origin��ƫ��,������ǽ��ٶ�,������ִ�нǶȻ�PID
 * @param {*}
 * @return {*}
 * @attention delta_T��imufilter.c���Ѿ�������˻������е�pid�������õ����delta_t
 */
//����ȫ�ֱ���
#define MAX_OUTPUT  300  //������ֵ
#define MIN_OUTPUT -300 //�����Сֵ  /*������Ҫ��ô����*/

void radianBodywordPID(struct motorcontrol *this)
{
    float deta;
    this->stance->radian_pid.radian_present_error = this->stance->car_yaw_radian - Origin_Radian;
    this->stance->radian_pid.radian_sum_error    += this->stance->radian_pid.radian_present_error;
    if(this->stance->radian_pid.radian_sum_error > MAX_OUTPUT/this->stance->radian_pid.bodywork_Ki)
    {
        this->stance->radian_pid.radian_sum_error = MAX_OUTPUT/this->stance->radian_pid.bodywork_Ki;
    }
    if (this->stance->radian_pid.radian_sum_error < MIN_OUTPUT/this->stance->radian_pid.bodywork_Ki)
    {
        this->stance->radian_pid.radian_sum_error = MIN_OUTPUT/this->stance->radian_pid.bodywork_Ki;
    }

    deta = (this->stance->radian_pid.radian_present_error - this->stance->radian_pid.radian_before_error) / delta_T;
    /*output = ki * sum_e + kp * e + kd * (e-before_e)/dt*/
    *(this->target_rotation_speed) = this->stance->radian_pid.bodywork_Kp * this->stance->radian_pid.radian_present_error + this->stance->radian_pid.bodywork_Kd * deta + this->stance->radian_pid.bodywork_Ki * this->stance->radian_pid.radian_sum_error;

    this->stance->radian_pid.radian_before_error = this->stance->radian_pid.radian_present_error;
    if (*(this->target_rotation_speed) > MAX_OUTPUT)
    {
        *(this->target_rotation_speed) = MAX_OUTPUT;
    }
    if (*(this->target_rotation_speed) < MIN_OUTPUT)
    {
        *(this->target_rotation_speed) = MIN_OUTPUT;
    }

}


/**
 * @description: ��ת����,���ݽǶ�pid�����ת�Ľ��ٶ�
 * @param {*}
 * @return {*}
 * @brief 
 */
void carspeedcontrol_radian(struct motorcontrol *this)
{
    //if (!state_flags.stop_flag  && state_flags.run_status_flag)
    //{
        /*
        �����������ǵĶ������ƫ��ĽǶ�
        ���Ŀ����ٶ�.
        */
        this->radianBodyworkPID(this);
        /*
        Ŀ����ٶ�ת��ΪĿ�공��ת��
        */
        this->inverse_target_wheel_speed(this);
        /*
        Ŀ�공��ת��ת��ΪĿ�������ת��
        */
        this->target_wheel_speed2encoder(this, Motor_left_front);
        this->target_wheel_speed2encoder(this, Motor_right_front);
        this->target_wheel_speed2encoder(this, Motor_left_back);
        this->target_wheel_speed2encoder(this, Motor_right_back);
        /*
        Ŀ�����������תPWM,��ʽ�ı�PWM�ٶ�
        */
        this->encode_value_goal2pwm(this);
    //}
}

/**
 * @description: ����x.y������ٶȿ���,��������PID�趨�µ��ٶ�
 * @param {*}
 * @return {*}
 * @brief 
 */
void carspeedcontrol_x_y(struct motorcontrol *this)
{
    
}

/**
 * @description: �˶�����
 * @param {*}
 * @return {*}
 * @brief 
 * ���ڵ�Ŀ����ʵ�ָ��ӿɿ��Ŀ���,��Ҫ�õ���ʼ�ĳ�����̬,
 * �ڸ�����ʻ�е�Ŀ���뵱ǰ��x��y�������,���
 * Vx��Vy(ͨ��PIDʵʱ����) Ȼ�󱣳����������̬����(ͨ������Ƕ�pid),���ڵ���ٶ�V��,
 * ������ӵõ�����ʵ�ʵ��˶��ٶ�
 */

void operationalControl()
{
    
}




/**
 * @description: �Ը�������ָ�븳ֵ,Ҳ�������ó�Ա����
 * @param {*}
 * @return {*}
 * @brief ����������,��Ϊǰ��Ҫ�������˲���ָ��ȥ
 */
void motorcontrol_Init(struct motorcontrol *this)
{
    this->motorcontrol_Constructor = motorcontrol_Constructor;
    this->EncoderGet               = EncoderGet;
    this->PIDParameterInit         = PIDParameterInit;
    this->signalMotorControl       = signalMotorControl;
    this->encode_value_goal2pwm    = encode_value_goal2pwm;
    this->multiplier               = multiplier;
    this->motorIncrementalPID      = motorIncrementalPID;
    this->inverse_target_wheel_speed = inverse_target_wheel_speed;
    this->inversewheel_speed       = inversewheel_speed;
    this->wheel_speed2rotation_speed = wheel_speed2rotation_speed;
    this->wheel_speed2x_y_speed    = wheel_speed2x_y_speed;
    this->encoder2wheel_speed      = encoder2wheel_speed;
    this->target_wheel_speed2encoder = target_wheel_speed2encoder;
    this->radian_PID_init          = radian_PID_init;
    this->radianBodyworkPID        = radianBodywordPID;
    this->carspeedcontrol_radian   = carspeedcontrol_radian;
}
