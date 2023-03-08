#include "zf_common_headfile.h"
//启动时会有时候不能同时启动,所以会有便宜,所以根据陀螺仪
//然后对其计算车模角度,辅助纠正车模姿态保持始终车身向前
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

/*
目标地址
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
    车体速度的基变量。
*/
extern uint16_t straight_speed;
extern uint16_t rotation_speed;
extern uint16_t carry_speed;
extern float mProport;
extern uint8_t increase_adjust_count;
extern uint8_t decrease_adjust_count;
extern int8_t rotation_direction;
extern float deviation_radian;
extern float pan_direction;
extern float scale_factor;
extern float test_car_x; 
extern float test_car_y;
extern float growthRateMultiplier; //增长比例乘数
extern int8_t counterclockwiseDetection;

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
    uint16_t *straight_speed;
    uint16_t *rotation_speed;
    uint16_t *carry_speed;
    float *mProport;
    uint8_t *increase_adjust_count;
    uint8_t *decrease_adjust_count;
    int8_t *rotation_direction;
    float *deviation_radian;
    float *pan_direction;
    float *scale_factor;
    float *test_car_x;
    float *test_car_y;
    float *growthRateMultiplier;
    int8_t *counterclockwiseDetection;

    void (*motorcontrol_Constructor)(struct motorcontrol *this);
    void (*motorcontrol_Destructor)(struct motorcontrol *this);
    void (*EncoderGet)(struct motorcontrol *this);
    void (*signalMotorControl)(struct motorcontrol *this, Motor motor, uint8_t dir, uint16_t set_encode_data); //所有步骤都需要这个来改变电机的PWM
    void (*carRunControl)(struct motorcontrol *this, float radian /* -pi~pi */, uint16_t speed);
    void (*multiplier)(struct motorcontrol *this, float proport);
    void (*motorIncrementalPID)(struct motorcontrol *this);
    float (*radianBodyworkPID)(struct motorcontrol *this, float error);
};

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
        this->straight_speed = &straight_speed;
        this->rotation_speed = &rotation_speed;
        this->carry_speed = &carry_speed;
        this->mProport = &mProport;
        this->increase_adjust_count = &increase_adjust_count;
        this->decrease_adjust_count = &decrease_adjust_count;
        this->pan_direction = &pan_direction;
        this->scale_factor = &scale_factor;
        this->test_car_x = &test_car_x;
        this->test_car_y = &test_car_y;
        this->growthRateMultiplier = &growthRateMultiplier;
        this->counterclockwiseDetection = &counterclockwiseDetection;

    }
    else
    {

    }
}

/**
 * @description: 电机PID参数初始化
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
 * @description: 编码器初始化
 * @param {*}
 * @return {*}
 */
void encoderInit()
{
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); 
}
/**
 * @description: 电机初始化
 * @param {*}
 * @return {*}
 */
void motorInit()
{
    gpio_init(Motor_left_front_dir, GPO, 0, GPO_PUSH_PULL); /*原处理中这里GPI_PULL_UP，但由于是输出后面两个参数无效*/
    pwm_init(Motor_left_front_pwm, 17000, 0);
    gpio_init(Motor_right_front_dir, GPO, 0, GPO_PUSH_PULL);
    pwm_init(Motor_right_front_pwm, 17000, 0);
    gpio_init(Motor_left_back_dir, GPO, 0, GPO_PUSH_PULL);
    pwm_init(Motor_left_back_pwm, 17000, 0);
    gpio_init(Motor_right_back_dir, GPO, 0, GPO_PUSH_PULL);
    pwm_init(Motor_right_back_pwm, 17000, 0);
}

/**
 * @description: 编码器值获取（转换为1s时间的编码器值）
 * @param {*}
 * @return {*}
 */
void EncoderGet(struct motorcontrol *this)
{
    if (this != 0)
    {
        /* code */
    this->left_front->get_encode_data = (int16_t)(encoder_get_count(ENCODER_1) / position_time_resolution);
    this->left_front->pid.present_value = this->left_front->get_encode_data;
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

    this->right_front->get_encode_data = 0 - (int16_t)(encoder_get_count(ENCODER_2) / position_time_resolution);
    this->right_front->pid.present_value = this->right_front->get_encode_data;
    encoder_clear_count(ENCODER_2);                                             // 清空编码器计数

    this->left_back->get_encode_data = (int16_t)(encoder_get_count(ENCODER_3)/ position_time_resolution);
    this->left_back->pid.present_value = this->left_back->get_encode_data;
    encoder_clear_count(ENCODER_3);                                             // 清空编码器计数

    this->right_back->get_encode_data = 0 - (int16_t)(encoder_get_count(ENCODER_4) / position_time_resolution);
    this->right_back->pid.present_value = this->right_back->get_encode_data;
    encoder_clear_count(ENCODER_4);                                             // 清空编码器计数
    static int vibration_count = 0;                                             //震动计数
    //要弧度调整时
#define VibrationRate 1000
    if (this->state_flags->radian_adjustment_flag && abs(this->left_front->get_encode_data) < VibrationRate && abs(this->right_front->get_encode_data) < VibrationRate && abs(this->left_back->get_encode_data) < VibrationRate && abs(this->right_back->get_encode_data) < VibrationRate)
    {
        vibration_count++;
        *this->growthRateMultiplier = (float)(vibration_count) / 100.0f;
    }
    else
    {
        vibration_count = 0;
        *this->growthRateMultiplier = -1;
    }
//                //PRINTF("speed:%d,%d,%d,%d\n", left_front.get_encode_data, right_front.get_encode_data, left_back.get_encode_data, right_back.get_encode_data);
    }
    else
    {
        ;
    }

}

/**
 * @description: 单电机控制根据设定值和方向调节速度
 * @param {Motor} motor
 * @param {uint8_t} dir
 * @param {uint16_t} set_encode_data
 * @return {*}
 */
void signalMotorControl(struct motorcontrol *this, Motor motor, uint8_t dir, uint16_t set_encode_data)
{
    if (this != 0)
    {
        /* code */
        set_encode_data = set_encode_data > 40000 ? 40000 : set_encode_data;
        if (set_encode_data == 40000)
        {
            switch (motor)
            {
            case Motor_left_front:
                this->left_front->set_encode_data = set_encode_data;
                break;
            case Motor_right_front:
                this->right_front->set_encode_data = set_encode_data;
                break;
            case Motor_left_back:
                this->left_back->set_encode_data = set_encode_data;
                break;
            case Motor_right_back:
                this->right_back->set_encode_data = set_encode_data;
                break;
            default:
                break;
            }
        }
//        if (this->state_flags->radian_adjustment_flag && *this->growthRateMultiplier >0)
//        {
//#define GrowthBase 10000
        // set_encode_data += (growthRateMultiplier > 2.0f ? 2.0f : growthRateMultiplier) * GrowthBase; 
//        }
            switch (motor)
            {
            case Motor_left_front:
                gpio_set_level(Motor_left_front_dir, dir);             //会自动扩展,设置方向
                pwm_set_duty(Motor_left_front_pwm, set_encode_data); //编码器值转pwm
                this->left_front->dir = dir;
                break;
            case Motor_right_front:
                gpio_set_level(Motor_right_front_dir, dir);
                pwm_set_duty(Motor_right_front_pwm, set_encode_data);
                this->right_front->dir = dir;
                break;
            case Motor_left_back:
                gpio_set_level(Motor_left_back_dir, dir);
                pwm_set_duty(Motor_left_back_pwm, set_encode_data);
                this->left_back->dir = dir;
                break;
            case Motor_right_back:
                gpio_set_level(Motor_right_back_dir, dir);
                pwm_set_duty(Motor_right_back_pwm, set_encode_data);
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

/**
 * @description: 结合姿态数据、目标位置，大致方向控制
 * @param {float} radian
 * @param {uint16_t} speed
 * @return {*}
 */
void carRunControl(struct motorcontrol *this, float radian /* -pi~pi */, uint16_t speed)
{
    if (this != 0)
    {
        if (this->state_flags->start_up_flag)
        {
            *this->increase_adjust_count = IncreaseOrder;
            *this->decrease_adjust_count = DecreaseOrder;
            pit_disable(PIT_CH2);
        }
        this->state_flags->speed_control_flag = 0;
        int16_t Vx, Vy;
        Vy = (int16_t)(speed * sin(radian));
        Vx = (int16_t)(speed * cos(radian));
        int16_t Vlf = Vy + Vx,
        Vrf = Vy - Vx,
        Vlb = Vrf,
        Vrb = Vlf;

        /*
        四个轮子的速度设置 
        */
        this->left_front->set_encode_data = Vlf;
        this->right_front->set_encode_data = Vrf;
        this->left_back->set_encode_data = Vlb;
        this->right_back->set_encode_data = Vrb;

        this->left_front->encode_value_goal = Vlf;
        this->right_front->encode_value_goal = Vrf;
        this->left_back->encode_value_goal = Vlb;
        this->right_back->encode_value_goal = Vrb;
        /*开始后非微调非丢失目标进入*/
        if (!this->state_flags->position_adjust_flag && !this->state_flags->missing_target_flag && this->state_flags->start_up_flag)
        {
            *this->increase_adjust_count = IncreaseOrder;
            *this->decrease_adjust_count = DecreaseOrder;
            this->state_flags->speed_control_flag = 1;
            *this->mProport = 2.5f;
            this->multiplier(this,1 / *(this->mProport));
            pit_ms_init(PIT_CH2, 100);

        }
        if (Vlf < 0)
        {
            this->signalMotorControl(this,Motor_left_front, 0, 0 - left_front.encode_value_goal);
        }
        else
        {
            this->signalMotorControl(this,Motor_left_front, 1, left_front.encode_value_goal);
        }
        if (Vrf < 0)
        {
            this->signalMotorControl(this,Motor_right_front, 0, 0 - right_front.encode_value_goal);
        }
        else
        {
            this->signalMotorControl(this,Motor_right_front, 1, right_front.encode_value_goal);
        }
        if (Vlb < 0)
        {
            this->signalMotorControl(this,Motor_left_back, 0, 0 - left_back.encode_value_goal);
        }
        else
        {
            this->signalMotorControl(this,Motor_left_back, 1, left_back.encode_value_goal);
        }
        if (Vrb < 0)
        {
            this->signalMotorControl(this,Motor_right_back, 0, 0 - right_back.encode_value_goal);
        }
        else
        {
            this->signalMotorControl(this,Motor_right_back, 1, right_back.encode_value_goal);
        }
        this->state_flags->run_status_flag =1;
        this->state_flags->body_rotation_flag = 0;
        this->state_flags->stop_flag = 0;

    }
    else
    {
        ;
    }
    
}

/**
 * @description: 倍速运行,结合PID控制
 * @param {*}
 * @return {*}
 */
void multiplier(struct motorcontrol *this, float proport)
{
        this->left_front->set_encode_data  *= proport;
        this->right_front->set_encode_data *= proport;
        this->left_back->set_encode_data   *= proport;
        this->right_back->set_encode_data  *= proport;

        this->left_front->encode_value_goal  *= proport;
        this->right_front->encode_value_goal *= proport;
        this->left_back->encode_value_goal   *= proport;
        this->right_back->encode_value_goal  *= proport;
}

/**
 * @description: 电机PID调节
 * @param {*}
 * @return {*}
 */
void motorIncrementalPID(struct motorcontrol *this)
{
    /* 参数更新 */
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

    /* 输出增量 */
    this->left_front->pid.set_error_value = this->left_front->pid.Kp *(this->left_front->pid.present_error - this->left_front->pid.before_error) + this->left_front->pid.Ki * this->left_front->pid.present_error + this->left_front->pid.Kd *(this->left_front->pid.present_error - 2 * this->left_front->pid.before_error + this->left_front->pid.b_before_error);
    this->right_front->pid.set_error_value = this->right_front->pid.Kp * (this->right_front->pid.present_error - this->right_front->pid.before_error) + this->right_front->pid.Ki * this->right_front->pid.present_error + this->right_front->pid.Kd * (this->right_front->pid.present_error - 2 * this->right_front->pid.before_error + this->right_front->pid.b_before_error);
    this->left_back->pid.set_error_value = this->left_back->pid.Kp * (this->left_back->pid.present_error - this->left_back->pid.before_error) + this->left_back->pid.Ki * this->left_back->pid.present_error + this->left_back->pid.Kd * (this->left_back->pid.present_error - 2 * this->left_back->pid.before_error + this->left_back->pid.b_before_error);
    this->right_back->pid.set_error_value = this->right_back->pid.Kp * (this->right_back->pid.present_error - this->right_back->pid.before_error) + this->right_back->pid.Ki * this->right_back->pid.present_error + this->right_back->pid.Kd * (this->right_back->pid.present_error - 2 * this->right_back->pid.before_error + this->right_back->pid.b_before_error);
}

/**
 * @description: 车身角度PID
 * @param {*}
 * @return {*}
 */
float radianBodywordPID(struct motorcontrol *this, float error)
{
    const float bodywork_Kp = 15, bodywork_Ki = 0.001, bodywork_Kd = 60;
    static float error_sum = 0, before_error = 0;
    float radian_adjust_value = bodywork_Kp * error + bodywork_Ki * error_sum + bodywork_Kd * (error - before_error);

    error_sum += error;
    if (error_sum > 0.1 || error_sum < -0.1)
    {
        error_sum -= error;
    }
    before_error = error;
    return radian_adjust_value;
}

/**
 * @description: 运动控制
 * @param {*}
 * @return {*}
 */
void operationalControl()
{
    
}