#include "zf_common_headfile.h"
//����ʱ����ʱ����ͬʱ����,���Ի��б���,���Ը���������
//Ȼ�������㳵ģ�Ƕ�,����������ģ��̬����ʼ�ճ�����ǰ
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
extern float growthRateMultiplier; //������������
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
    void (*signalMotorControl)(struct motorcontrol *this, Motor motor, uint8_t dir, uint16_t set_encode_data); //���в��趼��Ҫ������ı�����PWM
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
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); 
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
    this->left_front->get_encode_data = (int16_t)(encoder_get_count(ENCODER_1) / position_time_resolution);
    this->left_front->pid.present_value = this->left_front->get_encode_data;
    encoder_clear_count(ENCODER_1);                                             // ��ձ���������

    this->right_front->get_encode_data = 0 - (int16_t)(encoder_get_count(ENCODER_2) / position_time_resolution);
    this->right_front->pid.present_value = this->right_front->get_encode_data;
    encoder_clear_count(ENCODER_2);                                             // ��ձ���������

    this->left_back->get_encode_data = (int16_t)(encoder_get_count(ENCODER_3)/ position_time_resolution);
    this->left_back->pid.present_value = this->left_back->get_encode_data;
    encoder_clear_count(ENCODER_3);                                             // ��ձ���������

    this->right_back->get_encode_data = 0 - (int16_t)(encoder_get_count(ENCODER_4) / position_time_resolution);
    this->right_back->pid.present_value = this->right_back->get_encode_data;
    encoder_clear_count(ENCODER_4);                                             // ��ձ���������
    static int vibration_count = 0;                                             //�𶯼���
    //Ҫ���ȵ���ʱ
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
 * @description: ��������Ƹ����趨ֵ�ͷ�������ٶ�
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
                gpio_set_level(Motor_left_front_dir, dir);             //���Զ���չ,���÷���
                pwm_set_duty(Motor_left_front_pwm, set_encode_data); //������ֵתpwm
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
 * @description: �����̬���ݡ�Ŀ��λ�ã����·������
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
        �ĸ����ӵ��ٶ����� 
        */
        this->left_front->set_encode_data = Vlf;
        this->right_front->set_encode_data = Vrf;
        this->left_back->set_encode_data = Vlb;
        this->right_back->set_encode_data = Vrb;

        this->left_front->encode_value_goal = Vlf;
        this->right_front->encode_value_goal = Vrf;
        this->left_back->encode_value_goal = Vlb;
        this->right_back->encode_value_goal = Vrb;
        /*��ʼ���΢���Ƕ�ʧĿ�����*/
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
 * @description: ��������,���PID����
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
 * @description: ���PID����
 * @param {*}
 * @return {*}
 */
void motorIncrementalPID(struct motorcontrol *this)
{
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

    /* ������� */
    this->left_front->pid.set_error_value = this->left_front->pid.Kp *(this->left_front->pid.present_error - this->left_front->pid.before_error) + this->left_front->pid.Ki * this->left_front->pid.present_error + this->left_front->pid.Kd *(this->left_front->pid.present_error - 2 * this->left_front->pid.before_error + this->left_front->pid.b_before_error);
    this->right_front->pid.set_error_value = this->right_front->pid.Kp * (this->right_front->pid.present_error - this->right_front->pid.before_error) + this->right_front->pid.Ki * this->right_front->pid.present_error + this->right_front->pid.Kd * (this->right_front->pid.present_error - 2 * this->right_front->pid.before_error + this->right_front->pid.b_before_error);
    this->left_back->pid.set_error_value = this->left_back->pid.Kp * (this->left_back->pid.present_error - this->left_back->pid.before_error) + this->left_back->pid.Ki * this->left_back->pid.present_error + this->left_back->pid.Kd * (this->left_back->pid.present_error - 2 * this->left_back->pid.before_error + this->left_back->pid.b_before_error);
    this->right_back->pid.set_error_value = this->right_back->pid.Kp * (this->right_back->pid.present_error - this->right_back->pid.before_error) + this->right_back->pid.Ki * this->right_back->pid.present_error + this->right_back->pid.Kd * (this->right_back->pid.present_error - 2 * this->right_back->pid.before_error + this->right_back->pid.b_before_error);
}

/**
 * @description: ����Ƕ�PID
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
 * @description: �˶�����
 * @param {*}
 * @return {*}
 */
void operationalControl()
{
    
}