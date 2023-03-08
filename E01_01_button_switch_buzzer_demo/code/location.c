#include "location.h"
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
bool target_update = true;

/* 重点区域,暂时不懂什么作用,或许是调试使用 */
const static float scheme_coord_one[2][12] = {{50, 190, 310, 430, 70, 270, 410, 170, 90, 210, 350, 450}, {90, 70, 50, 50, 170, 130, 150, 230, 290, 310, 350, 270}};
/* 数字牌子,3分类前面是决定放进那个分类篮,后面是数字牌号*/
const static uint8_t scheme_picture_one[2][12] = {{'1', '1', '2', '1', '3', '3', '2', '2', '2', '3', '1', '3'}, {'2', '5', '1', '3', '5', '1', '3', '2', '5', '3', '1', '2'}};
const static float scheme_coord_two[2][12] = {{90, 230, 390, 90, 190, 350, 50, 170, 310, 430, 290, 410}, {50, 70, 90, 190, 150, 170, 270, 290, 250, 230, 330, 350}};
const static uint8_t scheme_picture_two[2][12] = {{'1', '2', '3', '2', '3', '1', '3', '1', '3', '2', '2', '1'}, {'5', '2', '3', '1', '5', '3', '1', '1', '2', '5', '3', '2'}};
const static float scheme_coord_three[2][12] = {{50, 150, 250, 430, 110, 270, 290, 390, 70, 170, 330, 450}, {110, 50, 70, 90, 210, 230, 150, 190, 350, 290, 330, 310}};
const static uint8_t scheme_picture_three[2][12] = {{'3', '2', '1', '2', '2', '2', '3', '1', '1', '3', '1', '3'}, {'1', '4', '1', '5', '1', '3', '5', '3', '5', '4', '4', '3'}};
const static float scheme_coord_four[2][12] = {{70, 150, 230, 290, 390, 90, 210, 410, 50, 190, 330, 430}, {130, 70, 150, 90, 50, 230, 270, 210, 310, 350, 330, 290}};
const static uint8_t scheme_picture_four[2][12] = {{'1', '3', '1', '2', '3', '2', '1', '1', '3', '2', '3', '2'}, {'2', '4', '4', '2', '3', '4', '3', '1', '5', '3', '2', '5'}};
const static float scheme_coord_five[2][12] = {{70, 170, 270, 350, 430, 50, 210, 290, 390, 110, 310, 450}, {110, 150, 50, 170, 90, 290, 270, 230, 250, 350, 330, 310}};
const static uint8_t scheme_picture_five[2][12] = {{'1', '2', '3', '1', '3', '3', '1', '2', '1', '2', '3', '2'}, {'4', '1', '5', '5', '2', '4', '2', '5', '1', '2', '1', '4'}};
const static float scheme_coord_six[2][12] = {{50, 190, 410, 130, 290, 110, 390, 70, 170, 250, 310, 450}, {90, 50, 70, 130, 110, 210, 190, 310, 270, 290, 350, 330}};
const static uint8_t scheme_picture_six[2][12] = {{'1', '3', '2', '2', '1', '3', '1', '2', '1', '3', '2', '3'}, {'3', '4', '1', '3', '4', '2', '2', '2', '1', '1', '4', '3'}};

/*
    当你第一次使用这种语法的时候请一定要记得,要先给this分配内存，不能生命struct location *this这样会没有内存分配
    struct main b;
    struct main *this = &b;
    this->test = &test;这样才是正确的分配了内存

    struct AntColonySystem acs;
    struct ACSAnt ants[M];
    或者像这样直接输入地址
    AntColonySystemInitialize(&acs);
	acs.AntColonySystem_Constructor(&acs, N);

*/
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
    float(*location_solve_inverse_movement_rotation_speed)(struct location *this);
    float(*location_solve_gyro_rotation_speed)(struct location *this);
    float(*gyroMixFilter)(struct location *this);
    float*(*accMixFilter)(struct location *this);
    void (*radiansGet)(struct location *this);
    void (*positioningSystemInit)(struct location *this, uint8_t num);
    void (*coordChange)(struct location *this);
    void (*nextRecentTargetDiscrimination)(struct location *this);
    uint8_t (*currentStageCompleteJudgment)(struct location *this);
    void (*positioningSystemChange)(struct location *this);
    void (*regularRadians)(float *radian); 
};

void location_Constructor(struct location *this)
{
    if (this!=0)
    {
        this->state_flags = &state_flags;
        this->coord = &coord;
        this->stance = &stance;
        this->eulerAngle = &eulerAngle;
        this->left_front = &left_front ;
        this->right_front= &right_front;
        this->left_back  = &left_back  ;
        this->right_back = &right_back ;
        this->car_body_size = &car_body_size;
        this->imu_data = &imu_data;

        this->target_coord = target_coord;
        this->target_lable = target_lable;
        this->city_result = city_result;
        this->current_obj_index = &current_obj_index;
        this->object_num = &object_num;
        this->compensate_radian = &compensate_radian;
        this->target_update = &target_update;
        this->A4_select = &A4_select;
        this->Carry = &Carry;

    }
    else
    {

    }
}

void location_Destructor(struct location *this)
{
    if(this != 0)
    {

    }
    else
	{
	}
}
/* 内部函数 */
/**
 * @description: 逆运动学解算角速度。
 * @param {&struct location this}
 * @return {*}
 */
float location_solve_inverse_movement_rotation_speed(struct location *this)
{
    if (this != 0)
    {
        /* code */
        return  (this->right_front->get_encode_data - this->left_front->get_encode_data + this->right_back->get_encode_data - this->left_back->get_encode_data)/ 4.0 / (this->car_body_size->Half_length + this->car_body_size->Half_width) * this->car_body_size->Encoder2Velocity; /* 转换系数，根据麦轮转换的角速度 */
    }
	else
	{
		return 0;
	}
}

/* 内部函数 */
/**
 * @description: 陀螺仪滤波解算角速度。
 * @param {&struct location this}
 * @return {*}
 */
float location_solve_gyro_rotation_speed(struct location *this)
{
    if (this != 0 )
    {
        /* code */
        //这里需要经过gyroOffset_init() -> IMU_getValues()
        return hullMovingWindow(this->imu_data->gyro_z);
    }
	else
	{
		return 0;
	}
    
}

/* 内部函数 */
/**
 * @description: 陀螺仪滤波混合逆运动学解算角速度。
 * @param {&struct location this}
 * @return {*}
 */
float gyroMixFilter(struct location *this)
{
    #define Proportion 1 //看情况调整
    if(this != 0)
    {
        float inverse_movement_rotation_speed = location_solve_inverse_movement_rotation_speed(this);
        float gyro_rotation_speed =  location_solve_gyro_rotation_speed(this);
            /*这里考虑一下是哪一个权重比例更大，是逆运动学的还是陀螺仪*/                                                                                                                                                                                                                                                             //     	return inverse_movement_rotation_speed;
        return inverse_movement_rotation_speed * (1 - Proportion) + gyro_rotation_speed * Proportion;
    }
	else
	{
			return 0;
	}

}

//我们IMU只用来传输出了角度，我们还需要加速度计的数字,原式中完全舍弃了加速度计的值，只使用了编码器加上
float *accMixFilter(struct location *this)
{
    if (this != 0)
    {
        /* code */
        static float V[2] = {0, 0};
        float acc_X, acc_Y;

        if (this->state_flags->integralDriftClearanceFlag) //停车时抑制积分漂移
        {
            V[0] = 0;
            V[1] = 0;
            this->state_flags->integralDriftClearanceFlag = 0;
        }

        //    acc_X = 0 - hullMovingWindow((icm_acc_x - acc_X_drift_value - acc_X_drift_value_2) / 4096.0) * Gravity_Acc;
        //    acc_Y = 0 - hullMovingWindow((icm_acc_y - acc_Y_drift_value - acc_Y_drift_value_2) / 4096.0) * Gravity_Acc;
        //    acc_X = acc_X * sin(stance.car_yaw_radian) + acc_X * sin(stance.car_yaw_radian - 1.570796);
        //		acc_Y = acc_Y * sin(stance.car_yaw_radian) + acc_X * sin(stance.car_yaw_radian - 1.570796);
        // V[0] = V[0] + (acc_X + (acc_X - A[0]) / 2.0)*position_time_resolution;
        // V[1] = V[1] + (acc_Y + (acc_Y - A[1]) / 2.0)*position_time_resolution;
        float Y_inverse_motion_solution_speed = /* hullMovingWindow_Y */ ((this->left_front->get_encode_data + this->right_front->get_encode_data + this->left_back->get_encode_data + this->right_back->get_encode_data) * this->car_body_size->Encoder2Velocity /4.0);
        float X_inverse_motion_solution_speed = /* hullMovingWindow_X */ ((this->left_front->get_encode_data - this->right_front->get_encode_data - this->left_back->get_encode_data + this->right_back->get_encode_data) * this->car_body_size->Encoder2Velocity /4.0);
        /*x,y实际的速度*/                             //this->eulerAngle->yaw,两种都可以，也可以直接把stance那个地址指向yaw的地址
        V[0] = Y_inverse_motion_solution_speed * cos(this->stance->car_yaw_radian) + X_inverse_motion_solution_speed * sin(this->stance->car_yaw_radian);
        V[1] = Y_inverse_motion_solution_speed * sin(this->stance->car_yaw_radian) - X_inverse_motion_solution_speed * cos(this->stance->car_yaw_radian);
        //    V[0] = kalmanFilter_X(X_inverse_motion_solution_speed, acc_X);
        //    V[1] = kalmanFilter_Y(Y_inverse_motion_solution_speed, acc_Y);
        // PRINTF("VY:%f,%f\n",(left_front.get_encode_data + right_front.get_encode_data + left_back.get_encode_data + right_back.get_encode_data) * car_body_size.Encoder2Velocity / 4.0, Y_inverse_motion_solution_speed);
        // PRINTF("VX:%f,%f\n",(left_front.get_encode_data - right_front.get_encode_data - left_back.get_encode_data + right_back.get_encode_data) * car_body_size.Encoder2Velocity / 4.0, X_inverse_motion_solution_speed);
        //    PRINTF(":%f,%f\n",Y_inverse_motion_solution_speed,X_inverse_motion_solution_speed);
        return V;
    }
    
}
/**
 * @description: 规整弧度至-Π~Π
 * @param {float} radian
 * @return {*}
 */
void regularRadians(float *radian)
{
    while (*radian - 3.141592 >= FLT_MIN_SELF)
    {
        *radian -= 6.283185;
    }
    while (*radian + 3.141592 <= -FLT_MIN_SELF)
    {
        *radian += 6.283185;
    }
}

/**
 * @description: 车体转动角度更新
 * @param {&struct location this}
 * @return {*}
 */
void radiansGet(struct location *this)
{
    #define Proportion 0.3 //看情况调整
    /* imu数据处理，因为是在中断当中处理所以可以给出准确的处理时间 */
    this->stance->car_yaw_radian = (this->eulerAngle->yaw * Proportion);//以互补滤波以基准，上面加上了麦克钠姆轮的速度，有一定积分误差
    this->stance->car_yaw_radian += (this->location_solve_inverse_movement_rotation_speed(this) *position_time_resolution *(1-Proportion));

    //this->stance->car_yaw_radian += gyroMixFilter(this)*position_time_resolution; // kalman result, includes the integration process */ * position_time_resolution;
    // stance.car_yaw_radian -= compensate_radian;
    this->regularRadians(&this->stance->car_yaw_radian);

    //    PRINTF("r:%f\n", stance.car_yaw_radian);
    //   stance.target_relative_radian = atan2(coord.target_coord_y - coord.current_coord_y, coord.target_coord_x - coord.current_coord_x);

    /* 判断整体运动方向 */
    // float Y_inverse_motion_solution_speed = (left_front.get_encode_data + right_front.get_encode_data + left_back.get_encode_data + right_back.get_encode_data);
    // float X_inverse_motion_solution_speed = (left_front.get_encode_data - right_front.get_encode_data - left_back.get_encode_data + right_back.get_encode_data);
    // /* 加阈值 */
    // if (X_inverse_motion_solution_speed == 0 && Y_inverse_motion_solution_speed == 0)
    // {
    //     stance.direction_progress_radian = stance.target_relative_radian;
    // }
    // else
    // {
    //     stance.direction_progress_radian = atan2(Y_inverse_motion_solution_speed,
    //                                              X_inverse_motion_solution_speed);
    // }
}



/**
 * @description: 定位体系初始化.num输入0进入测试
 * @param {&struct location this, num}
 * @return {*}
 */
void positioningSystemInit(struct location *this, uint8_t num)
{
    if(this != 0)
    {
        if(num)
        {
            *this->object_num = num;
            for (uint8_t i = 0; i < *this->object_num; i++)
            {
                /* code */
                this->target_coord[0][i] = block_size * floor(this->target_coord[0][i] / block_size) + 10;
                this->target_coord[1][i] = block_size * floor(this->target_coord[1][i] / block_size) + 10;
            }
            
        }
        else
        {
            *this->object_num = OBJNUM;
            switch (*this->A4_select)
            {
            case 0:
                this->target_coord[0][0] = 50;
                this->target_coord[1][0] = 150;
                this->target_coord[0][1] = 310;
                this->target_coord[1][1] = 210;
                this->target_coord[0][2] = 210;
                this->target_coord[1][2] = 90;
                break;            
            case 1:
                memcpy(this->target_coord[0], scheme_coord_one[0], OBJNUM * sizeof(float));
                memcpy(this->target_coord[1], scheme_coord_one[1], OBJNUM * sizeof(float));
                memcpy(this->target_lable[0], scheme_picture_one[0], OBJNUM * sizeof(uint8_t));
                memcpy(this->target_lable[1], scheme_picture_one[1], OBJNUM * sizeof(uint8_t));
                break;
            case 2:
                memcpy(this->target_coord[0], scheme_coord_two[0], OBJNUM * sizeof(float));
                memcpy(this->target_coord[1], scheme_coord_two[1], OBJNUM * sizeof(float));
                memcpy(this->target_lable[0], scheme_picture_two[0], OBJNUM * sizeof(uint8_t));
                memcpy(this->target_lable[1], scheme_picture_two[1], OBJNUM * sizeof(uint8_t));
                break;
            case 3:
                memcpy(this->target_coord[0], scheme_coord_three[0], OBJNUM * sizeof(float));
                memcpy(this->target_coord[1], scheme_coord_three[1], OBJNUM * sizeof(float));
                memcpy(this->target_lable[0], scheme_picture_three[0], OBJNUM * sizeof(uint8_t));
                memcpy(this->target_lable[1], scheme_picture_three[1], OBJNUM * sizeof(uint8_t));
                break;
            case 4:
                memcpy(this->target_coord[0], scheme_coord_four[0], OBJNUM * sizeof(float));
                memcpy(this->target_coord[1], scheme_coord_four[1], OBJNUM * sizeof(float));
                memcpy(this->target_lable[0], scheme_picture_four[0], OBJNUM * sizeof(uint8_t));
                memcpy(this->target_lable[1], scheme_picture_four[1], OBJNUM * sizeof(uint8_t));
                break;
            case 5:
                memcpy(this->target_coord[0], scheme_coord_five[0], OBJNUM * sizeof(float));
                memcpy(this->target_coord[1], scheme_coord_five[1], OBJNUM * sizeof(float));
                memcpy(this->target_lable[0], scheme_picture_five[0], OBJNUM * sizeof(uint8_t));
                memcpy(this->target_lable[1], scheme_picture_five[1], OBJNUM * sizeof(uint8_t));
                break;
            case 6:
                memcpy(this->target_coord[0], scheme_coord_six[0], OBJNUM * sizeof(float));

                memcpy(this->target_coord[1], scheme_coord_six[1], OBJNUM * sizeof(float));
                memcpy(this->target_lable[0], scheme_picture_six[0], OBJNUM * sizeof(uint8_t));
                memcpy(this->target_lable[1], scheme_picture_six[1], OBJNUM * sizeof(uint8_t));
                break;
            }
        }
        this->coord->current_coord_x = 12; //设定出发位置
        this->coord->current_coord_y = -51;
        this->coord->before_self_coord_x = this->coord->current_coord_x;
        this->coord->before_self_coord_y = this->coord->current_coord_y;
        this->stance->car_yaw_radian = M_PI/2;
        if(*this->Carry)
        {
            float bestdis;//用于存放蚁群算法最佳的走位
            while (!Ant_findBestPath((*this->object_num) + 1, &bestdis))
            {
                /* code */
                ;
            }
            for (uint8_t i = 0; i < *(this->object_num)+1;)
            {
                /* code */
                static uint8_t index = 0;
                if (index > *(this->object_num))
                {
                    /* code */
                    break;
                }
                else if(index)
                {
                    this->city_result[index - 1] = globalTour[i][0];
                    //从obj的下一个到达的城市开始记录
                    index++;
                }
                if (globalTour[i][0] == *(this->object_num))
                {
                    /* code */
                    index = 1;
                }
                if (i == *(this->object_num))
                {
                    /* code */
                    i = 0;
                    continue;
                }
                i++;               
            }        
        }
        this->positioningSystemChange(this);
    }
}

/**
 * @description: 坐标更新
 * @param {&struct location this}
 * @return {*}
 */
void coordChange(struct location *this)
{
    /* icm数据处理 */
    //结合陀螺仪再次分配位移
    // float path_displacement = accMixFilter();
    // coord.current_coord_x += path_displacement * cos(stance.car_yaw_radian) * position_time_resolution;
    // coord.current_coord_y += path_displacement * sin(stance.car_yaw_radian) * position_time_resolution;
    float *V /* X、Y方向速度 */;
    static float D[2] = {0} /* X、Y方向前一时刻速度 */;
    if (this->state_flags->integralDriftClearanceFlag) //停车时抑制积分漂移
    {
        D[0] = 0;
        D[1] = 0;
    }
    V = this->accMixFilter(this);
    this->coord->current_coord_x += (*(V)) * position_time_resolution * 100;         //单位换算为cm，v的单位原本是m/s
    this->coord->current_coord_y += (*(V + 1)) * position_time_resolution * 100;     //单位换算为cm，v的单位原本是m/s
    //    coord.current_coord_x = coord.current_coord_x + (*(V) + (*(V)-D[0]) / 2.0) * position_time_resolution * 100;
    //    coord.current_coord_y = coord.current_coord_y + (*(V + 1) + (*(V + 1) - D[1]) / 2.0) * position_time_resolution * 100;
    //    D[0] = *(V);
    //    D[1] = *(V + 1);
    //    coord.current_coord_x += *(V)*position_time_resolution * 100;       //单位换算为cm
    //    coord.current_coord_y += *(V + 1) * position_time_resolution * 100; //单位换算为cm
    // PRINTF(":%f,%f,%f,%f\n", coord.current_coord_x, coord.current_coord_y, gyro_Z_drift_value, gyro_Z_drift_value_2);
}

/**
 * @description: 判断下一最近目标点位置
 * ]
 * @param {struct location *this}
 * @return {*}
 */
void nextRecentTargetDiscrimination(struct location *this)
{
    *(this->target_update) = true;
    (*this->current_obj_index)++;
    if (*(this->current_obj_index) < *(this->object_num))
    {
        /* code 有带着东西时，选择排序,以此从最小的的位移开始送，也就是最后选择位置放牌子的顺序，依据最近的原则（也可以用快速排序）,数量少都可以*/
        if (*(this->Carry) == 1 )
        {
            /* code */
            float temp[2], X_distance, Y_distance;
            X_distance = this->coord->current_coord_x - this->target_coord[0][*(this->current_obj_index)];
            Y_distance = this->coord->current_coord_y - this->target_coord[1][*(this->current_obj_index)];
            uint8_t min_index = *(this->current_obj_index);

            temp[0] = X_distance * X_distance + Y_distance * Y_distance;
            for (uint8_t i = *(this->current_obj_index) + 1; i < *(this->object_num); i++)
            {
                /* code */
                float temp_X_distance, temp_Y_distance;
                temp_X_distance = this->coord->current_coord_x - this->target_coord[0][i];
                temp_Y_distance = this->coord->current_coord_y - this->target_coord[1][i];
                temp[1] = temp_X_distance*temp_X_distance + temp_Y_distance*temp_Y_distance;
                if (temp[0] - temp[1] > FLT_MIN_SELF)
                {
                    /* code */
                    temp[0] = temp[1];
                    min_index = i;
                }
            }
            //temp无实际意义，交换数组，或者枚举中间介量
            temp[0] = this->target_coord[0][*this->current_obj_index];
            temp[1] = this->target_coord[1][*this->current_obj_index];
            this->target_coord[0][*this->current_obj_index] = this->target_coord[0][min_index];
            this->target_coord[1][*this->current_obj_index] = this->target_coord[1][min_index];
            this->target_coord[0][min_index] = temp[0];
            this->target_coord[1][min_index] = temp[1];
            if (*this->A4_select)
            {
                /* code ，交换数组的结果*/
                temp[0] = this->target_lable[0][*this->current_obj_index];
                temp[1] = this->target_lable[1][*this->current_obj_index];
                this->target_lable[0][*this->current_obj_index] = target_lable[0][min_index];
                this->target_lable[1][*this->current_obj_index] = target_lable[1][min_index];
                this->target_lable[0][min_index] = temp[0];
                this->target_lable[1][min_index] = temp[1];
            }
            this->coord->target_coord_x = this->target_coord[0][*this->current_obj_index];
            this->coord->target_coord_y = this->target_coord[1][*this->current_obj_index];
        }
        else if(*this->Carry == 0)
        {
//            if (*this->current_obj_index)
//            {
//                /* code */
//貌似是被废弃    this->coord->before_target_coord_x = (float)this->target_coord[0][this->city_result[*this->current_obj_index]];
//貌似是被废弃    this->coord->before_target_coord_y = (float)this->target_coord[1][this->city_result[*this->current_obj_index]];
//
//            }
            this->coord->target_coord_x = (float)this->target_coord[0][this->city_result[*this->current_obj_index]];//18个点6个位置
            this->coord->target_coord_y = (float)this->target_coord[1][this->city_result[*this->current_obj_index]];//18个点6个位置
            
        }
            this->coord->ahead_coord_x = this->coord->target_coord_x - AheadDist * cos(this->stance->car_yaw_radian);
            this->coord->ahead_coord_y = this->coord->target_coord_y - AheadDist * sin(this->stance->car_yaw_radian); 
//        if (*this->current_obj_index == 10)
//        {
//            /* code */
//            *this->current_obj_index = *this->object_num;
//            this->coord->target_coord_x = XEndPoint;
//            this->coord->target_coord_y = YEndPoint;
//            *this->target_update = false;
//            this->state_flags->mandatory_parked = false;
//        }
    }
    else//当前走完了所有标志
    {
        this->coord->target_coord_x = XEndPoint;  //回到结束地点
        this->coord->target_coord_y = YEndPoint;  //回到结束地点
        *this->target_update = false;
        this->state_flags->mandatory_parked = false;         
    }

}

/* 内部函数 */
/**
 * @description: 是否满足停车条件判断
 * @param {int8_t} current_obj_index
 * @return {*}
 */
uint8_t currentStageCompleteJudgment(struct location *this)
{
    /* 若存在搬运,target_coord另调函数改变 */
    if ((*this->current_obj_index) == -1)
    {
        this->nextRecentTargetDiscrimination(this);

        return 1;
    }
    // else if (target_update&&)
    // {
    //     coord.ahead_coord_x = coord.target_coord_x - AheadDist * cos(stance.car_yaw_radian), coord.ahead_coord_y = coord.target_coord_y - AheadDist * sin(stance.car_yaw_radian);
    //     if ((coord.current_coord_x - coord.ahead_coord_x) * (coord.current_coord_x - coord.ahead_coord_x) + (coord.current_coord_y - coord.ahead_coord_y) * (coord.current_coord_y - coord.ahead_coord_y) < Initial_park_error)
    //     {
    //         /* 停车 */
    //         state_flags.missing_target_flag = 1;
    //         PRINTF("g\n");
    //         missingTargetDisposs();
    //         state_flags.run_status_flag = 0;
    //         target_update = false;
    //         // nextRecentTargetDiscrimination();
    //         return 1;
    //     }
    // }
    else
    {
        return 0;
    }
}

/**
 * @description: 坐标体系状态更新
 * @param {*}
 * @return {*}
 */
void positioningSystemChange(struct location *this)
{
    if (*this->current_obj_index < *this->object_num)
    {
        /* code */
        if(!this->currentStageCompleteJudgment(this))
        {
            return;
        }
    }
    else
    {
        
    }
}

