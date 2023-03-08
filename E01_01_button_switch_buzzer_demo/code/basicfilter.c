#include "basicfilter.h"

/*
注意事项这里array都是为static对象，会一直保存在内存中，
每一次滤波都会把前面几次的结果考虑进去，是不是类似卷积。
结果是递进的，这样子看来滤波器只能对一个对象专门使用，如只针对IMU设计
*/

float meanMovingWindow(float current_value, uint8_t window_size)
{
    static float array[filter_length] = {0};

    for (uint8_t i = 0; i < filter_length - 1; i++)
    {
        //新一个时间段，东西往前移动
        array[i] = array[i + 1];
    }
    array[filter_length - 1] = current_value * window_size;
    /*
    加权均值滤波，这个过程也类似卷积
    */
    for (uint8_t i = filter_length - window_size, j = 1; i < filter_length - 1; i++, j++)
    {
        array[filter_length - 1] += j * array[i];
    }
    array[filter_length - 1] /= window_size * (window_size + 1) / 2;

    return array[filter_length - 1];
}

float meanMovingWindow_X(float current_value, uint8_t window_size)
{
    static float array[filter_length_X] = {0};

    for (uint8_t i = 0; i < filter_length_X - 1; i++)
    {
        array[i] = array[i + 1];
    }
    array[filter_length_X - 1] = current_value * window_size;

    for (uint8_t i = filter_length_X - window_size, j = 1; i < filter_length_X - 1; i++, j++)
    {
        array[filter_length_X - 1] += j * array[i];
    }
    array[filter_length_X - 1] /= window_size * (window_size + 1) / 2;

    return array[filter_length_X - 1];
}

float meanMovingWindow_Y(float current_value, uint8_t window_size)
{
    static float array[filter_length_Y] = {0};

    for (uint8_t i = 0; i < filter_length_Y - 1; i++)
    {
        array[i] = array[i + 1];
    }
    array[filter_length_Y - 1] = current_value * window_size;

    for (uint8_t i = filter_length_Y - window_size, j = 1; i < filter_length_Y - 1; i++, j++)
    {
        array[filter_length_Y - 1] += j * array[i];
    }
    array[filter_length_Y - 1] /= window_size * (window_size + 1) / 2;

    return array[filter_length_Y - 1];
}



float hullMovingHalfWindow(float current_value, uint8_t window_size)
{
    static float array[filter_length] = {0};

    for (uint8_t i = 0; i < filter_length - 1; i++)
    {
        array[i] = array[i + 1];
    }
    array[filter_length - 1] = current_value * window_size;

    for (uint8_t i = filter_length - window_size, j = 1; i < filter_length - 1; i++, j++)
    {
        array[filter_length - 1] += j * array[i];
    }
    array[filter_length - 1] /= window_size * (window_size + 1) / 2;

    return array[filter_length - 1];
}

float hullMovingfast_SqrtWindow(float current_value, uint8_t window_size)
{
    static float array[filter_length] = {0};

    for (uint8_t i = 0; i < filter_length - 1; i++)
    {
        array[i] = array[i + 1];
    }
    array[filter_length - 1] = current_value * window_size;

    for (uint8_t i = filter_length - window_size, j = 1; i < filter_length - 1; i++, j++)
    {
        array[filter_length - 1] += j * array[i];
    }
    array[filter_length - 1] /= window_size * (window_size + 1) / 2;

    return array[filter_length - 1];
}

float hullMovingWindow(float current_value)
{
    static float array[filter_length] = {0};

    for (uint8_t i = 0; i < filter_length - 1; i++)
    {
        array[i] = array[i + 1];
    }
    uint8_t window_size = fast_sqrt(filter_length);
    //两个半窗-全窗，最后值在进入
    array[filter_length - 1] = 2 * hullMovingHalfWindow(current_value, filter_length / 2) - meanMovingWindow(current_value, filter_length);

    return hullMovingfast_SqrtWindow(array[filter_length - 1], window_size);
}

float hullMovingHalfWindow_X(float current_value, uint8_t window_size)
{
    static float array[filter_length_X] = {0};

    for (uint8_t i = 0; i < filter_length_X - 1; i++)
    {
        array[i] = array[i + 1];
    }
    array[filter_length_X - 1] = current_value * window_size;

    for (uint8_t i = filter_length_X - window_size, j = 1; i < filter_length_X - 1; i++, j++)
    {
        array[filter_length_X - 1] += j * array[i];
    }
    array[filter_length_X - 1] /= window_size * (window_size + 1) / 2;

    return array[filter_length_X - 1];
}
float hullMovingfast_SqrtWindow_X(float current_value, uint8_t window_size)
{
    static float array[filter_length_X] = {0};

    for (uint8_t i = 0; i < filter_length_X - 1; i++)
    {
        array[i] = array[i + 1];
    }
    array[filter_length_X - 1] = current_value * window_size;

    for (uint8_t i = filter_length_X - window_size, j = 1; i < filter_length_X - 1; i++, j++)
    {
        array[filter_length_X - 1] += j * array[i];
    }
    array[filter_length_X - 1] /= window_size * (window_size + 1) / 2;

    return array[filter_length_X - 1];
}
float hullMovingWindow_X(float current_value)
{
    static float array[filter_length_X] = {0};

    for (uint8_t i = 0; i < filter_length_X - 1; i++)
    {
        array[i] = array[i + 1];
    }
    uint8_t window_size = fast_sqrt(filter_length_X);
    array[filter_length_X - 1] = 2 * hullMovingHalfWindow_X(current_value, filter_length_X / 2) - meanMovingWindow_X(current_value, filter_length_X);

    return hullMovingfast_SqrtWindow_X(array[filter_length_X - 1], window_size);
}

float hullMovingHalfWindow_Y(float current_value, uint8_t window_size)
{
    static float array[filter_length_Y] = {0};

    for (uint8_t i = 0; i < filter_length_Y - 1; i++)
    {
        array[i] = array[i + 1];
    }
    array[filter_length_Y - 1] = current_value * window_size;

    for (uint8_t i = filter_length_Y - window_size, j = 1; i < filter_length_Y - 1; i++, j++)
    {
        array[filter_length_Y - 1] += j * array[i];
    }
    array[filter_length_Y - 1] /= window_size * (window_size + 1) / 2;

    return array[filter_length_Y - 1];
}
float hullMovingfast_SqrtWindow_Y(float current_value, uint8_t window_size)
{
    static float array[filter_length_Y] = {0};

    for (uint8_t i = 0; i < filter_length_Y - 1; i++)
    {
        array[i] = array[i + 1];
    }
    array[filter_length_Y - 1] = current_value * window_size;

    for (uint8_t i = filter_length_Y - window_size, j = 1; i < filter_length_Y - 1; i++, j++)
    {
        array[filter_length_Y - 1] += j * array[i];
    }
    array[filter_length_Y - 1] /= window_size * (window_size + 1) / 2;

    return array[filter_length_Y - 1];
}
float hullMovingWindow_Y(float current_value)
{
    static float array[filter_length_Y] = {0};

    for (uint8_t i = 0; i < filter_length_Y - 1; i++)
    {
        array[i] = array[i + 1];
    }
    uint8_t window_size = fast_sqrt(filter_length_Y);
    array[filter_length_Y - 1] = 2 * hullMovingHalfWindow_Y(current_value, filter_length_Y / 2) - meanMovingWindow_Y(current_value, filter_length_Y);

    return hullMovingfast_SqrtWindow_Y(array[filter_length_Y - 1], window_size);
}

float highPassFIRFilter(float current_value)
{
#define HIGH_FILTER_ORDER 2                                                                                                  //滤波器阶数定义
    const double out_IIR_coefficient[HIGH_FILTER_ORDER + 1] = {0.9982244250264, -1.998222847292, 1}; /* 降序 */              //{1,  -0.9824057931084,   0.3476653948517};
    const double in_IIR_coefficient[HIGH_FILTER_ORDER + 1] = {0.9991118180796, -1.998223636159, 0.9991118180796}; /* 降序 */ //{0.58251779699,    -1.16503559398,     0.58251779699};

    static float out_array[HIGH_FILTER_ORDER + 1] = {0};
    static float in_array[HIGH_FILTER_ORDER + 1] = {0};

    for (uint8_t i = 0; i < HIGH_FILTER_ORDER; i++)
    {
        in_array[i] = in_array[i + 1];
        out_array[i] = out_array[i + 1];
    }
    in_array[HIGH_FILTER_ORDER] = current_value;
    out_array[HIGH_FILTER_ORDER] = 0;
    //计算输出滤波数值
    for (uint8_t i = 0; i < HIGH_FILTER_ORDER; i++)
    {
        //同时计算过去，和当前进来的影响，和过去出来的影响
        out_array[HIGH_FILTER_ORDER] += (in_IIR_coefficient[i] * in_array[i]);
        out_array[HIGH_FILTER_ORDER] -= (out_IIR_coefficient[i] * out_array[i]);
    }
    out_array[HIGH_FILTER_ORDER] += (in_IIR_coefficient[HIGH_FILTER_ORDER] * in_array[HIGH_FILTER_ORDER]);
    out_array[HIGH_FILTER_ORDER] /= out_IIR_coefficient[HIGH_FILTER_ORDER];

    return out_array[HIGH_FILTER_ORDER];
}

float lowPassFIRFilter(float current_value)
{
#define LOW_FILTER_ORDER 2                                                                                                       //滤波器阶数定义
    const double out_IIR_coefficient[LOW_FILTER_ORDER + 1] = {0.7436551950489, -1.705552145544, 1}; /* 降序 */                   //{1, -1.705552145544, 0.7436551950489};
    const double in_IIR_coefficient[LOW_FILTER_ORDER + 1] = {0.009525762376195, 0.01905152475239, 0.009525762376195}; /* 降序 */ //{0.009525762376195, 0.01905152475239, 0.009525762376195};

    static float out_array[LOW_FILTER_ORDER + 1] = {0};
    static float in_array[LOW_FILTER_ORDER + 1] = {0};

    for (uint8_t i = 0; i < LOW_FILTER_ORDER; i++)
    {
        in_array[i] = in_array[i + 1];
        out_array[i] = out_array[i + 1];
    }
    in_array[LOW_FILTER_ORDER] = current_value;
    out_array[LOW_FILTER_ORDER] = 0;
    //计算输出滤波数值
    for (uint8_t i = 0; i < LOW_FILTER_ORDER; i++)
    {
        //同时计算过去，和当前进来的影响，和过去出来的影响
        out_array[LOW_FILTER_ORDER] += (in_IIR_coefficient[i] * in_array[i]);
        out_array[LOW_FILTER_ORDER] -= (out_IIR_coefficient[i] * out_array[i]);
    }

    out_array[LOW_FILTER_ORDER] += (in_IIR_coefficient[LOW_FILTER_ORDER] * in_array[LOW_FILTER_ORDER]);
    out_array[LOW_FILTER_ORDER] /= out_IIR_coefficient[LOW_FILTER_ORDER];

    return out_array[LOW_FILTER_ORDER];
}

/// @attention 源代码出现的卡尔曼滤波均被注释或者没有使用，以下仅作为参考使用

/**
 * @description: kalman原型，一个对象单位为另一对象单位积分(暂用于加速度计X方向)
 * @param {float} prolonged_credibility
 * @param {float} temporary_credibility
 * @return {*}
 */
float kalmanFilter_X(float prolonged_credibility, float temporary_credibility)
{
    static float bias = 0.0f;
    static float Q_object_property = 0.001f;
    static float Q_temporary_bias = 0.0015f;
    static float R_measure = 0.1f;
    static float object_property = 0.0f; // Reset the object_property

    static float P[2][2] = {{1.0, 0.0},
                            {0.0, 1.0}};

    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    object_property += position_time_resolution * (temporary_credibility - bias);

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += position_time_resolution * (position_time_resolution * P[1][1] - P[0][1] - P[1][0] + Q_object_property);
    P[0][1] -= position_time_resolution * P[1][1];
    P[1][0] -= position_time_resolution * P[1][1];
    P[1][1] += Q_temporary_bias * position_time_resolution;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate object_property and bias - Update estimate with measurement zk (prolonged_credibility)
    /* Step 3 */
    float y = prolonged_credibility - object_property; // object_property difference
    /* Step 6 */
    object_property += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return object_property;
}

/**
 * @description: kalman原型，一个对象单位为另一对象单位积分(暂用于加速度计Y方向)
 * @param {float} prolonged_credibility
 * @param {float} temporary_credibility
 * @return {*}
 */
float kalmanFilter_Y(float prolonged_credibility, float temporary_credibility)
{
    static float bias = 0.0f;
    static float Q_object_property = 0.001f;
    static float Q_temporary_bias = 0.0015f;
    float R_measure = 0.01f;
    static float object_property = 0.0f; // Reset the object_property

    static float P[2][2] = {{1.0, 0.0},
                            {0.0, 1.0}};
    static float b_prolonged_credibility = 0;
    R_measure += (prolonged_credibility - b_prolonged_credibility) / 100.0;
    b_prolonged_credibility = prolonged_credibility;
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    object_property += position_time_resolution * (temporary_credibility - bias);

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += position_time_resolution * (position_time_resolution * P[1][1] - P[0][1] - P[1][0] + Q_object_property);
    P[0][1] -= position_time_resolution * P[1][1];
    P[1][0] -= position_time_resolution * P[1][1];
    P[1][1] += Q_temporary_bias * position_time_resolution;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate object_property and bias - Update estimate with measurement zk (prolonged_credibility)
    /* Step 3 */
    float y = prolonged_credibility - object_property; // object_property difference
    /* Step 6 */
    object_property += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return object_property;
}

/**
 * @description: kalman用于同单位对象滤波
 * @param {float} prolonged_credibility
 * @param {float} temporary_credibility
 * @return {*}
 */
float kalmanFilterHomogeneous(float prolonged_credibility, float temporary_credibility)
{
    static float bias = 0.0f;
    static float Q_object_property = 0.001f;
    static float Q_temporary_bias = 0.0015f;
    static float R_measure = 0.1f;
    static float object_property = 0.0f; // Reset the object_property

    static float P[2][2] = {{1.0, 0.0},
                            {0.0, 1.0}};

    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    object_property += (temporary_credibility - bias);

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += (position_time_resolution * P[1][1] - P[0][1] - P[1][0] + Q_object_property);
    P[0][1] -= P[1][1];
    P[1][0] -= P[1][1];
    P[1][1] += Q_temporary_bias;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate object_property and bias - Update estimate with measurement zk (prolonged_credibility)
    /* Step 3 */
    float y = prolonged_credibility - object_property; // object_property difference
    /* Step 6 */
    object_property += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return object_property;
}

/**
 * @description: 陀螺仪与编码器等效角速度卡尔曼滤波（不太行）
 * @param {float} gyro_speed
 * @param {float} enc_speed
 * @return {*}
 */
float kalman_gyro_enc_combine(float gyro_speed, float enc_speed)
{
    static float P_angle = 0.001, K_angle = 0.0, Q_angle = 0.001, R_angle = 0.01;
    static float angle = 0;

    P_angle += Q_angle;
    K_angle = P_angle / (P_angle + R_angle);
    P_angle = (1 - K_angle) * P_angle;
    angle += (gyro_speed + K_angle * (enc_speed - gyro_speed)) * position_time_resolution; // angle,应该是speed？
    return angle;
}

/**
 * @description: 单输入卡尔曼滤波
 * @param {float} input
 * @return {*}
 */
float kalman_single_object(float input)
{
    const float Q = 10, R = 1.5;
    static float LastP = 0, Now_P = 0, Kg = 0, out = 0;

    //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    Now_P = LastP + Q;
    //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    Kg = Now_P / (Now_P + R);
    //更新协方差方程: 本次的系统协方差付给 LastP 为下一次运算准备。
    LastP = (1 - Kg) * Now_P;
    //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    out += Kg * (input - out); //因为这一次的预测值就是上一次的输出值

    return out;
}

/* LMS算法 */
/*
 * 算法递推形式：
 * 估计瞬时误差：        e(k) = d(k) - x'(k)w(k)      (1)
 * 估计滤波系数矢量：w(k+1) = w(k) + 2 niu e(k)x(k)   (2)
 * 初始条件：                w(0) = 0;
 */
/* xn--------输入的信号序列（列向量）
 * itr-------迭代次数，标量，默认为xn的长度，M<itr<sizeof(xn)
 * en--------误差序列（itr*1）列向量
 * dn--------所期望的响应序列（列向量）
 * M---------滤波器的阶数（标量）
 * mu--------收敛因子（步长）标量
 * W---------滤波器权值矩阵，大小为M*itr
 * yn--------实际输出序列（列向量）*/

/*LMS具体算法*/
float *LMS_Filter(int itr, const float *xn, const float *dn, double mu, int length)
{
    static int i = 0;
    static int k = 0;
    static float y = 0.0;
    static float en[F_COUNT];
    static float W[MM][F_COUNT];
    static float x[MM];
    static float yn[F_COUNT];

    /*创建一个en全零矩阵，en(k)表示第k次迭代时预期输出与实际输入的误差*/
    for (i = 0; i < itr; i++)
    {
        en[i] = 0;
    }

    /*创建一个W全零矩阵，每一行代表一个加权参量，每一列代表一次迭代*/
    for (i = 0; i < MM; i++)
        for (k = 0; k < itr; k++)
            W[i][k] = 0;

    /*创建一个x全零矩阵*/
    for (i = 0; i < MM; i++)
        x[i] = 0;

    /*迭代计算*/
    for (k = MM; k <= itr; k++)
    {
        /* 滤波器M个抽头的输入：从xn第k-1个值倒序取出M个样点的值放入x
         * y为滤波器输出：W的第K-2列与x的积的和*/
        for (i = 0; i < MM; i++)
        {
            x[i] = xn[k - i - 1];
            y += W[i][k - 2] * x[i];
        }

        en[k - 1] = dn[k - 1] - y; //第k次迭代的误差

        /*滤波器权值计算的迭代式*/
        for (i = 0; i < MM; i++)
        {
            W[i][k - 1] = W[i][k - 2] + 2 * mu * en[k - 1] * x[i];
        }

        y = 0.0;
    }

    /*创建一个yn全无穷大矩阵，维数与xn一样*/
    for (i = 0; i < itr; i++)
    {
        yn[i] = 0.0;
    }

    /*求最优时滤波器的输出序列*/
    for (k = MM; k <= length; k++)
    {
        for (i = 0; i < MM; i++)
        {
            x[i] = xn[k - i - 1];
            y += W[i][itr - 1] * x[i];
        }

        yn[k - 1] = y;
        y = 0.0;
    }

    return yn;
}
