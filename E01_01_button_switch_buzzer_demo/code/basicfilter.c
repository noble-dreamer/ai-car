#include "basicfilter.h"

/*
ע����������array����Ϊstatic���󣬻�һֱ�������ڴ��У�
ÿһ���˲������ǰ�漸�εĽ�����ǽ�ȥ���ǲ������ƾ����
����ǵݽ��ģ������ӿ����˲���ֻ�ܶ�һ������ר��ʹ�ã���ֻ���IMU���
*/

float meanMovingWindow(float current_value, uint8_t window_size)
{
    static float array[filter_length] = {0};

    for (uint8_t i = 0; i < filter_length - 1; i++)
    {
        //��һ��ʱ��Σ�������ǰ�ƶ�
        array[i] = array[i + 1];
    }
    array[filter_length - 1] = current_value * window_size;
    /*
    ��Ȩ��ֵ�˲����������Ҳ���ƾ��
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
    //�����봰-ȫ�������ֵ�ڽ���
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
#define HIGH_FILTER_ORDER 2                                                                                                  //�˲�����������
    const double out_IIR_coefficient[HIGH_FILTER_ORDER + 1] = {0.9982244250264, -1.998222847292, 1}; /* ���� */              //{1,  -0.9824057931084,   0.3476653948517};
    const double in_IIR_coefficient[HIGH_FILTER_ORDER + 1] = {0.9991118180796, -1.998223636159, 0.9991118180796}; /* ���� */ //{0.58251779699,    -1.16503559398,     0.58251779699};

    static float out_array[HIGH_FILTER_ORDER + 1] = {0};
    static float in_array[HIGH_FILTER_ORDER + 1] = {0};

    for (uint8_t i = 0; i < HIGH_FILTER_ORDER; i++)
    {
        in_array[i] = in_array[i + 1];
        out_array[i] = out_array[i + 1];
    }
    in_array[HIGH_FILTER_ORDER] = current_value;
    out_array[HIGH_FILTER_ORDER] = 0;
    //��������˲���ֵ
    for (uint8_t i = 0; i < HIGH_FILTER_ORDER; i++)
    {
        //ͬʱ�����ȥ���͵�ǰ������Ӱ�죬�͹�ȥ������Ӱ��
        out_array[HIGH_FILTER_ORDER] += (in_IIR_coefficient[i] * in_array[i]);
        out_array[HIGH_FILTER_ORDER] -= (out_IIR_coefficient[i] * out_array[i]);
    }
    out_array[HIGH_FILTER_ORDER] += (in_IIR_coefficient[HIGH_FILTER_ORDER] * in_array[HIGH_FILTER_ORDER]);
    out_array[HIGH_FILTER_ORDER] /= out_IIR_coefficient[HIGH_FILTER_ORDER];

    return out_array[HIGH_FILTER_ORDER];
}

float lowPassFIRFilter(float current_value)
{
#define LOW_FILTER_ORDER 2                                                                                                       //�˲�����������
    const double out_IIR_coefficient[LOW_FILTER_ORDER + 1] = {0.7436551950489, -1.705552145544, 1}; /* ���� */                   //{1, -1.705552145544, 0.7436551950489};
    const double in_IIR_coefficient[LOW_FILTER_ORDER + 1] = {0.009525762376195, 0.01905152475239, 0.009525762376195}; /* ���� */ //{0.009525762376195, 0.01905152475239, 0.009525762376195};

    static float out_array[LOW_FILTER_ORDER + 1] = {0};
    static float in_array[LOW_FILTER_ORDER + 1] = {0};

    for (uint8_t i = 0; i < LOW_FILTER_ORDER; i++)
    {
        in_array[i] = in_array[i + 1];
        out_array[i] = out_array[i + 1];
    }
    in_array[LOW_FILTER_ORDER] = current_value;
    out_array[LOW_FILTER_ORDER] = 0;
    //��������˲���ֵ
    for (uint8_t i = 0; i < LOW_FILTER_ORDER; i++)
    {
        //ͬʱ�����ȥ���͵�ǰ������Ӱ�죬�͹�ȥ������Ӱ��
        out_array[LOW_FILTER_ORDER] += (in_IIR_coefficient[i] * in_array[i]);
        out_array[LOW_FILTER_ORDER] -= (out_IIR_coefficient[i] * out_array[i]);
    }

    out_array[LOW_FILTER_ORDER] += (in_IIR_coefficient[LOW_FILTER_ORDER] * in_array[LOW_FILTER_ORDER]);
    out_array[LOW_FILTER_ORDER] /= out_IIR_coefficient[LOW_FILTER_ORDER];

    return out_array[LOW_FILTER_ORDER];
}

/// @attention Դ������ֵĿ������˲�����ע�ͻ���û��ʹ�ã����½���Ϊ�ο�ʹ��

/**
 * @description: kalmanԭ�ͣ�һ������λΪ��һ����λ����(�����ڼ��ٶȼ�X����)
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
 * @description: kalmanԭ�ͣ�һ������λΪ��һ����λ����(�����ڼ��ٶȼ�Y����)
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
 * @description: kalman����ͬ��λ�����˲�
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
 * @description: ���������������Ч���ٶȿ������˲�����̫�У�
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
    angle += (gyro_speed + K_angle * (enc_speed - gyro_speed)) * position_time_resolution; // angle,Ӧ����speed��
    return angle;
}

/**
 * @description: �����뿨�����˲�
 * @param {float} input
 * @return {*}
 */
float kalman_single_object(float input)
{
    const float Q = 10, R = 1.5;
    static float LastP = 0, Now_P = 0, Kg = 0, out = 0;

    //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
    Now_P = LastP + Q;
    //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
    Kg = Now_P / (Now_P + R);
    //����Э�����: ���ε�ϵͳЭ����� LastP Ϊ��һ������׼����
    LastP = (1 - Kg) * Now_P;
    //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
    out += Kg * (input - out); //��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ

    return out;
}

/* LMS�㷨 */
/*
 * �㷨������ʽ��
 * ����˲ʱ��        e(k) = d(k) - x'(k)w(k)      (1)
 * �����˲�ϵ��ʸ����w(k+1) = w(k) + 2 niu e(k)x(k)   (2)
 * ��ʼ������                w(0) = 0;
 */
/* xn--------������ź����У���������
 * itr-------����������������Ĭ��Ϊxn�ĳ��ȣ�M<itr<sizeof(xn)
 * en--------������У�itr*1��������
 * dn--------����������Ӧ���У���������
 * M---------�˲����Ľ�����������
 * mu--------�������ӣ�����������
 * W---------�˲���Ȩֵ���󣬴�СΪM*itr
 * yn--------ʵ��������У���������*/

/*LMS�����㷨*/
float *LMS_Filter(int itr, const float *xn, const float *dn, double mu, int length)
{
    static int i = 0;
    static int k = 0;
    static float y = 0.0;
    static float en[F_COUNT];
    static float W[MM][F_COUNT];
    static float x[MM];
    static float yn[F_COUNT];

    /*����һ��enȫ�����en(k)��ʾ��k�ε���ʱԤ�������ʵ����������*/
    for (i = 0; i < itr; i++)
    {
        en[i] = 0;
    }

    /*����һ��Wȫ�����ÿһ�д���һ����Ȩ������ÿһ�д���һ�ε���*/
    for (i = 0; i < MM; i++)
        for (k = 0; k < itr; k++)
            W[i][k] = 0;

    /*����һ��xȫ�����*/
    for (i = 0; i < MM; i++)
        x[i] = 0;

    /*��������*/
    for (k = MM; k <= itr; k++)
    {
        /* �˲���M����ͷ�����룺��xn��k-1��ֵ����ȡ��M�������ֵ����x
         * yΪ�˲��������W�ĵ�K-2����x�Ļ��ĺ�*/
        for (i = 0; i < MM; i++)
        {
            x[i] = xn[k - i - 1];
            y += W[i][k - 2] * x[i];
        }

        en[k - 1] = dn[k - 1] - y; //��k�ε��������

        /*�˲���Ȩֵ����ĵ���ʽ*/
        for (i = 0; i < MM; i++)
        {
            W[i][k - 1] = W[i][k - 2] + 2 * mu * en[k - 1] * x[i];
        }

        y = 0.0;
    }

    /*����һ��ynȫ��������ά����xnһ��*/
    for (i = 0; i < itr; i++)
    {
        yn[i] = 0.0;
    }

    /*������ʱ�˲������������*/
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
