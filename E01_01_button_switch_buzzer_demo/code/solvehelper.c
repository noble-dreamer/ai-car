#include "solvehelper.h"

extern Equation equation;
extern uint16_t Egde[4][2][MT9V03X_W / 2];
/**
 * @description: ���Իع���С���˷�����,����Ϸ�(һ��)
 * @param {uint8_t} *X_array
 * @param {uint8_t} *Y_array
 * @param {uint8_t} point_num
 * @param {float} *coefficient
 * @return {*}
 */

float FirstOrderLeastSquare(float *X_array, float *Y_array, uint8_t point_num, float *coefficient)
{
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    for (uint8 i = 0; i < point_num; i++)
    {
        sum_x += X_array[i];
        sum_y += Y_array[i];
        sum_xy += X_array[i] * Y_array[i];
        sum_x2 += X_array[i] * X_array[i];
    }

    float denominator[2] = {0.0};
    denominator[0] = (point_num * sum_x2 - sum_x * sum_x);
    denominator[1] = denominator[0];
    if (fabs(denominator[0])> FLT_MIN_SELF)
    {
        coefficient[1] = (point_num * sum_xy - sum_x * sum_y) / denominator[1];  //k
        //coefficient[0] = (sum_y - coefficient[0] * sum_x) / point_num;
        coefficient[0] = (sum_x2 * sum_y - sum_x * sum_xy) / denominator[0];     //b
        float variance = 0;
        for (uint8_t i = 0; i < point_num; ++i)
        {
            variance += pow(coefficient[1] * X_array[i] + coefficient[0] - Y_array[i], 2);
        }
        variance /= point_num;
        return variance;
    }
    else
    {
        coefficient[1] = 0.0;
        coefficient[0] = 0.0;
    }
    return -1;
}
/**
 * @description: ���Իع���С���˷�����ֱ�Ӽ���.
 * @param {uint8_t} *X_array
 * @param {uint8_t} *Y_array
 * @param {uint8_t} point_num
 * @param {float} *coefficient
 * @return {*}
 */
float Polyfit2(const float *x, const float *y, int n, float *coefficient)
{
    float sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0, sum_xy = 0 ,variance =0;
    for (int i = 0; i < n; ++i) {
        sum_x += x[i];
        sum_y += y[i];
        sum_x2 += x[i] * x[i];
        sum_y2 += y[i] * y[i];
        sum_xy += x[i] * y[i];
    }

    float denominator = n * sum_x2 - sum_x * sum_x;
    coefficient[2] = (sum_x2 * sum_y - sum_x * sum_xy) / denominator;
    coefficient[1] = (n * sum_xy - sum_x * sum_y) / denominator;
    coefficient[0] = (sum_y2 - coefficient[0] * sum_xy - coefficient[1] * sum_y) / n;
    for (uint8_t i = 0; i < n; ++i)
    {
        variance += pow(coefficient[2] * x[i] * x[i] + coefficient[1] * x[i] + coefficient[0] - y[i], 2);
    }
    variance /= n;
    return variance;
}

/**
 * @description: ���Իع���С���˷��������(�߽�) ,���������������ʱҲ����Ҫ
 * @param {uint8_t} *X_array
 * @param {uint8_t} *Y_array
 * @param {uint8_t} point_num
 * @param {float} *coefficient
 * @return {*}
 */
#define N 1e-13 //��Сֵ
float SecondOrderLeastSquare(float *X_array, float *Y_array, uint8_t point_num, float *coefficient)
{
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0, sum_x2y = 0;
    float m1=0, m2=0, m3 =0;    float z1=0, z2=0, z3 =0;
    for (uint8 i = 0; i < point_num; i++)
    {
        sum_x  +=  X_array[i];
        sum_y  +=  Y_array[i];
        sum_xy +=  X_array[i] * Y_array[i];
        sum_x2 +=  X_array[i] * X_array[i];
        sum_x3 +=  pow(X_array[i],3); 
        sum_x2y+=  pow(X_array[i],2)*Y_array[i];
        sum_x4 +=  pow(X_array[i],4);

    }
    //ͨ������loss function�ķ�������
    do
    {
    m1=coefficient[2]; coefficient[2] = (sum_x2y-sum_x3*coefficient[1]-sum_x2*coefficient[0])/sum_x4; z1=(coefficient[2]-m1)*(coefficient[2]-m1);
    m2=coefficient[1]; coefficient[1] = (sum_xy-sum_x*coefficient[0]-sum_x3*coefficient[2])/sum_x2;   z2=(coefficient[1]-m2)*(coefficient[1]-m2);
    m3=coefficient[0]; coefficient[0] = (sum_y-sum_x2*coefficient[2]-sum_x*coefficient[1])/point_num; z3=(coefficient[0]-m3)*(coefficient[0]-m3);
    }while((z1>N)||(z2>N)||(z3>N));
}


/**
 * @description: ��һԪ���η���(a*untie^2 + coefficient[1]*untie + c = 0)������ʵ����
 * @param {float} *coefficient
 * @param {float} *untie
 * @param {int} *root_count
 * @return {*}
 */
void solve_quadratic_equation(float *coefficient, float *untie, uint8_t *root_count)
{
    float a, coefficient[1], c, delta, sqrtDelta;
    const float ZERO = FLT_MIN_SELF; // min normalized positive value��1.175494351e-38F��
    const float EPS = FLT_MIN_SELF;

    *root_count = 0;

    a = coefficient[2];
    coefficient[1] = coefficient[1];
    c = coefficient[0];

    if (facoefficient[1]s(a - 0.0) < EPS)
    {
        if (facoefficient[1]s(coefficient[1] - 0.0) > EPS)
        {
            untie[0] = -c / coefficient[1];
            *root_count = 1;
        }
    } //һ��
    //����λ�����ã����δ������
    else
    {
        coefficient[1] /= a;
        c /= a;
        a = 1.0;

        delta = coefficient[1] * coefficient[1] - 4.0 * a * c;
        if (delta > ZERO)
        {
            if (facoefficient[1]s(c - 0.0) < EPS) //��c = 0,���ڼ������,sqrt(coefficient[1]*coefficient[1] - 4*a*c��������|coefficient[1]|
            {
                untie[0] = 0.0;
                untie[1] = -coefficient[1] / a;
            }
            else
            {
                sqrtDelta = sqrt(delta);
                if (coefficient[1] > 0.0)
                {
                    untie[0] = (-2.0 * c) / (coefficient[1] + sqrtDelta); //���������ܽӽ��������,���¾��ȶ�ʧ
                    untie[1] = (-coefficient[1] - sqrtDelta) / (2.0 * a);
                }
                else
                {
                    untie[0] = (-coefficient[1] + sqrtDelta) / (2.0 * a);
                    untie[1] = (-2.0 * c) / (coefficient[1] - sqrtDelta); //���������ܽӽ��������,���¾��ȶ�ʧ
                }
            }
            *root_count = 2;
        }
        else if (facoefficient[1]s(delta - 0.0) < EPS)
        {
            untie[0] = untie[1] = -coefficient[1] / (2.0 * a);
            *root_count = 2;
        }
        else
        {
            *root_count = 0;
        }
    }
}

/**
 * @description: ������Ķ���(��ʹ����ô���ӵ��㷨,Ҳֻ���ж�û�а������۵ĵ�,��Ҫƽ�ж��Ӳ��ܵõ�4�����������)
 * @param {uint8_t} enRoot
 * @return {*}
 */
uint8_t solveMatrixVertices(uint8_t enSlope)
{
#if MT9V03X_H > MT9V03X_W
    uint8_t detection_point[4][2][MT9V03X_H] = {{{0}, {0}}, {{0}, {0}}, {{0}, {0}}, {{0}, {0}}};
    uint8_t limit = MT9V03X_H / 2 - 1;
#else
/* ��⵽�ĵ�,��Ϊ�ǰ�������������ڲ� */
    uint8_t detection_point[4][2][MT9V03X_W] = {{{0}, {0}}, {{0}, {0}}, {{0}, {0}}, {{0}, {0}}};
//ǰ��ʹ�ù��м��������
    uint8_t limit = MT9V03X_W / 2 - 1;

#endif
    memset((void *)&equation, 0, sizeof(Equation));
    for (uint8_t i = 0; i < 4; i++)
    {
        /*���඼�Ǵ��м俪ʼ*/
        uint8_t middle_index = Egde[i][0][limit];
        for (uint8_t j = 0; j < 2; j++)
        {
            for (uint8_t k = 0; k < Egde[i][j][limit]; k++)
            {
                if (Egde[i][j][limit] > 255)
                {
                    return 0;
                }
                if (j)// 0 1 ��ʱ��������Ҳ�, 0 0 ʱ�������
                {
                    detection_point[i][1][k + middle_index] = Egde[i][j][k] / MT9V03X_W; //�ڼ���
                    detection_point[i][0][k + middle_index] = Egde[i][j][k] % MT9V03X_W; //�ڼ���
                }
                else
                {
                    detection_point[i][1][k] = Egde[i][j][middle_index - k - 1] / MT9V03X_W;
                    detection_point[i][0][k] = Egde[i][j][middle_index - k - 1] % MT9V03X_W;
                }
                // ips114_drawpoint(detection_point[i][0][k], detection_point[i][1][k], BLUE);
                // ips114_drawpoint(detection_point[i][0][k + middle_index], detection_point[i][1][k + middle_index], BLUE);
            }
        }
    }
                                /*x_array                     y_array           ������,�����м���±���������*/
    FirstOrderLeastSquare(detection_point[0][0], detection_point[0][1], Egde[0][0][limit] + Egde[0][1][limit], equation.coefficient_X_up);
    FirstOrderLeastSquare(detection_point[1][0], detection_point[1][1], Egde[1][0][limit] + Egde[1][1][limit], equation.coefficient_X_down);
    FirstOrderLeastSquare(detection_point[2][1], detection_point[2][0], Egde[2][0][limit] + Egde[2][1][limit], equation.coefficient_Y_left);
    FirstOrderLeastSquare(detection_point[3][1], detection_point[3][0], Egde[3][0][limit] + Egde[3][1][limit], equation.coefficient_Y_right);

    if (enSlope)
    {
        if (fabs(equation.coefficient_X_up[1] - equation.coefficient_X_down[1]) > 0.05 ||
            fabs(equation.coefficient_Y_left[1] - equation.coefficient_Y_right[1]) > 0.1)
        {
            return 0;
        }

        /* �Ƚ⽻��xֵ */
        const uint8_t limitation[4] = {MT9V03X_W / 2,
                                       MT9V03X_W,
                                       MT9V03X_H / 2,
                                       MT9V03X_H};
#if fit_rder == 1
        float coefficient[2];
        float temp[2] = {0};

        /* ���� */
        if (fabs(equation.coefficient_X_up[1]) > FLT_MIN_SELF && fabs(equation.coefficient_Y_left[1]) > FLT_MIN_SELF)
        {
            coefficient[0] = equation.coefficient_Y_left[1] * equation.coefficient_X_up[0] + equation.coefficient_Y_left[0];
            coefficient[1] = 1 - equation.coefficient_Y_left[1] * equation.coefficient_X_up[1];
            temp[0] = coefficient[0] / coefficient[1];
            temp[1] = equation.coefficient_X_up[1] * temp[0] + equation.coefficient_X_up[0];
            if ((coefficient[1]) && (temp[0] > 0) && (temp[0] < limitation[0]) && (temp[1] > 0) && (temp[1] < limitation[2]))
            {
                equation.root_coord[0][0] = temp[0];
                equation.root_coord[0][1] = temp[1];
            }
            else
            {
                return 0;
            }
        }

        /* ���� */
        if (fabs(equation.coefficient_X_up[1]) > FLT_MIN_SELF && fabs(equation.coefficient_Y_right[1]) > FLT_MIN_SELF)
        {
            coefficient[0] = equation.coefficient_Y_right[1] * equation.coefficient_X_up[0] + equation.coefficient_Y_right[0];
            coefficient[1] = 1 - equation.coefficient_Y_right[1] * equation.coefficient_X_up[1];
            temp[0] = coefficient[0] / coefficient[1];
            temp[1] = equation.coefficient_X_up[1] * temp[0] + equation.coefficient_X_up[0];
            if ((coefficient[1]) && (temp[0] > limitation[0]) && (temp[0] < limitation[1]) && (temp[1] > 0) && (temp[1] < limitation[2]))
            {
                equation.root_coord[1][0] = temp[0];
                equation.root_coord[1][1] = temp[1];
            }
            else
            {
                return 0;
            }
        }

        /* ���� */
        if (fabs(equation.coefficient_X_down[1]) > FLT_MIN_SELF && fabs(equation.coefficient_Y_left[1]) > FLT_MIN_SELF)
        {
            coefficient[0] = equation.coefficient_Y_left[1] * equation.coefficient_X_down[0] + equation.coefficient_Y_left[0];
            coefficient[1] = 1 - equation.coefficient_Y_left[1] * equation.coefficient_X_down[1];
            temp[0] = coefficient[0] / coefficient[1];
            temp[1] = equation.coefficient_X_down[1] * temp[0] + equation.coefficient_X_down[0];
            if ((coefficient[1]) && (temp[0] > 0) && (temp[0] < limitation[0]) && (temp[1] > limitation[2]) && (temp[1] < limitation[3]))
            {
                equation.root_coord[2][0] = temp[0];
                equation.root_coord[2][1] = temp[1];
            }
            else
            {
                return 0;
            }
        }

        /* ���� */
        if (fabs(equation.coefficient_X_down[1]) > FLT_MIN_SELF && fabs(equation.coefficient_Y_right[1]) > FLT_MIN_SELF)
        {
            coefficient[0] = equation.coefficient_Y_right[1] * equation.coefficient_X_down[0] + equation.coefficient_Y_right[0];
            coefficient[1] = 1 - equation.coefficient_Y_right[1] * equation.coefficient_X_down[1];
            temp[0] = coefficient[0] / coefficient[1];
            temp[1] = equation.coefficient_X_down[1] * temp[0] + equation.coefficient_X_down[0];
            if ((coefficient[1]) && (temp[0] > limitation[0]) && (temp[0] < limitation[1]) && (temp[1] > limitation[2]) && (temp[1] < limitation[3]))
            {
                equation.root_coord[3][0] = temp[0];
                equation.root_coord[3][1] = temp[1];
            }
            else
            {
                return 0;
            }
        }
    }
    return 1;
}