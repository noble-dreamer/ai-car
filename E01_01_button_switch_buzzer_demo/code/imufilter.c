#include "imufilter.h"

#define M_PI       3.14159f

static float I_ex, I_ey, I_ez; //������

static float param_Kp = 0.17;   // ���ٶȼƵ��������ʱ������� 
static float param_Ki = 0.004;   //�������������ʵĻ������� 0.004

quater_param_t Q_info = {1, 0, 0,0}; //ȫ����Ԫ��

imu_param_t imu_data;
gyro_param_t gyroffset;

extern euler_param_t eulerAngle; //ŷ����

#define filter_count 10000
/*
С����ʱ�侫ȷ�Ⱥͱ�׼���е�sqrt()���࣬
���ǣ��ر��Ƕ��ڽϴ������ʱ,��sqrt()
O(log n) �� o(n)�õ���ʱƽ�����㵹��,��Ҫ��*x
*/
float fast_sqrt_InvSqrt (float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float fast_sqrt (float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return x*y;
}

void gyroOffset_init(void) /////////��������Ʈ��ʼ��
{
    gyroffset.Xdata = 0;
    gyroffset.Ydata = 0;
    gyroffset.Zdata = 0;
    /*��ֵ�˳������ǵõ���xyz*/
    for (uint16_t i = 0; i < filter_count; ++i)
    {
        /* code */
    imu963ra_get_acc ();
    imu963ra_get_gyro ();
    imu963ra_get_mag ();
    gyroffset.Xdata += imu963ra_gyro_x;
    gyroffset.Ydata += imu963ra_gyro_y;
    gyroffset.Zdata += imu963ra_gyro_z;
    //system_delay_ms(10);
    }
    gyroffset.Xdata  /= filter_count;
    gyroffset.Ydata  /= filter_count;
    gyroffset.Zdata  /= filter_count;

    if(fabs(gyroffset.Zdata) < abs(imu963ra_gyro_z))
    {
        state_flags.imufilterdriftinit= 1;
    }
    else
    {
        state_flags.imufilterdriftinit= 0;
    }

}

#define alpha           0.3f

//ת�������ǵ�ֵ
void IMU_getValues()
{
    //imu_data.acc_x =  imu963ra_acc_transition(imu963ra_acc_x);
    //imu_data.acc_y =  imu963ra_acc_transition(imu963ra_acc_y);
    //imu_data.acc_z =  imu963ra_acc_transition(imu963ra_acc_z);
    //imu_data.gyro_x =  imu963ra_gyro_transition(imu963ra_gyro_x);
    //imu_data.gyro_y =  imu963ra_gyro_transition(imu963ra_gyro_y);
    //imu_data.gyro_z =  imu963ra_gyro_transition(imu963ra_gyro_z);
    imu_data.mag_x =  imu963ra_mag_transition(imu963ra_mag_x);
    imu_data.mag_y =  imu963ra_mag_transition(imu963ra_mag_y);
    imu_data.mag_z =  imu963ra_mag_transition(imu963ra_mag_z);
    
    imu_data.acc_x = (((float) imu963ra_acc_x) * alpha) / 4098 + imu_data.acc_x * (1 - alpha);
    imu_data.acc_y = (((float) imu963ra_acc_y) * alpha) / 4098 + imu_data.acc_y * (1 - alpha);
    imu_data.acc_z = (((float) imu963ra_acc_z) * alpha) / 4098 + imu_data.acc_z * (1 - alpha);


    //�����ǽǶ�ת����
    imu_data.gyro_x = ((float) imu963ra_gyro_x - gyroffset.Xdata) * M_PI / 180 / 14.3f;
    imu_data.gyro_y = ((float) imu963ra_gyro_y - gyroffset.Ydata) * M_PI / 180 / 14.3f;
    imu_data.gyro_z = ((float) imu963ra_gyro_z - gyroffset.Zdata) * M_PI / 180 / 14.3f;
}

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx,float my,float mz)
{
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;    //��ǰ�Ļ�������ϵ�ϵ�������λ����
    float ex, ey, ez;    //��Ԫ������ֵ����ٶȼƲ���ֵ�����
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    //�Լ��ٶ����ݽ��й�һ�� �õ���λ���ٶ�
    float norm = fast_sqrt_InvSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //norm = fast_sqrt_InvSqrt(mx*mx + my*my + mz*mz);   
    //mx = mx * norm;  
    //my = my * norm;  
    //mz = mz * norm;  
    //���ݵ�ǰ��Ԫ������ֵ̬����������������������ںͼ��ټ�ʵ�ʲ��������ĸ������������жԱȣ��Ӷ�ʵ�ֶ�������̬������

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //vz = (q0*q0-0.5f+q3 * q3) * 2;
//�������������������ʵ�ʲ�����������������������֮�����
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    //�ò���������PI����������ƫ��
    //ͨ������ param_Kp��param_Ki ����������
    //���Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶȡ�
    I_ex += halfT * ex;   // integral error scaled by Ki
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;
    //��Ԫ��΢�ַ��̣�����halfTΪ�������ڵ�1/2��gx gy gzΪ�����ǽ��ٶȣ����¶�����֪��������ʹ����һ��������������Ԫ��΢�ַ���
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    // normalise quaternion
    norm = fast_sqrt_InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
/*
@breif ���Ǽ����˵شżƵ���Ԫ���㷨������Ϊδ���ϵشżƵ��㷨
        float norm;               
        float hx, hy, hz, bx, bz;
        float vx, vy, vz, wx, wy, wz; 
        float ex, ey, ez;  

        // ����һЩ������������ת������
        float q0q0 = q0*q0;  
        float q0q1 = q0*q1;  
        float q0q2 = q0*q2;  
        float q0q3 = q0*q3;  
        float q1q1 = q1*q1;  
        float q1q2 = q1*q2;  
        float q1q3 = q1*q3;  
        float q2q2 = q2*q2;  
        float q2q3 = q2*q3;  
        float q3q3 = q3*q3;  
         
        // ��һ�����ٶȼƺ͵شżƵĶ��� 
        norm = sqrt(ax*ax + ay*ay + az*az);   
        ax = ax / norm;  
        ay = ay / norm;  
        az = az / norm;  
        norm = sqrt(mx*mx + my*my + mz*mz);   
        mx = mx / norm;  
        my = my / norm;  
        mz = mz / norm;  
         
        //��bϵ�еĵشżƷ���[mx,my,mz]ת����nϵ,�õ�[hx,hy,hz]  
        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);  
        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);  
        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);        

        //�õ�nϵ�еĵش���������ʵֵ[bx,bz,by],����by=0   
        bx = sqrt((hx*hx) + (hy*hy));  
        bz = hz;     

        //nϵ�еĵش�����[bx��by,bz]ת����bϵ�У��õ�[wx,wy,wz]
        wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);  
        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);  
        wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);                        

        //nϵ���������ٶ�[0,0,1]ת����bϵ�еõ���������[vx,vy,vz]        
        vx = 2*(q1q3 - q0q2);  
        vy = 2*(q0q1 + q2q3);  
        vz = q0q0 - q1q1 - q2q2 + q3q3;    
         
        //����[wx,wy,wz] X [mx,my,mz],[ax,at,az] X [vx,vy,vz]���õ������������
        ex = (ay*vz - az*vy) + (my*wz - mz*wy);  
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);  
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);  
         
        //PI�������еĻ��ֲ���
        I_ex = I_ex + ex*param_Ki* halfT;  
        I_ey = I_ey + ey*param_Ki* halfT;  
        I_ez = I_ez + ez*param_Ki* halfT;  
        
        //����PI�����������,Ȼ�󲹳������ٶȵ�����������Kp��Ki����Ҫ���ڵĲ���
        gx = gx + param_Kp*ex + I_ex;  
        gy = gy + param_Kp*ey + I_ey;  
        gz = gz + param_Kp*ez + I_ez;               
        
        //һ�����������������Ԫ��  
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;  
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;  
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;  
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;    
         
        // ��һ����Ԫ��
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);  
        q0 = q0 / norm;  
        q1 = q1 / norm;  
        q2 = q2 / norm;  
        q3 = q3 / norm;  

*/
}


void IMU_getEulerianAngles(void)
{
//    while (!state_flags.imufilterdriftinit)
//    {
//        /* code */
//        gyroOffset_init();
//    }
    
    imu963ra_get_acc ();
    imu963ra_get_gyro ();
    imu963ra_get_mag ();

    IMU_getValues();//�������ж��л��
    IMU_AHRSupdate(imu_data.gyro_x,imu_data.gyro_y,imu_data.gyro_z,imu_data.acc_x,imu_data.acc_y,imu_data.acc_z,imu_data.mag_x,imu_data.mag_y,imu_data.mag_z);
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
//    //��Ԫ������ŷ����
//    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI; // pitch
//    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI; // roll
//    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI; // yaw
///*   ��̬����*/
//    if (eulerAngle.roll > 90 || eulerAngle.roll < -90) {
//        if (eulerAngle.pitch > 0) {
//            eulerAngle.pitch = 180 - eulerAngle.pitch;
//        }
//        if (eulerAngle.pitch < 0) {
//            eulerAngle.pitch = -(180 + eulerAngle.pitch);
//        }
//    }
//
//    if (eulerAngle.yaw > 360) {
//        eulerAngle.yaw -= 360;
//    } else if (eulerAngle.yaw < 0) {
//        eulerAngle.yaw += 360;
//    }
    //��Ԫ������ŷ������paiΪ��ʾ
    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) ; // pitch
    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1); // roll
    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1); // yaw
/*   ��̬����*/

    if (eulerAngle.roll > M_PI/2 || eulerAngle.roll < -M_PI/2) {
        if (eulerAngle.pitch > 0) {
            eulerAngle.pitch = M_PI - eulerAngle.pitch;
        }
        if (eulerAngle.pitch < 0) {
            eulerAngle.pitch = -(M_PI + eulerAngle.pitch);
        }
    }

    if (eulerAngle.yaw -M_PI >= FLT_MIN_SELF ) {
        eulerAngle.yaw -= 2*M_PI;
    } else if (eulerAngle.yaw +M_PI <= - FLT_MIN_SELF) {
        eulerAngle.yaw += 2*M_PI;
    }


}