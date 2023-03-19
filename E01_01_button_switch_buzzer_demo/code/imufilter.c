#include "imufilter.h"

#define M_PI       3.14159f

static float I_ex, I_ey, I_ez; //误差积分

static float param_Kp = 0.17;   // 加速度计的收敛速率比例增益 
static float param_Ki = 0.004;   //陀螺仪收敛速率的积分增益 0.004

quater_param_t Q_info = {1, 0, 0,0}; //全局四元数

imu_param_t imu_data;
gyro_param_t gyroffset;

extern euler_param_t eulerAngle; //欧拉角

#define filter_count 10000
/*
小数字时其精确度和标准库中的sqrt()相差不多，
但是，特别是对于较大的数字时,用sqrt()
O(log n) 与 o(n)得到的时平方根你倒数,需要再*x
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

void gyroOffset_init(void) /////////陀螺仪零飘初始化
{
    gyroffset.Xdata = 0;
    gyroffset.Ydata = 0;
    gyroffset.Zdata = 0;
    /*均值滤出陀螺仪得到的xyz*/
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

//转换陀螺仪的值
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


    //陀螺仪角度转弧度
    imu_data.gyro_x = ((float) imu963ra_gyro_x - gyroffset.Xdata) * M_PI / 180 / 14.3f;
    imu_data.gyro_y = ((float) imu963ra_gyro_y - gyroffset.Ydata) * M_PI / 180 / 14.3f;
    imu_data.gyro_z = ((float) imu963ra_gyro_z - gyroffset.Zdata) * M_PI / 180 / 14.3f;
}

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx,float my,float mz)
{
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;    //当前的机体坐标系上的重力单位向量
    float ex, ey, ez;    //四元数计算值与加速度计测量值的误差
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
    //对加速度数据进行归一化 得到单位加速度
    float norm = fast_sqrt_InvSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //norm = fast_sqrt_InvSqrt(mx*mx + my*my + mz*mz);   
    //mx = mx * norm;  
    //my = my * norm;  
    //mz = mz * norm;  
    //根据当前四元数的姿态值来估算出各重力分量。用于和加速计实际测量出来的各重力分量进行对比，从而实现对四轴姿态的修正

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //vz = (q0*q0-0.5f+q3 * q3) * 2;
//叉积来计算估算的重力和实际测量的重力这两个重力向量之间的误差。
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    //用叉乘误差来做PI修正陀螺零偏，
    //通过调节 param_Kp，param_Ki 两个参数，
    //可以控制加速度计修正陀螺仪积分姿态的速度。
    I_ex += halfT * ex;   // integral error scaled by Ki
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;
    //四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
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
@breif 这是加上了地磁计的四元数算法，上面为未加上地磁计的算法
        float norm;               
        float hx, hy, hz, bx, bz;
        float vx, vy, vz, wx, wy, wz; 
        float ex, ey, ez;  

        // 定义一些辅助变量用于转换矩阵
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
         
        // 归一化加速度计和地磁计的度数 
        norm = sqrt(ax*ax + ay*ay + az*az);   
        ax = ax / norm;  
        ay = ay / norm;  
        az = az / norm;  
        norm = sqrt(mx*mx + my*my + mz*mz);   
        mx = mx / norm;  
        my = my / norm;  
        mz = mz / norm;  
         
        //将b系中的地磁计分量[mx,my,mz]转换到n系,得到[hx,hy,hz]  
        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);  
        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);  
        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);        

        //得到n系中的地磁向量的真实值[bx,bz,by],其中by=0   
        bx = sqrt((hx*hx) + (hy*hy));  
        bz = hz;     

        //n系中的地磁向量[bx，by,bz]转换到b系中，得到[wx,wy,wz]
        wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);  
        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);  
        wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);                        

        //n系中重力加速度[0,0,1]转换到b系中得到三个分量[vx,vy,vz]        
        vx = 2*(q1q3 - q0q2);  
        vy = 2*(q0q1 + q2q3);  
        vz = q0q0 - q1q1 - q2q2 + q3q3;    
         
        //计算[wx,wy,wz] X [mx,my,mz],[ax,at,az] X [vx,vy,vz]，得到两个误差后求和
        ex = (ay*vz - az*vy) + (my*wz - mz*wy);  
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);  
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);  
         
        //PI控制器中的积分部分
        I_ex = I_ex + ex*param_Ki* halfT;  
        I_ey = I_ey + ey*param_Ki* halfT;  
        I_ez = I_ez + ez*param_Ki* halfT;  
        
        //误差经过PI控制器后输出,然后补偿到角速度的三个分量，Kp、Ki是需要调节的参数
        gx = gx + param_Kp*ex + I_ex;  
        gy = gy + param_Kp*ey + I_ey;  
        gz = gz + param_Kp*ez + I_ez;               
        
        //一阶龙格库塔法更新四元数  
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;  
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;  
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;  
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;    
         
        // 归一化四元数
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

    IMU_getValues();//可以在中断中获得
    IMU_AHRSupdate(imu_data.gyro_x,imu_data.gyro_y,imu_data.gyro_z,imu_data.acc_x,imu_data.acc_y,imu_data.acc_z,imu_data.mag_x,imu_data.mag_y,imu_data.mag_z);
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
//    //四元数计算欧拉角
//    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI; // pitch
//    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI; // roll
//    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI; // yaw
///*   姿态限制*/
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
    //四元数计算欧拉角以pai为表示
    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) ; // pitch
    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1); // roll
    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1); // yaw
/*   姿态限制*/

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