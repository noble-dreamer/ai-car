#include "unitest.h"
extern struct motorcontrol MotorControl;
extern struct motorcontrol *thisMotorControl;

extern struct location car_location;
extern struct location *thiscar_location;
//-----------------------------------------------
// ���öԸ�����λ���㷨����Ԫ���԰���test_imgprocess,motor,imu,location��
//----------------------------------------------
//-----------------------------------------------
// test_imgprocess
// otsu              pass
// sobel             pass
// GaussianBlur      pass
// 
//
//----------------------------------------------
void test_imgprocess()
{
    ips114_init();
    ips114_show_string(0, 0, "mt9v03x init.");
    while (1)
    {
        if (mt9v03x_init())
        {          
            ips114_show_string(0, 16, "mt9v03x reinit.");
        }
        else
            break;
        system_delay_ms(500);   
    }
    ips114_show_string(0, 16, "init success.");
    interrupt_global_enable(0);
    while (1)
    {
        if (mt9v03x_finish_flag)
        {
        float a_array[MT9V03X_H * MT9V03X_W];
        uint8 threshold = 0;
//        gaussianBlur(mt9v03x_image,MT9V03X_H,MT9V03X_W,KERNEL_SIZE,Sigma);
//        threshold = otsuThreshold(mt9v03x_image,MT9V03X_H,MT9V03X_W);
//        otsuThreshold(mt9v03x_image,MT9V03X_H,MT9V03X_W); 
//        for (int i = 0; i < MT9V03X_H; i++)
//        {
//            for (int j = 0; i < MT9V03X_W; i++)
//            {
//                printf("%d\n",mt9v03x_image[i][j]);
//            }
//            
//        }      
//        ips114_displayimage03x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//        ips114_show_gray_image(0,0,(const uint8 *)mt9v03x_image,MT9V03X_W,MT9V03X_H,MT9V03X_W,MT9V03X_H, threshold);
//        fastGaussianBlur(mt9v03x_image,MT9V03X_H,MT9V03X_W);
//        sobelEdgeDetection(mt9v03x_image,a_array,MT9V03X_W,MT9V03X_H);
        cannyEdgeDetection(mt9v03x_image,MT9V03X_W,MT9V03X_H,Sigma);
        ips114_show_gray_image(0,0,(const uint8 *)mt9v03x_image,MT9V03X_W,MT9V03X_H,MT9V03X_W,MT9V03X_H, 0);
        mt9v03x_finish_flag = 0;
        }
        
    }
    
}
//���Ϊ��,�ɲ��Կ�֪1Ϊ��ǰת
void test_motor_stright()
{
    ips114_init();
    ips114_show_string(0, 0, "start.");
    motorcontrol_Init(thisMotorControl);
    motorInit();
    ips114_show_string(0, 20, "Init.");
    thisMotorControl->motorcontrol_Constructor(thisMotorControl);
    thisMotorControl->signalMotorControl(thisMotorControl,Motor_left_front,1,3000); //���
    //thisMotorControl->signalMotorControl(thisMotorControl,Motor_left_back,1,3000);  //��ǰ
    thisMotorControl->signalMotorControl(thisMotorControl,Motor_right_front,1,3000);//�Һ�
    thisMotorControl->signalMotorControl(thisMotorControl,Motor_right_back,1,3000); //��ǰ
    ips114_show_string(0, 40, "OK.");
    
}

void test_encoder_get()
{
    ips114_init();
    ips114_show_string(0, 0, "start.");
    motorcontrol_Init(thisMotorControl);
    encoderInit();
    thisMotorControl->motorcontrol_Constructor(thisMotorControl);
    pit_ms_init(PIT_CH0, delta_T*1000);
    ips114_show_string(0, 20, "Init.");
    while(1)
    {
        printf("ENCODER_LEFT_FRONT counter \t%d .\r\n", thisMotorControl->left_front->get_encode_data);                 // ���������������Ϣ
        printf("ENCODER_right_front counter \t%d .\r\n", thisMotorControl->right_front->get_encode_data);                 // ���������������Ϣ  
        printf("ENCODER_left_back counter \t%d .\r\n", thisMotorControl->left_back->get_encode_data);                 // ���������������Ϣ
        printf("ENCODER_right_back counter \t%d .\r\n", thisMotorControl->right_back->get_encode_data);                 // ���������������Ϣ  
        system_delay_ms(delta_T*1000);
        ips114_show_string(0, 40, "OK.");
    }

}
void test_imu_filter()
{
    ips114_init();
    ips114_show_string(0, 0, "start.");
    motorcontrol_Init(thisMotorControl);
    location_Init(thiscar_location);
    motorInit();
    encoderInit();
    thisMotorControl->motorcontrol_Constructor(thisMotorControl);
    thiscar_location->location_Constructor(thiscar_location);
    while(1)
    {
        if(imu963ra_init())
        {
            ips114_show_string(0, 20, "IMU963RA init error.");
            //printf("\r\nIMU963RA init error.");                                 // IMU963RA ��ʼ��ʧ��
        }
        else
        {
            break;
        }
        //gpio_toggle_level(LED1);                                                // ��ת LED ���������ƽ ���� LED ���� ��ʼ����������ƻ����ĺ���
    }
    gyroOffset_init();
    ips114_show_string(0, 20, "Init.");

    while (1)
    {
        IMU_getEulerianAngles(); //�ɴ˿��Ի��IMU��ֵ
        thiscar_location->radiansGet(thiscar_location);
        ips114_show_string(0, 40, "imu_get");
        printf("%4f",thiscar_location->eulerAngle->yaw);
        system_delay_ms(delta_T*1000);
    }
    
    
}
void test_motor_turnround()
{
    //��ʼ������
    ips114_init();
    ips114_show_string(0, 0, "start.");
    motorcontrol_Init(thisMotorControl);
    location_Init(thiscar_location);
    motorInit();
    encoderInit();
    thisMotorControl->motorcontrol_Constructor(thisMotorControl);
    thiscar_location->location_Constructor(thiscar_location);
    while(1)
    {
        if(imu963ra_init())
        {
            ips114_show_string(0, 20, "IMU963RA init error.");
            //printf("\r\nIMU963RA init error.");                                 // IMU963RA ��ʼ��ʧ��
        }
        else
        {
            break;
        }
        //gpio_toggle_level(LED1);                                                // ��ת LED ���������ƽ ���� LED ���� ��ʼ����������ƻ����ĺ���
    }
    gyroOffset_init();
    ips114_show_string(0, 20, "Init.");
    pit_ms_init(PIT_CH0, delta_T*1000);
    while (1)
    {
        
    }
    

    
}