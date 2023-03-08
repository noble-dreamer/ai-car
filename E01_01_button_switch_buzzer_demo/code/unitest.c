#include "unitest.h"
//-----------------------------------------------
// 作用对各个部位的算法做单元测试包括test_imgprocess,motor,imu,location等
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