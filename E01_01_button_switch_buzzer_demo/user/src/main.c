/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ����ֲ�ÿչ���

StateFlags state_flags = {0};
euler_param_t eulerAngle;
Coord coord = {0};
Stance stance = {0};
uint8_t target_lable[2][12] = {{0}, {0}};//���ڱ�ʾ���

CarBodySize car_body_size = {0};
LocalImageSize local_image_size = {0};

Motor_info left_front = {0};
Motor_info right_front = {0};
Motor_info left_back = {0};
Motor_info right_back = {0};

uint16_t city_result[object_num_limit] = {0}; //����Ҳ�Ǵ��ÿ��ͼƬ����������
float target_coord[2][object_num_limit] = {{0}, {0}}; //�������꣬��¼ʵ�ʾ���
/*���ڴ��ȫ�ֵ��ƶ�,���ȥ��,�´�ȥ��*/
int globalTour[STATIC_ARRAY_SIZE][2];

int8_t current_obj_index = -1;  //����location����ʾ��ǰ����
uint8_t object_num = 0;         //����location,��ʾ����
float compensate_radian = 0;    //����location
uint8_t A4_select = 0;
uint8_t Carry = 0;

/*
    �����ٶȵĻ�������
*/

float straight_speed_x = 0;
float straight_speed_y = 0;
float target_straight_speed_x = 0;
float target_straight_speed_y = 0;

float rotation_speed = 0;
float target_rotation_speed = 0;
float carry_speed = 0;
float mProport = 0;

int8_t rotation_direction = 0;

float deviation_radian, pan_direction = 5000, scale_factor = 0;
float test_car_x = 0, test_car_y = 0;
float growthRateMultiplier = -1; //������������
int8_t counterclockwiseDetection = 0;

/*
    ͼ��任�Ļ�����
*/
//[��������][˫������][����]
#if MT9V03X_H > MT9V03X_W
uint16_t Egde[4][2][MT9V03X_H / 2];
#else
/*
//���������������,[��������][��������][���а볤]
  ��ԴΪimage_one_dimension[]��index
*/
uint16_t Egde[4][2][MT9V03X_W / 2];
#endif

uint16_t A4_center = Center;
uint16_t demarcate_center;
uint16_t edge_size_limit;
// uint16 image_one_dimension[MT9V03X_H * MT9V03X_W];
// uint16 (*image_two_dimension)[MT9V03X_W] = (uint16 (*)[MT9V03X_W]) image_one_dimension;�˷������Խ���ά�����һά������ϵ����
uint8_t image_one_dimension[MT9V03X_IMAGE_SIZE] = {0};
BinaryThreshold binary_threshold = {0};
Equation equation = {0};







/*
����������л�����,����location,motor��
*/
extern struct motorcontrol MotorControl;
extern struct motorcontrol *thisMotorControl;
extern struct location car_location;
extern struct location *thiscar_location;

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ����ɾ��
    debug_init();                   // ���Զ˿ڳ�ʼ��

    // �˴���д�û����� ���������ʼ�������
	test_encoder_get();
    //test_imgprocess();
    // �˴���д�û����� ���������ʼ�������
    //while(1)
    //{
    //    // �˴���д��Ҫѭ��ִ�еĴ���
    //    
    //    // �˴���д��Ҫѭ��ִ�еĴ���
    //}
	return 0;
}




