#include "img_process.h"

extern float target_coord[2][object_num_limit];
//[上下左右][双向搜索][索引]
#if MT9V03X_H > MT9V03X_W
extern uint16_t Egde[4][2][MT9V03X_H / 2];
#else
extern uint16_t Egde[4][2][MT9V03X_W / 2];
#endif
extern float target_coord[2][object_num_limit];
extern int globalTour[STATIC_ARRAY_SIZE][2];
extern uint16_t A4_center;
extern uint16_t demarcate_center;
extern uint16_t edge_size_limit;
extern uint8_t image_one_dimension[MT9V03X_IMAGE_SIZE];
extern BinaryThreshold binary_threshold;
extern Equation equation;

//vuint8 mt9v03x_finish_flag = 0;                                                  // 一场图像采集完成标志位
//// 图像缓冲区  如果用户需要访问图像数据 最好通过tempImage来访问数据，最好不要直接访问缓冲区
//AT_DTCM_SECTION_ALIGN(uint8 mt9v03x_image1[MT9V03X_H][MT9V03X_W], 64);
//AT_DTCM_SECTION_ALIGN(uint8 mt9v03x_image2[MT9V03X_H][MT9V03X_W], 64);
////
//// 用户访问图像数据直接访问这个指针变量就可以
//// 访问方式非常简单，可以直接使用下标的方式访问
//// 例如访问第10行 50列的点，tempImage[10][50]就可以了
//uint8 (*mt9v03x_image)[MT9V03X_W];
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     高斯模糊
// 参数说明     (*image)[MT9V03X_W]  总钻风采集到的图像
// 参数说明     height          图像实际高度
// 参数说明     width           图像实际宽度
// 参数说明     size            卷积核的大小
// 参数说明     sigma           卷积核的参数,在3时为0.8
// 返回参数     void
// 使用示例     gaussianBlur((const uint8 *)mt9v03x_image,MT9V03X_H,MT9V03X_W,KERNEL_SIZE,Sigma)
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------

AT_ITCM_SECTION_INIT(void gaussianBlur(uint8 (*image)[MT9V03X_W], uint8 height, uint8 width, int size, float sigma))
{
    int i, j, m, n;
    float sum;
    float *kernel = (float*)malloc(sizeof(float) * size * size);
    uint8 *padImage = (uint8*)malloc(sizeof(uint8) * (height + size - 1) * (width + size - 1));
    uint8 (*blurred)[MT9V03X_W] = (uint8 (*)[MT9V03X_W])malloc(sizeof(uint8[MT9V03X_W]) * height);

    // Initialize pad image with 0
    memset(padImage, 0, sizeof(uint8) * (height + size - 1) * (width + size - 1));

    // Generate Gaussian kernel
    generateGaussianKernel(kernel, size, sigma);

    // Pad original image with 0
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            padImage[(i + size / 2) * (width + size - 1) + j + size / 2] = image[i][j];
        }
    }

    // Copy the pixels from the first row of the original image to the corresponding row in the padded image
//    for (j = 0; j < size / 2; j++)
//    {
//        for (i = size / 2; i < height + size / 2; i++)
//        {
//            padImage[i * (width + size - 1) + j] = image[i - size / 2][0];
//            padImage[i * (width + size - 1) + width + size / 2 + j] = image[i - size / 2][width - 1];
//        }
//    }
//
//    // Copy the pixels from the first and last columns of the original image to the corresponding columns in the padded image
//    for (i = 0; i < size / 2; i++)
//    {
//        for (j = 0; j < width + size - 1; j++)
//        {
//            padImage[i * (width + size - 1) + j] = padImage[size / 2 * (width + size - 1) + j];
//            padImage[(height + size / 2 + i) * (width + size - 1) + j] = padImage[(height + size / 2 - 1) * (width + size - 1) + j];
//        }
//    }

    // Apply Gaussian Blur
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            sum = 0.0;
            for (m = 0; m < size; m++)
            {
                for (n = 0; n < size; n++)
                {
                    sum += padImage[(i + m) * (width + size - 1) + j + n] * kernel[m * size + n];
                }
            }
            blurred[i][j] = (uint8)round(sum);
        }
    }

    // Free memory
    free(padImage);
    free(kernel);

    // Copy result back to original image
    for (i = 0; i < height; i++)
    {
    for (j = 0; j < width; j++)
        {
            image[i][j] = blurred[i][j];
        }
    }

    free(blurred);
}

//--------------------------------------------------------------------------------
// 生成高斯核的函数(内部调用)
//float GaussianKernel[3][3] = {
//    {1.0 / 16, 1.0 / 8, 1.0 / 16},
//    {1.0 / 8, 1.0 / 4, 1.0 / 8},
//    {1.0 / 16, 1.0 / 8, 1.0 / 16}
//};
//事实上高斯核不用内部生成,这回占用大量时间,选择时间存储即可
//signed char gx[GX_KERNEL_SIZE][GX_KERNEL_SIZE] = {
//    { -1, 0, 1 },
//    { -2, 0, 2 },
//    { -1, 0, 1 }
//}; sobel核的生成
//
//signed char gy[GY_KERNEL_SIZE][GY_KERNEL_SIZE] = {
//    { -1, -2, -1 },
//    { 0, 0, 0 },
//    { 1, 2, 1 }
//};
AT_ITCM_SECTION_INIT(void generateGaussianKernel(float *kernel, int size, float sigma))
{
    int i, j;
    float sum = 0.0;
    float d = 2 * sigma * sigma;
    int center = size / 2;

    for (i = 0; i < size; i++)
    {
        for (j = 0; j < size; j++)
        {
            kernel[i * size + j] = exp(-((i - center) * (i - center) + (j - center) * (j - center)) / d) / (M_PI * d);
            sum += kernel[i * size + j];
        }
    }

    // Normalize the kernel
    for (i = 0; i < size * size; i++)
    {
        kernel[i] /= sum;
    }
}
/**-----------------------------------------------------------------------------
 * @description: 快速高斯模糊,不用padding和生成核的方式
 * @param {*}
 * @return {*}
 */
//定义高斯模糊核
float GaussianKernel[3][3] = {
    {1.0 / 16, 1.0 / 8, 1.0 / 16},
    {1.0 / 8, 1.0 / 4, 1.0 / 8},
    {1.0 / 16, 1.0 / 8, 1.0 / 16}
};

//高斯模糊
AT_ITCM_SECTION_INIT(void fastGaussianBlur(uint8 (*image)[MT9V03X_W], uint8 height, uint8 width)) 
{
    int i, j, k, l;
    float sum;
    uint8 tempImage[height][width];
    memcpy(tempImage, image, sizeof tempImage);
    
    for (i = 1; i < height - 1; i++) {
        for (j = 1; j < width - 1; j++) {
            sum = 0.0;
            for (k = -1; k <= 1; k++) {
                for (l = -1; l <= 1; l++) {
                    sum += GaussianKernel[k + 1][l + 1] * tempImage[i + k][j + l];
                }
            }
            image[i][j] = (uint8)sum;
        }
    }
}



/**-----------------------------------------------------------------------------
 * @description: 大津法阈值（可考虑处理区域）
 * @param {*}
 * @return {*}
 */
/*该代码实现了大津法二值化*/
AT_ITCM_SECTION_INIT(uint8 otsuThreshold(uint8 (*image)[MT9V03X_W], uint8 height, uint8 width))
{
    int i, j;
    int histogram[MAX_GRAY_LEVEL] = { 0 };
    int total = height * width;

    // Calculate the histogram
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            histogram[image[i][j]]++;
        }
    }

    // Threshold calculation
    int sum = 0;
    for (i = 0; i < MAX_GRAY_LEVEL; i++)
    {
        sum += i * histogram[i];
    }

    int sumB = 0;
    int wB = 0;
    int wF = 0;
    float maxVariance = 0.0;
    uint8 threshold = 0;
    /*you can use i = i + n instead of i++ */
    for (i = 0; i < MAX_GRAY_LEVEL; i +=2 )
    {
        wB += histogram[i];
        if (wB == 0)
            continue;

        wF = total - wB;
        if (wF == 0)
            break;

        sumB += i * histogram[i];
        float mB = (float)sumB / wB;
        float mF = (float)(sum - sumB) / wF;
        float variance = (float)wB * wF * (mB - mF) * (mB - mF);

        if (variance > maxVariance)
        {
            maxVariance = variance;
            threshold = i;
        }
    }
    return threshold;
//    // Apply threshold
//    for (i = 0; i < height; i++)
//    {
//        for (j = 0; j < width; j++)
//        {
//            if (image[i][j] > threshold)
//            {
//                image[i][j] = 255;
//            }
//            else
//            {
//                image[i][j] = 0;
//            }
////            printf("%d",image[i][j]);
//        }
}


AT_ITCM_SECTION_INIT(void sobelEdgeDetection(uint8 (*image)[MT9V03X_W], float *a_array,uint8 width, uint8 height))
{
    uint16 i, j;
    int16 gx, gy;
    uint16_t gradient;
    uint8 tempImage[height][width];
    memcpy(tempImage, image, sizeof tempImage);
    for (i = 0; i < height ; i++) 
    {
        for (j = 0; j < width ; j++) 
        {
            if (i < 2 || j < 2 || i > height - 3 || j > width - 3)
			{
				//padding，貌似是有一点问题的，r_array，a_array需要多一个padding位，而不是把边缘直接变成0
				//不过这样子的化倒是可以少一个ON^2的运算
				image[i][j] = 0;
				a_array[i * width + j] = 0;
			}
            else
            {
                gx = (tempImage[i][j + 2] - tempImage[i][j]) +
						2 * (tempImage[i + 1][j + 2] - tempImage[i + 1][j]) +
						(tempImage[i + 2][j + 2] - tempImage[i + 2][j]);
                gy = (tempImage[i + 2][j] - tempImage[i][j]) +
						2 * (tempImage[i + 2][j + 1] - tempImage[i][j + 1]) +
						(tempImage[i + 2][j + 2] - tempImage[i][j + 2]);
                gradient =(uint16_t)fast_sqrt(((float)gx * (float)gx + (float)gy * (float)gy));
                gradient = gradient > 255 ? 255 : gradient;
                image[i][j] = gradient;
                a_array[i * width + j] = (float)gy / (float)gx; // tan角,用于canny算法
            }
        }
    }
}

AT_ITCM_SECTION_INIT(void nonMaximalSuppression(uint8 (*image)[MT9V03X_W], float *a_array,uint8 width, uint8 height))
{
    int i, j;
    uint8 temp[height][width];
    memcpy(temp, image, sizeof temp);
    for (i = 0; i < height ; i++) 
    {
        for (j = 0; j < width ; j++) 
        {
//            int gradient = a_array[i* width + j];
//            int direction = 0;
//            if (gradient != 0) 
//            {
////                int gx = image[i][j + 1] - image[i][j - 1];
////                int gy = image[i + 1][j] - image[i - 1][j];
//                direction = atan(gradient) * 180 / M_PI;
//                direction = (direction + 180) / 45;
//            }
//            switch (direction) 
//            {
//                case 0:
//                    if (gradient <= image[i][j - 1] || gradient <= image[i][j + 1])
//                        gradient = 0;
//                    break;
//                case 1:
//                    if (gradient <= image[i - 1][j + 1] || gradient <= image[i + 1][j - 1])
//                        gradient = 0;
//                    break;
//                case 2:
//                    if (gradient <= image[i - 1][j] || gradient <= image[i + 1][j])
//                        gradient = 0;
//                    break;
//                case 3:
//                    if (gradient <= image[i - 1][j - 1] || gradient <= image[i + 1][j + 1])
//                        gradient = 0;
//                    break;
//                default:
//                    break;
//            }
//            image[i][j] = gradient;
			if (i == 0 || j == 0 || i == height - 1 || j == width - 1)
			{
				temp[i][j] = 0;
			}
            //  a里面放的已经时tan角
            else if ((a_array[i * width + j] >= -0.414214) && (a_array[i * width + j] <= 0.414214))
			{
				/*
				angle(x,y)=0

				如果在edge(x,y)、edge(x?1,y)、edge(x+1,y)中edge(x,y)不是最大的，那么edge(x,y)=0；

				angle(x,y)=45

				如果在edge(x,y)、edge(x?1,y)、edge(x+1,y)中edge(x,y)不是最大的，那么edge(x,y)=0；

				angle(x,y)=90

				如果在edge(x,y)、edge(x?1,y)、edge(x+1,y)中edge(x,y)不是最大的，那么edge(x,y)=0；

				angle(x,y)=135

				如果在edge(x,y)、edge(x?1,y)、edge(x+1,y)中edge(x,y)不是最大的，那么edge(x,y)=0；
				
				*/
				if (temp[i][j] < temp[i][j - 1] || temp[i][j] < temp[i][j + 1])
				{
					temp[i][j] = 0;
				}
			}
			else if ((a_array[i * width + j] < 2.414214) && (a_array[i * width + j] > 0.414214))
			{
				if (temp[i][j] < temp[i + 1][j - 1] || temp[i][j] < temp[i - 1][j + 1])
				{
					temp[i][j] = 0;
				}
			}
			else if ((a_array[i * width + j] >= 2.414214) || (a_array[i * width + j] <= -2.414214))
			{
				if (temp[i][j] < temp[i - 1][j] || temp[i][j] < temp[i + 1][j])
				{
					temp[i][j] = 0;
				}
			}
			else if ((a_array[i * width + j] < -0.414214) && (a_array[i * width + j] > -2.414214))
			{
				if (temp[i][j] < temp[i - 1][j - 1] || temp[i][j] < temp[i + 1][j + 1])
				{
					temp[i][j] = 0;
				}
			}
        }
    }
    doubleThresholding(image,temp, width, height, binary_threshold.L_canny_th, binary_threshold.H_canny_th);
}

AT_ITCM_SECTION_INIT(void doubleThresholding(uint8 (*image)[MT9V03X_W],uint8 (*temp)[MT9V03X_W], uint8 width, uint8 height, int lowThreshold, int highThreshold))
{
    int i, j;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            int gradient = temp[i][j];
            if (gradient >= highThreshold) 
            {
                image[i][j] = 255;
            } 
            else //if (gradient < lowThreshold) 
            {
                image[i][j] = 0;
            } 
            //else 
            //{
            //    image[i][j] = 128;
            //}
        }
    }
}

AT_ITCM_SECTION_INIT(void cannyEdgeDetection(uint8 (*image)[MT9V03X_W], uint8 width, uint8 height, float sigma))
{
    binary_threshold.H_canny_th = 220;
    binary_threshold.L_canny_th = 110;
    float kernel[KERNEL_SIZE];
    float a_array[height * width];
//    generateGaussianKernel(kernel, KERNEL_SIZE, sigma);
//    加上高斯模糊以后运算速度太慢或许没有padding可以加速
//    gaussianBlur(image, height,width, KERNEL_SIZE,sigma);
    fastGaussianBlur(image,height,width);
    sobelEdgeDetection(image, a_array,width, height);
    nonMaximalSuppression(image, a_array,width, height);
}

/* 
    此边缘检测法参照老办法,采取领域分析法进行，所有的边缘提取算法均对称
    因为可以通过手法把image_one_dimension跟image_two_dimension联系起来。
    就不需要更改输入canny跟EdgeExtraction的参数
	根据摄像头拍摄到的图像分析4边框对应的函数表达式得到顶点坐标

	attention:这种方法会导致新的坐标纸,如果出现多联通,如新加入的坐标纸上多了四个额外加分区域
	可能会导致的识别失败提前拐弯
 */
/**
 * @description: 上边缘坐标提取
 * @param {uint8_t} *r_array
 * @return {*}
 */
AT_ITCM_SECTION_INIT(uint8_t UpEdgeExtraction(uint8_t *r_array))
{
	//检测单侧邻接3个像素
	uint8_t point = 0;
	int16_t up_index_left = 0;
	uint8_t up_bool_left = 1;
	int16_t up_index_right = 0;
	uint8_t up_bool_right = 1;

	//检测上边缘,从中心位置开始,所以要保证中间是再矩形中,不然会错乱.
	for (int16_t index = demarcate_center; index > MT9V03X_W; index -= MT9V03X_W) //一次跳一行
	{
		if (r_array[index] == 255)//如果是白色边框才能进去,不行就继续上一行
		{
			//上左检测,index是整个图片中的那个像素点
			point = 0;
			up_index_left = index;
			while (up_bool_left)
			{
				/* 领域检测 */
				if (r_array[up_index_left - 1] == 255 || r_array[up_index_left - MT9V03X_W - 1] == 255 || r_array[up_index_left + MT9V03X_W - 1] == 255 ||
					r_array[up_index_left - 2] == 255 || r_array[up_index_left - MT9V03X_W - 2] == 255 || r_array[up_index_left + MT9V03X_W - 2] == 255)
					/* check around */
				{
					/* 如果周围有白色就放进来,并且记录当前坐标*/
					Egde[0][0][point++] = up_index_left;
					float pixel_sum = (float)(r_array[up_index_left - 1] + r_array[up_index_left - MT9V03X_W - 1] + r_array[up_index_left + MT9V03X_W - 1]);
					if ((int)pixel_sum)
					{
						uint16_t x3[3] = {up_index_left - MT9V03X_W - 1, up_index_left - 1, up_index_left + MT9V03X_W - 1};
						/*
						上中下三个地方给予不同侧权重,让其选择延伸方向(这里倒是可以仔细考虑延伸方向是否合理)
						这下就合理了为什么不用二值化,因为这样可以更好的看出边缘延伸的方向通过灰度图
						但是也可以通过双阈值滤波来得到接近的效果
						然后会更新up_index_left,到下一个更有可能的边界线
						之后再将其填入egde内
						*/
						up_index_left = x3[(int)round((float)(r_array[up_index_left - MT9V03X_W - 1] * 0 + r_array[up_index_left - 1] * 1 + r_array[up_index_left + MT9V03X_W - 1] * 2) /
													  pixel_sum)];
					}
					else /*如果没到边界*/
					{
						uint16_t x3[3] = {up_index_left - MT9V03X_W - 2, up_index_left - 2, up_index_left + MT9V03X_W - 2};
						up_index_left = x3[(int)round((float)(r_array[up_index_left - MT9V03X_W - 2] * 0 + r_array[up_index_left - 2] * 1 + r_array[up_index_left + MT9V03X_W - 2] * 2) /
													  (float)(r_array[up_index_left - 2] + r_array[up_index_left - MT9V03X_W - 2] + r_array[up_index_left + MT9V03X_W - 2]))];
					}
					if (point > MT9V03X_W / 2 - 3)
					//找了很多点了egde中心位置就放上有多少个点,不在存放像素点位置
					{
#if MT9V03X_H > MT9V03X_W
						Egde[0][0][MT9V03X_H / 2 - 1] = point;
#else
						Egde[0][0][MT9V03X_W / 2 - 1] = point;
#endif
						up_bool_left = 0;
					}
				}
				else/*左边不再有白点*/
				{
					/*如果超出最小边缘,所以可以看出放坐标纸的位置不能太离谱比如距离中心太远,且不能周围有杂质*/
					if (point > edge_size_limit)
					{
#if MT9V03X_H > MT9V03X_W
						Egde[0][0][MT9V03X_H / 2 - 1] = point;
#else
						Egde[0][0][MT9V03X_W / 2 - 1] = point;
#endif
						up_bool_left = 0;
					}
					else
					{
						break;
					}
				}
#if MT9V03X_H > MT9V03X_W
				if (point > MT9V03X_H / 2)
				{
					return 0;
				}
#else
				if (point > MT9V03X_W / 2)
				{
					return 0;
				}
#endif
			}
			point = 0;
			//上右检测
			up_index_right = index;
			while (up_bool_right)
			{
				if (r_array[up_index_right + 1] == 255 || r_array[up_index_right - MT9V03X_W + 1] == 255 || r_array[up_index_right + MT9V03X_W + 1] == 255 ||
					r_array[up_index_right + 2] == 255 || r_array[up_index_right - MT9V03X_W + 2] == 255 || r_array[up_index_right + MT9V03X_W + 2] == 255)
				{
					Egde[0][1][point++] = up_index_right;
					float pixel_sum = (float)(r_array[up_index_right + 1] + r_array[up_index_right - MT9V03X_W + 1] + r_array[up_index_right + MT9V03X_W + 1]);
					if ((int)pixel_sum)
					{
						uint16_t x3[3] = {up_index_right - MT9V03X_W + 1, up_index_right + 1, up_index_right + MT9V03X_W + 1};
						up_index_right = x3[(int)round((float)(r_array[up_index_right - MT9V03X_W + 1] * 0 + r_array[up_index_right + 1] * 1 + r_array[up_index_right + MT9V03X_W + 1] * 2) /
													   pixel_sum)];
					}
					else
					{
						uint16_t x3[3] = {up_index_right - MT9V03X_W + 2, up_index_right + 2, up_index_right + MT9V03X_W + 2};
						up_index_right = x3[(int)round((float)(r_array[up_index_right - MT9V03X_W + 2] * 0 + r_array[up_index_right + 2] * 1 + r_array[up_index_right + MT9V03X_W + 2] * 2) /
													   (float)(r_array[up_index_right + 2] + r_array[up_index_right - MT9V03X_W + 2] + r_array[up_index_right + MT9V03X_W + 2]))];
					}
					if (point > MT9V03X_W / 2 - 3)
					{
#if MT9V03X_H > MT9V03X_W
						Egde[0][1][MT9V03X_H / 2 - 1] = point;
#else
						Egde[0][1][MT9V03X_W / 2 - 1] = point;
#endif
						up_bool_right = 0;
					}
				}
				else
				{
					if (point > edge_size_limit)
					{
#if MT9V03X_H > MT9V03X_W
						Egde[0][1][MT9V03X_H / 2 - 1] = point;
#else
						Egde[0][1][MT9V03X_W / 2 - 1] = point;
#endif
						up_bool_right = 0;
					}
					else
					{
						break;
					}
				}
#if MT9V03X_H > MT9V03X_W
				if (point > MT9V03X_H / 2)
				{
					return 0;
				}
#else
				if (point > MT9V03X_W / 2)
				{
					return 0;
				}
#endif
			}
			point = 0;
		}
		if (!up_bool_left && !up_bool_right)
		{
			return 1;
		}
	}
	return 0;

}

/**
 * @description: 下边缘坐标提取
 * @param {uint8_t} *r_array
 * @return {*}
 */
AT_ITCM_SECTION_INIT(uint8_t DownEdgeExtraction(uint8_t *r_array))
{
	//检测单侧邻接3个像素
	uint8_t point = 0;
	int16_t down_index_left = 0;
	uint8_t down_bool_left = 1;
	int16_t down_index_right = 0;
	uint8_t down_bool_right = 1;

	//检测下边缘
	for (int16_t index = demarcate_center; index < MT9V03X_W * MT9V03X_H; index += MT9V03X_W)
	{
		if (r_array[index] == 255)
		{
			point = 0;
			//下左检测
			down_index_left = index;
			while (down_bool_left)
			{
				if (r_array[down_index_left - 1] == 255 || r_array[down_index_left - MT9V03X_W - 1] == 255 ||
					r_array[down_index_left + MT9V03X_W - 1] == 255 || r_array[down_index_left - 2] == 255 || r_array[down_index_left - MT9V03X_W - 2] == 255 ||
					r_array[down_index_left + MT9V03X_W - 2] == 255)
				{
					Egde[1][0][point++] = down_index_left;
					float pixel_sum = (float)(r_array[down_index_left - 1] + r_array[down_index_left - MT9V03X_W - 1] + r_array[down_index_left + MT9V03X_W - 1]);
					if ((int)pixel_sum)
					{
						uint16_t x3[3] = {down_index_left - MT9V03X_W - 1, down_index_left - 1, down_index_left + MT9V03X_W - 1};
						down_index_left = x3[(int16_t)round((float)(r_array[down_index_left - MT9V03X_W - 1] * 0 +
																	r_array[down_index_left - 1] * 1 +
																	r_array[down_index_left + MT9V03X_W - 1] * 2) /
															pixel_sum)];
					}
					else
					{
						uint16_t x3[3] = {down_index_left - MT9V03X_W - 2, down_index_left - 2, down_index_left + MT9V03X_W - 2};
						down_index_left = x3[(int16_t)round((float)(r_array[down_index_left - MT9V03X_W - 2] * 0 +
																	r_array[down_index_left - 2] * 1 +
																	r_array[down_index_left + MT9V03X_W - 2] * 2) /
															(float)(r_array[down_index_left - 2] + r_array[down_index_left - MT9V03X_W - 2] + r_array[down_index_left + MT9V03X_W - 2]))];
					}
					if (point > MT9V03X_W / 2)
					{
#if MT9V03X_H > MT9V03X_W
						Egde[1][0][MT9V03X_H / 2 - 1] = point;
#else
						Egde[1][0][MT9V03X_W / 2 - 1] = point;
#endif
						down_bool_left = 0;
					}
				}
				else
				{

					if (point > edge_size_limit)
					{
#if MT9V03X_H > MT9V03X_W
						Egde[1][0][MT9V03X_H / 2 - 1] = point;
#else
						Egde[1][0][MT9V03X_W / 2 - 1] = point;
#endif
						down_bool_left = 0;
					}
					else
					{
						break;
					}
				}
#if MT9V03X_H > MT9V03X_W
				if (point > MT9V03X_H / 2)
				{
					return 0;
				}
#else
				if (point > MT9V03X_W / 2)
				{
					return 0;
				}
#endif
			}
			point = 0;
			//下右检测
			down_index_right = index;
			while (down_bool_right)
			{
				if (r_array[down_index_right + 1] == 255 || r_array[down_index_right - MT9V03X_W + 1] == 255 ||
					r_array[down_index_right + MT9V03X_W + 1] == 255 || r_array[down_index_right + 2] == 255 || r_array[down_index_right - MT9V03X_W + 2] == 255 ||
					r_array[down_index_right + MT9V03X_W + 2] == 255)
				{
					Egde[1][1][point++] = down_index_right;
					float pixel_sum = (float)(r_array[down_index_right + 1] + r_array[down_index_right - MT9V03X_W + 1] + r_array[down_index_right + MT9V03X_W + 1]);
					if ((int)pixel_sum)
					{
						uint16_t x3[3] = {down_index_right - MT9V03X_W + 1, down_index_right + 1, down_index_right + MT9V03X_W + 1};
						down_index_right = x3[(int)round((float)(r_array[down_index_right - MT9V03X_W + 1] * 0 +
																 r_array[down_index_right + 1] * 1 +
																 r_array[down_index_right + MT9V03X_W + 1] * 2) /
														 pixel_sum)];
					}
					else
					{
						uint16_t x3[3] = {down_index_right - MT9V03X_W + 2, down_index_right + 2, down_index_right + MT9V03X_W + 2};
						down_index_right = x3[(int)round((float)(r_array[down_index_right - MT9V03X_W + 2] * 0 +
																 r_array[down_index_right + 2] * 1 +
																 r_array[down_index_right + MT9V03X_W + 2] * 2) /
														 (float)(r_array[down_index_right + 2] + r_array[down_index_right - MT9V03X_W + 2] + r_array[down_index_right + MT9V03X_W + 2]))];
					}
					if (point > MT9V03X_W / 2 - 3)
					{
#if MT9V03X_H > MT9V03X_W
						Egde[1][1][MT9V03X_H / 2 - 1] = point;
#else
						Egde[1][1][MT9V03X_W / 2 - 1] = point;
#endif
						down_bool_right = 0;
					}
				}
				else
				{
					if (point > edge_size_limit)
					{
#if MT9V03X_H > MT9V03X_W
						Egde[1][1][MT9V03X_H / 2 - 1] = point;
#else
						Egde[1][1][MT9V03X_W / 2 - 1] = point;
#endif
						down_bool_right = 0;
					}
					else
					{
						break;
					}
				}
#if MT9V03X_H > MT9V03X_W
				if (point > MT9V03X_H / 2)
				{
					return 0;
				}
#else
				if (point > MT9V03X_W / 2)
				{
					return 0;
				}
#endif
			}
			point = 0;
		}
		if (!down_bool_left && !down_bool_right)
		{
			return 1;
			break;
		}
	}
	return 0;
}

/**
 * @description: 左边缘坐标提取
 * @param {uint8_t} *r_array
 * @return {*}
 */
AT_ITCM_SECTION_INIT(uint8_t LeftEdgeExtraction(uint8_t *r_array))
{
	//检测单侧邻接3个像素
	uint8_t point = 0;
	int16_t left_index_up = 0;
	uint8_t left_bool_up = 1;
	int16_t left_index_down = 0;
	uint8_t left_bool_down = 1;

	//检测左边缘
	for (int16_t index = demarcate_center; index > demarcate_center - MT9V03X_W / 2; index--)
	{
		if (r_array[index] == 255)
		{
			point = 0;
			//左上检测
			left_index_up = index;
			while (left_bool_up)
			{
				if (left_index_up > 3 * MT9V03X_W)
				{
					if (r_array[left_index_up - MT9V03X_W] == 255 || r_array[left_index_up - MT9V03X_W - 1] == 255 || r_array[left_index_up - MT9V03X_W + 1] == 255 ||
						r_array[left_index_up - MT9V03X_W - MT9V03X_W] == 255 || r_array[left_index_up - MT9V03X_W - MT9V03X_W - 1] == 255 || r_array[left_index_up - MT9V03X_W - MT9V03X_W + 1] == 255)
					{
						Egde[2][0][point++] = left_index_up;
						int pixel_sum = r_array[left_index_up - MT9V03X_W] + r_array[left_index_up - MT9V03X_W - 1] + r_array[left_index_up - MT9V03X_W + 1];
						if (pixel_sum)
						{
							int widget_sum = (r_array[left_index_up - MT9V03X_W] * (left_index_up - MT9V03X_W) + r_array[left_index_up - MT9V03X_W - 1] * (left_index_up - MT9V03X_W - 1) + r_array[left_index_up - MT9V03X_W + 1] * (left_index_up - MT9V03X_W + 1));
							left_index_up = widget_sum / pixel_sum;
						}
						else
						{
							int widget_sum = r_array[left_index_up - MT9V03X_W - MT9V03X_W] * (left_index_up - MT9V03X_W - MT9V03X_W) + r_array[left_index_up - MT9V03X_W - MT9V03X_W - 1] * (left_index_up - MT9V03X_W - MT9V03X_W - 1) + r_array[left_index_up - MT9V03X_W - MT9V03X_W + 1] * (left_index_up - MT9V03X_W - MT9V03X_W + 1);
							left_index_up = widget_sum / (r_array[left_index_up - MT9V03X_W - MT9V03X_W] + r_array[left_index_up - MT9V03X_W - MT9V03X_W - 1] + r_array[left_index_up - MT9V03X_W - MT9V03X_W + 1]);
						}
						if (point > MT9V03X_H / 2 - 3)
						{
#if MT9V03X_H > MT9V03X_W
							Egde[2][0][MT9V03X_H / 2 - 1] = point;
#else
							Egde[2][0][MT9V03X_W / 2 - 1] = point;
#endif
							left_bool_up = 0;
						}
					}
					else
					{
						if (point > edge_size_limit)
						{
#if MT9V03X_H > MT9V03X_W
							Egde[2][0][MT9V03X_H / 2 - 1] = point;
#else
							Egde[2][0][MT9V03X_W / 2 - 1] = point;
#endif
							left_bool_up = 0;
						}
						else
						{
							break;
						}
					}
				}
				else
				{
					break;
				}
#if MT9V03X_H > MT9V03X_W
				if (point > MT9V03X_H / 2)
				{
					return 0;
				}
#else
				if (point > MT9V03X_W / 2)
				{
					return 0;
				}
#endif
			}
			point = 0;
			//左下检测
			left_index_down = index;
			while (left_bool_down)
			{
				if (left_index_down < (MT9V03X_H - 3) * MT9V03X_W)
				{
					if (r_array[left_index_down + MT9V03X_W] == 255 || r_array[left_index_down + MT9V03X_W - 1] == 255 || r_array[left_index_down + MT9V03X_W + 1] == 255 ||
						r_array[left_index_down + MT9V03X_W + MT9V03X_W] == 255 || r_array[left_index_down + MT9V03X_W + MT9V03X_W - 1] == 255 || r_array[left_index_down + MT9V03X_W + MT9V03X_W + 1] == 255)
					{
						Egde[2][1][point++] = left_index_down;
						int pixel_sum = r_array[left_index_down + MT9V03X_W] + r_array[left_index_down + MT9V03X_W - 1] + r_array[left_index_down + MT9V03X_W + 1];
						if (pixel_sum)
						{
							int widget_sum = (r_array[left_index_down + MT9V03X_W] * (left_index_down + MT9V03X_W) + r_array[left_index_down + MT9V03X_W - 1] * (left_index_down + MT9V03X_W - 1) + r_array[left_index_down + MT9V03X_W + 1] * (left_index_down + MT9V03X_W + 1)

							);
							left_index_down = widget_sum /
											  pixel_sum;
						}
						else
						{
							int widget_sum = (r_array[left_index_down + MT9V03X_W + MT9V03X_W] * (left_index_down + MT9V03X_W + MT9V03X_W) + r_array[left_index_down + MT9V03X_W + MT9V03X_W - 1] * (left_index_down + MT9V03X_W + MT9V03X_W - 1) + r_array[left_index_down + MT9V03X_W + MT9V03X_W + 1] * (left_index_down + MT9V03X_W + MT9V03X_W + 1)

							);
							left_index_down = widget_sum /
											  (r_array[left_index_down + MT9V03X_W + MT9V03X_W] + r_array[left_index_down + MT9V03X_W + MT9V03X_W - 1] + r_array[left_index_down + MT9V03X_W + MT9V03X_W + 1]);
						}
						if (point > MT9V03X_H / 2 - 3)
						{
#if MT9V03X_H > MT9V03X_W
							Egde[2][1][MT9V03X_H / 2 - 1] = point;
#else
							Egde[2][1][MT9V03X_W / 2 - 1] = point;
#endif
							left_bool_down = 0;
						}
					}
					else
					{
						if (point > edge_size_limit)
						{
#if MT9V03X_H > MT9V03X_W
							Egde[2][1][MT9V03X_H / 2 - 1] = point;
#else
							Egde[2][1][MT9V03X_W / 2 - 1] = point;
#endif
							left_bool_down = 0;
						}
						else
						{
							break;
						}
					}
				}
				else
				{
					break;
				}
			}
#if MT9V03X_H > MT9V03X_W
			if (point > MT9V03X_H / 2)
			{
				return 0;
			}
#else
			if (point > MT9V03X_W / 2)
			{
				return 0;
			}
#endif
			point = 0;
		}
		if (!left_bool_up && !left_bool_down)
		{
			return 1;
			break;
		}
	}
	return 0;
}

/**
 * @description: 右边缘坐标提取
 * @param {uint8_t} *r_array
 * @return {*}
 */
AT_ITCM_SECTION_INIT(uint8_t RightEdgeExtraction(uint8_t *r_array))
{
	//检测单侧邻接3个像素
	uint8_t point = 0;
	int16_t right_index_up = 0;
	uint8_t right_bool_up = 1;
	int16_t right_index_down = 0;
	uint8_t right_bool_down = 1;

	//检测右边缘
	for (int16_t index = demarcate_center; index < demarcate_center + MT9V03X_W / 2; index++)
	{
		if (r_array[index] == 255)
		{
			point = 0;
			//右上检测
			right_index_up = index;
			while (right_bool_up)
			{
				if (right_index_up > 3 * MT9V03X_W)
				{
					if (r_array[right_index_up - MT9V03X_W] == 255 || r_array[right_index_up - MT9V03X_W - 1] == 255 || r_array[right_index_up - MT9V03X_W + 1] == 255 ||
						r_array[right_index_up - MT9V03X_W - MT9V03X_W] == 255 || r_array[right_index_up - MT9V03X_W - MT9V03X_W - 1] == 255 || r_array[right_index_up - MT9V03X_W - MT9V03X_W + 1] == 255)
					{
						Egde[3][0][point++] = right_index_up;
						int pixel_sum = r_array[right_index_up - MT9V03X_W] + r_array[right_index_up - MT9V03X_W - 1] + r_array[right_index_up - MT9V03X_W + 1];
						if (pixel_sum)
						{
							int widget_sum = (r_array[right_index_up - MT9V03X_W] * (right_index_up - MT9V03X_W) + r_array[right_index_up - MT9V03X_W - 1] * (right_index_up - MT9V03X_W - 1) + r_array[right_index_up - MT9V03X_W + 1] * (right_index_up - MT9V03X_W + 1));
							right_index_up = widget_sum / pixel_sum;
						}
						else
						{
							int widget_sum = (r_array[right_index_up - MT9V03X_W - MT9V03X_W] * (right_index_up - MT9V03X_W - MT9V03X_W) + r_array[right_index_up - MT9V03X_W - MT9V03X_W - 1] * (right_index_up - MT9V03X_W - MT9V03X_W - 1) + r_array[right_index_up - MT9V03X_W - MT9V03X_W + 1] * (right_index_up - MT9V03X_W - MT9V03X_W + 1));
							right_index_up = widget_sum /
											 (r_array[right_index_up - MT9V03X_W - MT9V03X_W] + r_array[right_index_up - MT9V03X_W - MT9V03X_W - 1] + r_array[right_index_up - MT9V03X_W - MT9V03X_W + 1]);
						}
						if (point > MT9V03X_H / 2 - 3)
						{
#if MT9V03X_H > MT9V03X_W
							Egde[3][0][MT9V03X_H / 2 - 1] = point;
#else
							Egde[3][0][MT9V03X_W / 2 - 1] = point;
#endif
							right_bool_up = 0;
						}
					}
					else
					{
						if (point > edge_size_limit)
						{
#if MT9V03X_H > MT9V03X_W
							Egde[3][0][MT9V03X_H / 2 - 1] = point;
#else
							Egde[3][0][MT9V03X_W / 2 - 1] = point;
#endif
							right_bool_up = 0;
						}
						else
						{
							break;
						}
					}
				}
				else
				{
					break;
				}
#if MT9V03X_H > MT9V03X_W
				if (point > MT9V03X_H / 2)
				{
					return 0;
				}
#else
				if (point > MT9V03X_W / 2)
				{
					return 0;
				}
#endif
			}
			point = 0;
			//右下检测
			right_index_down = index;
			while (right_bool_down)
			{
				if (right_index_down < (MT9V03X_H - 3) * MT9V03X_W)
				{
					if (r_array[right_index_down + MT9V03X_W] == 255 || r_array[right_index_down + MT9V03X_W - 1] == 255 || r_array[right_index_down + MT9V03X_W + 1] == 255 ||
						r_array[right_index_down + MT9V03X_W + MT9V03X_W] == 255 || r_array[right_index_down + MT9V03X_W + MT9V03X_W - 1] == 255 || r_array[right_index_down + MT9V03X_W + MT9V03X_W + 1] == 255)
					{
						Egde[3][1][point++] = right_index_down;
						int pixel_sum = r_array[right_index_down + MT9V03X_W] + r_array[right_index_down + MT9V03X_W - 1] + r_array[right_index_down + MT9V03X_W + 1];
						if (pixel_sum)
						{
							int widget_sum = (r_array[right_index_down + MT9V03X_W] * (right_index_down + MT9V03X_W) + r_array[right_index_down + MT9V03X_W - 1] * (right_index_down + MT9V03X_W - 1) + r_array[right_index_down + MT9V03X_W + 1] * (right_index_down + MT9V03X_W + 1));
							right_index_down = widget_sum / pixel_sum;
						}
						else
						{
							int widget_sum = (r_array[right_index_down + MT9V03X_W + MT9V03X_W] * (right_index_down + MT9V03X_W + MT9V03X_W) + r_array[right_index_down + MT9V03X_W + MT9V03X_W - 1] * (right_index_down + MT9V03X_W + MT9V03X_W - 1) + r_array[right_index_down + MT9V03X_W + MT9V03X_W + 1] * (right_index_down + MT9V03X_W + MT9V03X_W + 1));
							right_index_down = widget_sum /
											   (r_array[right_index_down + MT9V03X_W + MT9V03X_W] + r_array[right_index_down + MT9V03X_W + MT9V03X_W - 1] + r_array[right_index_down + MT9V03X_W + MT9V03X_W + 1]);
						}
						if (point > MT9V03X_H / 2 - 3)
						{
#if MT9V03X_H > MT9V03X_W
							Egde[3][1][MT9V03X_H / 2 - 1] = point;
#else
							Egde[3][1][MT9V03X_W / 2 - 1] = point;
#endif
							right_bool_down = 0;
						}
					}
					else
					{
						if (point > edge_size_limit)
						{
#if MT9V03X_H > MT9V03X_W
							Egde[3][1][MT9V03X_H / 2 - 1] = point;
#else
							Egde[3][1][MT9V03X_W / 2 - 1] = point;
#endif
							right_bool_down = 0;
						}
						else
						{
							break;
						}
					}
				}
				else
				{
					break;
				}
#if MT9V03X_H > MT9V03X_W
				if (point > MT9V03X_H / 2)
				{
					return 0;
				}
#else
				if (point > MT9V03X_W / 2)
				{
					return 0;
				}
#endif
			}
			point = 0;
		}
		if (!right_bool_up && !right_bool_down)
		{
			return 1;
			break;
		}
	}
	return 0;
}


/**
 * @description: A4纸目标点检测(old)
 * @param {uint8_t} *r_array
 * @return {*}
 */
AT_ITCM_SECTION_INIT(uint8_t A4checktarget(uint8_t *r_array))
{
//	cannyEdgeDetection(r_array,MT9V03X_W,MT9V03X_H,Sigma);
	demarcate_center = A4_center;
	edge_size_limit = MT9V03X_H / 10; //可以依据上面边缘提取提出更高要求
	memset(Egde, 0, sizeof(Egde));
	memset(target_coord, 0, sizeof(target_coord));
	if (!UpEdgeExtraction(r_array))
	{
		zf_log(0,"Please correct the location of the A4");
		return 0;
	}
	if (!DownEdgeExtraction(r_array))
	{
		zf_log(0,"Please correct the location of the A4");
		return 0;
	}
	if (!LeftEdgeExtraction(r_array))
	{
		zf_log(0,"Please correct the location of the A4");
		return 0;
	}
	if (!RightEdgeExtraction(r_array))
	{
		zf_log(0,"Please correct the location of the A4");
		return 0;
	}
	if (!solveMatrixVertices(1))
	{
		return 0;
	}
	/*每条边线的长度可分析出*/
	float up_side_length = fast_sqrt((equation.root_coord[1][0] - equation.root_coord[0][0]) *
									(equation.root_coord[1][0] - equation.root_coord[0][0]) +
								(equation.root_coord[1][1] - equation.root_coord[0][1]) *
									(equation.root_coord[1][1] - equation.root_coord[0][1])),
		  down_side_length = fast_sqrt((equation.root_coord[3][0] - equation.root_coord[2][0]) *
									  (equation.root_coord[3][0] - equation.root_coord[2][0]) +
								  (equation.root_coord[3][1] - equation.root_coord[2][1]) *
									  (equation.root_coord[3][1] - equation.root_coord[2][1])),
		  left_side_length = fast_sqrt((equation.root_coord[2][0] - equation.root_coord[0][0]) *
									  (equation.root_coord[2][0] - equation.root_coord[0][0]) +
								  (equation.root_coord[2][1] - equation.root_coord[0][1]) *
									  (equation.root_coord[2][1] - equation.root_coord[0][1])),
		  right_side_length = fast_sqrt((equation.root_coord[3][0] - equation.root_coord[1][0]) *
									   (equation.root_coord[3][0] - equation.root_coord[1][0]) +
								   (equation.root_coord[3][1] - equation.root_coord[1][1]) *
									   (equation.root_coord[3][1] - equation.root_coord[1][1]));
	if (fabs(up_side_length - down_side_length) - edges_difference_degree > FLT_MIN_SELF || fabs(left_side_length - right_side_length) - edges_difference_degree > FLT_MIN_SELF)
	{
		/*这里也看出来,如果放的不够正,上下边线长度不太行,就会导致报错*/
		zf_log(0,"Please correct the location of the A4");
		return 0;
	}

	float
		img_length = (up_side_length + down_side_length) / 2,
		img_width = (left_side_length + right_side_length) / 2,
		//识别点的大小
		target_min_size = img_length * 0.03, target_max_size = target_min_size * 2;
		/* 最高高度,最低高度 */		
	uint8_t up_limit = (equation.root_coord[1][1] > equation.root_coord[0][1]) ? equation.root_coord[0][1] : equation.root_coord[1][1],
			down_limit = (equation.root_coord[3][1] > equation.root_coord[2][1]) ? equation.root_coord[3][1] : equation.root_coord[2][1],
			edge_point_coord[2][white_points_limitation] = {{0}, {0}}, X_index = 0, Y_index = up_limit;
	int16_t index = 0;
	uint16_t check_point = up_limit * MT9V03X_W;
	int8_t result = 0;
	//对点进行检测,这直接索引了所有点
	while (result != 2)
	{
		result = adjustEdgeLimit(&check_point, &X_index, &Y_index, up_limit, down_limit, target_min_size);//
		//符合要求的点才会放进来?,可以把左边界右边界放入进来减少运算量
		if (result == 1)
		{
			if (r_array[check_point] == 255)
			{
				edge_point_coord[0][index] = X_index;   //直接把x,y作为坐标,后续可以考虑把四个平均值作为中心
				edge_point_coord[1][index++] = Y_index; //直接把x,y作为坐标,后续可以考虑把四个平均值作为中心
				if (index >= white_points_limitation)
				{
					return 0;
				}
			}
			check_point++;
			X_index++;
			if (X_index > MT9V03X_W)
			{
				return 0;
			}
		}
		else if (result == -1)
		{
			return 0;
		}
	}
	uint8_t theoretical_detection_targets_count = 0;
	if (!clusterInitialization(edge_point_coord, target_min_size, target_max_size, &theoretical_detection_targets_count))
	{
		return 0;
	}

	/*调试使用*/
	if (!theoretical_detection_targets_count)
	{
//		PRINTF("change clusterInitialization parameters\n");
		return 0;
	}
// 进一步解算
	if (!kmeans(&theoretical_detection_targets_count, edge_point_coord, (uint8_t)ceil(3 * target_min_size)))
	{
		return 0;
	}

	/*调试使用*/
	if (!theoretical_detection_targets_count)
	{
//		PRINTF("change kmeans parameters\n");
		return 0;
	}

	// localBinaryFeedback(theoretical_detection_targets_count);

	absolute2RelativeCoordinates(theoretical_detection_targets_count);

	for (uint8_t i = 0; i < theoretical_detection_targets_count; i++)
	{
		if (target_coord[0][i] >= Venue_Length || target_coord[1][i] >= Venue_Width)
		{
			return 0;
		}
	}
	memset(target_coord[0] + theoretical_detection_targets_count, 0, object_num_limit - theoretical_detection_targets_count); //剔除多余点
	memset(target_coord[1] + theoretical_detection_targets_count, 0, object_num_limit - theoretical_detection_targets_count); //剔除多余点

	return theoretical_detection_targets_count; // return resultDetect(theoretical_detection_targets_count, r_array, target_min_size);
}




/* 内部函数 */
/**
 * @description: 判断检测点是否位于矩形边缘,(一个point的四方边检测将其看作是一个矩形对象)
 * @param {uint16_t} *check_point
 * @param {uint8_t} *X_index
 * @param {uint8_t} *Y_index
 * @param {uint8_t} up_limit
 * @param {uint8_t} down_limit
 * @return {*}
 */
AT_ITCM_SECTION_INIT(int8_t adjustEdgeLimit(uint16_t *check_point, uint8_t *X_index, uint8_t *Y_index, uint8_t up_limit, uint8_t down_limit, uint8_t Advance_amount))
{
	/**/
#define advance_amount 2
	uint8_t left, right, up, down;
	//没到就继续往下
	if ((*Y_index) < up_limit + advance_amount)
	{
		(*Y_index) += advance_amount;
		(*check_point) += advance_amount * MT9V03X_W;
	}
	//超出范围,最上左坐标为0,0
	if ((*Y_index) + advance_amount >= down_limit)
	{
		return 2;
	}
	else
	{
		if (up_limit == equation.root_coord[0][1]) //左边翘
		{
			if ((*Y_index) < equation.root_coord[2][1]) //在左下上
			{
				left = ceil(equation.coefficient_Y_left[1] * (*Y_index) + equation.coefficient_Y_left[0] + advance_amount);   //用左边的x = ky + b,向上取整,避免边缘检测不完全贴合
			}
			else
			{
				left = ceil(((*Y_index) - equation.coefficient_X_down[0]) / equation.coefficient_X_down[1] + advance_amount); //用下面的y = kx + b,避免边缘检测不完全贴合
			}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
			if ((*Y_index) < equation.root_coord[1][1]) //再右上上
			{
				right = floor(((*Y_index) - equation.coefficient_X_up[0]) / equation.coefficient_X_up[1] - advance_amount);   //向下取整,避免边缘检测不完全贴合
			}
			else
			{
				right = floor(equation.coefficient_Y_right[1] * (*Y_index) + equation.coefficient_Y_right[0] - advance_amount);//避免边缘检测不完全贴合
			}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
			if ((*X_index) < equation.root_coord[0][0])
			{
				up = ceil(((*X_index) - equation.coefficient_Y_left[0]) / equation.coefficient_Y_left[1] + advance_amount); //避免边缘检测不完全贴合
			}
			else
			{
				up = ceil(equation.coefficient_X_up[1] * (*X_index) + equation.coefficient_X_up[0] + advance_amount); //避免边缘检测不完全贴合
			}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
			if ((*X_index) < equation.root_coord[3][0])
			{
				down = floor(equation.coefficient_X_down[1] * (*X_index) + equation.coefficient_X_down[0] - advance_amount); //避免边缘检测不完全贴合
			}
			else
			{
				down = floor(((*X_index) - equation.coefficient_Y_right[0]) / equation.coefficient_Y_right[1] - advance_amount); //避免边缘检测不完全贴合
			}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
		}
		else
		{
			if ((*Y_index) < equation.root_coord[0][1])
			{
				left = ceil(((*Y_index) - equation.coefficient_X_up[0]) / equation.coefficient_X_up[1] + advance_amount); //避免边缘检测不完全贴合
			}
			else
			{
				left = ceil(equation.coefficient_Y_left[1] * (*Y_index) + equation.coefficient_Y_left[0] + advance_amount); //避免边缘检测不完全贴合
			}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
			if ((*Y_index) < equation.root_coord[3][1])
			{
				right = floor(equation.coefficient_Y_right[1] * (*Y_index) + equation.coefficient_Y_right[0] - advance_amount); //避免边缘检测不完全贴合
			}
			else
			{
				right = floor(((*Y_index) - equation.coefficient_X_down[0]) / equation.coefficient_X_down[1] - advance_amount); //避免边缘检测不完全贴合
			}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
			if ((*X_index) < equation.root_coord[1][0])
			{
				up = ceil(equation.coefficient_X_up[1] * (*X_index) + equation.coefficient_X_up[0] + advance_amount); //避免边缘检测不完全贴合
			}
			else
			{
				up = ceil(((*X_index) - equation.coefficient_Y_right[0]) / equation.coefficient_Y_right[1] + advance_amount); //避免边缘检测不完全贴合
			}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
			if ((*X_index) < equation.root_coord[2][0])
			{
				down = floor(((*X_index) - equation.coefficient_Y_left[0]) / equation.coefficient_Y_left[1] - advance_amount); //避免边缘检测不完全贴合
			}
			else
			{
				down = floor(equation.coefficient_X_down[1] * (*X_index) + equation.coefficient_X_down[0] - advance_amount); //避免边缘检测不完全贴合
			}
		}
	}
	if (left < right) //左边缘,右边缘
	{
		if ((*X_index) > right) //错误点不要了
		{
			(*X_index) = 0;
			(*Y_index)++;
			*check_point = (*Y_index) * MT9V03X_W;
			return 0;
		}
		else if ((*X_index) < left || (*Y_index) < up || (*Y_index) > down)
		{
			(*check_point)++;
			(*X_index)++;
			return 0;
		}
		else //满足才正确
		{
			return 1;
		}
	}
	else
	{
		return -1;
	}
}


/**
 * @description: 初步聚类初始点设置
 * @param {uint8_t} target_num
 * @param {uint16_t} point_count
 * @param {float} min_distance
 * @param {float} max_distance
 * @param {uint8_t} *theoretical_detection_targets_count
 * @return {*}
 */
AT_ITCM_SECTION_INIT(uint8_t clusterInitialization(uint8_t (*edge_point_coord)[white_points_limitation], float min_distance, float max_distance, uint8_t *theoretical_detection_targets_count))
{
	uint8_t target_count[object_num_limit] = {0};

	*theoretical_detection_targets_count = 0;

	/* 聚类数目确认方案 */
	target_coord[0][0] = edge_point_coord[0][0];
	target_coord[1][0] = edge_point_coord[1][0];
	target_count[0]++;//存放一个point包含几个edge_point_coord

	uint16_t i = 1;
	while (edge_point_coord[0][i] && edge_point_coord[1][i])
	{
		/*聚类很明显了,把edge_point_coord属于同一个point的放到一起去*/
		for (uint8_t j = 0; j <= *theoretical_detection_targets_count; j++)
		{
			uint8_t X_distance = abs(target_coord[0][j] - edge_point_coord[0][i]),
					Y_distance = abs(target_coord[1][j] - edge_point_coord[1][i]);
					//这里使用哈夫曼距离是因为,此时target_coord还尚处于离散状态里面存放的式x,y对应屏幕上的像素点的坐标,不能使用欧式距离
			if ((X_distance < (uint8_t)ceil(min_distance + max_distance)) && (Y_distance < (uint8_t)ceil(min_distance + max_distance)))
			{
				target_count[j]++;
				break;
			}
			else
			{
				if (*theoretical_detection_targets_count == j)
				{
					(*theoretical_detection_targets_count)++;
					//如果探索了前面所有的点依然超距离了,就说明又新增了一个点
					if ((*theoretical_detection_targets_count) > object_num_limit)
					{
						return 0;
					}
					target_coord[0][*theoretical_detection_targets_count] = edge_point_coord[0][i];
					target_coord[1][*theoretical_detection_targets_count] = edge_point_coord[1][i];
				}
			}
		}
		i++;
	}
	(*theoretical_detection_targets_count)++;
	return 1;
}

/* 内部函数 */
/**
 * @description: 聚类分析边缘点，求中心位置,k类均值聚类
 * @param {uint8_t} *target_num
 * @param {uint8_t} size_restriction
 * @return {*}
 */
AT_ITCM_SECTION_INIT(uint8_t kmeans(uint8_t *target_num, uint8_t (*edge_point_coord)[white_points_limitation], uint8_t size_restriction))
{
	uint16_t index = 0;
	uint8_t category_index[object_num_limit] = {0}, clustering_matrix[object_num_limit][2][100]; //聚类中心
	int8_t num_clustering_cycles = 1;															 // keams聚类循环次数（懒得写循环限制条件）

	memset(clustering_matrix, 0, sizeof(clustering_matrix));
	while (num_clustering_cycles--)
	{
		while (edge_point_coord[0][index] && edge_point_coord[1][index])
		{
			//找到距离edge_point_coord近的哈夫曼距离的target_coord
			uint8_t min_target = 0;
			uint16_t temp[2] = {10000};
			for (uint8_t i = 0; i < *target_num; i++)
			{
				temp[1] = abs(edge_point_coord[0][index] - target_coord[0][i]) +
						  abs(edge_point_coord[1][index] - target_coord[1][i]); //依旧是哈夫曼距离
				if (temp[0] > temp[1])
				{
					temp[0] = temp[1];
					min_target = i;
				}
			}
			//存放所有target_coord中最近的edge_point_coord
			clustering_matrix[min_target][0][category_index[min_target]] = edge_point_coord[0][index];
			clustering_matrix[min_target][1][category_index[min_target]] = edge_point_coord[1][index];
			category_index[min_target]++;
			index++;
		}
		for (uint8_t i = 0; i < *target_num; i++)
		{
			if (!category_index[i]) //没找到离他最近的
			{
				return 0;
			}
			uint16_t temp[2] = {0, 0};
			for (uint8_t j = 0; j < category_index[i]; j++)
			{
				temp[0] += clustering_matrix[i][0][j];
				temp[1] += clustering_matrix[i][1][j];
			}
			target_coord[0][i] = temp[0] / category_index[i];
			target_coord[1][i] = temp[1] / category_index[i];
			// 再去平均一下
		}
	}

	for (uint8_t i = 0; i < *target_num;)
	{
		if (category_index[i] < size_restriction)
		{
			for (uint8_t j = i; j < *target_num - 1; j++)
			{
				target_coord[0][j] = target_coord[0][j + 1];
				target_coord[1][j] = target_coord[1][j + 1];
				category_index[j] = category_index[j + 1];
			}
			(*target_num)--;
			continue;
		}
		i++;
	}

	float diameter[object_num_limit];
	/* 最小二乘法解圆 */
	for (uint8_t i = 0; i < *target_num; i++)
	{
		float center_x = 0;
		float center_y = 0;
		float radius = 0;
		float sum_x = 0, sum_y = 0;
		float sum_x2 = 0, sum_y2 = 0;
		float sum_x3 = 0, sum_y3 = 0;
		float sum_xy = 0, sum_x1y2 = 0, sum_x2y1 = 0;
		for (uint8_t j = 0; j < category_index[i]; j++)
		{
			float x = clustering_matrix[i][0][j];
			float y = clustering_matrix[i][1][j];
			float x2 = x * x;
			float y2 = y * y;
			sum_x += x;
			sum_y += y;
			sum_x2 += x2;
			sum_y2 += y2;
			sum_x3 += x2 * x;
			sum_y3 += y2 * y;
			sum_xy += x * y;
			sum_x1y2 += x * y2;
			sum_x2y1 += x2 * y;
		}

		float C, D, E, G, H;
		float a, b, c;

		C = category_index[i] * sum_x2 - sum_x * sum_x;
		D = category_index[i] * sum_xy - sum_x * sum_y;
		E = category_index[i] * sum_x3 + category_index[i] * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
		G = category_index[i] * sum_y2 - sum_y * sum_y;
		H = category_index[i] * sum_x2y1 + category_index[i] * sum_y3 - (sum_x2 + sum_y2) * sum_y;
		a = (H * D - E * G) / (C * G - D * D);
		b = (H * C - E * D) / (D * D - G * C);
		c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / category_index[i];

		target_coord[0][i] = a / (-2);
		target_coord[1][i] = b / (-2);
		diameter[i] = sqrt(a * a + b * b - 4 * c);
	}
	for (uint8_t i = 0; i < *target_num;)
	{
		if (diameter[i] < size_restriction / 3 || diameter[i] > size_restriction)
		{
			for (uint8_t j = i; j < *target_num - 1; j++)
			{
				target_coord[0][j] = target_coord[0][j + 1];
				target_coord[1][j] = target_coord[1][j + 1];
				diameter[j] = diameter[j + 1];
			}
			(*target_num)--;
			continue;
		}
		i++;
	}
	return 1;
}

/**
 * @description: 获取目标点相对坐标,将屏幕上对应的像素点的坐标,真实的转换为A4纸上对应的纸面坐标
 * @param {uint8_t} target_num
 * @return {*}
 */
AT_ITCM_SECTION_INIT(void absolute2RelativeCoordinates(uint8_t target_num))
{
	for (uint8_t i = 0; i < target_num; i++)
	{	// perspectiveTransformation(&target_coord[0][i], &target_coord[1][i]);
		// 
		float left_edge_dis = fabs(target_coord[1][i] * equation.coefficient_Y_left[1] - target_coord[0][i] + equation.coefficient_Y_left[0] - 1.0) / sqrt(equation.coefficient_Y_left[1] * equation.coefficient_Y_left[1] + 1),
			  up_edge_dis = fabs(target_coord[0][i] * equation.coefficient_X_up[1] - target_coord[1][i] + equation.coefficient_X_up[0] - 1.0) / sqrt(equation.coefficient_X_up[1] * equation.coefficient_X_up[1] + 1),
			  right_edge_dis = fabs(target_coord[1][i] * equation.coefficient_Y_right[1] - target_coord[0][i] + equation.coefficient_Y_right[0] + 1.0) / sqrt(equation.coefficient_Y_right[1] * equation.coefficient_Y_right[1] + 1),
			  down_edge_dis = fabs(target_coord[0][i] * equation.coefficient_X_down[1] - target_coord[1][i] + equation.coefficient_X_down[0] + 1.0) / sqrt(equation.coefficient_X_down[1] * equation.coefficient_X_down[1] + 1);

#define SUBLENGTH 700
#define SUBWIDTH 500
		target_coord[0][i] = left_edge_dis / (left_edge_dis + right_edge_dis) * SUBLENGTH;
		target_coord[1][i] = down_edge_dis / (up_edge_dis + down_edge_dis) * SUBWIDTH;
	}

	memset(target_coord[0] + target_num, 0, object_num_limit - target_num); //剔除多余点
	memset(target_coord[1] + target_num, 0, object_num_limit - target_num); //剔除多余点
}