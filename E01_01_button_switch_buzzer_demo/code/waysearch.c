#include "waysearch.h"

#include <math.h>
#include <time.h>
#include <stdlib.h>

double CityPos[STATIC_ARRAY_SIZE][2] = {0};
//	{
//	{10,10},
//	{50,430},
//	{50,210},
//	{90,90},
//	{170,390},
//	{190,190},
//	{250,259},
//	{370,470},
//	{410,250},
//	{450,90},
//	{550,410},
//	{530,210},
//	{670,350} };

#define M 30

int NcMax = 300;
/*参数的初始化过程*/
double alpha = 2, _beta = 3, rou = 0.1, alpha1 = 0.1, qzero = 0.01;
/*用于存放全局的移动*/
extern int globalTour[STATIC_ARRAY_SIZE][2];

double Lnn;
//矩阵表示两两城市之间的距离
double allDistance[STATIC_ARRAY_SIZE][STATIC_ARRAY_SIZE];

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
extern bool target_update;

//计算两个城市之间的距离
double calculateDistance(int i, int j)
{
	return sqrt(pow((CityPos[i][0] - CityPos[j][0]), 2.0) + pow((CityPos[i][1] - CityPos[j][1]), 2.0));
}

//由矩阵表示两两城市之间的距离
void calculateAllDistance(int N)
{
	for (int i = 0; i < N; i++)
	{
		/*这里也许可以改成 j <= i*/
		for (int j = 0; j < N; j++)
		{
			if (i != j)
			{
				allDistance[i][j] = calculateDistance(i, j);
				allDistance[j][i] = allDistance[i][j];
			}
		}
	}
}

//获得经过n个城市的路径长度吗，tour是一个N*2的矩阵
double calculateSumOfDistance(int *tour, int N)
{
	double sum = 0;
	for (int i = 0; i < N; i++)
	{	
		/*行更像一个索引号，列才是存放了需要的参数*/
		int row = *(tour + 2 * i); 
		int col = *(tour + 2 * i + 1);
		sum += allDistance[row][col];
	}
	return sum;
}

//------------------------------------------
//选择下一个节点，配合下面的函数来计算的长度
int ChooseNextNode(int currentNode, int visitedNode[], int N)
{
	int nextNode = -1;
	double shortDistance = 0.0;
	for (int i = 0; i < N; i++)
	{
		//去掉已走过的节点,从剩下节点中选择距离最近的节点
		if (1 == visitedNode[i])
		{
			if (shortDistance == 0.0)
			{
				shortDistance = allDistance[currentNode][i];
				nextNode = i;
			}
			if (shortDistance > allDistance[currentNode][i])
			{
				/* code */
				shortDistance = allDistance[currentNode][i];
				nextNode = i;
			}
			/*貌似有bug下面为原话*/
			//if (shortDistance < allDistance[currentNode][i])
			//{
			//	nextNode = i;
			//}
		}
	}
	return nextNode;
}

//给一个节点由最近邻距离方法计算长度
double CalAdjacentDistance(int node, int N)
{
	double sum = 0.0;
	int visitedNode[STATIC_ARRAY_SIZE];
	/*所有点除了自己都没走过*/
	for (int j = 0; j < N; j++)
	{
		visitedNode[j] = 1;
	}
	visitedNode[node] = 0;
	int currentNode = node;
	int nextNode;
	do
	{
		nextNode = ChooseNextNode(currentNode, visitedNode, N);
		if (nextNode >= 0)
		{
			sum += allDistance[currentNode][nextNode];
			currentNode = nextNode;
			visitedNode[currentNode] = 0;
		}
	} while (nextNode >= 0);
	sum += allDistance[currentNode][node];
	return sum;
}

struct AntColonySystem;
struct ACSAnt;
/*
定义了一个函数指针的方法仔细看仿佛更像是一个面向对象的方式
里面甚至定义了一个this指针self的方式，方便理解  
*/
struct AntColonySystem
{
	int N;
	double info[STATIC_ARRAY_SIZE][STATIC_ARRAY_SIZE], visible[STATIC_ARRAY_SIZE][STATIC_ARRAY_SIZE]; //节点之间的信息素强度,节点之间的能见度
	//信息素，我们还是简单的定义成1/长度就好了
	void (*AntColonySystem_Constructor)(struct AntColonySystem *ptr_this, int in_N);
	void (*AntColonySystem_Destructor)(struct AntColonySystem *ptr_this);

	double (*AntColonySystem_Transition)(struct AntColonySystem *ptr_this, int i, int j);
	void (*AntColonySystem_UpdateLocalPathRule)(struct AntColonySystem *ptr_this, int i, int j);
	void (*AntColonySystem_InitParameter)(struct AntColonySystem *ptr_this, double value);
	void (*AntColonySystem_UpdateGlobalPathRule)(struct AntColonySystem *ptr_this, int *bestTour, double globalBestLength);
};

void AntColonySystem_Constructor(struct AntColonySystem *ptr_this, int in_N)
{
	if (ptr_this != 0)
	{
		ptr_this->N = in_N;
	}
}

void AntColonySystem_Destructor(struct AntColonySystem *ptr_this)
{
	if (ptr_this != 0)
	{
	}
}

//计算当前节点到下一节点转移的概率
double AntColonySystem_Transition(struct AntColonySystem *ptr_this, int i, int j)
{
	if (ptr_this != 0)
	{
		if (i != j)
		{
			/*
			信息素浓度乘以可见度（启发信息）
			可以看下面为什么是这样
			*/
			return (pow(ptr_this->info[i][j], alpha) * pow(ptr_this->visible[i][j], _beta));
		}
		else
		{
			return 0.0;
		}
	}
	return 0.0;
}

//局部更新规则
void AntColonySystem_UpdateLocalPathRule(struct AntColonySystem *ptr_this, int i, int j)
{
	if (ptr_this != 0)
	{
		/*
		信息素更新，这里的局部更新引用的还是最短路径生成的总长度，参考值相对来说也较少，
		这是发生在第一次走完蚁群之前的
		纵观全局却也没有更改Lnn
		*/
		ptr_this->info[i][j] = (1.0 - alpha1) * ptr_this->info[i][j] + alpha1 * (1.0 / (ptr_this->N * Lnn));
		ptr_this->info[j][i] = ptr_this->info[i][j];
	}
}

//初始化
void AntColonySystem_InitParameter(struct AntColonySystem *ptr_this, double value)
{
	if (ptr_this != 0)
	{
		//初始化路径上的信息素强度tao0（是作为后续更新信息素所使用的参数）
		for (int i = 0; i < ptr_this->N; i++)
		{
			//for (int j = 0; j < ptr_this->N; j++)
			/*代码对称性*/
			for (int j = 0; j <= i; j++)
			{
				ptr_this->info[i][j] = value;
				ptr_this->info[j][i] = value;
				if (i != j)
				{
					ptr_this->visible[i][j] = 1.0 / allDistance[i][j];
					ptr_this->visible[j][i] = ptr_this->visible[i][j];
				}
			}
		}
	}
}

//全局信息素更新，在走完一趟以后更新
void AntColonySystem_UpdateGlobalPathRule(struct AntColonySystem *ptr_this, int *bestTour, double globalBestLength)
{
	if (ptr_this != 0)
	{
		for (int i = 0; i < ptr_this->N; i++)
		{
			int row = *(bestTour + 2 * i);
			int col = *(bestTour + 2 * i + 1);
			//老的信息素保存含量会挥发一部分+新增的信息素
			ptr_this->info[row][col] = (1.0 - rou) * ptr_this->info[row][col] + rou * (1.0 / globalBestLength);
			ptr_this->info[col][row] = ptr_this->info[row][col];
		}
	}
}

void AntColonySystemInitialize(struct AntColonySystem *ptr_this)
{
	if (ptr_this != 0)
	{
		ptr_this->AntColonySystem_Constructor = AntColonySystem_Constructor;
		ptr_this->AntColonySystem_Destructor = AntColonySystem_Destructor;
		ptr_this->AntColonySystem_Transition = AntColonySystem_Transition;
		ptr_this->AntColonySystem_UpdateLocalPathRule = AntColonySystem_UpdateLocalPathRule;
		ptr_this->AntColonySystem_InitParameter = AntColonySystem_InitParameter;
		ptr_this->AntColonySystem_UpdateGlobalPathRule = AntColonySystem_UpdateGlobalPathRule;
	}
}

struct ACSAnt
{
	int N;
	struct AntColonySystem *antColony;
	int startCity, cururentCity;	//初始城市编号，当前城市编号
	int allowed[STATIC_ARRAY_SIZE]; //禁忌表
	int Tour[STATIC_ARRAY_SIZE][2]; //当前路径
	int currentTourIndex;			//当前路径索引，从0开始，存储蚂蚁经过城市的编号

	void (*ACSAnt_Constructor)(struct ACSAnt *ptr_this, struct AntColonySystem *acs, int start, int in_N);
	void (*ACSAnt_Destructor)(struct ACSAnt *ptr_this, struct AntColonySystem *acs, int start);
	int *(*ACSAnt_Search)(struct ACSAnt *ptr_this);
	int (*ACSAnt_Choose)(struct ACSAnt *ptr_this);
	void (*ACSAnt_MoveToNextCity)(struct ACSAnt *ptr_this, int nextCity);
};

void ACSAnt_Constructor(struct ACSAnt *ptr_this, struct AntColonySystem *acs, int start, int in_N)
{
	if (ptr_this != 0)
	{
		ptr_this->antColony = acs;
		ptr_this->startCity = start;
		ptr_this->N = in_N;
	}
}

void ACSAnt_Destructor(struct ACSAnt *ptr_this, struct AntColonySystem *acs, int start)
{
	if (ptr_this != 0)
	{
	}
}

//开始搜索
int *ACSAnt_Search(struct ACSAnt *ptr_this)
{
	if (ptr_this != 0)
	{
		ptr_this->cururentCity = ptr_this->startCity;
		int toCity;
		ptr_this->currentTourIndex = 0;
		for (int i = 0; i < ptr_this->N; i++)
		{
			ptr_this->allowed[i] = 1;
		}
		ptr_this->allowed[ptr_this->cururentCity] = 0;
		int endCity;
		int count = 0;
		do
		{
			count++;
			endCity = ptr_this->cururentCity;
			toCity = ptr_this->ACSAnt_Choose(ptr_this);
			if (toCity >= 0)
			{
				ptr_this->ACSAnt_MoveToNextCity(ptr_this, toCity);
				ptr_this->antColony->AntColonySystem_UpdateLocalPathRule(ptr_this->antColony, endCity, toCity);
				//局部更新信息素，这是一种在线更新的方法，走一步就更改一次信息素
				ptr_this->cururentCity = toCity;
			}
		} while (toCity >= 0);
		ptr_this->ACSAnt_MoveToNextCity(ptr_this, ptr_this->startCity);
		ptr_this->antColony->AntColonySystem_UpdateLocalPathRule(ptr_this->antColony, endCity, ptr_this->startCity);

		return *(ptr_this->Tour);
	}
	else
	{
		return 0;
	}
}

//选择下一节点
int ACSAnt_Choose(struct ACSAnt *ptr_this)
{
	if (ptr_this != 0)
	{
		int nextCity = -1;
		double q = rand() / (double)RAND_MAX;
		//如果 q <= q0,按先验知识，否则则按概率转移，
		//这个参数不知道为什么
		if (q <= qzero)
		{
			double probability = -1.0; //转移到下一节点的概率
			for (int i = 0; i < ptr_this->N; i++)
			{
				//由面向对象的知识，这里的currentCity随ptr_this传递过来
				//去掉禁忌表中已走过的节点,从剩下节点中选择最大概率的可行节点
				if (1 == ptr_this->allowed[i])
				{
					double prob = ptr_this->antColony->AntColonySystem_Transition(ptr_this->antColony, ptr_this->cururentCity, i);
					if (prob > probability)
					{
						nextCity = i;
						probability = prob;
					}
				}
			}
		}
		else
		{
			//按概率转移
			double p = rand() / (double)RAND_MAX; //生成一个随机数,用来判断落在哪个区间段
			double sum = 0.0;
			double probability = 0.0; //概率的区间点，p 落在哪个区间段，则该点是转移的方向
			//计算概率公式的分母的值
			for (int i = 0; i < ptr_this->N; i++)
			{
				if (1 == ptr_this->allowed[i])
				{
					sum += ptr_this->antColony->AntColonySystem_Transition(ptr_this->antColony, ptr_this->cururentCity, i);
				}
			}
			for (int j = 0; j < ptr_this->N; j++)
			{
				if (1 == ptr_this->allowed[j] && sum > 0)
				{
					probability += ptr_this->antColony->AntColonySystem_Transition(ptr_this->antColony, ptr_this->cururentCity, j) / sum;
					/*经典轮盘赌算法*/
					if (probability >= p || (p > 0.9999 && probability > 0.9999))
					{
						nextCity = j;
						break;
					}
				}
			}
		}
		//如果没有城市了返回了-1
		return nextCity;
	}
	else
	{
		return 0;
	}
}

//移动到下一节点
void ACSAnt_MoveToNextCity(struct ACSAnt *ptr_this, int nextCity)
{
	if (ptr_this != 0)
	{
		ptr_this->allowed[nextCity] = 0;
		ptr_this->Tour[ptr_this->currentTourIndex][0] = ptr_this->cururentCity;
		ptr_this->Tour[ptr_this->currentTourIndex][1] = nextCity;
		/*这里就可以看出来这个编号也是递增的用于一个一个索引*/
		ptr_this->currentTourIndex++;
		ptr_this->cururentCity = nextCity;
	}
}

void ACSAntInitialize(struct ACSAnt *ptr_this)
{
	if (ptr_this != 0)
	{
		ptr_this->ACSAnt_Constructor = ACSAnt_Constructor;
		ptr_this->ACSAnt_Destructor = ACSAnt_Destructor;
		ptr_this->ACSAnt_Search = ACSAnt_Search;
		ptr_this->ACSAnt_Choose = ACSAnt_Choose;
		ptr_this->ACSAnt_MoveToNextCity = ACSAnt_MoveToNextCity;
	}
}

//---------------------------------结束---------------------------------------------

//蚁群系统对象
struct AntColonySystem acs;
struct ACSAnt ants[M];

int Ant_findBestPath(int N, float *bestDistanceLength)
{
	/*
	产生随机值，从srand (seed)中指定的seed开始，返回一个[seed, RAND_MAX（0x7fff）)间的随机整数
	srand(uint16_t seed);可以播下一个随机种子
	*/
	srand(30);

	for (int i = 0; i < N - 1; i++)
	{
		CityPos[i][0] = target_coord[0][i];
		CityPos[i][1] = target_coord[1][i];
	}
	CityPos[N - 1][0] = coord.current_coord_x;
	CityPos[N - 1][1] = coord.current_coord_y;

	//由矩阵表示两两城市之间的距离
	calculateAllDistance(N);

	AntColonySystemInitialize(&acs);
	acs.AntColonySystem_Constructor(&acs, N);

	//蚂蚁均匀分布在城市上
	for (int k = 0; k < M; k++)
	{
		ACSAntInitialize(&ants[k]);
		/*用取余的方法可以很好的均分*/
		ants[k].ACSAnt_Constructor(&ants[k], &acs, (int)(k % N), N);
	}
	/*
	calculateAllDistance(N);
	这句貌似多余了
	*/
	
	//随机选择一个节点计算由最近邻方法得到的一个长度
	int node = rand() % N;
	Lnn = CalAdjacentDistance(node, N);

	//各条路径上初始化的信息素强度，引用了最短路径算法得到长度
	double initInfo = 1 / (N * Lnn);
	acs.AntColonySystem_InitParameter(&acs, initInfo);

	//全局最优长度
	double globalBestLength = 0.0;
	for (int i = 0; i < NcMax; i++)
	{
		//局部最优路径
		int localTour[STATIC_ARRAY_SIZE][2];
		//局部最优长度
		double localBestLength = 0.0;
		//当前路径长度
		double tourLength;
		for (int j = 0; j < M; j++)
		{
			int *tourPath = ants[j].ACSAnt_Search(&ants[j]);
			tourLength = calculateSumOfDistance(tourPath, N);
			//局部比较，并记录路径和长度
			if (tourLength < localBestLength || fabs(localBestLength - 0.0) < FLT_MIN_SELF)
			{
				for (int m = 0; m < N; m++)
				{
					int row = *(tourPath + 2 * m);
					int col = *(tourPath + 2 * m + 1);
					localTour[m][0] = row;
					localTour[m][1] = col;
				}
				localBestLength = tourLength;
			}
		}
		//全局比较，并记录路径和长度
		if (localBestLength < globalBestLength || fabs(globalBestLength - 0.0) < FLT_MIN_SELF)
		{
			for (int m = 0; m < N; m++)
			{
				globalTour[m][0] = localTour[m][0];
				globalTour[m][1] = localTour[m][1];
			}
			globalBestLength = localBestLength;
		}
		acs.AntColonySystem_UpdateGlobalPathRule(&acs, *globalTour, globalBestLength);
	}

	//输出全局最优路径
	*bestDistanceLength = (float)globalBestLength;

	// for (int m = 0; m < N; m++)
	//{
	//	cout << globalTour[m][0] << ".";
	// }

	return 1;
}


/**
 * @description: 模拟退火获取最优解
 * @param {*}
 * @return {*}
 */
#define T_start 1000.0  //初始温度，20最适，升高和降低都会增加路径长度
#define T_end (1e-4)    //截止温度，-4最优，升高会增加路径长度，减小会增加运算时间，但结果不变
#define q 0.965         //降温系数，0.965最优，升高或降低都会导致路径增长
#define L 100           //重复降温次数,20最优，升高和降低都会导致路径增长
double city[OBJNUM][2]; //城市个数
void creat()
{
    int point1, point2, temp;
    point1 = rand() % OBJNUM;  //取余数，18个city中任意一个
    point2 = rand() % OBJNUM;  //取余数，18个city中任意一个
    temp = city_result[point1];
    city_result[point1] = city_result[point2];
    city_result[point2] = temp;
}
double distance(double *city1, double *city2)
{
    double x1, x2, y1, y2, dis;
    x1 = *city1;
    x2 = *city2;
    y1 = *(city1 + 1);
    y2 = *(city2 + 1);
    dis = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    return dis;
}
double path(uint16_t *city_sort)
{
    double sum = 0;
    sum += fast_sqrt(pow(city[city_sort[0]][0], 2) + pow(city[city_sort[0]][1], 2));
    for (uint8_t i = 0; i < OBJNUM - 1; i++)
    { 
        sum += distance((double *)city[city_sort[i]], (double *)city[city_sort[i + 1]]);
    }
    //    sum += sqrt(pow(target_coord[0][city_sort[OBJNUM - 1]], 2) + pow(target_coord[1][city_sort[OBJNUM - 1]], 2)); //路径目标中包括起点
    return sum;
}
/// @brief 模拟退火 
void annealingDeterminesPath()
{
    uint16_t city_copyresult[OBJNUM];
    double path1, path2;
    double dE;
    double r;
    double T;

    for (uint8_t i = 0; i < OBJNUM; i++)
    {
        city_result[i] = i;
        city[i][0] = target_coord[0][i];
        city[i][1] = target_coord[1][i];
    }
    T = T_start;
    while (T > T_end)
    {
        for (int i = 0; i < L; i++)			//进行重复选点
        {
            memcpy(city_copyresult, city_result, OBJNUM * sizeof(uint16_t));
            creat();                        //直接生成顺序，然后再把任意两个换位子
            path1 = path(city_copyresult);  //city_result和city_copyresult都是索引号
            path2 = path(city_result);      //city_result和city_copyresult都是索引号
            dE = path2 - path1;
            if (dE > 0)						//改了以后反而变长了，则是有概率记录
            {
                r = rand();
                if (exp(-dE / T) *(RAND_MAX) <= r)
                    memcpy(city_result, city_copyresult, OBJNUM * sizeof(uint16_t));
            }
        }
        T *= q;
    }
    //    for (uint16_t i = 0; i < OBJNUM; i++)
    //    {
    //        PRINTF("index:%d,%d\n", i, city_result[i]);
    //    }
}
