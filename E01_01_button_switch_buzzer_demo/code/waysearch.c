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
/*�����ĳ�ʼ������*/
double alpha = 2, _beta = 3, rou = 0.1, alpha1 = 0.1, qzero = 0.01;
/*���ڴ��ȫ�ֵ��ƶ�*/
extern int globalTour[STATIC_ARRAY_SIZE][2];

double Lnn;
//�����ʾ��������֮��ľ���
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
Ŀ���ַ
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

//������������֮��ľ���
double calculateDistance(int i, int j)
{
	return sqrt(pow((CityPos[i][0] - CityPos[j][0]), 2.0) + pow((CityPos[i][1] - CityPos[j][1]), 2.0));
}

//�ɾ����ʾ��������֮��ľ���
void calculateAllDistance(int N)
{
	for (int i = 0; i < N; i++)
	{
		/*����Ҳ����Ըĳ� j <= i*/
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

//��þ���n�����е�·��������tour��һ��N*2�ľ���
double calculateSumOfDistance(int *tour, int N)
{
	double sum = 0;
	for (int i = 0; i < N; i++)
	{	
		/*�и���һ�������ţ��в��Ǵ������Ҫ�Ĳ���*/
		int row = *(tour + 2 * i); 
		int col = *(tour + 2 * i + 1);
		sum += allDistance[row][col];
	}
	return sum;
}

//------------------------------------------
//ѡ����һ���ڵ㣬�������ĺ���������ĳ���
int ChooseNextNode(int currentNode, int visitedNode[], int N)
{
	int nextNode = -1;
	double shortDistance = 0.0;
	for (int i = 0; i < N; i++)
	{
		//ȥ�����߹��Ľڵ�,��ʣ�½ڵ���ѡ���������Ľڵ�
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
			/*ò����bug����Ϊԭ��*/
			//if (shortDistance < allDistance[currentNode][i])
			//{
			//	nextNode = i;
			//}
		}
	}
	return nextNode;
}

//��һ���ڵ�������ھ��뷽�����㳤��
double CalAdjacentDistance(int node, int N)
{
	double sum = 0.0;
	int visitedNode[STATIC_ARRAY_SIZE];
	/*���е�����Լ���û�߹�*/
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
������һ������ָ��ķ�����ϸ���·������һ���������ķ�ʽ
��������������һ��thisָ��self�ķ�ʽ���������  
*/
struct AntColonySystem
{
	int N;
	double info[STATIC_ARRAY_SIZE][STATIC_ARRAY_SIZE], visible[STATIC_ARRAY_SIZE][STATIC_ARRAY_SIZE]; //�ڵ�֮�����Ϣ��ǿ��,�ڵ�֮����ܼ���
	//��Ϣ�أ����ǻ��Ǽ򵥵Ķ����1/���Ⱦͺ���
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

//���㵱ǰ�ڵ㵽��һ�ڵ�ת�Ƶĸ���
double AntColonySystem_Transition(struct AntColonySystem *ptr_this, int i, int j)
{
	if (ptr_this != 0)
	{
		if (i != j)
		{
			/*
			��Ϣ��Ũ�ȳ��Կɼ��ȣ�������Ϣ��
			���Կ�����Ϊʲô������
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

//�ֲ����¹���
void AntColonySystem_UpdateLocalPathRule(struct AntColonySystem *ptr_this, int i, int j)
{
	if (ptr_this != 0)
	{
		/*
		��Ϣ�ظ��£�����ľֲ��������õĻ������·�����ɵ��ܳ��ȣ��ο�ֵ�����˵Ҳ���٣�
		���Ƿ����ڵ�һ��������Ⱥ֮ǰ��
		�ݹ�ȫ��ȴҲû�и���Lnn
		*/
		ptr_this->info[i][j] = (1.0 - alpha1) * ptr_this->info[i][j] + alpha1 * (1.0 / (ptr_this->N * Lnn));
		ptr_this->info[j][i] = ptr_this->info[i][j];
	}
}

//��ʼ��
void AntColonySystem_InitParameter(struct AntColonySystem *ptr_this, double value)
{
	if (ptr_this != 0)
	{
		//��ʼ��·���ϵ���Ϣ��ǿ��tao0������Ϊ����������Ϣ����ʹ�õĲ�����
		for (int i = 0; i < ptr_this->N; i++)
		{
			//for (int j = 0; j < ptr_this->N; j++)
			/*����Գ���*/
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

//ȫ����Ϣ�ظ��£�������һ���Ժ����
void AntColonySystem_UpdateGlobalPathRule(struct AntColonySystem *ptr_this, int *bestTour, double globalBestLength)
{
	if (ptr_this != 0)
	{
		for (int i = 0; i < ptr_this->N; i++)
		{
			int row = *(bestTour + 2 * i);
			int col = *(bestTour + 2 * i + 1);
			//�ϵ���Ϣ�ر��溬����ӷ�һ����+��������Ϣ��
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
	int startCity, cururentCity;	//��ʼ���б�ţ���ǰ���б��
	int allowed[STATIC_ARRAY_SIZE]; //���ɱ�
	int Tour[STATIC_ARRAY_SIZE][2]; //��ǰ·��
	int currentTourIndex;			//��ǰ·����������0��ʼ���洢���Ͼ������еı��

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

//��ʼ����
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
				//�ֲ�������Ϣ�أ�����һ�����߸��µķ�������һ���͸���һ����Ϣ��
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

//ѡ����һ�ڵ�
int ACSAnt_Choose(struct ACSAnt *ptr_this)
{
	if (ptr_this != 0)
	{
		int nextCity = -1;
		double q = rand() / (double)RAND_MAX;
		//��� q <= q0,������֪ʶ�������򰴸���ת�ƣ�
		//���������֪��Ϊʲô
		if (q <= qzero)
		{
			double probability = -1.0; //ת�Ƶ���һ�ڵ�ĸ���
			for (int i = 0; i < ptr_this->N; i++)
			{
				//����������֪ʶ�������currentCity��ptr_this���ݹ���
				//ȥ�����ɱ������߹��Ľڵ�,��ʣ�½ڵ���ѡ�������ʵĿ��нڵ�
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
			//������ת��
			double p = rand() / (double)RAND_MAX; //����һ�������,�����ж������ĸ������
			double sum = 0.0;
			double probability = 0.0; //���ʵ�����㣬p �����ĸ�����Σ���õ���ת�Ƶķ���
			//������ʹ�ʽ�ķ�ĸ��ֵ
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
					/*�������̶��㷨*/
					if (probability >= p || (p > 0.9999 && probability > 0.9999))
					{
						nextCity = j;
						break;
					}
				}
			}
		}
		//���û�г����˷�����-1
		return nextCity;
	}
	else
	{
		return 0;
	}
}

//�ƶ�����һ�ڵ�
void ACSAnt_MoveToNextCity(struct ACSAnt *ptr_this, int nextCity)
{
	if (ptr_this != 0)
	{
		ptr_this->allowed[nextCity] = 0;
		ptr_this->Tour[ptr_this->currentTourIndex][0] = ptr_this->cururentCity;
		ptr_this->Tour[ptr_this->currentTourIndex][1] = nextCity;
		/*����Ϳ��Կ�����������Ҳ�ǵ���������һ��һ������*/
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

//---------------------------------����---------------------------------------------

//��Ⱥϵͳ����
struct AntColonySystem acs;
struct ACSAnt ants[M];

int Ant_findBestPath(int N, float *bestDistanceLength)
{
	/*
	�������ֵ����srand (seed)��ָ����seed��ʼ������һ��[seed, RAND_MAX��0x7fff��)����������
	srand(uint16_t seed);���Բ���һ���������
	*/
	srand(30);

	for (int i = 0; i < N - 1; i++)
	{
		CityPos[i][0] = target_coord[0][i];
		CityPos[i][1] = target_coord[1][i];
	}
	CityPos[N - 1][0] = coord.current_coord_x;
	CityPos[N - 1][1] = coord.current_coord_y;

	//�ɾ����ʾ��������֮��ľ���
	calculateAllDistance(N);

	AntColonySystemInitialize(&acs);
	acs.AntColonySystem_Constructor(&acs, N);

	//���Ͼ��ȷֲ��ڳ�����
	for (int k = 0; k < M; k++)
	{
		ACSAntInitialize(&ants[k]);
		/*��ȡ��ķ������Ժܺõľ���*/
		ants[k].ACSAnt_Constructor(&ants[k], &acs, (int)(k % N), N);
	}
	/*
	calculateAllDistance(N);
	���ò�ƶ�����
	*/
	
	//���ѡ��һ���ڵ����������ڷ����õ���һ������
	int node = rand() % N;
	Lnn = CalAdjacentDistance(node, N);

	//����·���ϳ�ʼ������Ϣ��ǿ�ȣ����������·���㷨�õ�����
	double initInfo = 1 / (N * Lnn);
	acs.AntColonySystem_InitParameter(&acs, initInfo);

	//ȫ�����ų���
	double globalBestLength = 0.0;
	for (int i = 0; i < NcMax; i++)
	{
		//�ֲ�����·��
		int localTour[STATIC_ARRAY_SIZE][2];
		//�ֲ����ų���
		double localBestLength = 0.0;
		//��ǰ·������
		double tourLength;
		for (int j = 0; j < M; j++)
		{
			int *tourPath = ants[j].ACSAnt_Search(&ants[j]);
			tourLength = calculateSumOfDistance(tourPath, N);
			//�ֲ��Ƚϣ�����¼·���ͳ���
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
		//ȫ�ֱȽϣ�����¼·���ͳ���
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

	//���ȫ������·��
	*bestDistanceLength = (float)globalBestLength;

	// for (int m = 0; m < N; m++)
	//{
	//	cout << globalTour[m][0] << ".";
	// }

	return 1;
}


/**
 * @description: ģ���˻��ȡ���Ž�
 * @param {*}
 * @return {*}
 */
#define T_start 1000.0  //��ʼ�¶ȣ�20���ʣ����ߺͽ��Ͷ�������·������
#define T_end (1e-4)    //��ֹ�¶ȣ�-4���ţ����߻�����·�����ȣ���С����������ʱ�䣬���������
#define q 0.965         //����ϵ����0.965���ţ����߻򽵵Ͷ��ᵼ��·������
#define L 100           //�ظ����´���,20���ţ����ߺͽ��Ͷ��ᵼ��·������
double city[OBJNUM][2]; //���и���
void creat()
{
    int point1, point2, temp;
    point1 = rand() % OBJNUM;  //ȡ������18��city������һ��
    point2 = rand() % OBJNUM;  //ȡ������18��city������һ��
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
    //    sum += sqrt(pow(target_coord[0][city_sort[OBJNUM - 1]], 2) + pow(target_coord[1][city_sort[OBJNUM - 1]], 2)); //·��Ŀ���а������
    return sum;
}
/// @brief ģ���˻� 
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
        for (int i = 0; i < L; i++)			//�����ظ�ѡ��
        {
            memcpy(city_copyresult, city_result, OBJNUM * sizeof(uint16_t));
            creat();                        //ֱ������˳��Ȼ���ٰ�����������λ��
            path1 = path(city_copyresult);  //city_result��city_copyresult����������
            path2 = path(city_result);      //city_result��city_copyresult����������
            dE = path2 - path1;
            if (dE > 0)						//�����Ժ󷴶��䳤�ˣ������и��ʼ�¼
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
