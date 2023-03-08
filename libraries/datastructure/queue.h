/*
 * @Author: lzx
 * @Date: 2023-02-04 18:00:07
 * @Description: ╤сап
 * @Vision: V1.0
 */
#ifndef _QUEUE_H
#define _QUEUE_H
#include "zf_common_headfile.h"
typedef struct Queue {
	int theSize;
	int capacity;
	void **dataStore;
	
	void (*enqueue)();
	void *(*dequeue)();
	int (*size)();
	bool (*isEmpty)();
	void *(*font)();
	void *(*end)();
	void (*clear)();
	void (*destory)();
} Queue;

/* Prototypes */
Queue *queueCreate();
void queueEnqueue(Queue *, void *);
void *queueDequeue(Queue *);
int queueSize(Queue *);
bool queueIsEmpty(Queue *);
void *queueFont(Queue *);
void *queueEnd(Queue *);
void queueClear(Queue *);
void queueDestory(Queue *);





#endif