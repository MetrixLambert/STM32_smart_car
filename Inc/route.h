
#ifndef __ROUTE_H__
#define __ROUTE_H__
#include "main.h"
#include "stm32f1xx_hal.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#define ROUTENUM 30
#define Route_Num_Max 10
#define SQUARE_THRE 12
struct RoadNode
{
	int id;
	struct RoadNode *next;
};

typedef struct Line{
	struct  RoadNode* head;
	struct  RoadNode* rear;
}Queue,Stack;

//area
//square
int is_in_square(int x, int y);
int dis_to_center(int start_x, int start_y, int end_x, int end_y);
void square(int start_x, int start_y, int end_x, int end_y);
int  square_get_id(int x, int y);
void current_area(int x,int y,int *roadid,int *road_next_node);
void target_area(int x,int y,int *roadid,int *road_prev_node);
int node_to_road(int prev,int next);
int dist_to_node(int x,int y,int node);
//void road_to_node(int roadid,short *i);
void insert_node_start( int road, int x, int y);
void insert_node_end( int road, int x, int y);
void road_io_node(int road_id);
//Stack
void InitStack(Stack *S);
void Push(Stack *S,int NodeId);
struct RoadNode* Pop(Stack *S);
int StackEmpty(Stack *S);
void DeleteStack(Stack *S);
//Queue
void InitQueue(Queue *Q);
void EnQueue(Queue *Q,int NodeId);
struct RoadNode* DeQueue(Queue *Q);
int QueueEmpty(Queue *Q);
void DeleteQueue(Queue *Q);
void cal_route(int start_x,int start_y,int end_x,int end_y);

#endif
