#include "route.h"
#include "control.h"
#include "main.h"

#define NODE_LEN 20
#define RoadNum 25
#define NodeNum 12
#define ROUTENUM 30
#define INTMAX 1000
#define crossing 26
#define NO_ROAD 60

short route_num;			//the num of route
short current_route;		//to remember which road we are in
short route[ROUTENUM];			//save the route
short road_len;	//save the len of each road

const short key_point[59][2]=
{
	{78,42},//1
	{68,69},//2
	{38,78},//3
	{15,189},//4
	{81,256},//5
	{166,163},//6
	{256,85},//7
	{188,13},//8
	{30,186},//9
	{30,240},//10 
	{70,240},//11
	{60,73},//12->42
	{89,221},//13 
	{147,160},//14
	{24,124},//15 
	{160,146},//16
	{240,70},//17->19
	{30,186},//18->9
	{240,70},//19
	{243,32},//20
	{186,30},//21
	{26,22},//22 
	{186,30},//23->21
	{127,26},//24 
	{71,58},//25
	{94,18},//26 
	{119,5},//27 
	{186,10},//28
	{186,10},//29->28
	{264,10},//30 
	{260,76},//31
	{260,76},//32->31
	{197,145},//33->45
	{162,175},//34
	{80,260},//35
	{105,266},//36 
	{192,260},//37 
	{208,246},//38 
	{204,217},//39 
	{162,175},//40->34
	{147,160},//41->14
	{60,73},//42
	{71,58},//43 -> 25
	{160,146},//44->16
	{176,160},//45
	{219,203},//46 
	{243,206},//47 
	{260,185},//48 
	{268,119},//49 
	{30,85},//50
	{6,118},//51 
	{10,186},//52
	{10,186},//53->52
	{9,264},//54 
	{80,260},//55->35
	{28,70},//56
	{70,28},//57
	{28,50},//58
	{50,28}//59
};
/*
//CAD (x,y)
const short key_point[59[2]=
{
	{78,40}, //1
	{68,69}, //2
	{40,78},//3 
	{20,185},//4
	{78,260},//5
	{161,161},//6
	{260,78},//7
	{185,20},//8
	{30,185},//9
	{30,240},//10
	{79,240},//11
	{60,73},//12
	{95,214},//13
	{147,160},//14
	{30,123},//15
	{161,147},//16
	{240,67},//17
	{30,185},//18
	{240,67},//19
	{240,30},//20
	{185,30},//21
	{27,27},//22
	{185,30},//23
	{126,30},//24
	{71,58},//25
	{96,25},//26
	{118,11},//27
	{185,10},//28
	{185,10},//29
	{260,10},//30
	{260,78},//31
	{260,78},//32
	{176,160},//33
	{160,175},//34
	{78,260},//35
	{122,260},//36
	{189,260}, //37
	{208,246},//38
	{205,217},//39
	{160,175},//40
	{147,160},//41
	{60,73},//42
	{71,58},//43
	{161,147},//44
	{176,160},//45
	{220,203},//46 
	{243,206},//47
	{260,185},//48
	{260,78},//49
	{30,85},//50
	{10,120},//51
	{10,186},//52
	{10,186},//53
	{10,260},//54
	{78,260},//55
	{23,70}, //56
	{71,30},//57
	{20,50},//58
	{50,20}//59
*/
/****distance between node and node*****/
const int node_distance[NodeNum][NodeNum]=
{
	{0,32,103,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,26},//1
	{32,0,32,INTMAX,INTMAX,98,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX},//2
	{103,32,0,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,60,INTMAX,INTMAX},//3
	{INTMAX,INTMAX,INTMAX,0,95,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,66,INTMAX},//4
	{INTMAX,INTMAX,INTMAX,142,0,110,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX},//5
	{INTMAX,98,INTMAX,INTMAX,120,0,111,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX},//6
	{INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,118,0,91,INTMAX,INTMAX,INTMAX,INTMAX},//7
	{INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,143,0,59,INTMAX,INTMAX,INTMAX},//8
	{61,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,0,INTMAX,INTMAX,INTMAX},//9
	{INTMAX,INTMAX,INTMAX,63,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,0,INTMAX,INTMAX},//10
	{INTMAX,INTMAX,39,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,0,INTMAX},//11
	{INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,INTMAX,66,INTMAX,INTMAX,INTMAX,0}//12
};	

/*****************************************************************************************
****Description:find out the shortest path between current(x,y) and target(x,y)***********
******************************************************************************************/
void cal_route(int start_x,int start_y,int end_x,int end_y)
{
	int i;
	for (i = 0; i < ROUTENUM; i++)
	{
		route[i] = 0;
	}
	route_num = 0;
	if (is_in_square(start_x, start_y)*is_in_square(end_x, end_y) == 1)
	{
		route[0] = 0;
		square(start_x, start_y, end_x, end_y);
		route[route_num] = -1;
		printf("(%d,%d)->", start_x, start_y);
		for (i = 1; i < route_num; i++)
		{
			printf("(%d,%d)->", key_point[route[i] - 1][0], key_point[route[i] - 1][1]);
		}
		printf("(%d,%d)\n",end_x, end_y);
		return;
	}
	
	
	
	int source_road,end_road;
	int current_next_node,target_prev_node;
	int road_route[NodeNum];
	int node_route[NodeNum]={0,0,0,0,0,0,0,0,0,0,0,0};		//to save the route of node			
	int dist[NodeNum]={0,0,0,0,0,0,0,0,0,0,0,0};
	for( i=0; i<NodeNum; i++)
	{
		road_route[i]=0;
	}
	
	current_area(start_x,start_y,&source_road,&current_next_node);
	target_area(end_x,end_y,&end_road,&target_prev_node);
	
	if (source_road == -1 || end_road == -1)
	{
		road_len = INTMAX * 10;
	}

	int source=current_next_node-1;
	int end=target_prev_node-1;

/*SPFA start
**
*/
	int visited[NodeNum];
	for( i=0; i<NodeNum; i++)
	{
		visited[i]=0;
		dist[i]=INTMAX;
		node_route[i]=source;
	}//???

	Queue Q;
	InitQueue(&Q);
	EnQueue(&Q,source);
	dist[source]=0;
	visited[source]=1;

	while(!QueueEmpty(&Q))
	{
		struct RoadNode *p;
		p=DeQueue(&Q);
		int u=p->id;
		free(p);
		
		visited[u]=0;
		int v;

		for( v=0; v<NodeNum; v++)
		{
			if(node_distance[u][v]!= INTMAX)
			{
				if(dist[u] + node_distance[u][v] <dist[v])//????
				{
					dist[v]=dist[u]+node_distance[u][v];
					node_route[v]=u;
					if(!visited[v])
					{
						EnQueue(&Q,v);
						visited[v]=1;
					}
				}
			}
		}
	}

/*SPFA end
**
*/
			int p = end;
            Stack S;
			InitStack(&S);
            while (p != source)  //????????,???????
            {
                Push(&S,p);
                p = node_route[p];
            }
			
			int node_list[20];
			int node_number=0;
			node_list[node_number++]=source+1;
			printf("node_list:%d->",node_list[0]);
			struct RoadNode *tmp;
			while (!StackEmpty(&S)) 
            {
				tmp=Pop(&S);
				node_list[node_number++]=tmp->id+1;
//				printf("%d->",node_list[node_number-1]);
				free(tmp);
            }
			//if (node_list[node_number - 1] == 3 && node_list[node_number - 2] == 2)
			//{
			//	node_list[node_number - 1] = 0;
			//	node_list[node_number - 2] = 0;
			//	node_number-=2;
			//}
			DeleteStack(&S);
			DeleteQueue(&Q);
			
			i=0;
			int road_number=node_number-1;
				road_number++;
				road_route[i]=source_road;
				
				printf("road_route:%d",source_road);
				for(i=1;i<road_number;i++)
				{	
					road_route[i]=node_to_road(node_list[i-1],node_list[i]);
					printf("%d->", road_route[i]);
					if(road_route[i]==NO_ROAD)
					{
						#ifdef ROUTE_DEBUG 
							printf("no road\r\n");
						#endif
						road_route[i]=0;
					}
				}

			road_number++;
			road_route[road_number-1]=end_road;
			printf("%d\n",road_route[road_number-1]);

			route_num=0;
			route[route_num++]=0;//start
			insert_node_start(source_road,start_x,start_y);
			if (is_in_square(start_x, start_y) == 1)
			{
				int last_node = 0;
				int out_node = 0;
				int time = 0;
				for (i = 1; i < road_number - 1; i++)
				{
					road_io_node(road_route[i]);
			
					if (time==0&&is_in_square(key_point[route[route_num - 1]-1][0], key_point[route[route_num - 1]-1][1]) == 0)
					{
						time = 1;
						out_node = route[route_num - 1];
						route[route_num - 1] = 0;
						route_num -= 1;
						last_node = route[route_num - 1];
						route[route_num - 1] = 0;
						route_num -= 1;
						while (route[route_num - 1]!= 0)
						{
							route[route_num - 1] = 0;
							route_num--;
						}
						square(start_x, start_y, key_point[last_node-1][0], key_point[last_node-1][1]);
						route[route_num] = last_node;
						route_num++;
						route[route_num] = out_node;
						route_num++;
					}
		
				}
				insert_node_end(end_road, end_x, end_y);
			}
			else if (is_in_square(end_x, end_y) == 1)
			{
				//int out_node = 0;
				int first_node = 0;
				int time = 0;
				for (i = 1; i < road_number - 1; i++)
				{
					road_io_node(road_route[i]);
					if (time == 0 && is_in_square(key_point[route[route_num - 1] - 1][0], key_point[route[route_num - 1] - 1][1]) == 1)
					{
						time = 1;
						first_node = route[route_num - 1];
						break;
						//if (route[route_num - 2] != 0)
						//{
						//	out_node = route[route_num - 1];
						//}
					}
					/*else if (time == 1)
					{
						if(route[route_num-1]==first_node)
						{ }
						else
						{
							route[route_num - 1] = 0;
							route_num -= 1;
						}
					}*/
				}
				square(key_point[first_node - 1][0], key_point[first_node - 1][1], end_x, end_y);
				route[route_num] = -1;
			}
			else
			{
				for (i = 1; i < road_number - 1; i++)
				{
					road_io_node(road_route[i]);
				}
				insert_node_end(end_road, end_x, end_y);
			}
			/*if (road_route[road_number - 1] == 2 && road_route[road_number - 2] == 1)
			{
				if (end_y > end_x)
				{
					route_num--;
					route[route_num] = 42;
					route_num++;
					if (dis_to_center(key_point[42-1][0],key_point[42-1][1],end_x,end_y)<15)
					{
						route[route_num] = 56;
						route_num++;
					}
				}
				else
				{
					route_num -= 2;
					route[route_num] = 25;
					route_num++;
					if (dis_to_center(key_point[25 - 1][0], key_point[25 - 1][1], end_x, end_y) < 15)
					{
						route[route_num] = 57;
						route_num++;
					}
					
				}
			}*/
			route[route_num]=-1;//end



			for(i=0; i<route_num;i++)
			{
			printf("%d->",route[i]);
			}
			printf("%d\n",route[i]);

			printf("(%d,%d)->",start_x,start_y);
			for( i=1; i<route_num; i++)
			{
				printf("(%d,%d)->",key_point[route[i]-1][0],key_point[route[i]-1][1]);
			}
			printf("(%d,%d)\n",end_x,end_y);
			


			
			dist[end]+=dist_to_node(start_x,start_y,current_next_node)+
			dist_to_node(end_x,end_y,target_prev_node);
			road_len=dist[end]+(route_num-1)*NODE_LEN;
			printf("distance :%d\n",road_len);
}


/**************************************************************************************/
/*******************Stack**************************************************************/
/**************************************************************************************/
void InitStack(Stack *S)
{
	struct RoadNode *p;
	p=(struct RoadNode*)malloc(sizeof(struct RoadNode));
	S->head=NULL;
	S->rear=NULL;
	
	p->id=-1;
	p->next=NULL;
	S->head=p;
	S->rear=p;
}
void DeleteStack(Stack *S)
{
	free(S->head);
}
void Push(Stack *S,int NodeId)
{
	struct RoadNode* e=(struct RoadNode*)malloc(sizeof(struct RoadNode));
	e->id=NodeId;
	struct RoadNode *p=(*S).head;
	(*S).head=e;
	e->next=p;
}
struct RoadNode *Pop(Stack *S)
{
	struct RoadNode *p=(*S).head;
	(*S).head=p->next;
	return p;
}
int StackEmpty(Stack *S)
{
	if((*S).head==(*S).rear) return 1;
	else return 0;
}
/**************************************************************************************/
/*******************Queue**************************************************************/
/**************************************************************************************/
void InitQueue(Queue *Q)
{
	struct RoadNode *p;
	p=(struct RoadNode*)malloc(sizeof(struct RoadNode));
	
	p->id=-1;
	p->next=NULL;
	Q->head=p;
	Q->rear=p;
}
void EnQueue(Queue *Q,int NodeId)
{
	struct RoadNode *e=(struct RoadNode*)malloc(sizeof(struct RoadNode));
	e->id=NodeId;
	(*Q).rear->next=e;
	(*Q).rear=e;
	(*Q).rear->next=NULL;
}
struct RoadNode* DeQueue(Queue *Q)
{
	struct RoadNode *p=(*Q).head->next;
	if(p==(*Q).rear)
	{
		(*Q).head->next=p->next;
		(*Q).rear=(*Q).head;
	}
	else
	(*Q).head->next=p->next;

	return p;
}
void DeleteQueue(Queue *Q)
{
	free(Q->head);
}
int QueueEmpty(Queue *Q)
{
	if((*Q).head==(*Q).rear) return 1;
	else return 0;
}
/********************************************************************************
******************area***********************************************************
********************************************************************************/

/*****Square****/
int is_in_square(int x, int y)
{
	if ((x - 50)*(x - 50) + (y - 50)*(y - 50) <= 55*55 ||
		(y <= 0.17*x + 95 && x + y <= 170 && x <= 0.17*y + 82))
	{
		return 1;
	}
	else return 0;
}

int dis_to_center(int start_x, int start_y, int end_x, int end_y)
{
	int A = end_y - start_y;
	int B = start_x - end_x;
	int C = end_x * start_y - start_x * end_y;
	int d = abs((int)((50 * A + 50 * B + C) / sqrt(A*A + B * B)));
	return d;
}

void square( int start_x, int start_y, int end_x, int end_y)
{
	int d = dis_to_center(start_x,start_y,end_x,end_y);
	int start_square_id = 0;
	int end_square_id = 0;

	if (d >= SQUARE_THRE)
	{
	}
	else
	{
		start_square_id = square_get_id(start_x, start_y);
		end_square_id = square_get_id(end_x, end_y);

		switch (start_square_id)
		{
		case 1:
			if (end_square_id == 1)//do not run out of line
			{
				if ((start_y - 50)*(end_y - 50) >= 0)
				{
				}
				else
				{
					route[route_num] = 25;
					route_num++;
				}
			}
			else if (end_square_id == 2)
			{

				route[route_num] = 2;
				route_num += 1;
			}
			else if (end_square_id == 3)
			{
				if (start_y >= 50 && end_y >= 50)
				{
					route[route_num] = 42;
					route_num += 1;
				}
				else if (start_y < 50 && end_y < 50)
				{
					route[route_num] = 59;
					route_num += 1;
				}
				else if(start_y>=50)
				{
					route[route_num] = 57;
					route_num += 1;
					if (dis_to_center(key_point[57 - 1][0], key_point[57 - 1][1], end_x, end_y) < SQUARE_THRE)
					{
						route[route_num] = 22;
						route_num += 1;
					}
				}
				else
				{
					route[route_num] = 2;
					route_num += 1;
					if (dis_to_center(key_point[2 - 1][0], key_point[2 - 1][1], end_x, end_y) < SQUARE_THRE)
					{
						route[route_num] = 56;
						route_num += 1;
					}
				}

			}
			else
			{
				route[route_num] = 57;
				route_num += 1;
			}
			break;
		case 2:
			if (end_square_id == 1)
			{
				route[route_num] = 2;
				route_num += 1;
			}
			else if (end_square_id == 2)//do not run out of line 
			{
				if ((start_x - 50)*(end_x - 50) >= 0)
				{
				}
				else
				{
					route[1] = 42;
					route_num += 1;
				}
			}
			else if (end_square_id == 3)
			{
				route[route_num] = 56;
				route_num += 1;
			}
			else
			{
				if (start_x <= 50 && end_x <= 50)
				{
					route[route_num] = 58;
					route_num += 1;
				}
				else if (start_x > 50 && end_x > 50)
				{
					route[route_num] = 25;
					route_num += 1;
				}
				else if(start_x<=50)
				{
					route[route_num] = 2;
					route_num += 1;
					if(dis_to_center(key_point[route[route_num-1]-1][0],key_point[route[route_num-1]-1][1],end_x,end_y)<SQUARE_THRE)
					{
						route[route_num] = 57;
						route_num += 1;
					}
				}
				else
				{
				route[route_num] = 56;
				route_num += 1;
				if (dis_to_center(key_point[route[route_num - 1] - 1][0], key_point[route[route_num - 1] - 1][1], end_x, end_y) < SQUARE_THRE)
				{
					route[route_num] = 22;
					route_num += 1;
				}
				}
			}
			break;
		case 3:
			if (end_square_id == 1)
			{
				if (start_y >= 50 && end_y >= 50)
				{
					route[route_num] = 42;
					route_num += 1;
				}
				else if (start_y < 50 && end_y < 50)
				{
					route[route_num] = 59;
					route_num += 1;
				}
				else if (start_y >= 50)
				{
					route[route_num] = 22;
					route_num += 1;
					if (dis_to_center(key_point[route[route_num - 1] - 1][0], key_point[route[route_num - 1] - 1][1], end_x, end_y) < SQUARE_THRE)
					{
						route[route_num] = 57;
						route_num += 1;
					}
				}
				else
				{
					route[route_num] = 56;
					route_num += 1;
					if (dis_to_center(key_point[route[route_num - 1] - 1][0], key_point[route[route_num - 1] - 1][1], end_x, end_y) < SQUARE_THRE)
					{ 
					route[route_num] = 2;
					route_num += 1;
					}
				}
			}
			else if (end_square_id == 2)
			{
				route[route_num] = 56;
				route_num += 1;
			}
			else if (end_square_id == 3)//do not run out of line
			{
				if ((start_y - 50)*(end_y - 50) >= 0)
				{
				}
				else
				{
					route[route_num] = 58;
					route_num += 1;
				}
			}
			else
			{
				route[route_num] = 22;
				route_num += 1;
			}
			break;
		case 4:
			if (end_square_id == 1)
			{
				route[route_num] = 57;
				route_num += 1;
			}
			else if (end_square_id == 2)
			{
				if (start_x >= 50 && end_x >= 50)
				{
					route[route_num] = 25;
					route_num += 1;
				}
				else if (start_x < 50 && end_x < 50)
				{
					route[route_num] = 58;
					route_num += 1;
				}
				else if (start_x >= 50)
				{
					route[route_num] = 22;
					route_num += 1;
					if (dis_to_center(key_point[route[route_num - 1] - 1][0], key_point[route[route_num - 1] - 1][1], end_x, end_y) <SQUARE_THRE)
					{
						route[route_num] = 56;
						route_num += 1;
					}
				}
				else
				{
					route[route_num] = 57;
					route_num += 1;
					if (dis_to_center(key_point[route[route_num - 1] - 1][0], key_point[route[route_num - 1] - 1][1], end_x, end_y) < SQUARE_THRE)
					{ 
					route[route_num] = 2;
					route_num += 1;
					}
				}
			}
			else if (end_square_id == 3)
			{
				route[route_num] = 22;
				route_num += 1;
			}
			else//do not run out of line
			{
				if ((start_x - 50)*(end_x - 50) >= 0)
				{
				}
				else
				{
					route[route_num] = 59;
					route_num += 1;
				}
			}
			break;
		default:
			break;
		}
	}

}

int  square_get_id(int x, int y)
{
	if (y >= x)
	{
		if (x + y >= 100)
		{
			return 2;
		}
		else
		{
			return 3;
		}
	}
	else
	{
		if (x + y > 100)
		{
			return 1;
		}
		else
		{
			return 4;
		}
	}
}
/******************************
Description:Based on x,y, return roadid,road_next_rode
******************************/
void current_area(int x,int y,int *roadid,int *road_next_node)
{
	//Node_4
	if((y>=165-2  && y<=205-2 && x>=0 && x<=40)||
		( x>=55 && x<=80 && y>=170 && y<=200))
	{
		*roadid=26;
		*road_next_node=4;
		return;
	}
	
	//Node_5
	if(x >= 59-2 && x <= 122 && y <= 270
		&& y >= -0.71*(x - 57) + 229 && y >=1.04* (x-88) + 207)
	{
		//59 229 ->86 208-> 115 237 ->122 249
		//57 229 ->88 207 -> 116 236 ->121 249
		*roadid = 27;
		*road_next_node = 5;
		return;
	}
	
	//Node_6
	if(y>=x-50 && y<=x+50 && x+y<=373 && x+y>=274)
	{
		*roadid = 28;
		*road_next_node = 6;
		return;
	}
	
	//Node_7
	if (y >= 64 && y <= 120 && x <= 270
		&& y>= -(x - 227) + 64 && y <=1.03*(x-207)+84 )
	{
		//227 64,207 84,242 120 
		*roadid = 29;
		*road_next_node = 7;
		return;
	}
	
	//Node_8
	if(( x>=165 -2 && x<=205 +2 && y>=0 && y<=40)||
		( y>=55 && y<=80 && x>=170 && x<=200))
	{
		//165 40,205 40
		*roadid=30;
		*road_next_node=8;
		return;
	}
	
	
	//roadid=5
	if((x>=20 && x<=35 && y>=165 && y<=235) 
		||(x>=35 && x<=60 && y>=165 && y<=250)
		||(x-35)*(x-35)+(y-235)*(y-235)<=15*15)
	{
		*roadid = 5;
		*road_next_node = 5;
		return;
	}
	
	//roadid=17
	if((x>=235&&x<=250&&y>=35&&y<=67)||
		(x>=205&&x<=235&&y>=20&&y<=67)||
		((x-235)*(x-235)+(y-35)*(y-35)<=15*15))
	{
		//250 67, 235 67,
		*roadid = 17;
		*road_next_node = 8;
		return;
	}
	
	//roadid=8
	if((x>=0 && x<=20 && y>=165 && y<=250)||
		(x>=0 && x<=60 && y>=250 && y <=270)||
		(x>=20 &&x<=35&&y>=235&&y<=250&&(x-35)*(x-35)+(y-235)*(y-235)>=15*15))
	{
		*roadid = 8;
		*road_next_node = 4;
		return;
	}
	
	//roadid=18;
	if((y>=0 && y<=20 && x>=165 && x<=250)||
		(y>=0 && y<=60 && x>=250 && x<=270)||
		(x>=235 && x<=250 && y>=20 &&y<=35)&&(x-235)*(x-235)+(y-35)*(y-35)>=15*15)
	{
		*roadid = 18;
		*road_next_node = 7;
		return;
	}

	
	//roadid=9
	if((x>=0 && x<=20 && y<=165 && y>=118)||
		(x>=19 && x<=22 && y>=118 &&y<=126 &&(x-35)*(x-35)+(y-126)*(y-126)>=15*15))
	{
		*roadid = 9;
		*road_next_node = 11;
		return;
	}

	//roadid=22;
	if((y>=0 && y<=20 && x<=165 && x>=118)||
		(x>=118 && x<=126 && y>=20 && y<=22 && (x-126)*(x-126)+(y-35)*(y-35)>=15*15))
	{
		*roadid = 22;
		*road_next_node = 8;
		return;
	}


	//roadid=4
	if(x>20 && x<=43 && y<=165 && y>=123)
	{
		*roadid = 4;
		*road_next_node = 4;
		return;
	}
		
	//roadid=19;
	if(y>=20 && y<=42 && x<=165 && x>=126&&(x-55)*(x-55)+(y-131)*(y-131)>=15*15)
	{
		*roadid = 19;
		*road_next_node = 9;
		return;
	}
	
	
	//roadid=6
	if(y<=x+123 && y>=x+50 &&x+y>=290 &&x+y<=323)
	//((y>x+27 && y<x+50 && x+y<=324 && x>129 && (x-112)*(x-112)+(y-161)*(y-161)>=15*15)||
	//(y>=x+50 && y<x+123 && x+y<=324 && x+y>=297)||
	//(x<101 && y<=230 && y>=x+123 && x>=60 && y>209))
	{
		*roadid=6;
		*road_next_node=6;
		return;
	}
	
	//roadid=16
	if(y<=x-50&&y>=x-132&&x+y>=290&&x+y<=323)
	//((x>y+27 && x<y+50 && x+y<=324 && y>129 && (y-112)*(y-112)+(x-161)*(x-161)>=15*15)||
	//(x>=y+50 && x<y+123 && x+y<=324 && x+y>=297)||
	//(y<101 && x<=230 && x>=y+123 && y>=60 && x>209))
	{
		*roadid=16;
		*road_next_node=7;
		return;
	}
	
	//roadid=11
	if((y>=x && x+y>=372 && x>=121 && y<=270 )||
		(x>=121 && x<=124 && y>=249 && y<=255))
	{
		*roadid = 11;
		*road_next_node = 6;
		return;
	}
	
	//roadid=14
	if((y<x && x+y>=372&&y>=119&&x<=270)||
		(x>=250 && x<=254 && y>=119 && y<=123))
	{
		*roadid = 14;
		*road_next_node = 7;
		return;
	}

	//roadid=7
	if(y<=x+123&&y>=x+50&&x+y>=323&&x+y<=355)
	{
		*roadid = 7;
		*road_next_node = 5;
		return;
	}
	
	//roadid=15
	if(y<=x-50 && y>=x-132&& x+y>=323 && x+y <=355)
	{
		*roadid = 15;
		*road_next_node = 6;
		return;
	}
	
	//roadid=12
	if(x+y<=274 && x+y>=170 && y>=x && y<=x+32)
	{
		*roadid=12;
		*road_next_node=2;
		return;
	}
	
	//roadid=13
	if(x+y<=274 && x+y>=170 && y<x && y>=x-32)
	{
		*roadid=13;
		*road_next_node=6;
		return;
	}
	
	//circle:roadid=0,1,2
	if((((x-50)*(x-50)+(y-50)*(y-50)<=53.5*53.5)||
		(y>=1.7*(x-32)+100 &&y<=-0.56*(x-110)+49&&x>=100 ) ||
		(x >= 90 && x <= 100 && y >= 15 && y <= 32))&&
		!(y<=x+28&&y>=x&&x+y>170))
		//||( y<=0.17*x+95 && x+y<=170 && x<=0.17*y+82))
	{
		if(x==50)
		{
			if(y>=50) 
			{
				*roadid=1;
				*road_next_node=3;
				return;
			}
			else 
			{
				*roadid=2;
				*road_next_node=1;
				return;
			}
		}
		else
		{
			if(x<50)
			{
				if(y<=-3*(x-50)+50)
				{
					*roadid=2;
					*road_next_node=1;
					return;
				}
				else
				{
					*roadid=1;
					*road_next_node=3;
					return;
				}
			}
			else
			{
				if(y<=-0.35*(x-50)+50)
				{
					*roadid=2;
					*road_next_node=1;
					return;
				}
				else if(y<x)
				{
					*roadid=0;
					*road_next_node=2;
					return;
				}
				else
				{
					*roadid=1;
					*road_next_node=3;
					return;
				}
			}
		}
	}
	
	//roadid=10
	if (y <= -1.8*(x - 22) + 118 && y <= 120 && y >= 0.56*(x - 15) + 90)
		/*((x>=0 && y<=126 && y>=85 && y<=-1.75*x+157 && (x-50)*(x-50)+(y-50)*(y-50)>50*50)||
		(x> 17 && x<=22 && y>-1.75*x+157 && y<=126 &&(x-35)*(x-35)+(y-126)*(y-126)>=15*15))*/
	{
		*roadid = 10;
		*road_next_node = 3;
		return;
	}

	//roadid=21
	if (x <= 120 && y <= -0.56*(x - 118) + 22 && y <= 1.7*(x - 90) + 15)
		//((y>=0 && x<=126 && x>=85 && x<=-1.75*y+157 && (y-50)*(y-50)+(x-50)*(x-50)>50*50)||
		//(y>17 && y<=22 && x>-1.75*y+157 && x<=126 &&(y-35)*(y-35)+(x-126)*(x-126)>=15*15))
	{
		*roadid = 21;
		*road_next_node = 12;
		return;
	}

	//roadid=3
	if (((y <= 124 && y <= -1.75*(x - 53) + 103 && y >= 0.14*(x - 53) + 103 && y >= -1.64*(x - 32) + 100 
		|| (x >= 41 && x <= 43 && y >= 120 && y <= 125))
		&& !(x <= 22 && x >= 19 && y <= 126 && y >= 118 && (x - 35)*(x - 35) + (y - 126)*(y - 126) >= 15 * 15))
		)
		/*((y<=126 && y>-1.75*x+157 && y<=-1.75*x+197 && y>0.17*x+95)&&
		!(x<=22 && x>17 && y<=126 && (x-35)*(x-35)+(y-126)*(y-126)>=15*15))*/
	{
		*roadid = 3;
		*road_next_node = 10;
		return;
	}

	//roadid=20
	if (y <= 7 * (x - 98) + 33 && y <= 55 && x <= 126 && y >= 20 && y >= -0.56*(x - 118) + 22
		&& !(x >= 118 && x <= 126 && y >= 20 && y <= 23 && (x - 125)*(x - 125) + (y - 35)*(y - 35) >= 15 * 15))
		/*((x<=126 && x>-1.75*y+157 && x<=-1.75*y+197 && x>0.17*y+95)&&
		!(y<=22 && y>17 && x<=126 && (y-35)*(y-35)+(x-126)*(x-126)>=15*15))*/
	{
		*roadid = 20;
		*road_next_node = 1;
		return;
	}

	*roadid=-1;
	*road_next_node=-1;
}

/******************************
Description:Based on x,y, return roadid,road_prev_rode
******************************/
void target_area(int x,int y,int *roadid,int *road_prev_node)
{
	
	//Node_4
	if ((y >= 165 - 2 && y <= 205 - 2 && x >= 0 && x <= 40) ||
		(x >= 55 && x <= 80 && y >= 170 && y <= 200))
	{
		*roadid=26;
		*road_prev_node=4;
		return;
	}
	
	//Node_5
	if (x >= 59 - 2 && x <= 122 && y <= 270
		&& y >= -0.71*(x - 57) + 229 && y >= 1.04* (x - 88) + 207)
	{
		*roadid = 27;
		*road_prev_node = 5;
		return;
	}
	
	//Node_6
	if (y >= x - 50 && y <= x + 50 && x + y <= 373 && x + y >= 274)
	{
		*roadid = 28;
		*road_prev_node = 6;
		return;
	}
	
	//Node_7
	if (y >= 64 && y <= 120 && x <= 270
		&& y >= -(x - 227) + 64 && y <= 1.03*(x - 207) + 84)
	{
		*roadid = 29;
		*road_prev_node = 7;
		return;
	}
	
	//Node_8
	if ((x >= 165 - 2 && x <= 205 + 2 && y >= 0 && y <= 40) ||
		(y >= 55 && y <= 80 && x >= 170 && x <= 200))
	{
		*roadid=30;
		*road_prev_node=8;
		return;
	}
	
	//5,7
	//roadid=5
	if ((x >= 20 && x <= 35 && y >= 165 && y <= 235)
		|| (x >= 35 && x <= 60 && y >= 165 && y <= 250)
		|| (x - 35)*(x - 35) + (y - 235)*(y - 235) <= 15 * 15)
	{
		*roadid = 5;
		*road_prev_node = 4;
		return;
	}
	
	//roadid=17
	if ((x >= 235 && x <= 250 && y >= 35 && y <= 67) ||
		(x >= 205 && x <= 235 && y >= 20 && y <= 67) ||
		((x - 235)*(x - 235) + (y - 35)*(y - 35) <= 15 * 15))
	{
		*roadid = 17;
		*road_prev_node = 7;
		return;
	}
	
	//8,18
	//roadid=8
	if ((x >= 0 && x <= 20 && y >= 165 && y <= 250) ||
		(x >= 0 && x <= 60 && y >= 250 && y <= 270) ||
		(x >= 20 && x <= 35 && y >= 235 && y <= 250 && (x - 35)*(x - 35) + (y - 235)*(y - 235) >= 15 * 15))
	{
		*roadid = 8;
		*road_prev_node = 5;
		return;
	}
	
	//roadid=18;
	if ((y >= 0 && y <= 20 && x >= 165 && x <= 250) ||
		(y >= 0 && y <= 60 && x >= 250 && x <= 270) ||
		(x >= 235 && x <= 250 && y >= 20 && y <= 35) && (x - 235)*(x - 235) + (y - 35)*(y - 35) >= 15 * 15)
	{
		*roadid = 18;
		*road_prev_node = 8;
		return;
	}

	
	//roadid=9
	if ((x >= 0 && x <= 20 && y <= 165 && y >= 118) ||
		(x >= 19 && x <= 22 && y >= 118 && y <= 126 && (x - 35)*(x - 35) + (y - 126)*(y - 126) >= 15 * 15))
	{
		*roadid = 9;
		*road_prev_node = 4;
		return;
	}

	//roadid=22;
	if ((y >= 0 && y <= 20 && x <= 165 && x >= 118) ||
		(x >= 118 && x <= 126 && y >= 20 && y <= 22 && (x - 126)*(x - 126) + (y - 35)*(y - 35) >= 15 * 15))
	{
		*roadid = 22;
		*road_prev_node = 12;
		return;
	}
	
	//roadid=4
	if (x > 20 && x <= 43 && y <= 165 && y >= 123)
	{
		*roadid = 4;
		*road_prev_node = 10;
		return;
	}
		
	//roadid=19;
	if (y >= 20 && y <= 42 && x <= 165 && x >= 126 && (x - 55)*(x - 55) + (y - 131)*(y - 131) >= 15 * 15)
	{
		*roadid = 19;
		*road_prev_node = 8;
		return;
	}

	//roadid=6
	if (y <= x + 123 && y >= x + 50 && x + y >= 290 && x + y <= 323)
	{
		*roadid = 6;
		*road_prev_node = 5;
		return;
	}
	
	//roadid=16
	if (y <= x - 50 && y >= x - 132 && x + y >= 290 && x + y <= 323)
	{
		*roadid = 16;
		*road_prev_node = 6;
		return;
	}

	//roadid=11
	if ((y >= x && x + y >= 372 && x >= 121 && y <= 270) ||
		(x >= 121 && x <= 124 && y >= 249 && y <= 255))
	{
		*roadid = 11;
		*road_prev_node = 5;
		return;
	}
	
	//roadid=14
	if ((y < x && x + y >= 372 && y >= 119 && x <= 270) ||
		(x >= 250 && x <= 254 && y >= 119 && y <= 123))
	{
		*roadid = 14;
		*road_prev_node = 6;
		return;
	}

	//roadid=7
	if (y <= x + 123 && y >= x + 50 && x + y >= 323 && x + y <= 355)
	{
		*roadid = 7;
		*road_prev_node = 6;
		return;
	}
	
	//roadid=15
	if (y <= x - 50 && y >= x - 132 && x + y >= 323 && x + y <= 355)
	{
		*roadid = 15;
		*road_prev_node = 7;
		return;
	}
	
	//roadid=12
	if (x + y <= 274 && x + y >= 170 && y >= x && y <= x + 32)
	{
		*roadid= 12 ;
		*road_prev_node= 6;
		return;
	}
	
	//roadid=13
	if (x + y <= 274 && x + y >= 170 && y < x && y >= x - 32)
	{
		*roadid=13;
		*road_prev_node=2;
		return;
	}
	
	//circle:roadid=0,1,2
	if (((x - 50)*(x - 50) + (y - 50)*(y - 50) < 53.5*53.5) ||
		(y >= 1.7*(x - 32) + 100 && y <= -0.56*(x - 110) + 49 && x >= 100)||
		(x>=90&&x<=100&&y>=15&&y<=32) &&
		!(y <= x + 28 && y >= x && x + y > 170)
		)
	{
		if (x == 50)
		{
			if (y >= 50)
			{
				*roadid = 1;
				*road_prev_node = 2;
				return;
			}
			else
			{
				*roadid = 2;
				*road_prev_node = 3;
				return;
			}
		}
		else
		{
			if (x < 50)
			{
				if (y <= -3 * (x - 50) + 50)
				{
					*roadid = 2;
					*road_prev_node = 3;
					return;
				}
				else
				{
					*roadid = 1;
					*road_prev_node = 2;
					return;
				}
			}
			else
			{
				if (y <= -0.35*(x - 50) + 50)
				{
					*roadid = 2;
					*road_prev_node = 3;
					return;
				}
				else if (y < x)
				{
					*roadid = 0;
					*road_prev_node = 1;
					return;
				}
				else
				{
					*roadid = 1;
					*road_prev_node = 2;
					return;
				}
			}
		}
	}

	//roadid=10
	if (y <= -1.8*(x - 22) + 118 && y <= 120 && y >= 0.56*(x - 15) + 90 &&x>=0)
	{
		*roadid=10;
		*road_prev_node=11;
		return;
	}
	
	//roadid=21
	if (x <= 120 && y <= -0.56*(x - 118) + 22 && y <= 1.7*(x - 90) + 15)
	{
		*roadid=21;
		*road_prev_node=1;
		return;
	}
	
	//roadid=3
	if (((y <= 124 && y <= -1.75*(x - 53) + 103 && y >= 0.14*(x - 53) + 103 && y >= -1.64*(x - 32) + 100 
		|| (x >= 41 && x <= 43 && y >= 120 && y <= 125))
		&& !(x <= 22 && x >= 19 && y <= 123 && y >= 118 && (x - 35)*(x - 35) + (y - 126)*(y - 126) >= 15 * 15)))
	{
		*roadid=3;
		*road_prev_node= 3;
		return;
	}
	
	//roadid=20
	if (y <= 7 * (x - 98) + 33 && y <= 55 && x <= 126 && y >= 20 && y >= -0.56*(x - 118) + 22
		&& !(x >= 118 && x <= 126 && y >= 20 && y <= 23 && (x - 125)*(x - 125) + (y - 35)*(y - 35) >= 15 * 15))
	{
		*roadid=20;
		*road_prev_node= 9;
		return;
	}
	
	*roadid = -1;
	*road_prev_node = -1;
}

/******************************
Description:Based on prev_node and next_node, return roadid
******************************/
int node_to_road(int prev,int next)
{
	switch(prev)
	{
	case 1:if(next==2) return 0;
		   else if(next==12) return 21;
		   else if(next==3) return 25;
		   else break;
	case 2:if(next==3) return 1;
		   else if(next==6) return 13;
		   else if(next==1) return 24;
		   else break;
	case 3:if(next==10) return 3;
		   else if(next==1) return 2;
		   else if(next==2) return 23;
		   else break;
	case 4:if(next==5) return 5;
		   else if(next==11) return 9;
		   else break;
	case 5:if(next==6) return 6;
		   else if(next==4) return 8;
		   else break;
	case 6:if(next==5) return 7;
		   else if(next==2) return 12;
		   else if(next==7) return 16;
		   else break;
	case 7:if(next==6) return 15;
		   else if(next==8) return 17;
		   else break;
	case 8:if(next==9) return 19;
		   else if(next==7) return 18;
		   else break;
	case 9:
		   if(next==1) return 20;
		   else break;
	case 10:if(next==4) return 4;
			else break;
	case 11:
			if(next==3) return 10;
			else break;
	case 12:
			if(next==8) return 22;
			else break;
	default: 
			break;
	}
	
	return NO_ROAD;
}

/******************************
Input:x,y,nodeid;
Output:the distance between (x,y) to the node
******************************/
int dist_to_node(int x,int y,int node)
{
	switch(node)
	{	
		case 1: 
			return sqrt((x-78)*(x-78)+(y-40)*(y-40));
		case 2:
			return sqrt((x-71)*(x-71)+(y-71)*(y-71));
		case 3:
			return sqrt((x-40)*(x-40)+(y-78)*(y-78));
		case 4:
			return sqrt((x-20)*(x-20)+(y-185)*(y-185));
		case 5:
			return sqrt((x-90)*(x-90)+(y-250)*(y-250));
		case 6:
			return sqrt((x-162)*(x-162)+(y-162)*(y-162));
		case 7:
			return sqrt((x-250)*(x-250)+(y-90)*(y-90));
		case 8:
			return sqrt((x-185)*(x-185)+(y-20)*(y-20));
		case 9:
			return sqrt((x-126)*(x-126)+(y-30)*(y-30));
		case 10:
			return sqrt((x-30)*(x-30)+(y-126)*(y-126));
		case 11: 
			return sqrt((x-10)*(x-10)+(y-126)*(y-126));
		case 12:
			return sqrt((x-126)*(x-126)+(y-10)*(y-10));
		default:
			break;
	
	}
	
	return 0;
}

void insert_node_start( int road_id, int x, int y)
{
	switch(road_id)
	{
		case 0:
			route[route_num]=42;
			route_num++;
			break;
		case 1:
			route[route_num]=50;
			route_num++;
			break;
		case 2:
			if(y>=x)
			{
				route[route_num]=22;
				route_num++;
			}
			route[route_num]=26;
			route_num++;
			break;
		case 3:
			route[route_num]=15;
			route_num++;
			break;
		case 4:
			if(y<=165)
			{
				route[route_num]=9;
				route_num++;
			}
			/*route[route_num]=4;
			route_num++;*/
			break;
		case 5:
			if(x+y<=270)
			{
				route[route_num]=10;
				route_num++;
				route[route_num]=11;
				route_num++;
			}
			else if(x<=68)
			{
			route[route_num]=11;
			route_num++;
			}
			break;
		case 6:
			if(y>=x+113)
			{
				route[route_num]=13;
				route_num++;
				route[route_num]=14;
				route_num++;
			}
			else if(y>=x+50)
			{
				route[route_num]=14;
				route_num++;
			}
			break;
		case 7:
			if (x + y >= 324 && x + y <= 352 && y <= x + 123 && y >= x)
			{
				route[route_num] = 35;
				route_num++;
			}
			else
			{
				route[route_num]=34;
				route_num++;
				route[route_num]=35;
				route_num++;
			}
			break;
		case 8:
			if(x>=68)
			{
				route[route_num]=35;
				route_num++;
				route[route_num]=54;
				route_num++;
				route[route_num]=52;
				route_num++;
			}
			else if(x+y>=270)
			{
				route[route_num]=54;
				route_num++;
				route[route_num]=52;
				route_num++;
			}
			else if( y>=205)
			{
				route[route_num]=52;
				route_num++;
			}
			break;
		case 9:
			if(y>=165)
			{
				route[route_num]=52;
				route_num++;
			}
			route[route_num]=51;
			route_num++;
			break;
		case 10:
			/*if(y>=0.56*(x-32)+101)
			{
				route[route_num]=50;
				route_num++;
			}*/
			route[route_num]=50;
			route_num++;
			break;
		case 11:
			if(x<=112)
			{
				route[route_num]=36;
				route_num++;
				route[route_num]=37;
				route_num++;
				route[route_num]=38;
				route_num++;
				route[route_num]=39;
				route_num++;
				route[route_num]=34;
				route_num++;
			}
			else if(x<=187&&y>=250)
			{
				route[route_num]=37;
				route_num++;
				route[route_num]=38;
				route_num++;
				route[route_num]=39;
				route_num++;
				route[route_num]=34;
				route_num++;
			}
			else if(y>=0.22*(x-187)+235)
			{
				route[route_num]=38;
				route_num++;
				route[route_num]=39;
				route_num++;
				route[route_num]=34;
				route_num++;
			}
			else if(x+y>=421)
			{
				route[route_num]=39;
				route_num++;
				route[route_num]=34;
				route_num++;
			}
			else if(x+y>=323)
			{
				route[route_num]=34;
				route_num++;
			}
			break;
		case 12:
			if(x+y>=264)
			{
				route[route_num]=14;
				route_num++;
				route[route_num]=42;
				route_num++;
			}
			else if(x+y>=170)
			{
				route[route_num]=42;
				route_num++;
			}
			break;
		case 13:
			if(x+y<=170)
			{
				route[route_num]=43;
				route_num++;
				route[route_num]=16;
				route_num++;
			}
			if(x+y<=264)
			{
				route[route_num]=16;
				route_num++;
			}
			break;
		case 14:
			if(x+y<=373&&x<210)
			{
				route[route_num]=45;
				route_num++;
				route[route_num]=46;
				route_num++;
				route[route_num]=47;
				route_num++;
				route[route_num]=48;
				route_num++;
				route[route_num]=31;
				route_num++;
			}
			else if(x+y<=421&&x<230)
			{
				route[route_num]=46;
				route_num++;
				route[route_num]=47;
				route_num++;
				route[route_num]=48;
				route_num++;
				route[route_num]=31;
				route_num++;
			}
			else if(y>=2.41*(x-235)+184)
			{
				route[route_num]=47;
				route_num++;
				route[route_num]=48;
				route_num++;
				route[route_num]=31;
				route_num++;
			}
			else if(y>=187)
			{
				route[route_num]=48;
				route_num++;
				route[route_num]=31;
				route_num++;
			}
			else if(y>=112)
			{
				route[route_num]=31;
				route_num++;
			}
			break;
		case 15:
			if(y<=x-132)
			{
				route[route_num]=31;
				route_num++;
				route[route_num]=45;
				route_num++;
			}
			else if(y<=x-50)
			{
				route[route_num]=45;
				route_num++;
			}
			break;
		case 16:
			if(y>=x-50)
			{
				route[route_num]=16;
				route_num++;
				route[route_num]=19;
				route_num++;
			}
			else if(y>=x-132)
			{
				route[route_num]=19;
				route_num++;
			}
			break;
		case 17:
			if(y>=68)
			{
				route[route_num]=19;
				route_num++;
				route[route_num]=20;
				route_num++;
				route[route_num]=21;
				route_num++;
			}
			else if(x+y>=270)
			{
				route[route_num]=20;
				route_num++;
				route[route_num]=21;
				route_num++;
			}
			else if(x>=205)
			{
				route[route_num]=21;
				route_num++;
			}
			break;
		case 18:
			if(x<=205)
			{
				route[route_num]=28;
				route_num++;
				route[route_num]=30;
				route_num++;
				route[route_num]=31;
				route_num++;
			}
			else if(x+y<=270)
			{
				route[route_num]=30;
				route_num++;
				route[route_num]=31;
				route_num++;
			}
			else if(y<=68)
			{
				route[route_num]=31;
				route_num++;
			}
			break;
		case 19:
			if(x>=165)
			{
				route[route_num]=21;
				route_num++;
			}
			route[route_num]=24;
			route_num++;
			break;
		case 20:
			//if(y<=1.8*(x-101)+32)
			route[route_num]=25;
			route_num++;
			break;
		case 21:
			if(y>=1.8*(x-101)+32)
			{
				route[route_num]=26;
				route_num++;
			}
			route[route_num]=27;
			route_num++;
			break;
		case 22:
			//if(x<=165)
			route[route_num]=28;
			route_num++;
			break;
		case 23://
			route[route_num]=2;
			route_num++;
			break;
		case 24://
			route[route_num]=1;
			route_num++;
			break;
		case 25://
			if(y<=x)
			{
				route[route_num]=22;
				route_num++;
			}
			route[route_num]=3;
			route_num++;
			break;
		//case 26:
		//	route[route_num]=4;
		//	route_num++;
		//	break;
		//case 27:
		//	route[route_num]=5;
		//	route_num++;
		//	break;
		/*case 28:
			if (x + y >= 324 && x + y <= 352 && y <= x + 123 && y >= x)
			{
				route[route_num] = 35;
				route_num++;
			}
			else
			{
				route[route_num] = 34;
				route_num++;
				route[route_num] = 35;
				route_num++;
			}
			break;*/
		//case 29:
		//	route[route_num]=7;
		//	route_num++;
		//	break;
		//case 30:
		//	route[route_num]=8;
		//	route_num++;
		//	break;
		default:break;
	}
}	

void insert_node_end( int road_id, int x, int y)
{
	switch(road_id)
	{
		case 0:
			break;
		case 1:
			break;
		case 2:
			if(y<=x)
			{
				route[route_num]=22;
				route_num++;
			}
			break;
		case 3:
			if(route[route_num-1]!=42)
			{
				route[route_num]=42;
				route_num++;
			}
			break;
		case 4:
			if(y<165)//
			{}
			else
			{
				route[route_num]=9;
				route_num++;
			}
			break;
		case 5:
			if(route[route_num-1]!=9)
			{
				route[route_num]=9;
				route_num++;
			}
			else
			{
				route_num--;
			}
			if(x>68)
			{
				route[route_num]=10;
				route_num++;
				route[route_num]=11;
				route_num++;
			}
			else if(x+y>270)
			{
				route[route_num]=10;
				route_num++;
			}

			break;
		case 6:
			if(y<x+50)
			{
				if(route[route_num-1]==11)
				{
					route[route_num]=14;
				}
				else 
				{
					route[route_num]=13;
					route_num++;
					route[route_num] = 14;
					route_num++;
				}
			}
			else if(y<x+113)
			{
				if(route[route_num-1]!=11)
				{
					route[route_num]=13;
					route_num++;
				}
			}
			break;
		case 7:
			if(y>x+113)
			{
				if(route[route_num-1]==45)
			{
				route[route_num-1]=35;
			}
			else if(route[route_num-1]==34)
			{
				route[route_num]=35;
				route_num++;
			}
			else
			{
				route[route_num]=34;
				route_num++;
				route[route_num]=35;
				route_num++;
			}
			}
			else if(y>x+50)
			{
				if(route[route_num-1]==45)
				{
					route[route_num-1]=34;
				}
				else if(route[route_num-1]==34)
				{}
				else
				{
					route[route_num]=34;
					route_num++;
				}
			}
			break;
		case 8:
			if(route[route_num-1]!=35)
			{
				route[route_num]=35;
				route_num++;
			}
			
			if(y<205)//
			{
				route[route_num]=54;
				route_num++;
				route[route_num]=52;
				route_num++;
			}
			else if(x+y<270)
			{
				route[route_num]=54;
				route_num++;
			}
			break;
		case 9:
			if(y<165)//
			{
				if(route[route_num-1]!=52)
				{
					route[route_num]=52;
					route_num++;
				}
			}
			break;
		case 10:
			if(y<0.56*(x-32)+101)//
			{
				route[route_num]=50;
				route_num++;
			}
			break;
		case 11:
			if(x<112){}//
			else if(x<187&&y>=250)//
			{
				if(route[route_num-1]!=35)
				{
					route[route_num]=36;
					route_num++;
				}
			}
			else if(y>0.22*(x-187)+235)//
			{
				if(route[route_num-1]!=35)
				{
					route[route_num]=36;
					route_num++;
				}
				route[route_num]=37;
				route_num++;
			}
			else if(x+y>421)//
			{
				if(route[route_num-1]!=35)
				{
					route[route_num]=36;
					route_num++;
				}
				route[route_num]=37;
				route_num++;
				route[route_num]=38;
				route_num++;
			}
			else if(x+y>323)
			{
				if(route[route_num-1]!=35)
				{
					route[route_num]=36;
					route_num++;
				}
				route[route_num]=37;
				route_num++;
				route[route_num]=38;
				route_num++;
				route[route_num]=39;
				route_num++;
			}
			else if(x>150)
			{
				if(route[route_num-1]!=35)
				{
					route[route_num]=36;
					route_num++;
				}
				route[route_num]=37;
				route_num++;
				route[route_num]=38;
				route_num++;
				route[route_num]=39;
				route_num++;
				route[route_num]=40;
				route_num++;
			}
			break;
		case 12:
			if(route[route_num-1]!=14)
			{
				route[route_num]=14;
				route_num++;
			}
			break;
		case 13:
			if (route[route_num - 1] != 25)
			{
				route[route_num] = 25;
				route_num++;
			}
			break;
		case 14:
			if(x+y<373&&x<210)
			{}
			else if(x+y<421&&x<230)//
			{
				if(route[route_num-1]!=16)
				{
					route[route_num]=45;
					route_num++;
				}
			}
			else if(y>2.41*(x-235)+184)
			{
				if(route[route_num-1]!=16)
				{
					route[route_num]=45;
					route_num++; 
					route[route_num] = 46;
					route_num++;
				}
				else
				{
					route[route_num - 1] = 46;
				}
			}
			else if(y>187)
			{
				if(route[route_num-1]!=16)
				{
					route[route_num]=45;
					route_num++;
					route[route_num]=46;
					route_num++;
					route[route_num]=47;
					route_num++;
				}
				else
				{
					route[route_num - 1] = 46;
					route[route_num] = 47;
					route_num++;
				}
			}
			else if(y>112)
			{
				if(route[route_num-1]!=16)
				{
					route[route_num]=45;
					route_num++;
					route[route_num] = 46;
					route_num++;
					route[route_num] = 47;
					route_num++;
					route[route_num] = 48;
					route_num++;
				}
				else
				{
					route[route_num - 1] = 46;
					route[route_num] = 47;
					route_num++;
					route[route_num] = 48;
					route_num++;
				}
			}
			else if(x>240)
			{
				if(route[route_num-1]!=16)
				{
					route[route_num]=45;
					route_num++;
					route[route_num] = 46;
					route_num++;
					route[route_num] = 47;
					route_num++;
					route[route_num] = 48;
					route_num++;
					route[route_num] = 49;
					route_num++;
				}
				else
				{
					route[route_num-1] = 46;
					route[route_num] = 47;
					route_num++;
					route[route_num] = 48;
					route_num++;
					route[route_num] = 49;
					route_num++;
				}
				
			}
			break;
		case 15:
			if(y>x-50)//
			{
				if(route[route_num-1]!=31)
				{
					route[route_num]=31;
					route_num++;
				}
				route[route_num]=45;
				route_num++;
			}
			else if(y>x-132)
			{
				if(route[route_num-1]!=31)
				{
					route[route_num]=31;
					route_num++;
				}
			}
			break;
		case 16:
			if(y>x-50)
			{}
			else if(y>x-132&&x+y<=324)
			{
				if(route[route_num-1] == 16)
				{
				}
				else if (route[route_num - 1] != 14)
				{
					route[route_num] = 16;
					route_num++;
				}
				else
				{
					route[route_num-1]=16;
				}
			}
			else 
			{
				if(route[route_num-1]!=14)
				{
					route[route_num]=16;
					route_num++;
					route[route_num]=19;
					route_num++;
				}
				else
				{
					route[route_num-1]=19;
				}
			}
			break;
		case 17:
			if(route[route_num-1]!=19)
			{
				route[route_num]=19;
				route_num++;
			}
			
			if(x<205)
			{
				route[route_num]=20;
				route_num++;
				route[route_num]=21;
				route_num++;
			}
			else if(x+y<270)
			{
				route[route_num]=20;
				route_num++;
			}
			else if(y<68)
			{}
			break;
		case 18:
			//if(x<205)//
			 if(x+y<270)
			{
				if(route[route_num-1]!=28)
				{
				route[route_num]=28;
				route_num++;
				}
			 }
			else if(y<68)
			{
				if(route[route_num-1]!=28)
				{
				route[route_num]=28;
				route_num++;
				route[route_num]=30;
				route_num++;
				}
				else
				{
					route[route_num-1]=30;
				}
			}
			else 
			{
				if(route[route_num-1]==27)
				{
					route[route_num]=30;
					route_num++;
					route[route_num]=31;
					route_num++;
				}
				else if(route[route_num-1]!=28)
				{
					route[route_num]=28;
					route_num++;
					route[route_num]=30;
					route_num++;
					route[route_num]=31;
					route_num++;
				}
				else if(route[route_num-2]==28)
				{
					route[route_num]=30;
					route_num++;
					route[route_num]=31;
					route_num++;
				}
			}
			break;
		case 19:
			if(x>165)
			{}
			else 
			{
				if(route[route_num-1]!=21)
				{
					route[route_num]=21;
					route_num++;
				}
			}
			break;
		case 20:
			if(y>1.8*(x-101)+32)//
			{
				route[route_num]=25;
				route_num++;
			}
			break;
		case 21:
			if(y<1.8*(x-101)+32)//
			{
				route[route_num]=26;
				route_num++;
			}
			break;
		case 22:
			if(x<165)
			{}
			else 
			{
				route[route_num]=28;
				route_num++;
			}
			break;
		case 23:
			break;
		case 24:
			break;
		case 25://
			if(y<x){}
			else 
			{
				route[route_num]=22;
				route_num++;
			}
			break;
		case 26:
			//route[route_num]=4;
			//route_num++;
			break;
		case 27:
			//route[route_num]=5;
			//route_num++;
			break;
		case 28:
			//route[route_num]=6;
			//route_num++;
			break;
		case 29:
			//route[route_num]=7;
			//route_num++;
			break;
		case 30:
			//route[route_num]=8;
			//route_num++;
			break;
		default:break;
	}
}

void road_io_node( int road_id)
{
	switch(road_id)
	{
		case 0://
			if (route[route_num - 1] != 25)
			{
				route[route_num] = 25;
				route_num++;
			}
			break;
		case 1://
			if (route[route_num - 1] == 25)
			{
				route[route_num - 1] = 42;
			}
			else if(route[route_num -1]!=42)
			{
				route[route_num] = 42;
				route[route_num]++;
			}
			break;
		case 2://
			if(route[route_num-1]!=50)
			{
				route[route_num]=50;
				route_num++;
			}
			route[route_num]=25;
			route_num++;
			break;
		case 3:
			if (route[route_num - 1] == 22)
			{
				route[route_num] = 3;
				route_num++;
				route[route_num] = 42;
				route_num++;
			}
			else if(route[route_num-1]!=42)
			{
				route[route_num]=42;
				route_num++;
			}
			route[route_num]=15;
			route_num++;
			break;
		case 4:
			if(route[route_num-1]!=15)
			{
				route[route_num]=15;
				route_num++;
			}
			route[route_num]=9;
			route_num++;
			break;
		case 5:
			if(route[route_num-1]!=9)
			{
				route[route_num]=9;
				route_num++;
				route[route_num]=10;
				route_num++;
				route[route_num]=11;
				route_num++;
			}
			else 
			{
				route[route_num-1]=10;
				route[route_num]=11;
				route_num++;
			}
			break;
		case 6:
			if(route[route_num-1]!=11)
			{
				route[route_num]=13;
				route_num++;
			}
			route[route_num]=14;
			route_num++;
			break;
		case 7:
			if(route[route_num-1]==45)
			{
				route[route_num-1]=35;
			}
			else if(route[route_num-1]==34)
			{
				route[route_num]=35;
				route_num++;
			}
			else if(route[route_num-1]==35)
			{ }
			else
			{
				route[route_num]=34;
				route_num++;
				route[route_num]=35;
				route_num++;
			}
			break;
		case 8:
			if(route[route_num-1]!=35)
			{
				route[route_num]=35;
				route_num++;
			}
			route[route_num]=54;
			route_num++;
			route[route_num]=52;
			route_num++;
			break;
		case 9:
			if(route[route_num-1]==52)
			{
				route[route_num-1]=51;
			}
			else
			{
				route[route_num]=52;
				route_num++;
				route[route_num]=51;
				route_num++;
			}
			break;
		case 10:
			route[route_num]=50;
			route_num++;
			break;
		case 11:
			if(route[route_num-1]!=35)
			{
				route[route_num]=36;
				route_num++;
			}
			route[route_num]=37;
			route_num++;
			route[route_num]=38;
			route_num++;
			route[route_num]=39;
			route_num++;
			route[route_num]=34;
			route_num++;
			break;
		case 12:
			if(route[route_num-1]==34)
			{
				route[route_num-1]=42;
			}
			else if(route[route_num-1]==14)
			{
				route[route_num]=42;
				route_num++;
			}
			else
			{
				route[route_num]=14;
				route_num++;
				route[route_num]=42;
				route_num++;
			}
			break;
		case 13:
			if (route[route_num - 1] == 22)
			{
				route[route_num] = 26;
				route_num++;
				route[route_num] = 25;
				route_num++;
			}
			else if (route[route_num - 1] != 25)
			{
				route[route_num] = 25;
				route_num++;
			}
			route[route_num]=16;
			route_num++;
			break;
		case 14:
			if(route[route_num-1]==16)
			{
				route[route_num-1]=46;
			}
			else if(route[route_num-1]==45)
			{
				route[route_num]=46;
				route_num++;
			}
			else 
			{
				route[route_num]=45;
				route_num++;
				route[route_num]=46;
				route_num++;
			}
			route[route_num]=47;
			route_num++;
			route[route_num]=48;
			route_num++;
			route[route_num]=31;
			route_num++;
			break;
		case 15:
			if(route[route_num-1]!=31)
			{
				route[route_num]=31;
				route_num++;
			}
			route[route_num]=45;
			route_num++;
			break;
		case 16:
			if(route[route_num-1]==14)
			{
				route[route_num-1]=19;
			}
			else if(route[route_num-1]==16)
			{
				route[route_num]=19;
				route_num++;
			}
			else
			{
				route[route_num]=16;
				route_num++;
				route[route_num]=19;
				route_num++;
			}
			break;
		case 17:
			if(route[route_num-1]!=19)
			{
				route[route_num]=19;
				route_num++;
			}
			route[route_num]=20;
			route_num++;
			route[route_num]=21;
			route_num++;
			break;
		case 18:
			if(route[route_num-1]==28)
			{
				route[route_num-1]=30;
				route[route_num]=31;
				route_num++;
			}
			else
			{
				route[route_num]=28;
				route_num++;
				route[route_num]=30;
				route_num++;
				route[route_num]=31;
				route_num++;
			}
			break;
		case 19:
			if(route[route_num-1]==21)
			{
				route[route_num-1]=24;
			}
			else 
			{
				route[route_num]=21;
				route_num++;
				route[route_num]=24;
				route_num++;
			}
			break;
		case 20:
			route[route_num]=25;
			route_num++;
			break;
		case 21:
			//if (route[route_num - 1] == 25)
			//{
			//	route[route_num - 1] = 26;
			//	route[route_num] = 27;
			//	route_num++;
			//}
			//else
			//{ 
				route[route_num]=26;
				route_num++;
				route[route_num]=27;
				route_num++;
			//}
			break;
		case 22:
			route[route_num]=28;
			route_num++;
			break;
		case 23://
			route[route_num]=42;
			route_num++;
			break;
		case 24:
			if(route[route_num-1]==42)
			{
				break;
			}
			else
			{
				route[route_num] = 42;
				route_num++;
				route[route_num] = 25;
				route_num++;
			}
			break;
		case 25://
			route[route_num]=42;
			route_num++;
			route[route_num]=3;
			route_num++;
			break;
		default:break;
	}
}
