#include "state.h"
#include "customer.h"
/*
*	internal variable
*/
//three different case 
enum car_case current_car_case;
enum game_state current_game_state=NOT_START;
enum state current_state=GAME_WAITING; 

//used in waiting function

short game_mode=0;
short current_round;

//record score of me and enemy 
short my_penalty;
short enemy_penalty;
short my_score;
short enemy_score;

/*
*	extern variable
*/

//from control.c
extern  short current_x;
extern  short current_y;
extern float current_angle;

//from cus.c 
extern short cus_num;
extern short cus_state[6];
extern short cus_tag;
extern short cus_x[6];
extern short cus_y[6];
extern short des_x[6];
extern short des_y[6];
extern short to_cus_len[6];
extern short to_des_len[6];

//from route.c 
extern short route[30];
extern short route_num;
extern short road_len;
extern short current_route;
extern short key_point[55][2];


//define here 
	//car current state 

/*
*	group 0: when waiting or end 
*/
void waiting(void)
{
#ifdef STATE_DEBUG
	printf("\r\n---state: waiting---\r\n");
#endif 
	
	while(1)
	{
		if(current_game_state==NOT_START)
		{
#ifdef	STATE_DEBUG
			printf("game not start\r\n");
#endif 
		}
		else if(current_game_state==PAUSE)
		{
#ifdef	STATE_DEBUG
			printf("game pause\r\n");	
#endif 
		}
		else 
		{
#ifdef	STATE_DEBUG
			printf("game start!!!\r\n\r\n");	
			printf("game start!!!\r\n\r\n");
			printf("game start!!!\r\n\r\n");
#endif 
			break;
		}
		HAL_Delay(MOVE_CHANGE_TIME);
		}	

}

/*
*	group 1: start game 
*/


/*
*	group 2: no cus and find a cus
*/

void find_cus(void)
{
	int i;
#ifdef STATE_DEBUG
	printf("\r\n---state: find cus---\r\n");
#endif 

	for(i=1;i<=cus_num;i++)
	{
		//if cus in other car 
		if((cus_state[i]==2&&current_car_case==A_CAR)||(cus_state[i]==1&&current_car_case==B_CAR))
		{
				continue;
		}
		
		cal_route(current_x,current_y,cus_x[i],cus_y[i]);
		to_cus_len[i]=road_len;
		
		cal_route(cus_x[i],cus_y[i],des_x[i],des_y[i]);
		to_des_len[i]=road_len;
	}
	
	
	//first find the target, and get cus_tag 
	sort_target();
#ifdef STATE_DEBUG
	printf("--target:%d\r\n",cus_tag);
#endif 

	//then cal the route
#ifdef STATE_DEBUG
	printf("--route:\r\n");
#endif 
	cal_route(current_x,current_y,cus_x[cus_tag],cus_y[cus_tag]);

	if(current_state==GAME_WAITING)
	{
		return ;
	}
	
	//final: change state 
	current_state=TO_CUS;
	
#ifdef STATE_DEBUG
	printf("---already find a cus---\r\n");
#endif 
}

/*
*	group 3: to cus
*/	
void to_cus(void)
{
#ifdef STATE_DEBUG
	printf("\r\n---state: to cus---\r\n");
#endif 
	
	for(current_route=1;current_route<route_num;current_route++)
	{
		
		if(current_state==GAME_WAITING)
		{
			return ;
		}
		
		if(current_state==NO_CUS)
		{
			#ifdef STATE_DEBUG 
					printf("\r\n!!! appear new cus or has lost cus\r\n");
			#endif 
			
			return ;
		}
		
		if(find_lucky_cus()!=0)
		{
			cus_tag=find_lucky_cus();
			current_state=TO_DES;
			
			#ifdef STATE_DEBUG
				printf("\r\n lucky customer(%d)!!!\r\n\r\n",cus_tag);
			#endif 
			
			return ;
		}
		
		if(is_target_in_other_car()!=0)
		{
			current_state=NO_CUS;
			
			#ifdef STATE_DEBUG 
				printf("\r\n customer have been obtained  !!!!\r\n");
			#endif
			
			return ;
		}
		
	#ifdef STATE_DEBUG 
		printf("-current route:%d\r\n",current_route);
		printf("--to target:%d,%d\r\n",key_point[route[current_route]-1][0],key_point[route[current_route]-1][1]);
		printf("current_x,current_y,current_angle:%d,%d,%f\r\n",current_x,current_y,current_angle);
	#endif 
		straight_to_target(key_point[route[current_route]-1][0],key_point[route[current_route]-1][1]);
		HAL_Delay(MOVE_CHANGE_TIME);
		
		
	}
	if(current_state==GAME_WAITING)
	{
			return ;
	}
	
	if(current_state==NO_CUS)
	{
		#ifdef STATE_DEBUG 
			printf("\r\n!!lost cus or appear new cus!!\r\n");
		#endif 
			return ;
	}	
		
	if(find_lucky_cus()!=0)
		{
			cus_tag=find_lucky_cus();
			cal_route(current_x,current_y,des_x[cus_tag],des_y[cus_tag]);
			current_state=TO_DES;
			
			#ifdef STATE_DEBUG
				printf("\r\n lucky customer(%d)!!!\r\n\r\n",cus_tag);
			#endif 
			
			return ;
		}
		
		if(is_target_in_other_car()!=0)
		{
			current_state=NO_CUS;
			
			#ifdef STATE_DEBUG 
				printf("\r\n customer have been obtained  !!!!\r\n");
			#endif
			
			return ;
		}
		
	
		
		
	#ifdef STATE_DEBUG
			printf("-current_route: last one\r\n");
		printf("--to target:%d,%d\r\n",cus_x[cus_tag],cus_y[cus_tag]);
	#endif 
	straight_to_target(cus_x[cus_tag],cus_y[cus_tag]);
	HAL_Delay(MOVE_CHANGE_TIME);
	
#ifdef STATE_DEBUG
	printf("---to cus end---\r\n");
#endif 
	current_state=WAIT_CUS;
}


/*
*	group 4: waiting "cus" (host computer to judge)
*/

void wait_cus(void)
{
#ifdef STATE_DEBUG
	printf("\r\n---state: wait cus---\r\n");
#endif 

		if(current_state==GAME_WAITING)
	{
			return ;
	}
	
	HAL_Delay(WAIT_COM_TIME/2);

	while(1)
	{
			if(current_state==GAME_WAITING)
		{
			return ;
		}
		
		if(current_state==NO_CUS)
		{
			#ifdef STATE_DEBUG 
					printf("\r\n!!! appear new cus or has lost cus\r\n");
			#endif 
			
			return ;
		}
		
		if(cus_state[cus_tag]==0)
		{
			tune_pos(cus_x[cus_tag],cus_y[cus_tag]);	
			HAL_Delay(WAIT_COM_TIME);
#ifdef STATE_DEBUG 
			printf("--not receive target\r\n");
#endif		
		}
		else if((cus_state[cus_tag]==1&&current_car_case==A_CAR)||(cus_state[cus_tag]==2&&current_car_case==B_CAR))
		{
#ifdef STATE_DEBUG
			printf("--cus received\r\n");
#endif 
			current_state=TO_DES;
			break;
		}
		else if((cus_state[cus_tag]==1&&current_car_case==B_CAR)||(cus_state[cus_tag]==2&&current_car_case==A_CAR))
		{
#ifdef STATE_DEBUG
			printf("--cus(%d) in other car:(%d)\r\n",cus_tag,cus_state[cus_tag]);
#endif 
			current_state=NO_CUS;
			break;
		}
		else 
		{
#ifdef STATE_DEBUG
			printf("--cus(%d) in wrong state:(%d)\r\n",cus_tag,cus_state[cus_tag]);
#endif 	
		}
			
		
	}
}


/*
*	group 5: to end 
*/

void to_des(void)
{
#ifdef STATE_DEBUG
	printf("\r\n---state: to end---\r\n");
#endif 
	short cus_des_x=des_x[cus_tag];
	short cus_des_y=des_y[cus_tag];
	//first cal the route to destination 
#ifdef STATE_DEBUG
	printf("--route: \r\n");
#endif 	
	cal_route(current_x,current_y,des_x[cus_tag],des_y[cus_tag]);

		if(current_state==GAME_WAITING)
	{
			return ;
	}
	
	
	//then begin the route 
	for(current_route=1;current_route<route_num;current_route++)
	{
			if(current_state==GAME_WAITING)
			{
			return ;
			}
			
			if((cus_state[cus_tag]==1&&current_car_case==A_CAR)||(cus_state[cus_tag]==2&&current_car_case==B_CAR))
			{
			
			}
			else 
			{
					current_state=NO_CUS;
				return; 
			}
		
#ifdef STATE_DEBUG
		printf("--current route:%d\r\n",current_route);
		printf("--to target:%d,%d\r\n",key_point[route[current_route]-1][0],key_point[route[current_route]-1][1]);
#endif 	
		straight_to_target(key_point[route[current_route]-1][0],key_point[route[current_route]-1][1]);
		HAL_Delay(MOVE_CHANGE_TIME);
		
	}

#ifdef STATE_DEBUG
	printf("--current route:last one\r\n");
	printf("--to target:%d,%d\r\n",cus_des_x,cus_des_y);
#endif 
	
		if(current_state==GAME_WAITING)
	{
			return ;
	}
	
	if((cus_state[cus_tag]==1&&current_car_case==A_CAR)||(cus_state[cus_tag]==2&&current_car_case==B_CAR))
			{
			
			}
			else 
			{
					current_state=NO_CUS;
				return; 
			}
	
	straight_to_target(cus_des_x,cus_des_y);
	HAL_Delay(MOVE_CHANGE_TIME);

#ifdef STATE_DEBUG 
	printf("---arrived destination---\r\n");
#endif 
	current_state=WAIT_GET_OFF;
}

/*
*	group 6: wait get off
*/

void wait_get_off(void)
{
#ifdef STATE_DEBUG
	printf("\r\n---state: wait get off---\r\n");
#endif 

		if(current_state==GAME_WAITING)
	{
			return ;
	}
	
	HAL_Delay(WAIT_COM_TIME/2);

	while(1)
	{
			if(current_state==GAME_WAITING)
	{
			return ;
	}
		
		
		if((cus_state[cus_tag]==1&&current_car_case==A_CAR)||(cus_state[cus_tag]==2&&current_car_case==B_CAR))
		{
			tune_pos(des_x[cus_tag],des_y[cus_tag]);
			HAL_Delay(WAIT_COM_TIME);
			
#ifdef STATE_DEBUG
			printf("--cus not get off\r\n");
#endif 
		}
		else if(cus_state[cus_tag]==0)
		{
#ifdef STATE_DEBUG
			printf("--cus get off\r\n");
#endif 
			current_state=NO_CUS;
			break;
		}
		else 
		{
		#ifdef STATE_DEBUG
			printf("--cus in wrong state:%d\r\n",cus_state[cus_tag]);
		#endif 
		}
	}
}

