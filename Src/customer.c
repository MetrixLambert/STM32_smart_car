#include "customer.h"
#include "control.h"
#include "state.h"
#include <math.h>

#define SERIOUS_ROUND 1000 //a para to tune ,defined by how fast we can get a good cus 
/*
*	variable to save target info
*/


/*
*	variable to save cus info
*/
short cus_num=2;		//current cus num in game
short old_cus_num=2;
short cus_tag;		//to remark which one is our target (1~5)
short enemy_cus_tag=0;	//if no cus , enemy is 0 
short old_enemy_cus_tag=0;
short cus_x[6];
short cus_y[6];
short des_x[6];
short des_y[6];
short cus_state[6];
short to_cus_len[6]={0};	//remeber to cus road len 
short to_des_len[6]={0};	//remeber to des road len

extern short current_x,current_y;
extern short current_round;
extern enum car_case current_car_case;
/*	note: cus_state
* 	if has but not receive target:  0
* 	if has and receive target: 		1
* 	if target get off: 				2
*/


short get_distance(short end_x,short end_y)
{
	return sqrt((current_x-end_x)*(current_x-end_x)+(current_y-end_y)*(current_y-end_y));
}
/*
*	functions to sort target
*	principle :  max score 
*/
//void sort_target()
//{
//	float a[6]={0};
//	int i,max=0;
//	
//	//use score to choose our customer
//	for(i=1;i<=cus_num;i++)
//	{
//		a[i]=sqrt(pow(cus_x[i]-des_x[i],2)+pow(cus_y[i]-des_y[i],2));
//	  
//		 //give a[i] a score
//    if (a[i]<75 && a[i] >= 0) { a[i] = 20; }
//	  else if (a[i]<175 && a[i] >= 75) { a[i] = 40; }
//	  else if (a[i]<300 && a[i] >= 175) { a[i] = 80; }
//	  else if (a[i]<423 && a[i] >= 300) { a[i] = 100; }
//	}
//	
//	   //find the biggest score customer and change the cus_tag
//	cus_tag=1;
//	for(i=1;i<=cus_num;i++)
//	{
//		if(a[i]>max)
//		{
//			max=a[i];
//			cus_tag=i;
//		}
//		else if(a[i]==max)
//		{
//			if(get_distance(cus_x[i],cus_y[i])<get_distance(cus_x[cus_tag],cus_y[cus_tag]))
//			{
//					cus_tag=i;//tune 
//			}
//		}
//	}			
//}

/*		version 2.0
*	functions to sort target
*	principle :  max get score effiency ; and if time is limited, change tactic to min len to go 
*/
void sort_target()
{
	#ifdef SORT_DEBUG 
		printf("---sort target begin---\r\n");
	#endif 
	
	float a[6]={0};
	float max=0;
	int i=1,min=10000;
	
	if(current_round<=SERIOUS_ROUND)
	{
	//use score to choose our customer
	for(i=1;i<=cus_num;i++)
	{
		a[i]=sqrt(pow(cus_x[i]-des_x[i],2)+pow(cus_y[i]-des_y[i],2));
	  
		 //give a[i] a score/road
    if (a[i]<75 && a[i] >= 0) { a[i] = 20.0/(to_cus_len[i]+to_des_len[i]); }
	  else if (a[i]<175 && a[i] >= 75) { a[i] = 40.0/(to_cus_len[i]+to_des_len[i]); }
	  else if (a[i]<300 && a[i] >= 175) { a[i] = 80.0/(to_cus_len[i]+to_des_len[i]); }
	  else if (a[i]<423 && a[i] >= 300) { a[i] = 100.0/(to_cus_len[i]+to_des_len[i]); }
	}
	
	
	//find the biggest score/road customer and change the cus_tag
	max=0;cus_tag=0;
	#ifdef SORT_DEBUG 
		printf("\r\n avaible cus:\r\n");
	#endif 
	for(i=1;i<=cus_num;i++)
	{
		//if customer in other car: not include this target 
		if((cus_state[i]==2&&current_car_case==A_CAR)||(cus_state[i]==1&&current_car_case==B_CAR))
		{
			#ifdef SORT_DEBUG 
				printf("cus %d not availble \r\n",i);
			#endif 
				continue;
		}
		//to judge
		
		#ifdef SORT_DEBUG 
			printf("--cus:%d (%d,%d)->(%d,%d)\r\n",i,cus_x[i],cus_y[i],des_x[i],des_y[i]);
			printf("--my pos:(%d,%d)\r\n",current_x,current_y);
			printf("--effiency:%f\r\n",a[i]);
			printf("--to cus len:%d\r\n",to_cus_len[i]);
			printf("--to des len:%d\r\n\r\n",to_des_len[i]);
		#endif 
		if(a[i]>max)
		{
			max=a[i];
			cus_tag=i;
		}
		else if(a[i]==max)
		{
			if(to_cus_len[i]<to_cus_len[cus_tag])
			{
					cus_tag=i;//tune 
					max=a[i];
			}
		}
	}	
	
	}
	else
	{
		//find the shortest road to save time
		min=10000;cus_tag=0;
	#ifdef SORT_DEBUG 
		printf("\r\n avaible cus:\r\n");
	#endif 
		
		for(i=1;i<=cus_num;i++)
		{
			if((cus_state[cus_tag]==2&&current_car_case==A_CAR)||(cus_state[cus_tag]==1&&current_car_case==B_CAR))
			{
			#ifdef SORT_DEBUG 
				printf("cus %d not availble \r\n",i);
			#endif 
				continue;
			}
			a[i]=to_cus_len[i]+to_des_len[i];
			
		#ifdef SORT_DEBUG 
			printf("--cus:%d\r\n",i);
			printf("--total road len:%f\r\n",a[i]);
			printf("--to cus len:%d\r\n",to_cus_len[i]);
			printf("--to des len:%d\r\n\r\n",to_des_len[i]);
		#endif 
		
			if(a[i]<min)
			{
				min=a[i];
				cus_tag=i;
			}
			else if(a[i]==min)
			{
				if(to_cus_len[i]<to_cus_len[cus_tag])
				{
					min=a[i];
					cus_tag=i;
				}
			}
		}
		
	}
	  
	#ifdef SORT_DEBUG 
		printf("---sort target end---\r\n");
	#endif 
}

/*		version 2.0
*	functions to sort target
*	principle :  max get score effiency ; and estimate time to judge if possible 
* 							and avoid enemy best target:based on effiency and  
*/


//if is in other car , return 1 ; else return 0 
short is_target_in_other_car(void)
{
	if(cus_state[cus_tag]==0)
	{
		return 0;
	}
	else if((cus_state[cus_tag]==1&&current_car_case==A_CAR)||(cus_state[cus_tag]==2&&current_car_case==B_CAR))
	{
		return 0;
	}
	else if((cus_state[cus_tag]==1&&current_car_case==B_CAR)||(cus_state[cus_tag]==2&&current_car_case==A_CAR))
	{
		return 1;
	}
	else 
	{
		#ifdef STATE_DEBUG 
			printf("!!!cus in wrong state:%d\r\n",cus_state[cus_tag]);
		#endif 
	}
	
	return 0;
}

//used find lucky cus
short find_lucky_cus(void)
{
	#ifdef STATE_DEBUG 
		printf("---scan lucky cus begin---\r\n");
	#endif 
	int i;
	for(i=1;i<=cus_num;i++)
	{
		if((cus_state[i]==1&&current_car_case==A_CAR)||(cus_state[i]==2&&current_car_case==B_CAR))
		{
			#ifdef STATE_DEBUG
				printf("find lucky cus: %d\r\n",i);
			#endif 
			return i;
		}
	}
	
	#ifdef STATE_DEBUG 
		printf("not found lucky cus\r\n");
	#endif 
	return 0;
}

//if enemy cus arrived , return 1; else return 0;
short scan_enemy_cus(void)
{
	int i;
	
	enemy_cus_tag=0;
	for(i=1;i<=cus_num;i++)
	{
			if((cus_state[i]==1&&current_car_case==B_CAR)||(cus_state[i]==2&&current_car_case==A_CAR))
			{
				enemy_cus_tag=i;
			}
	}
	
	if(enemy_cus_tag==0&&old_enemy_cus_tag!=0)
	{
			old_enemy_cus_tag=enemy_cus_tag;
		
		#ifdef STATE_DEBUG 
			printf("!!enemy cus arrived \r\n");
		#endif 
		
		return 1;
	}
	else 
	{
			old_enemy_cus_tag=enemy_cus_tag;
		
		return 0;
	}
	
}
	

void update_enemy_cus_tag(void)
{
	short i;
	enemy_cus_tag=0;
	for(i=1;i<=cus_num;i++)
	{
		if((cus_state[i]==1&&current_car_case==B_CAR)||(cus_state[i]==2&&current_car_case==A_CAR))
		{
				enemy_cus_tag=i;
		}
	}
	
	old_enemy_cus_tag=enemy_cus_tag;
}
