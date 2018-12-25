//timer 2,3,4,5
//      l1,r1,l2,r2
//timer 1 :ch1,ch4 					:l1f,l1b
//				 ch2,ch3(special) :r2f,r2b
//timer 8 :ch1,ch2 					:l2f,l2b
//				 ch3,ch4					:r1f,r1b

//	//l1 forward 
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,300);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
////	//r2 forward --this is the special one 
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,699);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
////	//l2 forward 
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,300);
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
////	//r1 forward 
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,300);
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);

#include "control.h"
#include "state.h"
#include "customer.h"
#include <math.h>
/*
* 	variables to save the pos and angle of car 
*/


//basic 
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

extern uint8_t uart3Rx[UART3_BUF_MAX];
extern uint16_t uart3RxLength;


//current info 
short current_x,current_y;	//to save current pos 
float current_angle;		//to save current angle
short speed_l1=0,speed_l2=0,speed_r1=0,speed_r2=0;// the speed of tire left 1(front), left 2(rear), right 1,right 2
short current_pwm_l1=0,current_pwm_l2=0,current_pwm_r1=0,current_pwm_r2=0;
extern enum state current_state;
extern enum car_case current_car_case;
extern short cus_state[6]; 

//enemy info
short enemy_x,enemy_y;		//enemy pos 

//game info
extern short current_round ;			//the current round 
extern short cus_tag;
extern short enemy_cus_tag;
extern short old_enemy_cus_tag;

//PID 
struct speed_PID
{
    short Value; 					//Speed Control 
    double Proportion; 	//Proportional Const
    double Integral; 		//Integral Const
    double Derivative;	//Derivative Const 
    short LastError; 			//remeber the last value
    short PrevError;			//set the prev value(even before last) 		
};

struct speed_PID pid_l1,pid_l2,pid_r1,pid_r2;

void USR_PIDInit(void);

void init_speed_l1(short speed);
void init_speed_r1(short speed);
void init_speed_l2(short speed);
void init_speed_r2(short speed);

void set_speed_l1(short speed);
void set_speed_r1(short speed);
void set_speed_l2(short speed);
void set_speed_r2(short speed);

int speed_PID_cal(struct speed_PID* PID, short real_speed);

//highest contorl function
short is_target_forward(short end_x,short end_y);
short is_target_forward2(short end_x,short end_y);
short is_arrived(short current_car_x,short current_car_y,short end_x,short end_y);
short is_arrived_s(short current_car_x,short current_car_y,short end_x,short end_y);
short target_direction(int end_x,int end_y,float* angle);
void straight_to_target(short end_x,short end_y);

/*
*	here are the function used to control how car move
*/


void init_speed_l1(short speed)
{	
		pid_l1.Value =speed;
		current_pwm_l1=speed*SPEED2PWM;
		
		if(current_pwm_l1>0)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,current_pwm_l1);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		}
		else 
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,-current_pwm_l1);

		}
}
void init_speed_l2(short speed)
{
		pid_l2.Value =speed;
		current_pwm_l2=speed*SPEED2PWM;
		
		if(current_pwm_l2>0)
		{
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,current_pwm_l2);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
		}
		else 
		{
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,-current_pwm_l2);
		}
}


void init_speed_r1(short speed)
{
		pid_r1.Value =speed;
		current_pwm_r1=speed*SPEED2PWM;
		
	if(current_pwm_r1>0)
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,current_pwm_r1);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	}
	else 
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,-current_pwm_r1);
	}
}

void init_speed_r2(short speed)
{

		pid_r2.Value =speed;
		current_pwm_r2=speed*SPEED2PWM;
	
		if(current_pwm_r2>0)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000-current_pwm_r2);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
		}
		else 
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000+current_pwm_r2);
		}
}


void set_speed_l1(short speed)
{
	short pwm_der;
	pid_l1.Value =speed;
	
	pwm_der=speed_PID_cal(&pid_l1,speed_l1);
		
	current_pwm_l1+=pwm_der;
	if(current_pwm_l1>PWM_MAX)
	{
			current_pwm_l1=PWM_MAX;
	}
	else if(current_pwm_l1<-PWM_MAX)
	{
		current_pwm_l1=-PWM_MAX;
	}
	else 
	{
		;
	}
		
		#ifdef PID_DEBUG_L1
			 printf("pwm l1:%d,speed: %d\r\n",current_pwm_l1,speed_l1);
		#endif 
		
	//if forward 
	if(current_pwm_l1>0)
	{	 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,current_pwm_l1);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	}
	//else backward 
	else 
	{		 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,-current_pwm_l1);
	}
}

void set_speed_l2(short speed)
{
	short pwm_der;
	pid_l2.Value =speed;
	
	pwm_der=speed_PID_cal(&pid_l2,speed_l2);
	current_pwm_l2+=pwm_der;
	
	if(current_pwm_l2>PWM_MAX)
	{
			current_pwm_l2=PWM_MAX;
	}
	else if(current_pwm_l2<-PWM_MAX)
	{
		current_pwm_l2=-PWM_MAX;
	}
	else 
	{
		;
	}
		
		#ifdef PID_DEBUG_L2
			 printf("pwm l2:%d,speed: %d\r\n",current_pwm_l2,speed_l2);
		#endif 
		
	//if forward 
	if(current_pwm_l2>0)
	{	 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,current_pwm_l2);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	}
	//else backward 
	else 
	{	
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,-current_pwm_l2);
	}
}


void set_speed_r1(short speed)
{
	short pwm_der;
	pid_r1.Value =speed;
	pwm_der=speed_PID_cal(&pid_r1,speed_r1);
	
	current_pwm_r1+=pwm_der;
	if(current_pwm_r1>PWM_MAX)
	{
			current_pwm_r1=PWM_MAX;
	}
	else if(current_pwm_r1<-PWM_MAX)
	{
		current_pwm_r1=-PWM_MAX;
	}
	else 
	{
		;
	}
		
		#ifdef PID_DEBUG_R1
			 printf("pwm r1:%d,speed: %d\r\n",current_pwm_r1,speed_r1);
		#endif 
		
	//if forward 
	if(current_pwm_r1>0)
	{	 
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,current_pwm_r1);	
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	}
	//else backward 
	else 
	{	
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);			
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,-current_pwm_r1);
	}
}

void set_speed_r2(short speed)
{
	short pwm_der;
	pid_r2.Value =speed;
	
	pwm_der=speed_PID_cal(&pid_r2,speed_r2);
	current_pwm_r2+=pwm_der;
	if(current_pwm_r2>PWM_MAX)
	{
			current_pwm_r2=PWM_MAX;
	}
	else if(current_pwm_r2<-PWM_MAX)
	{
		current_pwm_r2=-PWM_MAX;
	}
	else 
	{
		;
	}
		
		#ifdef PID_DEBUG_R2
			 printf("pwm r2:%d,speed: %d\r\n",current_pwm_r2,speed_r2);
		#endif 
		
	//if forward 
	if(current_pwm_r2>0)
	{	 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000-current_pwm_r2);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
	}
	//else backward 
	else 
	{	
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000+current_pwm_r2);
	}
}

void USR_MotorInit(void)
{
	//init left1 right1 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
	//init left2 right2
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	printf("all pwm \r\n");
	
	//init all encoder 
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	__HAL_TIM_SET_COUNTER(&htim5,0);
	printf("all encoder\r\n");
	
						//motor move forward
	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,300);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,699);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
//	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,300);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,300);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	
	HAL_Delay(1000);
	printf("forward\r\n");
	
//	//l1 backward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,300);
//	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,699);
//	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,300);
//	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,300);
	
	printf("backward\r\n");
	HAL_Delay(1000);
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
	
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	
	USR_PIDInit();
}

void USR_PIDInit(void)
{
	pid_l1.Proportion =P_DATA; 	//Proportional Const
	pid_l1.Integral =I_DATA; 		//Integral Const
	pid_l1.Derivative =D_DATA; 	//Derivative Const
	pid_l1.Value =0;						//Speed Control 
	pid_l1.LastError = 0; 			//remeber the last(prev of prev) value
	pid_l1.PrevError = 0; 			//set the prev value 
	
	pid_r1.Proportion =P_DATA; 	//Proportional Const
	pid_r1.Integral =I_DATA; 		//Integral Const
	pid_r1.Derivative =D_DATA; 	//Derivative Const
	pid_r1.Value =0;						//Speed Control 
	pid_r1.LastError = 0; 			//remeber the last(prev of prev) value
	pid_r1.PrevError = 0; 			//set the prev value 
	
	pid_l2.Proportion =P_DATA; 	//Proportional Const
	pid_l2.Integral =I_DATA; 		//Integral Const
	pid_l2.Derivative =D_DATA; 	//Derivative Const
	pid_l2.Value =0;						//Speed Control 
	pid_l2.LastError = 0; 			//remeber the last(prev of prev) value
	pid_l2.PrevError = 0; 			//set the prev value 
	
	pid_r2.Proportion =P_DATA; 	//Proportional Const
	pid_r2.Integral =I_DATA; 		//Integral Const
	pid_r2.Derivative =D_DATA; 	//Derivative Const
	pid_r2.Value =0;						//Speed Control 
	pid_r2.LastError = 0; 			//remeber the last(prev of prev) value
	pid_r2.PrevError = 0; 			//set the prev value 
	
#ifdef PID_DEBUG 
	printf("pid set up\r\n\r\n");
#endif 
}

int speed_PID_cal(struct speed_PID* PID, short real_speed)
{
   int iError=0, iIncpid=0; 
   iError = PID->Value-real_speed;
   iIncpid = PID->Proportion * iError 
             - PID->Integral * PID->LastError
             + PID->Derivative * PID->PrevError;
    PID->PrevError = PID->LastError; 
    PID->LastError = iError;
    return(iIncpid);                        
}

void forward(short speed) 	//forward
{
	short round=0;
	
	init_speed_l1(speed);
	init_speed_r1(speed);
	init_speed_l2(speed);
	init_speed_r2(speed);
	
	while(1)
	{
		if(round>130)		//htis is a para need to tune 
		{
			stop();
			break;
		}
		else 
		{
			round++;
		}
		
		set_speed_l1(speed);	
		set_speed_r1(speed);
		set_speed_l2(speed);
		set_speed_r2(speed);
		
		
		HAL_Delay(10);
	}
} 
//return if not arrived , return 1 if arrive , return 2 if to cus , and find a lucky cus or lost cus 
short forward_to_pos(short speed,short end_x,short end_y)
{
	short round=0;
	short arrive_flag=0;
	short near_result=0;
	
	//update enemy cus tag and old enemy cus tag
	if(current_state==TO_CUS)
	{
		update_enemy_cus_tag();
	}
	
	if(current_state==GAME_WAITING)
	{
			return 0;
	}
	
	
	init_speed_l1(speed);
	init_speed_r1(speed);
	init_speed_l2(speed);
	init_speed_r2(speed);
	
	while(1)
	{
		//if there game pause 
		if(current_state==GAME_WAITING)
		{
			return 0;
		}
		//if arrive 
		if(is_arrived(current_x,current_y,end_x,end_y))
		{
			arrive_flag=1;
			break;
		}
		
		//if target is not forward 
		if(is_target_forward(end_x,end_y)!=1)
		{
			arrive_flag=0;
			break;
		}
		
		if(round>1000)		//htis is a para need to tune 
		{
			break;
		}		
		else 
		{
			round++;
		}
		
		//change to lower speed 
		if(speed==SPEED_1&&(get_distance(end_x,end_y)<DIS_1))
		{
				return forward_to_pos(SPEED_2,end_x,end_y);
		}
		else if(speed==SPEED_2&&(get_distance(end_x,end_y)<DIS_2))
		{
				return forward_to_pos(SPEED_3,end_x,end_y);
		}
		else if(speed==SPEED_3&&(get_distance(end_x,end_y)<DIS_3))
		{
				return forward_to_pos(SPEED_4,end_x,end_y);
		}
		else if(speed==SPEED_4&&(get_distance(end_x,end_y)<DIS_4))
		{
				return forward_to_pos(SPEED_5,end_x,end_y);
		}
		else if(speed==SPEED_5&&(get_distance(end_x,end_y)<DIS_5))
		{
				return forward_to_pos(SPEED_6,end_x,end_y);
		}
		
		//if to cus , and find a lucky cus 
		if(current_state==TO_CUS&&find_lucky_cus()!=0)
		{
			stop();
			current_state=TO_DES;
			cus_tag=find_lucky_cus();
			return 2;
		}
		
		//if cus already in other car 
		if(current_state==TO_CUS&&is_target_in_other_car()!=0)
		{
				stop();
				current_state=NO_CUS;
				return 2;
		}
		//if enemy already arrive and appear a new one 
		if(current_state==TO_CUS&&scan_enemy_cus()==1)
		{
			stop();
			current_state=NO_CUS;
			return 2;
		}
		
		//if enemy is near 
		near_result=is_enemy_near();
		if(near_result==0)
		{
			set_speed_l1(speed);	
			set_speed_r1(speed);
			set_speed_l2(speed);
			set_speed_r2(speed);
		}
		else if(near_result==1)
		{
			waiting_stop();
			forward_a_little(end_x,end_y);
		}
		else if(near_result==2)
		{
			set_speed_l1(speed+SPEED_ACCE);	
			set_speed_r1(speed+SPEED_ACCE);
			set_speed_l2(speed+SPEED_ACCE);
			set_speed_r2(speed+SPEED_ACCE);
		}
		else 
		{
				#ifdef STATE_DEBUG 
					printf("near result wrong:%d!!!!\r\n",near_result);
				#endif 
		}
		
		//contorl pid 

		HAL_Delay(10);
	}
	
	
	//stop
	if(abs(speed_l1)>=SPEED_1-SPEED_CHANGE_THRE)
	{
				//l1 forward 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
		//r2 forward --this is the special one 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
		//l2 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
	//r1 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,999);
	
	HAL_Delay(100);
	
	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);

	}
	else if(abs(speed_l1)>=SPEED_2-SPEED_CHANGE_THRE)
	{
				//l1 forward 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
		//r2 forward --this is the special one 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
		//l2 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
	//r1 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,999);
	
	HAL_Delay(90);
	
	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);

	}
	else if(abs(speed_l1)>=SPEED_3-SPEED_CHANGE_THRE)
	{
				//l1 forward 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
		//r2 forward --this is the special one 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
		//l2 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
	//r1 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,999);
	
	HAL_Delay(70);
	
	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);

	}
	else if(abs(speed_l1)>=SPEED_4-SPEED_CHANGE_THRE)
	{
							//l1 forward 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
		//r2 forward --this is the special one 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
		//l2 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
	//r1 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,999);
	
	HAL_Delay(50);
	
	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	}
	else if(abs(speed_l1)>=SPEED_5-SPEED_CHANGE_THRE)
	{
		//l1 forward 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
		//r2 forward --this is the special one 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
		//l2 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
	//r1 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,999);
	
	HAL_Delay(50);
	
	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	}
	else 
	{	
		//l1 forward 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
		//r2 forward --this is the special one 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
		//l2 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
	//r1 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,999);
	
	HAL_Delay(50);
	
	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0); 
	}
	

	
	
	
	return arrive_flag;
}
//return 0 if is not arrived yet, return 1 if arrive 
short forward_a_little(short end_x,short end_y)
{
	#ifdef STATE_DEBUG 
	printf("move: forward a little\r\n");
	#endif 
	
	#ifndef CONTROL_DEBUG 
	if(current_state==GAME_WAITING)
	{
			return 0;
	}
	#endif 
	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,999);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,999);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	
	HAL_Delay(50);
	
	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	

	return 0;
}

void stop(void) 				//stop
{
//	short round=0;
//	
//	init_speed_l1(0);
//	init_speed_r1(0);
//	init_speed_l2(0);
//	init_speed_r2(0);
//	
//	HAL_Delay(10);
//	
//	while(1)
//	{
//		if(round>1000)		//htis is a para need to tune 
//		{
//			break;
//		}
//		else 
//		{
//			round++;
//		}
//		
//		if(abs(speed_l1)<6 && abs(speed_r1)<6 && abs(speed_l2)<6&& abs(speed_r2)<6)
//			break;
//		
//		
//		set_speed_l1(0);	
//		set_speed_r1(0);
//		set_speed_l2(0);
//		set_speed_r2(0);
//		
//		
//		HAL_Delay(10);
//	}
//	
//	//	//l1 forward 
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
////	//r2 forward --this is the special one 
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
////	//l2 forward 
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
////	//r1 forward 
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
//	
	if(abs(speed_l1)>STOP_LIMIT2)
	{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
//	//r2 forward --this is the special one 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
//	//l2 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
//	//r1 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,999);

		HAL_Delay(50);
	}
	else if(speed_l1>STOP_LIMIT)
	{
	//	//l1 forward 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
//	//r2 forward --this is the special one 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
//	//l2 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,500);
//	//r1 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);

		HAL_Delay(50);
	}
	else if(speed_l1<-STOP_LIMIT)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//	//r2 forward --this is the special one 
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,499);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
//	//l2 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//	//r1 forward 
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,500);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);

		HAL_Delay(50);
	}
		
	//	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
//	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	
}
void waiting_stop(void)
{
	#ifdef CONTROL_DEBUG 
			printf("into waiting stop\r\n");
	#endif 
	
	//l1 forward 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//	//r2 forward --this is the special one 
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
//	//l2 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//	//r1 forward 
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);	
	
	HAL_Delay(50);
	
}

void backward(short speed)	//backward
{
short round=0;
	
	init_speed_l1(-speed);
	init_speed_r1(-speed);
	init_speed_l2(-speed);
	init_speed_r2(-speed);
	
	HAL_Delay(1000);
	
	while(1)
	{
		if(round>1000)
		{
			break;
		}
		else 
		{
			round++;
		}
		
		set_speed_l1(-speed);	
		set_speed_r1(-speed);
		set_speed_l2(-speed);
		set_speed_r2(-speed);
		
		
		HAL_Delay(10);
	}
}
//angle , r is used to judge distance 
void left(short angle,short r)		//turn left
{
		if(angle>180)
		{
			right(360-angle,r);
			
			return; 
		}
	
		float angle_old=current_angle;
		float angle_need=angle;
	
#ifdef TO_TARGET_DEBUG 
		printf("\r\n---start left %d %d---\r\n",angle,r);
#endif 
		
				#ifdef CONTROL_DEBUG
				  printf("left begin %d\r\n",angle);
				#endif
				
				#ifndef CONTROL_DEBUG 
				
				if(current_state==GAME_WAITING)
		{
			return ;
		}
				#endif 
			
		
		if(angle>ANGLE_B)
		{
				init_speed_l1(-ROTAGE_SPEED_B); //-15
				init_speed_l2(-ROTAGE_SPEED_B); //-35
				init_speed_r1(ROTAGE_SPEED_B);	//35
				init_speed_r2(ROTAGE_SPEED_B);	//15
				
				while(1)
				{
					
					#ifndef CONTROL_DEBUG 
					if(current_state==GAME_WAITING)
				{
					return ;
				}
					#endif 
					set_speed_l1(-ROTAGE_SPEED_B);
					set_speed_l2(-ROTAGE_SPEED_B);
					set_speed_r1(ROTAGE_SPEED_B);
					set_speed_r2(ROTAGE_SPEED_B);
					
				#ifdef CONTROL_DEBUG
					printf("current angle:%f,old angle: %f\r\n",current_angle,angle_old);
				#endif 
					
/*				#ifdef TO_TARGET_DEBUG 
//					printf("speed l1,l2,r1,r2:%d,%d,%d,%d\r\n",speed_l1,speed_l2,speed_r1,speed_r2);
//					printf("current pwm l1,2,r1,r2:%d,%d,%d,%d\r\n",current_pwm_l1,current_pwm_l2,current_pwm_r1,current_pwm_r2);
//					printf("still in left\r\n");
//				#endif */
					if(current_angle<angle_old-ANGLE_THRE_SR)
					{
							angle_need=angle-(360-(angle_old-current_angle));
					}
					else 
					{
						 angle_need=angle-(current_angle-angle_old);
					}
				
					
					if(angle_need<ANGLE_B)
					{
						left(angle_need,r);
						return;
					}
					
					
					if(angle>ANGLE_B)
					{	
						if(current_angle<angle_old-ANGLE_THRE_SR)
						{
							if(current_angle+360-angle_old>angle-ANGLE_THRE_B)
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(current_angle-angle_old>angle-ANGLE_THRE_B)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,800);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,800);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
								HAL_Delay(50);
								
								break;
							}
						}
					}
					else if(angle>ANGLE_M)
					{
							if(current_angle<angle_old-ANGLE_THRE_SR)
							{
								if(current_angle+360-angle_old>angle-ANGLE_THRE_M)
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
							}
						else 
							{
							if(current_angle-angle_old>angle-ANGLE_THRE_M)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,800);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,800);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
								HAL_Delay(50);
								
								break;
							}
						}
					}
/*					else if(angle>ANGLE_S)
//					{
//						if(current_angle<angle_old-ANGLE_THRE_SR)
//						{
//							if(current_angle+360-angle_old>angle-ANGLE_THRE_M)
//							{
//								//turn a little to right
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
//		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
//								
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
//	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
//								
//								HAL_Delay(50);
//								
//								break;
//							}
//						}
					else 
					{
							if(current_angle-angle_old>angle-ANGLE_THRE_M)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
						}
					}*/
					else
					{
							if(current_angle<angle_old-ANGLE_THRE_SR)
							{
								if(current_angle+360-angle_old>angle-ANGLE_THRE_S)
								{
									//turn a little to right
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);//l1
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);//l2
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
									HAL_Delay(50);
								
									break;
								}	
							}
							else 
							{
								if(current_angle-angle_old>angle-ANGLE_THRE_S)	
								{
								//turn a little to right
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);//l1
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);//l2
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
									HAL_Delay(50);
								
									break;
								}
							}
					}
					
					
					HAL_Delay(10);
				}
		}
		else if(angle>ANGLE_M)
		{
				init_speed_l1(-ROTAGE_SPEED_M); //-15
				init_speed_l2(-ROTAGE_SPEED_M); //-35
				init_speed_r1(ROTAGE_SPEED_M);	//35
				init_speed_r2(ROTAGE_SPEED_M);	//15
				
				while(1)
				{
					#ifndef CONTROL_DEBUG 
					if(current_state==GAME_WAITING)
				{
					return ;
				}
				#endif 
					
					set_speed_l1(-ROTAGE_SPEED_M);
					set_speed_l2(-ROTAGE_SPEED_M);
					set_speed_r1(ROTAGE_SPEED_M);
					set_speed_r2(ROTAGE_SPEED_M);
					
				#ifdef CONTROL_DEBUG
					printf("current angle:%f,old angle: %f\r\n",current_angle,angle_old);
				#endif 
					
//				#ifdef TO_TARGET_DEBUG 
//					printf("speed l1,l2,r1,r2:%d,%d,%d,%d\r\n",speed_l1,speed_l2,speed_r1,speed_r2);
//					printf("current pwm l1,2,r1,r2:%d,%d,%d,%d\r\n",current_pwm_l1,current_pwm_l2,current_pwm_r1,current_pwm_r2);
//					printf("still in left\r\n");
//				#endif 
					if(current_angle<angle_old-ANGLE_THRE_SR)
					{
							angle_need=angle-(360-(angle_old-current_angle));
					}
					else 
					{
						 angle_need=angle-(current_angle-angle_old);
					}
				
					
					if(angle_need<ANGLE_M)
					{
						left(angle_need,r);
						return;
					}
					
					if(angle>ANGLE_B)
					{	
						if(current_angle<angle_old-ANGLE_THRE_SR)
						{
							if(current_angle+360-angle_old>angle-ANGLE_THRE_B)
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(current_angle-angle_old>angle-ANGLE_THRE_B)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,800);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,800);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
								HAL_Delay(50);
								
								break;
							}
						}
					}
					else if(angle>ANGLE_M)
					{
							if(current_angle<angle_old-ANGLE_THRE_SR)
							{
								if(current_angle+360-angle_old>angle-ANGLE_THRE_M)
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
							}
						else 
							{
							if(current_angle-angle_old>angle-ANGLE_THRE_M)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,800);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,800);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
								HAL_Delay(50);
								
								break;
							}
						}
					}
					/*else if(angle>ANGLE_S)
					{
						if(current_angle<angle_old-ANGLE_THRE_SR)
						{
							if(current_angle+360-angle_old>angle-ANGLE_THRE_M)
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(current_angle-angle_old>angle-ANGLE_THRE_M)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
						}
					}
					*/
					else
					{
							if(current_angle<angle_old-ANGLE_THRE_SR)
							{
								if(current_angle+360-angle_old>angle-ANGLE_THRE_S)
								{
									//turn a little to right
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);//l1
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);//l2
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
									HAL_Delay(50);
								
									break;
								}	
							}
							else 
							{
								if(current_angle-angle_old>angle-ANGLE_THRE_S)	
								{
								//turn a little to right
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);//l1
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);//l2
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
									HAL_Delay(50);
								
									break;
								}
							}
					}
					
					
					HAL_Delay(10);
				}
		}
/*		else if(angle>ANGLE_S)
//		{
//				init_speed_l1(-ROTAGE_SPEED_S); //-15
//				init_speed_l2(-ROTAGE_SPEED_S); //-35
//				init_speed_r1(ROTAGE_SPEED_S);	//35
//				init_speed_r2(ROTAGE_SPEED_S);	//15
//				
//				while(1)
//				{
//					
//					#ifndef CONTROL_DEBUG 
//					if(current_state==GAME_WAITING)
//				{
//					return ;
//				}
//				#endif 
//					
//					set_speed_l1(-ROTAGE_SPEED_S);
//					set_speed_l2(-ROTAGE_SPEED_S);
//					set_speed_r1(ROTAGE_SPEED_S);
//					set_speed_r2(ROTAGE_SPEED_S);
//					
//				#ifdef CONTROL_DEBUG
//					printf("current angle:%f,old angle: %f\r\n",current_angle,angle_old);
//				#endif 
//					
////				#ifdef TO_TARGET_DEBUG 
////					printf("speed l1,l2,r1,r2:%d,%d,%d,%d\r\n",speed_l1,speed_l2,speed_r1,speed_r2);
////					printf("current pwm l1,2,r1,r2:%d,%d,%d,%d\r\n",current_pwm_l1,current_pwm_l2,current_pwm_r1,current_pwm_r2);
////					printf("still in left\r\n");
////				#endif 
//					if(current_angle<angle_old-ANGLE_THRE_SR)
//					{
//							angle_need=angle-(360-(angle_old-current_angle));
//					}
//					else 
//					{
//						 angle_need=angle-(current_angle-angle_old);
//					}
//				
//					
//					if(angle_need<ANGLE_S)
//					{
//						left(angle_need,r);
//						return;
//					}
//					
//					if(angle>ANGLE_B)
//					{	
//						if(current_angle<angle_old-ANGLE_THRE_SR)
//						{
//							if(current_angle+360-angle_old>angle-ANGLE_THRE_B)
//							{
//								//turn a little to right
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
//		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
//								
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
//	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
//								
//								HAL_Delay(50);
//								
//								break;
//							}
//						}
//						else 
//						{
//							if(current_angle-angle_old>angle-ANGLE_THRE_B)	
//							{
//								//turn a little to right
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,800);//l1
//		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,800);//l2
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
//								
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
//	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
//								
//								HAL_Delay(50);
//								
//								break;
//							}
//						}
//					}
//					else if(angle>ANGLE_M)
//					{
//							if(current_angle<angle_old-ANGLE_THRE_SR)
//							{
//								if(current_angle+360-angle_old>angle-ANGLE_THRE_M)
//								{
//								//turn a little to right
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
//		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
//								
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
//	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
//								
//								HAL_Delay(50);
//								
//								break;
//							}
//							}
//						else 
//							{
//							if(current_angle-angle_old>angle-ANGLE_THRE_M)	
//							{
//								//turn a little to right
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,800);//l1
//		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,800);//l2
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
//								
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
//	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
//								
//								HAL_Delay(50);
//								
//								break;
//							}
//						}
//					}
//					else if(angle>ANGLE_S)
//					{
//						if(current_angle<angle_old-ANGLE_THRE_SR)
//						{
//							if(current_angle+360-angle_old>angle-ANGLE_THRE_M)
//							{
//								//turn a little to right
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
//		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
//								
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
//	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
//								
//								HAL_Delay(50);
//								
//								break;
//							}
//						}
//						else 
//						{
//							if(current_angle-angle_old>angle-ANGLE_THRE_M)	
//							{
//								//turn a little to right
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
//		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//								
//								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
//		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
//								
//								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
//	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
//								
//								HAL_Delay(50);
//								
//								break;
//							}
//						}
//					}
//					else
//					{
//							if(current_angle<angle_old-ANGLE_THRE_SR)
//							{
//								if(current_angle+360-angle_old>angle-ANGLE_THRE_S)
//								{
//									//turn a little to right
//									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);//l1
//									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//								
//									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);//l2
//									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//								
//									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
//									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
//								
//									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
//									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
//								
//									HAL_Delay(50);
//								
//									break;
//								}	
//							}
//							else 
//							{
//								if(current_angle-angle_old>angle-ANGLE_THRE_S)	
//								{
//								//turn a little to right
//									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);//l1
//									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//								
//									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);//l2
//									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//								
//									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
//									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
//								
//									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
//									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
//								
//									HAL_Delay(50);
//								
//									break;
//								}
//							}
//					}
//					
//					
//					HAL_Delay(10);
//				}
//		}	*/
		else 
		{
				init_speed_l1(-ROTAGE_SPEED_S); //-15
				init_speed_l2(-ROTAGE_SPEED_S); //-35
				init_speed_r1(ROTAGE_SPEED_S);	//35
				init_speed_r2(ROTAGE_SPEED_S);	//15
				
				while(1)
				{
					
					#ifndef CONTROL_DEBUG 
					if(current_state==GAME_WAITING)
				{
					return ;
				}
				#endif 
					
					set_speed_l1(-ROTAGE_SPEED_S);
					set_speed_l2(-ROTAGE_SPEED_S);
					set_speed_r1(ROTAGE_SPEED_S);
					set_speed_r2(ROTAGE_SPEED_S);
					
				#ifdef CONTROL_DEBUG
					printf("current angle:%f,old angle: %f\r\n",current_angle,angle_old);
				#endif 
					
//				#ifdef TO_TARGET_DEBUG 
//					printf("speed l1,l2,r1,r2:%d,%d,%d,%d\r\n",speed_l1,speed_l2,speed_r1,speed_r2);
//					printf("current pwm l1,2,r1,r2:%d,%d,%d,%d\r\n",current_pwm_l1,current_pwm_l2,current_pwm_r1,current_pwm_r2);
//					printf("still in left\r\n");
//				#endif 
					if(current_angle<angle_old-ANGLE_THRE_SR)
					{
							angle_need=angle-(360-(angle_old-current_angle));
					}
					else 
					{
						 angle_need=angle-(current_angle-angle_old);
					}
					
					if(angle>ANGLE_B)
					{	
						if(current_angle<angle_old-ANGLE_THRE_SR)
						{
							if(current_angle+360-angle_old>angle-ANGLE_THRE_B)
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(current_angle-angle_old>angle-ANGLE_THRE_B)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,800);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,800);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
								HAL_Delay(50);
								
								break;
							}
						}
					}
					else if(angle>ANGLE_M)
					{
							if(current_angle<angle_old-ANGLE_THRE_SR)
							{
								if(current_angle+360-angle_old>angle-ANGLE_THRE_M)
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
							}
						else 
							{
							if(current_angle-angle_old>angle-ANGLE_THRE_M)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,800);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,800);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
								HAL_Delay(50);
								
								break;
							}
						}
					}
					/*else if(angle>ANGLE_S)
					{
						if(current_angle<angle_old-ANGLE_THRE_SR)
						{
							if(current_angle+360-angle_old>angle-ANGLE_THRE_M)
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(current_angle-angle_old>angle-ANGLE_THRE_M)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,900);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,900);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
						}
					}*/
					else
					{
							if(current_angle<angle_old-ANGLE_THRE_SR)
							{
								if(current_angle+360-angle_old>angle-ANGLE_THRE_S)
								{
									//turn a little to right
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);//l1
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);//l2
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
									HAL_Delay(50);
								
									break;
								}	
							}
							else 
							{
								if(current_angle-angle_old>angle-ANGLE_THRE_S)	
								{
								//turn a little to right
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);//l1
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);//l2
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
								
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);//r1
									__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,500);
								
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,999);//r2
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,499);
								
									HAL_Delay(50);
								
									break;
								}
							}
					}
					
					
					HAL_Delay(10);
				}
		}
		

      
		waiting_stop();
		HAL_Delay(MOVE_CHANGE_TIME);
		
		//tune tactic 
		float angle_tune;
				
		if(current_angle<angle_old-ANGLE_THRE_SR)
		{
					angle_tune=current_angle+360-angle_old;
				}
		else 
		{
					angle_tune=current_angle-angle_old;
		}
					
		//tune tactic based on distance 
//		if(r>DIS_B)
//		{
//				if(angle_tune>angle+ANGLE_TUNE_THRE)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start left 2 right tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 			
//					
//					right(angle_tune-angle,r);
//				}
//				else if(angle_tune<angle-ANGLE_TUNE_THRE)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start left 2 left tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//								
//					left(angle-angle_tune,r);
//				}
//		}
//		else if(r>DIS_M)
//		{
//				if(angle_tune>angle+ANGLE_TUNE_THRE2)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start left 2 right tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 			
//					
//					right(angle_tune-angle,r);
//				}
//				else if(angle_tune<angle-ANGLE_TUNE_THRE2)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start left 2 left tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//								
//					left(angle-angle_tune,r);
//				}
//		}
//		else if(r>DIS_S)
//		{
//				if(angle_tune>angle+ANGLE_TUNE_THRE3)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start left 2 right tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 			
//					
//					right(angle_tune-angle,r);
//				}
//				else if(angle_tune<angle-ANGLE_TUNE_THRE3)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start left 2 left tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//								
//					left(angle-angle_tune,r);
//				}
//		}
//		else 
//		{
//				if(angle_tune>angle+ANGLE_TUNE_THRE4)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start left 2 right tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 			
//					
//					right(angle_tune-angle,r);
//				}
//				else if(angle_tune<angle-ANGLE_TUNE_THRE4)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start left 2 left tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//								
//					left(angle-angle_tune,r);
//				}
//		}
//		
}

void right(short angle,short r)		//turn right
{
#ifdef TO_TARGET_DEBUG 
		printf("\r\n---start right %d %d---\r\n",angle,r);
#endif 
	
		if(angle>180)
		{
			left(360-angle,r);
			return ;
		}
	
		float angle_old=current_angle;
		float angle_need=angle;
	
				#ifndef CONTROL_DEBUG 
				if(current_state==GAME_WAITING)
			{
			return ;
			}
			#endif 
				
				
				#ifdef CONTROL_DEBUG
				  printf("right begin %d\r\n",angle);
				#endif
				
				if(angle>ANGLE_B)
		{
				init_speed_l1(ROTAGE_SPEED_B); //15
				init_speed_l2(ROTAGE_SPEED_B); //35
				init_speed_r1(-ROTAGE_SPEED_B);	//-35
				init_speed_r2(-ROTAGE_SPEED_B);	//-15
				
				while(1)
				{
					#ifndef CONTROL_DEBUG 
					if(current_state==GAME_WAITING)
				{
					return ;
				}
				#endif 
					
					set_speed_l1(ROTAGE_SPEED_B);
					set_speed_l2(ROTAGE_SPEED_B);
					set_speed_r1(-ROTAGE_SPEED_B);
					set_speed_r2(-ROTAGE_SPEED_B);
					
				#ifdef CONTROL_DEBUG
					printf("current angle:%f,old angle: %f\r\n",current_angle,angle_old);
				#endif 
					
//				#ifdef TO_TARGET_DEBUG 
//					printf("speed l1,l2,r1,r2:%d,%d,%d,%d\r\n",speed_l1,speed_l2,speed_r1,speed_r2);
//					printf("current pwm l1,2,r1,r2:%d,%d,%d,%d\r\n",current_pwm_l1,current_pwm_l2,current_pwm_r1,current_pwm_r2);
//					printf("still in left\r\n");
//				#endif 
					if(current_angle>angle_old+ANGLE_THRE_SR)
					{
							angle_need=angle-(360+angle_old-current_angle);
					}
					else 
					{
						 angle_need=angle-(angle_old-current_angle);
					}
					
					
					
					if(angle_need<ANGLE_B)
					{
						right(angle_need,r);
						return;
					}
					
					
					
					if(angle>ANGLE_B)
					{	
						if(current_angle>angle_old+ANGLE_THRE_SR)
						{
							if(360-current_angle+angle_old>angle-ANGLE_THRE_BR)
							{
								//turn a little to left
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

								
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(angle_old-current_angle>angle-ANGLE_THRE_BR)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
						
								HAL_Delay(50);
								
								break;
							}
					}
					}
					else if(angle>ANGLE_M)
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
							{
								if(360-current_angle+angle_old>angle-ANGLE_THRE_MR)
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
					}
							else 
							{
								if(angle_old-current_angle>angle-ANGLE_THRE_MR)	
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,999);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
								HAL_Delay(70);
								
								break;
								}
							}
					}
					/*else if(angle>ANGLE_S)
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
						{
							if(360-current_angle+angle_old>angle-ANGLE_THRE_SR)
							{
								//turn a little to left
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(angle_old-current_angle>angle-ANGLE_THRE_SR)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

								HAL_Delay(50);
								
								break;
							}
						}
					}
					*/
					else
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
							{
								if(360-current_angle+angle_old>angle-ANGLE_THRE_SR)
								{
									//turn a little to right
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,500);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,499);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

									HAL_Delay(50);
								
									break;
								}	
							}
							else 
							{
								if(angle_old-current_angle>angle-ANGLE_THRE_SR)	
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,500);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,499);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

									HAL_Delay(50);
								
									break;
								}
							}
					}
					
					HAL_Delay(10);
				}
			}
		else if(angle>ANGLE_M)
		{
				init_speed_l1(ROTAGE_SPEED_M); //-15
				init_speed_l2(ROTAGE_SPEED_M); //-35
				init_speed_r1(-ROTAGE_SPEED_M);	//35
				init_speed_r2(-ROTAGE_SPEED_M);	//15
				
				while(1)
				{
					#ifndef CONTROL_DEBUG 
					if(current_state==GAME_WAITING)
				{
					return ;
				}
				#endif
					
					set_speed_l1(ROTAGE_SPEED_M);
					set_speed_l2(ROTAGE_SPEED_M);
					set_speed_r1(-ROTAGE_SPEED_M);
					set_speed_r2(-ROTAGE_SPEED_M);
					
				#ifdef CONTROL_DEBUG
					printf("current angle:%f,old angle: %f\r\n",current_angle,angle_old);
				#endif 
					
//				#ifdef TO_TARGET_DEBUG 
//					printf("speed l1,l2,r1,r2:%d,%d,%d,%d\r\n",speed_l1,speed_l2,speed_r1,speed_r2);
//					printf("current pwm l1,2,r1,r2:%d,%d,%d,%d\r\n",current_pwm_l1,current_pwm_l2,current_pwm_r1,current_pwm_r2);
//					printf("still in left\r\n");
//				#endif 
					if(current_angle>angle_old+ANGLE_THRE_SR)
					{
							angle_need=angle-(360+angle_old-current_angle);
					}
					else 
					{
						 angle_need=angle-(angle_old-current_angle);
					}
					
					
					
					if(angle_need<ANGLE_M)
					{
						right(angle_need,r);
						return;
					}
					
							
					if(angle>ANGLE_B)
					{	
						if(current_angle>angle_old+ANGLE_THRE_SR)
						{
							if(360-current_angle+angle_old>angle-ANGLE_THRE_BR)
							{
								//turn a little to left
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

								
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(angle_old-current_angle>angle-ANGLE_THRE_BR)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
						
								HAL_Delay(50);
								
								break;
							}
					}
					}
					else if(angle>ANGLE_M)
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
							{
								if(360-current_angle+angle_old>angle-ANGLE_THRE_MR)
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
					}
							else 
							{
								if(angle_old-current_angle>angle-ANGLE_THRE_MR)	
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,999);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
								HAL_Delay(70);
								
								break;
								}
							}
					}
					/*
					else if(angle>ANGLE_S)
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
						{
							if(360-current_angle+angle_old>angle-ANGLE_THRE_SR)
							{
								//turn a little to left
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(angle_old-current_angle>angle-ANGLE_THRE_SR)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

								HAL_Delay(50);
								
								break;
							}
						}
					}
					*/
					else
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
							{
								if(360-current_angle+angle_old>angle-ANGLE_THRE_SR)
								{
									//turn a little to right
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,500);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,499);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

									HAL_Delay(50);
								
									break;
								}	
							}
							else 
							{
								if(angle_old-current_angle>angle-ANGLE_THRE_SR)	
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,500);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,499);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

									HAL_Delay(50);
								
									break;
								}
							}
					}
					
					HAL_Delay(10);
				}
		}
		/*else if(angle>ANGLE_S)
		{
				init_speed_l1(ROTAGE_SPEED_S); //-15
				init_speed_l2(ROTAGE_SPEED_S); //-35
				init_speed_r1(-ROTAGE_SPEED_S);	//35
				init_speed_r2(-ROTAGE_SPEED_S);	//15
				
				while(1)
				{
					#ifndef CONTROL_DEBUG 
					if(current_state==GAME_WAITING)
				{
					return ;
				}
				#endif 
					
					set_speed_l1(ROTAGE_SPEED_S);
					set_speed_l2(ROTAGE_SPEED_S);
					set_speed_r1(-ROTAGE_SPEED_S);
					set_speed_r2(-ROTAGE_SPEED_S);
					
				#ifdef CONTROL_DEBUG
					printf("current angle:%f,old angle: %f\r\n",current_angle,angle_old);
				#endif 
					
//				#ifdef TO_TARGET_DEBUG 
//					printf("speed l1,l2,r1,r2:%d,%d,%d,%d\r\n",speed_l1,speed_l2,speed_r1,speed_r2);
//					printf("current pwm l1,2,r1,r2:%d,%d,%d,%d\r\n",current_pwm_l1,current_pwm_l2,current_pwm_r1,current_pwm_r2);
//					printf("still in left\r\n");
//				#endif 
					if(current_angle>angle_old+ANGLE_THRE_SR)
					{
							angle_need=angle-(360+angle_old-current_angle);
					}
					else 
					{
						 angle_need=angle-(angle_old-current_angle);
					}
					
					
					
					if(angle_need<ANGLE_S)
					{
						right(angle_need,r);
						return;
					}
					
							
					if(angle>ANGLE_B)
					{	
						if(current_angle>angle_old+ANGLE_THRE_SR)
						{
							if(360-current_angle+angle_old>angle-ANGLE_THRE_BR)
							{
								//turn a little to left
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

								
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(angle_old-current_angle>angle-ANGLE_THRE_BR)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
						
								HAL_Delay(50);
								
								break;
							}
					}
					}
					else if(angle>ANGLE_M)
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
							{
								if(360-current_angle+angle_old>angle-ANGLE_THRE_MR)
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
					}
							else 
							{
								if(angle_old-current_angle>angle-ANGLE_THRE_MR)	
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,999);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
								HAL_Delay(70);
								
								break;
								}
							}
					}
					else if(angle>ANGLE_S)
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
						{
							if(360-current_angle+angle_old>angle-ANGLE_THRE_SR)
							{
								//turn a little to left
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(angle_old-current_angle>angle-ANGLE_THRE_SR)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

								HAL_Delay(50);
								
								break;
							}
						}
					}
					else
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
							{
								if(360-current_angle+angle_old>angle-ANGLE_THRE_SR)
								{
									//turn a little to right
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,500);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,499);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

									HAL_Delay(50);
								
									break;
								}	
							}
							else 
							{
								if(angle_old-current_angle>angle-ANGLE_THRE_SR)	
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,500);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,499);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

									HAL_Delay(50);
								
									break;
								}
							}
					}
					
					HAL_Delay(10);
				}
		}	
		*/
		else 
		{
				init_speed_l1(ROTAGE_SPEED_S); //-15
				init_speed_l2(ROTAGE_SPEED_S); //-35
				init_speed_r1(-ROTAGE_SPEED_S);	//35
				init_speed_r2(-ROTAGE_SPEED_S);	//15
				
				while(1)
				{
					#ifndef CONTROL_DEBUG 
					if(current_state==GAME_WAITING)
				{
					return ;
				}
				#endif 
					
					set_speed_l1(ROTAGE_SPEED_S);
					set_speed_l2(ROTAGE_SPEED_S);
					set_speed_r1(-ROTAGE_SPEED_S);
					set_speed_r2(-ROTAGE_SPEED_S);
					
				#ifdef CONTROL_DEBUG
					printf("current angle:%f,old angle: %f\r\n",current_angle,angle_old);
				#endif 
					
//				#ifdef TO_TARGET_DEBUG 
//					printf("speed l1,l2,r1,r2:%d,%d,%d,%d\r\n",speed_l1,speed_l2,speed_r1,speed_r2);
//					printf("current pwm l1,2,r1,r2:%d,%d,%d,%d\r\n",current_pwm_l1,current_pwm_l2,current_pwm_r1,current_pwm_r2);
//					printf("still in left\r\n");
//				#endif 
					if(current_angle>angle_old+ANGLE_THRE_SR)
					{
							angle_need=angle-(360+angle_old-current_angle);
					}
					else 
					{
						 angle_need=angle-(angle_old-current_angle);
					}
					
					
					
							
					if(angle>ANGLE_B)
					{	
						if(current_angle>angle_old+ANGLE_THRE_SR)
						{
							if(360-current_angle+angle_old>angle-ANGLE_THRE_BR)
							{
								//turn a little to left
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

								
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(angle_old-current_angle>angle-ANGLE_THRE_BR)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
						
								HAL_Delay(50);
								
								break;
							}
					}
					}
					else if(angle>ANGLE_M)
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
							{
								if(360-current_angle+angle_old>angle-ANGLE_THRE_MR)
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,99);
								
								HAL_Delay(50);
								
								break;
							}
					}
							else 
							{
								if(angle_old-current_angle>angle-ANGLE_THRE_MR)	
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,999);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,999);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,999);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
								HAL_Delay(70);
								
								break;
								}
							}
					}
					/*else if(angle>ANGLE_S)
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
						{
							if(360-current_angle+angle_old>angle-ANGLE_THRE_SR)
							{
								//turn a little to left
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
								
								HAL_Delay(50);
								
								break;
							}
						}
						else 
						{
							if(angle_old-current_angle>angle-ANGLE_THRE_SR)	
							{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,900);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,900);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,99);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

								HAL_Delay(50);
								
								break;
							}
						}
					}
					*/
					else
					{
							if(current_angle>angle_old+ANGLE_THRE_SR)
							{
								if(360-current_angle+angle_old>angle-ANGLE_THRE_SR)
								{
									//turn a little to right
									__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,500);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,499);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

									HAL_Delay(50);
								
									break;
								}	
							}
							else 
							{
								if(angle_old-current_angle>angle-ANGLE_THRE_SR)	
								{
								//turn a little to right
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//l1
		            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//l2
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,500);
								
								__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,500);//r1
		            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
								
								__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,499);//r2
	            	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);

									HAL_Delay(50);
								
									break;
								}
							}
					}
					
					HAL_Delay(10);
				}
		}		
			

		

		waiting_stop();
		HAL_Delay(MOVE_CHANGE_TIME);
		
//		float angle_tune;	//used to cal angle already rotate
				
//				if(current_angle>angle_old)
//				{
//					angle_tune=360-current_angle+angle_old;
//				}
//				else 
//				{
//					angle_tune=angle_old-current_angle;
//				}
//				
//				
//				
//		if(r>DIS_B)
//		{
//				if(angle_tune>angle+ANGLE_TUNE_THRE)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start right 2 left tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//					
//					left(angle_tune-angle,r);
//				}
//				else if(angle_tune<angle-ANGLE_TUNE_THRE)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start right 2 right tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//					
//					right(angle-angle_tune,r);
//				}
//		}
//		else if(r>DIS_M)
//		{
//				if(angle_tune>angle+ANGLE_TUNE_THRE2)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start right 2 left tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//					
//					left(angle_tune-angle,r);
//				}
//				else if(angle_tune<angle-ANGLE_TUNE_THRE2)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start right 2 right tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//					
//					right(angle-angle_tune,r);
//				}
//		}
//		else if(r>DIS_S)
//		{
//				if(angle_tune>angle+ANGLE_TUNE_THRE3)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start right 2 left tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//					
//					left(angle_tune-angle,r);
//				}
//				else if(angle_tune<angle-ANGLE_TUNE_THRE3)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start right 2 right tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//					
//					right(angle-angle_tune,r);
//				}
//		}
//		else
//		{
//				if(angle_tune>angle+ANGLE_TUNE_THRE4)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start right 2 left tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//					
//					left(angle_tune-angle,r);
//				}
//				else if(angle_tune<angle-ANGLE_TUNE_THRE4)
//				{
//					#ifdef TO_TARGET_DEBUG 
//						printf("-----start right 2 right tune ------\r\n");
//						printf("--current angle:%f\r\n",current_angle);
//						printf("--angle old:%f\r\n",angle_old);
//						printf("--angle tune:%f\r\n",angle_tune);
//					#endif 	
//					
//					right(angle-angle_tune,r);
//				}
//		}
}



void turn_around(void )			//turn around(left)				
{
	left(180,60);
}



void upgrade_angle(void )		//upgradge the angle , not used 
{
	if(uart3RxLength!=0)
		{
			if(uart3Rx[22]==0x55)
			{
				if(uart3Rx[23]==0x53)
				{
				  current_angle=((float)(uart3Rx[29]*256)+uart3Rx[28])/32768*180;
#ifdef CONTROL_DEBUG
					printf("current_angle: %f\r\n\r\n",current_angle);
#endif					
				}
				else 
				{
					printf("the wrong bag\r\n\r\n");
				}
			}
			else 
			{
				printf("no bag\r\n\r\n");
			}
			uart3RxLength=0;
		}
}
void update_speed(void)
{
	//1ms a time to upgrade speed, and two pulse one round 
	//L1 SPEED 
	speed_l1=__HAL_TIM_GET_COUNTER(&htim2);	
	speed_l1*=2;
	__HAL_TIM_SET_COUNTER(&htim2,0);

#ifdef SPEED_DEBUG 
	printf("speed_l1:%d\r\n",speed_l1);
#endif 
	
	//R1 SPEED 
	speed_r1=__HAL_TIM_GET_COUNTER(&htim3);	
	speed_r1*=2;
	__HAL_TIM_SET_COUNTER(&htim3,0);
	
#ifdef SPEED_DEBUG 
	printf("speed_r1:%d\r\n",speed_r1);
#endif 
	
	//L2 SPEED 
	speed_l2=__HAL_TIM_GET_COUNTER(&htim4);	
	speed_l2*=2;
	__HAL_TIM_SET_COUNTER(&htim4,0);
	
#ifdef SPEED_DEBUG 
	printf("speed_l2:%d\r\n",speed_l2);
#endif 	
	
	//R2 SPEED 
	speed_r2=__HAL_TIM_GET_COUNTER(&htim5);	
	speed_r2*=2;
	__HAL_TIM_SET_COUNTER(&htim5,0);
	
#ifdef SPEED_DEBUG 
	printf("speed_r2:%d\r\n",speed_r2);
#endif 
}


void tune(void)			//used in tune button
{
		straight_to_target(140,127);
	
//	//l1 forward 
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
//	//r2 forward --this is the special one 
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,499);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,999);
//	//l2 forward 
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//	//r1 forward 
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,500);
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
}


//still keep this function, to prepare for someday 
short is_target_forward(short end_x,short end_y)
{
	float target_angle;
	short distance=get_distance(end_x,end_y);
	target_angle=atan2(end_y-current_y,end_x-current_x)*180/PI;
	
	if(target_angle<0)
			target_angle+=360;
	
	#ifdef TO_TARGET_DEBUG
		printf("target_angle:%f",target_angle);
		printf(";current_angle:%f\r\n",current_angle);
	#endif 
	
	if(distance>DIS_B)
	{
			if(fabs(target_angle-current_angle)<ANGLE_THRE)
		{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward1\r\n");
		#endif 
		return 1;
	}
	else 
	{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE)
			{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 1;
			}
			else 
			{
				return 0;
			}
	}
	}
	else if(distance>DIS_M)
	{
			if(fabs(target_angle-current_angle)<ANGLE_THRE2)
		{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward1\r\n");
		#endif 
		return 1;
		}
	else 
	{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE2)
			{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 1;
			}
			else 
			{
				return 0;
			}
	}
	
	}
	else if(distance>DIS_S)
	{
		if(fabs(target_angle-current_angle)<ANGLE_THRE3)
		{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward1\r\n");
		#endif 
		return 1;
		}
		else 
		{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE3)
			{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 1;
			}
			else 
			{
				return 0;
			}
	}
	}
	
	
	
	
	else 
	{
			if(fabs(target_angle-current_angle)<ANGLE_THRE4)
		{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward1\r\n");
		#endif 
		return 1;
		}
		else 
		{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE4)
			{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 1;
			}
			else 
			{
				return 0;
			}
	}
	}

}

//used to judge if target is near 
short is_target_forward2(short end_x,short end_y)
{
	float target_angle;
	short distance=get_distance(end_x,end_y);
	target_angle=atan2(end_y-current_y,end_x-current_x)*180/PI;
	
	if(target_angle<0)
			target_angle+=360;
	
	#ifdef TO_TARGET_DEBUG
		printf("target_angle:%f",target_angle);
		printf(";current_angle:%f\r\n",current_angle);
	#endif 
	
	if(distance>DIS_B)
	{
			if(fabs(target_angle-current_angle)<ANGLE_THRE)
		{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward1\r\n");
		#endif 
		return 1;
	}
	else 
	{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE)
			{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 1;
			}
			else 
			{
				return 0;
			}
	}
	}
	else if(distance>DIS_M)
	{
			if(fabs(target_angle-current_angle)<ANGLE_THRE2)
		{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward1\r\n");
		#endif 
		return 1;
		}
	else 
	{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE2)
			{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 1;
			}
			else 
			{
				return 0;
			}
	}
	
	}
	else if(distance>DIS_S)
	{
		if(fabs(target_angle-current_angle)<ANGLE_THRE3)
		{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward1\r\n");
		#endif 
		return 1;
		}
		else 
		{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE3)
			{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 1;
			}
			else 
			{
				return 0;
			}
	}
	}
	
	
	
	
	else 
	{
			if(fabs(target_angle-current_angle)<ANGLE_THRE4)
		{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward1\r\n");
		#endif 
		return 1;
		}
		else 
		{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE4)
			{
		#ifdef TO_TARGET_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 1;
			}
			else 
			{
				return 0;
			}
	}
	}
}
short is_arrived(short current_x,short current_y,short end_x,short end_y)
{
	if(end_x>current_x-ARRIVE_THRE&&end_x<current_x+ARRIVE_THRE)
	{
			if(end_y>current_y-ARRIVE_THRE&&end_y<current_y+ARRIVE_THRE)
			{
#ifdef TO_TARGET_DEBUG
				printf("near the target\r\n");
#endif
					return 1;
			}

			else 
			{
				return 0;
			}
				
	}
		return 0;
}

short is_arrived_s(short current_x,short current_y,short end_x,short end_y)
{
	if(end_x>current_x-ARRIVE_THRE_S&&end_x<current_x+ARRIVE_THRE_S)
	{
			if(end_y>current_y-ARRIVE_THRE_S&&end_y<current_y+ARRIVE_THRE_S)
			{
#ifdef TO_TARGET_DEBUG
				printf("near the target\r\n");
#endif
					return 1;
			}

			else 
			{
				return 0;
			}
				
	}
		return 0;
}

//core function
void straight_to_target(short end_x,short end_y)
{
	float angle;
	short direction;
	float distance;

#ifdef TO_TARGET_DEBUG
		printf("\r\n-----straight to target (%d,%d) start:-------\r\n",end_x,end_y);
#endif 
	
	if(current_state==GAME_WAITING)
	{
			return ;
	}
	
	while(!is_arrived(current_x,current_y,end_x,end_y))
	{
		//get target info
		direction=target_direction(end_x,end_y,&angle);
		distance=sqrt((current_x-end_x)*(current_x-end_x)+(current_y-end_y)*(current_y-end_y));
		
		#ifdef TO_TARGET_DEBUG
			printf("--direction:%d,angle:%f\r\n",direction,angle);
			printf("--distance:%f\r\n",distance);
			printf("--currnet angle:%f\r\n",current_angle);
			printf("--current (x,y): %d,%d\r\n",current_x,current_y);
		#endif 
		
		
		if(current_state==GAME_WAITING)
		{
			return ;
		}
		
		#ifdef TO_TARGET_DEBUG 
			printf("\r\n move:\r\n");
		#endif 
		//judget how to move 
		if(distance>DIS_1)
		{	
			if(direction==0)
			{
			#ifdef TO_TARGET_DEBUG 
				printf("----forward----\r\n");
			#endif 
				short result=forward_to_pos(SPEED_1,end_x,end_y);
				if(result==1)
				{
			#ifdef TO_TARGET_DEBUG 
				printf("---already forward to target---\r\n");
			#endif 
					break;
				}
				else if(result==2)
				{
						return ;
				}
					
			}
			else if(direction==-1)
			{
			#ifdef TO_TARGET_DEBUG 
				printf("----left %f----",angle);
			#endif 
				left((short)angle,(short)distance);
		}
			else if(direction==1)
			{
			#ifdef TO_TARGET_DEBUG 
				printf("---- right %f ----",angle);
			#endif 
				right((short)angle,distance);
		}
			else 
			{
			#ifdef TO_TARGET_DEBUG
				printf("wrong in direction\r\n");
			#endif 
			}
		
		}
		else if(distance>DIS_2)
		{
			if(direction==0)
			{
				#ifdef TO_TARGET_DEBUG 
					printf("----forward---\r\n");
				#endif 
					short result=forward_to_pos(SPEED_2,end_x,end_y);
				if(result==1)
				{
					break;
				}
				else if(result ==2)
				{
					return ;
				}
			}
			else if(direction==-1)
			{
			#ifdef TO_TARGET_DEBUG 
				printf("----left %f---\r\n",angle);
			#endif 
					left((short)angle,(short)distance);
			}
			else if(direction==1)
			{
				#ifdef TO_TARGET_DEBUG 
			printf("----right %f---\r\n",angle);
		#endif 
				right(angle,distance);
		}
		else 
			{
			#ifdef TO_TARGET_DEBUG
				printf("wrong in direction\r\n");
			#endif 
		}
		}
		else if(distance>DIS_3)
		{
				if(direction==0)
			{
				#ifdef TO_TARGET_DEBUG 
					printf("----forward---\r\n");
				#endif 
					short result=forward_to_pos(SPEED_3,end_x,end_y);
				if(result==1)
				{
					break;
				}
				else if(result==2)
				{
					return ;
				}
			}
			else if(direction==-1)
			{
			#ifdef TO_TARGET_DEBUG 
				printf("----left %f---\r\n",angle);
			#endif 
					left(angle,distance);
			}
			else if(direction==1)
			{
				#ifdef TO_TARGET_DEBUG 
			printf("----right %f---\r\n",angle);
		#endif 
				right(angle,distance);
		}
		else 
		{
			#ifdef TO_TARGET_DEBUG
				printf("wrong in direction\r\n");
			#endif 
		}
		}
		else if(distance>DIS_4)
		{
				if(direction==0)
			{
				#ifdef TO_TARGET_DEBUG 
					printf("----forward---\r\n");
				#endif 
					short result=forward_to_pos(SPEED_4,end_x,end_y);
				if(result==1)
				{
					break;
				}
				else if(result==2)
				{
					return ;
				}
			}
			else if(direction==-1)
			{
			#ifdef TO_TARGET_DEBUG 
				printf("----left %f---\r\n",angle);
			#endif 
					left(angle,distance);
			}
			else if(direction==1)
			{
				#ifdef TO_TARGET_DEBUG 
			printf("----right %f---\r\n",angle);
		#endif 
				right(angle,distance);
		}
		else 
		{
			#ifdef TO_TARGET_DEBUG
				printf("wrong in direction\r\n");
			#endif 
		}
		}
		
		else if(distance>DIS_5)
		{
				if(direction==0)
			{
				#ifdef TO_TARGET_DEBUG 
					printf("----forward---\r\n");
				#endif 
					short result=forward_to_pos(SPEED_5,end_x,end_y);
				if(result==1)
				{
					break;
				}
				else if(result==2)
				{
					return ;
				}
			}
			else if(direction==-1)
			{
			#ifdef TO_TARGET_DEBUG 
				printf("----left %f---\r\n",angle);
			#endif 
					left(angle,distance);
			}
			else if(direction==1)
			{
				#ifdef TO_TARGET_DEBUG 
			printf("----right %f---\r\n",angle);
		#endif 
				right(angle,distance);
		}
		else 
		{
			#ifdef TO_TARGET_DEBUG
				printf("wrong in direction\r\n");
			#endif 
		}
		}
		
		
		else 
		{
			if(direction==0)
			{
				#ifdef TO_TARGET_DEBUG 
					printf("----forward---\r\n");
				#endif 
					short result=forward_to_pos(SPEED_6,end_x,end_y);
					if(result==1)
					{
						break;
					}
					else if(result==2)
					{
						return ;
					}
			}
			else if(direction==-1)
			{
			#ifdef TO_TARGET_DEBUG 
				printf("----left %f---\r\n",angle);
			#endif 
					left(angle,distance);
			}
			else if(direction==1)
			{
				#ifdef TO_TARGET_DEBUG 
			printf("----right %f---\r\n",angle);
		#endif 
				right(angle,distance);
		}
		else 
		{
			#ifdef TO_TARGET_DEBUG
				printf("wrong in direction\r\n");
			#endif 
		}
		}
		
		HAL_Delay(MOVE_CHANGE_TIME);  		//if need be : change this 
	}
	
	printf("\r\n\r\n");
	
}




//if left -1, if forward(in thre) 0, if right 1
short target_direction(int end_x,int end_y,float* angle)
{
	float target_angle;
	short distance=get_distance(end_x,end_y);
	target_angle=atan2(end_y-current_y,end_x-current_x)*180/PI;
	*angle=0;
	if(target_angle<0)
			target_angle+=360;
	
	#ifdef DIRECTION_DEBUG
		printf("--target_angle:%f\r\n",target_angle);
		printf("--current_angle:%f\r\n",current_angle);
	#endif 

	
	if(distance>DIS_B)
	{
			if(fabs(target_angle-current_angle)<ANGLE_THRE)
	{
		#ifdef DIRECTION_DEBUG 
			printf("target is forward1\r\n");
		#endif 
		return 0;
	}
	else 
	{
		if(current_angle<ANGLE_THRE||current_angle>360-ANGLE_THRE)
		{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE)
			{
		#ifdef DIRECTION_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 0;
			}
		}
		
		
				if(target_angle>current_angle)
				{
						if(target_angle-current_angle>180)
						{
							*angle=360-(target_angle-current_angle);
						#ifdef DIRECTION_DEBUG
			printf("target is right1\r\n");
		#endif 
			return 1 ;
						}
						else if(target_angle-current_angle<=180)
						{
			*angle=target_angle-current_angle;
		#ifdef DIRECTION_DEBUG
			printf("target is left1\r\n");
		#endif 
			return -1;
						}
						else 
						{
		#ifdef DIRECTION_DEBUG
			printf("target is forward3\r\n");
		#endif 
			return 0;
						}
				}
				else if(target_angle<current_angle)
				{
						if(current_angle-target_angle>180)
						{
			*angle=360-(current_angle-target_angle);
		#ifdef DIRECTION_DEBUG
			printf("target is left2\r\n");
		#endif 
						return -1;
						}
						else if(current_angle-target_angle<=180)
						{
			*angle=current_angle-target_angle;
		#ifdef DIRECTION_DEBUG
			printf("target is right2\r\n");
		#endif 
			return 1;
						}
						else 
						{
			return 0;
						}
				}
				else 
				{
					return 0;
				}
		
	}

	}
	else if(distance>DIS_M)
	{
				if(fabs(target_angle-current_angle)<ANGLE_THRE2)
				{
		#ifdef DIRECTION_DEBUG 
			printf("target is forward1\r\n");
		#endif 
		return 0;
	}
	else 
	{
		if(current_angle<ANGLE_THRE2||current_angle>360-ANGLE_THRE2)
		{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE2)
			{
		#ifdef DIRECTION_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 0;
			}
		}
		
		
				if(target_angle>current_angle)
				{
						if(target_angle-current_angle>180)
						{
							*angle=360-(target_angle-current_angle);
						#ifdef DIRECTION_DEBUG
			printf("target is right1\r\n");
		#endif 
			return 1 ;
						}
						else if(target_angle-current_angle<=180)
						{
			*angle=target_angle-current_angle;
		#ifdef DIRECTION_DEBUG
			printf("target is left1\r\n");
		#endif 
			return -1;
						}
						else 
						{
		#ifdef DIRECTION_DEBUG
			printf("target is forward3\r\n");
		#endif 
			return 0;
						}
				}
				else if(target_angle<current_angle)
				{
						if(current_angle-target_angle>180)
						{
			*angle=360-(current_angle-target_angle);
		#ifdef DIRECTION_DEBUG
			printf("target is left2\r\n");
		#endif 
						return -1;
						}
						else if(current_angle-target_angle<=180)
						{
			*angle=current_angle-target_angle;
		#ifdef DIRECTION_DEBUG
			printf("target is right2\r\n");
		#endif 
			return 1;
						}
						else 
						{
			return 0;
						}
				}
				else 
				{
					return 0;
				}
		
	}

	}
	
	
	else if(distance>DIS_S)
	{
		if(fabs(target_angle-current_angle)<ANGLE_THRE3)
		{
		#ifdef DIRECTION_DEBUG 
			printf("target is forward1\r\n");
		#endif 
		return 0;
		}
		else 
		{
		if(current_angle<ANGLE_THRE3||current_angle>360-ANGLE_THRE3)
		{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE3)
			{
		#ifdef DIRECTION_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 0;
			}
		}
		
		
				if(target_angle>current_angle)
				{
						if(target_angle-current_angle>180)
						{
							*angle=360-(target_angle-current_angle);
						#ifdef DIRECTION_DEBUG
			printf("target is right1\r\n");
		#endif 
			return 1 ;
						}
						else if(target_angle-current_angle<=180)
						{
			*angle=target_angle-current_angle;
		#ifdef DIRECTION_DEBUG
			printf("target is left1\r\n");
		#endif 
			return -1;
						}
						else 
						{
		#ifdef DIRECTION_DEBUG
			printf("target is forward3\r\n");
		#endif 
			return 0;
						}
				}
				else if(target_angle<current_angle)
				{
						if(current_angle-target_angle>180)
						{
			*angle=360-(current_angle-target_angle);
		#ifdef DIRECTION_DEBUG
			printf("target is left2\r\n");
		#endif 
						return -1;
						}
						else if(current_angle-target_angle<=180)
						{
			*angle=current_angle-target_angle;
		#ifdef DIRECTION_DEBUG
			printf("target is right2\r\n");
		#endif 
			return 1;
						}
						else 
						{
			return 0;
						}
				}
				else 
				{
					return 0;
				}
		
	}
	}
	
	else 
	{
			if(fabs(target_angle-current_angle)<ANGLE_THRE4)
		{
		#ifdef DIRECTION_DEBUG 
			printf("target is forward1\r\n");
		#endif 
		return 0;
		}
		else 
		{
		if(current_angle<ANGLE_THRE4||current_angle>360-ANGLE_THRE4)
		{
			if(fabs(current_angle-target_angle)>360-ANGLE_THRE4)
			{
		#ifdef DIRECTION_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 0;
			}
		}
		
		
				if(target_angle>current_angle)
				{
						if(target_angle-current_angle>180)
						{
							*angle=360-(target_angle-current_angle);
						#ifdef DIRECTION_DEBUG
			printf("target is right1\r\n");
		#endif 
			return 1 ;
						}
						else if(target_angle-current_angle<=180)
						{
			*angle=target_angle-current_angle;
		#ifdef DIRECTION_DEBUG
			printf("target is left1\r\n");
		#endif 
			return -1;
						}
						else 
						{
		#ifdef DIRECTION_DEBUG
			printf("target is forward3\r\n");
		#endif 
			return 0;
						}
				}
				else if(target_angle<current_angle)
				{
						if(current_angle-target_angle>180)
						{
			*angle=360-(current_angle-target_angle);
		#ifdef DIRECTION_DEBUG
			printf("target is left2\r\n");
		#endif 
						return -1;
						}
						else if(current_angle-target_angle<=180)
						{
			*angle=current_angle-target_angle;
		#ifdef DIRECTION_DEBUG
			printf("target is right2\r\n");
		#endif 
			return 1;
						}
						else 
						{
			return 0;
						}
				}
				else 
				{
					return 0;
				}
		
	}
	}
	
	//	if(fabs(target_angle-current_angle)<ANGLE_THRE)
//	{
//		#ifdef DIRECTION_DEBUG 
//			printf("target is forward1\r\n");
//		#endif 
//		return 0;
//	}
//	else 
//	{
//		if(current_angle<ANGLE_THRE||current_angle>360-ANGLE_THRE)
//		{
//			if(fabs(current_angle-target_angle)>360-ANGLE_THRE)
//			{
//		#ifdef DIRECTION_DEBUG
//			printf("target is forward2\r\n");
//		#endif 
//				return 0;
//			}
//		}
//		
//		
//				if(target_angle>current_angle)
//				{
//						if(target_angle-current_angle>180)
//						{
//							*angle=360-(target_angle-current_angle);
//						#ifdef DIRECTION_DEBUG
//			printf("target is right1\r\n");
//		#endif 
//			return 1 ;
//						}
//						else if(target_angle-current_angle<=180)
//						{
//			*angle=target_angle-current_angle;
//		#ifdef DIRECTION_DEBUG
//			printf("target is left1\r\n");
//		#endif 
//			return -1;
//						}
//						else 
//						{
//		#ifdef DIRECTION_DEBUG
//			printf("target is forward3\r\n");
//		#endif 
//			return 0;
//						}
//				}
//				else if(target_angle<current_angle)
//				{
//						if(current_angle-target_angle>180)
//						{
//			*angle=360-(current_angle-target_angle);
//		#ifdef DIRECTION_DEBUG
//			printf("target is left2\r\n");
//		#endif 
//						return -1;
//						}
//						else if(current_angle-target_angle<=180)
//						{
//			*angle=current_angle-target_angle;
//		#ifdef DIRECTION_DEBUG
//			printf("target is right2\r\n");
//		#endif 
//			return 1;
//						}
//						else 
//						{
//			return 0;
//						}
//				}
//				else 
//				{
//					return 0;
//				}
//		
//	}

}




short target_direction_s(int end_x,int end_y,float* angle)
{
	float target_angle;
	short distance=get_distance(end_x,end_y);
	target_angle=atan2(end_y-current_y,end_x-current_x)*180/PI;
	*angle=0;
	if(target_angle<0)
			target_angle+=360;
	
	#ifdef DIRECTION_DEBUG
		printf("--target_angle:%f\r\n",target_angle);
		printf("--current_angle:%f\r\n",current_angle);
	#endif 
	
		if(fabs(target_angle-current_angle)<TUNE_POS_ANGLE_THRE)
		{
		#ifdef DIRECTION_DEBUG 
			printf("target is forward1\r\n");
		#endif 
		return 0;
		}
		else 
		{
		if(current_angle<TUNE_POS_ANGLE_THRE||current_angle>360-TUNE_POS_ANGLE_THRE)
		{
			if(fabs(current_angle-target_angle)>360-TUNE_POS_ANGLE_THRE)
			{
		#ifdef DIRECTION_DEBUG
			printf("target is forward2\r\n");
		#endif 
				return 0;
			}
		}
		
		
				
		if(target_angle>current_angle)
			{
						if(target_angle-current_angle>180)
						{
							*angle=360-(target_angle-current_angle);
						#ifdef DIRECTION_DEBUG
			printf("target is right1\r\n");
		#endif 
			return 1 ;
						}
						else if(target_angle-current_angle<=180)
						{
			*angle=target_angle-current_angle;
		#ifdef DIRECTION_DEBUG
			printf("target is left1\r\n");
		#endif 
			return -1;
						}
						else 
						{
		#ifdef DIRECTION_DEBUG
			printf("target is forward3\r\n");
		#endif 
			return 0;
						}
				}
				else if(target_angle<current_angle)
				{
						if(current_angle-target_angle>180)
						{
			*angle=360-(current_angle-target_angle);
		#ifdef DIRECTION_DEBUG
			printf("target is left2\r\n");
		#endif 
						return -1;
						}
						else if(current_angle-target_angle<=180)
						{
			*angle=current_angle-target_angle;
		#ifdef DIRECTION_DEBUG
			printf("target is right2\r\n");
		#endif 
			return 1;
						}
						else 
						{
			return 0;
						}
				}
				else 
				{
					return 0;
				}
	
}
	
	//	if(fabs(target_angle-current_angle)<ANGLE_THRE)
//	{
//		#ifdef DIRECTION_DEBUG 
//			printf("target is forward1\r\n");
//		#endif 
//		return 0;
//	}
//	else 
//	{
//		if(current_angle<ANGLE_THRE||current_angle>360-ANGLE_THRE)
//		{
//			if(fabs(current_angle-target_angle)>360-ANGLE_THRE)
//			{
//		#ifdef DIRECTION_DEBUG
//			printf("target is forward2\r\n");
//		#endif 
//				return 0;
//			}
//		}
//		
//		
//				if(target_angle>current_angle)
//				{
//						if(target_angle-current_angle>180)
//						{
//							*angle=360-(target_angle-current_angle);
//						#ifdef DIRECTION_DEBUG
//			printf("target is right1\r\n");
//		#endif 
//			return 1 ;
//						}
//						else if(target_angle-current_angle<=180)
//						{
//			*angle=target_angle-current_angle;
//		#ifdef DIRECTION_DEBUG
//			printf("target is left1\r\n");
//		#endif 
//			return -1;
//						}
//						else 
//						{
//		#ifdef DIRECTION_DEBUG
//			printf("target is forward3\r\n");
//		#endif 
//			return 0;
//						}
//				}
//				else if(target_angle<current_angle)
//				{
//						if(current_angle-target_angle>180)
//						{
//			*angle=360-(current_angle-target_angle);
//		#ifdef DIRECTION_DEBUG
//			printf("target is left2\r\n");
//		#endif 
//						return -1;
//						}
//						else if(current_angle-target_angle<=180)
//						{
//			*angle=current_angle-target_angle;
//		#ifdef DIRECTION_DEBUG
//			printf("target is right2\r\n");
//		#endif 
//			return 1;
//						}
//						else 
//						{
//			return 0;
//						}
//				}
//				else 
//				{
//					return 0;
//				}
//		
//	}

}






void tune_pos(short end_x,short end_y)
{
	float angle;
	short direction;

#ifdef TO_TARGET_DEBUG
		printf("\r\n-----tune pos to target (%d,%d) start:-------\r\n",end_x,end_y);
#endif 
	
	if(current_state==GAME_WAITING)
	{
			return ;
	}
	
	while(!is_arrived_s(current_x,current_y,end_x,end_y))
	{
		//get target info
		direction=target_direction_s(end_x,end_y,&angle);
		
		#ifdef TO_TARGET_DEBUG
			printf("--direction:%d,angle:%f\r\n",direction,angle);
			printf("--currnet angle:%f\r\n",current_angle);
			printf("--current (x,y): %d,%d\r\n",current_x,current_y);
		#endif 
		
		if(current_state==GAME_WAITING)
		{
			return ;
		}
		
		//if target in other car 
		if(current_state==WAIT_CUS&&is_target_in_other_car()!=0)
		{
				current_state=NO_CUS;
				return ;
		}
		

		
		
		#ifdef TO_TARGET_DEBUG 
			printf("\r\n move:\r\n");
		#endif 
		//judget how to move 
			if(direction==0)
			{
				#ifdef TO_TARGET_DEBUG 
					printf("----forward a little---\r\n");
				#endif 
					forward_a_little(end_x,end_y);
			}
			else if(direction==-1)
			{
			#ifdef TO_TARGET_DEBUG 
				printf("----left %f---\r\n",angle);
			#endif 
					left(angle,10);
			}
			else if(direction==1)
			{
				#ifdef TO_TARGET_DEBUG 
			printf("----right %f---\r\n",angle);
		#endif 
				right(angle,10);
		}
		else 
		{
			#ifdef TO_TARGET_DEBUG
				printf("wrong in direction\r\n");
			#endif 
		}
		
		HAL_Delay(MOVE_CHANGE_TIME);  		//if need be : change this 
	}
	
	printf("\r\n\r\n");
	
}

//to judge if other car is forward or near
//if near but not forward , return 2
//if near and forward , return 1 
//if not , return 0
short is_enemy_near(void)
{
	short distance=get_distance(enemy_x,enemy_y);
	
	if(distance<ENEMY_NEAR_DIS_S&&(is_target_forward2(enemy_x,enemy_y)==1))
	{
		#ifdef STATE_DEBUG 
				printf("\r\n!!!enemy is forward!!!\r\n");
				printf("distance:%d, target_forward:%d\r\n\r\n",distance,is_target_forward2(enemy_x,enemy_y));
		#endif 
		
		return 1;
	}
	else if(distance<ENEMY_NEAR_DIS_S&&(is_target_forward2(enemy_x,enemy_y)==0))
	{
		#ifdef STATE_DEBUG 
				printf("\r\n!!!enemy near , not forward \r\n");
				printf("distance:%d, target_forward:%d\r\n\r\n",distance,is_target_forward2(enemy_x,enemy_y));
		#endif 
		
		return 2;
	}
	
	return 0;
}









