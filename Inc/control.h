#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include <math.h>
#include "stm32f1xx_hal.h"
#include "all_header.h"


//used in straight to target 
#define ANGLE_THRE_B 9
#define ANGLE_THRE_M 7
#define ANGLE_THRE_S 1

#define ANGLE_THRE_BR 9		//9
#define ANGLE_THRE_MR 7		//7
#define ANGLE_THRE_SR 1		//1

//left and right 
#define ANGLE_B 40
#define ANGLE_M 20
//#define ANGLE_S 10

#define ROTAGE_SPEED_B  70
#define ROTAGE_SPEED_M  60 
#define ROTAGE_SPEED_S	46

//pid para
#define P_DATA 2		//1.9
#define I_DATA 1 			//1
#define D_DATA 0			//0
#define SPEED2PWM 	10   //9.2

#define MAX_SPEED 140
#define SPEED_UPHOLD 65535
#define PWM_MAX 1000

//used in straight to target 
#define ANGLE_TUNE_THRE 3	//3
#define ANGLE_TUNE_THRE2 3	//also waiting to tune 
#define ANGLE_TUNE_THRE3 4
#define ANGLE_TUNE_THRE4 5

//uesd in left and right 
#define ANGLE_1 100 //100
#define ANGLE_2 50	//50
#define ANGLE_3 38	//38
#define ANGLE_4 5
//#define ANGLE_4 38

#define ANGLE_ROTATE_SPEED_B 80
#define ANGLE_ROTATE_SPEED_M 70
#define ANGLE_ROTATE_SPEED_S 60 
#define ANGLE_ROTATE_SPEED_T 50
#define ANGLE_ROTATE_SPEED_ST 30

#define ANGLE_ROTATE_SPEED_CHANGE_THRE 5

#define ANGLE_STOP_THRE_B		27
#define ANGLE_STOP_THRE_M		25
#define ANGLE_STOP_THRE_S		12
#define ANGLE_STOP_THRE_T		4
#define ANGLE_STOP_THRE_ST  1

#define ANGLE_REVERSE_THRE 1
//in left and right , when to tune angel 
//#define ANGLE_TUNE_THRE 2	//3
//#define ANGLE_TUNE_THRE2 3	//also waiting to tune 
//#define ANGLE_TUNE_THRE3 4
//#define ANGLE_TUNE_THRE4 5
//#define ANGLE_TUNE_THRE5 6
//#define ANGLE_TUNE_THRE6 7

//used to judge if arrive
#define ARRIVE_THRE 9
#define ARRIVE_THRE_S 7

#define PWM_THRE 3	

//forward to target used 
#define SPEED_1 100 	//50
#define SPEED_2 80	//40
#define SPEED_3 70	//30
#define SPEED_4 60	//26
#define SPEED_5 45
#define SPEED_6 30

#define SPEED_CHANGE_THRE 5


#define DIS_1 100
#define DIS_2 70
#define DIS_3	55
#define DIS_4 45
#define DIS_5	30


//used in stop 
#define STOP_LIMIT 11	//
#define STOP_LIMIT2 40

//used in straight to target 
#define SPEED_ACCE 10


//direction && is_target_forward 

#define DIS_B	50
#define DIS_M 38 
#define DIS_S 15

#define ANGLE_THRE 7		//7
#define ANGLE_THRE2 10		//8
#define ANGLE_THRE3 12	//10
#define ANGLE_THRE4 23	//20

//tune function 
#define TUNE_POS_ANGLE_THRE  5

////left and right 


//#define ROTAGE_SPEED_B  50
//#define ROTAGE_SPEED_M  50 
//#define ROTAGE_SPEED_S	46
////#define ROTAGE_SPEED_T	40


//to deal with enemy 
#define ENEMY_NEAR_DIS_B 40		//a para need to tune 
#define ENEMY_NEAR_DIS_S 35 	//a para need to tune

/*
*	here are the function used to control how car move
*/

//used else where 
void USR_MotorInit(void);

//higher control function 
void forward(short speed);				//forward
short forward_to_pos(short speed,short end_x,short end_y); //forwad to x,y
void stop(void);				//stop
void backward(short speed);		//backward 
void left(short angle,short r);		//turn left
void right(short angle,short r);		//turn right
void turn_around(void );			//turn around(left)				
void waiting_stop(void);

void upgrade_angle(void);		//upgradge the angle 
void update_speed(void);
void tune(void);					//used in just a try 

//highest contorl function
void straight_to_target(short end_x,short end_y);
void tune_pos(short end_x, short end_y);
short is_target_forward(short end_x,short end_y);
short target_direction(int end_x,int end_y,float* angle);
short is_enemy_near(void);
short forward_a_little(short end_x,short end_y);
							//use the tactic: straight to target
#endif
