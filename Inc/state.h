#ifndef __STATE_H__
#define __STATE_H__

#include "main.h"
#include "stm32f1xx_hal.h"
#include "all_header.h"

/*
*the whole program is based on the state machine
*there is 7 state in this game:

	GAME_WAITTING
：not start, waiting :
//call an funciton : into endless loop to check if the state has changed
//if the state has been changed , out of loop :let the nvic function to change state 

	GAME_START:
//out of garage, based on where the cus cal and which road to take,
//to decide whether turn left or turn right

	NO_CUS:
still has no cus and no target
//first check which cus to go 
//then design the route
//change state 

	TO_CUS:
has not cus, but has target
//first by designed route, go to target
//if near the target, close the motor to wait for the judge
//at the meantime , change state 

	WAIT_CUS:
//waiting for judge 
//if in 3s the judge is not okay, change pos a little 
//if judge is ok, change state to TO_END

	TO_DES:
//to the destination
//first cal the route
//then use the route to go
//if near the target, close motor to wait for judge 
//at the meantime , change the state

	WAIT_GET_OFF:
waiting at the end point
//waiting for the judge 
//if in 3s the judge is not okay, change pos a little 
//if judge is okay, change state 

	GAME_PAUSE
imediately stop and wait for game restart 
//if is this state : call an end_function
//[stop the target and into the endless loop, until if the state is changed]
//if the state has been changed , out of loop :let the nvic function to change state 

*/

#define WAIT_COM_TIME 20	//every time wait for 20 ms 
#define MOVE_CHANGE_TIME 30

enum car_case {A_CAR,B_CAR};
enum game_state{NOT_START,ING,PAUSE};
enum state{GAME_WAITING,GAME_START,NO_CUS,TO_CUS,WAIT_CUS,TO_DES,WAIT_GET_OFF};

//this is an global variable, used to remember the state the car is in 


//GROUP 0: GAME_WAITING or GAME_END
/*function: called when game not start or already end;
* 			to judge: if the game end or start, let the car stop and wait; 
* 			else, end the function
*@para: none 
*@return: none 
*/
void waiting(void);

//GROUP 1:GAME_START
/*	void start_game(void)
*function:	when game start and out of garage
*@para: 	none
*@return: 	none
*/
void start_game(void);

//GROUP 2: NO_CUS

/*	void find_cus(void)
*function: to find the best cus and cal the route
*@para: 	none
*@return: 	none 
*/
void find_cus(void);

//GROUP 3: TO_CUS

/*	void to_cus(void)
*function:	go to the target
*@para: 	none(assume already cal the route)
*@return 	none
*/
void to_cus(void);

//GROUP 4: WAITING_CUS

/*	void wait_cus(void)
*function: 	"waiting for target",waiting for the host computer to judge
			if no okay in 0.2 second, tune the position
			if okay, change current state
*@para: 	none
*@return: 	none
*/
void wait_cus(void);

//GROUP 5: TO_END

/*	void to_end(void)
*function: first cal the route, then go to end point
*@para: 	none 
*@return: 	
*/
void to_des(void);

//GROUP 6: WAIT_GET_OFF

/*	void wait_get_off(void)
*function: 	"waiting for getoff",waiting for the host computer to judge
			if no okay in 0.2 second, tune the position
			if okay, change current state
*
*
*/
void wait_get_off(void);

//to deal with if other car is stamp our way 





#endif








