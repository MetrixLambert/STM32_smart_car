#include "wifi.h"
#include "control.h" 
#include "state.h"

//from main.c
extern UART_HandleTypeDef huart2;
extern uint8_t uart1Rx[UART1_BUF_MAX];
extern uint16_t uart1RxLength;

extern uint8_t uart2Rx[UART2_BUF_MAX];
extern uint16_t uart2RxLength;


//from state.c
extern short game_mode;
extern enum car_case current_car_case;
extern enum game_state current_game_state;
extern enum state current_state;
extern short current_round;

extern short my_penalty;
extern short enemy_penalty;
extern short my_score;
extern short enemy_score;

//from control.c
extern short current_x,current_y;
extern short enemy_x,enemy_y;	
extern float current_angle;

//from cus.c
extern short cus_num;
extern short old_cus_num;
extern short cus_state[6];
extern short cus_x[6];
extern short cus_y[6];
extern short des_x[6];
extern short des_y[6];


//extern state current_state;
//extern int current_x,current_y;	//to save current pos 
//extern int enemy_x,enemy_y;		//enemy pos 

void USR_WifiInit(void)
{
#ifdef WIFI_DEBUG
	int i;
#endif 
	
	//try AT
	HAL_UART_Transmit(&huart2, (uint8_t*)"AT\r\n", 4, 1000);
	HAL_Delay(100);
#ifdef WIFI_DEBUG
	while(1)
	{
		if(uart2RxLength==0)
		{
			printf("AT wrong\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)"AT\r\n", 4, 1000);
		}
		else 
		{
			for(i=0;i<uart2RxLength;i++)
				printf("%c",uart2Rx[i]);
			printf("\r\n");
			uart2RxLength=0;
			break;
		}	
		HAL_Delay(100);
	}
#endif 
//	//set mode
	HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CWMODE=3\r\n", 13, 1000);
	HAL_Delay(1000);
#ifdef WIFI_DEBUG
	while(1)
	{
		if(uart2RxLength==0)
		{
			printf("AT MODE wrong\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CWMODE=3\r\n", 13, 1000);
		}
			else 
		{
			for(i=0;i<uart2RxLength;i++)
				printf("%c",uart2Rx[i]);
			printf("\r\n");
						uart2RxLength=0;
			break;
		}	
		HAL_Delay(100);
	}
#endif	
	//reset
	HAL_UART_Transmit(&huart2, (uint8_t*)"AT+RST\r\n", 8, 1000);
	HAL_Delay(1000);
#ifdef WIFI_DEBUG	
	while(1)
	{
		if(uart2RxLength==0)
		{
			printf("AT RST wrong\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)"AT+RST\r\n", 8, 1000);
		}
			else 
		{
			for(i=0;i<uart2RxLength;i++)
				printf("%c",uart2Rx[i]);
			
			printf("\r\n");
			uart2RxLength=0;
			
			break;
		}	
		HAL_Delay(100);
	}
#endif	
	
	//join the access point
	HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CWJAP=\"EDC20\",\"12345678\"\r\n", 29, 1000);
	HAL_Delay(5000);
#ifdef WIFI_DEBUG	
	while(1)
	{
		if(uart2RxLength==0)
		{
			printf("AT CWJAP wrong\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CWJAP=\"EDC20\",\"12345678\"\r\n", 29, 1000);
		}
			else 
		{
			for(i=0;i<uart2RxLength;i++)
				printf("%c",uart2Rx[i]);
			printf("\r\n");
			uart2RxLength=0;
			
			break;
		}	
		HAL_Delay(1000);
	}
#endif		
	//start the tcp connection
	HAL_Delay(2000);
	uart2RxLength=0;
	HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CIPSTART=\"TCP\",\"192.168.1.124\",20000\r\n", 40, 1000);
	HAL_Delay(2000);
#ifdef WIFI_DEBUG	
	while(1)
	{
		if(uart2RxLength==0)
		{
			printf("AT CIPSTART wrong\r\n"); 
			HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CIPSTART=\"TCP\",\"192.168.1.124\",20000\r\n", 40, 1000);
		}
		else 
		{
			for(i=0;i<uart2RxLength;i++)
				printf("%c",uart2Rx[i]);
			printf("\r\n");
			uart2RxLength=0;
			break;
		}	
		HAL_Delay(1000);
	}
#endif		
	
#ifdef WIFI_DEBUG	
	//query if the connection is set up 
	HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CIPSTATUS\r\n", 14, 1000);
	while(1)
	{
		if(uart2RxLength==0)
		{
			printf("AT CWCIP wrong\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CIPSTATUS\r\n", 14, 1000);
		}
		else 
		{
			for(i=0;i<uart2RxLength;i++)
				printf("%c",uart2Rx[i]);
			printf("\r\n");
			uart2RxLength=0;
			break;
		}	
		HAL_Delay(100);
	}
	printf("wifi inited\r\n");
#endif		
}

void Wifi_Decode(void)
{
#ifdef WIFI_DECODE_DEBUG 
//		printf("---into wifi decode---\r\n");
#endif 

	
	
	if(uart2Rx[8]=='4')
	{
		game_mode=uart2Rx[10]>>6;

	if(game_mode==0)
	{
		current_game_state=NOT_START;
	}
	else if(game_mode==2)
	{
		current_game_state=PAUSE;
	}
	else 
	{
		if(current_game_state!=ING)
		{
			current_game_state=ING;
#ifdef STATE_DEBUG 
			printf("change current state!!!!\r\n");
#endif 
			current_state=NO_CUS;
		}
	}
	
	current_round = ((short)(uart2Rx[10]&0x3f)<<8)|uart2Rx[11];
#ifdef STATE_DEBUG  
		printf("\r\n-----current round:%d-----\r\n",current_round);
		printf("\r\n-----current x, y : ( %d , %d )\r\n",current_x,current_y); 
#endif 	

	if(current_round>=1200||current_round==0)
	{
		current_state=GAME_WAITING;
		current_game_state=PAUSE;
	}
	
	
	
	if(current_car_case==A_CAR)
	{
	//A car 
	current_x = ((short)(uart2Rx[12]&0x80)<<1)|uart2Rx[15];
	current_y = ((short)(uart2Rx[12]&0x40)<<2)|uart2Rx[16];
	
	//B car 
	enemy_x = ((short)(uart2Rx[12]&0x20)<<3)|uart2Rx[17];
	enemy_y = ((short)(uart2Rx[12]&0x10)<<4)|uart2Rx[18];
		
	my_penalty = uart2Rx[41];
	enemy_penalty = uart2Rx[42];
	my_score = ((short)uart2Rx[43]<<8)|uart2Rx[45];
	enemy_score = ((short)uart2Rx[44]<<8)|uart2Rx[46];	
	}
	else
	{
	//A car 
	enemy_x = ((short)(uart2Rx[12]&0x80)<<1)|uart2Rx[15];
	enemy_y = ((short)(uart2Rx[12]&0x40)<<2)|uart2Rx[16];
	
	//B car 
	current_x = ((short)(uart2Rx[12]&0x20)<<3)|uart2Rx[17];
	current_y = ((short)(uart2Rx[12]&0x10)<<4)|uart2Rx[18];
		
	//A CAR 
	enemy_penalty = uart2Rx[41];
	//B car 
	my_penalty = uart2Rx[42];
	//A car 
	enemy_score = ((short)uart2Rx[43]<<8)|uart2Rx[45];
	//B car 
	my_score = ((short)uart2Rx[44]<<8)|uart2Rx[46];		
	}
	
	cus_num = uart2Rx[19]>>2;
	
	if(cus_num>old_cus_num)
	{
		old_cus_num=cus_num;
		if(current_state==NO_CUS||current_state==TO_CUS)
		{
				current_state=NO_CUS;
		}
	}
	
	cus_state[1] = uart2Rx[19]&0x03;
	cus_state[2] = (uart2Rx[20]>>6)&0x03;
	cus_state[3] = (uart2Rx[20]>>4)&0x03;
	cus_state[4] = (uart2Rx[20]>>2)&0x03;
	cus_state[5] = uart2Rx[20]&0x03;
	
	cus_x[1] = ((short)(uart2Rx[12]&0x08)<<5)|uart2Rx[21];
	cus_x[2] = ((short)(uart2Rx[13]&0x80)<<1)|uart2Rx[25];
	cus_x[3] = ((short)(uart2Rx[13]&0x08)<<5)|uart2Rx[29];
	cus_x[4] = ((short)(uart2Rx[14]&0x80)<<1)|uart2Rx[33];
	cus_x[5] = ((short)(uart2Rx[14]&0x08)<<5)|uart2Rx[37];
	
	cus_y[1] = ((short)(uart2Rx[12]&0x04)<<6)|uart2Rx[22];
	cus_y[2] = ((short)(uart2Rx[13]&0x40)<<2)|uart2Rx[26];
	cus_y[3] = ((short)(uart2Rx[13]&0x04)<<6)|uart2Rx[30];
	cus_y[4] = ((short)(uart2Rx[14]&0x40)<<2)|uart2Rx[34];
	cus_y[5] = ((short)(uart2Rx[14]&0x04)<<6)|uart2Rx[38];
	
	des_x[1] = ((short)(uart2Rx[12]&0x02)<<7)|uart2Rx[23];
	des_x[2] = ((short)(uart2Rx[13]&0x20)<<3)|uart2Rx[27];
	des_x[3] = ((short)(uart2Rx[13]&0x02)<<7)|uart2Rx[31];
	des_x[4] = ((short)(uart2Rx[14]&0x20)<<3)|uart2Rx[35];
	des_x[5] = ((short)(uart2Rx[14]&0x02)<<7)|uart2Rx[39];
	
	des_y[1] = ((short)(uart2Rx[12]&0x01)<<8)|uart2Rx[24];
	des_y[2] = ((short)(uart2Rx[13]&0x10)<<4)|uart2Rx[28];
	des_y[3] = ((short)(uart2Rx[13]&0x01)<<8)|uart2Rx[32];
	des_y[4] = ((short)(uart2Rx[14]&0x10)<<4)|uart2Rx[36];
	des_y[5] = ((short)(uart2Rx[14]&0x01)<<8)|uart2Rx[40];
	}
}

int check_bt(void)
{
	if(uart1RxLength==0)
	{
#ifdef BT_DEBUG
		printf("nothig receive\r\n");
#endif 
			return 0;
	}
	else 
	{
		uart1RxLength=0;
		if(uart1Rx[0]==0x00)
		{
			//forward 
			if(uart1Rx[1]==0x00)
			{
#ifdef BT_DEBUG
				printf("forward\r\n");
#endif
				left(180,20);
				return 1;
			}
			//stop
			else if(uart1Rx[1]==0x01)
			{
#ifdef BT_DEBUG
				printf("stop\r\n");
#endif
				stop();
				return 2;
			}
			//backward
			else if(uart1Rx[1]==0x02)
			{
#ifdef BT_DEBUG
				printf("backward\r\n");
#endif
				backward(60);
				return 3;
			}
#ifdef BT_DEBUG 
			printf("the second not valid\r\n");
#endif 
			
		}
		else if(uart1Rx[0]==0x01)
		{
			//left 45
			if(uart1Rx[1]==0x00)
			{
#ifdef BT_DEBUG
				printf("left 45\r\n");
#endif
				left(45,60);
				return 4;
			}
			//left 90
			else if(uart1Rx[1]==0x01)
			{
				left(90,60);
#ifdef BT_DEBUG
				printf("left 90\r\n");
#endif
				return 5;
			}
			//left 180
			else if(uart1Rx[1]==0x02)
			{
				left(180,1);
#ifdef BT_DEBUG
				printf("left 180\r\n");
#endif
				return 6;
			}
			
#ifdef BT_DEBUG 
			printf("the second not valid\r\n");
#endif 
		}
		else if(uart1Rx[0]==0x10)
		{
			//right 45
			if(uart1Rx[1]==0x00)
			{
#ifdef BT_DEBUG
				printf("right 45\r\n");
#endif
				right(45,10);
				return 7;
			}
			//right 90
			else if(uart1Rx[1]==0x01)
			{
#ifdef BT_DEBUG
				printf("right 90\r\n");
#endif
				right(90,10);
				return 8;
			}
			//right 180
			else if(uart1Rx[1]==0x02)
			{
#ifdef BT_DEBUG
				printf("right 180\r\n");
#endif
				right(180,10);
				return 9;
			}
			
#ifdef BT_DEBUG 
			printf("the second not valid\r\n");
#endif 
						
		}
		else if(uart1Rx[0]==0x11)
		{
			//right 5 
			if(uart1Rx[1]==0x00)
			{
#ifdef BT_DEBUG
				printf("\r\nright 5\r\n");
#endif
				right(5,10);
				return 10;
			}
			//right 15 
			else if(uart1Rx[1]==0x01)
			{
#ifdef BT_DEBUG
				printf("right 15\r\n");
#endif
				right(15,10);
				return 11;
			}
			//nothing define 
			else if(uart1Rx[1]==0x02)
			{
#ifdef BT_DEBUG
				printf("tune\r\n");
#endif
				tune();
				return 12;
			}
			
#ifdef BT_DEBUG 
			printf("the second not valid\r\n");
#endif 
						
		}
		else 
		{
#ifdef BT_DEBUG
			printf("no valid thing come!\r\n");
#endif 

		}		
	}
//is input is invalid 
	return -1;
}
