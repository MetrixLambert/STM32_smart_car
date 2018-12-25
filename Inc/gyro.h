#include "gyro.h"

extern uint16_t uart3RxLength;
extern uint8_t uart3Rx[UART3_BUF_MAX];
extern float current_angle;

float get_current_angle(void)
{
		if(uart3RxLength!=0)
		{
			if(uart3Rx[22]==0x55)
			{
				if(uart3Rx[23]==0x53)
				{
				  current_angle=((float)(uart3Rx[29]*256)+uart3Rx[28])/32768*180+ANGLE_INIT;
#ifdef DEBUG
					printf("yaw= %f\r\n",current_angle);
#endif					
				}
			}
			uart3RxLength=0;
		}
		return current_angle;
}	