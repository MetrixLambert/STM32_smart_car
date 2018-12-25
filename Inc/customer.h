#ifndef __CUSTOMER_H__
#define __CUSTOMER_H__

#include "main.h"
#include "stm32f1xx_hal.h"
#include "all_header.h"

/* vodi sort_target()
*function: 	cal the best target availble
*@para: 	none (directly gain the info from global variable)
*@return: 	none (already save it in target_num)
*/
short get_distance(short end_x,short end_y);
void sort_target(void);
short find_lucky_cus(void);
short is_target_in_other_car(void);
short scan_enemy_cus(void);
void update_enemy_cus_tag(void);

#endif 
