
#include "detect.h"


#define r1 gpio_get_level(GPIO_NUM_11)
#define r2 gpio_get_level(GPIO_NUM_12)
#define l1 gpio_get_level(GPIO_NUM_14)
#define l2 gpio_get_level(GPIO_NUM_13)

// #define Condition (l1)*1000+(l2)*100+(r2)*10+(r1)

#define Condition (l1<<4) + (l2<<3) + (r2<<2) + (r1)

void detect_gpio_init()
{
	gpio_set_direction(GPIO_NUM_11,  GPIO_MODE_DEF_INPUT);
    gpio_set_direction(GPIO_NUM_12,  GPIO_MODE_DEF_INPUT);
    gpio_set_direction(GPIO_NUM_13,  GPIO_MODE_DEF_INPUT);
    gpio_set_direction(GPIO_NUM_14,  GPIO_MODE_DEF_INPUT);
}

void Detect_mode(){

	while(1)
	{
		if(Condition==17 ||Condition==6){
					motor_forward1();
			}
			else if(Condition==14 || Condition==16 || Condition==12){
					motor_L_return1();
			}
			else if(Condition==5 || Condition==7 || Condition==3){
					motor_R_return1();
			}
			else if(Condition==10){
					motor_L_return2();
			}
			else if(Condition==1)
					motor_R_return2();
	}

			

}

// void Detect_mode(){

// 			if(Condition==1111||Condition==110){
// 					motor_forward_detect();
// 			}
// 			if(Condition==1100||Condition==1110||Condition==1010){
// 					motor_L_return1();
// 			}
// 			if(Condition==101||Condition==111||Condition==11){
// 					motor_R_return1();
// 			}
// 			if(Condition==1000){
// 					motor_L_return2();
// 			}
// 			if(Condition==1)
// 					motor_R_return2();

// }