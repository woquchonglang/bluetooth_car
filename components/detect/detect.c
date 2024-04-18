
#include "detect.h"


#define DETECT "DETECT"

#define s1 gpio_get_level(GPIO_NUM_11)
#define s2 gpio_get_level(GPIO_NUM_12)
#define s3 gpio_get_level(GPIO_NUM_13)
#define s4 gpio_get_level(GPIO_NUM_14)

#define left_state 1
#define right_state 2
#define straight_right 3
#define straight_left 4
#define straight_state 0

// #define Condition (l1)*1000+(l2)*100+(r2)*10+(r1)

#define condition ((s1<<3) | (s2<<2) | (s3<<1) | (s4))

void detect_gpio_init()
{

	gpio_set_direction(GPIO_NUM_11,  GPIO_MODE_DEF_INPUT);
    gpio_set_direction(GPIO_NUM_12,  GPIO_MODE_DEF_INPUT);
    gpio_set_direction(GPIO_NUM_13,  GPIO_MODE_DEF_INPUT);
    gpio_set_direction(GPIO_NUM_14,  GPIO_MODE_DEF_INPUT);

	gpio_set_pull_mode(GPIO_NUM_11, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_NUM_14, GPIO_PULLUP_ONLY);
}

void Detect_mode(){

	ESP_LOGE(DETECT,"detect create");

	int last_move = straight_state;
	int x=0;

	while(1)
	{

		if(last_move == straight_state)
		{
			motor_forward1();

			if(condition == 8 || condition == 14)
			{
				last_move = left_state;
			}
			else if(condition == 1 || condition == 7)
			{
				last_move = right_state;
			}
			else if(condition == 6 || condition == 15)
			{
				last_move = straight_state;
			}
			else if(condition == 2)
			{
				last_move = straight_right;

			}
			else if(condition == 4)
			{
				last_move = straight_left;
			}


		}


		else if(last_move == right_state)
		{
			if(x==0){

			motor_forward1();
			vTaskDelay(120/portTICK_PERIOD_MS);
			x++;
			}

			if(condition == 6||condition == 2)
			{
				last_move=straight_state;
				x--;
			}
			motor_R_return1();
			
		}


		else if(last_move == left_state)
		{
			if(x==0){
				motor_forward1();
				vTaskDelay(120/portTICK_PERIOD_MS);
				x++;
			}
			if(condition == 6 || condition == 4)
			{
				last_move=straight_state;
				x--;
			}
			motor_L_return1();
			
		}


		else if(last_move == straight_left)
		{
			motor_LS_return();
			vTaskDelay(10/portTICK_PERIOD_MS);
			last_move = 0;
		}


		else if(last_move == straight_right)
		{
			motor_RS_return();
			vTaskDelay(10/portTICK_PERIOD_MS);
			last_move = 0;
		}
		vTaskDelay(10/portTICK_PERIOD_MS);

		

	}


}


