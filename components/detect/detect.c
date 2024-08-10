
#include "detect.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define DETECT "DETECT"

#define s1 gpio_get_level(GPIO_NUM_11)
#define s2 gpio_get_level(GPIO_NUM_12)
#define s3 gpio_get_level(GPIO_NUM_13)
#define s4 gpio_get_level(GPIO_NUM_14)

// #define Condition (l1)*1000+(l2)*100+(r2)*10+(r1)

#define Condition ((s1<<3) | (s2<<2) | (s3<<1) | (s4))

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

	// detect_state = ulTaskNotifyTake(pdTRUE, portMAX_DELAY );

	while(1)
	{

			//ESP_LOGE(DETECT,"%d",Condition);
			
			if(Condition==0 || Condition==9){
					motor_forward1();
			}

			if(Condition==15){
					motor_stop();
			}
	
			if(Condition==12 || Condition==14 || Condition==10 || Condition==8){
					motor_R_return1();
			}

			// if(Condition==8){
			// 		motor_R_return2();
			// }

			if(Condition==5 || Condition==7 || Condition==3 || Condition==1){
					motor_L_return1();
			}

			// if(Condition==1){
			// 		motor_L_return2();
			// }
			
			vTaskDelay(10/portTICK_PERIOD_MS);

	}


}

//feng
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

//yin
// void Detect_mode(){

// 			if(Condition==1111){
// 					motor_stop();
// 			}

// 			if(Condition==0 || Condition==0110){
// 					motor_forward1();
// 			}
					
// 			if(Condition==1100||Condition==1110||Condition==1010){
// 					motor_R_return1();
// 			}

// 			if(Condition==1000){
// 					motor_R_return2();
// 			}

// 			if(Condition==101||Condition==111||Condition==11){
// 					motor_L_return1();
// 			}

// 			if(Condition==1){
// 					motor_l_return2();
// 			}	

// }