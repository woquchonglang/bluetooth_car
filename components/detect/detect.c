
#include "detect.h"


#define r1 11
#define r2 12
#define l1 13
#define l2 14

#define Condition l1*1000+l2*100+r2*10+r1  

void Detect_mode(){

			if(Condition==1111||Condition==110){
					motor_forward2();
				}
			if(Condition==1100||Condition==1110||Condition==1010){
					motor_L_return1();
			}
			if(Condition==101||Condition==111||Condition==11){
					motor_R_return1();
			}
			if(Condition==1000){
					motor_L_return2();
			}
			if(Condition==1)
					motor_R_return2();

}