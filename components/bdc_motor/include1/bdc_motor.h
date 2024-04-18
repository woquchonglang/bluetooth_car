#pragma once

  #include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __BCD_MOTOR_H__
#define __BCD_MOTOR_H__


/**
 *        tou
 *     R1      R2
 *     L1      L2
 *
*/

#define LEDC_LS_TIMER0          LEDC_TIMER_0
#define LEDC_LS_TIMER1          LEDC_TIMER_1
#define LEDC_LS_TIMER2          LEDC_TIMER_2
#define LEDC_LS_TIMER3          LEDC_TIMER_3


#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_6_BIT
#define LEDC_FREQUENCY          (100000)
#define LEDC_CLK          LEDC_USE_RTC8M_CLK

#define L1_0_GPIO       4
#define L1_1_GPIO       5
#define L2_0_GPIO       6
#define L2_1_GPIO       7
#define R1_0_GPIO       15
#define R1_1_GPIO       16
#define R2_0_GPIO       17
#define R2_1_GPIO       18

#define L1_0    LEDC_CHANNEL_0
#define L1_1    LEDC_CHANNEL_1
#define L2_0    LEDC_CHANNEL_2
#define L2_1    LEDC_CHANNEL_3
#define R1_0    LEDC_CHANNEL_4
#define R1_1    LEDC_CHANNEL_5
#define R2_0    LEDC_CHANNEL_6
#define R2_1    LEDC_CHANNEL_7


void motor_init();
void motor_forward1();
void motor_forward2();
void motor_stop();
void motor_retreat1();
void motor_retreat2();
void motor_L_1();
void motor_L_2();
void motor_R_1();
void motor_R_2();
void motor_R_forward1();
void motor_R_forward2();
void motor_L_forward1();
void motor_L_forward2();
void motor_L_retreat1();
void motor_L_retreat2();
void motor_R_retreat1();
void motor_R_retreat2();
void motor_R_return1();
void motor_R_return2();
void motor_L_return1();
void motor_L_return2();
void motor_LS_return();
void motor_RS_return();
#endif

#ifdef __cplusplus
}
#endif
