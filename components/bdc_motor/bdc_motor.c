
#include "bdc_motor.h"
 #include "ledc.h"

void motor_init()
{
    ledc_timer_config_t ledc_timer0 = {
        .duty_resolution = LEDC_DUTY_RES, // resolution of PWM duty
        .freq_hz = LEDC_FREQUENCY,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER0,            // timer index
        .clk_cfg = LEDC_CLK,              // Auto select the source clock
        
    };
    ledc_timer_config_t ledc_timer1 = {
        .duty_resolution = LEDC_DUTY_RES, // resolution of PWM duty
        .freq_hz = LEDC_FREQUENCY,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER1,            // timer index
        .clk_cfg = LEDC_CLK,              // Auto select the source clock
        
    };
    ledc_timer_config_t ledc_timer2 = {
        .duty_resolution = LEDC_DUTY_RES, // resolution of PWM duty
        .freq_hz = LEDC_FREQUENCY,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER2,            // timer index
        .clk_cfg = LEDC_CLK,              // Auto select the source clock
        
    };
    ledc_timer_config_t ledc_timer3 = {
        .duty_resolution = LEDC_DUTY_RES, // resolution of PWM duty
        .freq_hz = LEDC_FREQUENCY,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER3,            // timer index
        .clk_cfg = LEDC_CLK,              // Auto select the source clock
        
    };

    ledc_timer_config(&ledc_timer0);
    ledc_timer_config(&ledc_timer1);
    ledc_timer_config(&ledc_timer2);
    ledc_timer_config(&ledc_timer3);

    ledc_channel_config_t ledc_channel_0 ={
            .channel    = L1_0,
            .duty       = 0,
            .gpio_num   = L1_0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER0,

        };
    ledc_channel_config_t ledc_channel_1 ={
            .channel    = L1_1,
            .duty       = 0,
            .gpio_num   = L1_1_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER0,

        };
    ledc_channel_config_t ledc_channel_2 ={
            .channel    = L2_0,
            .duty       = 0,
            .gpio_num   = 6,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER1,

        };
    ledc_channel_config_t ledc_channel_3 ={
            .channel    = L2_1,
            .duty       = 0,
            .gpio_num   = 7,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER1,

        };
    ledc_channel_config_t ledc_channel_4 ={
            .channel    = R1_0,
            .duty       = 0,
            .gpio_num   = R1_0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER2,

        };
    ledc_channel_config_t ledc_channel_5 ={
            .channel    = R1_1,
            .duty       = 0,
            .gpio_num   = R1_1_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER2,

        };
    ledc_channel_config_t ledc_channel_6 ={
            .channel    = R2_0,
            .duty       = 0,
            .gpio_num   = R2_0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER3,

        };
    ledc_channel_config_t ledc_channel_7 ={
            .channel    = R2_1,
            .duty       = 0,
            .gpio_num   = R2_1_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER3,

        };


    ledc_channel_config(&ledc_channel_0);
    ledc_channel_config(&ledc_channel_1);
    ledc_channel_config(&ledc_channel_2);
    ledc_channel_config(&ledc_channel_3);
    ledc_channel_config(&ledc_channel_4);
    ledc_channel_config(&ledc_channel_5);
    ledc_channel_config(&ledc_channel_6);
    ledc_channel_config(&ledc_channel_7);
    


}


void motor_update()
{
    ledc_update_duty(LEDC_LS_MODE, L1_0);
    ledc_update_duty(LEDC_LS_MODE, L2_0);
    ledc_update_duty(LEDC_LS_MODE, R1_0);
    ledc_update_duty(LEDC_LS_MODE, R2_0);
    ledc_update_duty(LEDC_LS_MODE, L1_1);
    ledc_update_duty(LEDC_LS_MODE, L2_1);
    ledc_update_duty(LEDC_LS_MODE, R1_1);
    ledc_update_duty(LEDC_LS_MODE, R2_1);
}



void motor_forward1(){
    ledc_set_duty(LEDC_LS_MODE, L1_0, 42);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 42);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 42);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 42);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_forward2(){
    ledc_set_duty(LEDC_LS_MODE, L1_0, 62);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 62);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 62);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 62);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_forward3(){
    ledc_set_duty(LEDC_LS_MODE, L1_0, 41);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 41);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 41);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 41);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_stop(){
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_retreat1(){
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 42);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 42);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 42);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 42);
    motor_update();
}

void motor_retreat2(){
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 62);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 62);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 62);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 62);
    motor_update();
}

void motor_L_1(){
    // bdc_motor_forward(motor1);
    // bdc_motor_reverse(motor2);
    // bdc_motor_reverse(motor3);
    // bdc_motor_forward(motor4);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 42);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 42);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 42);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 42);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_L_2(){
    ledc_set_duty(LEDC_LS_MODE, L1_0, 62);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 62);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 62);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 62);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_R_1(){
    // bdc_motor_reverse(motor1);
    // bdc_motor_forward(motor2);
    // bdc_motor_forward(motor3);
    // bdc_motor_reverse(motor4);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 42);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 42);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 42);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 42);
    motor_update();
}

void motor_R_2(){
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 62);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 62);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 62);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 62);
    motor_update();
}

void motor_R_forward1(){
    // bdc_motor_forward(motor2);
    // bdc_motor_forward(motor3);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 42);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 42);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_R_forward2(){
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 62);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 62);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_L_forward1(){
    // bdc_motor_forward(motor1);
    // bdc_motor_forward(motor4);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 42);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 42);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_L_forward2(){
    // bdc_motor_forward(motor1);
    // bdc_motor_forward(motor4);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 62);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 62);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_R_retreat1(){
    // bdc_motor_reverse(motor1);
    // bdc_motor_reverse(motor4);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 42);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 42);
    motor_update();
}

void motor_R_retreat2(){
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 62);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 62);
    motor_update();
}

void motor_L_retreat1(){
    // bdc_motor_reverse(motor2);
    // bdc_motor_reverse(motor3);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 42);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 42);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_L_retreat2(){
    // bdc_motor_reverse(motor2);
    // bdc_motor_reverse(motor3);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 62);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 62);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}
void motor_R_return1(){
    // bdc_motor_forward(motor1);
    // bdc_motor_forward(motor3);
    // bdc_motor_reverse(motor2);
    // bdc_motor_reverse(motor4);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 42);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 42);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 42);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 42);
    motor_update();
}

void motor_R_return2(){
    // bdc_motor_forward(motor1);
    // bdc_motor_forward(motor3);
    // bdc_motor_reverse(motor2);
    // bdc_motor_reverse(motor4);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 62);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 62);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 62);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 62);
    motor_update();
}

void motor_RS_return(){
        ledc_set_duty(LEDC_LS_MODE, L1_0, 28);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 28);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 28);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 28);
    motor_update();
}

void motor_L_return1(){
    // bdc_motor_forward(motor2);
    // bdc_motor_forward(motor4);
    // bdc_motor_reverse(motor1);
    // bdc_motor_reverse(motor3);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 42);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 42);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 42);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 42);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_L_return2(){
    // bdc_motor_forward(motor2);
    // bdc_motor_forward(motor4);
    // bdc_motor_reverse(motor1);
    // bdc_motor_reverse(motor3);
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 62);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 62);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 62);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 62);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}

void motor_LS_return()
{
    ledc_set_duty(LEDC_LS_MODE, L1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, L2_0, 28);
    ledc_set_duty(LEDC_LS_MODE, R1_0, 0);
    ledc_set_duty(LEDC_LS_MODE, R2_0, 28);
    ledc_set_duty(LEDC_LS_MODE, L1_1, 28);
    ledc_set_duty(LEDC_LS_MODE, L2_1, 0);
    ledc_set_duty(LEDC_LS_MODE, R1_1, 28);
    ledc_set_duty(LEDC_LS_MODE, R2_1, 0);
    motor_update();
}
// void motor_test1(){
//     // bdc_motor_forward(motor1);
//     // bdc_motor_forward(motor2);
//     // bdc_motor_forward(motor3);
//     // bdc_motor_forward(motor4);

//     // bdc_motor_set_speed(motor1, 200);
//     // vTaskDelay(500/portTICK_PERIOD_MS);
//     // bdc_motor_set_speed(motor1, 0);

//     // bdc_motor_set_speed(motor2, 200);
//     // vTaskDelay(500/portTICK_PERIOD_MS);
//     // bdc_motor_set_speed(motor2, 0);

//     // bdc_motor_set_speed(motor3, 200);
//     // vTaskDelay(500/portTICK_PERIOD_MS);
//     // bdc_motor_set_speed(motor3, 0);

//     // bdc_motor_set_speed(motor4, 200);
//     // vTaskDelay(500/portTICK_PERIOD_MS);
//     // bdc_motor_set_speed(motor4, 0);
// }