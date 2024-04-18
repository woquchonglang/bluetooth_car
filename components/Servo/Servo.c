#include "Servo.h"
#include "esp_log.h"
#define TAG  "TAG"
// mcpwm_timer_handle_t timer1 = NULL;
// mcpwm_timer_handle_t timer2 = NULL;
// mcpwm_timer_handle_t timer3 = NULL;


// mcpwm_oper_handle_t oper1 = NULL;
// mcpwm_oper_handle_t oper2 = NULL;
// mcpwm_oper_handle_t oper3 = NULL;


// static mcpwm_cmpr_handle_t comp1 = NULL;
// static mcpwm_cmpr_handle_t comp2 = NULL;
// static mcpwm_cmpr_handle_t comp3 = NULL;


// mcpwm_gen_handle_t gen1 = NULL;
// mcpwm_gen_handle_t gen2 = NULL;
// mcpwm_gen_handle_t gen3 = NULL;

// mcpwm_timer_config_t timer_config1 = {
//     .group_id = 0,
//     .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//     .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
//     .period_ticks = SERVO_TIMEBASE_PERIOD,
//     .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
// };
// mcpwm_timer_config_t timer_config2 = {
//     .group_id = 0,
//     .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//     .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
//     .period_ticks = SERVO_TIMEBASE_PERIOD,
//     .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
// };
// mcpwm_timer_config_t timer_config3 = {
//     .group_id = 0,
//     .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//     .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
//     .period_ticks = SERVO_TIMEBASE_PERIOD,
//     .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
// };

// mcpwm_operator_config_t operator_config1 = {
//     .group_id = 0, // operator must be in the same group to the timer
// };
// mcpwm_operator_config_t operator_config2 = {
//     .group_id = 0, // operator must be in the same group to the timer
// };
// mcpwm_operator_config_t operator_config3 = {
//     .group_id = 0, // operator must be in the same group to the timer
// };

// mcpwm_comparator_config_t comparator_config1 = {
//     .flags.update_cmp_on_tez = true,
// };
// mcpwm_comparator_config_t comparator_config2 = {
//     .flags.update_cmp_on_tez = true,
// };
// mcpwm_comparator_config_t comparator_config3 = {
//     .flags.update_cmp_on_tez = true,
// };

// mcpwm_generator_config_t generator_config1 = {
//     .gen_gpio_num = SERVO_GPIO_1,
// };
// mcpwm_generator_config_t generator_config2 = {
//     .gen_gpio_num = SERVO_GPIO_2,
// };
// mcpwm_generator_config_t generator_config3 = {
//     .gen_gpio_num = SERVO_GPIO_3,
// };

// void servo_init()
// {
//     mcpwm_new_timer(&timer_config1, &timer1);
//     mcpwm_new_timer(&timer_config2, &timer2);
//     mcpwm_new_timer(&timer_config3, &timer3);

//     mcpwm_new_operator(&operator_config1, &oper1);
//     mcpwm_new_operator(&operator_config2, &oper2);
//     mcpwm_new_operator(&operator_config3, &oper3);

//     mcpwm_operator_connect_timer(oper1, timer1);
//     mcpwm_operator_connect_timer(oper2, timer2);
//     mcpwm_operator_connect_timer(oper3, timer3);

//     mcpwm_new_comparator(oper1, &comparator_config1, &comp1);
//     mcpwm_new_comparator(oper2, &comparator_config2, &comp2);
//     mcpwm_new_comparator(oper3, &comparator_config3, &comp3);

//     mcpwm_new_generator(oper1, &generator_config1, &gen1);
//     mcpwm_new_generator(oper2, &generator_config2, &gen2);
//     mcpwm_new_generator(oper3, &generator_config3, &gen3);

//     mcpwm_comparator_set_compare_value(comp1, 1500);
//     mcpwm_comparator_set_compare_value(comp2, 1500);
//     mcpwm_comparator_set_compare_value(comp3, 1500);

//     mcpwm_generator_set_action_on_timer_event(gen1,
//                                               MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, 
//                                               MCPWM_TIMER_EVENT_EMPTY, 
//                                               MCPWM_GEN_ACTION_HIGH));
//     mcpwm_generator_set_action_on_compare_event(gen1,
//                     MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comp1, MCPWM_GEN_ACTION_LOW));
//     mcpwm_timer_enable(timer1);
//     mcpwm_timer_enable(timer2);
//     mcpwm_timer_enable(timer3);

//     mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP);
//     mcpwm_timer_start_stop(timer2, MCPWM_TIMER_START_NO_STOP);
//     mcpwm_timer_start_stop(timer3, MCPWM_TIMER_START_NO_STOP);
    
// }

// void servo1_up_1(){
//     ESP_LOGI(TAG,"servo1_up_1");
//     mcpwm_comparator_set_compare_value(comp1, 1800);
//     vTaskDelay(1000);

// }
// void servo1_up_2(){
//     mcpwm_comparator_set_compare_value(comp1, 2500);
// }
// void servo1_down_1(){
//     mcpwm_comparator_set_compare_value(comp1, 1200);
// }
// void servo1_down_2(){
//     mcpwm_comparator_set_compare_value(comp1, 500);
// }


// void servo2_up_1(){
//     mcpwm_comparator_set_compare_value(comp2, 1800);
// }
// void servo2_up_2(){
//     mcpwm_comparator_set_compare_value(comp2, 2500);
// }
// void servo2_down_1(){
//     mcpwm_comparator_set_compare_value(comp2, 1200);
// }
// void servo2_down_2(){
//     mcpwm_comparator_set_compare_value(comp2, 500);
// }

// void servo_claw_open(){
//     mcpwm_comparator_set_compare_value(comp3, 2500);
// }

// void servo_claw_close(){
//     mcpwm_comparator_set_compare_value(comp3, 500);
// }