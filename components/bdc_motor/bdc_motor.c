/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_check.h"
#include "bdc_motor.h"
#include "bdc_motor_interface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bdc_motor";

esp_err_t bdc_motor_enable(bdc_motor_handle_t motor)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->enable(motor);
}

esp_err_t bdc_motor_disable(bdc_motor_handle_t motor)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->disable(motor);
}

esp_err_t bdc_motor_set_speed(bdc_motor_handle_t motor, uint32_t speed)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->set_speed(motor, speed);
}

esp_err_t bdc_motor_forward(bdc_motor_handle_t motor)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->forward(motor);
}

esp_err_t bdc_motor_reverse(bdc_motor_handle_t motor)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->reverse(motor);
}

esp_err_t bdc_motor_coast(bdc_motor_handle_t motor)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->coast(motor);
}

esp_err_t bdc_motor_brake(bdc_motor_handle_t motor)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->brake(motor);
}

esp_err_t bdc_motor_del(bdc_motor_handle_t motor)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->del(motor);
}


bdc_motor_handle_t motor1 = NULL;
bdc_motor_handle_t motor2 = NULL;
bdc_motor_handle_t motor3 = NULL;
bdc_motor_handle_t motor4 = NULL;

void motor_init()
 {
    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config_1 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_L1_A,
        .pwmb_gpio_num =  BDC_MCPWM_GPIO_L1_B,
    };
    bdc_motor_config_t motor_config_2 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_L2_A,
        .pwmb_gpio_num =  BDC_MCPWM_GPIO_L2_B,
    };
    bdc_motor_config_t motor_config_3 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_R1_A,
        .pwmb_gpio_num =  BDC_MCPWM_GPIO_R1_B,
    };
    bdc_motor_config_t motor_config_4 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_R2_A,
        .pwmb_gpio_num =  BDC_MCPWM_GPIO_R2_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config_1 = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_mcpwm_config_t mcpwm_config_2 = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_mcpwm_config_t mcpwm_config_3 = {
        .group_id = 1,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_mcpwm_config_t mcpwm_config_4 = {
        .group_id = 1,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_new_mcpwm_device(&motor_config_1, &mcpwm_config_1, &motor1);
    bdc_motor_new_mcpwm_device(&motor_config_2, &mcpwm_config_2, &motor2);
    bdc_motor_new_mcpwm_device(&motor_config_3, &mcpwm_config_3, &motor3);
    bdc_motor_new_mcpwm_device(&motor_config_4, &mcpwm_config_4, &motor4);

    bdc_motor_enable(motor1);
    bdc_motor_enable(motor2);
    bdc_motor_enable(motor3);
    bdc_motor_enable(motor4);


   
}

void motor_forward1(){
    bdc_motor_forward(motor1);
    bdc_motor_forward(motor2);
    bdc_motor_forward(motor3);
    bdc_motor_forward(motor4);
    bdc_motor_set_speed(motor1, 200);
    bdc_motor_set_speed(motor2, 200);
    bdc_motor_set_speed(motor3, 200);
    bdc_motor_set_speed(motor4, 200);
}

void motor_forward2(){
    bdc_motor_forward(motor1);
    bdc_motor_forward(motor2);
    bdc_motor_forward(motor3);
    bdc_motor_forward(motor4);
    bdc_motor_set_speed(motor1, 400);
    bdc_motor_set_speed(motor2, 400);
    bdc_motor_set_speed(motor3, 400);
    bdc_motor_set_speed(motor4, 400);
}

void motor_stop(){
    bdc_motor_set_speed(motor1, 0);
    bdc_motor_set_speed(motor2, 0);
    bdc_motor_set_speed(motor3, 0);
    bdc_motor_set_speed(motor4, 0);
}

void motor_retreat1(){
    bdc_motor_reverse(motor1);
    bdc_motor_reverse(motor2);
    bdc_motor_reverse(motor3);
    bdc_motor_reverse(motor4);
    bdc_motor_set_speed(motor1, 200);
    bdc_motor_set_speed(motor2, 200);
    bdc_motor_set_speed(motor3, 200);
    bdc_motor_set_speed(motor4, 200);
}

void motor_retreat2(){
    bdc_motor_reverse(motor1);
    bdc_motor_reverse(motor2);
    bdc_motor_reverse(motor3);
    bdc_motor_reverse(motor4);
    bdc_motor_set_speed(motor1, 400);
    bdc_motor_set_speed(motor2, 400);
    bdc_motor_set_speed(motor3, 400);
    bdc_motor_set_speed(motor4, 400);
}

void motor_L_1(){
    bdc_motor_forward(motor1);
    bdc_motor_reverse(motor2);
    bdc_motor_reverse(motor3);
    bdc_motor_forward(motor4);
    bdc_motor_set_speed(motor1, 200);
    bdc_motor_set_speed(motor2, 200);
    bdc_motor_set_speed(motor3, 200);
    bdc_motor_set_speed(motor4, 200);
}

void motor_L_2(){
    bdc_motor_forward(motor1);
    bdc_motor_reverse(motor2);
    bdc_motor_reverse(motor3);
    bdc_motor_forward(motor4);
    bdc_motor_set_speed(motor1, 400);
    bdc_motor_set_speed(motor2, 400);
    bdc_motor_set_speed(motor3, 400);
    bdc_motor_set_speed(motor4, 400);
}

void motor_R_1(){
    bdc_motor_reverse(motor1);
    bdc_motor_forward(motor2);
    bdc_motor_forward(motor3);
    bdc_motor_reverse(motor4);
    bdc_motor_set_speed(motor1, 200);
    bdc_motor_set_speed(motor2, 200);
    bdc_motor_set_speed(motor3, 200);
    bdc_motor_set_speed(motor4, 200);
}

void motor_R_2(){
    bdc_motor_reverse(motor1);
    bdc_motor_forward(motor2);
    bdc_motor_forward(motor3);
    bdc_motor_reverse(motor4);
    bdc_motor_set_speed(motor1, 400);
    bdc_motor_set_speed(motor2, 400);
    bdc_motor_set_speed(motor3, 400);
    bdc_motor_set_speed(motor4, 400);
}

void motor_R_forward1(){
    bdc_motor_forward(motor2);
    bdc_motor_forward(motor3);
    bdc_motor_set_speed(motor1, 0);
    bdc_motor_set_speed(motor2, 200);
    bdc_motor_set_speed(motor3, 200);
    bdc_motor_set_speed(motor4, 0);
}

void motor_R_forward2(){
    bdc_motor_forward(motor2);
    bdc_motor_forward(motor3);
    bdc_motor_set_speed(motor1, 0);
    bdc_motor_set_speed(motor2, 400);
    bdc_motor_set_speed(motor3, 400);
    bdc_motor_set_speed(motor4, 0);
}

void motor_L_forward1(){
    bdc_motor_forward(motor1);
    bdc_motor_forward(motor4);
    bdc_motor_set_speed(motor1, 200);
    bdc_motor_set_speed(motor2, 0);
    bdc_motor_set_speed(motor3, 0);
    bdc_motor_set_speed(motor4, 200);
}

void motor_L_forward2(){
    bdc_motor_forward(motor1);
    bdc_motor_forward(motor4);
    bdc_motor_set_speed(motor1, 400);
    bdc_motor_set_speed(motor2, 0);
    bdc_motor_set_speed(motor3, 0);
    bdc_motor_set_speed(motor4, 400);
}

void motor_R_retreat1(){
    bdc_motor_reverse(motor1);
    bdc_motor_reverse(motor4);
    bdc_motor_set_speed(motor1, 200);
    bdc_motor_set_speed(motor2, 0);
    bdc_motor_set_speed(motor3, 0);
    bdc_motor_set_speed(motor4, 200);
}

void motor_R_retreat2(){
    bdc_motor_reverse(motor1);
    bdc_motor_reverse(motor4);
    bdc_motor_set_speed(motor1, 400);
    bdc_motor_set_speed(motor2, 0);
    bdc_motor_set_speed(motor3, 0);
    bdc_motor_set_speed(motor4, 400);
}

void motor_L_retreat1(){
    bdc_motor_reverse(motor2);
    bdc_motor_reverse(motor3);
    bdc_motor_set_speed(motor1, 0);
    bdc_motor_set_speed(motor2, 200);
    bdc_motor_set_speed(motor3, 200);
    bdc_motor_set_speed(motor4, 0);
}

void motor_L_retreat2(){
    bdc_motor_reverse(motor2);
    bdc_motor_reverse(motor3);
    bdc_motor_set_speed(motor1, 0);
    bdc_motor_set_speed(motor2, 400);
    bdc_motor_set_speed(motor3, 400);
    bdc_motor_set_speed(motor4, 0);
}

void motor_R_return1(){
    bdc_motor_forward(motor1);
    bdc_motor_forward(motor3);
    bdc_motor_reverse(motor2);
    bdc_motor_reverse(motor4);
    bdc_motor_set_speed(motor1, 200);
    bdc_motor_set_speed(motor2, 200);
    bdc_motor_set_speed(motor3, 200);
    bdc_motor_set_speed(motor4, 200);
}

void motor_R_return2(){
    bdc_motor_forward(motor1);
    bdc_motor_forward(motor3);
    bdc_motor_reverse(motor2);
    bdc_motor_reverse(motor4);
    bdc_motor_set_speed(motor1, 400);
    bdc_motor_set_speed(motor2, 400);
    bdc_motor_set_speed(motor3, 400);
    bdc_motor_set_speed(motor4, 400);
}

void motor_L_return1(){
    bdc_motor_forward(motor2);
    bdc_motor_forward(motor4);
    bdc_motor_reverse(motor1);
    bdc_motor_reverse(motor3);
    bdc_motor_set_speed(motor1, 200);
    bdc_motor_set_speed(motor2, 200);
    bdc_motor_set_speed(motor3, 200);
    bdc_motor_set_speed(motor4, 200);
}

void motor_L_return2(){
    bdc_motor_forward(motor2);
    bdc_motor_forward(motor4);
    bdc_motor_reverse(motor1);
    bdc_motor_reverse(motor3);
    bdc_motor_set_speed(motor1, 400);
    bdc_motor_set_speed(motor2, 400);
    bdc_motor_set_speed(motor3, 400);
    bdc_motor_set_speed(motor4, 400);
}

void motor_test1(){
    bdc_motor_forward(motor1);
    bdc_motor_forward(motor2);
    bdc_motor_forward(motor3);
    bdc_motor_forward(motor4);

    bdc_motor_set_speed(motor1, 200);
    vTaskDelay(500/portTICK_PERIOD_MS);
    bdc_motor_set_speed(motor1, 0);

    bdc_motor_set_speed(motor2, 200);
    vTaskDelay(500/portTICK_PERIOD_MS);
    bdc_motor_set_speed(motor2, 0);

    bdc_motor_set_speed(motor3, 200);
    vTaskDelay(500/portTICK_PERIOD_MS);
    bdc_motor_set_speed(motor3, 0);

    bdc_motor_set_speed(motor4, 200);
    vTaskDelay(500/portTICK_PERIOD_MS);
    bdc_motor_set_speed(motor4, 0);
}