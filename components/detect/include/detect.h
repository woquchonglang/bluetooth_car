#ifndef __DETECT_H__
#define __DETECT_H__

#include "bdc_motor.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"



void detect_gpio_init();

void Detect_mode();

#endif


