#include "driver/mcpwm_prelude.h"
#include "freertos/FREERTOS.h"
#include "freertos/task.h"
// #define SERVO_GPIO_1        8 
// #define SERVO_GPIO_2        9
// #define SERVO_GPIO_3        10



// #define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
// #define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
// #define SERVO_MIN_DEGREE        -180   // Minimum angle
// #define SERVO_MAX_DEGREE        180    // Maximum angle

// #define SERVO_PULSE_GPIO             0        // GPIO connects to the PWM signal line
// #define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
// #define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

// static inline uint32_t angle_to_compare(int angle)
// {
//     return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
// }

void servo_init();
void servo1_up_1();
void servo1_up_2();
void servo1_down_1();
void servo1_down_2();
void servo2_up_1();
void servo2_up_2();
void servo2_down_1();
void servo2_down_2();
void servo_claw_open();
void servo_claw_close();

