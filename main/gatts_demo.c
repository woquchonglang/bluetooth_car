#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "bdc_motor.h"
#include "detect.h"
//#include "BLE.h"
#include "Servo.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define GATTS_TAG "GATTS_DEMO"

#define SERVO_GPIO_1        8 
#define SERVO_GPIO_2        9
#define SERVO_GPIO_3        10

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -180   // Minimum angle
#define SERVO_MAX_DEGREE        180    // Maximum angle

static inline uint32_t angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

static int angle1 = 140;
static int angle2 = 0;
static int angle3 = (0);
static int bi_state = 0;

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -180   // Minimum angle
#define SERVO_MAX_DEGREE        180    // Maximum angle

#define SERVO_PULSE_GPIO             0        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms


mcpwm_timer_handle_t timer1 = NULL;
mcpwm_timer_handle_t timer2 = NULL;
mcpwm_timer_handle_t timer3 = NULL;


mcpwm_oper_handle_t oper1 = NULL;
mcpwm_oper_handle_t oper2 = NULL;
mcpwm_oper_handle_t oper3 = NULL;


static mcpwm_cmpr_handle_t comp1 = NULL;
static mcpwm_cmpr_handle_t comp2 = NULL;
static mcpwm_cmpr_handle_t comp3 = NULL;


mcpwm_gen_handle_t gen1 = NULL;
mcpwm_gen_handle_t gen2 = NULL;
mcpwm_gen_handle_t gen3 = NULL;

mcpwm_timer_config_t timer_config1 = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
    .period_ticks = SERVO_TIMEBASE_PERIOD,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
};
mcpwm_timer_config_t timer_config2 = {
    .group_id = 1,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
    .period_ticks = SERVO_TIMEBASE_PERIOD,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
};
mcpwm_timer_config_t timer_config3 = {
    .group_id = 1,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
    .period_ticks = SERVO_TIMEBASE_PERIOD,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
};

mcpwm_operator_config_t operator_config1 = {
    .group_id = 0, // operator must be in the same group to the timer
};
mcpwm_operator_config_t operator_config2 = {
    .group_id = 1, // operator must be in the same group to the timer
    // .flags = update_gen_action_on_tep;
};
mcpwm_operator_config_t operator_config3 = {
    .group_id = 1   , // operator must be in the same group to the timer
};

mcpwm_comparator_config_t comparator_config1 = {
    .flags.update_cmp_on_tez = true,
};
mcpwm_comparator_config_t comparator_config2 = {
    .flags.update_cmp_on_tez = true,
};
mcpwm_comparator_config_t comparator_config3 = {
    .flags.update_cmp_on_tez = true,
};

mcpwm_generator_config_t generator_config1 = {
    .gen_gpio_num = SERVO_GPIO_1,
};
mcpwm_generator_config_t generator_config2 = {
    .gen_gpio_num = SERVO_GPIO_2,
};
mcpwm_generator_config_t generator_config3 = {
    .gen_gpio_num = SERVO_GPIO_3,
};

void servo_init()
{
    mcpwm_new_timer(&timer_config1, &timer1);
    mcpwm_new_timer(&timer_config2, &timer2);
    mcpwm_new_timer(&timer_config3, &timer3);

    mcpwm_new_operator(&operator_config1, &oper1);
    mcpwm_new_operator(&operator_config2, &oper2);
    mcpwm_new_operator(&operator_config3, &oper3);

    mcpwm_operator_connect_timer(oper1, timer1);
    mcpwm_operator_connect_timer(oper2, timer2);
    mcpwm_operator_connect_timer(oper3, timer3);

    mcpwm_new_comparator(oper1, &comparator_config1, &comp1);
    mcpwm_new_comparator(oper2, &comparator_config2, &comp2);
    mcpwm_new_comparator(oper3, &comparator_config3, &comp3);

    mcpwm_new_generator(oper1, &generator_config1, &gen1);
    mcpwm_new_generator(oper2, &generator_config2, &gen2);
    mcpwm_new_generator(oper3, &generator_config3, &gen3);

    mcpwm_comparator_set_compare_value(comp1, angle_to_compare(angle1));
    mcpwm_comparator_set_compare_value(comp2, angle_to_compare(0));
    mcpwm_comparator_set_compare_value(comp3, angle_to_compare(-90));

    mcpwm_generator_set_action_on_timer_event(gen1,
                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, 
                                              MCPWM_TIMER_EVENT_EMPTY, 
                                              MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen1,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comp1, MCPWM_GEN_ACTION_LOW));
    

    mcpwm_generator_set_action_on_timer_event(gen2,
                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, 
                                              MCPWM_TIMER_EVENT_EMPTY, 
                                              MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen2,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comp2, MCPWM_GEN_ACTION_LOW));


    mcpwm_generator_set_action_on_timer_event(gen3,
                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, 
                                              MCPWM_TIMER_EVENT_EMPTY, 
                                              MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen3,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comp3, MCPWM_GEN_ACTION_LOW));

                    
    mcpwm_timer_enable(timer1);
    mcpwm_timer_enable(timer2);
    mcpwm_timer_enable(timer3);

    mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP);
    mcpwm_timer_start_stop(timer2, MCPWM_TIMER_START_NO_STOP);
    mcpwm_timer_start_stop(timer3, MCPWM_TIMER_START_NO_STOP);

    // mcpwm_cmpr_handle_t cb_h = NULL;
    // const mcpwm_comparator_event_callbacks_t  cb_t;
    
    // mcpwm_comparator_register_event_callbacks(cb_h,&cb_t,(void *)angle1);
}


void servo1_up_1(){
    angle1 -= 1;
    mcpwm_comparator_set_compare_value(comp1, angle_to_compare(angle1));
    // ESP_LOGI(GATTS_TAG,"%ld",angle_to_compare(angle1));
}
void servo1_up_2(){
    // mcpwm_comparator_set_compare_value(comp1, 1700);

    angle1 -= 4;
    mcpwm_comparator_set_compare_value(comp1, angle_to_compare(angle1));
}
void servo1_down_1(){
   angle1 += 1;
    mcpwm_comparator_set_compare_value(comp1, angle_to_compare(angle1));
    // mcpwm_comparator_set_compare_value(comp1, 1480);
}
void servo1_down_2(){
    mcpwm_comparator_set_compare_value(comp1, 1200);
    angle1 += 2;
    mcpwm_comparator_set_compare_value(comp1, angle_to_compare(angle1));
}


void servo2_up_1(){
    // mcpwm_comparator_set_compare_value(comp2, 1600);
    angle2 += 2;
    mcpwm_comparator_set_compare_value(comp2, angle_to_compare(angle2));
    ESP_LOGI(GATTS_TAG,"%ld",angle_to_compare(angle2));
}
void servo2_up_2(){
    // mcpwm_comparator_set_compare_value(comp2, 2000);
    angle2 += 2;
    mcpwm_comparator_set_compare_value(comp2, angle_to_compare(angle2));
    ESP_LOGI(GATTS_TAG,"%ld",angle_to_compare(angle2));
}
void servo2_down_1(){
    // mcpwm_comparator_set_compare_value(comp2, 1200);
    angle2 -=1;
    mcpwm_comparator_set_compare_value(comp2, angle_to_compare(angle2));
    ESP_LOGI(GATTS_TAG,"%ld",angle_to_compare(angle2));
}
void servo2_down_2(){
    // mcpwm_comparator_set_compare_value(comp2, 500);
     angle2 -=2;
    mcpwm_comparator_set_compare_value(comp2, angle_to_compare(angle2));
    // ESP_LOGI(GATTS_TAG,"%ld",angle_to_compare(angle2));
}

void servo_claw_open(){
   mcpwm_comparator_set_compare_value(comp3, angle_to_compare(-70));

}

void servo_claw_close(){
    mcpwm_comparator_set_compare_value(comp3, angle_to_compare(90));
}




static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

TaskFunction_t DETECT_Task;
TaskHandle_t xOpenDetect_Handle = NULL;
static uint32_t detect_state = 0;


#define TEST_DEVICE_NAME            "ESP_YJY"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;
// static esp_gatt_char_prop_t b_property = 0;

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f
};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;
// static prepare_type_env_t b_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        } else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret){
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret){
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        //ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            //ESP_LOGI(GATTS_TAG, " value :");
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
            //esp_log_buffer_char(GATTS_TAG, param->write.value, param->write.len);
            //ESP_LOGI(GATTS_TAG,"value:%p", param->write.value);
            if(param->write.value[4] == 0x00 && param->write.value[7] == 0x00 ){
                     motor_stop();
                     //mcpwm_comparator_set_compare_value(comp2, angle_to_compare(0));

                     if(detect_state == 1){
                        detect_state = 0;
                        vTaskSuspend(xOpenDetect_Handle);
                        // xTaskNotify(xOpenDetect_Handle, detect_state, eSetBits);

                        // ESP_LOGE(GATTS_TAG,"detect_state = %ld", detect_state);
                     }
                     if(bi_state == 1){
                        bi_state = 0;
                        mcpwm_comparator_set_compare_value(comp3, angle_to_compare(-45));
                        // xTaskNotify(xOpenDetect_Handle, detect_state, eSetBits);

                        // ESP_LOGE(GATTS_TAG,"detect_state = %ld", detect_state);
                     }
            }

            else if(param->write.value[1] == 0x02){         //夹取模式
                


                if(bi_state == 0){
                    bi_state = 1;
                    servo_claw_close();
                }

                if (param->write.value[4] == 0x00 && param->write.value[7] == 0x02 )
                {
                    motor_stop();
                    //mcpwm_comparator_set_compare_value(comp2, angle_to_compare(0));

                }
                
                else if(param->write.value[3] == 0x00 && param->write.value[4] != 0x00 && param->write.value[7] != 0x00){
                    if(param->write.value[4] == 0x01){
                        motor_forward1();

                    }
                    else if(param->write.value[4] == 0x02 ){
                        motor_forward2();

                    }
                    else if(param->write.value[4] == 0xff){
                        motor_retreat1();

                    }
                    else if(param->write.value[4] == 0xfe){
                        motor_retreat2();
                    }
                }

                else if(param->write.value[3] != 0x00 && param->write.value[4] == 0x00 && param->write.value[7] != 0x00){
                    if(param->write.value[3] == 0x01){
                        motor_R_1();

                    }
                    else if(param->write.value[3] == 0x02){
                        motor_R_2();

                    }
                    else if(param->write.value[3] == 0xff){
                        motor_L_1();

                    }

                }

                else if(param->write.value[3] != 0x00 && param->write.value[4] != 0x00 && param->write.value[7] == 0x02){
                    if(param->write.value[3] == 0xff){
                        motor_L_forward1();

                    }
                    else if(param->write.value[3] == 0xfe){
                        motor_L_forward2();

                    }
                    else if(param->write.value[3] == 0x01){
                        motor_R_retreat1();

                    }
                    else if(param->write.value[3] == 0x02){
                        motor_R_retreat2();

                    }
                    
                } 

                else if(param->write.value[3] != 0x00 && param->write.value[4] != 0x00){
                    if(param->write.value[4] == 0xff){
                        motor_L_retreat1();
                    }
                    else if(param->write.value[4] == 0xfe){
                        motor_L_retreat2();
                    }
                    else if(param->write.value[4] == 0x01){
                        motor_R_forward1();
                    }
                    else if(param->write.value[4] == 0x02){
                        motor_R_forward2();
                    } 
                }      

                else if(param->write.value[2] != 0x00){
                    if(param->write.value[2] == 0xff){
                        motor_L_return1();
                    }
                    else if(param->write.value[2] == 0xfe){
                        motor_L_return2();

                    }
                    else if( param->write.value[2] == 0x01){
                        motor_R_return1();

                    }
                    else if( param->write.value[2] == 0x02){
                        motor_R_return2();
                    }
                }

                else if(param->write.value[5] != 0x00){
                    if(param->write.value[5] == 0xff){
                        servo1_down_1();
                    }
                    else if(param->write.value[5] == 0xfe){
                        servo1_down_2();
                    }
                    else if( param->write.value[5] == 0x01){
                        servo1_up_1();
                    }
                    else if( param->write.value[5] == 0x02){
                        servo1_up_2();
                    }
                    
                }
                
                else if(param->write.value[6] != 0x00){
                    if(param->write.value[6] == 0xff){
                        servo2_down_1();
                    }
                    else if(param->write.value[6] == 0xfe){
                        servo2_down_2();
                    }
                    else if( param->write.value[6] == 0x01){
                        servo2_up_1();
                    }
                    else if( param->write.value[6] == 0x02){
                        servo2_up_2();
                    }
                
                }

                

            }





            else if(param->write.value[3] == 0x00 && param->write.value[4] != 0x00 && param->write.value[7] != 0x00){
                if(param->write.value[7] == 0x01){
                    motor_forward1();

                }
                else if(param->write.value[7] == 0x02 ){
                    motor_forward2();

                }
                else if(param->write.value[7] == 0xff){
                    motor_retreat1();

                }
                else if(param->write.value[7] == 0xfe){
                    motor_retreat2();
                }
            }

            else if(param->write.value[3] != 0x00 && param->write.value[4] == 0x00 && param->write.value[7] != 0x00){
                if(param->write.value[3] == 0x01){
                    motor_R_1();

                }
                else if(param->write.value[3] == 0x02){
                    motor_R_2();

                }
                else if(param->write.value[3] == 0xff){
                    motor_L_1();

                }
                else if(param->write.value[3] == 0xfe){
                    motor_L_2();

                }
            }

            else if(param->write.value[3] != 0x00 && param->write.value[4] != 0x00 && param->write.value[7] == 0x00){
                if(param->write.value[3] == 0xff){
                    motor_L_forward1();

                }
                else if(param->write.value[3] == 0xfe){
                    motor_L_forward2();

                }
                else if(param->write.value[3] == 0x01){
                    motor_R_retreat1();

                }
                else if(param->write.value[3] == 0x02){
                    motor_R_retreat2();

                }
                
            }

            else if(param->write.value[4] != 0x00 && param->write.value[7] != 0x00){
                if(param->write.value[4] == 0xff){
                    motor_L_retreat1();
                }
                else if(param->write.value[4] == 0xfe){
                    motor_L_retreat2();
                }
                else if(param->write.value[4] == 0x01){
                    motor_R_forward1();
                }
                else if(param->write.value[4] == 0x02){
                    motor_R_forward2();
                }
            }

            else if(param->write.value[2] != 0x00){
                if(param->write.value[2] == 0xff){
                    motor_L_return1();
                }
                else if(param->write.value[2] == 0xfe){
                    motor_L_return2();

                }
                else if( param->write.value[2] == 0x01){
                    motor_R_return1();

                }
                else if( param->write.value[2] == 0x02){
                    motor_R_return2();
                }
                
            }

            else if(param->write.value[5] != 0x00){
                if(param->write.value[5] == 0xff){
                    servo1_down_1();
                }
                else if(param->write.value[5] == 0xfe){
                    servo1_down_2();
                }
                else if( param->write.value[5] == 0x01){
                    servo1_up_1();
                }
                else if( param->write.value[5] == 0x02){
                    servo1_up_2();
                }
                
            }

            else if(param->write.value[6] != 0x00){
                if(param->write.value[6] == 0xff){
                    servo2_down_1();

                }
                else if(param->write.value[6] == 0xfe){
                    servo2_down_2();
                }
                else if( param->write.value[6] == 0x01){
                    servo2_up_1();

                    mcpwm_comparator_set_compare_value(comp2, angle_to_compare(angle2));
                }
                else if( param->write.value[6] == 0x02){
                    servo2_up_2();
                }
                
            }
            
            else if(param->write.value[1] == 0x01){
                vTaskResume(xOpenDetect_Handle);
                detect_state = 1;
            }




        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    // case ESP_GATTS_EXEC_WRITE_EVT:
    //     ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
    //     esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    //     example_exec_write_event_env(&a_prepare_write_env, param);
    //     break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    // case ESP_GATTS_UNREG_EVT:
    //     break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}





void app_main(void)
{

    nvs_flash_init();

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    motor_init();

    detect_gpio_init();

    xTaskCreate(Detect_mode, "detect", 2048, NULL, 5, &xOpenDetect_Handle);
    vTaskSuspend(xOpenDetect_Handle);

    servo_init();

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    esp_bt_controller_init(&bt_cfg);

    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_bluedroid_init();

    esp_bluedroid_enable();

    esp_ble_gatts_register_callback(gatts_event_handler);

    esp_ble_gap_register_callback(gap_event_handler);

    esp_ble_gatts_app_register(PROFILE_A_APP_ID);

    esp_ble_gatt_set_local_mtu(500);


    return;
}
