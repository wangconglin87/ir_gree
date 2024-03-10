/*
 * 使用ESP32解析并复刻格力空调遥控器
 * 接收编码如下：
 * 引导码：  9ms低电平   + 4.5ms高电平
 * 数据码 0：660us低电平 + 540us高电平
 * 数据码 1：660us低电平 + 1680us高电平
 * 短连接码：660us低电平 + 20000us高电平
 * 长连接码：660us低电平 + 40000us高电平
 * 结束码：  660us低电平
 * 以上描述的接收编码，故发射编码正好相反
 */

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_sleep.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/rmt_types.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#include "mqtt_client.h"
#include "esp_smartconfig.h"
#include "driver/gpio.h"

#define IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us
#define IR_TX_GPIO_NUM 18

// 引导码
#define LEADING_CODE_DURATION_0 9000
#define LEADING_CODE_DURATION_1 4500

// 数据码 0
#define PAYLOAD_ZERO_DURATION_0 660
#define PAYLOAD_ZERO_DURATION_1 540

// 数据码 1
#define PAYLOAD_ONE_DURATION_0 660
#define PAYLOAD_ONE_DURATION_1 1680

// 短连接码
#define SHORT_CONNCECT_CODE_DURATION_0 660
#define SHORT_CONNCECT_CODE_DURATION_1 20000

// 长连接码
#define LONG_CONNCECT_CODE_DURATION_0 660
#define LONG_CONNCECT_CODE_DURATION_1 20000
#define LONG_CONNCECT_CODE_DURATION_3 10000

// 结束码
#define END_CODE_DURATION_0 660
#define END_CODE_DURATION_1 0x7FFF

// 一共4段数据码，第一段和第三段都是35位，其中后7位是固定的，
// 又因为乐鑫RMT驱动只能按字节编码，故后三位在编码阶段使用硬编码
typedef struct
{
    uint32_t data1;
    uint32_t data2;
    uint32_t data3;
    uint32_t data4;
} ir_gree_scan_code_t;

typedef struct
{
    uint32_t resolution;
} ir_gree_encoder_config_t;

typedef struct
{
    rmt_encoder_t base;

    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t gree_leading_symbol;
    rmt_symbol_word_t gree_ending_symbol;
    rmt_symbol_word_t short_connecting_symbol;
    // 最大间隔为32767，所以用两段编码表示
    rmt_symbol_word_t long_connecting_symbol[2];
    // 第一段和第三段数据有35位，因乐鑫rmt库只能按byte，无法按bit编码
    // 且后三位固定为010，故采用硬编码,写在copy_encoder里面
    rmt_symbol_word_t last_3_bits_symbol[3];

    rmt_encoder_t *bytes_encoder;
    int state;
} rmt_ir_gree_encoder_t;

static const char *TAG = "My RMT";
static int s_retry_num = 0;
static rmt_channel_handle_t tx_channel = NULL;
static rmt_encoder_handle_t gree_encoder = NULL;

static EventGroupHandle_t s_wifi_event_group;
static const int NETWORK_CONFIGED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;

static size_t rmt_encode_ir_gree(rmt_encoder_t *encoder,
                                 rmt_channel_handle_t channel,
                                 const void *primary_data,
                                 size_t data_size,
                                 rmt_encode_state_t *ret_stat)
{
    rmt_ir_gree_encoder_t *gree_encoder = __containerof(encoder, rmt_ir_gree_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    ir_gree_scan_code_t *scan_code = (ir_gree_scan_code_t *)primary_data;
    rmt_encoder_handle_t copy_encoder = gree_encoder->copy_encoder;
    rmt_encoder_handle_t bytes_encoder = gree_encoder->bytes_encoder;

    switch (gree_encoder->state)
    {
    // 引导码
    case 0:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &gree_encoder->gree_leading_symbol,
                                                sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 编码数据段1，调用bytes_encoder
    case 1:
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &scan_code->data1, sizeof(uint32_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 2;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 补充数据段1的后三位，调用copy_encoder,这个是数组，大小要乘以3
    case 2:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &gree_encoder->last_3_bits_symbol,
                                                3 * sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 3;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 短连接码
    case 3:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &gree_encoder->short_connecting_symbol,
                                                sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 4;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 编码数据段2，调用bytes_encoder
    case 4:
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &scan_code->data2, sizeof(uint32_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 5;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 长连接码, 数组 2倍大小
    case 5:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &gree_encoder->long_connecting_symbol,
                                                2 * sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 6;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 引导码，注意数据段3和4，不是1和2的重复，有不同内容
    case 6:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &gree_encoder->gree_leading_symbol,
                                                sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 7;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 编码数据段3，调用bytes_encoder
    case 7:
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &scan_code->data3, sizeof(uint32_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 8;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 补充数据段3的后三位，调用copy_encoder,这个是数组，大小要乘以3
    case 8:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &gree_encoder->last_3_bits_symbol,
                                                3 * sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 9;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 短连接码
    case 9:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &gree_encoder->short_connecting_symbol,
                                                sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 10;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 编码数据段4，调用bytes_encoder
    case 10:
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &scan_code->data4, sizeof(uint32_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            gree_encoder->state = 11;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        [[fallthrough]];
    // fall-through 注意这里没break
    // 结束码
    case 11:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &gree_encoder->gree_ending_symbol,
                                                sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        { // 这里跟前面的都不一样了
            gree_encoder->state = RMT_ENCODING_RESET;
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    }

out:
    *ret_stat = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_ir_gree_encoder(rmt_encoder_t *encoder)
{
    rmt_ir_gree_encoder_t *gree_encoder = __containerof(encoder, rmt_ir_gree_encoder_t, base);
    rmt_del_encoder(gree_encoder->copy_encoder);
    rmt_del_encoder(gree_encoder->bytes_encoder);
    free(gree_encoder);
    return ESP_OK;
}

static esp_err_t rmt_ir_gree_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_ir_gree_encoder_t *gree_encoder = __containerof(encoder, rmt_ir_gree_encoder_t, base);
    rmt_encoder_reset(gree_encoder->copy_encoder);
    rmt_encoder_reset(gree_encoder->bytes_encoder);
    gree_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_ir_gree_encoder(const ir_gree_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    rmt_ir_gree_encoder_t *gree_encoder = NULL;

    gree_encoder = calloc(1, sizeof(rmt_ir_gree_encoder_t));
    ESP_GOTO_ON_FALSE(gree_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for ir nec encoder");
    gree_encoder->base.encode = rmt_encode_ir_gree;
    gree_encoder->base.del = rmt_del_ir_gree_encoder;
    gree_encoder->base.reset = rmt_ir_gree_encoder_reset;

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &gree_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    // construct the leading code and ending code with RMT symbol format
    gree_encoder->gree_leading_symbol = (rmt_symbol_word_t){
        .level0 = 1,
        .duration0 = LEADING_CODE_DURATION_0,
        .level1 = 0,
        .duration1 = LEADING_CODE_DURATION_1,
    };

    gree_encoder->gree_ending_symbol = (rmt_symbol_word_t){
        .level0 = 1,
        .duration0 = END_CODE_DURATION_0,
        .level1 = 0,
        .duration1 = END_CODE_DURATION_1,
    };
    gree_encoder->short_connecting_symbol = (rmt_symbol_word_t){
        .level0 = 1,
        .duration0 = SHORT_CONNCECT_CODE_DURATION_0,
        .level1 = 0,
        .duration1 = SHORT_CONNCECT_CODE_DURATION_1,
    };
    // 15bit最大表示为32767，所以要用两段表示
    // 660us发射 + 20000us空闲 + 10000us空闲 + 10000us空闲
    gree_encoder->long_connecting_symbol[0] = (rmt_symbol_word_t){
        .level0 = 1,
        .duration0 = LONG_CONNCECT_CODE_DURATION_0,
        .level1 = 0,
        .duration1 = LONG_CONNCECT_CODE_DURATION_1,
    };
    gree_encoder->long_connecting_symbol[1] = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = LONG_CONNCECT_CODE_DURATION_3,
        .level1 = 0,
        .duration1 = LONG_CONNCECT_CODE_DURATION_3,
    };
    gree_encoder->last_3_bits_symbol[0] = (rmt_symbol_word_t){
        .level0 = 1,
        .duration0 = PAYLOAD_ZERO_DURATION_0,
        .level1 = 0,
        .duration1 = PAYLOAD_ZERO_DURATION_1,
    };
    gree_encoder->last_3_bits_symbol[1] = (rmt_symbol_word_t){
        .level0 = 1,
        .duration0 = PAYLOAD_ONE_DURATION_0,
        .level1 = 0,
        .duration1 = PAYLOAD_ONE_DURATION_1,
    };
    gree_encoder->last_3_bits_symbol[2] = (rmt_symbol_word_t){
        .level0 = 1,
        .duration0 = PAYLOAD_ZERO_DURATION_0,
        .level1 = 0,
        .duration1 = PAYLOAD_ZERO_DURATION_1,
    };

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = PAYLOAD_ZERO_DURATION_0,
            .level1 = 0,
            .duration1 = PAYLOAD_ZERO_DURATION_1,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = PAYLOAD_ONE_DURATION_0,
            .level1 = 0,
            .duration1 = PAYLOAD_ONE_DURATION_1,
        },
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &gree_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");

    *ret_encoder = &gree_encoder->base;
    return ret;
err:
    return ret;
}

void init_ir(void)
{
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .gpio_num = IR_TX_GPIO_NUM,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));

    ir_gree_encoder_config_t encoder_cfg = {
        .resolution = IR_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(rmt_new_ir_gree_encoder(&encoder_cfg, &gree_encoder));

    ESP_ERROR_CHECK(rmt_enable(tx_channel));
}

static void rmt_start()
{
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
    };
    const ir_gree_scan_code_t scan_code = {
        .data1 = 0xaaaaaaaa,
        .data2 = 0xaaaaaaaa,
        .data3 = 0xaaaaaaaa,
        .data4 = 0xaaaaaaaa,
    };
    ESP_ERROR_CHECK(rmt_transmit(tx_channel, gree_encoder, &scan_code, sizeof(scan_code), &transmit_config));
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "topic/test", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "topic/test", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        rmt_start();
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.50.229",
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
}

static void smartconfig_task(void *parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    while (1)
    {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if (uxBits & ESPTOUCH_DONE_BIT)
        {
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

static void event_handler(void *event_handler_arg,
                          esp_event_base_t event_base,
                          int32_t event_id,
                          void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        EventBits_t uxBits = xEventGroupGetBits(s_wifi_event_group);
        if (uxBits & NETWORK_CONFIGED_BIT)
        {
            ESP_ERROR_CHECK(esp_wifi_connect());
        }
        else
        {
            xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < 10)
        {
            ESP_ERROR_CHECK(esp_wifi_connect());
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            ESP_LOGI(TAG, "connect to the AP fail");
            // ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(60*1000*1000));
            esp_deep_sleep(60 * 1000 * 1000);
            s_retry_num = 0;
            ESP_ERROR_CHECK(esp_wifi_connect());
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;

        mqtt_app_start();
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD)
    {
        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true)
        {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }
        nvs_handle_t nvs_handle;
        ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "isNetConfed", 1));
        nvs_close(nvs_handle);
        xEventGroupSetBits(s_wifi_event_group, NETWORK_CONFIGED_BIT);
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        esp_wifi_connect();
    }
}

#define GPIO_INPUT_IO_0 GPIO_NUM_9
#define GPIO_INPUT_PIN_SEL 1ULL << GPIO_INPUT_IO_0
#define ESP_INTR_FLAG_DEFAULT 0

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    ESP_LOGI(TAG, "Im %lu", gpio_num);
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    uint8_t flag = 0;
    ret = nvs_get_u8(nvs_handle, "isNetConfed", &flag);
    if (ret == ESP_OK)
    {
        if (flag == 1)
        {
            xEventGroupSetBits(s_wifi_event_group, NETWORK_CONFIGED_BIT);
        }
    }
    else
    {
        xEventGroupClearBits(s_wifi_event_group, NETWORK_CONFIGED_BIT);
    }

    if (flag == 1)
    {
        // 设置恢复初始参数的按键
        ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_6, GPIO_MODE_INPUT));
        ESP_ERROR_CHECK(gpio_pullup_en(GPIO_NUM_6));
        int level = gpio_get_level(GPIO_NUM_6);
        if (level == 0)
        {
            nvs_set_u8(nvs_handle, "isNetConfed", 0);
            xEventGroupClearBits(s_wifi_event_group, NETWORK_CONFIGED_BIT);
        }
    }
    nvs_close(nvs_handle);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &event_handler,
                                               NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    EventBits_t uxBits = xEventGroupGetBits(s_wifi_event_group);
    if (uxBits & NETWORK_CONFIGED_BIT)
    {
        wifi_config_t wifi_config;
        ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &wifi_config));
    }
    ESP_ERROR_CHECK(esp_wifi_start());

    init_ir();

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << GPIO_INPUT_IO_0,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *)GPIO_INPUT_IO_0);
}