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
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "esp_check.h"

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
static uint32_t data1 = 0;
static uint32_t data2 = 0;
static uint32_t data3 = 0;
static uint32_t data4 = 0;

static const char *TAG = "My RMT";

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

    // fall-through 注意这里没break
    // 结束码
    case 11:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                &gree_encoder->gree_ending_symbol,
                                                sizeof(rmt_symbol_word_t),
                                                &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {   // 这里跟前面的都不一样了
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

void app_main(void)
{
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .gpio_num = IR_TX_GPIO_NUM,
    };
    rmt_channel_handle_t tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0, // no loop
    };

    ir_gree_encoder_config_t encoder_cfg = {
        .resolution = IR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t gree_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_ir_gree_encoder(&encoder_cfg, &gree_encoder));

    ESP_ERROR_CHECK(rmt_enable(tx_channel));

    while (1)
    {
        // 阻塞5s
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        const ir_gree_scan_code_t scan_code = {
            .data1 = 0xaaaaaaaa,
            .data2 = 0xaaaaaaaa,
            .data3 = 0xaaaaaaaa,
            .data4 = 0xaaaaaaaa,
        };
        ESP_ERROR_CHECK(rmt_transmit(tx_channel, gree_encoder, &scan_code, sizeof(scan_code), &transmit_config));
    }
}