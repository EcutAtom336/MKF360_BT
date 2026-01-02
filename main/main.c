#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/errno.h>

#include "driver/gpio.h"
#include "driver/i2s_common.h"
#include "driver/i2s_std.h"
#include "driver/i2s_types.h"
#include "esp_a2dp_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_resample.h"
#include "esp_system.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include "soc/gpio_num.h"

#define MKF360_AUDIO_SAMPLE_RATE_HZ (48000U)
#define MKF360_AUDIO_SAMPLE_NUM_1MS (MKF360_AUDIO_SAMPLE_RATE_HZ / 1000U)
#define MKF360_AUDIO_SAMPLE_SIZE (2U)

#define IIS_DMA_DESC_NUM (2U)
#define IIS_DMA_ISR_INTERVAL_MS (40U)                                                 // 40ms 一次中断
#define IIS_DMA_FRAME_NUM (IIS_DMA_ISR_INTERVAL_MS * MKF360_AUDIO_SAMPLE_NUM_1MS / 2) // 双声道传输单声道数据

#define IIS_WRITER_MSG_A2D_SINK_AUDIO_STARTED (0U)
#define IIS_WRITER_MSG_A2D_SINK_AUDIO_SUSPEND (1U)
#define IIS_WRITER_MSG_HFP_AUDIO_CONNECTED (2U)
#define IIS_WRITER_MSG_HFP_AUDIO_DISCONNECTED (3U)

#define IIS_READER_MSG_HFP_AUDIO_CONNECTED (0U)
#define IIS_READER_MSG_HFP_AUDIO_DISCONNECTED (1U)

static QueueHandle_t iis_writer_task_queue_handle;
static QueueHandle_t iis_reader_task_queue_handle;

static StreamBufferHandle_t a2d_sink_hf_in_stream;
static StreamBufferHandle_t hf_out_stream;

static i2s_chan_handle_t tx_chan;
static i2s_chan_handle_t rx_chan;

static const char *conn_state_str[] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};

static void iis_writer();
static void iis_reader();
static void init();
static void nvs_init();
static void sw_init();
static void bt_init();
static void gpio_init();
static void iis_init();
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void a2d_sink_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *p_param);
static void a2d_sink_data_cb(const uint8_t *data, uint32_t len);
static void hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param);
static void hf_client_incoming_data_cb(const uint8_t *buf, uint32_t len);
static uint32_t hf_client_outgoing_data_cb(uint8_t *buf, uint32_t len);

void app_main()
{
    init();

    ESP_LOGI("APP MAIN", "Free head size: %" PRIu32, esp_get_free_heap_size());
}

static void iis_writer()
{
    const char *TAG = "iisWriter";

    uint8_t *a2d_sink_resampler_in_buffer = NULL;
    uint8_t *a2d_sink_resampler_out_buffer = NULL;
    const uint32_t a2d_sink_resampler_in_size =
        441 * 1 * 2 * 4; // (samples per 10ms) * (channel number) * (sample size) * (n)
    resample_info_t a2d_sink_resampler_info = {
        .src_rate = 44100,
        .src_ch = 1,
        .dest_rate = 48000,
        .dest_bits = 16,
        .dest_ch = 1,
        .src_bits = 16,
        .mode = RESAMPLE_DECODE_MODE,
        .max_indata_bytes = a2d_sink_resampler_in_size,
        .out_len_bytes = 0,
        .type = ESP_RESAMPLE_TYPE_AUTO,
        .complexity = 5,
        .down_ch_idx = 0,
        .prefer_flag = ESP_RSP_PREFER_TYPE_SPEED,
    };
    void *a2d_sink_resampler_handle =
        esp_resample_create(&a2d_sink_resampler_info, &a2d_sink_resampler_in_buffer, &a2d_sink_resampler_out_buffer);
    ESP_ERROR_CHECK(a2d_sink_resampler_handle == NULL ? ESP_FAIL : ESP_OK);

    uint8_t *hfp_in_resampler_in_buffer = NULL;
    uint8_t *hfp_in_resampler_out_buffer = NULL;
    const uint32_t hfp_in_resampler_in_size =
        160 * 1 * 2 * 4; // (samples per 10ms) * (channel number) * (sample size) * (n)
    resample_info_t hfp_in_resampler_info = {
        .src_rate = 16000,
        .src_ch = 1,
        .dest_rate = 48000,
        .dest_bits = 16,
        .dest_ch = 1,
        .src_bits = 16,
        .mode = RESAMPLE_DECODE_MODE,
        .max_indata_bytes = hfp_in_resampler_in_size,
        .out_len_bytes = 0,
        .type = ESP_RESAMPLE_TYPE_AUTO,
        .complexity = 5,
        .down_ch_idx = 0,
        .prefer_flag = ESP_RSP_PREFER_TYPE_SPEED,
    };
    void *hfp_in_resampler_handle =
        esp_resample_create(&hfp_in_resampler_info, &hfp_in_resampler_in_buffer, &hfp_in_resampler_out_buffer);
    ESP_ERROR_CHECK(hfp_in_resampler_handle == NULL ? ESP_FAIL : ESP_OK);

    bool tx_channel_enabled = false;
    bool a2d_sink_audio_active = false;
    bool hfp_audio_active = false;

    int sz = 0;

    uint8_t msg_val = 0;
    BaseType_t has_msg = pdFALSE;
    esp_err_t ret_esp = ESP_OK;

    uint8_t log_interval = 0;

    while (1)
    {
        if (a2d_sink_audio_active == true || hfp_audio_active == true)
        {
            has_msg = xQueueReceive(iis_writer_task_queue_handle, &msg_val, 0);
        }
        else
        {
            has_msg = xQueueReceive(iis_writer_task_queue_handle, &msg_val, portMAX_DELAY);
        }

        if (has_msg == pdTRUE)
        {
            if (msg_val == IIS_WRITER_MSG_A2D_SINK_AUDIO_STARTED)
            {
                ESP_LOGI(TAG, "Receive msg: IIS_WRITER_MSG_A2D_SINK_AUDIO_STARTED.");
                ESP_ERROR_CHECK(a2d_sink_audio_active == false ? ESP_OK : ESP_FAIL);
                a2d_sink_audio_active = true;
            }
            else if (msg_val == IIS_WRITER_MSG_A2D_SINK_AUDIO_SUSPEND)
            {
                ESP_LOGI(TAG, "Receive msg: IIS_WRITER_MSG_A2D_SINK_AUDIO_SUSPEND.");
                ESP_ERROR_CHECK(a2d_sink_audio_active == true ? ESP_OK : ESP_FAIL);
                a2d_sink_audio_active = false;
                if (xStreamBufferReset(a2d_sink_hf_in_stream) == pdPASS)
                {
                    ESP_LOGI(TAG, "Reset a2d_sink_hf_in_stream.");
                }
                else
                {
                    ESP_LOGW(TAG, "Fail to reset a2d_sink_hf_in_stream.");
                }
            }
            else if (msg_val == IIS_WRITER_MSG_HFP_AUDIO_CONNECTED)
            {
                ESP_LOGI(TAG, "Receive msg: IIS_WRITER_MSG_HFP_AUDIO_CONNECTED.");
                ESP_ERROR_CHECK(hfp_audio_active == false ? ESP_OK : ESP_FAIL);
                hfp_audio_active = true;
            }
            else if (msg_val == IIS_WRITER_MSG_HFP_AUDIO_DISCONNECTED)
            {
                ESP_LOGI(TAG, "Receive msg: IIS_WRITER_MSG_HFP_AUDIO_DISCONNECTED.");
                ESP_ERROR_CHECK(hfp_audio_active == true ? ESP_OK : ESP_FAIL);
                hfp_audio_active = false;
                if (xStreamBufferReset(a2d_sink_hf_in_stream) == pdPASS)
                {
                    ESP_LOGI(TAG, "Reset a2d_sink_hf_in_stream.");
                }
                else
                {
                    ESP_LOGW(TAG, "Fail to reset a2d_sink_hf_in_stream.");
                }
            }
            else
            {
                ESP_LOGW(TAG, "Unknow msg.");
            }

            if ((a2d_sink_audio_active == true || hfp_audio_active == true) && tx_channel_enabled == false)
            {
                ESP_LOGI(TAG, "Enable IIS tx channel.");
                tx_channel_enabled = true;
                ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
            }
            else if (a2d_sink_audio_active == false && hfp_audio_active == false && tx_channel_enabled == true)
            {
                ESP_LOGI(TAG, "Disable IIS tx channel.");
                tx_channel_enabled = false;
                ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
            }

            if (hfp_audio_active == true)
            {
                ESP_ERROR_CHECK(xStreamBufferSetTriggerLevel(a2d_sink_hf_in_stream, hfp_in_resampler_in_size) == pdTRUE
                                    ? ESP_OK
                                    : ESP_FAIL);
                ESP_LOGI(TAG, "Set a2d_sink_hf_in_stream trigger level to %" PRIu32, hfp_in_resampler_in_size);
            }
            else if (a2d_sink_audio_active == true)
            {
                ESP_ERROR_CHECK(xStreamBufferSetTriggerLevel(a2d_sink_hf_in_stream, a2d_sink_resampler_in_size) ==
                                        pdTRUE
                                    ? ESP_OK
                                    : ESP_FAIL);
                ESP_LOGI(TAG, "Set a2d_sink_hf_in_stream trigger level to %" PRIu32, a2d_sink_resampler_in_size);
            }

            continue;
        }

        if (hfp_audio_active == true)
        {
            xStreamBufferReceive(a2d_sink_hf_in_stream, hfp_in_resampler_in_buffer, hfp_in_resampler_in_size,
                                 portMAX_DELAY);
            uint32_t start = xTaskGetTickCount();
            esp_resp_err_t ret_resp =
                esp_resample_run(hfp_in_resampler_handle, &hfp_in_resampler_info, hfp_in_resampler_in_buffer,
                                 hfp_in_resampler_out_buffer, hfp_in_resampler_in_size, &sz);
            uint32_t end = xTaskGetTickCount();
            if (ret_resp != hfp_in_resampler_in_size)
            {
                ESP_LOGW(TAG, "Resample error, code: %d", ret_resp);
            }
            if (log_interval++ >= 50)
            {
                log_interval = 0;
                ESP_LOGI(TAG, "Down sample in %" PRIu32 " ticks", end - start);
            }
            ret_esp = i2s_channel_write(tx_chan, hfp_in_resampler_out_buffer, sz, NULL, 40);
            if (ret_esp != ESP_OK)
            {
                ESP_LOGE(TAG, "iis write %d bytes error, reason: %s, reset iis tx channel.", sz,
                         esp_err_to_name(ret_esp));
                ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
                ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
            }
        }
        else if (a2d_sink_audio_active == true)
        {
            xStreamBufferReceive(a2d_sink_hf_in_stream, a2d_sink_resampler_in_buffer, a2d_sink_resampler_in_size,
                                 portMAX_DELAY);
            uint32_t start = xTaskGetTickCount();
            esp_resp_err_t ret_resp =
                esp_resample_run(a2d_sink_resampler_handle, &a2d_sink_resampler_info, a2d_sink_resampler_in_buffer,
                                 a2d_sink_resampler_out_buffer, a2d_sink_resampler_in_size, &sz);
            if (ret_resp != a2d_sink_resampler_in_size)
            {
                ESP_LOGW(TAG, "Resample error, code: %d", ret_resp);
            }
            uint32_t end = xTaskGetTickCount();
            if (log_interval++ >= 50)
            {
                log_interval = 0;
                ESP_LOGI(TAG, "Resample in %" PRIu32 " ticks", end - start);
            }
            ret_esp = i2s_channel_write(tx_chan, a2d_sink_resampler_out_buffer, sz, NULL, 40);
            if (ret_esp != ESP_OK)
            {
                ESP_LOGE(TAG, "iis write %d bytes error, reason: %s, reset iis tx channel.", sz,
                         esp_err_to_name(ret_esp));
                ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
                ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
            }
        }
    }
}

static void iis_reader()
{
    const char *TAG = "iisReader";

    esp_err_t ret_esp = ESP_OK;
    esp_resp_err_t ret_resp = RESAMPLE_SUCCESS;

    uint8_t *hfp_out_resampler_in_buffer = NULL;
    uint8_t *hfp_out_resampler_out_buffer = NULL;
    const uint32_t hfp_out_resampler_in_size =
        480 * 1 * 2 * 4; // (samples per 10ms) * (channel number) * (sample size) * (n)
    resample_info_t hfp_out_resampler_info = {
        .src_rate = 48000,
        .src_ch = 1,
        .dest_rate = 16000,
        .dest_bits = 16,
        .dest_ch = 1,
        .src_bits = 16,
        .mode = RESAMPLE_DECODE_MODE,
        .max_indata_bytes = hfp_out_resampler_in_size,
        .out_len_bytes = 0,
        .type = ESP_RESAMPLE_TYPE_AUTO,
        .complexity = 5,
        .down_ch_idx = 0,
        .prefer_flag = ESP_RSP_PREFER_TYPE_SPEED,
    };
    void *hfp_out_resampler_handle =
        esp_resample_create(&hfp_out_resampler_info, &hfp_out_resampler_in_buffer, &hfp_out_resampler_out_buffer);
    ESP_ERROR_CHECK(hfp_out_resampler_handle == NULL ? ESP_FAIL : ESP_OK);

    bool hfp_audio_active = false;

    int sz = 0;

    uint8_t msg_val = 0;
    BaseType_t has_msg = pdFALSE;

    while (1)
    {
        if (hfp_audio_active == true)
        {
            has_msg = xQueueReceive(iis_reader_task_queue_handle, &msg_val, 0);
        }
        else
        {
            has_msg = xQueueReceive(iis_reader_task_queue_handle, &msg_val, portMAX_DELAY);
        }

        if (has_msg == pdTRUE)
        {
            if (msg_val == IIS_READER_MSG_HFP_AUDIO_CONNECTED)
            {
                ESP_ERROR_CHECK(hfp_audio_active == false ? ESP_OK : ESP_FAIL);
                hfp_audio_active = true;
                ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
            }
            else if (msg_val == IIS_READER_MSG_HFP_AUDIO_DISCONNECTED)
            {
                ESP_ERROR_CHECK(hfp_audio_active == true ? ESP_OK : ESP_FAIL);
                hfp_audio_active = false;
                ESP_ERROR_CHECK(i2s_channel_disable(rx_chan));
            }
            else
            {
                ESP_LOGW(TAG, "Unknow msg.");
            }
            continue;
        }

        if (hfp_audio_active == true)
        {
            size_t recv_size = 0;
            ret_esp = i2s_channel_read(rx_chan, hfp_out_resampler_in_buffer, hfp_out_resampler_in_size, &recv_size,
                                       portMAX_DELAY);
            if (ret_esp != ESP_OK)
            {
                ESP_LOGW(TAG, "iis read error, code: %d", ret_esp);
            }
            ret_resp = esp_resample_run(hfp_out_resampler_handle, &hfp_out_resampler_info, hfp_out_resampler_in_buffer,
                                        hfp_out_resampler_out_buffer, recv_size, &sz);
            if (ret_resp <= 0)
            {
                ESP_LOGW(TAG, "Resample error, code: %d", ret_resp);
            }
            size_t saved_size = xStreamBufferSend(hf_out_stream, hfp_out_resampler_out_buffer, sz, 40);
            if (saved_size != sz)
            {
                ESP_LOGW(TAG, "Data size: %d, saved size: %zu", sz, saved_size);
            }
            esp_hf_client_outgoing_data_ready();
        }
    }
}

static void init()
{
    nvs_init();
    sw_init();
    bt_init();
    gpio_init();
    iis_init();
}

static void nvs_init()
{
    esp_err_t ret_esp = ESP_OK;

    ret_esp = nvs_flash_init();
    if (ret_esp == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret_esp = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret_esp);
}

static void sw_init()
{
    // (samples per 10ms) * (channel number) * (sample size) * (n)

    a2d_sink_hf_in_stream = xStreamBufferCreate(480 * 1 * 2 * 20, 1);
    ESP_ERROR_CHECK(a2d_sink_hf_in_stream == NULL ? ESP_FAIL : ESP_OK);

    hf_out_stream = xStreamBufferCreate(160 * 1 * 2 * 20, 1);
    ESP_ERROR_CHECK(hf_out_stream == NULL ? ESP_FAIL : ESP_OK);

    iis_writer_task_queue_handle = xQueueCreate(8, sizeof(uint8_t));
    ESP_ERROR_CHECK(iis_writer_task_queue_handle == NULL ? ESP_FAIL : ESP_OK);

    iis_reader_task_queue_handle = xQueueCreate(8, sizeof(uint8_t));
    ESP_ERROR_CHECK(iis_reader_task_queue_handle == NULL ? ESP_FAIL : ESP_OK);

    ESP_ERROR_CHECK(xTaskCreate(iis_writer, "iisWriter", 4096, NULL, 20, NULL) == pdPASS ? ESP_OK : ESP_FAIL);

    ESP_ERROR_CHECK(xTaskCreate(iis_reader, "iisReader", 4096, NULL, 20, NULL) == pdPASS ? ESP_OK : ESP_FAIL);
}

static void bt_init()
{
    // Generate a bluetooth device name(maybe repeat)
    uint8_t factory_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(&factory_mac[0], ESP_MAC_EFUSE_FACTORY));
    char device_name[] = "MKF360-XXXXXX";
    snprintf(device_name, sizeof(device_name), "MKF360-%02X%02X%02X", factory_mac[3], factory_mac[4], factory_mac[5]);

    // Base
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(device_name));

    // GAP
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    ESP_ERROR_CHECK(esp_bt_gap_set_pin(pin_type, 0, pin_code));
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));

    // A2D
    ESP_ERROR_CHECK(esp_a2d_sink_init());
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(a2d_sink_data_cb));
    ESP_ERROR_CHECK(esp_a2d_register_callback(a2d_sink_cb));

    // HFP
    ESP_ERROR_CHECK(esp_hf_client_register_callback(hf_client_cb));
    ESP_ERROR_CHECK(esp_hf_client_init());
    ESP_ERROR_CHECK(esp_hf_client_register_data_callback(hf_client_incoming_data_cb, hf_client_outgoing_data_cb));

    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "GAP CB";

    const char *event_name[] = {
        "ESP_BT_GAP_DISC_RES_EVT",
        "ESP_BT_GAP_DISC_STATE_CHANGED_EVT",
        "ESP_BT_GAP_RMT_SRVCS_EVT",
        "ESP_BT_GAP_RMT_SRVC_REC_EVT",
        "ESP_BT_GAP_AUTH_CMPL_EVT",
        "ESP_BT_GAP_PIN_REQ_EVT",
        "ESP_BT_GAP_CFM_REQ_EVT",
        "ESP_BT_GAP_KEY_NOTIF_EVT",
        "ESP_BT_GAP_KEY_REQ_EVT",
        "ESP_BT_GAP_READ_RSSI_DELTA_EVT",
        "ESP_BT_GAP_CONFIG_EIR_DATA_EVT",
        "ESP_BT_GAP_SET_AFH_CHANNELS_EVT",
        "ESP_BT_GAP_READ_REMOTE_NAME_EVT",
        "ESP_BT_GAP_MODE_CHG_EVT",
        "ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT",
        "ESP_BT_GAP_QOS_CMPL_EVT",
        "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT",
        "ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT",
        "ESP_BT_GAP_SET_PAGE_TO_EVT",
        "ESP_BT_GAP_GET_PAGE_TO_EVT",
        "ESP_BT_GAP_ACL_PKT_TYPE_CHANGED_EVT",
        "ESP_BT_GAP_ENC_CHG_EVT",
        "ESP_BT_GAP_SET_MIN_ENC_KEY_SIZE_EVT",
        "ESP_BT_GAP_GET_DEV_NAME_CMPL_EVT",
        "ESP_BT_GAP_EVT_MAX",
    };
    ESP_LOGI(TAG, "%s", event_name[event]);

    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        }
        else
        {
            ESP_LOGI(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit)
        {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        }
        else
        {
            ESP_LOGI(TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT: {
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_12, 1));
        break;
    }
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT: {
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_12, 0));
        break;
    }
    default: {
        break;
    }
    }
}

static void gpio_init()
{
    esp_err_t ret_esp = ESP_OK;

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << GPIO_NUM_12),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret_esp = gpio_config(&gpio_cfg);
    ESP_ERROR_CHECK(ret_esp);
    ret_esp = gpio_set_level(GPIO_NUM_12, 0);
    ESP_ERROR_CHECK(ret_esp);
}

static void iis_init()
{
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_AUTO,
        .role = I2S_ROLE_SLAVE,
        .dma_desc_num = IIS_DMA_DESC_NUM,
        .dma_frame_num = IIS_DMA_FRAME_NUM,
        .auto_clear_after_cb = false,
        .auto_clear_before_cb = true,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan));

    // 使用两个声道传输一个声道的数据，
    // 具体数据格式查看主控程序 audio_iis.c
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MKF360_AUDIO_SAMPLE_RATE_HZ / 2),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = GPIO_NUM_NC,
                .bclk = GPIO_NUM_0,
                .ws = GPIO_NUM_4,
                .dout = GPIO_NUM_13,
                .din = GPIO_NUM_15,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
}

static void a2d_sink_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *p_param)
{
    const char *TAG = "A2D CB";
    const char *event_name[] = {
        "ESP_A2D_CONNECTION_STATE_EVT",    "ESP_A2D_AUDIO_STATE_EVT",         "ESP_A2D_AUDIO_CFG_EVT",
        "ESP_A2D_MEDIA_CTRL_ACK_EVT",      "ESP_A2D_PROF_STATE_EVT",          "ESP_A2D_SNK_PSC_CFG_EVT",
        "ESP_A2D_SNK_SET_DELAY_VALUE_EVT", "ESP_A2D_SNK_GET_DELAY_VALUE_EVT", "ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT",
    };
    ESP_LOGI(TAG, "event: %s", event_name[event]);

    esp_a2d_cb_param_t *a2d = NULL;
    switch (event)
    {
    case ESP_A2D_CONNECTION_STATE_EVT:
        a2d = (esp_a2d_cb_param_t *)(p_param);
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (a2d->audio_stat.state == ESP_A2D_AUDIO_STATE_SUSPEND)
        {
            ESP_LOGI(TAG, "ESP_A2D_AUDIO_STATE_SUSPEND");
            xQueueSend(iis_writer_task_queue_handle, &((uint8_t){IIS_WRITER_MSG_A2D_SINK_AUDIO_SUSPEND}), 0);
        }
        else if (a2d->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED)
        {
            ESP_LOGI(TAG, "ESP_A2D_AUDIO_STATE_STARTED");
            xQueueSend(iis_writer_task_queue_handle, &((uint8_t){IIS_WRITER_MSG_A2D_SINK_AUDIO_STARTED}), 0);
        }
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGD(TAG, "A2DP audio stream configuration, codec type %d", a2d->audio_cfg.mcc.type);
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC)
        {
            int sample_rate = 16000;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6))
            {
                sample_rate = 32000;
            }
            else if (oct0 & (0x01 << 5))
            {
                sample_rate = 44100;
            }
            else if (oct0 & (0x01 << 4))
            {
                sample_rate = 48000;
            }
            ESP_LOGD(TAG, "Bluetooth configured, sample rate=%d", sample_rate);
        }
        break;
    default:
        ESP_LOGD(TAG, "Unhandled A2DP event: %d", event);
        break;
    }
}

static void a2d_sink_data_cb(const uint8_t *data, uint32_t len)
{
    const char *TAG = "A2D SINK CB";
    if (xStreamBufferSpacesAvailable(a2d_sink_hf_in_stream) < len / 2)
    {
        ESP_LOGW(TAG, "a2d_sink_hf_in_stream space not enough, drop data.");
        return;
    }
    for (size_t i = 0; i < len / sizeof(int16_t); i++)
    {
        ((int16_t *)data)[i] = ((int16_t *)data)[i * 2];
    }
    uint32_t sent_size = xStreamBufferSend(a2d_sink_hf_in_stream, data, len / 2, 0);
    ESP_ERROR_CHECK(sent_size == len / 2 ? ESP_OK : ESP_FAIL);
}

static void hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    const char *TAG = "HF CLIENT CB";
    const char *event_name[] = {
        "ESP_HF_CLIENT_CONNECTION_STATE_EVT",
        "ESP_HF_CLIENT_AUDIO_STATE_EVT",
        "ESP_HF_CLIENT_BVRA_EVT",
        "ESP_HF_CLIENT_CIND_CALL_EVT",
        "ESP_HF_CLIENT_CIND_CALL_SETUP_EVT",
        "ESP_HF_CLIENT_CIND_CALL_HELD_EVT",
        "ESP_HF_CLIENT_CIND_SERVICE_AVAILABILITY_EVT",
        "ESP_HF_CLIENT_CIND_SIGNAL_STRENGTH_EVT",
        "ESP_HF_CLIENT_CIND_ROAMING_STATUS_EVT",
        "ESP_HF_CLIENT_CIND_BATTERY_LEVEL_EVT",
        "ESP_HF_CLIENT_COPS_CURRENT_OPERATOR_EVT",
        "ESP_HF_CLIENT_BTRH_EVT",
        "ESP_HF_CLIENT_CLIP_EVT",
        "ESP_HF_CLIENT_CCWA_EVT",
        "ESP_HF_CLIENT_CLCC_EVT",
        "ESP_HF_CLIENT_VOLUME_CONTROL_EVT",
        "ESP_HF_CLIENT_AT_RESPONSE_EVT",
        "ESP_HF_CLIENT_CNUM_EVT",
        "ESP_HF_CLIENT_BSIR_EVT",
        "ESP_HF_CLIENT_BINP_EVT",
        "ESP_HF_CLIENT_RING_IND_EVT",
        "ESP_HF_CLIENT_PKT_STAT_NUMS_GET_EVT",
    };
    ESP_LOGI(TAG, "event: %s", event_name[event]);

    switch (event)
    {
    case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC)
        {
            ESP_LOGI(TAG, "ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC");
            xQueueSend(iis_writer_task_queue_handle, &((uint8_t){IIS_WRITER_MSG_HFP_AUDIO_CONNECTED}), 0);
            xQueueSend(iis_reader_task_queue_handle, &((uint8_t){IIS_READER_MSG_HFP_AUDIO_CONNECTED}), 0);
        }
        else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED)
        {
            ESP_LOGI(TAG, "ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED");
            xQueueSend(iis_writer_task_queue_handle, &((uint8_t){IIS_WRITER_MSG_HFP_AUDIO_DISCONNECTED}), 0);
            xQueueSend(iis_reader_task_queue_handle, &((uint8_t){IIS_READER_MSG_HFP_AUDIO_DISCONNECTED}), 0);
        }
        break;
    default:
        break;
    }
}

static void hf_client_incoming_data_cb(const uint8_t *buf, uint32_t len)
{
    const char *TAG = "HFP IN CB";
    uint32_t saved_size = xStreamBufferSend(a2d_sink_hf_in_stream, buf, len, 0);
    if (saved_size != len)
    {
        ESP_LOGW(TAG, "Saved size: %" PRIu32 ", data size: %" PRIu32, saved_size, len);
    }
}

static uint32_t hf_client_outgoing_data_cb(uint8_t *buf, uint32_t len)
{
    const char *TAG = "HFP OUT CB";
    uint32_t available_size = xStreamBufferBytesAvailable(hf_out_stream);
    if (available_size < len)
    {
        return 0;
    }
    if (available_size - len >= len)
    {
        esp_hf_client_outgoing_data_ready();
    }
    xStreamBufferReceive(hf_out_stream, buf, len, 0);
    return len;
}
