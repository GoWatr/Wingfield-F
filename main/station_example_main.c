#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_task_wdt.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "cJSON.h"

#include "board.h"
#include "transducer.h"
#include "spi.h"

// #define configCHECK_FOR_STACK_OVERFLOW 1
#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

//FIRMWARE UPDATE URL
#define FLOWMETER_ID CONFIG_FLOW_METER_ID
#define UPDATE_JSON_URL CONFIG_JSON_URL

//FETCH VALVE STATUS URL 
#define DOOR_NO CONFIG_DOOR_NO
#define VSTATUS_URL CONFIG_VS_FETCH_URL

#define FIRMWARE_VERSION 1.0
#define LED_GPIO 2
#define WEB_SERVER "gowatr.com"
#define WEB_PORT "80"
#define WEB_PATH "/"

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define duration_ota_update (1 * 120)
#define duration_wifi_reconnect (1 * 40)
#define duration_write_flash (5 * 60)

static int s_retry_num = 0;
static int wifi_counter = 0;
static int ota_update_counter = 0;
static int flash_counter = 0;
int32_t Totalizer = 0;
int32_t tot_in_flash = 0;
char rcv_buffer[200];
char REQUEST[256];

#define CUBIC_METERS_TO_GALLONS 264.172
#define CUBIC_METERS_TO_LITERS 1000
#define TIMER_DIVIDER (16)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)

static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle s_timer_queue;

uint32_t Counter;
uint32_t http_get_timer;
uint8_t get_request_data = 0;
max3510x_tof_results_t s_tof_results;
static volatile uint32_t s_results_count;
static volatile uint32_t s_timeout_count;

uint8_t c = 0;
uint8_t enable_sampling = 0;
uint32_t ts, last_ts, ts_temp = 0;

uint8_t interrupt_sts = 0;
uint16_t IntrStatus = 0;

uint8_t n;
double flowrate;
double_t flow_check;
double_t last_flow = 0.0, sum_flow = 0, flow;
uint8_t projid[32] = "wingfield";
int prev_valve_value = -1;
int valve_value = 1;

#define MOTOR_BLUE_PIN 26
#define MOTOR_YELLOW_PIN 27
#define MOTOR_IN1 14
#define MOTOR_IN2 12
uint8_t bluepin_state, yellowpin_state;
// uint8_t door[32] = "1A";
// #define URL "https://www.gowatr.com/wms/getdata/wingfield_getval.php?door_no=1A"
// #define URL "https://www.gowatr.com/wms/getdata/wingfield_getval.php?door_no="DOOR_NO
// static const char *TAG_JSON = "JSON_HTTP";
void fetch_and_parse_json(void);

typedef struct
{
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

/**
 * @brief A sample structure to pass events from the timer ISR to task
 */
typedef struct
{
    example_timer_info_t info;
    uint64_t timer_counter_value;
} example_timer_event_t;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        gpio_set_level(LED_GPIO, 0);
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        { //
            esp_wifi_connect();
            s_retry_num++; //
            ESP_LOGI("event_handler", "retry to connect to the AP");
        }
        else
        {
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); //
            gpio_set_level(LED_GPIO, 0);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("event_handler", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        gpio_set_level(LED_GPIO, 1);
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI("wifi_init_sta", "wifi_init_sta finished.");
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI("wifi_init_sta", "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        gpio_set_level(LED_GPIO, 1);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI("wifi_init_sta", "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        gpio_set_level(LED_GPIO, 0);
    }
    else
    {
        ESP_LOGE("wifi_init_sta", "UNEXPECTED EVENT");
    }
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    // static char variable_name[32];
    // static char variable_value[32];
    static char *output_buffer = NULL;
    static int output_len = 0;

    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        break;
    case HTTP_EVENT_ON_CONNECTED:
        break;
    case HTTP_EVENT_HEADER_SENT:
        break;
    case HTTP_EVENT_ON_HEADER:
        break;
    case HTTP_EVENT_ON_DATA:
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            strncpy(rcv_buffer, (char *)evt->data, evt->data_len);
            if (evt->user_data)
            {
                memcpy(evt->user_data + output_len, evt->data, evt->data_len); // Copy data to user buffer
            }
            else
            {
                // Allocate memory for the output buffer if it hasn't been allocated yet
                if (output_buffer == NULL)
                {
                    output_buffer = (char *)malloc(esp_http_client_get_content_length(evt->client) + 1);
                    if (output_buffer == NULL)
                    {
                        ESP_LOGE("_http_event_handler", "Failed to allocate memory for output buffer");
                        return ESP_FAIL; // Return an error if memory allocation fails
                    }
                    output_len = 0; // Reset output length
                }
                memcpy(output_buffer + output_len, evt->data, evt->data_len); // Copy data to the output buffer
            }
            output_len += evt->data_len; // Update the output length
        }
        else
        {
            // Handle chunked response
            if (evt->user_data)
            {
                memcpy(evt->user_data + output_len, evt->data, evt->data_len); // Copy data to user buffer
            }
            else
            {
                output_buffer = realloc(output_buffer, output_len + evt->data_len + 1);
                if (output_buffer == NULL)
                {
                    ESP_LOGE("_http_event_handler", "Failed to reallocate memory for output buffer");
                    return ESP_FAIL; // Return an error if memory reallocation fails
                }
                memcpy(output_buffer + output_len, evt->data, evt->data_len); // Copy data to the output buffer
            }
            output_len += evt->data_len;      // Update the output length
            output_buffer[output_len] = '\0'; // Null-terminate the buffer
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI("_http_event_handler", "HTTP_EVENT_ON_FINISH"); // Log the finish event
        if (output_buffer != NULL)
        {
            // Print the received HTTP response
            printf("HTTP GET Response: %s\n", output_buffer);

            // Parse the JSON response
            cJSON *json = cJSON_Parse(output_buffer);
            if (json == NULL)
            {
                ESP_LOGE("_http_event_handler", "Failed to parse JSON"); // Log an error if JSON parsing fails
            }
            else
            {
                // Get "door_no"
                cJSON *door_no = cJSON_GetObjectItem(json, "door_no");
                if (cJSON_IsString(door_no) && (door_no->valuestring != NULL))
                {
                    printf("Door Number: %s\n", door_no->valuestring);
                }

                // Extract the value of "valve" from the JSON response
                cJSON *valve = cJSON_GetObjectItem(json, "valve");
                if (cJSON_IsString(valve) && (valve->valuestring != NULL))
                {
                    ESP_LOGI("_http_event_handler", "\"valve\" = \"%s\"\n", valve->valuestring);

                    int new_valve_status = atoi(valve->valuestring);
                    printf("New Valve Status: %d\n", new_valve_status);

                    if (new_valve_status != prev_valve_value)  // Detect change
                    {
                        prev_valve_value = new_valve_status; // Update previous value
                        valve_value = new_valve_status; // Set new value

                        get_request_data = 1; // Immediately trigger API call
                    }
                    // // Store the variable name and value
                    // strcpy(variable_name, "valve");
                    // strcpy(variable_value, valve->valuestring);
                    // printf("Variable Name: %s, Variable Value: %s\n", variable_name, variable_value);
                    // printf("The gpio valve state:%s\n", variable_value);

                    // // Convert the variable value to an integer
                    // int valve_status = atoi(variable_value);
                    // printf("The int valve state:%d\n", valve_status);
                    // valve_value = valve_status;
                }
                cJSON_Delete(json); // Clean up the JSON object
            }

            free(output_buffer); // Free the output buffer
            output_buffer = NULL;
        }
        output_len = 0; // Reset the output length
        break;
    case HTTP_EVENT_DISCONNECTED:
        break;
    }
    return ESP_OK;
}

void check_update_task(void *pvParameter)
{
    while (1)
    {
        ota_update_counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (ota_update_counter >= duration_ota_update)
        {
            ota_update_counter = 0;
            printf("Looking for a new firmware...\n");
            esp_http_client_config_t config = {
                .url = UPDATE_JSON_URL,
                .event_handler = _http_event_handler,
            };
            esp_http_client_handle_t client = esp_http_client_init(&config);
            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK)
            {
                cJSON *json = cJSON_Parse(rcv_buffer);
                if (json == NULL)
                    printf("downloaded file is not a valid json, aborting...\n");
                else
                {
                    cJSON *version = cJSON_GetObjectItemCaseSensitive(json, "version");
                    cJSON *file = cJSON_GetObjectItemCaseSensitive(json, "file");
                    if (!cJSON_IsNumber(version))
                        printf("unable to read new version, aborting...\n");
                    else
                    {
                        double new_version = version->valuedouble;
                        if (new_version > FIRMWARE_VERSION)
                        {
                            printf("current firmware version (%.1f) is lower than the available one (%.1f), upgrading...\n", FIRMWARE_VERSION, new_version);
                            if (cJSON_IsString(file) && (file->valuestring != NULL))
                            {
                                printf("downloading and installing new firmware (%s)...\n", file->valuestring);
                                esp_http_client_config_t ota_client_config = {
                                    .url = file->valuestring,
                                    .cert_pem = NULL,
                                };
                                esp_err_t ret = esp_https_ota(&ota_client_config);
                                ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
                                if (ret == ESP_OK)
                                {
                                    printf("OTA OK, restarting...\n");
                                    esp_restart();
                                }
                                else
                                {
                                    printf("OTA failed...\n");
                                }
                            }
                            else
                                printf("unable to read the new file name, aborting...\n");
                        }
                        else
                            printf("current firmware version (%.1f) is greater or equal to the available one (%.1f), nothing to do...\n", FIRMWARE_VERSION, new_version);
                    }
                }
            }
            else
            {
                printf("unable to download the json file, aborting...\n");
            }
            esp_http_client_cleanup(client);
            printf("Process Completed\n");
        }
    }
}

static void http_get_task(void *pvParameters) // SEND DATA TO API
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];
    while (1)
    {
        if (get_request_data)
        {
            get_request_data = 0;
            esp_task_wdt_reset();
            int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);
            if (err != 0 || res == NULL)
            {
                ESP_LOGE("http_get_task", "DNS lookup failed err=%d res=%p", err, res);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }
            addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
            ESP_LOGI("http_get_task", "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));
            s = socket(res->ai_family, res->ai_socktype, 0);
            if (s < 0)
            {
                ESP_LOGE("http_get_task", "... Failed to allocate socket.");
                freeaddrinfo(res);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGI("http_get_task", "... allocated socket");
            if (connect(s, res->ai_addr, res->ai_addrlen) != 0)
            {
                ESP_LOGE("http_get_task", "... socket connect failed errno=%d", errno);
                close(s);
                freeaddrinfo(res);
                vTaskDelay(4000 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGI("http_get_task", "... connected");
            freeaddrinfo(res);
            n = sprintf(REQUEST, "%s%s", "GET ", "/wms/pushdata/wingfield_api.php?project_id=");
            n += sprintf(REQUEST + n, "%s&data=fmid:%s,flow:%.2f,tot:%d,valve:%d,door:%s,ver:%.2f HTTP/1.0\r\n", projid, FLOWMETER_ID, flowrate, Totalizer, valve_value, DOOR_NO, FIRMWARE_VERSION);
            n += sprintf(REQUEST + n, "%s", "Host: " WEB_SERVER ":" WEB_PORT "\r\n");
            n += sprintf(REQUEST + n, "%s", "User-Agent: esp-idf/1.0 esp32\r\n");
            n += sprintf(REQUEST + n, "%s", "\r\n");
            if (write(s, REQUEST, strlen(REQUEST)) < 0)
            {
                ESP_LOGE("http_get_task", "... socket send failed");
                close(s);
                vTaskDelay(4000 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGI("http_get_task", "... socket send success");
            struct timeval receiving_timeout;
            receiving_timeout.tv_sec = 5;
            receiving_timeout.tv_usec = 0;
            if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                           sizeof(receiving_timeout)) < 0)
            {
                ESP_LOGE("http_get_task", "... failed to set socket receiving timeout");
                close(s);
                vTaskDelay(4000 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGI("http_get_task", "... set socket receiving timeout success");
            do
            {
                bzero(recv_buf, sizeof(recv_buf));
                r = read(s, recv_buf, sizeof(recv_buf) - 1);
                for (int i = 0; i < r; i++)
                {
                    putchar(recv_buf[i]);
                }
            } while (r > 0);
            ESP_LOGI("http_get_task", "... done reading from socket. Last read return=%d errno=%d.", r, errno);
            close(s);
            ESP_LOGI("http_get_task", "Starting again!");
        }
    }
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Time   : %.8f s\r\n", (double)counter_value / TIMER_SCALE);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            if (enable_sampling == 1)
            {
                interrupt_sts = 1;
            }
        }
    }
}

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    example_timer_info_t *info = (example_timer_info_t *)args;
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);
    example_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value};
    if (!info->auto_reload)
    {
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }
    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

static void timer_task_example(void *arg)
{
    example_timer_event_t evt;
    for (;;)
    {
        xQueueReceive(s_timer_queue, &evt, portMAX_DELAY);
        Counter++;
        ts_temp++;
        http_get_timer++;
        if (ts_temp >= 50)
        {
            ts_temp = 0;
            ts++;
        }

        if (http_get_timer > 50 * 60)
        { // 60 seconds
            http_get_timer = 0;
            get_request_data = 1;
        }
    }
}

/**
 * @brief Initialize selected timer of timer group
 *
 * @param group Timer Group number, index from 0
 * @param timer timer ID, index from 0
 * @param auto_reload whether auto-reload on alarm event
 * @param timer_interval_sec interval of alarm
 */
static void example_tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_msec)
{
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    };
    timer_init(group, timer, &config);
    timer_set_counter_value(group, timer, 0);
    timer_set_alarm_value(group, timer, ((timer_interval_msec * TIMER_SCALE) / 1000));
    timer_enable_intr(group, timer);
    example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = (timer_interval_msec / 1000);
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);
    timer_start(group, timer);
}

void fetch_and_parse_json(void)
{
    esp_http_client_config_t config = {
        .url = VSTATUS_URL,
        .event_handler = _http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        ESP_LOGE("fetch_and_parse_json", "Failed to initialize HTTP client");
        vTaskDelay(pdMS_TO_TICKS(10000)); // Delay for 60 seconds
        return;
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI("fetch_and_parse_json", "HTTP GET Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE("fetch_and_parse_json", "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
// {
//     ESP_LOGE("STACK_OVERFLOW", "Stack overflow in task: %s", pcTaskName);
//     // Optionally enter an infinite loop or reset the system
//     while (1)
//         ;
// }

//     UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL); // NULL for current task
// ESP_LOGI("WIFI_COUNTER", "Stack high water mark: %d", uxHighWaterMark);

void app_main(void)
{
    uint32_t counter = 0;
    bool led_state = false;
    wifi_counter++;
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_MAX35101READY, gpio_isr_handler, (void *)GPIO_INPUT_MAX35101READY);
    s_timer_queue = xQueueCreate(10, sizeof(example_timer_event_t));
    // xTaskCreate(timer_task_example, "timer_task_example", 2048, NULL, 10, NULL);
    example_tg_timer_init(TIMER_GROUP_0, TIMER_0, true, 20); // 20 ms for sampling Interval
    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "CONFIG_MISO_GPIO=%d", CONFIG_MISO_GPIO);
    ESP_LOGI(TAG, "CONFIG_MOSI_GPIO=%d", CONFIG_MOSI_GPIO);
    ESP_LOGI(TAG, "CONFIG_SCLK_GPIO=%d", CONFIG_SCLK_GPIO);
    ESP_LOGI(TAG, "CONFIG_CS_GPIO=%d", CONFIG_CS_GPIO);
    printf("MAX Starting Up!\n");
    MAX35101_Init(CONFIG_MISO_GPIO, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO);
    board_init();
    counter = Counter;
    enable_sampling = 1;

    gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(MOTOR_BLUE_PIN);
    gpio_set_pull_mode(MOTOR_BLUE_PIN, GPIO_PULLDOWN_ONLY);
    gpio_set_direction(MOTOR_BLUE_PIN, GPIO_MODE_INPUT);

    gpio_pad_select_gpio(MOTOR_YELLOW_PIN);
    gpio_set_pull_mode(MOTOR_YELLOW_PIN, GPIO_PULLDOWN_ONLY);
    gpio_set_direction(MOTOR_YELLOW_PIN, GPIO_MODE_INPUT);

    gpio_pad_select_gpio(MOTOR_IN1);
    gpio_set_direction(MOTOR_IN1, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(MOTOR_IN2);
    gpio_set_direction(MOTOR_IN2, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    void wifi_counter_task(void *pvParameter)
    {
        while (1)
        {
            wifi_counter++;
            //     UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL); // NULL for current task
            // ESP_LOGI("WIFI_COUNTER", "Stack high water mark: %d", uxHighWaterMark);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            if (wifi_counter > duration_wifi_reconnect)
            {
                wifi_counter = 0;
                wifi_ap_record_t ap_info;
                esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
                if (ret == ESP_OK)
                {
                    ESP_LOGI("WIFI", "Connected to SSID:%s", ap_info.ssid);
                    ESP_LOGI("WIFI", "RSSI:%d", ap_info.rssi);
                }
                else
                {
                    ESP_LOGE("WIFI", "Failed to get AP info");
                }
                if (esp_wifi_connect())
                {
                    printf("WiFi is connected.\n");
                }
                else
                {
                }

                fetch_and_parse_json();
            }
            else
            {
            }
        }
    }
    void flash_counter_task(void *pvParameter)
    {
        while (1)
        {
            esp_err_t ret = nvs_flash_init();
            if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
            {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ret = nvs_flash_init();
            }
            ESP_ERROR_CHECK(ret);
            flash_counter++;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            // open
            nvs_handle_t my_handle;
            ret = nvs_open("storage", NVS_READWRITE, &my_handle);

            // Read
            ret = nvs_get_i32(my_handle, "Totalizer", &tot_in_flash);
            vTaskDelay(pdMS_TO_TICKS(100));
            Totalizer = tot_in_flash;
            Totalizer += sum_flow;
            vTaskDelay(pdMS_TO_TICKS(100));
            if (flash_counter >= duration_write_flash) // Time
            {
                printf("flash function \n");
                flash_counter = 0;
                ret = nvs_set_i32(my_handle, "Totalizer", Totalizer);
                vTaskDelay(pdMS_TO_TICKS(100));
                ret = nvs_commit(my_handle);
                printf("******************************Last Value written in flash is = %dL\n", Totalizer);
                sum_flow = 0;
                vTaskDelay(pdMS_TO_TICKS(100));
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
            nvs_close(my_handle);
        }
    }

    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 6, NULL);
    xTaskCreate(timer_task_example, "timer_task_example", 2048, NULL, 5, NULL);
    xTaskCreate(&check_update_task, "check_update_task", 8192, NULL, 4, NULL); // OTA Update check
    xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 1, NULL);         // Data push to API
    xTaskCreate(&wifi_counter_task, "wifi_counter_task", 4096, NULL, 2, NULL);
    xTaskCreate(&flash_counter_task, "flash_counter_task", 2048, NULL, 3, NULL);

    while (1)
    {
        if (interrupt_sts == 1)
        {
            interrupt_sts = 0;
            IntrStatus = max3510x_interrupt_status(&g_max35103);

            if ((IntrStatus & (MAX3510X_REG_INTERRUPT_STATUS_TOF | MAX3510X_REG_INTERRUPT_STATUS_TO)) == MAX3510X_REG_INTERRUPT_STATUS_TOF)
            {
                max3510x_read_tof_results(&g_max35103, &s_tof_results);
                s_results_count++;
            }
            else if (IntrStatus)
            {
                s_timeout_count++;
            }
        }

        if (Counter != counter)
        {
            if ((!s_results_count) && (enable_sampling == 1))
                max3510x_tof_diff(&g_max35103);
            counter = Counter;
        }

        // flow = 0;
        if (s_results_count)
        {
            if (max3510x_validate_measurement(&s_tof_results.up, transducer_hit_count()) &&
                max3510x_validate_measurement(&s_tof_results.down, transducer_hit_count()))
            {
                float_t t2_ideal = max3510x_ratio_to_float(s_tof_results.up.t2_ideal);
                float_t t1_t2 = max3510x_ratio_to_float(s_tof_results.up.t1_t2);
                // if (t2_ideal >= 0.95f && t1_t2 > 0.50f)
                if (t2_ideal >= 0.75f && t1_t2 > 0.40f)
                {

                    double_t up = max3510x_fixed_to_double(&s_tof_results.up.average);
                    double_t down = max3510x_fixed_to_double(&s_tof_results.down.average);
                    flow = last_flow = transducer_flow(up, down);
                }
                else
                {
                    // printf("t1 & t2 ideal issue\r\n");
                }
            }

            if (!flow)
            {
                flow = last_flow;
            }

            if (flow < 0)
            {
                last_flow = 0;
                flow = 0;
            }
            flow_check = (flow * CUBIC_METERS_TO_LITERS / 50.0);
            if (flow_check > 0.00015)
            {
                sum_flow += (1.15 * (flow * CUBIC_METERS_TO_LITERS / 50.0));
            }
            s_results_count = 0;
            c++;
        }

        if (ts != last_ts)
        {
            board_led(led_state);
            led_state = !led_state;
            flowrate = last_flow * CUBIC_METERS_TO_LITERS * 60.0;
            printf("################# After Sensor ########################## \n");
            printf("Flowmeter ID = %s,  Door No. =%s\r\n", FLOWMETER_ID, DOOR_NO);
            printf("Flow = %.2fL,  last_flow =%.2fLPM\r\n", flow, last_flow);
            printf("Flowcheck = %.10fL\r\n", flow_check);
            printf("Total Flow = %.2fL, Flow Rate =%.2fLPM\r\n", sum_flow, flowrate); // gallons per minute
            printf("Totalizer = %dL\n", Totalizer);
            bluepin_state = gpio_get_level(MOTOR_BLUE_PIN);
            yellowpin_state = gpio_get_level(MOTOR_YELLOW_PIN);
            printf("The gpio bluepin state:%d\n", bluepin_state);
            printf("The gpio yellowpin state:%d\n", yellowpin_state);
            printf("Current valve status:%d, previouse valve status:%d\n", valve_value, prev_valve_value);

            // Repeat the control logic based on the valve_value
            if (valve_value == 1)
            {
                gpio_set_level(MOTOR_IN1, 0);
                gpio_set_level(MOTOR_IN2, 1);
                printf("VALVE OPEN\n");
                if (bluepin_state == 1)
                {
                    gpio_set_level(MOTOR_IN1, 1);
                    gpio_set_level(MOTOR_IN2, 1);
                }
            }
            else if (valve_value == 0)
            {
                gpio_set_level(MOTOR_IN1, 1);
                gpio_set_level(MOTOR_IN2, 0);
                printf("VALVE CLOSE\n");
                if (yellowpin_state == 1)
                {
                    gpio_set_level(MOTOR_IN1, 1);
                    gpio_set_level(MOTOR_IN2, 1);
                }
            }
            else
            {
                printf("motor no rotate on degree\n"); // Print a message if the value is unexpected
            }
            last_ts = ts;
        }
    }
}
