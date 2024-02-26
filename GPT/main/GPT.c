#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
//#include "esp_eap_client.h"
#include "esp_log.h"
#include "network.h"
#include "HTML.h"


#define GPIO_OUT_PIN_SEL    (1ULL << GPIO_NUM_16)
#define GPIO_IN_PIN_SEL    (1ULL << GPIO_NUM_17)

bool StartData = false;
bool StartClock = false;


// struct to store sensor data
struct sensor_data {
    short gyroscopeX[302];
    short gyroscopeY[302];
    short gyroscopeZ[302];

    short accelerometerX[302];
    short accelerometerY[302];
    short accelerometerZ[302];
};

// instantiate global struct for sensor data
struct sensor_data SensorData;


static void alarm_event_handler(void* arg)
{
    // pass in the alarm handler from arg list
    esp_timer_handle_t alarm_timer_handle = (esp_timer_handle_t) arg;    
    // stop the timer
    esp_timer_stop(alarm_timer_handle);

    StartData = false;
}


// WiFi configuration log
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting WIFI_EVENT_STA_START ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected WIFI_EVENT_STA_CONNECTED ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection WIFI_EVENT_STA_DISCONNECTED ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}


// establish WiFi connection using info from network.h
void wifi_connection()
{
    nvs_flash_init();
    esp_netif_init();                    
    esp_event_loop_create_default();

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);
    
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = SSID,
            /*
            .pmf_cfg = {
                .required = false,
                .capable = true
            }
            */
            .password = PASS
        }};
    
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_set_mode(WIFI_MODE_STA);

    /*
    // set extra parameters for enterprise WiFi config
    ESP_ERROR_CHECK(esp_eap_client_set_identity((uint8_t *)EAP_ID, strlen(EAP_ID)));
    ESP_ERROR_CHECK(esp_eap_client_set_username((uint8_t *)EAP_USER, strlen(EAP_USER)));
    ESP_ERROR_CHECK(esp_eap_client_set_password((uint8_t *)EAP_PASS, strlen(EAP_PASS)));
    ESP_ERROR_CHECK(esp_eap_client_set_ca_cert((uint8_t *)ROOTCERT, strlen(ROOTCERT)));
    ESP_ERROR_CHECK(esp_eap_client_set_ttls_phase2_method(ESP_EAP_TTLS_PHASE2_MSCHAPV2));
    ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable());
    */

    esp_wifi_start();
    esp_wifi_connect();
}


// handler for GET call that has the main page
static esp_err_t get_handler_main(httpd_req_t *req)
{
    const char resp[] = HTML;
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


// hanlder for GET call that initiates data collection
static esp_err_t get_handler_start(httpd_req_t *req)
{
    // switch the boolean to start the data collection via polling
    StartData = true;
    StartClock = true;

    const char resp[] = HTML;
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}


static esp_err_t get_handler_gyroX(httpd_req_t *req)
{
    // allocate response buffer
    char resp[2000];

    // store sensor data in the buffer separated by commas
    int rind = 0;
    for (unsigned int i = 0; i < 300; i++)
    {
        snprintf(resp+rind, 3, "%hi", SensorData.gyroscopeX[i]);
        snprintf(resp+rind+3, 2, "%c", ',');
        rind += 5;
    }

    httpd_resp_send(req, resp, rind);
    return ESP_OK;
}

static esp_err_t get_handler_gyroY(httpd_req_t *req)
{
    // allocate response buffer
    char resp[2000];

    // store sensor data in the buffer separated by commas
    int rind = 0;
    for (unsigned int i = 0; i < 300; i++)
    {
        snprintf(resp+rind, 3, "%hi", SensorData.gyroscopeY[i]);
        snprintf(resp+rind+3, 2, "%c", ',');
        rind += 5;
    }

    httpd_resp_send(req, resp, rind);
    return ESP_OK;
}

static esp_err_t get_handler_accelY(httpd_req_t *req)
{
    // allocate response buffer
    char resp[2000];

    // store sensor data in the buffer separated by commas
    int rind = 0;
    for (unsigned int i = 0; i < 300; i++)
    {
        snprintf(resp+rind, 3, "%hi", SensorData.accelerometerY[i]);
        snprintf(resp+rind+3, 2, "%c", ',');
        rind += 5;
    }

    httpd_resp_send(req, resp, rind);
    return ESP_OK;
}

static esp_err_t get_handler_accelZ(httpd_req_t *req)
{
    // allocate response buffer
    char resp[2000];

    // store sensor data in the buffer separated by commas
    int rind = 0;
    for (unsigned int i = 0; i < 300; i++)
    {
        snprintf(resp+rind, 3, "%hi", SensorData.accelerometerZ[i]);
        snprintf(resp+rind+3, 2, "%c", ',');
        rind += 5;
    }

    httpd_resp_send(req, resp, rind);
    return ESP_OK;
}


void server_initiation()
{
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server_handle = NULL;
    httpd_start(&server_handle, &server_config);

    httpd_uri_t uri_get_main = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_handler_main,
        .user_ctx = NULL
    };

    httpd_uri_t uri_get_start = {
        .uri = "/start",
        .method = HTTP_GET,
        .handler = get_handler_start,
        .user_ctx = NULL
    };

    httpd_uri_t uri_get_gyroX = {
        .uri = "/gyrox",
        .method = HTTP_GET,
        .handler = get_handler_gyroX,
        .user_ctx = NULL
    };

    httpd_uri_t uri_get_gyroY = {
        .uri = "/gyroy",
        .method = HTTP_GET,
        .handler = get_handler_gyroY,
        .user_ctx = NULL
    };

    httpd_uri_t uri_get_accelY = {
        .uri = "/accely",
        .method = HTTP_GET,
        .handler = get_handler_accelY,
        .user_ctx = NULL
    };

    httpd_uri_t uri_get_accelZ = {
        .uri = "/accelz",
        .method = HTTP_GET,
        .handler = get_handler_accelZ,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(server_handle, &uri_get_main);
    httpd_register_uri_handler(server_handle, &uri_get_start);
    httpd_register_uri_handler(server_handle, &uri_get_gyroX);
    httpd_register_uri_handler(server_handle, &uri_get_gyroY);
    httpd_register_uri_handler(server_handle, &uri_get_accelY);
    httpd_register_uri_handler(server_handle, &uri_get_accelZ);
}


void app_main(void)
{
    //connect to wifi on startup
    wifi_connection();
    server_initiation();

    //zero-initialize the config structure.
    gpio_config_t io_conf = {
        //disable interrupt
        .intr_type = GPIO_INTR_DISABLE,
        //set as output mode
        .mode = GPIO_MODE_OUTPUT,
        //bit mask of the output pin GPIO16
        .pin_bit_mask = GPIO_OUT_PIN_SEL,
        //disable pull-down mode
        .pull_down_en = 0,
        //disable pull-up mode
        .pull_up_en = 0
    };

    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // create SPI handle
    spi_device_handle_t handle;

    // configure the SPI bus and ports
    spi_bus_config_t spi_conf = {
        .mosi_io_num = GPIO_NUM_23,
        .miso_io_num = GPIO_NUM_19,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
        //.max_transfer_sz = 32
    };

    // configure the IMU device that will communicate via SPI
    spi_device_interface_config_t dev_conf = {
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .clock_speed_hz = 100000, // 100kHz clock speed (maximum is 1MHz)
        .duty_cycle_pos = 128,
        .mode = 0,
        .spics_io_num = GPIO_NUM_5,
        .cs_ena_posttrans = 0,
        .cs_ena_pretrans = 0,
        .queue_size = 1
    };

    esp_err_t ret;

    // initialize the bus using the above configuration
    ret = spi_bus_initialize(VSPI_HOST, &spi_conf, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);
    // initialize the device using the above configuration
    ret = spi_bus_add_device(VSPI_HOST, &dev_conf, &handle);
    assert(ret == ESP_OK);

    // collect 14 registers that in succession (gyro & accel)
    char recvbuf[14] = "";
    memset(recvbuf, 0, 1);
    // instantiate the transaction between IMU and ESP32
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    printf("Master input:\n");
    int index = 0;
    while (1)
    {
        // if the GET call initiates data collection, start timer
        if (StartClock)
        {
            // instantiate alarm timer
            const esp_timer_create_args_t alarm_timer_args = {
                    .callback = &alarm_event_handler,
                    .name = "DataAlarm"
            };

            esp_timer_handle_t alarm_timer;
            esp_timer_create(&alarm_timer_args, &alarm_timer);

            // send three buzzes through the vibrating motor
            for (unsigned int i = 0; i < 3; i++)
            {
                vTaskDelay(300);
                gpio_set_level(GPIO_NUM_16, 1);
                vTaskDelay(100);
                gpio_set_level(GPIO_NUM_16, 0);
            }

            // start a timer for three seconds
            esp_timer_start_once(alarm_timer, 3000000);

            StartClock = false;
        }

        // if the GET call initiates data collection
        if (StartData)
        {
            // use the SPI bus for data transmissions
            spi_device_acquire_bus(handle, portMAX_DELAY);
            // set the length and initial address of the transaction
            t.length = sizeof(recvbuf) * 8;
            t.addr = 0xBB; // start register for accel and gyro data
            t.rx_buffer = recvbuf;
            t.rxlength = 0;
            // poll for the data until complete
            spi_device_polling_transmit(handle, &t);

            // store gyroscope data in sensor data struct
            SensorData.accelerometerX[index] = ((short)(recvbuf[0] * .39));
            SensorData.accelerometerY[index] = ((short)(recvbuf[2] * .39));
            SensorData.accelerometerZ[index] = ((short)(recvbuf[4] * .39));

            SensorData.gyroscopeX[index] = ((short)(recvbuf[8] * .39));
            SensorData.gyroscopeY[index] = ((short)(recvbuf[10] * .39));
            SensorData.gyroscopeZ[index] = ((short)(recvbuf[12] * .39));

            printf("Gyro X: %hi\n", (((short)(recvbuf[8] * .39))));
            printf("Gyro Y: %hi\n", (((short)(recvbuf[10] * .39))));
            printf("Gyro Z: %hi\n\n", (((short)(recvbuf[12] * .39))));
            

            /*
            printf("Accel X: %f\n", (((float)recvbuf[0] * .39) + ((float)recvbuf[1] * .0039)));
            printf("Accel Y: %f\n", (((float)recvbuf[2] * .39) + ((float)recvbuf[3] * .0039)));
            printf("Accel Z: %f\n\n", (((float)recvbuf[4] * .39) + ((float)recvbuf[5] * .0039)));

            printf("Gyro X: %f\n", (((float)recvbuf[8] * .39) + ((float)recvbuf[9] * .0039)));
            printf("Gyro Y: %f\n", (((float)recvbuf[10] * .39) + ((float)recvbuf[11] * .0039)));
            printf("Gyro Z: %f\n", (((float)recvbuf[12] * .39) + ((float)recvbuf[13] * .0039)));
            */

            // wait and release the SPI bus to free resources before next run
            //vTaskDelay(1);

            spi_device_release_bus(handle);
            index++;
            printf("%i\n", index);
        }
        else{
            index = 0;
        }
        vTaskDelay(1);
    }
    
    //Never reached.
    spi_bus_remove_device(handle);
}
