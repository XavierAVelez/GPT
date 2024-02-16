#include <stdio.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>


#define GPIO_OUT_PIN_SEL    (1ULL << GPIO_NUM_16)
#define GPIO_IN_PIN_SEL    (1ULL << GPIO_NUM_17)
#define SPI_

static QueueHandle_t gpio_evt_queue = NULL;


static void IRAM_ATTR ButtonISR(void* arg)
{
    // send a gpio task to be handled by the specified function: LED_Control
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


void IMU_Control(void* params)
{
    // function to toggle the LED
    uint32_t pinNumber;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &pinNumber, portMAX_DELAY)) {
            printf("GPIO Interrupt Working");
            gpio_set_level(GPIO_NUM_16, 1);
            vTaskDelay(100);
            gpio_set_level(GPIO_NUM_16, 0);
        }
    }
}


void app_main(void)
{
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

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the input pin GPIO17
    io_conf.pin_bit_mask = GPIO_IN_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(IMU_Control, "IMU_Control", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_NUM_17, ButtonISR, (void*) GPIO_NUM_17);


    spi_device_handle_t handle;

    spi_bus_config_t spi_conf = {
        .mosi_io_num = GPIO_NUM_23,
        .miso_io_num = GPIO_NUM_19,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
        //.max_transfer_sz = 32
    };

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

    ret = spi_bus_initialize(VSPI_HOST, &spi_conf, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(VSPI_HOST, &dev_conf, &handle);
    assert(ret == ESP_OK);

    char recvbuf[14] = ""; // collect 14 registers that in succession (these contain gyro & accel)
    memset(recvbuf, 0, 1);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    

    printf("Master input:\n");
    while (1)
    {
        spi_device_acquire_bus(handle, portMAX_DELAY);
        t.length = sizeof(recvbuf) * 8;
        t.addr = 0xBB; // start register for accel and gyro data
        t.rx_buffer = recvbuf;
        t.rxlength = 0;
        spi_device_polling_transmit(handle, &t);
        printf("Accel X: %f\n", (((float)recvbuf[0] * .39) + ((float)recvbuf[1] * .0039)));
        printf("Accel Y: %f\n", (((float)recvbuf[2] * .39) + ((float)recvbuf[3] * .0039)));
        printf("Accel Z: %f\n\n", (((float)recvbuf[4] * .39) + ((float)recvbuf[5] * .0039)));

        printf("Gyro X: %f\n", (((float)recvbuf[8] * .39) + ((float)recvbuf[9] * .0039)));
        printf("Gyro Y: %f\n", (((float)recvbuf[10] * .39) + ((float)recvbuf[11] * .0039)));
        printf("Gyro Z: %f\n", (((float)recvbuf[12] * .39) + ((float)recvbuf[13] * .0039)));

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        spi_device_release_bus(handle);
    }
    

    /*
    //When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
	spi_device_acquire_bus(handle, portMAX_DELAY);

    while (1){
        //----- TRANSMIT BYTES -----
        uint8_t TxData = 0x04;
        spi_transaction_t trans;
        memset(&trans, 0, sizeof(trans));			//Zero out the transaction ready to use
        trans.length = 1 * 8;								//Transaction length is in bits
        trans.tx_buffer = &TxData;							//Set the tx data buffer
        trans.flags = SPI_TRANS_CS_KEEP_ACTIVE;			//Keep CS active after data transfer
        //Do it
        spi_device_polling_transmit(handle, &trans);		//Waits until the transfer is complete


        //----- RECEIVE BYTES -----
        uint8_t RxData = 0x04;
        memset(&trans, 0, sizeof(trans));			//Zero out the transaction ready to use
        trans.length = 3 * 8;								//Transaction length is in bits
        trans.rx_buffer = &RxData;
        //Do it
        spi_device_polling_transmit(handle, &trans);		//Waits until the transfer is complete
    }
    */

    //Never reached.
    spi_bus_remove_device(handle);

}
