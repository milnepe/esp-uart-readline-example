/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define MESSAGE_LENGTH 77

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define PATTERN_CHR_NUM    (3) 
static QueueHandle_t uart0_queue;

// static void echo_task(void *arg)
// {
//     // Configure a temporary buffer for the incoming data
//     uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

//     while (1) {
//         // Read data from the UART
//         // int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
//         int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 10 / portTICK_PERIOD_MS);
//         // Write data back to the UART
//         uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
//         if (len) {
//             data[len] = '\0';
//             ESP_LOGI(TAG, "Recv str: %s", (char *) data);
//         }
//     }
// }

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    // uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    uint8_t* rx_buffer = (uint8_t*) malloc(RD_BUF_SIZE);
    int rx_len = 0;

    for (;;)
    {
        //Waiting for UART event.
        // memset(dtmp, 0, RD_BUF_SIZE);
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            ESP_LOGI(TAG, "uart[%d] event:", ECHO_UART_PORT_NUM);
            switch (event.type)
            {
            //Event of UART receiving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                // uint8_t data[128];
                uint8_t data[MESSAGE_LENGTH];            
                int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, sizeof(data), 10 / portTICK_PERIOD_MS);

                // for (int i = 0; i < len; i++) {
                for (int i = 0; i < MESSAGE_LENGTH; i++) {
                    if (rx_len < BUF_SIZE - 1) {
                        rx_buffer[rx_len++] = data[i];

                        // Example: End on '\n'
                        if ((data[i] == '\n') && (rx_len == MESSAGE_LENGTH)) {
                            rx_buffer[rx_len] = '\0';  // Null-terminate
                            // printf("Received line: %s", (char*)rx_buffer);
                            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*) rx_buffer, rx_len);

                            // Reset for next line
                            rx_len = 0;
                        }
                    } else {
                        // Overflow handling
                        rx_len = 0;
                        ESP_LOGE(TAG,"Line buffer overflow, clearing...");
                    }
                }

                // uart_read_bytes(ECHO_UART_PORT_NUM, rx_buffer, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "[DATA EVT]:");
                // uart_write_bytes(ECHO_UART_PORT_NUM, (const char*) rx_buffer, event.size);
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(ECHO_UART_PORT_NUM);
                xQueueReset(uart0_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(ECHO_UART_PORT_NUM);
                xQueueReset(uart0_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
            }
        }
    }
    free(rx_buffer);
    rx_buffer = NULL;
    vTaskDelete(NULL);
}
   

void app_main(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    // poll the UART driver
    // ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    // Use the event queue
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(uart_event_task, "uart_event_task", 3072, NULL, 12, NULL);
}
