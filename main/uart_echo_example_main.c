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
 * with hardware flow control turned off. It uses a UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: see defines below (See Kconfig)
 */

#define MESSAGE_LENGTH 77

#define UART1_TXD (17)
#define UART1_RXD (16)
#define UART1_RTS (UART_PIN_NO_CHANGE)
#define UART1_CTS (UART_PIN_NO_CHANGE)

#define UART1_PORT_NUM      (2)
#define UART1_BAUD_RATE     (2000000)  // 2M baud rate
#define UART1_TASK_STACK_SIZE    (3072)
#define UART1_QUEUE_SIZE     (20) // Number of events in the UART event queue

static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)
#define LINE_BUF_SIZE (128)  // Line buffer size - large enough to hold a single line

#define REPORT_INTERVAL_MS (10000)  // Report interval in milliseconds

static QueueHandle_t uart1_queue;

volatile uint32_t good_line_count = 0;
volatile uint32_t bad_line_count = 0;
static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

void uart_report_task(void *arg)
{
    char message[64];
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(REPORT_INTERVAL_MS)); // 10 seconds

        uint32_t good_count_snapshot;
        uint32_t bad_count_snapshot = 0;

        // Atomic swap (safe because it's just a 32-bit int on ESP32)
        taskENTER_CRITICAL(&my_spinlock);
        good_count_snapshot = good_line_count;
        good_line_count = 0;
        bad_count_snapshot += bad_line_count; // Include bad lines in the report
        bad_line_count = 0;
        taskEXIT_CRITICAL(&my_spinlock);

        snprintf(message, sizeof(message), "Good lines (msg/s): %lu ", (unsigned long)good_count_snapshot * 1000 / REPORT_INTERVAL_MS);
        uart_write_bytes(UART1_PORT_NUM, message, strlen(message));
        snprintf(message, sizeof(message), "Bad lines (msg/s): %lu\r\n", (unsigned long)bad_count_snapshot * 1000 / REPORT_INTERVAL_MS);
        uart_write_bytes(UART1_PORT_NUM, message, strlen(message));
        ESP_LOGI(TAG, "Reported %lu good lines", (unsigned long)good_count_snapshot);
        ESP_LOGI(TAG, "Reported %lu bad lines", (unsigned long)bad_count_snapshot);
    }
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t rx_buffer[LINE_BUF_SIZE];
    int rx_len = 0;

    for (;;)
    {
        //Waiting for UART event.
        if (xQueueReceive(uart1_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            // ESP_LOGI(TAG, "uart[%d] event:", UART1_PORT_NUM);
            switch (event.type)
            {
            //Event of UART receiving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uint8_t data[LINE_BUF_SIZE];  // Buffer to hold incoming data
                uart_read_bytes(UART1_PORT_NUM, data, MESSAGE_LENGTH, 10 / portTICK_PERIOD_MS);

                for (int i = 0; i < MESSAGE_LENGTH; i++)
                {
                    if (rx_len >= LINE_BUF_SIZE - 1)
                    {
                        // Overflow handling
                        rx_len = 0;
                        ESP_LOGE(TAG,"Line buffer overflow, clearing...");
                        continue;  // Exit if buffer is full
                    }

                    char ch = data[i];
                    rx_buffer[rx_len++] = ch;

                    if (ch != '\n')
                    {
                        continue;
                    }

                    // Null-terminate for safe logging/debugging (won't be sent over UART)
                    // rx_buffer[rx_len] = '\0';

                    // If we reach here, it means we have a complete line
                    if(rx_len == MESSAGE_LENGTH)
                    {
                        // Handle line of correct length
                        // uart_write_bytes(UART1_PORT_NUM, (const char*) rx_buffer, rx_len);
                        // uart_write_bytes(UART1_PORT_NUM, ".", 1);  // Echo back a dot
                        good_line_count++; // Count a good line
                    }
                    else
                    {
                        // Handle incomplete line
                        uart_write_bytes(UART1_PORT_NUM, (const char*) rx_buffer, rx_len);
                        ESP_LOGW(TAG, "Incomplete line: %.*s", rx_len, rx_buffer);
                        bad_line_count++; // Count a bad line
                    }
                    // Reset for next line
                    rx_len = 0;
                }
                // ESP_LOGI(TAG, "[DATA EVT]:");
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART1_PORT_NUM);
                xQueueReset(uart1_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART1_PORT_NUM);
                xQueueReset(uart1_queue);
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
    vTaskDelete(NULL);
}
   

void app_main(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART1_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // poll the UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART1_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, UART1_QUEUE_SIZE, &uart1_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART1_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART1_PORT_NUM, UART1_TXD, UART1_RXD, UART1_RTS, UART1_CTS));

    xTaskCreate(uart_event_task, "uart_event_task", UART1_TASK_STACK_SIZE, NULL, 12, NULL);
    xTaskCreate(uart_report_task, "uart_report_task", 2048, NULL, 5, NULL);
}
