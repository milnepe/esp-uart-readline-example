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
#include "freertos/ringbuf.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

/**
 * This is an example which echos any data lines it receives on configured UART back to the sender,
 * with hardware flow control turned off. It uses a UART driver event queue.
 *
 * A line is considered "good" if it is exactly MESSAGE_LENGTH characters long, including the newline character.
 * A line is considered "bad" if it is shorter or longer than MESSAGE_LENGTH characters.
 *
 * A report task runs every REPORT_INTERVAL_MS milliseconds to log the number of good and bad lines received.
 *
 * The UART driver is configured with the following parameters:
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: see defines below (See Kconfig)
 */

#define MESSAGE_LENGTH 77

#define UART2_TXD (17)
#define UART2_RXD (16)
#define UART2_RTS (UART_PIN_NO_CHANGE)
#define UART2_CTS (UART_PIN_NO_CHANGE)

#define UART2_PORT_NUM      (2)
#define UART2_BAUD_RATE     (2000000)  // 2M baud rate
#define UART2_TASK_STACK_SIZE    (4096)
#define UART2_QUEUE_SIZE     (20) // Number of events in the UART event queue

static const char *TAG = "UART TEST";

#define BUF_SIZE MESSAGE_LENGTH
#define MAX_PACKETS 2000  // depends on heap available
// #define LINE_BUF_SIZE (128)  // Line buffer size - large enough to hold a single line

#define REPORT_INTERVAL_MS (10000)  // Report interval in milliseconds

// static QueueHandle_t uart2_queue;
static RingbufHandle_t ringbuf = NULL;

volatile uint32_t good_line_count = 0;
volatile uint32_t bad_line_count = 0;
static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;


void uart2_report_task(void *arg)
{
    char message[64];
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(REPORT_INTERVAL_MS)); // 10 seconds

        uint32_t good_count_snapshot = 0;
        uint32_t bad_count_snapshot = 0;

        // Atomic swap
        taskENTER_CRITICAL(&my_spinlock);
        good_count_snapshot = good_line_count;
        good_line_count = 0;
        bad_count_snapshot = bad_line_count;
        bad_line_count = 0;
        taskEXIT_CRITICAL(&my_spinlock);

        snprintf(message, sizeof(message), "Good lines (msg/s): %lu ", (unsigned long)good_count_snapshot * 1000 / REPORT_INTERVAL_MS);
        uart_write_bytes(UART2_PORT_NUM, message, strlen(message));
        snprintf(message, sizeof(message), "Bad lines (msg/s): %lu\r\n", (unsigned long)bad_count_snapshot * 1000 / REPORT_INTERVAL_MS);
        uart_write_bytes(UART2_PORT_NUM, message, strlen(message));
        ESP_LOGI(TAG, "Reported %lu good lines", (unsigned long)good_count_snapshot);
        ESP_LOGI(TAG, "Reported %lu bad lines", (unsigned long)bad_count_snapshot);
    }
}

static void uart2_event_task(void *pvParameters)
{
    QueueHandle_t uart_queue = (QueueHandle_t)pvParameters;
    uart_event_t event;
    uint8_t data[BUF_SIZE];

    while (1) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            if (event.type == UART_DATA && event.size >= BUF_SIZE) {
                while (event.size >= BUF_SIZE) {
                    int len = uart_read_bytes(UART2_PORT_NUM, data, BUF_SIZE, 10 / portTICK_PERIOD_MS);
                    if (len == BUF_SIZE) {
                        if (xRingbufferSend(ringbuf, data, BUF_SIZE, pdMS_TO_TICKS(100)) != pdTRUE) {
                            ESP_LOGW(TAG, "Ring buffer full, data lost");
                        }
                    }
                    event.size -= BUF_SIZE;
                }
            } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                ESP_LOGW(TAG, "UART overflow");
                uart_flush_input(UART2_PORT_NUM);
                xQueueReset(uart_queue);
            }
        }
    }
}

void socket_sender_task(void *arg) {
    // int sock = -1;
    // struct sockaddr_in server_addr;
    // server_addr.sin_family = AF_INET;
    // server_addr.sin_port = htons(SERVER_PORT);
    // inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    while (1) {
        // if (sock < 0) {
        //     sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        //     if (sock < 0) {
        //         ESP_LOGE(TAG, "Unable to create socket");
        //         vTaskDelay(pdMS_TO_TICKS(2000));
        //         continue;
        //     }
        //     if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        //         ESP_LOGE(TAG, "Socket connection failed");
        //         close(sock);
        //         sock = -1;
        //         vTaskDelay(pdMS_TO_TICKS(2000));
        //         continue;
        //     }
        //     ESP_LOGI(TAG, "Connected to server");
        // }

        uint8_t send_buf[BUF_SIZE * 5]; // Batch up to 5 packets (adjust as needed)
        size_t total_len = 0;

        for (int i = 0; i < 5; i++) {
            size_t item_size;
            void *item = xRingbufferReceive(ringbuf, &item_size, pdMS_TO_TICKS(200));
            if (!item || item_size != BUF_SIZE) {
                bad_line_count++; // Count a bad line
                break;
            }
            memcpy(send_buf + total_len, item, item_size);
            total_len += item_size;
            vRingbufferReturnItem(ringbuf, item);
        }

        if (total_len > 0) {
            if (total_len % 5 == 0) {
                good_line_count +=5; // Count a good line
            }
            // int sent = send(sock, send_buf, total_len, 0);
            // int sent = uart_write_bytes(UART2_PORT_NUM, send_buf, total_len);
            // if (sent < 0) {
                // ESP_LOGE(TAG, "Send failed");
                // close(sock);
                // sock = -1;
            // }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void app_main(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART2_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART2_PORT_NUM, 1024 * 4, 1024 * 4, UART2_QUEUE_SIZE, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART2_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART2_PORT_NUM, UART2_TXD, UART2_RXD, UART2_RTS, UART2_CTS));

    ringbuf = xRingbufferCreate(MAX_PACKETS * BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (ringbuf == NULL) {
        ESP_LOGE(TAG, "Failed to create ring buffer");
        return;
    }

    xTaskCreate(uart2_event_task, "uart2_event_task", UART2_TASK_STACK_SIZE, uart_queue, 12, NULL);
    xTaskCreate(uart2_report_task, "uart2_report_task", 2048, NULL, 5, NULL);

    xTaskCreate(socket_sender_task, "socket_sender", 4096, NULL, 10, NULL);
}
