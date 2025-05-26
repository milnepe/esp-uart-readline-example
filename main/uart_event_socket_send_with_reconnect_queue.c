/*
    UART-to-TCP forwarder with:

    No hardware flow control

    DMA-based UART reads

    Reliable resend queue for handling disconnects

    Auto-reconnect logic

    Todo: Add network connection!
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "esp_timer.h"
#include <string.h>

#define UART_NUM          UART_NUM_1
#define UART_BAUD         2000000
#define UART_BUF_SIZE     4096
#define UART_DMA_SIZE     2048
#define PACKET_SIZE       77
#define PACKETS_PER_SEND  5
#define SEND_BUFFER_SIZE  (PACKET_SIZE * PACKETS_PER_SEND)

#define SERVER_IP         "192.168.1.100"
#define SERVER_PORT       9000
#define QUEUE_SIZE        10

static const char *TAG = "UART_NET";

static QueueHandle_t uart_queue;

// Resend queue
static uint8_t resend_queue[QUEUE_SIZE][SEND_BUFFER_SIZE];
static int queue_head = 0;
static int queue_tail = 0;
static int queue_count = 0;

bool queue_is_empty() { return queue_count == 0; }
bool queue_is_full()  { return queue_count == QUEUE_SIZE; }

bool queue_push(const uint8_t *data) {
    if (queue_is_full()) return false;
    memcpy(resend_queue[queue_tail], data, SEND_BUFFER_SIZE);
    queue_tail = (queue_tail + 1) % QUEUE_SIZE;
    queue_count++;
    return true;
}

bool queue_pop(uint8_t *out) {
    if (queue_is_empty()) return false;
    memcpy(out, resend_queue[queue_head], SEND_BUFFER_SIZE);
    queue_head = (queue_head + 1) % QUEUE_SIZE;
    queue_count--;
    return true;
}

static int connect_to_server(void) {
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
        .sin_addr.s_addr = inet_addr(SERVER_IP),
    };

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket creation failed: errno=%d", errno);
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Socket connect failed: errno=%d", errno);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "Connected to server");
    return sock;
}

void uart_socket_task(void *pvParameters) {
    static uint8_t send_buffer[SEND_BUFFER_SIZE];
    static uint8_t uart_buf[UART_BUF_SIZE];
    static uint8_t partial_buf[PACKET_SIZE];
    static int partial_len = 0;

    int buffer_index = 0;
    int sock = -1;
    int64_t last_flush_time = esp_timer_get_time();

    uart_event_t event;

    while (1) {
        if (sock < 0) {
            while ((sock = connect_to_server()) < 0) {
                ESP_LOGI(TAG, "Reconnecting in 2 seconds...");
                vTaskDelay(pdMS_TO_TICKS(2000));
            }

            while (!queue_is_empty()) {
                uint8_t temp[SEND_BUFFER_SIZE];
                if (queue_pop(temp)) {
                    int sent = send(sock, temp, SEND_BUFFER_SIZE, 0);
                    if (sent < 0) {
                        ESP_LOGW(TAG, "Reconnect failed to flush queue");
                        close(sock);
                        sock = -1;
                        break;
                    }
                }
            }
        }

        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    int len = uart_read_bytes(UART_NUM, uart_buf, event.size, portMAX_DELAY);
                    int offset = 0;

                    // Prepend leftover bytes from last read
                    if (partial_len > 0) {
                        int to_copy = PACKET_SIZE - partial_len;
                        if (len >= to_copy) {
                            memcpy(&partial_buf[partial_len], uart_buf, to_copy);
                            if (buffer_index + PACKET_SIZE > SEND_BUFFER_SIZE) {
                                int sent = send(sock, send_buffer, buffer_index, 0);
                                if (sent < 0) {
                                    ESP_LOGE(TAG, "Send failed, queueing...");
                                    if (!queue_push(send_buffer)) {
                                        ESP_LOGW(TAG, "Queue full, dropping packet");
                                    }
                                    close(sock);
                                    sock = -1;
                                    buffer_index = 0;
                                    break;
                                }
                                buffer_index = 0;
                                last_flush_time = esp_timer_get_time();
                            }
                            memcpy(&send_buffer[buffer_index], partial_buf, PACKET_SIZE);
                            buffer_index += PACKET_SIZE;
                            offset += to_copy;
                            partial_len = 0;
                        } else {
                            memcpy(&partial_buf[partial_len], uart_buf, len);
                            partial_len += len;
                            break;
                        }
                    }

                    while ((len - offset) >= PACKET_SIZE) {
                        if (buffer_index + PACKET_SIZE > SEND_BUFFER_SIZE) {
                            int sent = send(sock, send_buffer, buffer_index, 0);
                            if (sent < 0) {
                                ESP_LOGE(TAG, "Send failed, queueing...");
                                if (!queue_push(send_buffer)) {
                                    ESP_LOGW(TAG, "Queue full, dropping packet");
                                }
                                close(sock);
                                sock = -1;
                                buffer_index = 0;
                                break;
                            }
                            buffer_index = 0;
                            last_flush_time = esp_timer_get_time();
                        }
                        memcpy(&send_buffer[buffer_index], &uart_buf[offset], PACKET_SIZE);
                        buffer_index += PACKET_SIZE;
                        offset += PACKET_SIZE;
                    }

                    // Store leftover bytes
                    if (offset < len) {
                        partial_len = len - offset;
                        memcpy(partial_buf, &uart_buf[offset], partial_len);
                    }
                    break;
                }
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART overflow or buffer full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                case UART_PARITY_ERR:
                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "UART error: %d", event.type);
                    break;
                default:
                    break;
            }
        }

        int64_t now = esp_timer_get_time();
        if (buffer_index > 0 && (now - last_flush_time) > 100000) {
            int sent = send(sock, send_buffer, buffer_index, 0);
            if (sent < 0) {
                ESP_LOGE(TAG, "Flush send failed, queueing...");
                if (!queue_push(send_buffer)) {
                    ESP_LOGW(TAG, "Queue full, dropping flush packet");
                }
                close(sock);
                sock = -1;
                buffer_index = 0;
                continue;
            }
            buffer_index = 0;
            last_flush_time = now;
        }
    }
}

void app_main(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_DMA_SIZE, 20, &uart_queue, 0);

    xTaskCreate(uart_socket_task, "uart_socket_task", 4096, NULL, 10, NULL);
}
