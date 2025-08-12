#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>

#define HOST_BAUD_RATE 115200
#define HOST_TX_PIN UART_PIN_NO_CHANGE
#define HOST_RX_PIN UART_PIN_NO_CHANGE
#define DATA_BAUD_RATE 115200
#define DATA_TX_PIN 1
#define DATA_RX_PIN 2

#define UART_BUF_SIZE (1024 * 4) // UART buffer
#define UART_QUEUE_SIZE 20
#define DMA_BUF_SIZE 1024

static const char *TAG = "UART_DMA";
static QueueHandle_t uart0_queue;
static QueueHandle_t uart2_queue;

static uint8_t dma_buffer[DMA_BUF_SIZE]; // DMA buffer
static SemaphoreHandle_t dma_semaphore;

typedef struct
{
    uart_port_t uart_num;
    uint8_t *data;
    size_t size;
} uart_dma_data_t;

void uart_setup_with_queue(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate, QueueHandle_t *queue)
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUF_SIZE, UART_BUF_SIZE, UART_QUEUE_SIZE, queue, 0)); // event queue for DMA
    uart_enable_pattern_det_baud_intr(uart_num, '\n', 1, 9, 0, 0);                                           // packet detection
    uart_pattern_queue_reset(uart_num, UART_QUEUE_SIZE);
    ESP_LOGI(TAG, "UART%d configured with DMA-style queue - Baud: %d", uart_num, baud_rate);
}

void uart_dma_process_task(void *arg)
{
    uart_port_t uart_num = (uart_port_t)(uintptr_t)arg;
    QueueHandle_t *queue = (uart_num == UART_NUM_0) ? &uart0_queue : &uart2_queue;
    uart_event_t event;
    uint8_t *temp_buffer = malloc(DMA_BUF_SIZE);

    if (!temp_buffer)
    {
        ESP_LOGE(TAG, "Failed to allocate DMA temp buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "DMA processing task started for UART%d", uart_num);

    while (1)
    {
        if (xQueueReceive(*queue, (void *)&event, pdMS_TO_TICKS(100)))
        {
            switch (event.type)
            {
            case UART_DATA:
                if (event.size > 0)
                {
                    if (xSemaphoreTake(dma_semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                    {
                        size_t read_size = (event.size > DMA_BUF_SIZE) ? DMA_BUF_SIZE : event.size;
                        int len = uart_read_bytes(uart_num, dma_buffer, read_size, 0);
                        if (len > 0)

                        {
                            ESP_LOGI(TAG, "DMA UART%d: Processed %d bytes", uart_num, len);

                            for (int i = 0; i < len; i += 64)
                            {
                                int chunk_size = (len - i > 64) ? 64 : (len - i);

                                uint32_t checksum = 0;
                                for (int j = i; j < i + chunk_size; j++)
                                {
                                    checksum += dma_buffer[j];
                                }

                                ESP_LOGD(TAG, "Chunk %d: size=%d, checksum=0x%08lX", i / 64, chunk_size, checksum);
                            }

                            // Echo processed data back
                            // uart_write_bytes(uart_num, "DMA_ACK:", 8);
                            // uart_write_bytes(uart_num, dma_buffer, (len > 32) ? 32 : len);
                            // uart_write_bytes(uart_num, "\r\n", 2);
                        }

                        xSemaphoreGive(dma_semaphore);
                    }
                }
                break;

            case UART_PATTERN_DET:
                ESP_LOGI(TAG, "Pattern detected on UART%d", uart_num);

                int pos = uart_pattern_pop_pos(uart_num);
                if (pos != -1)
                {
                    ESP_LOGI(TAG, "Pattern at position: %d", pos);
                    // read up to pattern
                    int len = uart_read_bytes(uart_num, temp_buffer, pos + 1, 0);
                    if (len > 0)
                    {
                        temp_buffer[len] = '\0';
                        ESP_LOGI(TAG, "Pattern data: %s", (char *)temp_buffer);
                    }
                }
                break;

            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "UART%d FIFO overflow - consider increasing buffer", uart_num);
                uart_flush_input(uart_num);
                break;

            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART%d buffer full - processing too slow", uart_num);
                uart_flush_input(uart_num);
                break;

            case UART_BREAK:
                ESP_LOGI(TAG, "UART%d break detected", uart_num);
                break;

            case UART_PARITY_ERR:
                ESP_LOGW(TAG, "UART%d parity error", uart_num);
                break;

            case UART_FRAME_ERR:
                ESP_LOGW(TAG, "UART%d frame error", uart_num);
                break;

            default:
                ESP_LOGW(TAG, "UART%d unknown event type: %d", uart_num, event.type);
                break;
            }
        }
    }

    free(temp_buffer);
    vTaskDelete(NULL);
}

void uart_high_speed_reader_task(void *arg)
{
    uart_port_t uart_num = (uart_port_t)(uintptr_t)arg;
    uint8_t *data = malloc(DMA_BUF_SIZE);

    if (!data)
    {
        ESP_LOGE(TAG, "Failed to allocate high-speed buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "High-speed reader task started for UART%d", uart_num);

    while (1)
    {
        int len = uart_read_bytes(uart_num, data, DMA_BUF_SIZE, pdMS_TO_TICKS(1)); // non blocking

        if (len > 0)
        {
            ESP_LOGI(TAG, "High-speed read: %d bytes on UART%d", len, uart_num);

            for (int i = 0; i < len; i += 4) // 4 byte chunks
            {
                uint32_t *word_ptr = (uint32_t *)(data + i);
                if (i + 4 <= len)
                {
                    uint32_t word = *word_ptr;
                    ESP_LOGV(TAG, "Word %d: 0x%08lX", i / 4, word);
                }
            }

            // uart_write_bytes(UART_NUM_2, data, len);
        }

        // minimal delay to prevent watchdog issues
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    free(data);
    vTaskDelete(NULL);
}

void uart_status_task(void *arg)
{
    while (1)
    {
        // Monitor UART buffer usage and performance
        size_t uart0_buffered = 0;
        size_t uart2_buffered = 0;

        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, &uart0_buffered));
        if (uart2_queue)
        {
            ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, &uart2_buffered));
        }

        ESP_LOGI(TAG, "UART Status - UART0: %d bytes, UART2: %d bytes, Free heap: %lu",
                 uart0_buffered, uart2_buffered, esp_get_free_heap_size());

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Starting ESP32 UART DMA Application");

    // Create semaphore for DMA buffer protection
    dma_semaphore = xSemaphoreCreateMutex();
    if (!dma_semaphore)
    {
        ESP_LOGE(TAG, "Failed to create DMA semaphore");
        return;
    }

    // Setup UART0 (USB) with event queue
    uart_setup_with_queue(UART_NUM_0, HOST_TX_PIN, HOST_RX_PIN, HOST_BAUD_RATE, &uart0_queue);

    // Setup UART2 (Data) with event queue - uncomment if needed
    // uart_setup_with_queue(UART_NUM_2, DATA_TX_PIN, DATA_RX_PIN, DATA_BAUD_RATE, &uart2_queue);

    // Create DMA processing tasks with high priority
    xTaskCreatePinnedToCore(uart_dma_process_task, "uart0_dma", 4096,
                            (void *)(uintptr_t)UART_NUM_0, 15, NULL, 1);

    // Alternative high-speed reader (choose one approach)
    // xTaskCreatePinnedToCore(uart_high_speed_reader_task, "uart0_fast", 4096,
    //                        (void *)(uintptr_t)UART_NUM_0, 14, NULL, 1);

    // Create status monitoring task
    xTaskCreate(uart_status_task, "uart_status", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "All DMA tasks created successfully");

    // Send test data periodically
    uint32_t counter = 0;
    while (1)
    {
        char test_msg[128];
        int len = snprintf(test_msg, sizeof(test_msg),
                           "DMA Test #%lu - Send some data to test DMA processing\n",
                           counter++);
        uart_write_bytes(UART_NUM_0, test_msg, len);

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}