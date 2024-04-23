#include "sensors.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_event.h"
#include "screen.h"
#include "esp_log.h"

#define AXIS_COUNT 3
#define MAX_LINE_LENGTH 50
#define TOTALDURATION 3000  // total time in ms
#define DELAYDURATION 100    // delay per loop iteration in ms
#define NUMCHUNKS 5
#define SAMPLES (TOTALDURATION / DELAYDURATION)  // calculating number of samples

static const char *TAG = "main";


extern void tcp_client_send(char *payload, char *response_buffer);
extern void tcp_client_connect(void);

void arrayToCSV(float data[][AXIS_COUNT], int samples, char* buffer) {
    int pos = 0;  // Position in the buffer
    for (int i = 0; i < samples; i++) {
        // Write one line of CSV data; ensure not to overflow the buffer
        int written = snprintf(buffer + pos, MAX_LINE_LENGTH, "%f,%f,%f\n", data[i][0], data[i][1], data[i][2]);
        if (written > 0) {
            pos += written;  // Increase the buffer position
        }
        if (pos >= SAMPLES * MAX_LINE_LENGTH) {
            // Prevent buffer overflow
            break;
        }
    }
}


void main_task(void *pvParameters)
{
    ESP_ERROR_CHECK(i2c_master_init());
    screen_main("");

    write_screen("initializing", "connecting to Wifi");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    ESP_LOGI(TAG, "I2C initialized successfully");

    init_mpu6886();

    float ax, ay, az;
    float accelData[SAMPLES][AXIS_COUNT];  // Array to hold accelerometer data

    int current_chunk = 0;

    //connect to server
    tcp_client_connect();
    
    
    while (current_chunk < NUMCHUNKS)
    {
        char displayText[50];
        sprintf(displayText, "chunk %d/%d", current_chunk+1, NUMCHUNKS);
        write_screen(displayText, "collecting data...");
        int i = 0;
        while (i < SAMPLES)
        {
            getAccelData(&ax, &ay, &az);
            accelData[i][0] = ax;  // Store x-axis data
            accelData[i][1] = ay;  // Store y-axis data
            accelData[i][2] = az;  // Store z-axis data
            ESP_LOGI(TAG, "Accel ADC: ax=%f, ay=%f, az=%f", ax, ay, az);
            vTaskDelay(DELAYDURATION / portTICK_PERIOD_MS);
            i++;
        }

        char csvBuffer[SAMPLES * MAX_LINE_LENGTH];
        arrayToCSV(accelData, SAMPLES, csvBuffer);
        ESP_LOGI(TAG, "CSV data: %s", csvBuffer);
        write_screen(displayText, "sending...");
        char response_buffer[128];
        tcp_client_send(csvBuffer, response_buffer);

        ESP_LOGI(TAG, "Sent chunk %d", current_chunk);
        current_chunk++;
    }
    char response_buffer[24];
    tcp_client_send("end", response_buffer);
    write_screen("collection complete", response_buffer);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
}

void app_main() {
    xTaskCreate(main_task, "main_task", 32768, NULL, 10, NULL);
    vTaskDelete(NULL); // Delete the initial app_main task to recover its stack space
}