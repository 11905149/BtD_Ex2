/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "sensors.h"
#include "screen.h"
#include "sdkconfig.h"
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h>            // struct addrinfo
#include <arpa/inet.h>
#include "esp_netif.h"
#include "esp_log.h"
#if defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
#include "addr_from_stdin.h"
#include "esp_random.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";
int sock = -1;
char rx_buffer[128];
char host_ip[] = HOST_IP_ADDR;

void tcp_client_connect()
{
    int addr_family = 0;
    int ip_protocol = 0;
    struct sockaddr_in dest_addr;
    inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    while (sock < 0) {
    write_screen("connecting", "to server...");
        sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            sock = -1;
        }
        ESP_LOGI(TAG, "Waiting to connect..");

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            sock = -1;
        }
        if (sock < 0) {
            int seconds = 5;
            while (seconds > 0) {
                char displayText[50];
                sprintf(displayText, "retrying in %ds", seconds);
                write_screen("connection error", displayText);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                seconds--;
            }
        }
    }
    ESP_LOGI(TAG, "Connected");
    
}

char * tcp_client_send(char *payload, char *response_buffer)
{
    if (sock < 0) {
        ESP_LOGW(TAG, "Socket not connected");
        tcp_client_connect();
    }
    int err = send(sock, payload, strlen(payload), 0);
    ESP_LOGI(TAG, "Sent payload");
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        sock = -1;
        tcp_client_send(payload, response_buffer);
    }

    int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
    // Error occurred during receiving
    if (len < 0) {
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        sock = -1;
    }
    // Data received
    else {
        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
        ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
        ESP_LOGI(TAG, "%s", rx_buffer);
    }
    strcpy(response_buffer, rx_buffer);
    return 0;
}
