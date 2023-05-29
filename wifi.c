#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// UART pins for communication with ESP8266
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0 //GP0 ---> RX
#define UART_RX_PIN 1 //GP1 ---> TX

// Wi-Fi credentials
#define WIFI_SSID "Marx"
#define WIFI_PASSWORD "16deenero"

// API details
#define API_HOST "localhost"
#define API_PORT 8002
#define API_PATH "/load_data"

"""
Send AT commands to the ESP8266 module over the UART connection
"""
void sendATCommand(const char* command) {
    uart_puts(UART_ID, command);
    uart_puts(UART_ID, "\r\n");
}

"""
Configures the ESP8266 module to operate in client mode and connects 
it to the Wi-Fi network. It sends the AT command 'AT+CWMODE=1' to set the 
module's mode to client mode. Then, it dynamically generates the AT command 
to connect to the Wi-Fi network using the SSID and password specified in the 
WIFI_SSID and WIFI_PASSWORD constants, respectively.
"""
void connectToWiFi() {
    // Set ESP8266 as client
    sendATCommand("AT+CWMODE=1");
    // Connect to Wi-Fi network
    char connectCommand[64];
    sprintf(connectCommand, "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
    sendATCommand(connectCommand);
}

"""
Establishes a TCP connection to the API server and sends an HTTP GET request. It sends the AT command 
to initiate a TCP connection with the specified API host on port 8000 (the standard HTTP port). 
It then generates an HTTP GET request string in the request buffer, including the API path and 
host in the appropriate format.
"""
void sendHTTPRequest() {
    // Establish TCP connection to API server
    char connectCommand[64];
    sprintf(connectCommand, "AT+CIPSTART=\"TCP\",\"%s\",%d", API_HOST, API_PORT);
    sendATCommand(connectCommand);
    // Prepare HTTP request
    char request[256];
    sprintf(request, "PUT %s HTTP/1.1\r\nHost: %s:%d\r\nContent-Length: 1\r\n\r\n5", API_PATH, API_HOST, API_PORT);

    // // Prepare HTTP request with payload
    // char request[256];
    // int contentLength = snprintf(request, sizeof(request), "PUT %s HTTP/1.1\r\nHost: %s:%d\r\nContent-Length: %d\r\n\r\n", API_PATH, API_HOST, API_PORT, sizeof(float)*3 + sizeof(int)*6);
    // char* requestBody = request + contentLength;
    // requestBody += snprintf(requestBody, sizeof(request) - contentLength, "%.2f, %.2f, %.2f, %d, %d, %d, %d, %d, %d", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, magne_x, magne_y, magne_z);


    // Send HTTP request
    char sendCommand[64];
    sprintf(sendCommand, "AT+CIPSEND=%d", strlen(request));
    sendATCommand(sendCommand);
    sendATCommand(request);
}

int main() {
    stdio_init_all();

    // Configure UART pins
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);

    // Connect to Wi-Fi
    connectToWiFi();

    // // Set the values
    // float accel_x = 1.23;
    // float accel_y = 2.34;
    // float accel_z = 3.45;
    // int gyro_x = 100;
    // int gyro_y = 200;
    // int gyro_z = 300;
    // int magne_x = 400;
    // int magne_y = 500;
    // int magne_z = 600;

    // // Send HTTP request with the values
    // sendHTTPRequest(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, magne_x, magne_y, magne_z);

    // Send HTTP request
    sendHTTPRequest();

    while (1) {
        // Handle other tasks or wait for response from the server
        if (uart_is_readable(UART_ID)) {
            char receivedChar = uart_getc(UART_ID);
            // Ignore response headers
            if (receivedChar == '\r') {
                continue;
            }
            if (receivedChar == '\n') {
                if (responseStarted) {
                    // End of response
                    break;
                } else {
                    // Start of response
                    responseStarted = true;
                }
            }
            // Print response
            if (responseStarted) {
                putchar(receivedChar);
            }
        }
    }
    return 0;
}
