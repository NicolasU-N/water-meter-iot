#ifndef RADIO_LORA_E5_H
#define RADIO_LORA_E5_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"

#include "driver/uart.h"

#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024
#define ATTEMPTS 10

extern const char *TAG_UART;

void send_AT(const char *cmd);
char *receive_uart();
void test_uart_connection();
void get_eui_from_radio();
void set_app_key(char *app_key);
void configure_regional_settings(char *band, char *DR, char *channels);
bool join_the_things_network();
void send_message(char *message);
void set_radio_low_power();

#endif
