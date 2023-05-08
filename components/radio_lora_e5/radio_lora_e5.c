#include "radio_lora_e5.h"

/**
 * @brief Send AT command to the module
 *
 * @param cmd
 */
void send_AT(const char *cmd)
{
    char msg[TX_BUF_SIZE];
    sprintf(msg, "AT%s\r\n", cmd); // agregamos el prefijo "AT" y el sufijo "\r\n"
    uart_write_bytes(UART_NUM_2, msg, strlen(msg));
}

/**
 * @brief Read response from the module
 *
 * @return char*
 */
char *receive_uart()
{
    char *data = (char *)malloc(RX_BUF_SIZE + 1);
    int len = 0;
    while (len == 0)
    {
        len = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, pdMS_TO_TICKS(10));
    }
    data[len] = '\0';
    ESP_LOGI(TAG_UART, "RX data: %s", data);
    return data;
}

/**
 * @brief Función para verificar la conexión UART enviando un comando AT vacío
 *
 */
void test_uart_connection()
{
    send_AT("");
    char *data = receive_uart();

    if (strcmp(data, "+AT: OK\r\n") == 0)
    {
        ESP_LOGI(TAG_UART, "LoRa radio is ready");
        free(data);
    }
    else
    {
        ESP_LOGE(TAG_UART, "LoRa-E5 not detected");
        free(data);
        exit(1);
    }
}

/**
 * @brief Get the eui from radio object and print it
 *
 */
void get_eui_from_radio()
{
    char *device_eui = NULL;
    char *join_eui = NULL;

    send_AT("+ID=DevEui");
    device_eui = receive_uart();
    if (device_eui != NULL)
    {
        device_eui = strtok(device_eui, " ");
        device_eui = strtok(NULL, " ");
        // printf("DevEUI: %s\n", device_eui);
        ESP_LOGI(TAG_UART, "DevEUI: %s", device_eui);
    }

    send_AT("+ID=AppEui");
    join_eui = receive_uart();
    if (join_eui != NULL)
    {
        join_eui = strtok(join_eui, " ");
        join_eui = strtok(NULL, " ");
        // printf("JoinEUI: %s\n", join_eui);
        ESP_LOGI(TAG_UART, "JoinEUI: %s", join_eui);
    }
}

/**
 * @brief Set the app key object
 *
 * @param app_key
 */
void set_app_key(char *app_key)
{
    char command[64];
    sprintf(command, "+KEY=APPKEY,\"%s\"", app_key);
    send_AT(command);
    receive_uart();
    // printf("AppKey: %s\n", app_key);
    ESP_LOGI(TAG_UART, "AppKey: %s", app_key);
}

void configure_regional_settings(char *band, char *DR, char *channels)
{
    // Configurar el ancho de banda
    // send_AT("+DR=" + band);
    //    char *band_to_send = "+DR=";
    //	strcat(band_to_send, band);
    char band_to_send[16];
    sprintf(band_to_send, "+DR=%s", band);
    send_AT(band_to_send);

    // Configurar la tasa de datos
    // send_AT("+DR=" + DR);
    //    char *dr = "+DR=";
    //    strcat(dr, DR);
    char dr[16];
    sprintf(dr, "+DR=%s", DR);
    send_AT(dr);

    // Configurar los canales
    // send_AT("+CH=NUM," + channels);
    //    char *ch_num = "+CH=NUM,";
    //    strcat(ch_num, channels);
    char ch_num[16];
    sprintf(ch_num, "+CH=NUM,%s", channels);
    send_AT(ch_num);

    // Configurar el modo
    send_AT("+MODE=LWOTAA");

    // Vaciar el buffer de entrada
    receive_uart();

    // Verificar la configuración del ancho de banda
    send_AT("+DR");
    char *data = receive_uart();
    printf("%s", data);

    // Verificar la configuración de los canales
    // send_AT("+CH=NUM");
    // data = receive_uart();
    // printf("%s", data);

    // Listar los canales disponibles
    // send_AT("+CH=?");
    // data = receive_uart();
    // printf("%s", data);
}

bool join_the_things_network()
{
    // Enviar comando de unión
    send_AT("+JOIN");
    char *data = receive_uart();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Esperar hasta que se establezca la conexión o falle
    uint8_t retry_count = 0;
    while (retry_count < ATTEMPTS)
    {
        data = receive_uart();
        ESP_LOGW(TAG_UART, "Waiting... %d", retry_count);
        if (strstr(data, "joined") != NULL)
        {
            ESP_LOGI(TAG_UART, "Connected!!");
            return true;
        }
        if (strstr(data, "failed") != NULL)
        {
            ESP_LOGE(TAG_UART, "Join Failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(50));
            send_AT("+JOIN"); // Enviar comando de unión
            retry_count++;
        }
        // vTaskDelay(pdMS_TO_TICKS(500));
    }
    // printf("Unable to join after 10 retries, giving up.\n");
    ESP_LOGE(TAG_UART, "Unable to join after 10 retries, giving up.");
    return false;
}

void send_message(char *message)
{
    // Enviar mensaje
    char send_msg[TX_BUF_SIZE - 5];
    snprintf(send_msg, TX_BUF_SIZE, "+MSG=\"%s\"", message);
    send_AT(send_msg);

    // Esperar hasta que se complete el envío o falle
    int done = 0;
    while (!done)
    {
        char *data = receive_uart();
        if (strstr(data, "Done") != NULL || strstr(data, "ERROR") != NULL)
        {
            done = 1;
        }
        if (strlen(data) > 0)
        {
            printf("%s", data);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void set_radio_low_power()
{
    // Enviar cmd
    send_AT("+LOWPOWER");
    vTaskDelay(pdMS_TO_TICKS(15)); // datasheet pag. 48 - 49
}
