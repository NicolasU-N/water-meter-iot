/* Deep sleep wake up example

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>
#include <ctype.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_timer.h"
#include "esp_log.h"

#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "eeprom_24lc256.h"
#include "radio_lora_e5.h"

// TIME CONSTANTS
#define WAKEUP_TIME_SEC 60	   /*!< Sleep time in seconds */
#define VOLUME_CALC_TIME_SEC 3 /*!< Calculate volume time in seconds */

//========================================================================= PIN CONSTANTS
#define EXT_WAKEUP_PIN_0 27 /*!< Reed switch pin */
// #define FLOW_SENSOR_PIN 4
#define EN_BAT_PIN 13

// ADC
#define V_BAT_ADC1_CHAN6 ADC_CHANNEL_6 // V_BAT_PIN 34
#define MIN_VOLTAGE 610				   // voltaje mínimo en mV
#define MAX_VOLTAGE 1010			   // voltaje máximo en mV

// UART
#define TX_PIN (GPIO_NUM_17)
#define RX_PIN (GPIO_NUM_16)
#define BUF_SIZE (1024)

// I2C
#define SDA_PIN 21					/*!< GPIO number used for I2C master data  */
#define SCL_PIN 22					/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

// LORA CONSTANTS
#define APP_KEY "99887766554433221199887766554433"
#define CHANNELS "0-15"
#define BAND "AU915"
#define DR "1"

// WATER METER CONSTANT
#define DEBOUNCE_DELAY 200 // Intervalo mínimo entre rebotes en milisegundos
#define PULSE_FACTOR 2	   // Nummber of blinks per L of your meter
#define get_volume(_pulse_count) (float)(_pulse_count / (float)PULSE_FACTOR)
uint32_t last_debounce_time = 0;
uint16_t pulse_count = 0;

static RTC_DATA_ATTR struct timeval sleep_enter_time;

const char *TAG_I2C = "I2C";
const char *TAG_UART = "UART";
const char *TAG_ADC = "ADC";
const char *TAG_TIMER = "TIMER";
const char *TAG_EEPROM = "EEPROM";
const char *TAG_BATTERY = "BAT";
const char *TAG_RS_INTERRUP = "RS_INTERRUP";

esp_err_t init_pins(void);
esp_err_t init_esp_sleep_conf(void);
esp_err_t is_first_boot(void);
void rst_one_shot_timer(void);
float read_voltage(void);
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);

QueueHandle_t interput_queue;

esp_timer_handle_t timer_handle;

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
bool do_calibration1;

TaskHandle_t int_queue_handler_task_handle;
TaskHandle_t send_data_to_radio_handle;

void timer_callback(void *arg)
{
	if (pulse_count != 0)
	{
		//? read the volume from eeprom and sum it with the current volume
		float volume;
		esp_err_t ret1 = eeprom_read_float(0x0, &volume);
		if (ret1 == ESP_OK)
		{
			ESP_LOGI(TAG_EEPROM, "Volume read from EEPROM: %f\n", volume);
		}

		volume += get_volume(pulse_count);
		ESP_LOGI(TAG_TIMER, "Volume: %f", volume);

		ret1 = eeprom_update_float(0x0, volume);
		if (ret1 == ESP_OK)
		{
			// printf("Volume written to EEPROM successfully!\n");
			ESP_LOGI(TAG_EEPROM, "Volume written to EEPROM successfully!");
			float read_vol;
			ret1 = eeprom_read_float(0x0, &read_vol);
			if (ret1 == ESP_OK)
			{
				// printf("Volume read from EEPROM: %f\n", read_vol);
				ESP_LOGI(TAG_EEPROM, "Current volume: %f", read_vol);
			}
		}
	}

	//======================================================= DEEP SLEEP
	// printf("Timer expired, Entering deep sleep\n");
	ESP_LOGW(TAG_TIMER, "Timer expired, Entering deep sleep");

	// ========================================== GPIO WAKEUP
	printf("Enabling EXT0 wakeup on pin GPIO%d\n", EXT_WAKEUP_PIN_0);

	esp_sleep_enable_ext0_wakeup(EXT_WAKEUP_PIN_0, !gpio_get_level(EXT_WAKEUP_PIN_0));

	// Configure pullup/downs via RTCIO to tie wakeup pins to inactive level during deepsleep.
	// EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
	// No need to keep that power domain explicitly, unlike EXT1.
	rtc_gpio_set_direction(EXT_WAKEUP_PIN_0, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_set_direction_in_sleep(EXT_WAKEUP_PIN_0, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_pullup_en(EXT_WAKEUP_PIN_0);
	rtc_gpio_pulldown_dis(EXT_WAKEUP_PIN_0);
	// ==========================================

	gettimeofday(&sleep_enter_time, NULL);
	esp_deep_sleep_start();
	//=======================================================
}

/**
 * @brief manejador de interrupciones para el reed switch
 *
 * @param args
 */
static void IRAM_ATTR int_handler_reed_sw(void *args)
{
	uint32_t current_time = xTaskGetTickCount(); // Obtener el tiempo actual

	// Verificar si ha pasado suficiente tiempo desde el último rebote
	if (current_time - last_debounce_time >= DEBOUNCE_DELAY)
	{
		// Procesar la interrupción aquí
		int pin_number = (int)args;
		xQueueSendFromISR(interput_queue, &pin_number, NULL);
		// Actualizar el tiempo de último rebote
		last_debounce_time = current_time;
	}
}

/**
 * @brief tarea para manejar la cola de interrupciones
 *
 * @param pvParameters
 */
void int_queue_handler_task(void *pvParameters)
{
	int pin_number = 0;
	while (true)
	{
		if (xQueueReceive(interput_queue, &pin_number, portMAX_DELAY))
		{
			rst_one_shot_timer();
			if (pin_number == EXT_WAKEUP_PIN_0)
			{
				pulse_count++;
				ESP_LOGI(TAG_RS_INTERRUP, "Pulse count: %d", pulse_count);
			}
		}
		else
		{
			ESP_LOGE(TAG_RS_INTERRUP, "Error receiving from interput_queue");
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

/**
 * @brief tarea para enviar los datos a la estación base
 *
 * @param pvParameters
 */
void send_data_to_radio(void *pvParameters)
{
	//? reset the timer so it will be triggered again in 2 seconds rst_one_shot_timer
	esp_timer_stop(timer_handle); // detener el timer

	test_uart_connection();

	get_eui_from_radio();

	set_app_key(APP_KEY);

	configure_regional_settings(BAND, DR, CHANNELS);

	// Send  data
	if (join_the_things_network())
	{

		//? read voltage from battery
		float v_bat = read_voltage(); // voltaje en mV
		ESP_LOGI(TAG_BATTERY, "voltage read from battery mV: %f\n", v_bat);

		float battery_percentage = ((v_bat - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0;

		if (battery_percentage < 0)
		{
			battery_percentage = 0;
		}
		else if (battery_percentage > 100)
		{
			battery_percentage = 100;
		}

		ESP_LOGI(TAG_BATTERY, "Battery percentage: %f%%", battery_percentage);
		// printf("Battery percentage: %f%%\n", battery_percentage);

		//? send the volume to the gateway
		ESP_LOGI(TAG_UART, "sending messages...");

		//? read the volume from eeprom and sum it with the current volume
		float volume;
		esp_err_t ret1 = eeprom_read_float(0x0, &volume);
		if (ret1 == ESP_OK)
		{
			ESP_LOGI(TAG_EEPROM, "initial_volume read from EEPROM: %f\n", volume);
		}

		volume += get_volume(pulse_count);

		char msg[64];
		snprintf(msg, sizeof(msg), "{'vol':%.1f,'batt_lvl':%.1f}", volume, battery_percentage);
		ESP_LOGI(TAG_UART, "msg: %s", msg);

		send_message(msg);
	}
	vTaskDelay(pdMS_TO_TICKS(100));

	set_radio_low_power();

	//? init the timer again
	esp_timer_start_once(timer_handle, VOLUME_CALC_TIME_SEC * 1000000); // 2 seconds in microseconds
	// Se elimina la tarea actual
	vTaskDelete(NULL);
}

void app_main(void)
{
	// ?Inicialización de la cola de interrupciones
	interput_queue = xQueueCreate(16, sizeof(int));

	xTaskCreate(int_queue_handler_task, "int_queue_handler_task", configMINIMAL_STACK_SIZE * 5, NULL, configMAX_PRIORITIES - 3, &int_queue_handler_task_handle);
	xTaskCreate(send_data_to_radio, "send_data_to_radio", configMINIMAL_STACK_SIZE * 5, NULL, configMAX_PRIORITIES - 2, &send_data_to_radio_handle);
	vTaskSuspend(send_data_to_radio_handle);

	//? Inicialización de pines
	init_pins();

	//? Init sleep config
	init_esp_sleep_conf();

	//? Init timer
	esp_timer_create_args_t timer_args = {
		.callback = &timer_callback,
		.arg = NULL,
		.name = "my_timer"};
	esp_timer_create(&timer_args, &timer_handle);

	//? Validar si es el primer arranque
	is_first_boot();

	//? erase memory
	// eeprom_erase_all();

	//? read voltage from battery
	// read_voltage();

	rst_one_shot_timer();
}

/**
 * @brief Inicializa los pines a utilizar
 *
 * @return esp_err_t
 */
esp_err_t init_pins()
{
	// Configurar los pines de la interfaz serial
	uart_config_t uart_config =
		{.baud_rate = 9600,
		 .data_bits = UART_DATA_8_BITS,
		 .parity = UART_PARITY_DISABLE,
		 .stop_bits = UART_STOP_BITS_1,
		 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		 .source_clk = UART_SCLK_DEFAULT};
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
	ESP_ERROR_CHECK(
		uart_set_pin(UART_NUM_2,
					 TX_PIN, RX_PIN,
					 UART_PIN_NO_CHANGE,
					 UART_PIN_NO_CHANGE));
	//  esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size, int queue_size, QueueHandle_t *uart_queue, int intr_alloc_flags)
	ESP_ERROR_CHECK(
		uart_driver_install(UART_NUM_2, BUF_SIZE, 0, 0, NULL, 0));

	// Configurar los pines de la interfaz I2C
	i2c_config_t i2c_config =
		{.mode = I2C_MODE_MASTER,
		 .sda_io_num = SDA_PIN,
		 .sda_pullup_en = GPIO_PULLUP_ENABLE,
		 .scl_io_num = SCL_PIN,
		 .scl_pullup_en = GPIO_PULLUP_ENABLE,
		 .master.clk_speed = 100000};

	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
	ESP_ERROR_CHECK(
		i2c_driver_install(I2C_NUM_0,
						   I2C_MODE_MASTER,
						   I2C_MASTER_RX_BUF_DISABLE,
						   I2C_MASTER_TX_BUF_DISABLE,
						   0));

	// Configurar el pin REED_SW como entrada con pull up
	rtc_gpio_deinit(EXT_WAKEUP_PIN_0);
	gpio_config_t reed_sw_config =
		{.pin_bit_mask = (1ULL << EXT_WAKEUP_PIN_0),
		 .mode = GPIO_MODE_INPUT,
		 .pull_up_en = GPIO_PULLUP_ENABLE,
		 .pull_down_en = GPIO_PULLDOWN_DISABLE,
		 .intr_type = GPIO_INTR_POSEDGE};
	ESP_ERROR_CHECK(gpio_config(&reed_sw_config));

	// Configurar interrupción para el pin REED_SW
	gpio_install_isr_service(0);
	gpio_isr_handler_add(EXT_WAKEUP_PIN_0, int_handler_reed_sw, (void *)EXT_WAKEUP_PIN_0);

	// Configurar el pin FLOW_SENSOR como entrada con pull up
	// gpio_config_t flow_sensor_config =
	// 	{.pin_bit_mask = (1ULL << FLOW_SENSOR_PIN),
	// 	 .mode = GPIO_MODE_INPUT,
	// 	 .pull_up_en = GPIO_PULLUP_ENABLE,
	// 	 .intr_type = GPIO_INTR_NEGEDGE};
	// ESP_ERROR_CHECK(gpio_config(&flow_sensor_config));

	// Configurar el pin EN_BAT como salida
	gpio_config_t en_bat_config =
		{.pin_bit_mask = (1ULL << EN_BAT_PIN),
		 .mode = GPIO_MODE_OUTPUT,
		 .pull_up_en = GPIO_PULLUP_DISABLE,
		 .pull_down_en = GPIO_PULLDOWN_DISABLE,
		 .intr_type = GPIO_INTR_DISABLE};
	ESP_ERROR_CHECK(gpio_config(&en_bat_config));

	// Configurar el pin V_BAT como entrada analógica
	//-------------ADC1 Init---------------//
	adc_oneshot_unit_init_cfg_t init_config1 = {
		.unit_id = ADC_UNIT_1,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

	//-------------ADC1 Config---------------//
	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.atten = ADC_ATTEN_DB_11,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, V_BAT_ADC1_CHAN6, &config));

	//-------------ADC1 Calibration Init---------------//
	do_calibration1 = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);
	return ESP_OK;
}

/**
 * @brief Inicializa la configuración de sleep
 *
 * @return esp_err_t
 */
esp_err_t init_esp_sleep_conf()
{
	struct timeval now;
	gettimeofday(&now, NULL);
	int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

	switch (esp_sleep_get_wakeup_cause())
	{
	case ESP_SLEEP_WAKEUP_EXT0:
	{
		printf("Wake up from ext0 level -> %d\n", (int)rtc_gpio_get_level(EXT_WAKEUP_PIN_0));
		int pin_number = EXT_WAKEUP_PIN_0;
		if (!xQueueSend(interput_queue, &pin_number, 0))
		{
			ESP_LOGE(TAG_RS_INTERRUP, "Error sending to interput_queue, %i", EXT_WAKEUP_PIN_0);
		}
		break;
	}

	case ESP_SLEEP_WAKEUP_TIMER:
	{
		printf("Wake up from timer. Time spent in deep sleep: %dms\n",
			   sleep_time_ms);
		// Reanudar tarea comunicación con radio
		vTaskResume(send_data_to_radio_handle);
		break;
	}

	case ESP_SLEEP_WAKEUP_UNDEFINED:
	default:
		printf("Not a deep sleep reset\n");
	}

	// =============================================================== TIMER WAKEUP
	printf("Enabling timer wakeup, %ds\n", WAKEUP_TIME_SEC);
	esp_sleep_enable_timer_wakeup(WAKEUP_TIME_SEC * 1000000);
	// ===============================================================

#if CONFIG_IDF_TARGET_ESP32
	// Isolate GPIO12 pin from external circuits. This is needed for modules
	// which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
	// to minimize current consumption.
	rtc_gpio_isolate(GPIO_NUM_12);
#endif
	return ESP_OK;
}

/**
 * @brief Valida si es el primer arranque del dispositivo leyendo la memoria eeprom
 *
 * @return esp_err_t
 */
esp_err_t is_first_boot()
{
	float initial_volume;
	esp_err_t ret1 = eeprom_read_float(0x0, &initial_volume);
	if (ret1 == ESP_OK)
	{
		printf("initial volume read from EEPROM: %f\n", initial_volume);
		// Verificar si el volumen es cero
		if (initial_volume == 0)
		{
			ESP_LOGW(TAG_EEPROM, "You must initialize the liters value of the meter!");
			//======================================================================== EEPROM WRITE INITIAL VOLUME
			// long initial_volume = 0;
			// esp_err_t ret1 = eeprom_update_float(0x0, initial_volume);
			// if (ret1 == ESP_OK)
			// {
			// 	ESP_LOGI(TAG_EEPROM, "initial_volume written to EEPROM successfully!");
			// 	float read_data_float;
			// 	ret1 = eeprom_read_float(0x0, &read_data_float);
			// 	if (ret1 == ESP_OK)
			// 	{
			// 		printf("initial_volume read from EEPROM: %f\n", read_data_float);
			// 	}
			// }
		}
		return ESP_OK;
	}
	else
	{
		ESP_LOGE(TAG_EEPROM, "Error reading from EEPROM");
		return ESP_FAIL;
	}
	return ESP_OK;
}

/**
 * @brief Resetea el timer para volver a calcular el volumen
 *
 */
void rst_one_shot_timer()
{
	esp_timer_stop(timer_handle); // detener el timer
	eTaskState task_state = eTaskGetState(send_data_to_radio_handle);
	if (task_state == eDeleted || task_state == eSuspended)
	{
		ESP_LOGI(TAG_TIMER, "(rst_one_shot_timer) Timer has started");
		// ESP INIT TIMER
		esp_timer_start_once(timer_handle, VOLUME_CALC_TIME_SEC * 1000000); // 2 seconds in microseconds
	}
	else
	{
		ESP_LOGI(TAG_TIMER, "(rst_one_shot_timer) Timer has not started");
	}
}

/**
 * @brief Leer el voltaje de la batería
 *
 * @return float
 */
float read_voltage()
{
	// Leer el valor del pin analógico
	int adc_reading = 0;
	int adc_volt = 0;
	int sum = 0;

	// SET PIN EN_BAT_PIN TO HIGH
	gpio_set_level(EN_BAT_PIN, 1);

	if (do_calibration1)
	{
		for (int i = 0; i < 10; i++)
		{
			adc_oneshot_read(adc1_handle, V_BAT_ADC1_CHAN6, &adc_reading);
			// ESP_LOGI(TAG_ADC, "ADC -> %d", adc_reading);
			// vTaskDelay(pdMS_TO_TICKS(5)); // esperar un poco antes de tomar la siguiente lectura
			adc_cali_raw_to_voltage(adc1_cali_handle, adc_reading, &adc_volt);
			vTaskDelay(pdMS_TO_TICKS(10)); // esperar un poco antes de tomar la siguiente lectura
			sum += adc_volt;
		}
		adc_volt = sum / 10;
	}
	else
	{
		ESP_LOGE(TAG_BATTERY, "Error in calibration");
	}
	// SET PIN EN_BAT_PIN TO LOW
	gpio_set_level(EN_BAT_PIN, 0);
	ESP_LOGI(TAG_ADC, "ADC BATT VOLT -> %d", adc_volt);
	return (float)adc_volt;
}

/**
 * @brief Inicializa el ADC para realizar la calibración
 *
 * @param unit
 * @param atten
 * @param out_handle
 * @return true
 * @return false
 */
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
	adc_cali_handle_t handle = NULL;
	esp_err_t ret = ESP_FAIL;
	bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
	if (!calibrated)
	{
		ESP_LOGI(TAG_ADC, "calibration scheme version is %s", "Curve Fitting");
		adc_cali_curve_fitting_config_t cali_config = {
			.unit_id = unit,
			.atten = atten,
			.bitwidth = ADC_BITWIDTH_DEFAULT,
		};
		ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
		if (ret == ESP_OK)
		{
			calibrated = true;
		}
	}
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
	if (!calibrated)
	{
		ESP_LOGI(TAG_ADC, "calibration scheme version is %s", "Line Fitting");
		adc_cali_line_fitting_config_t cali_config = {
			.unit_id = unit,
			.atten = atten,
			.bitwidth = ADC_BITWIDTH_DEFAULT,
		};
		ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
		if (ret == ESP_OK)
		{
			calibrated = true;
		}
	}
#endif

	*out_handle = handle;
	if (ret == ESP_OK)
	{
		ESP_LOGI(TAG_ADC, "Calibration Success");
	}
	else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
	{
		ESP_LOGW(TAG_ADC, "eFuse not burnt, skip software calibration");
	}
	else
	{
		ESP_LOGE(TAG_ADC, "Invalid arg or no memory");
	}

	return calibrated;
}
