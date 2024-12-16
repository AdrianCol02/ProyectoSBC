#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <inttypes.h>

// Pines de conexión del sensor de luz
#define LIGHT_SENSOR_PIN ADC_CHANNEL_3 
               
// URL de la API de ThingsBoard para enviar los datos de los sensores
#define FIRMWARE_URL "http://demo.thingsboard.io/api/v1/6OaUbcakpzr2PpJuKKWi/telemetry"
#define TAG "PROYECTO"

// Credenciales de la red WiFi
#define WIFI_SSID "SBC"
#define WIFI_PASS "SBCwifi$"

// Constantes para el manejo de los sensores
#define MAX_RETRY 5 // Número máximo de intentos de conexión al WiFi
#define MAX_DECIBELS 120.0 // Valor máximo de decibeles
#define ADC_MAX_VALUE 4095 // Valor máximo de la conversión analógica-digital
#define REF_VOLTAGE 3.3  // Voltaje de referencia

// Coordenadas del laboratorio
#define LATITUDE 40.3897656
#define LONGITUDE -3.6280461

// Pines de conexión del micrófono
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

// Pines de conexión de los LEDs
#define LED_RED_PIN 25
#define LED_YELLOW_PIN 26
#define LED_GREEN_PIN 27

// Variables para el manejo de la conexión al WiFi
static int s_retry_num = 0;
static SemaphoreHandle_t wifi_semaphore;

// Variables globales para almacenar los valores de los sensores
static float global_voltage = 0; 
static float global_decibels = 0;


/* En las funciones de los sensores se encuentran comentadas las trazas de monitoreo*/

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Reintentando conexión al WiFi...");
        } else {
            ESP_LOGI(TAG, "No se pudo conectar al WiFi después de %d intentos", MAX_RETRY);
            xSemaphoreGive(wifi_semaphore);
        }
        ESP_LOGI(TAG, "Conexión al WiFi fallida");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conectado al WiFi, dirección IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xSemaphoreGive(wifi_semaphore);
    }
}

void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    esp_wifi_connect();
}

void send_voltage_to_thingsboard(float voltage) {
    char post_data[50];
    snprintf(post_data, sizeof(post_data), "{\"voltage\": %.2f}", voltage);

    esp_http_client_config_t config = {
        .url = FIRMWARE_URL,
        .method = HTTP_METHOD_POST,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %" PRId64,
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

void send_decibels_to_thingsboard(float decibels) {
    char post_data[50];
    snprintf(post_data, sizeof(post_data), "{\"decibels\": %.2f}", decibels);

    esp_http_client_config_t config = {
        .url = FIRMWARE_URL,
        .method = HTTP_METHOD_POST,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %" PRId64,
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

void send_location_data(void) {
    char post_data[100];
    snprintf(post_data, sizeof(post_data), "{\"latitude\": %.6f, \"longitude\": %.6f}", LATITUDE, LONGITUDE);

    esp_http_client_config_t config = {
        .url = FIRMWARE_URL,
        .method = HTTP_METHOD_POST,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %" PRId64,
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

void light_sensor_task(void *pvParameter) {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_6,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc1_handle, LIGHT_SENSOR_PIN, &config);

    while (1) {
        int int_value;
        adc_oneshot_read(adc1_handle, LIGHT_SENSOR_PIN, &int_value);

        float voltage = (int_value * REF_VOLTAGE) / 4095.0;
        global_voltage = voltage;
        //printf("Valor entero: %d | Voltaje: %.2f V\n", int_value, voltage); // Trazas de depuración
        send_voltage_to_thingsboard(voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void mic3_task(void *pvParameter) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,          
        .mode = 0,                           
        .spics_io_num = PIN_NUM_CS,          
        .queue_size = 7,                     
    };

    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    spi_device_handle_t spi;
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    while (1) {
        uint8_t data[2];
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 8 * 2;                
        t.rx_buffer = data;               
        ret = spi_device_transmit(spi, &t);
        ESP_ERROR_CHECK(ret);

        int mic_value = (data[0] << 8) | data[1];
        mic_value = mic_value & 0xFFF;
        float decibels = (((float)mic_value / 4096) * MAX_DECIBELS) - 5;

        if (decibels < 50) {
            decibels = 75; 
        } else if (decibels > MAX_DECIBELS) {
            decibels = MAX_DECIBELS;
        }

        global_decibels = decibels;
        
        /*printf("Datos crudos: 0x%02X%02X | MIC3: %d | Decibeles: %.2f dB\n",
               data[0], data[1], mic_value, decibels);*/ // Trazas de depuración


        send_decibels_to_thingsboard(decibels);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void led_task(void *pvParameter) {
    gpio_reset_pin(LED_RED_PIN);
    gpio_reset_pin(LED_YELLOW_PIN);
    gpio_reset_pin(LED_GREEN_PIN);
    gpio_set_direction(LED_RED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_YELLOW_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        if (global_voltage < 2 && global_decibels < 65) {
            // Encender el LED verde
            gpio_set_level(LED_GREEN_PIN, 1);
            gpio_set_level(LED_YELLOW_PIN, 0);
            gpio_set_level(LED_RED_PIN, 0);
        } else if (global_voltage >= 2 && global_decibels >= 65) {
            // Encender el LED rojo
            gpio_set_level(LED_GREEN_PIN, 0);
            gpio_set_level(LED_YELLOW_PIN, 0);
            gpio_set_level(LED_RED_PIN, 1);
        } else {
            // Encender el LED amarillo de manera intermitente cada segundo
            gpio_set_level(LED_GREEN_PIN, 0);
            gpio_set_level(LED_RED_PIN, 0);
            gpio_set_level(LED_YELLOW_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_YELLOW_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_semaphore = xSemaphoreCreateBinary();  //Semáforo para la conexión al WiFi
    wifi_init_sta(); // Iniciar la conexión al WiFi
    xSemaphoreTake(wifi_semaphore, portMAX_DELAY);
    send_location_data(); // Enviar las coordenadas previamente definidas
    // Iniciar las tareas
    xTaskCreate(&light_sensor_task, "light_sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(&mic3_task, "mic3_task", 4096, NULL, 5, NULL);
    xTaskCreate(&led_task, "led_task", 2048, NULL, 5, NULL);
}
