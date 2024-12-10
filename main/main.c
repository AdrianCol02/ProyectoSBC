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
#include <inttypes.h>

#define LIGHT_SENSOR_PIN ADC_CHANNEL_3 // GPIO39
#define ADC_MAX_VALUE 4095               // Valor máximo de un ADC de 12 bits
#define REF_VOLTAGE 3.3                 // Voltaje de referencia de la ESP32

#define FIRMWARE_URL "http://demo.thingsboard.io/api/v1/6OaUbcakpzr2PpJuKKWi/telemetry"
#define TAG "PROYECTO"

// Configuración de WiFi
#define WIFI_SSID "SBC"
#define WIFI_PASS "SBCwifi$"

#define MAX_RETRY 5

#define LATITUDE 40.3897656
#define LONGITUDE -3.6280461

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

static int s_retry_num = 0;
static SemaphoreHandle_t wifi_semaphore;

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Reintentando conexión al WiFi...");
        } else {
            ESP_LOGI(TAG, "No se pudo conectar al WiFi después de %d intentos", MAX_RETRY);
            xSemaphoreGive(wifi_semaphore); // Liberar el semáforo después de los intentos
        }
        ESP_LOGI(TAG, "Conexión al WiFi fallida");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conectado al WiFi, dirección IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xSemaphoreGive(wifi_semaphore); // Liberar el semáforo al obtener IP
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
    // Configuración del ADC para el sensor de luz
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
        // Leer el valor analógico entero del sensor
        int int_value;
        adc_oneshot_read(adc1_handle, LIGHT_SENSOR_PIN, &int_value);

        // Convertir el valor a voltaje
        float voltage = (int_value * REF_VOLTAGE) / 4095.0;

        // Mostrar resultados en el terminal
        printf("Valor entero: %d | Voltaje: %.2f V\n", int_value, voltage);

        // Enviar el valor de voltaje a ThingsBoard
        send_voltage_to_thingsboard(voltage);

        // Esperar un segundo
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
        .clock_speed_hz = 1000000,           // Clock out at 1 MHz
        .mode = 0,                           // SPI mode 0
        .spics_io_num = PIN_NUM_CS,          // CS pin
        .queue_size = 7,                     // We want to be able to queue 7 transactions at a time
    };

    // Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);

    // Attach the MIC3 to the SPI bus
    spi_device_handle_t spi;
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    while (1) {
        uint8_t data[2];
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));       // Zero out the transaction
        t.length = 8 * 2;               // Transaction length is in bits
        t.rx_buffer = data;             // Data to be received

        // Transmit and receive data
        ret = spi_device_transmit(spi, &t);
        ESP_ERROR_CHECK(ret);

        // Print raw data received
        ESP_LOGI(TAG, "Raw data received: 0x%02X 0x%02X", data[0], data[1]);

        // Process received data
        int mic_value = (data[0] << 8) | data[1];

        // Check if the mic_value is within the expected range
        if (mic_value < 0 || mic_value > ADC_MAX_VALUE) {
            ESP_LOGE(TAG, "MIC3 value out of range: %d", mic_value);
            continue;
        }

        // Convert the digital value to voltage
        float voltage = (mic_value * REF_VOLTAGE) / ADC_MAX_VALUE;

        // Ensure voltage is positive and greater than zero
        if (voltage <= 0) {
            voltage = 0.0001; // Small positive value to avoid log(0)
        }

        // Convert the voltage to decibels (assuming a reference voltage of 0.775V for 0 dB)
        float decibels = 20 * log10(mic_value / 0.775);

        // Print the value in decibels
        printf("MIC3 value: %d | Voltage: %.2f V | Decibels: %.2f dB\n", mic_value, voltage, decibels);

        // Enviar el valor de decibeles a ThingsBoard
        send_decibels_to_thingsboard(decibels);

        // Esperar 3 ms
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void app_main(void) {
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Crear el semáforo binario
    wifi_semaphore = xSemaphoreCreateBinary();

    // Inicializar WiFi
    wifi_init_sta();

    // Esperar hasta que el WiFi esté conectado
    xSemaphoreTake(wifi_semaphore, portMAX_DELAY);

    // Enviar datos de ubicación
    send_location_data();

    // Crear la tarea para leer el sensor de luz
    xTaskCreate(&light_sensor_task, "light_sensor_task", 4096, NULL, 5, NULL);

    // Crear la tarea para leer el micrófono
    xTaskCreate(&mic3_task, "mic3_task", 4096, NULL, 5, NULL);
}
