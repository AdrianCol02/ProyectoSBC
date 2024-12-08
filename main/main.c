#include <stdio.h>
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/adc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <math.h>

// Configuración del ADC
#define LIGHT_SENSOR_PIN ADC1_CHANNEL_3 // GPIO39
#define ADC_MAX_VALUE 4095              // Resolución de 12 bits
#define REF_VOLTAGE 3.3                 // Voltaje de referencia de la ESP32

#define FIRMWARE_URL "http://your-thingsboard-server.com/firmware.bin"

// Configuración del SPI
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

// Configuración de WiFi
#define WIFI_SSID "SBC"
#define WIFI_PASS "SBCwifi$"

#define DB_BUFFER_SIZE 10

#define LATITUDE 40.7128
#define LONGITUDE -74.0060

static const char *TAG = "PROYECTO";
static TimerHandle_t ota_timer;

// Declaraciones de funciones
void send_voltage_data(float voltage);
void send_decibels_data(float decibels);
void send_location_data(void);

void wifi_task(void *pvParameter)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    while (1) {
        esp_err_t ret = esp_wifi_connect();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Connected to WiFi network: %s", WIFI_SSID);
            vTaskDelay(pdMS_TO_TICKS(30000)); // Esperar 30 segundos antes de volver a intentar
        } else {
            ESP_LOGE(TAG, "Failed to connect to WiFi network: %s", WIFI_SSID);
            vTaskDelay(pdMS_TO_TICKS(30000)); // Esperar 30 segundos antes de volver a intentar
        }
    }
}

void ota_task(void *pvParameter)
{
    esp_http_client_config_t config = {
        .url = FIRMWARE_URL,
        .timeout_ms = 5000,
    };

    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA update successful, restarting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA update failed...");
    }

    vTaskDelete(NULL);
}

void ota_timer_callback(TimerHandle_t xTimer)
{
    xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
}

void sensor_task(void *pvParameter)
{
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_6,
    };
    adc_oneshot_config_channel(adc1_handle, LIGHT_SENSOR_PIN, &config);

    while (1) {
        int int_value;
        adc_oneshot_read(adc1_handle, LIGHT_SENSOR_PIN, &int_value);
        
        // Convertir el valor a voltaje
        float voltage = (int_value * REF_VOLTAGE) / ADC_MAX_VALUE;

        // Mostrar resultados en el terminal
        printf("Valor entero: %d | Voltaje: %.2f V\n", int_value, voltage);

        // Enviar datos de voltaje a ThingsBoard
        send_voltage_data(voltage);

        // Esperar un segundo
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    adc_oneshot_del_unit(adc1_handle);
}

void mic3_task(void *pvParameter)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // Clock out at 1 MHz
        .mode = 0,                         // SPI mode 0
        .spics_io_num = PIN_NUM_CS,        // CS pin
        .queue_size = 7,                   // We want to be able to queue 7 transactions at a time
    };
    spi_device_handle_t spi;
    spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    spi_bus_add_device(HSPI_HOST, &devcfg, &spi);

    while (1) {
        uint8_t data[2];
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));       // Zero out the transaction
        t.length = 8 * 2;               // Length is in bytes
        t.rx_buffer = data;             // Data to be received
        spi_device_transmit(spi, &t);   // Transmit!

        // Process received data
        int mic_value = (data[0] << 8) | data[1];
        if (mic_value < 0) {
            mic_value = 0;
        } else if (mic_value > ADC_MAX_VALUE) {
            mic_value = ADC_MAX_VALUE;
        }
        float voltage = (mic_value * REF_VOLTAGE) / ADC_MAX_VALUE;
        float decibels = 20 * log10(mic_value / 0.775);

        // Mostrar resultados en el terminal
        printf("MIC3 value: %d | Voltage: %.2f V | Decibels: %.2f dB\n", mic_value, voltage, decibels);

        // Enviar datos de decibeles a ThingsBoard
        send_decibels_data(decibels);

        // Esperar 300 ms
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void send_voltage_data(float voltage)
{
    // Crear el JSON con los datos
    char post_data[256];
    snprintf(post_data, sizeof(post_data), "{\"voltage\": %.2f}", voltage);

    // Configurar el cliente HTTP
    esp_http_client_config_t config = {
        .url = "http://your-thingsboard-server.com/api/v1/YOUR_ACCESS_TOKEN/telemetry",
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    // Enviar los datos
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Datos de voltaje enviados correctamente a ThingsBoard");
    } else {
        ESP_LOGE(TAG, "Error al enviar datos de voltaje a ThingsBoard: %s", esp_err_to_name(err));
    }

    // Limpiar el cliente HTTP
    esp_http_client_cleanup(client);
}

void send_decibels_data(float decibels)
{
    // Crear el JSON con los datos
    char post_data[256];
    snprintf(post_data, sizeof(post_data), "{\"decibels\": %.2f}", decibels);

    // Configurar el cliente HTTP
    esp_http_client_config_t config = {
        .url = "http://your-thingsboard-server.com/api/v1/YOUR_ACCESS_TOKEN/telemetry",
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    // Enviar los datos
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Datos de decibeles enviados correctamente a ThingsBoard");
    } else {
        ESP_LOGE(TAG, "Error al enviar datos de decibeles a ThingsBoard: %s", esp_err_to_name(err));
    }

    // Limpiar el cliente HTTP
    esp_http_client_cleanup(client);
}

void send_location_data(void)
{
    // Crear el JSON con los datos
    char post_data[256];
    snprintf(post_data, sizeof(post_data), "{\"latitude\": %.4f, \"longitude\": %.4f}", LATITUDE, LONGITUDE);

    // Configurar el cliente HTTP
    esp_http_client_config_t config = {
        .url = "http://your-thingsboard-server.com/api/v1/YOUR_ACCESS_TOKEN/telemetry",
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    // Enviar los datos
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Datos de ubicación enviados correctamente a ThingsBoard");
    } else {
        ESP_LOGE(TAG, "Error al enviar datos de ubicación a ThingsBoard: %s", esp_err_to_name(err));
    }

    // Limpiar el cliente HTTP
    esp_http_client_cleanup(client);
}

void app_main(void)
{
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Enviar datos de ubicación una vez
    send_location_data();

    // Crear una tarea para manejar la conexión WiFi
    xTaskCreate(&wifi_task, "wifi_task", 4096, NULL, 5, NULL);

    // Crear un temporizador que se dispare una vez cada semana (604800000 ms)
    ota_timer = xTimerCreate("ota_timer", pdMS_TO_TICKS(604800000), pdTRUE, (void *)0, ota_timer_callback);
    if (ota_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create OTA timer");
    } else {
        if (xTimerStart(ota_timer, 0) != pdPASS) {
            ESP_LOGE(TAG, "Failed to start OTA timer");
        }
    }

    xTaskCreate(&sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(&mic3_task, "mic3_task", 4096, NULL, 5, NULL);
}
