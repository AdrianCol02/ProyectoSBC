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

static const char *TAG = "PROYECTO";
static TimerHandle_t ota_timer;

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

        // Process received data
        int mic_value = (data[0] << 8) | data[1];
        
        // Convert the digital value to voltage
        float voltage = (mic_value * REF_VOLTAGE) / ADC_MAX_VALUE;

        // Convert the voltage to decibels (assuming a reference voltage of 1V for 0 dB)
        float decibels = 20 * log10(voltage / 3.3);

        // Print the value in decibels
        printf("MIC3 value: %d | Voltage: %.2f V | Decibels: %.2f dB\n", mic_value, voltage, decibels);

        // Esperar un segundo
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
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
