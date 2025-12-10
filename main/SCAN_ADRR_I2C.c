#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_PORT     I2C_NUM_0


static const char *TAG = "I2C_SCAN";


void app_main(void)
{
    // 1) Configurar el bus I2C (driver nuevo)
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,   // usamos pull-ups internos por ahora
        },
    };

    i2c_master_bus_handle_t bus_handle = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error creando bus I2C: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Bus I2C inicializado. Escaneando...");

    // 2) Escanear direcciones 7-bit
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        esp_err_t probe_ret = i2c_master_probe(bus_handle, addr, 50); // 50 ms

        if (probe_ret == ESP_OK) {
            ESP_LOGW(TAG, "Dispositivo encontrado en 0x%02X", addr);
        } else if (probe_ret == ESP_ERR_NOT_FOUND) {
            // Nada en esta dirección → ignorar
        } else if (probe_ret == ESP_ERR_TIMEOUT) {
            // Problema de bus (pull-ups / cableado)
            ESP_LOGE(TAG,
                     "Timeout al probar 0x%02X (revisa conexiones/pull-ups)",
                     addr);
        } else {
            // Otro error raro
            ESP_LOGE(TAG,
                     "Error al probar 0x%02X: %s",
                     addr, esp_err_to_name(probe_ret));
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // pequeño respiro entre probes
    }

    ESP_LOGI(TAG, "Scan I2C terminado.");

    // 3) Mantener la tarea viva (por si quieres ver solo una vez el scan)
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
