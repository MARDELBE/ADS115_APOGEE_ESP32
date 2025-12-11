#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

static const char *TAG = "VEML7700";

// --- Bus I2C (ajusta si usas otros pines) ---
#define I2C_SDA_IO      21
#define I2C_SCL_IO      22
#define I2C_PORT        I2C_NUM_0
#define I2C_CLK_HZ      100000

// --- VEML7700 ---
#define VEML_ADDR_7B    0x10  // Dirección fija 7-bit
// Registros
#define REG_ALS_CONF_0  0x00
#define REG_ALS         0x04
#define REG_ID          0x07

// Bits de ALS_CONF_0 (ver datasheet)
#define ALS_GAIN_X1     (0x0u << 11)
#define ALS_GAIN_X2     (0x1u << 11)
#define ALS_GAIN_DIV8   (0x2u << 11)
#define ALS_GAIN_DIV4   (0x3u << 11)

#define ALS_IT_25MS     (0xCu << 6)  // 1100b
#define ALS_IT_50MS     (0x8u << 6)  // 1000b
#define ALS_IT_100MS    (0x0u << 6)  // 0000b
#define ALS_IT_200MS    (0x1u << 6)  // 0001b
#define ALS_IT_400MS    (0x2u << 6)  // 0010b
#define ALS_IT_800MS    (0x3u << 6)  // 0011b

#define ALS_PERS_1      (0x0u << 4)
#define ALS_INT_DISABLE (0u   << 1)  // sin interrupciones
#define ALS_POWER_ON    (0u   << 0)  // ALS_SD=0

// Con GAIN=×2 e IT=800ms, la resolución oficial es 0.0042 lx/ct (datasheet)
#define LUX_PER_COUNT   0.0042f

static i2c_master_bus_handle_t bus = NULL;
static i2c_master_dev_handle_t dev = NULL;

// Escribe: comando(1B) + value(LSB,MSB)
static esp_err_t veml_write_u16(uint8_t reg, uint16_t value)
{
    uint8_t tx[3];
    tx[0] = reg;
    tx[1] = (uint8_t)(value & 0xFF);       // LSB primero
    tx[2] = (uint8_t)((value >> 8) & 0xFF);// MSB
    return i2c_master_transmit(dev, tx, sizeof(tx), pdMS_TO_TICKS(100));
}

// Lee palabra 16-bit de un registro (envía reg, luego lee LSB,MSB)
static esp_err_t veml_read_u16(uint8_t reg, uint16_t *out)
{
    uint8_t rx[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, rx, 2, pdMS_TO_TICKS(100));
    if (err != ESP_OK) return err;
    *out = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8); // LSB,MSB
    return ESP_OK;
}

void app_main(void)
{
    // 1) Crear bus I2C (driver nuevo)
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // 2) Agregar el dispositivo VEML7700
    i2c_device_config_t dev_cfg = {
        .device_address   = VEML_ADDR_7B,
        .scl_speed_hz = I2C_CLK_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));
    ESP_LOGI(TAG, "VEML7700 @0x%02X agregado.", VEML_ADDR_7B);

    // 3) Power on + configuración: GAIN=×2, IT=800ms, sin INT, PERS=1
    uint16_t conf = 0;
    conf |= ALS_GAIN_X2;
    conf |= ALS_IT_800MS;
    conf |= ALS_PERS_1;
    conf |= ALS_INT_DISABLE;
    conf |= ALS_POWER_ON; // ALS_SD=0
    ESP_ERROR_CHECK(veml_write_u16(REG_ALS_CONF_0, conf));
    ESP_LOGI(TAG, "ALS_CONF_0=0x%04X escrito. (GAIN x2, IT 800ms, ON)", conf);

    // 4) (Opcional) Leer ID para verificar (low byte = 0x81)
    uint16_t id = 0;
    if (veml_read_u16(REG_ID, &id) == ESP_OK) {
        ESP_LOGI(TAG, "ID=0x%04X (esperado low=0x81)", id);
    }

    // 5) Bucle de lectura: ALS → lux
    while (true) {
        uint16_t als = 0;
        esp_err_t err = veml_read_u16(REG_ALS, &als);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error leyendo ALS: %s", esp_err_to_name(err));
        } else {
            float lux = als * LUX_PER_COUNT;
            ESP_LOGI(TAG, "ALS=%u  ->  lux=%.3f", als, lux);
        }

        // Con IT=800ms, los “refresh” son lentos: lee cada ~1 s
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
