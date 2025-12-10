#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO          22
#define I2C_MASTER_SDA_IO          21
#define I2C_MASTER_PORT            I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0
#define APOGEE_CAL_FACTOR_UMOL_PER_MV  1.6f

// Dirección del ADS1115 (ADDR a GND)
#define ADS1115_ADDR               0x48

// Registros del ADS1115
#define ADS1115_REG_CONVERSION     0x00
#define ADS1115_REG_CONFIG         0x01

static const char *TAG = "ADS1115_APOGEE";

// ---------------- I2C INIT ----------------

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode,
                              I2C_MASTER_TX_BUF_DISABLE,
                              I2C_MASTER_RX_BUF_DISABLE, 0);
}

// ------------- HELPERS ADS1115 -------------

static esp_err_t ads1115_write_register(uint8_t reg, uint16_t value)
{
    uint8_t tx_buf[3];
    tx_buf[0] = reg;
    tx_buf[1] = (value >> 8) & 0xFF;   // MSB
    tx_buf[2] = value & 0xFF;          // LSB

    return i2c_master_write_to_device(
        I2C_MASTER_PORT,
        ADS1115_ADDR,
        tx_buf,
        sizeof(tx_buf),
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t ads1115_read_register(uint8_t reg, uint16_t *out_value)
{
    esp_err_t ret;
    uint8_t tx = reg;
    uint8_t rx[2];

    ret = i2c_master_write_read_device(
        I2C_MASTER_PORT,
        ADS1115_ADDR,
        &tx,
        1,
        rx,
        2,
        pdMS_TO_TICKS(100)
    );

    if (ret != ESP_OK) {
        return ret;
    }

    *out_value = ((uint16_t)rx[0] << 8) | rx[1];  // MSB primero
    return ESP_OK;
}

// Lee A0 (AIN0) en modo single-shot, PGA = ±0.256 V, 128 SPS
static esp_err_t ads1115_read_raw_ch0(int16_t *raw_out)
{
    // Config word:
    // OS=1 (start single conv)
    // MUX=100 (AIN0 vs GND)
    // PGA=101 (±0.256 V)
    // MODE=1 (single-shot)
    // DR=100 (128 SPS)
    // COMP_MODE=0, COMP_POL=0, COMP_LAT=0, COMP_QUE=11 (comparator disabled)
    uint16_t config = 0;
    config |= (1 << 15);     // OS
    config |= (4 << 12);     // MUX = 100b -> AIN0-GND
    config |= (5 << 9);      // PGA = 101b -> ±0.256 V
    config |= (1 << 8);      // MODE = 1 (single-shot)
    config |= (4 << 5);      // DR = 100b -> 128 SPS
    // bits 4,3,2 = 0
    config |= 3;             // COMP_QUE = 11 -> disable comparator

    esp_err_t ret = ads1115_write_register(ADS1115_REG_CONFIG, config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Tiempo de conversión: a 128 SPS, ~7.8 ms -> esperamos ~10 ms
    vTaskDelay(pdMS_TO_TICKS(10));

    uint16_t raw_u16 = 0;
    ret = ads1115_read_register(ADS1115_REG_CONVERSION, &raw_u16);
    if (ret != ESP_OK) {
        return ret;
    }

    *raw_out = (int16_t)raw_u16;  // el registro es signed 16 bits
    return ESP_OK;
}

// Convierte el raw a voltios y mV usando FSR ±0.256 V
static void ads1115_raw_to_voltage_mv(int16_t raw, float *volts, float *mv)
{
    // Para ±0.256 V: 1 LSB ≈ 256 mV / 32768 ≈ 0.0078125 mV
    const float LSB_MV = 0.0078125f;   // mV por cuenta

    float mv_local = raw * LSB_MV;
    float v_local  = mv_local / 1000.0f;

    if (volts) {
        *volts = v_local;
    }
    if (mv) {
        *mv = mv_local;
    }
}




//------------------ Convierte mV a PPFD usando factor de calibración Apogee
static float apogee_mv_to_ppfd(float mv)
{
    // 1) corregir offset si quieres (ver punto 4)
    // 2) evitar negativos
    if (mv < 0.0f) {
        mv = 0.0f;
    }

    // 3) convertir a PPFD
    return mv * APOGEE_CAL_FACTOR_UMOL_PER_MV;
}


// ----------------- app_main ----------------

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C inicializado, ADS1115 en 0x%02X", ADS1115_ADDR);

    while (1) {
        int16_t raw = 0;
        esp_err_t ret = ads1115_read_raw_ch0(&raw);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error leyendo ADS1115: %s", esp_err_to_name(ret));
        } else {
            float v = 0.0f;
            float mv = 0.0f;
            ads1115_raw_to_voltage_mv(raw, &v, &mv);
            float ppfd = apogee_mv_to_ppfd(mv);

        ESP_LOGI(TAG,
                "RAW=%6d  V=%.6f V  mV=%.3f mV  PPFD=%.1f umol m-2 s-1",
                raw, v, mv, ppfd);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
