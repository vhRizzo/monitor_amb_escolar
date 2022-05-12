#include "global_data.h"

#ifdef BME280_SENSOR
#include "bme280_setup.h"

struct bme280_data comp_data;
struct bme280_dev bme;
uint8_t settings_sel;
int8_t bmestatus = BME280_OK;
bme280_t bme_dados;
QueueHandle_t bme_queue;

void i2c_master_init()
{
    i2c_config_t i2c_config;
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = SDA_GPIO;
    i2c_config.scl_io_num = SCL_GPIO;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
 
//*************// Verifica se o slave I2C esta funcionando corretamente.
    esp_err_t f_retval;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR_PRIM << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    f_retval = i2c_master_cmd_begin(I2C_NUM_0, cmd, RTOS_DELAY_1SEC);
    if (f_retval != ESP_OK) {
        ESP_LOGE(__func__, "I2C slave NOT working or wrong I2C slave address - error (%i)", f_retval);
    }
    i2c_cmd_link_delete(cmd);
}

int8_t i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t iError;
    int8_t i2c_addr = *((int *)intf_ptr);
    esp_err_t esp_err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, reg_addr, true);
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd_handle, reg_data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd_handle, reg_data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd_handle);
    esp_err = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
 
    if (esp_err == ESP_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    i2c_cmd_link_delete(cmd_handle);
    return iError;
}

int8_t i2c_reg_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t i2c_addr = *((int *)intf_ptr);
    int8_t iError;
    esp_err_t esp_err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, reg_addr, true);
    i2c_master_write(cmd_handle, reg_data, length,true);
    i2c_master_stop(cmd_handle);
    esp_err = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
    if (esp_err == ESP_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    i2c_cmd_link_delete(cmd_handle);
    return iError;
}

void delay_us(uint32_t us, void *intf_ptr) { vTaskDelay((us/1000) / portTICK_PERIOD_MS); }

void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BME280_OK) {
        if (rslt == BME280_E_NULL_PTR) {
            ESP_LOGE(api_name, "Error [%d] : Null pointer error", rslt);
        } else if (rslt == BME280_E_DEV_NOT_FOUND) {
            ESP_LOGE(api_name, "Error [%d] : Device not found", rslt);
        } else if (rslt == BME280_E_INVALID_LEN) {
            ESP_LOGE(api_name, "Error [%d] : Invalid Lenght", rslt);
        } else if (rslt == BME280_E_COMM_FAIL) {
            ESP_LOGE(api_name, "Error [%d] : Bus communication failed", rslt);
        } else if (rslt == BME280_E_SLEEP_MODE_FAIL) {
            ESP_LOGE(api_name, "Error [%d] : SLEEP_MODE_FAIL", rslt);
        } else {
            /* For more error codes refer "*_defs.h" */
            ESP_LOGE(api_name, "Error [%d] : Unknown error code", rslt);
        }
    }else{
        ESP_LOGI(api_name, " BME280 status [%d]",rslt);
    }
}

void store_data (struct bme280_data *comp_data)
{
    bme_dados.temperatura = comp_data->temperature; 
    bme_dados.umidade = comp_data->humidity;
    bme_dados.pressao = comp_data->pressure;
    xQueueOverwrite(bme_queue, &bme_dados);
}

void bme280_task(void *pvParameters)
{
    i2c_master_init();

    uint8_t dev_addr = BME280_I2C_ADDR_PRIM;    // 0x76
    bme.intf_ptr = &dev_addr;                   
    bme.intf = BME280_I2C_INTF;
    bme.read = i2c_reg_read;
    bme.write = (bme280_write_fptr_t)i2c_reg_write;
    bme.delay_us = delay_us;

    bmestatus = bme280_init(&bme);
    print_rslt("bme280_init status", bmestatus);

    /* Recommended mode of operation: Indoor navigation */
    bme.settings.osr_h = BME280_OVERSAMPLING_1X;
    bme.settings.osr_p = BME280_OVERSAMPLING_16X;
    bme.settings.osr_t = BME280_OVERSAMPLING_2X;
    bme.settings.filter = BME280_FILTER_COEFF_16;
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    bmestatus = bme280_set_sensor_settings(settings_sel, &bme);
    print_rslt("bme280_set_sensor_settings status", bmestatus);
    while (1) {
        bmestatus = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme);
        vTaskDelay( (TEMPO_ANALISE*1000)/portTICK_PERIOD_MS );
        bmestatus = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);
        store_data(&comp_data);
    }
}
#endif
