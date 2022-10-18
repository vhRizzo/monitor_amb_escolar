/*
 * Implementation of I2C sensor BME280 based on SFeli's code available at:
 * https://github.com/SFeli/ESP32_BME280_IDF.
 * 
 * (c)2022 Victor Moura
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *    
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef BME280_SETUP_H_
#define BME280_SETUP_H_

#include "global_data.h"
#include "bme280.h"

#define SDA_GPIO                21
#define SCL_GPIO                22
#define I2C_MASTER_FREQ_HZ      CONFIG_I2C_MASTER_FREQUENCY
#define RTOS_DELAY_1SEC         ( 1 * 1000 / portTICK_PERIOD_MS)

void i2c_master_init ();
int8_t i2c_reg_read  ( uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr );
int8_t i2c_reg_write ( uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr );
void delay_us        ( uint32_t us, void *intf_ptr );
void print_rslt      ( const char api_name[], int8_t rslt );
void store_data      ( struct bme280_data *comp_data );
void bme280_task     ( void *pvParameters );

#endif /* BME280_SETUP_H_ */
