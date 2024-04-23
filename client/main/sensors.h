#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"


char * read_sensor_data();
esp_err_t i2c_master_init(void);
esp_err_t mpu6886_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t mpu6886_register_write_byte(uint8_t reg_addr, uint8_t data);
void init_mpu6886(void);
void getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az);
void getAccelData(float* ax, float* ay, float* az);