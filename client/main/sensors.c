#include "sensors.h"

# define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
# define I2C_MASTER_SDA_IO           21                         /*!< GPIO number used for I2C master data  */
# define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number */
# define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
# define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
# define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
# define I2C_MASTER_TIMEOUT_MS       1000

# define MPU6886_SENSOR_ADDR                     0x68        /*!< Slave address of the MPU6866 sensor */
# define MPU6886_WHO_AM_I_REG_ADDR               0x75        /*!< Register addresses of the "who am I" register */
# define MPU6886_SMPLRT_DIV_REG_ADDR             0x19
# define MPU6886_CONFIG_REG_ADDR                 0x1A
# define MPU6886_ACCEL_CONFIG_REG_ADDR           0x1C
# define MPU6886_ACCEL_CONFIG_2_REG_ADDR         0x1D
# define MPU6886_FIFO_EN_REG_ADDR                0x23
# define MPU6886_INT_PIN_CFG_REG_ADDR            0x37
# define MPU6886_INT_ENABLE_REG_ADDR             0x38
# define MPU6886_ACCEL_XOUT_REG_ADDR             0x3B
# define MPU6886_USER_CRTL_REG_ADDR              0x6A
# define MPU6886_PWR_MGMT_1_REG_ADDR             0x6B
# define MPU6886_PWR_MGMT_2_REG_ADDR             0x6C


static const char *TAG = "sensors";


esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t mpu6886_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6886_SENSOR_ADDR, &reg_addr, 1, data, len, 10*I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6886_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6886_SENSOR_ADDR, write_buf, sizeof(write_buf), 10*I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

void init_mpu6886(void)
{
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1_REG_ADDR, (0x01 << 7)));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1_REG_ADDR, (0x01 << 0)));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_ACCEL_CONFIG_REG_ADDR, 0x18));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_CONFIG_REG_ADDR, 0x01));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_SMPLRT_DIV_REG_ADDR, 0x05));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_ENABLE_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_ACCEL_CONFIG_2_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_USER_CRTL_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_FIFO_EN_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_PIN_CFG_REG_ADDR, 0x22));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_ENABLE_REG_ADDR, 0x01));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "MPU6866 intialized successfully");
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az)
{
    uint8_t buf[6];
    mpu6886_register_read(MPU6886_ACCEL_XOUT_REG_ADDR, buf, 6);

    *ax = ((int16_t)buf[0] << 8) | buf[1];
    *ay = ((int16_t)buf[2] << 8) | buf[3];
    *az = ((int16_t)buf[4] << 8) | buf[5];
}

void getAccelData(float* ax, float* ay, float* az)
{
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    getAccelAdc(&accX, &accY, &accZ);
    float aRes = 16.0 / 32768.0;

    *ax = (float)accX * aRes;
    *ay = (float)accY * aRes;
    *az = (float)accZ * aRes;
}