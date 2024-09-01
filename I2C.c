#include <stdio.h>
#include "i2c_master.h"
#include "sdkconfig.h"
#include "esp_log.h"


#define I2C_ADDR_BIT_LEN_7  7
#define I2C_PORT_NUM_0 I2C_NUM_0
#define I2C_MASTER_SCL_IO 10
#define I2C_MASTER_SDA_IO 9
#define DEVICE_ADDRESS 0x10
#define SCL_FREQUENCY 100000
#define ACK_VAL I2C_MASTER_ACK
#define NACK_VAL I2C_MASTER_NACK
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ

#define VEML7700_ALS_CONFIG 0x00        ///< Light configuration register
#define VEML7700_ALS_THREHOLD_HIGH 0x01 ///< Light high threshold for irq
#define VEML7700_ALS_THREHOLD_LOW 0x02  ///< Light low threshold for irq
#define VEML7700_ALS_POWER_SAVE 0x03    ///< Power save regiester
#define VEML7700_ALS_DATA 0x04          ///< The light data output
#define VEML7700_WHITE_DATA 0x05        ///< The white light data output
#define VEML7700_INTERRUPTSTATUS 0x06   ///< What IRQ (if any)

#define VEML7700_INTERRUPT_HIGH 0x4000 ///< Interrupt status for high threshold
#define VEML7700_INTERRUPT_LOW 0x8000  ///< Interrupt status for low threshold

#define VEML7700_GAIN_1 0x00   ///< ALS gain 1x
#define VEML7700_GAIN_2 0x01   ///< ALS gain 2x
#define VEML7700_GAIN_1_8 0x02 ///< ALS gain 1/8x
#define VEML7700_GAIN_1_4 0x03 ///< ALS gain 1/4x

#define VEML7700_IT_100MS 0x00 ///< ALS intetgration time 100ms
#define VEML7700_IT_200MS 0x01 ///< ALS intetgration time 200ms
#define VEML7700_IT_400MS 0x02 ///< ALS intetgration time 400ms
#define VEML7700_IT_800MS 0x03 ///< ALS intetgration time 800ms
#define VEML7700_IT_50MS 0x08  ///< ALS intetgration time 50ms
#define VEML7700_IT_25MS 0x0C  ///< ALS intetgration time 25ms

#define VEML7700_PERS_1 0x00 ///< ALS irq persisance 1 sample
#define VEML7700_PERS_2 0x01 ///< ALS irq persisance 2 samples
#define VEML7700_PERS_4 0x02 ///< ALS irq persisance 4 samples
#define VEML7700_PERS_8 0x03 ///< ALS irq persisance 8 samples

#define VEML7700_POWERSAVE_MODE1 0x00 ///< Power saving mode 1
#define VEML7700_POWERSAVE_MODE2 0x01 ///< Power saving mode 2
#define VEML7700_POWERSAVE_MODE3 0x02 ///< Power saving mode 3
#define VEML7700_POWERSAVE_MODE4 0x03 ///< Power saving mode 4




void newI2CBus(i2c_master_bus_handle_t *busHandle);
void newI2CDevice(i2c_master_dev_handle_t *deviceHandle);


void newI2CBus(i2c_master_bus_handle_t *busHandle)
{
    i2c_master_bus_config_t i2cConfig = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT_NUM_0,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
    };

    esp_err_t busCheck = i2c_new_master_bus(&i2cConfig, busHandle);
    ESP_ERROR_CHECK(busCheck);
}


void newI2CDevice(i2c_master_dev_handle_t *deviceHandle)
{
    i2c_device_config_t sensorConfig = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = DEVICE_ADDRESS,
    .scl_speed_hz = SCL_FREQUENCY,
    };

    esp_err_t deviceCheck = i2c_master_bus_add_device(I2C_PORT_NUM_0, &sensorConfig, deviceHandle);
    ESP_ERROR_CHECK(deviceCheck);
}


void readSensorData(uint8_t i2cInit, uint8_t data_rd, size_t size, i2c_master_dev_handle_t handle)
{
    esp_err_t readCheck = i2c_master_transmit_receive(handle, i2cInit, 2, data_rd, size, -1); 
}



