#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_log.h"
#include <string.h>

#define CONFIG_BITS 0x3000  //configuration bits for DAC write command as specified from datasheet. Operates with a gain of 1x.
const char *TAG = "DAC";
uint8_t *buffer;

esp_err_t initializeSPIBus(spi_host_device_t, const spi_bus_config_t*);
void addDAC(spi_host_device_t, const spi_device_interface_config_t*, spi_device_handle_t*);
esp_err_t transmitToDAC(spi_transaction_t*, spi_device_handle_t);
spi_bus_config_t configureSPIBus(void);
spi_device_interface_config_t configureDAC(int);    //configure the DAC for Red LED channel, chip select from an input gpio pin
spi_transaction_t createTransaction(const uint16_t*);
//void spi_pre_transfer_callback(spi_transaction_t*);



esp_err_t initializeSPIBus(spi_host_device_t host, const spi_bus_config_t* busConfig)
{
    esp_err_t error = spi_bus_initialize(host, busConfig, SPI_DMA_DISABLED);    
    return error;
}

void addDAC(spi_host_device_t host, const spi_device_interface_config_t* device, spi_device_handle_t* handle)
{
    spi_bus_add_device(host, device, handle);
}

esp_err_t transmitToDAC(spi_transaction_t* transaction, spi_device_handle_t handle)
{	
    spi_device_acquire_bus(handle, portMAX_DELAY);
    esp_err_t error = spi_device_polling_transmit(handle, transaction);
    spi_device_release_bus(handle);
    return error;
}

spi_bus_config_t configureSPIBus(void)
{
    spi_bus_config_t spiBus = {
    .mosi_io_num = 12,
    .miso_io_num = -1,
    .sclk_io_num = 13,
    .quadhd_io_num = -1,
    .quadwp_io_num = -1,
    .max_transfer_sz = 2
    };

    return spiBus;
}

spi_device_interface_config_t configureDAC(int chipSelectGPIO)
{
    spi_device_interface_config_t configDAC = {
    .clock_speed_hz = SPI_MASTER_FREQ_16M,
    .duty_cycle_pos = 128,
    .mode = 0,
    .spics_io_num = chipSelectGPIO,
    .queue_size = 1,
    .flags = SPI_DEVICE_NO_DUMMY
    };

    return configDAC;
}

spi_transaction_t createTransaction(const uint16_t* commandData)
{
    spi_transaction_t transaction = {
        .length = 16,
        .tx_buffer = commandData
    };

    return transaction;
}
/*
void spi_pre_transfer_callback(spi_transaction_t* t)
{
    ESP_LOGI(TAG, "Transmitting Data %d cmd", t->cmd);
}
*/