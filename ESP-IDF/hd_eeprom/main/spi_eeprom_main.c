#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"


#  define PIN_NUM_MISO 10
#  define PIN_NUM_MOSI 11
#  define PIN_NUM_CLK  12
#  define PIN_NUM_CS   7

void app_main(void)
{
    printf("\n\nStart of app_main\n\n");
    esp_err_t ret;
    printf("Initializing bus SPI2.\n");
    spi_bus_config_t buscfg =
    {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Configure and add the ADC to the bus.
    printf("Add ADC device to bus SPI2.\n");
    spi_device_interface_config_t devcfg =
    {
        .command_bits = 8,
        .mode = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_10M,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1
    };
    spi_device_handle_t adc;
    spi_bus_add_device(SPI2_HOST, &devcfg, &adc);

    // Configure an example transaction for the ADC.
    printf("Create SPI transaction.\n");
    uint8_t tx_data_cfg[] = { 0b11110011, 0b00001100, 0b10001011, 0b11110000 };
    uint8_t rx_data[4] = { 0, 0, 0, 0 };
    spi_transaction_t adc_cfg_transaction = 
    {
        .cmd = 0b01000110, // device addr 01, incremental write config0 register
        .length = 32,
        .rxlength = 32,
        .tx_buffer = tx_data_cfg,
        .rx_buffer = rx_data
    };
    // Send the transaction.
    printf("Send CONFIG0, CONFIG1, CONFIG2, CONFIG3 over SPI2.\n");
    spi_device_transmit(adc, &adc_cfg_transaction);

    uint8_t tx_data_mux = 0b00011100; // CH0, REFIN-
    spi_transaction_t adc_mux_transaction = 
    {
        .cmd = 0b01011010, // device addr 01, incremental write config0 register
        .length = 8,
        .rxlength = 8,
        .tx_buffer = &tx_data_mux,
        .rx_buffer = rx_data
    };
    // Send the transaction.
    printf("Send MUX over SPI2.\n");
    spi_device_transmit(adc, &adc_mux_transaction);

    uint8_t tx_data_data[] = { 0, 0, 0, 0 };
    spi_transaction_t adc_data_transaction = 
    {
        .cmd = 0b01000011, // device addr 01, incremental read adc_data register.
        .length = 32,
        .rxlength = 32,
        .tx_buffer = tx_data_data,
        .rx_buffer = rx_data
    };
    printf("Read Data over SPI2.\n");
    // Read the data from the ADC and print to the Serial port.
    while (1)
    {
        // Send the transaction.
        spi_device_transmit(adc, &adc_data_transaction);
        printf("Data received: %02x%02x%02x%02x\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
        vTaskDelay(10);
    }
}
