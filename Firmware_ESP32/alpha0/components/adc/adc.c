#include "adc.h"

// len is in bytes, excludes the command byte at the beginning of the transfer.
void spi_transfer(spi_device_handle_t *adc, const uint8_t *tx_data, uint8_t *rx_data, const uint8_t data_len)
{
#ifdef DEBUG
    printf("called spi_transfer()\n");
#endif
    spi_transaction_t adc_cfg_transaction = 
    {
        .cmd = tx_data[0],
        .length = 8 * data_len,
        .rxlength = 8 * data_len,
        .tx_buffer = &tx_data[1],
        .rx_buffer = &rx_data[0]
    };
    // Send the transaction.
    spi_device_transmit(*adc, &adc_cfg_transaction);
}

void adc_init(spi_device_handle_t *adc, uint8_t cs_pin)
{
#ifdef DEBUG
    printf("Initializing bus SPI2.\n");
#endif
    spi_bus_config_t buscfg =
    {
        .miso_io_num = SPI_PIN_NUM_MISO,
        .mosi_io_num = SPI_PIN_NUM_MOSI,
        .sclk_io_num = SPI_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    // Initialize the SPI2 bus.
    esp_err_t ret;
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Configure and add the ADC to the bus.
#ifdef DEBUG
    printf("Add ADC device to bus SPI2.\n");
#endif
    spi_device_interface_config_t devcfg =
    {
        .command_bits = 8,
        .mode = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_10M,
        .spics_io_num = cs_pin,
        .queue_size = 1
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, adc);

    // Allocate enough memory for the reads.
    uint8_t rx_data[10];

    // Send config.
#ifdef DEBUG
    printf("Send CONFIG0, CONFIG1, CONFIG2, CONFIG3 over SPI2.\n");
#endif
    spi_transfer(
        adc,
        (const uint8_t [5]){ 0b01000110, 0b11110011, 0b00001100, 0b10001011, 0b11110000 },
        &rx_data[0],
        4
    );

    // Send ch0?, refin-.
#ifdef DEBUG
    printf("Send MUX over SPI2.\n");
#endif
    spi_transfer(
        adc,
        (const uint8_t [2]){ 0b01011010, 0b00011100 },
        &rx_data[0],
        1
    );
}

uint32_t adc_read_raw(spi_device_handle_t *adc, uint8_t cs_pin)
{
    // Get adc data register.
    uint8_t rx_data[4] = { 0, 0, 0, 0 };
    spi_transfer(
        adc,
        (const uint8_t [5]){ 0b01000011, 0, 0, 0, 0 },
        &rx_data[0],
        4
    );
    return (uint32_t)((rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]);
}