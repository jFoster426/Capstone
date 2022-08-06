#include "adc_st.h"

// len is in bytes, excludes the command byte at the beginning of the transfer.
void spi_transfer(spi_device_handle_t *adc, const uint8_t *tx_data, uint8_t *rx_data, const uint8_t data_len)
{
    spi_transaction_t adc_transaction = 
    {
        .cmd = tx_data[0],
        .length = 8 * (data_len),
        .rxlength = 8 * (data_len),
        .tx_buffer = &tx_data[1],
        .rx_buffer = &rx_data[0]
    };
    // Send the transaction.
    spi_device_transmit(*adc, &adc_transaction);
}

#define ADC_CMD_READ_SEQ        (0b01000011)
#define ADC_CMD_WRITE_SEQ       (0b01000010)
#define SDC_CMD_WRITE_STATIC    (0b01000001)

#define ADC_REG_ADCDATA         (0x0 << 2)
#define ADC_REG_CONFIG0         (0x1 << 2)
#define ADC_REG_CONFIG1         (0x2 << 2)
#define ADC_REG_CONFIG2         (0x3 << 2)
#define ADC_REG_CONFIG3         (0x4 << 2)
#define ADC_REG_IRQ             (0x5 << 2)
#define ADC_REG_MUX             (0x6 << 2)
#define ADC_REG_SCAN            (0x7 << 2)
#define ADC_REG_TIMER           (0x8 << 2)
#define ADC_REG_OFFSETCAL       (0x9 << 2)
#define ADC_REG_GAINCAL         (0xA << 2)
#define ADC_REG_LOCK            (0xD << 2)
#define ADC_REG_CRCCFG          (0xF << 2)

void adc_init(spi_device_handle_t *adc, uint8_t cs_pin)
{
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
    spi_transfer(adc,
                                                                         // CONFIG0    CONFIG1     CONFIG2     CONFIG3     IRQ         MUX         SCAN        SCAN        SCAN
                 (const uint8_t [10]){ ADC_CMD_WRITE_SEQ | ADC_REG_CONFIG0, 0b11110011, 0b00001100, 0b10001011, 0b11110000, 0b00000011, 0b00000000, 0b00000000, 0b00000000, 0b00000011 },
                 &rx_data[0],
                 10);
}

void adc_read_raw(spi_device_handle_t *adc, uint32_t *ch0, uint32_t *ch1)
{
    uint8_t rx_data[4] = { 0, 0, 0, 0 };

    while (1)
    {
        // Get adc data register for ch0.
        spi_transfer(adc,
                    (const uint8_t [5]){ ADC_CMD_READ_SEQ | ADC_REG_ADCDATA, 0, 0, 0, 0 },
                    &rx_data[0],
                    4);
        *ch0 = (uint32_t)((rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]);

        // Wait for ch1 data.
        if (((*ch0 >> 28) & 0xf) == 0) break;
    }

    while (1)
    {
        // Get adc data register for ch1.
        spi_transfer(adc,
                    (const uint8_t [5]){ ADC_CMD_READ_SEQ | ADC_REG_ADCDATA, 0, 0, 0, 0 },
                    &rx_data[0],
                    4);
        *ch1 = (uint32_t)((rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]);

        // Wait for ch1 data.
        if (((*ch1 >> 28) & 0xf) == 1) break;
    }

    // remove channel data, keep one sign bit.
    *ch0 &= 0x1ffffff;
    *ch1 &= 0x1ffffff;
}

void adc_read_voltage(spi_device_handle_t *adc, double *ch0v, double *ch1v)
{
    uint32_t ch0, ch1;
    adc_read_raw(adc, &ch0, &ch1);

    if (ch0 & (1 << 24)) ch0 = 0;
    *ch0v = (double)ch0 * (2.5) / (double)(0x0800000);

    if (ch1 & (1 << 24)) ch1 = 0;
    *ch1v = (double)ch1 * (2.5) / (double)(0x0800000);
}