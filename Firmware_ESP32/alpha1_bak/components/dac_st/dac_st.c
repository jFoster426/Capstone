#include "dac_st.h"

void dac_init(uint32_t sample_rate)
{
    i2s_config_t i2s_config =
    {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 8,
        .dma_buf_len = DAC_BUF_LEN,
        .use_apll = false,
        .intr_alloc_flags = 0 // Default interrupt level
    };
    i2s_pin_config_t pin_config =
    {
        .mck_io_num = I2S_SCK_IO,
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = -1
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
    i2s_set_clk(I2S_NUM, sample_rate, 32, I2S_CHANNEL_STEREO);
}

size_t dac_write(double lVal, double rVal)
{
    // Write -1.0 to 1.0 to get full value range.
    for (int i = 0; i < DAC_BUF_LEN * 8; i += 2)
    {
        dac_buffer[i]     = lVal * ((pow(2, 32) / 2) - 1);
        dac_buffer[i + 1] = rVal * ((pow(2, 32) / 2) - 1);
    }


    // TODO: can someone figure out a more efficient way to write to I2S rather than spamming the buffer until its full.
    // What's the correct number of bytes to write?
    size_t bytes_written = 0;
    for (int i = 0; i < 100; i++)
    {
        i2s_write(I2S_NUM, dac_buffer, DAC_BUF_LEN * 8, &bytes_written, 100); // 100 represents timeout of 1 second.
    }

    return bytes_written; // 0 if timeout has occurred.
}