#include "dac_st.h"

// TODO: make this library the same formatting for DEBUG as the i2c library.

void dac_init(uint32_t sample_rate)
{
#ifdef DEBUG
    printf("called dac_init()\n");
#endif
    i2s_config_t i2s_config =
    {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 2,
        .dma_buf_len = 64,
        .use_apll = false,
        .intr_alloc_flags = 0 // Default interrupt level
    };
    i2s_pin_config_t pin_config =
    {
        .mck_io_num = I2S_MCK_IO,
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = -1
    };
#ifdef DEBUG
    printf("install i2s driver\n");
#endif
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
#ifdef DEBUG
    printf("set i2s pins\n");
#endif
    i2s_set_pin(I2S_NUM, &pin_config);
#ifdef DEBUG
    printf("set clock\n");
#endif
    i2s_set_clk(I2S_NUM, sample_rate, 32, 2);
}

size_t dac_write(double lVal, double rVal)
{
    // Write -1 to 1 to get full value range.
    int32_t buf[128];
    for (int i = 0; i < 128; i += 2)
    {
        buf[i]     = lVal * ((pow(2, 32) / 2) - 1);
        buf[i + 1] = rVal * ((pow(2, 32) / 2) - 1);
    };
    
#ifdef DEBUG
    printf("write data: %.2f, %.2f\n", lVal, rVal);
#endif
    size_t bytes_written = 0;
    for (int i = 0; i < 8; i++)
        i2s_write(I2S_NUM, buf, 128, &bytes_written, 100); // 100 represents timeout of 1 second.
    return bytes_written; // 0 if timeout has occurred.
}