#include "ism330_st.h"

void ism330_init(void)
{
    // Set device configuration = 1.
    i2c_register_write_byte(ISM330_I2C_ADDR, ISM330_REG_CTRL9_XL, 0b00000010);


    // Testing filtering
    i2c_register_write_byte(ISM330_I2C_ADDR, ISM330_REG_CTRL6_C, 0b00000111);

    

    // Accelerometer ODR = 6.66kHz (high performance), FS = +/- 2G, LPF = off.
    uint8_t odr_xl = 0b0100; // 0b1010;
    uint8_t fs_xl = ACC_FS_2G;
    uint8_t lpf2_xl_en = 0;

    odr_xl &= 0xF;
    fs_xl &= 0x3;
    i2c_register_write_byte(ISM330_I2C_ADDR, ISM330_REG_CTRL1_XL, (odr_xl << 4) | (fs_xl << 2) | lpf2_xl_en);

    // Gyroscope ODR = 6.66kHz (high performance), FS = +/- 500DPS, FS125 = off, FS4000 = off.
    uint8_t odr_g = 0b0100; // 0b1010;
    uint8_t fs_g = 0;

    // TODO: define this as global and use in the other functions.
    float g_dataRate = GYR_FS_2000;

    // TODO: add other test cases.
    if (g_dataRate == GYR_FS_500) fs_g = 0x01;
    else if (g_dataRate == GYR_FS_2000) fs_g = 0x03;
    uint8_t fs_125 = 0;
    uint8_t fs_4000 = 0;

    odr_g &= 0xF;
    fs_g &= 0x3;
    fs_125 &= 0;
    fs_4000 &= 0;
    i2c_register_write_byte(ISM330_I2C_ADDR, ISM330_REG_CTRL2_G, (odr_g << 4) | (fs_g << 2) | (fs_125 << 1) | fs_4000);


    // Set device configuration = 0.
    i2c_register_write_byte(ISM330_I2C_ADDR, ISM330_REG_CTRL9_XL, 0b00000000);
}

int16_t ism330_get_acc_x(void)
{
    uint8_t l, h;
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTX_L_A, &l);
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTX_H_A, &h);

    uint16_t ret = (h << 8) | l;
    return (int16_t)ret;
}

float ism330_get_acc_x_g(void)
{
    uint8_t fs_xl = ACC_FS_2G;
    int16_t raw = ism330_get_acc_x();
    
    float ret = 0.0;
    if (fs_xl == ACC_FS_2G)
    {
        ret = ((float)raw * 2.0) / 32768.0;
    }
    return ret;
}

int16_t ism330_get_acc_y(void)
{
    uint8_t l, h;
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTY_L_A, &l);
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTY_H_A, &h);

    uint16_t ret = (h << 8) | l;
    return (int16_t)ret;
}

float ism330_get_acc_y_g(void)
{
    uint8_t fs_xl = ACC_FS_2G;
    int16_t raw = ism330_get_acc_y();
    
    float ret = 0.0;
    if (fs_xl == ACC_FS_2G)
    {
        ret = ((float)raw * 2.0) / 32768.0;
    }
    return ret;
}

int16_t ism330_get_acc_z(void)
{
    uint8_t l, h;
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTZ_L_A, &l);
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTZ_H_A, &h);

    uint16_t ret = (h << 8) | l;
    return (int16_t)ret;
}

float ism330_get_acc_z_g(void)
{
    uint8_t fs_xl = ACC_FS_2G;
    int16_t raw = ism330_get_acc_z();
    
    float ret = 0.0;
    if (fs_xl == ACC_FS_2G)
    {
        ret = ((float)raw * 2.0) / 32768.0;
    }
    return ret;
}

int16_t ism330_get_gyr_x(void)
{
    uint8_t l, h;
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTX_L_G, &l);
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTX_H_G, &h);

    uint16_t ret = (h << 8) | l;
    return (int16_t)ret;
}

float ism330_get_gyr_x_dps(void)
{
    float fs_g = GYR_FS_2000;
    int16_t raw = ism330_get_gyr_x();
    
    float ret = 0.0;
    ret = ((float)raw * fs_g) / 32768.0;
    return ret;
}

int16_t ism330_get_gyr_y(void)
{
    uint8_t l, h;
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTY_L_G, &l);
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTY_H_G, &h);

    uint16_t ret = (h << 8) | l;
    return (int16_t)ret;
}

float ism330_get_gyr_y_dps(void)
{
    float fs_g = GYR_FS_2000;
    int16_t raw = ism330_get_gyr_y();
    
    float ret = 0.0;
    ret = ((float)raw * fs_g) / 32768.0;
    return ret;
}

int16_t ism330_get_gyr_z(void)
{
    uint8_t l, h;
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTZ_L_G, &l);
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUTZ_H_G, &h);

    uint16_t ret = (h << 8) | l;
    return (int16_t)ret;
}

float ism330_get_gyr_z_dps(void)
{
    float fs_g = GYR_FS_2000;
    int16_t raw = ism330_get_gyr_z();
    
    float ret = 0.0;
    ret = ((float)raw * fs_g) / 32768.0;
    return ret;
}

int16_t ism330_get_temp(void)
{
    uint8_t l, h;
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUT_TEMP_L, &l);
    i2c_register_read_byte(ISM330_I2C_ADDR, ISM330_REG_OUT_TEMP_H, &h);

    uint16_t ret = (h << 8) | l;
    return (int16_t) ret;
}

float ism330_get_temp_celcius(void)
{
    int16_t temp = ism330_get_temp();
    float actualTemperature = ((float)temp / 256.0) + 25.0;
    return actualTemperature;
}

// TODO: Fix accelerometer conversions to match the gyro conversions (boost in performance).

float ism330_convert_acc_x_g(int16_t x)
{
    uint8_t fs_xl = ACC_FS_2G;
    float ret = 0.0;
    if (fs_xl == ACC_FS_2G)
    {
        ret = ((float)x * 2.0) / 32768.0;
    }
    return ret;
}

float ism330_convert_acc_y_g(int16_t y)
{
    uint8_t fs_xl = ACC_FS_2G;
    float ret = 0.0;
    if (fs_xl == ACC_FS_2G)
    {
        ret = ((float)y * 2.0) / 32768.0;
    }
    return ret;
}

float ism330_convert_acc_z_g(int16_t z)
{
    uint8_t fs_xl = ACC_FS_2G;
    float ret = 0.0;
    if (fs_xl == ACC_FS_2G)
    {
        ret = ((float)z * 2.0) / 32768.0;
    }
    return ret;
}

float ism330_convert_gyr_x_dps(int16_t x)
{
    float fs_g = GYR_FS_2000;
    
    float ret = 0.0;
    ret = ((float)x * fs_g) / 32768.0;
    return ret;
}

float ism330_convert_gyr_y_dps(int16_t y)
{
    float fs_g = GYR_FS_2000;
    
    float ret = 0.0;
    ret = ((float)y * fs_g) / 32768.0;
    return ret;
}

float ism330_convert_gyr_z_dps(int16_t z)
{
    float fs_g = GYR_FS_2000;
    
    float ret = 0.0;
    ret = ((float)z * fs_g) / 32768.0;
    return ret;
}
