#ifndef ism330_st_h
#define ism330_st_h

#include <stdio.h>
#include "esp_log.h"

#include "../../i2c_st/include/i2c_st.h"

// ISM330 I2C address definition.
#define ISM330_I2C_ADDR                             (0x6B)

// ISM330 register definitions.
#define ISM330_REG_FUNC_CGF_ACCESS                  (0x01)
#define ISM330_REG_PIN_CTRL                         (0x02)
#define ISM330_REG_FIFO_CTRL1                       (0x07)
#define ISM330_REG_FIFO_CTRL2                       (0x08)
#define ISM330_REG_FIFO_CTRL3                       (0x09)
#define ISM330_REG_FIFO_CTRL4                       (0x0A)
#define ISM330_REG_COUNTER_BDR_REG1                 (0x0B)
#define ISM330_REG_COUNTER_BDR_REG2                 (0x0C)
#define ISM330_REG_INT1_CTRL                        (0x0D)
#define ISM330_REG_INT2_CTRL                        (0x0E)
#define ISM330_REG_WHO_AM_I                         (0x0F)
#define ISM330_REG_CTRL1_XL                         (0x10)
#define ISM330_REG_CTRL2_G                          (0x11)
#define ISM330_REG_CTRL3_C                          (0x12)
#define ISM330_REG_CTRL4_C                          (0x13)
#define ISM330_REG_CTRL5_C                          (0x14)
#define ISM330_REG_CTRL6_C                          (0x15)
#define ISM330_REG_CTRL7_G                          (0x16)
#define ISM330_REG_CTRL8_XL                         (0x17)
#define ISM330_REG_CTRL9_XL                         (0x18)
#define ISM330_REG_CTRL10_C                         (0x19)
#define ISM330_REG_ALL_INT_SRC                      (0x1A)
#define ISM330_REG_WAKE_UP_SRC                      (0x1B)
#define ISM330_REG_TAP_SRC                          (0x1C)
#define ISM330_REG_D6D_SRC                          (0x1D)
#define ISM330_REG_STATUS_REG                       (0x1E)
#define ISM330_REG_OUT_TEMP_L                       (0x20)
#define ISM330_REG_OUT_TEMP_H                       (0x21)
#define ISM330_REG_OUTX_L_G                         (0x22)
#define ISM330_REG_OUTX_H_G                         (0x23)
#define ISM330_REG_OUTY_L_G                         (0x24)
#define ISM330_REG_OUTY_H_G                         (0x25)
#define ISM330_REG_OUTZ_L_G                         (0x26)
#define ISM330_REG_OUTZ_H_G                         (0x27)
#define ISM330_REG_OUTX_L_A                         (0x28)
#define ISM330_REG_OUTX_H_A                         (0x29)
#define ISM330_REG_OUTY_L_A                         (0x2A)
#define ISM330_REG_OUTY_H_A                         (0x2B)
#define ISM330_REG_OUTZ_L_A                         (0x2C)
#define ISM330_REG_OUTZ_H_A                         (0x2D)
#define ISM330_REG_EMB_FUNC_STATUS_MAINPAGE         (0x35)
#define ISM330_REG_FSM_STATUS_A_MAINPAGE            (0x36)
#define ISM330_REG_FSM_STATUS_B_MAINPAGE            (0x37)
#define ISM330_REG_MLC_STATUS_MAINPAGE              (0x38)
#define ISM330_REG_STATUS_MASTER_MAINPAGE           (0x39)
#define ISM330_REG_FIFO_STATUS_MAINPAGE             (0x3A)
#define ISM330_REG_FIFO_STATUS1                     (0x3B)
#define ISM330_REG_FIFO_STATUS2                     (0x3C)
#define ISM330_REG_TIMESTAMP0                       (0x40)
#define ISM330_REG_TIMESTAMP1                       (0x41)
#define ISM330_REG_TIMESTAMP2                       (0x42)
#define ISM330_REG_TIMESTAMP3                       (0x43)
#define ISM330_REG_TAP_CGF0                         (0x56)
#define ISM330_REG_TAP_CFG1                         (0x57)
#define ISM330_REG_TAP_CFG2                         (0x58)
#define ISM330_REG_TAP_THS_6D                       (0x59)
#define ISM330_REG_INT_DUR2                         (0x5A)
#define ISM330_REG_WAKE_UP_THS                      (0x5B)
#define ISM330_REG_WAKE_UP_DUR                      (0x5C)
#define ISM330_REG_FREE_FALL                        (0x5D)
#define ISM330_REG_MD1_CFG                          (0x5E)
#define ISM330_REG_MD2_CFG                          (0x5F)
#define ISM330_REG_INTERNAL_FREQ_FINE               (0x63)
#define ISM330_REG_INT_OIS                          (0x6F)
#define ISM330_REG_CTRL1_OIS                        (0x70)
#define ISM330_REG_CTRL2_OIS                        (0x71)
#define ISM330_REG_CTRL3_OIS                        (0x72)
#define ISM330_REG_X_OFS_USR                        (0x73)
#define ISM330_REG_Y_OFS_USR                        (0x74)
#define ISM330_REG_Z_OFS_USR                        (0x75)
#define ISM330_REG_FIFO_DATA_OUT_TAG                (0x78)
#define ISM330_REG_FIFO_DATA_OUT_X_L                (0x79)
#define ISM330_REG_FIFO_DATA_OUT_X_H                (0x7A)
#define ISM330_REG_FIFO_DATA_OUT_Y_L                (0x7B)
#define ISM330_REG_FIFO_DATA_OUT_Y_H                (0x7C)
#define ISM330_REG_FIFO_DATA_OUT_Z_L                (0x7D)
#define ISM330_REG_FIFO_DATA_OUT_Z_H                (0x7E)

// ISM330 register settings definitions.
#define ACC_FS_2G                                   (0x00)

#define GYR_FS_500                                  (0x01)


void ism330_init(void);

int16_t ism330_get_acc_x(void);
float ism330_get_acc_x_g(void);
int16_t ism330_get_acc_y(void);
float ism330_get_acc_y_g(void);
int16_t ism330_get_acc_z(void);
float ism330_get_acc_z_g(void);

int16_t ism330_get_gyr_x(void);
float ism330_get_gyr_x_dps(void);
int16_t ism330_get_gyr_y(void);
float ism330_get_gyr_y_dps(void);
int16_t ism330_get_gyr_z(void);
float ism330_get_gyr_z_dps(void);

int16_t ism330_get_temp(void);
float ism330_get_temp_celcius(void);

#endif