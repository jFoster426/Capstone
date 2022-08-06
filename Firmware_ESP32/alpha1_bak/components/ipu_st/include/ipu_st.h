#ifndef ipu_st_h
#define ipu_st_h

#include <stdio.h>
#include "esp_log.h"

#include "../../i2c_st/include/i2c_st.h"

// Shin strap I2C address definition.
#define IPU_I2C_ADDR                                (0x64)

// Shin strap register definitions.
#define IPU_REG_CFG                                 (0x00)
// Bit definitions:
// Bit 7 - 
// Bit 6 - 
// Bit 5 - 
// Bit 4 - 
// Bit 3 - 
// Bit 2 - 
// Bit 1 - 
// Bit 0 - 

#define IPU_REG_PWR_CFG
// Bit definitions:
// Bit 7 - VDD_5V_EN
// Bit 6 - VDD_3V3_EN
// Bit 5 - 
// Bit 4 - 
// Bit 3 - 
// Bit 2 - 
// Bit 1 - 
// Bit 0 - 

#define IPU_REG_PWR_STATUS
// Bit definitions:
// Bit 7 - VDD_3V3_PGOOD
// Bit 6 - VDD_5V_PGOOD. 5V rail in the range of 4.7 - 5.3 V.
// Bit 5 - USB_VBUS_CONN. Detects prescense of the USB lead, as well as bus voltage within the range of 4.7 - 5.3 V.
// Bit 4 - VBATT_PRESENT. Detects prescensce of the battery connection.
// Bit 3 - 
// Bit 2 - 
// Bit 1 - 
// Bit 0 - 


#define IPU_REG_VBATT_FB
// VBATT_FB voltage readout.
// 8 bits represents 0 - 10V.
// Resolution: 39.0625 mV / bit.
// Offset: 0V.

#define IPU_REG_VBATT_PERCENT
// VBATT battery percentage readout.
// Converts directly to a percentage from 0 - 100%.

#define IPU_REG_USB_VBUS_FB
// USB_VBUS_FB voltage readout.
// 8 bits represents 0 - 10V.
// Resolution: 39.0625 mV / bit.
// Offset: 0V.

#define IPU_REG_5V_FB
// 5V_FB voltage readout.
// 8 bits represents 0 - 10V.
// Resolution: 39.0625 mV / bit.
// Offset: 0V.

#define IPU_REG_BTN_STATUS
// Bit definitions:
// Bit 7 - BTN1 state (1: pin is low (button pressed), 0: pin is high (button not pressed).
// Bit 6 - BTN1 status (1: has been pressed, 0: not pressed since last poll. Read this register to clear this bit).
// Bit 5 - BTN1 press - hold (1: press - hold has been triggered since last poll, 0: no new event. Read this register to clear this bit).
// Bit 4 - Unused.
// Bit 3 - BTN1 state (1: pin is low (button pressed), 0: pin is high (button not pressed).
// Bit 2 - BTN1 status (1: has been pressed, 0: not pressed since last poll. Read this register to clear this bit).
// Bit 1 - BTN1 press - hold (1: press - hold has been triggered since last poll, 0: no new event. Read this register to clear this bit).
// Bit 0 - Unused.

esp_err_t ipu_init(void);

esp_err_t ipu_read_reg(uint8_t reg, uint8_t *val);

esp_err_t ipu_write_reg(uint8_t reg, uint8_t val);

#endif