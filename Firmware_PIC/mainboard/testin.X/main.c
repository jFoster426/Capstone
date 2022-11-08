#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/i2c1_slave.h"

// Register mapping:
// Address          Description                             Intended Use (R/W)
// -----------------------------------------------------------------------------
// 0x00             Shin strap gyroscope x-axis LSB         R
// 0x01             Shin strap gyroscope x-axis MSB         R
// 0x02             Shin strap gyroscope y-axis LSB         R
// 0x03             Shin strap gyroscope y-axis MSB         R
// 0x04             Shin strap gyroscope z-axis LSB         R
// 0x05             Shin strap gyroscope z-axis MSB         R
// 0x06             Shin strap accelerometer x-axis LSB     R
// 0x07             Shin strap accelerometer x-axis MSB     R
// 0x08             Shin strap accelerometer y-axis LSB     R
// 0x09             Shin strap accelerometer y-axis MSB     R
// 0x0A             Shin strap accelerometer z-axis LSB     R
// 0x0B             Shin strap accelerometer z-axis MSB     R
// 0x0C             Empty                                   N/A
// 0x0D             Button 1 state                          R
// 0x0E             Button 2 state                          R
// 0x0F             Empty                                   N/A
// 0x10             USB_VBUS_FB ADC reading LSB             R
// 0x11             USB_VBUS_FB ADC reading MSB             R
// 0x12             V_BATT_FB ADC reading LSB               R
// 0x13             V_BATT_FB ADC reading MSB               R
// 0x14             Empty                                   N/A
// 0x15             Sleep counter                           R/W
// 0x16             Shin strap connected                    R

extern volatile uint8_t Ji2c_registers[128];

uint8_t uartTimeout = 0;

uint8_t acquire_sync(void)
{
    // Turn off the LED to show sync loss.
    LATBbits.LATB4 = 0;
    
    // Show that the shin strap is not connected.
    Ji2c_registers[0x16] = 0;
    
    // First sync character.
    while (UART1_Read() != 0xAA);
    
    // Reduce the probability of error to very small.
    for (int i = 0; i < 10; i++)
    {
        // 12 data bits.
        for (int i = 0; i < 12; i++) UART1_Read();
        // Should be the next sync character.
        if (UART1_Read() != 0xAA)
            return false;
    }
    return true;
}

void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
    
//    PMD0 = 0xFF;
//    PMD1 = 0xFF;
//    PMD2 = 0xFF;
//    PMD3 = 0xFF;
//    PMD4 = 0xFF;
//    PMD5 = 0xFF;
    
    //VREGCON = 0b00100010;
    
    CPUDOZEbits.IDLEN = 0;
    //CPUDOZEbits.DOZE = 0b111;
    // DSEN = 1;
    Sleep();
    
    Nop();
    
    //I2C1_Open();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();

    while (1);
}
/**
 End of File
*/