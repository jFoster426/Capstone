/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F16Q40
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"

/*
                         Main application
 */
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();
    
    // nSS_TX_EN.
    LATCbits.LATC2 = 0;
    
    while (1)
    {
        // Attempt to communicate to the IMU.
        uint8_t who_am_i = I2C1_Read1ByteRegister(0x6B, 0x0F);
        if (who_am_i == 0x6B)
        {
            // Green LED on.
            LATCbits.LATC4 = 1;
            
            // Red LED off.
            LATCbits.LATC3 = 0;
            
            // IMU communication was successful, proceed to the main program.
            break;
        }
        else
        {
            // Red LED on.
            LATCbits.LATC3 = 1;
            
            // Green LED off.
            LATCbits.LATC4 = 0;
        }
        // Small delay as to not spam the I2C bus unnecessarily.
        __delay_ms(100);
    }
    
    // ISM330 initial initialization.
    I2C1_Write1ByteRegister(0x6B, 0x18, 0b00000010);
    I2C1_Write1ByteRegister(0x6B, 0x10, 0b10100000);
    // 2000DPS.
    I2C1_Write1ByteRegister(0x6B, 0x11, 0b10101100);
    I2C1_Write1ByteRegister(0x6B, 0x18, 0b00000000);
    
    while (1)
    {
        // Sync bit.
        UART1_Write(0xAA);
        // Data registers for accelerometer and gyroscope.
        for (uint8_t i = 0x22; i <= 0x2D; i++)
        {
            uint8_t dataByte = I2C1_Read1ByteRegister(0x6B, i);
            UART1_Write(dataByte);
        }
    }
}
/**
 End of File
*/