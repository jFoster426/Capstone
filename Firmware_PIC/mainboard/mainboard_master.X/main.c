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
    // Initialize the device.
    SYSTEM_Initialize();

    // Enable high priority global interrupts.
    INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    INTERRUPT_GlobalInterruptLowEnable();

    // nSS_TX_EN.
    LATCbits.LATC2 = 1;
    
    // VDD_PIC_EN.
    LATBbits.LATB5 = 1;
    
    // VDD_5V_EN.
    LATBbits.LATB6 = 1;
    
    // VDD_3V3_CE.
    LATCbits.LATC7 = 1;
    
    bool measurementStarted = false;

    while (1)
    {
        // Attempt to communicate to the IMU.
        uint8_t who_am_i = I2C1_Read1ByteRegister(0x6B, 0x0F);
        if (who_am_i == 0x6B)
        {
            // IMU communication was successful, proceed to the main program.
            break;
        }
        // Small delay as to not spam the I2C bus unnecessarily.
        __delay_ms(100);
    }

    // ISM330 initial initialization.
    // +/- 2G
    I2C1_Write1ByteRegister(0x6B, 0x18, 0b00000010);
    I2C1_Write1ByteRegister(0x6B, 0x10, 0b01000000);
    // +/- 2000DPS.
    I2C1_Write1ByteRegister(0x6B, 0x11, 0b01001100);
    I2C1_Write1ByteRegister(0x6B, 0x18, 0b00000000);
    
    // combined_data packet structure:
    // 0      '$'
    // 1-12   foot_imu_data
    // 13-24  shin_imu_data
    // 25     Button 2 state
    // 26     
    // 27     
    // 28     V_BATT_FB ADC reading
    // 29     USB_VBUS_FB ADC reading
    // 30     '\n'
    // 31     '\0'
    
    combined_data[0] = '$';
    
    uint8_t *foot_imu_data = &combined_data[1];
    uint8_t *shin_imu_data = &combined_data[13];
    
    combined_data[30] = '\n';
    combined_data[31] = '\0';
    
    while (1)
    {
        // Stop the measurement if started and button pressed.
        if (measurementStarted == true && PORTAbits.RA5 == 0)
        {
            measurementStarted = false;
            __delay_ms(50);
            while (PORTAbits.RA5 == 0);
            __delay_ms(50);
        }
        
        // Start the measurement if the button pressed and measurement started.
        else if (measurementStarted == false && PORTAbits.RA5 == 0)
        {
            measurementStarted = true;
            __delay_ms(50);
            while (PORTAbits.RA5 == 0);
            __delay_ms(50);
        }
        
        // Fill foot imu data.
        for (uint8_t i = 0x22; i <= 0x2D; i++)
        {
            foot_imu_data[i - 0x22] = I2C1_Read1ByteRegister(0x6B, i);
        }
        
        bool dataValid = false;
        uint8_t i;
        
        // Transfer shin imu out of DMA.
        for (i = 0; i < 13; i++)
        {
            if (uart_dma_data[i] == 0xAA)
            {
                uart_dma_data[i] = 0;
                dataValid = true;
                break;
            }
        }
        
        
        
        // Parse shin IMU data into correct bin.
        for (uint8_t j = 0; j < 12; j++)
        {
            if (i == 12) i = 0;
            else i++;
            // Only proceed if a successful transaction with the shinstrap.
            if (dataValid == false || measurementStarted == false)
            {
                shin_imu_data[j] = '$';
                LATBbits.LATB4 = 0;
            }
            else
            {
                shin_imu_data[j] = uart_dma_data[i];
                LATBbits.LATB4 = 1;
            }
        }
    }
}

/**
 End of File
 */