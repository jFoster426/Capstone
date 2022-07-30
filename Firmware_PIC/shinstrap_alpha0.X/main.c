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

/*
                         Main application
 */
void main(void)
{
    // Initialize the device
    // SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    // INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();
    
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
    
    while (1)
    {
        LATCbits.LATC3 = 1;
        LATCbits.LATC4 = 0;
        for (volatile uint32_t i = 0; i < 50000; i++) Nop();
        LATCbits.LATC3 = 0;
        LATCbits.LATC4 = 1;
        for (volatile uint32_t i = 0; i < 50000; i++) Nop();
        
    }
    
}
    
    
    
    /*
    
}
    
    
    // I2C1_Open();
    
    // VDD_3V3_CE = RC7.
    LATCbits.LATC7 = 1;
    
    // VDD_PIC_EN = RB5.
    LATBbits.LATB5 = 1;
    
    // VDD_5V_EN = RB6.
    LATBbits.LATB6 = 1;
    
    // VDD_3V3_PGOOD = RB7.
    // PIC_LED = RB4.
    
    for (volatile uint32_t i = 0; i < 500000; i++) Nop();
    
    while (1)
    {
        if (PORTBbits.RB7 == 1)
        {
            LATBbits.LATB4 = 1;
            for (volatile uint32_t i = 0; i < 50000; i++) Nop();
            LATBbits.LATB4 = 0;
            for (volatile uint32_t i = 0; i < 50000; i++) Nop();
        }
        else
        {
            LATCbits.LATC7 = 0;
            for (volatile uint32_t i = 0; i < 50000; i++) Nop();
            LATCbits.LATC7 = 1;
            for (volatile uint32_t i = 0; i < 50000; i++) Nop();
        }
    }
}
*/