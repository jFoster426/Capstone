#include "io.h"

void io_init(void)
{
    // BTN1 = RA4.
    TRISAbits.TRISA4 = 1;
    ANSELAbits.ANSELA4 = 0;
    
    // BTN2 = RA5.
    TRISAbits.TRISA5 = 1;
    ANSELAbits.ANSELA5 = 0;
    
    // PIC_LED = RB4.
    TRISBbits.TRISB4 = 0;
    
    // VDD_PIC_EN = RB5.
    TRISBbits.TRISB5 = 0;
    
    // VDD_5V_EN = RB6.
    TRISBbits.TRISB6 = 0;
    
    // VDD_3V3_PGOOD = RB7.
    TRISBbits.TRISB7 = 1;
    ANSELBbits.ANSELB7 = 0;
    
    // nSS_TX_EN = RC2.
    TRISCbits.TRISC2 = 0;
    
    // VDD_3V3_CE = RC7.
    TRISCbits.TRISC7 = 0;
}

uint8_t io_checkBtn1Status(void)
{
    return PORTAbits.RA4;
}

uint8_t io_checkBtn2Status(void)
{
    return PORTAbits.RA5;
}

void io_PIC_LED_write(uint8_t state)
{
    LATBbits.LATB4 = state;
}

void io_VDD_PIC_EN_write(uint8_t state)
{
    LATBbits.LATB5 = state;
}

void io_VDD_5V_EN_write(uint8_t state)
{
    LATBbits.LATB6 = state;
}

uint8_t io_checkVDD_3V3_PGOODState(void)
{
    return PORTBbits.RB7;
}

void io_nSS_TX_EN_write(uint8_t state)
{
    state == 0 ? LATCbits.LATC7 = 0 : 1;
}

void io_VDD_3V3_CE_write(uint8_t state)
{
    LATCbits.LATC7 = state;
}