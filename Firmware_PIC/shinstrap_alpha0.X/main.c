// CONFIG1
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_64MHZ

// CONFIG2
#pragma config CLKOUTEN = OFF
#pragma config PR1WAY = ON
#pragma config CSWEN = ON
#pragma config FCMEN = ON
#pragma config FCMENP = ON
#pragma config FCMENS = ON

// CONFIG3
#pragma config MCLRE = EXTMCLR
#pragma config PWRTS = PWRT_OFF
#pragma config MVECEN = ON
#pragma config IVT1WAY = ON
#pragma config LPBOREN = OFF
#pragma config BOREN = SBORDIS

// CONFIG4
#pragma config BORV = VBOR_1P9
#pragma config ZCD = OFF
#pragma config PPS1WAY = ON
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config XINST = OFF

// CONFIG5
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = OFF

// CONFIG6
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC

// CONFIG7
#pragma config BBSIZE = BBSIZE_512
#pragma config BBEN = OFF
#pragma config SAFEN = OFF
#pragma config DEBUG = OFF

// CONFIG8
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config WRTSAF = OFF
#pragma config WRTAPP = OFF

// CONFIG9
#pragma config CP = OFF

#include <xc.h>

void cmp1_init()
{
    // Disable global interrupts.
    //INTCON0bits.GIE = 0;
    // Enable CM1 hysteresis.
    CM1CON0bits.C1HYS = 1;
    // Enable CM1 rising and falling interrupt.
    //CM1CON1bits.INTP = 1;
    //CM1CON1bits.INTN = 1;
    // Reset CM1 interrupt flag and enable CM1 interrupt.
    //PIR1bits.C1IF = 0;
    //PIE1bits.CM1IE = 1;
    // Set the inverting channel to C1IN0-.
    CM1NCH = 0;
    // Set the non-inverting channel to C1IN0+.
    CM1PCH = 0;
    // Set the C1IN0- and C1IN0+ pins to inputs.
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    // Set the ANSEL bits for C1IN0- and C1IN0+ to MUX to the comparator.
    ANSELAbits.ANSELA0 = 1;
    ANSELAbits.ANSELA1 = 1;
    // Set RA2 as C1OUT.
    RA2PPS = 0x19;
    // Enable CM1.
    CM1CON0bits.C1EN = 1;
    // Enable global interrupts.
    //INTCON0bits.GIE = 1;
}

void uart1_init()
{
    // BRGS high speed; MODE DALI Control Device mode; RXEN enabled; TXEN enabled; ABDEN disabled; 
    U1CON0 = 0xB8;
    // RXBIMD Set RXBKIF on rising RX input; BRKOVR disabled; WUE disabled; SENDB disabled; ON enabled; 
    U1CON1 = 0x80;
    // TXPOL not inverted; FLO off; C0EN Checksum Mode 0; RXPOL not inverted; RUNOVF RX input shifter stops all activity; STP Transmit 1Stop bit, receiver verifies first Stop bit; 
    U1CON2 = 0x00;
    // BRGL 15; 
    U1BRGL = 0x0F;
    // BRGH 0; 
    U1BRGH = 0x00;
    // STPMD in middle of first Stop bit; TXWRE No error; 
    U1FIFO = 0x00;
    // ABDIF Auto-baud not enabled or not complete; WUIF WUE not enabled by software; ABDIE disabled; 
    U1UIR = 0x00;
    // ABDOVF Not overflowed; TXCIF 0; RXBKIF No Break detected; RXFOIF not overflowed; CERIF No Checksum error; 
    U1ERRIR = 0x00;
    // TXCIE disabled; FERIE disabled; TXMTIE disabled; ABDOVE disabled; CERIE disabled; RXFOIE disabled; PERIE disabled; RXBKIE disabled; 
    U1ERRIE = 0x00;
    // Select RC0 as U1RX input.
    U1RXPPSbits.PORT = 0b010;
    U1RXPPSbits.PIN = 0;
    
    INTCON0bits.GIE = 0;
}

uint8_t uart1_read(void)
{
    // Wait for valid data in the receive buffer.
    while(!PIR4bits.U1RXIF);
    // Return the data.
    return U1RXB;
}

void main(void)
{
    // PIC_LED.
    TRISBbits.TRISB4 = 0;
    
    // VDD_3V3_CE.
    TRISCbits.TRISC7 = 0;
    LATCbits.LATC7 = 1;
    
    // VDD_5V_EN.
    TRISBbits.TRISB5 = 0;
    LATBbits.LATB5 = 1;
    
    // VDD_3V3_PGOOD.
    ANSELBbits.ANSELB7 = 0;
    TRISBbits.TRISB7 = 1;
    
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
            for (volatile uint32_t i = 0; i < 500000; i++) Nop();
        }
    }
    
//    TRISCbits.TRISC3 = 0;
//    TRISCbits.TRISC4 = 0;
//    
//    LATCbits.LATC3 = 0;
//    LATCbits.LATC4 = 0;
//    
//    cmp1_init();
//    uart1_init();
//    
//    while (1)
//    {
//        if (uart1_read() == 'G')
//        {
//            LATCbits.LATC4 = 1;
//        }
//        else
//        {
//            LATCbits.LATC3 = 1;
//        }
//        for (uint32_t i = 0; i < 100000; i++) Nop();
//        LATCbits.LATC3 = 0;
//        LATCbits.LATC4 = 0;
//        for (uint32_t i = 0; i < 100000; i++) Nop();
//    }
//    return;
}

/*
void __interrupt(irq(CMP1), base(8)) CM1_ISR(void)
{
    // Clear CM1 interrupt flag.
    PIR1bits.C1IF = 0;
    LATCbits.LATC3 = 1;
}
*/