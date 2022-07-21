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

void OSCILLATOR_Initialize(void)
{
    // NOSC HFINTOSC; NDIV 1; 
    OSCCON1 = 0x60;
    // CSWHOLD may proceed; SOSCPWR Low power; 
    OSCCON3 = 0x00;
    // MFOEN disabled; LFOEN disabled; ADOEN disabled; PLLEN enabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    OSCEN = 0x01;
    // HFFRQ 64_MHz; 
    OSCFRQ = 0x08;
    // TUN 0; 
    OSCTUNE = 0x00;
    // ACTUD enabled; ACTEN disabled; 
    ACTCON = 0x00;
}

void PMD_Initialize(void)
{
    // CLKRMD CLKR enabled; SYSCMD SYSCLK enabled; SCANMD SCANNER enabled; LVDMD HLVD enabled; FVRMD FVR enabled; IOCMD IOC enabled; CRCMD CRC enabled; 
    PMD0 = 0x00;
    // ZCDMD ZCD enabled; TMR0MD TMR0 enabled; TMR1MD TMR1 enabled; TMR4MD TMR4 enabled; SMT1MD SMT1 enabled; TMR2MD TMR2 enabled; TMR3MD TMR3 enabled; CM1MD CM1 enabled; 
    PMD1 = 0x00;
    // NCO1MD NCO1 enabled; ADCMD ADC enabled; DSM1MD DSM enabled; CWG1MD CWG1 enabled; ACTMD ACT enabled; CM2MD CM2 enabled; DAC1MD DAC1 enabled; 
    PMD2 = 0x00;
    // PWM2MD PWM2 enabled; PWM1MD PWM1 enabled; PWM3MD PWM3 enabled; SPI2MD SPI2 enabled; SPI1MD SPI1 enabled; U2MD UART2 enabled; U1MD UART1 enabled; I2C1MD I2C1 enabled; 
    PMD3 = 0x00;
    // CLC3MD CLC3 enabled; CLC4MD CLC4 enabled; DMA1MD DMA1 enabled; DMA2MD DMA2 enabled; DMA3MD DMA3 enabled; CLC1MD CLC1 enabled; CLC2MD CLC2 enabled; 
    PMD4 = 0x00;
    // DMA4MD DMA4 enabled; DAC2MD DAC2 enabled; 
    PMD5 = 0x00;
}

void PIN_MANAGER_Initialize(void)
{
    /**
    LATx registers
    */
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;

    /**
    TRISx registers
    */
    TRISA = 0x3F;
    TRISB = 0xC0;
    TRISC = 0x37;

    /**
    ANSELx registers
    */
    ANSELC = 0xB7;
    ANSELB = 0x70;
    ANSELA = 0x37;

    /**
    WPUx registers
    */
    WPUB = 0x00;
    WPUA = 0x00;
    WPUC = 0x00;

    /**
    ODx registers
    */
    ODCONA = 0x00;
    ODCONB = 0x00;
    ODCONC = 0x4C;

    /**
    SLRCONx registers
    */
    SLRCONA = 0x37;
    SLRCONB = 0xF0;
    SLRCONC = 0xFF;

    /**
    INLVLx registers
    */
    INLVLA = 0x3F;
    INLVLB = 0xF0;
    INLVLC = 0xFF;
	
    I2C1SDAPPS = 0x16;   //RC6->I2C1:SDA1;    
    RC3PPS = 0x21;   //RC3->I2C1:SCL1;    
    RC6PPS = 0x22;   //RC6->I2C1:SDA1;    
    I2C1SCLPPS = 0x13;   //RC3->I2C1:SCL1;    
}

void I2C1_Initialize()
{
    I2C1CON0 = 0x01;

    I2C1CON1 = 0x00;
    
    I2C1CON2 = 0x00;
    
    I2C1CNTL = 0x02;
    I2C1CNTH = 0x00;
}

#define I2C1_SLAVE_ADDRESS 100
#define I2C1_SLAVE_MASK    127

void I2C1_SlaveSetSlaveAddr(uint8_t slaveAddr)
{
    I2C1ADR0 = (uint8_t) (slaveAddr << 1);
    I2C1ADR2 = (uint8_t) (slaveAddr << 1);
}

void I2C1_SlaveSetSlaveMask(uint8_t maskAddr)
{
    I2C1ADR1 = (uint8_t) (maskAddr << 1);
    I2C1ADR3 = (uint8_t) (maskAddr << 1);
}

// Define interrupt handler.
void __interrupt(irq(I2C1)) I2C1_InterruptHandler(void)
{
    I2C1PIR = 0x00;
    
    if (I2C1PIRbits.SC1IF == 1)
    {
        LATBbits.LATB4 = 1;
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        LATBbits.LATB4 = 0;
    }
    
    if (I2C1PIRbits.PCIF == 1)
    {
        LATBbits.LATB4 = 1;
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        LATBbits.LATB4 = 0;
    }
    
    I2C1CNTL = 0x02;
}

void I2C1_Open() 
{
    I2C1CON0bits.EN = 1;
    I2C1_SlaveSetSlaveAddr(I2C1_SLAVE_ADDRESS);
    I2C1_SlaveSetSlaveMask(I2C1_SLAVE_MASK);
    PIE7bits.I2C1IE    = 1;
    PIE7bits.I2C1EIE   = 1;
    PIE7bits.I2C1RXIE  = 1;
    PIE7bits.I2C1TXIE  = 1;
    I2C1PIEbits.PCIE = 1;
    I2C1PIEbits.SCIE = 1;
    I2C1PIEbits.ADRIE = 1;
    I2C1ERRbits.NACKIE = 1;
}

void main(void)
{
    OSCILLATOR_Initialize();
    PMD_Initialize();
    PIN_MANAGER_Initialize();
    I2C1_Initialize();
    
    I2C1_Open();
    
    INTCON0bits.GIE = 1;
    
    // VDD_3V3_CE = RC7.
    LATCbits.LATC7 = 1;
    
    // VDD_5V_EN = RB5.
    LATBbits.LATB5 = 1;
    
    // PIC_LED = RB4.
    LATBbits.LATB4 = 0;
    
    while (1)
    {
//        LATBbits.LATB4 = 1;
//        for(volatile uint32_t i = 0; i < 300000; i++) Nop();
//        LATBbits.LATB4 = 0;
//        for(volatile uint32_t i = 0; i < 300000; i++) Nop();
    }
}
