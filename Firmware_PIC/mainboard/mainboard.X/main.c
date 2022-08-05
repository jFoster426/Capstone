#include <xc.h>

#include "cmp1.h"
#include "config.h"
#include "i2c1.h"
#include "interrupt.h"
#include "io.h"
#include "uart1.h"

void main(void)
{
    interrupt_init();
    system_init();
    io_init();
    I2C1_init();
    UART1_Initialize();
    CMP1_Initialize();
    
    INTERRUPT_GlobalInterruptHighEnable();
    INTERRUPT_GlobalInterruptLowEnable();
    
    io_VDD_PIC_EN_write(IO_HIGH);
    io_VDD_5V_EN_write(IO_HIGH);
    io_VDD_3V3_CE_write(IO_HIGH);
    // io_PIC_LED_write(IO_HIGH);
    
    while (1)
    {
        uint8_t data = UART1_Read();
        LATBbits.LATB4=1;
        __delay_ms(100);
        LATBbits.LATB4=0;
        __delay_ms(100);
    }
}
