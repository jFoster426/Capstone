/**
  I2C1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    i2c1_slave.c

  @Summary
    This is the generated driver implementation file for the I2C1 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides implementations for driver APIs for I2C1.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F16Q40
        Driver Version    :  2.0.1
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above or later
        MPLAB             :  MPLAB X 6.00
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

#include "i2c1.h"
#include <xc.h>

#define I2C1_SLAVE_ADDRESS      100
#define I2C1_SLAVE_MASK         127

typedef enum
{
    I2C1_IDLE,
    I2C1_ADDR_TX,
    I2C1_ADDR_RX,
    I2C1_DATA_TX,
    I2C1_DATA_RX
} i2c1_slave_state_t;

/**
 Section: Global Variables
 */
volatile uint8_t i2c1WrData;
volatile uint8_t i2c1RdData;
volatile uint8_t i2c1SlaveAddr;
static volatile i2c1_slave_state_t i2c1SlaveState = I2C1_IDLE;

volatile uint8_t Ji2c_state = 0;
volatile uint8_t Jaccess_reg = 0;
volatile uint8_t Ji2c_registers[128] = { 0 };

/**
 Section: Function declarations
 */
static void I2C1_Isr(void);
static void I2C1_SlaveDefRdInterruptHandler(void);
static void I2C1_SlaveDefWrInterruptHandler(void);
static void I2C1_SlaveDefAddrInterruptHandler(void);
static void I2C1_SlaveDefWrColInterruptHandler(void);
static void I2C1_SlaveDefBusColInterruptHandler(void);

static void I2C1_SlaveRdCallBack(void);
static void I2C1_SlaveWrCallBack(void);
static void I2C1_SlaveAddrCallBack(void);
static void I2C1_SlaveWrColCallBack(void);
static void I2C1_SlaveBusColCallBack(void);

static inline bool I2C1_SlaveOpen();
static inline void I2C1_SlaveClose();
static inline void I2C1_SlaveSetSlaveAddr(uint8_t slaveAddr);
static inline void I2C1_SlaveSetSlaveMask(uint8_t maskAddr);
static inline void I2C1_SlaveEnableIrq(void);
static inline bool I2C1_SlaveIsAddr(void);
static inline bool I2C1_SlaveIsRead(void);
static inline void I2C1_SlaveClearAddrFlag(void);
static inline void I2C1_SlaveClearStartFlag(void);
static inline bool I2C1_SlaveIsStop(void);
static inline void I2C1_SlaveClearBuff(void);
static inline void I2C1_SlaveClearIrq(void);
static inline void I2C1_SlaveSetCounter(uint8_t counter);
static inline void I2C1_SlaveReleaseClock(void);
static inline bool I2C1_SlaveIsWriteCollision(void);
static inline bool I2C1_SlaveIsTxBufEmpty(void);
static inline bool I2C1_SlaveIsData(void);
static inline bool I2C1_SlaveIsRxBufFull(void);
static inline void I2C1_SlaveSendTxData(uint8_t data);
static inline uint8_t I2C1_SlaveGetRxData(void);
static inline uint8_t I2C1_SlaveGetAddr(void);
static inline void I2C1_SlaveSendAck(void);
static inline void I2C1_SlaveSendNack(void);
static inline bool I2C1_SlaveIsNack(void);
static inline void I2C1_SlaveClearNack(void);

void I2C1_init()
{
    //EN disabled; RSEN disabled; S Cleared by hardware after Start; CSTR Enable clocking; MODE two 7-bit address with masking; 
    I2C1CON0 = 0x01;
    //ACKCNT Acknowledge; ACKDT Acknowledge; P Cleared by hardware after sending Stop; RXO No overflow; TXU No underflow; CSD Clock Stretching enabled; 
    I2C1CON1 = 0x00;
    //ACNT disabled; GCEN disabled; FME disabled; ABD enabled; SDAHT 300 ns hold time; BFRET 8 I2C Clock pulses; 
    I2C1CON2 = 0x00;
    //CNT 0; 
    I2C1CNTL = 0x00;
    I2C1CNTH = 0x00;
    //BAUD 0; 
    I2C1BAUD = 0x00;
    
    I2C1_Open();
}

void I2C1_Open() 
{
    I2C1_SlaveOpen();
    I2C1_SlaveSetSlaveAddr(I2C1_SLAVE_ADDRESS);
    I2C1_SlaveSetSlaveMask(I2C1_SLAVE_MASK);
    I2C1_SlaveSetIsrHandler(I2C1_Isr);
    I2C1_SlaveSetBusColIntHandler(I2C1_SlaveDefBusColInterruptHandler);
    I2C1_SlaveSetWriteIntHandler(I2C1_SlaveDefWrInterruptHandler);
    I2C1_SlaveSetReadIntHandler(I2C1_SlaveDefRdInterruptHandler);
    I2C1_SlaveSetAddrIntHandler(I2C1_SlaveDefAddrInterruptHandler);
    I2C1_SlaveSetWrColIntHandler(I2C1_SlaveDefWrColInterruptHandler);
    I2C1_SlaveEnableIrq();    
    
    TRISCbits.TRISC3=0;
    TRISCbits.TRISC6=0;

    ANSELCbits.ANSELC3=0;
    ANSELCbits.ANSELC6=0;

    WPUCbits.WPUC3=0;
    WPUCbits.WPUC6=0;

    ODCONCbits.ODCC3=1;
    ODCONCbits.ODCC6=1;

	
    I2C1SDAPPS = 0x13;   //RC3->I2C1:SDA1;    
    RC3PPS = 0x22;   //RC3->I2C1:SDA1;    
    RC6PPS = 0x21;   //RC6->I2C1:SCL1;    
    I2C1SCLPPS = 0x16;   //RC6->I2C1:SCL1;    
}

void I2C1_Close() 
{
    I2C1_SlaveClose();
}

uint8_t I2C1_Read()
{
   return I2C1_SlaveGetRxData();
}

void I2C1_Write(uint8_t data)
{
    I2C1_SlaveSendTxData(data);
}

uint8_t I2C1_ReadAddr()
{
   return I2C1_SlaveGetAddr();
}

bool I2C1_IsRead()
{
    return I2C1_SlaveIsRead();
}

void I2C1_Enable()
{
    //  I2C1_Initialize();
}

void I2C1_SendAck()
{
    I2C1_SlaveSendAck();
}

void I2C1_SendNack()
{
    I2C1_SlaveSendNack();
}

static void I2C1_Isr() 
{ 
    if(I2C1_SlaveIsStop())
    {
        I2C1_SlaveClearIrq();
        I2C1_SlaveClearBuff();
        I2C1_SlaveSetCounter(0xFF);
        I2C1_SlaveReleaseClock();
        i2c1SlaveState = I2C1_IDLE;
        
        // Stop condition resets the state machine.
        Ji2c_state = 0;
    }
    else
    {
        I2C1_SlaveClearIrq();
        if(I2C1_SlaveIsAddr())
        {
            if(I2C1_SlaveIsRead())
            {
                i2c1SlaveState = I2C1_ADDR_TX;
            }
            else
            {
                i2c1SlaveState = I2C1_ADDR_RX;
            }
        }
        else
        {
            if(I2C1_SlaveIsRead())
            {
                i2c1SlaveState = I2C1_DATA_TX;
            }
            else
            {
                i2c1SlaveState = I2C1_DATA_RX;
            }
        }

        switch(i2c1SlaveState)
        {
            case I2C1_ADDR_TX:
                I2C1_SlaveAddrCallBack();
                I2C1_SlaveClearBuff();
                if(I2C1_SlaveIsTxBufEmpty())
                {
                    I2C1_SlaveWrCallBack();
                }
                break;
            case I2C1_ADDR_RX:
                I2C1_SlaveAddrCallBack();
                I2C1_SlaveClearBuff();
                break;
            case I2C1_DATA_TX:
                if(I2C1_SlaveIsTxBufEmpty())
                {
                    I2C1_SlaveWrCallBack();
                }
                break;
            case I2C1_DATA_RX:
                if(I2C1_SlaveIsRxBufFull())
                {
                    I2C1_SlaveRdCallBack();
                }
                break;
            default:
                break;
        }
        I2C1_SlaveReleaseClock();
    }
}

// Common Event Interrupt Handlers
void I2C1_SlaveSetIsrHandler(i2c1InterruptHandler handler)
{
    I2C1_InterruptHandler = handler;
}

// Read Event Interrupt Handlers
void I2C1_SlaveSetReadIntHandler(i2c1InterruptHandler handler) {
    I2C1_SlaveRdInterruptHandler = handler;
}

static void I2C1_SlaveRdCallBack() {
    // Add your custom callback code here
    if (I2C1_SlaveRdInterruptHandler) 
    {
        I2C1_SlaveRdInterruptHandler();
    }
}

static void I2C1_SlaveDefRdInterruptHandler()
{
    i2c1RdData = I2C1_SlaveGetRxData();
    // This is the first data byte transferred, so we know it has to be
    // a register.
    if (Ji2c_state == 0)
    {
        Jaccess_reg = i2c1RdData;
    }
    // If this is not the first data byte transferred, then we know that
    // the master wants to write data to the slave now.
    else
    {
        Ji2c_registers[Jaccess_reg] = i2c1RdData;
    }
    // Increment to show that the master has sent the slave some data
    // on this transaction.
    Ji2c_state++;
}

// Write Event Interrupt Handlers
void I2C1_SlaveSetWriteIntHandler(i2c1InterruptHandler handler) {
    I2C1_SlaveWrInterruptHandler = handler;
}

static void I2C1_SlaveWrCallBack() {
    // Add your custom callback code here
    if (I2C1_SlaveWrInterruptHandler) 
    {
        I2C1_SlaveWrInterruptHandler();
    }
}

static void I2C1_SlaveDefWrInterruptHandler()
{
    // Send back data from one of the registers to the master. Since
    // Jaccess_reg should already be updated from the previous read request,
    // we can just send out the data.
    I2C1_SlaveSendTxData(Ji2c_registers[Jaccess_reg]);
}

// ADDRESS Event Interrupt Handlers
void I2C1_SlaveSetAddrIntHandler(i2c1InterruptHandler handler){
    I2C1_SlaveAddrInterruptHandler = handler;
}

static void I2C1_SlaveAddrCallBack() {
    // Add your custom callback code here
    if (I2C1_SlaveAddrInterruptHandler) {
        I2C1_SlaveAddrInterruptHandler();
    }
}

static void I2C1_SlaveDefAddrInterruptHandler() {
    i2c1SlaveAddr = I2C1_SlaveGetAddr();
}

// Write Collision Event Interrupt Handlers
void I2C1_SlaveSetWrColIntHandler(i2c1InterruptHandler handler){
    I2C1_SlaveWrColInterruptHandler = handler;
}

static void  I2C1_SlaveWrColCallBack() {
    // Add your custom callback code here
    if ( I2C1_SlaveWrColInterruptHandler) 
    {
         I2C1_SlaveWrColInterruptHandler();
    }
}

static void I2C1_SlaveDefWrColInterruptHandler() {
}

// Bus Collision Event Interrupt Handlers
void I2C1_SlaveSetBusColIntHandler(i2c1InterruptHandler handler){
    I2C1_SlaveBusColInterruptHandler = handler;
}

static void  I2C1_SlaveBusColCallBack() {
    // Add your custom callback code here
    if ( I2C1_SlaveBusColInterruptHandler) 
    {
         I2C1_SlaveBusColInterruptHandler();
    }
}

static void I2C1_SlaveDefBusColInterruptHandler() {
}

static inline bool I2C1_SlaveOpen()
{
    if(!I2C1CON0bits.EN)
    {  
        I2C1CON0bits.EN = 1;
        return true;
    }
    return false;
}

static inline void I2C1_SlaveClose()
{
    I2C1CON0bits.EN = 0;
    I2C1PIR = 0x00;
    I2C1STAT1bits.CLRBF = 1;
}

static inline void I2C1_SlaveSetSlaveAddr(uint8_t slaveAddr)
{
    I2C1ADR0 = (uint8_t) (slaveAddr << 1);
    I2C1ADR2 = (uint8_t) (slaveAddr << 1);
}

static inline void I2C1_SlaveSetSlaveMask(uint8_t maskAddr)
{
    I2C1ADR1 = (uint8_t) (maskAddr << 1);
    I2C1ADR3 = (uint8_t) (maskAddr << 1);
}

static inline void I2C1_SlaveEnableIrq()
{
    PIE7bits.I2C1IE    = 1;
    PIE7bits.I2C1EIE   = 1;
    PIE7bits.I2C1RXIE  = 1;
    PIE7bits.I2C1TXIE  = 1;
    I2C1PIEbits.PCIE = 1; 
    I2C1PIEbits.ADRIE = 1; 
    I2C1ERRbits.NACKIE = 1; 
}

static inline bool I2C1_SlaveIsAddr()
{
    return !(I2C1STAT0bits.D); 
}

static inline bool I2C1_SlaveIsRead()
{
    return I2C1STAT0bits.R; 
}

static inline void I2C1_SlaveClearAddrFlag()
{
    I2C1PIRbits.ADRIF = 0; 
}

static inline void I2C1_SlaveClearStartFlag()
{
    I2C1PIRbits.SCIF = 0; 
}

static inline bool I2C1_SlaveIsStop()
{
    return I2C1PIRbits.PCIF; 
}

static inline void I2C1_SlaveClearBuff()
{
    I2C1STAT1bits.CLRBF = 1; 
}

static inline void I2C1_SlaveClearIrq()
{
    I2C1PIR = 0x00; 
}

static inline void I2C1_SlaveSetCounter(uint8_t counter)
{
    I2C1CNTL = counter;
    I2C1CNTH = 0x00;
}

static inline void I2C1_SlaveReleaseClock()
{
    I2C1CON0bits.CSTR = 0;
}

static inline bool I2C1_SlaveIsWriteCollision()
{
    return I2C1ERRbits.BCLIF;
}

static inline bool I2C1_SlaveIsTxBufEmpty()
{
    return I2C1STAT1bits.TXBE;
}

static inline bool I2C1_SlaveIsData()
{
    return I2C1STAT0bits.D;
}

static inline bool I2C1_SlaveIsRxBufFull()
{
    return I2C1STAT1bits.RXBF;
}

static inline void I2C1_SlaveSendTxData(uint8_t data)
{
    I2C1TXB = data;
}

static inline uint8_t I2C1_SlaveGetRxData()
{
    return I2C1RXB;
}

static inline uint8_t I2C1_SlaveGetAddr()
{
    return I2C1ADB0 >> 1;
}

static inline void I2C1_SlaveSendAck()
{
    I2C1CON1bits.ACKDT = 0;
}


static inline void I2C1_SlaveSendNack()
{
    I2C1CON1bits.ACKDT = 1;
}

static inline bool I2C1_SlaveIsNack()
{
    return I2C1ERRbits.NACKIF;
}

static inline void I2C1_SlaveClearNack()
{
    I2C1ERRbits.NACKIF = 0;
}

void __interrupt(irq(I2C1TX,I2C1RX,I2C1E,I2C1),base(8)) I2C1_ISR()
{
    I2C1_Isr();
}
