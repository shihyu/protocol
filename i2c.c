//=============================================================================
// Project:
// Module:      i2c.c
// Copyright:   Chilin Co. 2008
// Author:      enos
// Date:        Sep. 24, 2008
//=============================================================================
// Description: IIC driver
//
//=============================================================================
#include <8051.h>
#include <stdio.h>

/*****************************************************************************
 * local definition
 *****************************************************************************/
#define    HIGH        1
#define    LOW         0
#define    I2C_SCL     P1_6
#define    I2C_SDA     P1_7

/*****************************************************************************
 * Author:      enos
 * Function:    i2c_wait
 * Description: wait for some time to get proper I2c timing
 *****************************************************************************/
void i2c_wait(void)
{
    _asm
    nop nop nop nop nop
    nop nop nop nop nop
    _endasm;
}

/*****************************************************************************
 * Author:      enos
 * Function:    i2c_wait2
 * Description: wait for some time to get proper I2c timing
 *****************************************************************************/
void i2c_wait2(void)
{
    _asm
    nop nop nop nop nop
    _endasm;
}

/*****************************************************************************
 * Author:      enos
 * Function:    i2c_start
 * Description: I2C start signal.
 * <comment>
 *  SCL ________
 *              \_________
 *  SDA _____
 *           \____________
 *
 * Return value: None
 *****************************************************************************/
void i2c_start(void)
{
    // for second start signal on i2c_read
    I2C_SDA = HIGH;
    I2C_SCL = HIGH;
    i2c_wait();

    // send start signal
    I2C_SDA = LOW;
    i2c_wait2();
    I2C_SCL = LOW;
}

/*****************************************************************************
 * Author:      enos
 * Function:    i2c_stop
 * Description: I2C stop signal.
 * <comment>
 *              ____________
 *  SCL _______/
 *                 _________
 *  SDA __________/
 *
 * assume I2C_SCL = LOW, I2C_SDA = LOW
 *****************************************************************************/
void i2c_stop(void)
{
    i2c_wait2();
    I2C_SDA = LOW;
    i2c_wait2();
    I2C_SCL = HIGH;
    i2c_wait2();
    I2C_SDA = HIGH;
}

/*****************************************************************************
 * Author:      enos
 * Function:    i2c_write
 * Description: I2C write command
 * I2C_SCL = LOW    data change
 * I2C_SCL = HIGH   data stable
 *****************************************************************************/
bit i2c_write(unsigned char value)
{
    char i=9;
    bit  ack;

    while(--i)
    {
        // upload data
        i2c_wait2();
        I2C_SDA = (value & 0x80) ? HIGH : LOW;
        i2c_wait2();
        // send data
        I2C_SCL = HIGH;
        i2c_wait();
        value <<= 1;
        I2C_SCL = LOW;
    }

    // get acknowledgement
    i2c_wait2();
    I2C_SDA = HIGH;
    i2c_wait2();
    I2C_SCL = HIGH;
    i2c_wait2();
    ack = I2C_SDA;
    i2c_wait2();
    I2C_SCL = LOW;

    return ack;
}

/*****************************************************************************
 * Author:      enos
 * Function:    i2c_read
 * Description: I2C read command
 * I2C_SCL = LOW    data change
 * I2C_SCL = HIGH   data stable
 *****************************************************************************/
unsigned char i2c_read(bit acknowledge)
{
    unsigned char value=0;
    char i=9;

    // read data
     while(--i)
    {
        // download data
        value <<= 1;
        i2c_wait();
        // get data
        I2C_SCL = HIGH;
        i2c_wait2();
        value |= I2C_SDA;
        i2c_wait2();
        I2C_SCL = LOW;
    }

    // send acknowledge
    i2c_wait2();
    I2C_SDA = acknowledge;
    i2c_wait2();
    I2C_SCL = HIGH;
    i2c_wait();
    I2C_SCL = LOW;

    // return data
    return value;
}

/*****************************************************************************
 * Author:      enos
 * Function:    i2c_write_byte
 * Description: write one byte
 *****************************************************************************/
void i2c_write_byte(unsigned char slave_addr, unsigned char reg_addr, unsigned char value)
{
    char i=10;

    EA = 0;               // Disable All Interrupt
    while (--i)
    {
        i2c_start();                        // start I2C
        if(i2c_write(slave_addr)) continue; // slave address
        if(i2c_write(reg_addr))  continue;  // register address
        if(i2c_write(value))     continue;  // send data
        i2c_stop();                         // stop I2C
        break;
    }
    EA = 1;                // Enable All Interrupt
}

/*****************************************************************************
 * Author:      enos
 * Function:    i2c_read_byte
 * Description: read one byte
 *****************************************************************************/
unsigned char i2c_read_byte(unsigned char slave_addr, unsigned char reg_addr)
{
    char i=10;
    unsigned char value=0;

    EA = 0;               // Disable All Interrupt
    while (--i)
    {
        // send register address
        i2c_start();                                // start I2C
        if(i2c_write(slave_addr&0xfe)) continue;    // slave address
        if(i2c_write(reg_addr)) continue;           // register address
        // read data
        i2c_start();                                // restart I2C
        if(i2c_write(slave_addr | 1)) continue;     // slave address
        value = i2c_read(1);                        // read data
        i2c_stop();                                 // stop I2C
        break;
    }
    EA = 1;                // Enable All Interrupt

    return value;
}
