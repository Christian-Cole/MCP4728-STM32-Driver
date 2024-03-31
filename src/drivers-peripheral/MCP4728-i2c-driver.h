#pragma once
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#ifndef MCP4728_I2C_DRIVER_H_
#define MCP4728_I2C_DRIVER_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

//address definitions (already left-justified for R/W bit)
#define ADDR_A0 0b11000000
#define ADDR_A1 0b11000010
#define ADDR_A2 0b11000100
#define ADDR_A3 0b11000110
#define ADDR_A4 0b11001000
#define ADDR_A5 0b11001010
#define ADDR_A6 0b11001100
#define ADDR_A7 0b11001110

//general call definitions
#define GEN_CALL_ADDR   0x00
#define GEN_CALL_RST    0x06
#define GEN_CALL_WKUP   0x09
#define GEN_CALL_SWUP   0x08
#define GEN_CALL_RDADDR 0x0C

//command definitions (left justified to ease calculations with other bits)
#define FAST_WRITE       0b00000000
#define MULTI_WRITE      0b01000000
#define SEQ_WRITE        0b01010000
#define SINGLE_WRITE     0b01011000
#define ADDR_BITS_WRITE  0b01100000
#define REF_BITS_WRITE   0b10000000
#define GAIN_BITS_WRITE  0b11000000
#define PWRDN_BITS_WRITE 0b10100000

class MCP4728
{
public:
    MCP4728(uint8_t _i2c_address, I2C_HandleTypeDef *_i2c_handle, GPIO_TypeDef *_LDAC_pinPort, uint16_t _LDAC_pin, GPIO_TypeDef *_RDY_pinPort, uint16_t _RDY_pin);

    void singleWrite(uint8_t channelSelect, uint8_t channelUpdate, uint8_t referenceUpdate, uint8_t powerDownUpdate, uint8_t gainUpdate, uint16_t outputValue);
    void singleWrite(uint8_t channelSelect, uint16_t outputValue);
    void singleWriteNoLoad(uint8_t channelSelect, uint16_t outputValue);
    void singleWriteEEPROM(uint8_t channelSelect, uint8_t channelUpdate, uint8_t referenceUpdate, uint8_t powerDownUpdate, uint8_t gainUpdate, uint16_t outputValue);

    void multiwrite(uint8_t numUpdates, uint8_t* channelSelect, uint8_t* channelUpdates, uint8_t* referenceUpdates, uint8_t* powerDownUpdates, uint8_t* gainUpdates, uint16_t* outputValues);
    void multiwrite(uint8_t numUpdates, uint8_t* channelSelect, uint16_t* outputValues);

    void sequentialWrite(uint8_t channelStart, uint8_t* channelUpdates, uint8_t* referenceUpdates, uint8_t* powerDownUpdates, uint8_t* gainUpdates, uint16_t* outputValues);
    void sequentialWrite(uint8_t channelStart, uint16_t* outputValues);
    void sequentialWriteEEPROM(uint8_t channelStart, uint8_t channelUpdate, uint8_t* referenceUpdates, uint8_t* powerDownUpdates, uint8_t* gainUpdates, uint16_t* outputValues);
    
    void fastWrite(uint8_t* powerDownUpdates, uint16_t* outputValues, uint8_t numChannelWrites);
    void fastWrite(uint16_t* outputValues, uint8_t numChannelWrites);
    void fastWrite(uint16_t outputChannel1, uint16_t outputChannel2, uint16_t outputChannel3, uint16_t outputChannel4);

    void writeAddressBits(); //not implemented or tested
    void writeReferenceBits(uint8_t referenceConfig);
    void writeGainBits(uint8_t gainConfig);
    void writePowerDownBits(uint8_t powerDownConfig);
    
    void setLDAC(GPIO_PinState pinState);
    GPIO_PinState getRDYState(); 

    void genCall_reset();
    void genCall_wakeUp();
    void genCall_softwareUpdate();
    void genCall_readAddrBits(); //not implemented or tested

private:
    I2C_HandleTypeDef* i2c_handle;
    uint8_t i2c_address;
    GPIO_TypeDef* LDAC_pinPort;
    uint16_t LDAC_pin;
    GPIO_TypeDef* RDY_pinPort;
    uint16_t RDY_pin;
    
};

#endif