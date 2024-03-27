#include "MCP4728-i2c-driver.h"

#include "stm32f103xb.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_i2c.h"
#include <stdint.h>


MCP4728::MCP4728(uint8_t _i2c_address, I2C_HandleTypeDef *_i2c_handle, GPIO_TypeDef *_LDAC_pinPort, uint16_t _LDAC_pin, GPIO_TypeDef *_RDY_pinPort, uint16_t _RDY_pin)
{
    i2c_handle = _i2c_handle;
    i2c_address = _i2c_address;
    LDAC_pinPort = _LDAC_pinPort;
    LDAC_pin = _LDAC_pin;
    RDY_pinPort = _RDY_pinPort;
    RDY_pin = _RDY_pin;
}



void MCP4728::singleWrite(uint8_t channelSelect, uint8_t channelUpdate, uint8_t referenceUpdate, uint8_t powerDownUpdate, uint8_t gainUpdate, uint16_t outputValue)
{
    uint8_t data_to_send[3];
    data_to_send[0] = MULTI_WRITE | ((channelSelect << 1) & 0x06) | (channelUpdate & 0x01);
    data_to_send[1] = ((referenceUpdate << 7) & 0x80) | ((powerDownUpdate << 5) & 0x60) | ((gainUpdate << 4) & 0x10) | ((outputValue >> 8) & 0x0F);
    data_to_send[2] = outputValue & 0xFF;  
    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, 3, HAL_MAX_DELAY);
}

void MCP4728::singleWrite(uint8_t channelSelect, uint16_t outputValue)
{
    uint8_t data_to_send[3];
    data_to_send[0] = MULTI_WRITE | ((channelSelect << 1) & 0x06);
    data_to_send[1] = (outputValue >> 8) & 0x0F;
    data_to_send[2] = outputValue & 0xFF;  
    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, 3, HAL_MAX_DELAY);
}

void MCP4728::singleWriteNoLoad(uint8_t channelSelect, uint16_t outputValue)
{
    uint8_t data_to_send[3];
    data_to_send[0] = MULTI_WRITE | ((channelSelect << 1) & 0x06) | 0x01;
    data_to_send[1] = (outputValue >> 8) & 0x0F;
    data_to_send[2] = outputValue & 0xFF;  
    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, 3, HAL_MAX_DELAY);
}

void MCP4728::singleWriteEEPROM(uint8_t channelSelect, uint8_t channelUpdate, uint8_t referenceUpdate, uint8_t powerDownUpdate, uint8_t gainUpdate, uint16_t outputValue)
{
    uint8_t data_to_send[3];
    data_to_send[0] = SINGLE_WRITE | ((channelSelect & 0x03) << 1) | (channelUpdate & 0x01); 
    data_to_send[1] = ((referenceUpdate << 7) & 0x80) | ((powerDownUpdate << 5) & 0x60) | ((gainUpdate << 4) & 0x10) | ((outputValue >> 8) & 0x0F);
    data_to_send[2] = outputValue & 0xFF; 
    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, 3, HAL_MAX_DELAY);
}



void MCP4728::multiwrite(uint8_t numUpdates, uint8_t* channelSelect, uint8_t* channelUpdates, uint8_t* referenceUpdates, uint8_t* powerDownUpdates, uint8_t* gainUpdates, uint16_t* outputValues)
{
    if (numUpdates > 4)
    {
        return;
    }

    uint8_t data_to_send[numUpdates * 3];
    for (int i = 0; i < numUpdates; i++)
    {
        data_to_send[(i * 3) + 0] = MULTI_WRITE | ((channelSelect[i] << 1) & 0x06) | (channelUpdates[i] & 0x01);
        data_to_send[(i * 3) + 1] = ((referenceUpdates[i] << 7) & 0x80) | ((powerDownUpdates[i] << 5) & 0x60) | ((gainUpdates[i] << 4) & 0x10) | ((outputValues[i] >> 8) & 0x0F);
        data_to_send[(i * 3) + 2] = outputValues[i] & 0xFF; 
    }

    uint8_t bytesTransfered = numUpdates * 3;

    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, bytesTransfered, HAL_MAX_DELAY);
}

void MCP4728::multiwrite(uint8_t numUpdates, uint8_t* channelSelect, uint16_t* outputValues)
{
    if (numUpdates > 4)
    {
        return;
    }

    uint8_t data_to_send[numUpdates * 3];
    for (int i = 0; i < numUpdates; i++)
    {
        data_to_send[(i * 3) + 0] = MULTI_WRITE | ((channelSelect[i] << 1) & 0x06);
        data_to_send[(i * 3) + 1] = (outputValues[i] >> 8) & 0x0F;
        data_to_send[(i * 3) + 2] = outputValues[i] & 0xFF; 
    }

    uint8_t bytesTransfered = numUpdates * 3;

    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, bytesTransfered, HAL_MAX_DELAY);
}



void MCP4728::sequentialWrite(uint8_t channelStart, uint8_t* channelUpdates, uint8_t* referenceUpdates, uint8_t* powerDownUpdates, uint8_t* gainUpdates, uint16_t* outputValues)
{
    if (channelStart > 3)
    {
        return;
    }
    uint8_t channel = channelStart;

    uint8_t data_to_send[(4 - channelStart) * 3];
    for (int i = 0; i < (4 - channelStart); i++)
    {
        data_to_send[(i * 3) + 0] = MULTI_WRITE | ((channel << 1) & 0x06) | (channelUpdates[i] & 0x01);
        data_to_send[(i * 3) + 1] = ((referenceUpdates[i] << 7) & 0x80) | ((powerDownUpdates[i] << 5) & 0x60) | ((gainUpdates[i] << 4) & 0x10) | ((outputValues[i] >> 8) & 0x0F);
        data_to_send[(i * 3) + 2] = outputValues[i] & 0xFF;

        channel++;
    }

    uint8_t bytesTransfered = (4 - channelStart) * 3;

    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, bytesTransfered, HAL_MAX_DELAY);
}

void MCP4728::sequentialWrite(uint8_t channelStart, uint16_t* outputValues)
{
    if (channelStart > 3)
    {
        return;
    }
    uint8_t channel = channelStart;

    uint8_t data_to_send[(4 - channelStart) * 3];
    for (int i = 0; i < (4 - channelStart); i++)
    {
        data_to_send[(i * 3) + 0] = MULTI_WRITE | ((channel << 1) & 0x06);
        data_to_send[(i * 3) + 1] = (outputValues[i] >> 8) & 0x0F;
        data_to_send[(i * 3) + 2] = outputValues[i] & 0xFF;

        channel++;
    }

    uint8_t bytesTransfered = (4 - channelStart) * 3;

    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, bytesTransfered, HAL_MAX_DELAY);
}

void MCP4728::sequentialWriteEEPROM(uint8_t channelStart, uint8_t channelUpdate, uint8_t* referenceUpdates, uint8_t* powerDownUpdates, uint8_t* gainUpdates, uint16_t* outputValues)
{
    if (channelStart > 3)
    {
        return;
    }
    
    uint8_t data_to_send[(2 * (4 - channelStart)) + 1];
    data_to_send[0] = SEQ_WRITE | ((channelStart & 0x03) << 1) | (channelUpdate & 0x01); 
    for (int i = 0; i < channelStart; i++)
    {
        data_to_send[(i * 2) + 1] = ((referenceUpdates[i] << 7) & 0x80) | ((powerDownUpdates[i] << 5) & 0x60) | ((gainUpdates[i] << 4) & 0x10) | ((outputValues[i] >> 8) & 0x0F);
        data_to_send[(i * 2) + 2] = outputValues[i] & 0xFF; 
    }

    uint8_t bytesTransfered = (2 * (4 - channelStart)) + 1;

    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, bytesTransfered, HAL_MAX_DELAY);
}



void MCP4728::fastWrite(uint8_t* powerDownUpdates, uint16_t* outputValues, uint8_t numChannelWrites)
{
    if (numChannelWrites > 4)
    {
        numChannelWrites = 4;
    }

    uint8_t data_to_send[(numChannelWrites * 2)];
    for (int i = 0; i < numChannelWrites; i++)
    {
        data_to_send[(i * 2)] = FAST_WRITE | ((powerDownUpdates[i] << 4) & 0x30) | ((outputValues[i] >> 8) & 0x0F);
        data_to_send[(i * 2) + 1] = outputValues[i] & 0xFF;
    }

    uint8_t bytesTransfered = 2 * numChannelWrites;

    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, bytesTransfered, HAL_MAX_DELAY);
}

void MCP4728::fastWrite(uint16_t* outputValues, uint8_t numChannelWrites)
{
    if (numChannelWrites > 4)
    {
        numChannelWrites = 4;
    }

    uint8_t data_to_send[(numChannelWrites * 2)];
    for (int i = 0; i < numChannelWrites; i++)
    {
        data_to_send[(i * 2)] = FAST_WRITE | ((outputValues[i] >> 8) & 0x0F);
        data_to_send[(i * 2) + 1] = outputValues[i] & 0xFF;
    }

    uint8_t bytesTransfered = 2 * numChannelWrites;

    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, bytesTransfered, HAL_MAX_DELAY);
}

void MCP4728::fastWrite(uint16_t outputChannel1, uint16_t outputChannel2, uint16_t outputChannel3, uint16_t outputChannel4)
{
    uint8_t data_to_send[8];
    
    data_to_send[0] = FAST_WRITE | ((outputChannel1 >> 8) & 0x0F);
    data_to_send[1] = outputChannel1 & 0xFF;
    data_to_send[2] = FAST_WRITE | ((outputChannel2 >> 8) & 0x0F);
    data_to_send[3] = outputChannel2 & 0xFF;
    data_to_send[4] = FAST_WRITE | ((outputChannel3 >> 8) & 0x0F);
    data_to_send[5] = outputChannel3 & 0xFF;
    data_to_send[6] = FAST_WRITE | ((outputChannel4 >> 8) & 0x0F);
    data_to_send[7] = outputChannel4 & 0xFF;

    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, 8, HAL_MAX_DELAY);
}



void MCP4728::writeAddressBits() //more complicated, required LDAC change in the middle of sending, DO NOT USE RN
{
    uint8_t data_to_send[3];
    data_to_send[0] = ADDR_BITS_WRITE; //not done
    data_to_send[1] = ADDR_BITS_WRITE; //not done
    data_to_send[2] = ADDR_BITS_WRITE; //not done
    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, 3, HAL_MAX_DELAY);
}

void MCP4728::writeReferenceBits(uint8_t referenceConfig)
{
    uint8_t data_to_send = REF_BITS_WRITE | (referenceConfig & 0x0F); //masks bits 0-3 for referenceConfig to prevent it from affecting the command bits
    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, &data_to_send, 1, HAL_MAX_DELAY);
}

void MCP4728::writeGainBits(uint8_t gainConfig)
{
    uint8_t data_to_send = GAIN_BITS_WRITE | (gainConfig & 0x0F); //masks bits 0-3 for gainConfig to prevent it from affecting the command bits
    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, &data_to_send, 1, HAL_MAX_DELAY);
}

void MCP4728::writePowerDownBits(uint8_t powerDownConfig)
{
    uint8_t data_to_send[2];
    data_to_send[0] = PWRDN_BITS_WRITE | ((powerDownConfig >> 4) & 0x0F); //first byte is the command bits and the first 4 bits of powerConfig data
    data_to_send[1] = (powerDownConfig << 4) & 0xF0; //second byte is the last 4 bits of powerConfig data shifted to the left
    HAL_I2C_Master_Transmit(i2c_handle, i2c_address, data_to_send, 2, HAL_MAX_DELAY);
}



void MCP4728::setLDAC(GPIO_PinState pinState)
{
    HAL_GPIO_WritePin(LDAC_pinPort, LDAC_pin, pinState);
}

GPIO_PinState MCP4728::getRDYState()
{
    return HAL_GPIO_ReadPin(RDY_pinPort, RDY_pin);
}



void MCP4728::genCall_reset()
{
    uint8_t data_to_send = GEN_CALL_RST;
    HAL_I2C_Master_Transmit(i2c_handle, GEN_CALL_ADDR, &data_to_send, 1, HAL_MAX_DELAY);
}

void MCP4728::genCall_wakeUp()
{
    uint8_t data_to_send = GEN_CALL_WKUP;
    HAL_I2C_Master_Transmit(i2c_handle, GEN_CALL_ADDR, &data_to_send, 1, HAL_MAX_DELAY);
}

void MCP4728::genCall_softwareUpdate()
{
    uint8_t data_to_send = GEN_CALL_WKUP;
    HAL_I2C_Master_Transmit(i2c_handle, GEN_CALL_ADDR, &data_to_send, 1, HAL_MAX_DELAY);
}

void MCP4728::genCall_readAddrBits() //USES THE LDAC PIN TO INDICATE DEV ON INTEREST, THIS DOESNT WORK RN
{
    uint8_t data_to_send = GEN_CALL_RDADDR;
    HAL_I2C_Master_Transmit(i2c_handle, GEN_CALL_ADDR, &data_to_send, 1, HAL_MAX_DELAY);
}