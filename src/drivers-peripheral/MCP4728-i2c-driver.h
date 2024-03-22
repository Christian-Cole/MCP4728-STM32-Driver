#pragma once
#ifndef LP50XX_I2C_DRIVER_H_
#define LP50XX_I2C_DRIVER_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

//register definitions
#define DEVICE_CONFIG0  0x00
#define DEVICE_CONFIG1  0x01
#define LED_CONFIG0     0x02
#define BANK_BRIGHTNESS 0x03
#define BANK_A_COLOR    0x04
#define BANK_B_COLOR    0x05
#define BANK_C_COLOR    0x06
#define LED0_BRIGHTNESS 0x07
#define LED1_BRIGHTNESS 0x08
#define LED2_BRIGHTNESS 0x09
#define LED3_BRIGHTNESS 0x0A
#define OUT0_COLOR      0x0B
#define OUT1_COLOR      0x0C
#define OUT2_COLOR      0x0D
#define OUT3_COLOR      0x0E
#define OUT4_COLOR      0x0F
#define OUT5_COLOR      0x10
#define OUT6_COLOR      0x11
#define OUT7_COLOR      0x12
#define OUT8_COLOR      0x13
#define OUT9_COLOR      0x14
#define OUT10_COLOR     0x15
#define OUT11_COLOR     0x16
#define RESET           0x17

//important data values
#define CHIP_EN         0x40
#define CONFIG1_RST_VAL 0x3C
#define RESET_VALUE     0xFF

class LP50xx
{
public:
    LP50xx(uint8_t _i2c_address, I2C_HandleTypeDef *_i2c_handle);

    void enable();
    void config(uint8_t configValue);
    void reset();

    void bankConfig(uint8_t bankConfigValue);
    void setBankBrightness(uint8_t bankBrightnessValue);
    void setBankColor(uint8_t bank, uint8_t bankColorValue);

    void setLEDBrightness(uint8_t LED, uint8_t LEDBrightnessValue);
    void setOutColor(uint8_t out, uint8_t outColorValue);
    void setRGBColor(uint8_t LED, uint8_t valueR, uint8_t valueG, uint8_t valueB); //currently only works when auto-increment is enabled (is by default)

    void registerWrite(uint8_t reg, uint8_t value);
    uint8_t registerRead(uint8_t reg);

private:
    I2C_HandleTypeDef* i2c_handle;
    uint8_t i2c_address;
    uint8_t data_to_send;

};

#endif