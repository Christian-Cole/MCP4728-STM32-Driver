#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_rcc.h>
#include <stm32f1xx_hal_rcc_ex.h>

#include "MCP4728-i2c-driver.h"

// NUCLEO-F103 green led - PA5
#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5  
#define LED_PORT_CLK_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE

#define LDAC_PORT GPIOB
#define LDAC_PIN GPIO_PIN_13
#define LDAC_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE

#define RDY_PORT GPIOB
#define RDY_PIN GPIO_PIN_5  
#define RDY_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE

I2C_HandleTypeDef hi2c2;

extern "C" {
// This prevent name mangling for functions used in C/assembly files.
void SysTick_Handler(void)
{ 
    HAL_IncTick();

    // 1 Hz blinking
    if ((HAL_GetTick() % 500) == 0)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
}

}

void initGPIO()
{
    // on board LED
    GPIO_InitTypeDef GPIO_Config;

    GPIO_Config.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Config.Pull = GPIO_NOPULL;
    GPIO_Config.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_Config.Pin = LED_PIN;

    LED_PORT_CLK_ENABLE();
    HAL_GPIO_Init(LED_PORT, &GPIO_Config);

    // LDAC
    GPIO_Config.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Config.Pull = GPIO_NOPULL;
    GPIO_Config.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_Config.Pin = LDAC_PIN;

    LDAC_PORT_CLK_ENABLE();
    HAL_GPIO_Init(LDAC_PORT, &GPIO_Config);

    // RDY
    GPIO_Config.Mode = GPIO_MODE_INPUT;
    GPIO_Config.Pull = GPIO_PULLUP;
    GPIO_Config.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_Config.Pin = RDY_PIN;

    RDY_PORT_CLK_ENABLE();
    HAL_GPIO_Init(RDY_PORT, &GPIO_Config);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C2)
  {
    // __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  }
}

static void initI2C2(void)
{
  
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  __HAL_RCC_I2C1_CLK_ENABLE();

  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    initGPIO();
    initI2C2();
    // 1kHz ticks
    HAL_SYSTICK_Config(SystemCoreClock / 1000);

    MCP4728 adc1(ADDR_A0, &hi2c2, LDAC_PORT, LDAC_PIN, RDY_PORT, RDY_PIN);
    adc1.setLDAC(GPIO_PIN_SET);

    adc1.fastWrite(0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF);
    adc1.setLDAC(GPIO_PIN_RESET);
    HAL_Delay(1);
    adc1.setLDAC(GPIO_PIN_SET);

    uint16_t i = 0;

    uint8_t pdwn[4] {0, 0, 0, 0};
    uint8_t ref[4] {0, 0, 0, 0};
    uint8_t gain[4] {0, 0, 0, 0};
    uint8_t channel[4] {0, 1, 2, 3};
    uint8_t channelupdate[4] {0, 0, 0, 0};
    uint16_t high[4] {0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF};
    uint16_t low[4] {0x0000, 0x0000, 0x0000, 0x0000};

    adc1.writePowerDownBits(0xFF);

    HAL_Delay(50);

    adc1.genCall_wakeUp();

    while (1)
    {
      

      //HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

      //HAL_Delay(1000);

    }
    return 0;
}