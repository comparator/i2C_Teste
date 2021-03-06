#include "hal.h"
#include "hal_gpio.h"

void hal_gpio_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;     // GPIO
}

// HAL GPIO Functions
void hal_gpio_cfg(GPIO_TypeDef * GPIOx, uint16_t Mask, uint16_t Mode)
{
    uint16_t pinpos;
    uint32_t pos;
    uint32_t pupd = Mode & 0x03;            // Float / Pull-Up / Pull-Down
    uint32_t ppod = (Mode & 0x04) >> 2;     // Push-Pull / Open Drain
    uint32_t mod  = (Mode & 0x18) >> 3;     // Input / Output / AF / Analog
    uint32_t spd  = (Mode & 0x60) >> 5;     // Low / Medim / Fast / High Speed
    uint32_t afr  = (Mode & 0x0F00) >> 8;   // Alternative function

    for(pinpos = 0; pinpos < 0x10; pinpos++)
    {
        pos = ((uint32_t)0x01) << pinpos;
        if(Mask & pos)
        {
            pos = pinpos;
            pos <<= 1;

            // default state input
            GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << pos);
            GPIOx->MODER |= mod << pos;

            if((mod == 1) || (mod == 2))
            {
                // Low Speed
                GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << pos);
                GPIOx->OSPEEDR |= spd << pos;

                // Output type - push/pull
                GPIOx->OTYPER &= ~((GPIO_OTYPER_OT_0) << pinpos);
                GPIOx->OTYPER |= ppod << pinpos;
            }
            // without PullUp/Down
            GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << pos);
            GPIOx->PUPDR |= pupd << pos;

            if(mod == 2)
            {
                // Set Alternative function
                if(pinpos < 8)              // AFR0
                {
                    pos = pinpos;
                    pos <<= 2;
                    GPIOx->AFR[0] &= ~((uint32_t)0x0000000F << pos);
                    GPIOx->AFR[0] |= ((uint32_t)afr << pos);
                }
                else                        // AFR1
                {
                    pos = pinpos;
                    pos -= 8;
                    pos <<= 2;
                    GPIOx->AFR[1] &= ~((uint32_t)0x0000000F << pos);
                    GPIOx->AFR[1] |= ((uint32_t)afr << pos);
                }
            }
        }
    }
}
// End HAL GPIO Functions
