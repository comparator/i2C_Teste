#ifndef __HAL_GPIO_H
#define __HAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////////
// DIO/GPIO Section

// DIO Modes
// 11-8 bits:   AF number
//  6-5 bits:   Low / Medium / Fast / High Speed
//  4-3 bits:   Input / Output / AF / Analog
//  2 bit:      Push-Pull / Open Drain
//  0-1 bits:   Float / PullUp / PullDown

#define DIO_AF_OFFS                 8

#define DIO_MODE_IN_FLOAT           0x00
#define DIO_MODE_IN_PU              0x01
#define DIO_MODE_IN_PD              0x02
#define DIO_MODE_OUT_PP             0x08
//#define DIO_MODE_OUT_OD             0x0C
#define DIO_MODE_OUT_PP_HS          0x68    // Output, push-pull, high speed
#define DIO_MODE_AF_PP              0x10
//#define DIO_MODE_AF_PU              0x11
//#define DIO_MODE_AF_PD              0x12
#define DIO_MODE_AF_OD              0x14
#define DIO_MODE_AF_PP_HS           0x70    // Alternative function, Push/pull, high speed
#define DIO_MODE_AIN                0x18

void        hal_gpio_init(void);
void        hal_gpio_cfg(GPIO_TypeDef * GPIOx, uint16_t Mask, uint16_t Mode);
__STATIC_INLINE void hal_gpio_set(GPIO_TypeDef * GPIOx, uint16_t Mask){GPIOx->BSRR = Mask;}
__STATIC_INLINE void hal_gpio_rst(GPIO_TypeDef * GPIOx, uint16_t Mask){GPIOx->BRR = Mask;}
__STATIC_INLINE bool hal_gpio_get(GPIO_TypeDef * GPIOx, uint16_t Mask){return ((GPIOx->IDR & Mask) != 0);}

#ifdef __cplusplus
}
#endif

#endif  //  __HAL_GPIO_H
