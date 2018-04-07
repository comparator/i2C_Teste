#ifndef _HW_CONFIG_H
#define _HW_CONFIG_H

// GPIOA
// Pin  Port    Nucleo  Func
//   0  PA0     A0      AIN0               #TIM2_CH1
//   1  PA1     A1      AIN1                TIM2_CH2
//   2  PA2     A7            * USART2_TX
//   3  PA3     A2      AIN3                TIM2_CH4
//   4  PA4     A3      AIN4               #TIM3_CH2
//   5  PA5     A4      AIN5               #TIM2_CH1
//   6  PA6     A5      AIN6               #TIM3_CH1
//   7  PA7     A6      AIN7               #TIM17_CH1
//   8  PA8     D9                          TIM1_CH1
//   9  PA9     D1              USART1_TX   TIM1_CH2
//  10  PA10    D0              USART1_RX   TIM1_CH3
//  11  PA11    D10                         TIM1_CH4
//  12  PA12    D2                          TIM16_CH1
//  13  PA13          * SWDIO
//  14  PA14          * SWCLK
//  15  PA15                  * USART2_RX
// GPIOB
//  16  PB0     D3      AIN8                TIM3_CH3
//  17  PB1     D6      AIN9               #TIM3_CH4
//  19  PB3     D13                        #TIM3_CH1
//  20  PB4     D12                        #TIM3_CH2
//  21  PB5     D11                        #TIM17_CH1
//  22  PB6     D5      SCL1
//  23  PB7     D4      SDA1               #TIM3_CH4
// PF0 - D7 / PF1 - D8: Not used

// Solder Bridges
// SB6: Off  - PF0 Disconnected from D7
// SB4: On   - MCO From ST-LINK Connect to OSC-IN - PF0
// SB16: Off - PB6 Disconnected from A5 - PA6
// SB18: Off - PB7 Disconnected from A4 - PA5

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif // _S3Sn10_H
