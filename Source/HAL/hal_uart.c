#include "hal.h"
#include "hal_gpio.h"
#include "hal_uart.h"

// USART 2, PA2/PA15

#define HAL_SIZEOF_UART_RX_FIFO     256

#define hal_pclk1                   18000000UL
#define HAL_USART2_AF               ((7<<DIO_AF_OFFS) | DIO_MODE_AF_PP)
#define USART2_TX_DMA               DMA1_Channel7
#define USART2_RX_DMA               DMA1_Channel6

static uint8_t  Rx2_FIFO[HAL_SIZEOF_UART_RX_FIFO];
static uint16_t Rx2_Tail = 0;

void hal_uart_init_hw(uint32_t baud)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;           // Enable UART Clock
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;               // Enable the peripheral clock DMA1

    USART2->BRR = ((hal_pclk1 + (baud/2))/baud);

    USART2->CR1 = 0;                                // Disable USART
    USART2->CR2 = 0;                                // 8N1
    USART2->CR3 = 0;                                // Without flow control

    // Configure DIO
    hal_gpio_cfg(GPIOA, ((1<<2) | (1<<15)), HAL_USART2_AF);

    // DMA1 USART2_RX config
    USART2_RX_DMA->CPAR = (uint32_t)&(USART2->RDR); // Peripheral address
    USART2_RX_DMA->CMAR = (uint32_t)Rx2_FIFO;       // Memory address
    USART2_RX_DMA->CNDTR = HAL_SIZEOF_UART_RX_FIFO; // Data size
    USART2_RX_DMA->CCR =                            // Priority - Low
                                                    // Memory Size - 8 bit
                                                    // Peripheral size - 8 bit
                        DMA_CCR_MINC |              // Memory increment
                                                    // Peripheral increment disabled
                        DMA_CCR_CIRC |              // Circular mode
                        DMA_CCR_EN;                 // DMA Channel Enable

    // Configure UART
    USART2->CR1 |= USART_CR1_RE;                    // Enable RX
    USART2->CR3 |= USART_CR3_DMAR;                  // DMA enable receiver
    // Set Variables
    Rx2_Tail = 0;

    USART2->CR1 |= USART_CR1_TE;                    // Enable TX
    USART2->CR3 |= USART_CR3_DMAT;                  // DMA enable transmitter

    USART2->CR1 |= USART_CR1_UE;                        // Enable USART
}

bool hal_uart_datardy(void)
{
    uint32_t tmp = HAL_SIZEOF_UART_RX_FIFO;
    tmp -= Rx2_Tail;
    return (tmp != USART2_RX_DMA->CNDTR);
}

uint8_t hal_uart_get(void)
{
    uint8_t retval;
    retval = Rx2_FIFO[Rx2_Tail++];
    if(Rx2_Tail >= HAL_SIZEOF_UART_RX_FIFO)
    {
        Rx2_Tail = 0;
    }
    return retval;
}

/*
static bool hal_uart_free(void)
{
    if(USART2_TX_DMA->CCR & DMA_CCR_EN)
    {
        if(USART2_TX_DMA->CNDTR == 0)
        {
            USART2_TX_DMA->CCR &= ~DMA_CCR_EN;
        }
        else
        {
            return false;
        }
    }
    return true;
}
*/
void hal_uart_send(uint8_t len, uint8_t * pBuf)
{
    // DMA1 USART2_TX config
    USART2_TX_DMA->CPAR = (uint32_t)&(USART2->TDR);     // Peripheral address
    USART2_TX_DMA->CMAR = (uint32_t)pBuf;               // Memory address
    USART2_TX_DMA->CNDTR = len;
    USART2_TX_DMA->CCR =                                // Priority - Low
                                                        // Memory Size - 8 bit
                                                        // Peripheral size - 8 bit
                            DMA_CCR_MINC |              // Memory increment
                                                        // Peripheral increment disabled
                            DMA_CCR_DIR |               // Read from memory
                            DMA_CCR_EN;                 // DMA Channel Enable

    while((USART2_TX_DMA->CCR & DMA_CCR_EN) != 0)
    {
        if(USART2_TX_DMA->CNDTR == 0)
        {
            USART2_TX_DMA->CCR &= ~DMA_CCR_EN;
        }
    }
}
