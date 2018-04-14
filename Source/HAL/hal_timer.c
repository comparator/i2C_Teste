#include "hal.h"
#include "hal_timer.h"


void hal_timer_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
}

// timeout in 1 mS
void hal_timer_start(uint16_t timeout)
{
    // Stop Timer
    TIM7->CR1   = 0;
    timeout *= 2;
    timeout++;

    // Initialise timer TIM7
    TIM7->CR1   = TIM_CR1_OPM | TIM_CR1_URS;
    TIM7->CNT   = 0;
    TIM7->ARR   = timeout;
    TIM7->PSC   = (36000 - 1);  // Ftim = 72M/36K = 2KHz
    TIM7->EGR   = TIM_EGR_UG;   // Update registers
    TIM7->SR    = 0;            // Reset Flags
    TIM7->CR1  |= TIM_CR1_CEN;  // Start Timer
}

bool hal_timer_timeout(void)
{
    return ((TIM7->SR & TIM_SR_UIF) != 0);
}