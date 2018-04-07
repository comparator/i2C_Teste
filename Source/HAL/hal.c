#include "hal.h"

#define HSE_STARTUP_TIME            0x00005000UL

static uint32_t CriticalNesting = 0;

void halEnterCritical(void)
{
    __disable_irq();

    CriticalNesting++;

    __DSB();    //  Data Synchronization Barrier
    __ISB();    //  Instruction Synchronization Barrier
}

void halLeaveCritical(void)
{
    if(CriticalNesting == 0)
    {
        while(1);               // Error
    }

    CriticalNesting--;
    if(CriticalNesting == 0)
    {
        __enable_irq();
    }
}

void hal_system_init(void)
{
    // System Init
    /* FPU settings ------------------------------------------------------------*/
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
    #endif

    /* Reset the RCC clock configuration to the default reset state ------------*/
    /* Set HSION bit */
    RCC->CR |= 0x00000001U;

    /* Reset CFGR register */
    RCC->CFGR &= 0xF87FC00CU;

    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= 0xFEF6FFFFU;

    /* Reset HSEBYP bit */
    RCC->CR &= 0xFFFBFFFFU;

    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
    RCC->CFGR &= 0xFF80FFFFU;

    /* Reset PREDIV1[3:0] bits */
    RCC->CFGR2 &= 0xFFFFFFF0U;

    /* Reset USARTSW[1:0], I2CSW and TIMs bits */
    RCC->CFGR3 &= 0xFF00FCCCU;

    /* Disable all interrupts */
    RCC->CIR = 0x00000000U;
    
    SCB->VTOR = FLASH_BASE;     /* Vector Table Relocation in Internal FLASH */


    uint32_t StartUpCounter = 0;

    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV2;

    // Wait till HSE is ready and if Time out is reached exit
    while(((RCC->CR & RCC_CR_HSERDY) == 0) && (StartUpCounter < HSE_STARTUP_TIME))
    {
        StartUpCounter++;
    }

    // Enable Prefetch Buffer
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    // Set Flash Latency, Flash 2 wait state
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_1;

    // PLL->CFGR reset in SystemInit()
    // SYSCLK = 72, on PLL
    // AHB Prescaler = 1, HCLK = SYSCLK
    // APB2 Prescaler = 1, PCLK2 = HCLK
    // APB1 Prescaler = 2, PCLK1 = HCLK/2
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    if(StartUpCounter < HSE_STARTUP_TIME)
    {
        // PLL configuration: PLLCLK = HSE * 9 = 72 MHz
        RCC->CFGR |= (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL9);
    }
    else
    {
        while(1);
    }

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;

    // Wait till PLL is ready
    while((RCC->CR & RCC_CR_PLLRDY) == 0);

    // Select PLL as system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait till PLL is used as system clock source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    
    CriticalNesting = 0;
}
