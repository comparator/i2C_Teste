#include "hal.h"
#include "hal_gpio.h"
#include "hal_twi.h"


// PB6/PB7 - SCL/SDA, bus1

#define I2C_GPIO        GPIOB
#define I2C_PIN_SCL     (1<<6)
#define I2C_PIN_SDA     (1<<7)
#define DIO_MODE_TWI    ((4<<DIO_AF_OFFS) | DIO_MODE_AF_OD)

#define I2C_TIMING      0x2000090E
#define I2C_BUS         I2C1
#define I2C_EV_IRQn     I2C1_EV_IRQn
#define I2C_ER_IRQn     I2C1_ER_IRQn

/*
// Global variable defined in exttwi.c
static volatile uint8_t twi_pnt = 0;

static uint8_t twi_access = 0;
static uint8_t twi_address = 0;
static uint8_t twi_write = 0;
static uint8_t twi_read = 0;
static uint8_t twi_data[32];
*/

bool hal_twi_check_pin(void)
{
    hal_gpio_cfg(I2C_GPIO, (I2C_PIN_SCL | I2C_PIN_SDA), DIO_MODE_IN_PU);

    uint16_t timeout = 0x8000;
    while(timeout != 0)
    {
        if((I2C_GPIO->IDR & (I2C_PIN_SCL | I2C_PIN_SDA)) != (I2C_PIN_SCL | I2C_PIN_SDA))
        {
            timeout++;
            if(timeout == 0)
            {
                // Release GPIO
                hal_gpio_cfg(I2C_GPIO, (I2C_PIN_SCL | I2C_PIN_SDA), DIO_MODE_IN_FLOAT);
                return false;
            }
        }
        else
        {
            if(timeout > 0x8000)
            {
                timeout = 0x8000;
            }
            else
            {
                timeout--;
            }
        }
    }
    return true;
}


/*
uint8_t hal_twi_status(void)
{
    return twi_access;
}



bool hal_twi_enable(void)
{
    if(enable)
    {
        // Check GPIO

        // Configure GPIO as inputs with pull down


        // Configure GPIO
        hal_gpio_cfg(I2C_GPIO, (I2C_PIN_SCL | I2C_PIN_SDA), DIO_MODE_TWI);

        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;     // Enable I2C clock
        // Reset I2C1
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

        // Set Timings
        // I2C Clock = 8MHz
        // I2C Bus = 100 KHz
        I2C_BUS->TIMINGR = I2C_TIMING;

        // PEC Disable
        // SMBus Disable
        // General Call Disable
        // Wakeup Disable
        // Clock stretching in slave mode enabled
        // DMA disabled
        // Analog Filter On
        // Digital Filter Off

        // Enable I2C
        I2C_BUS->CR1 = I2C_CR1_PE;

        NVIC_SetPriority(I2C_EV_IRQn, 0);
        NVIC_EnableIRQ(I2C_EV_IRQn);
        NVIC_SetPriority(I2C_ER_IRQn, 0);
        NVIC_EnableIRQ(I2C_ER_IRQn);
    }
    else
    {
        NVIC_DisableIRQ(I2C_EV_IRQn);
        NVIC_DisableIRQ(I2C_ER_IRQn);

        // Reset I2C1
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
        // Disable clock
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;

        // Release GPIO
        hal_gpio_cfg(I2C_GPIO, (I2C_PIN_SCL | I2C_PIN_SDA), DIO_MODE_IN_FLOAT);
    }

    return true;
}

void hal_twi_stop(void)
{
    I2C_BUS->CR1 &= ~I2C_CR1_PE;                                // Disable I2C
}

void hal_twi_start(void)
{
    uint32_t isr = I2C_BUS->ISR;
    if(isr & (I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR))       // Bus Error
    {
        I2C_BUS->CR1 &= ~I2C_CR1_PE;                            // Disable I2C
        twi_access |= TWI_FL_ERROR;
        return;
    }

    if(isr & I2C_ISR_BUSY)                                      // Bus Busy
    {
        return;
    }

    I2C_BUS->CR1 &= ~I2C_CR1_PE;                                // Reset I2C

    twi_pnt = 0;
    twi_access |= TWI_FL_BUSY;

    I2C_BUS->CR1 =  ( I2C_CR1_PE |                              // Enable I2C
                                                                // Enable Interrupts on:
                      I2C_CR1_ERRIE |                           //  Errors
                      I2C_CR1_NACKIE);                          //  NACK received

    I2C_BUS->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;             // Clear STOP & NACK flags.

    if(twi_access & TWI_FL_WRITE)
    {
        I2C_BUS->CR2 = (uint32_t)(twi_address << 1) |   // Slave address
                      ((uint32_t)(twi_write) << 16);    // Bytes to send

        if(twi_write > 0)
        {
            I2C_BUS->CR1 |= I2C_CR1_TXIE;                       // Interrupt on Tx Buffer empty
        }

        if((twi_access & TWI_FL_READ) == 0)
        {
            I2C_BUS->CR2 |= I2C_CR2_AUTOEND;
            I2C_BUS->CR1 |= I2C_CR1_STOPIE;
        }
        else
        {
            I2C_BUS->CR1 |= I2C_CR1_TCIE;                       // Enable TC IRQ
        }
    }
    else    // Only Read Access
    {
        uint32_t read = twi_read;
        if(read == 0)
        {
            read = 1;
        }

        I2C_BUS->CR2 = ((uint32_t)(twi_address << 1) |  // Slave address
                                                (read << 16) |  // Bytes to read
                    I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);          // Read request with stop

        I2C_BUS->CR1 |= I2C_CR1_RXIE;
    }

    I2C_BUS->CR2 |= I2C_CR2_START;                              // Send Start & Address
}

void I2C1_ER_IRQHandler();
void I2C1_ER_IRQHandler()
{
    if(I2C_BUS->ISR & (I2C_ISR_BERR |                               // Bus error
                       I2C_ISR_ARLO |                               // Arbitration lost
                       I2C_ISR_OVR))                                // Over-Run/Under-Run
    {
        I2C_BUS->CR1 &= ~I2C_CR1_PE;                                // Disable I2C
        twi_access |= TWI_FL_ERROR;
    }
    // else WTF ?
}

void I2C1_EV_IRQHandler(void);
void I2C1_EV_IRQHandler()
{
    uint32_t isr = I2C_BUS->ISR;

    if(isr & I2C_ISR_NACKF)                                     // NACK received
    {
        I2C_BUS->ICR = I2C_ICR_NACKCF;                          // Clear NACK Flag

        if(twi_pnt == 0)
        {
            I2C_BUS->CR1 = I2C_CR1_PE;                          // Disable Interrupts
            twi_access |= TWI_FL_SLANACK;
        }
        else if(twi_access & TWI_FL_WRITE)
        {
            twi_write = twi_pnt;
            
            if(twi_access & TWI_FL_READ)
            {
                uint32_t read = twi_read;
                if(read == 0)
                {
                    read = 1;
                }

                I2C_BUS->CR2 = ((uint32_t)(twi_address << 1) |  // Slave address
                                                        (read << 16) |  // Bytes to read
                            I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);          // Read request with stop
                I2C_BUS->CR2 |= I2C_CR2_START;                      // Send Repeat Start & Address

                I2C_BUS->CR1 |= I2C_CR1_RXIE;
                I2C_BUS->CR1 &= ~(                                      // Disable Interrupts
                                  I2C_CR1_TCIE |                        // Transfer complete
                                  I2C_CR1_TXIE);                        // Tx
                twi_pnt = 0;
            }
            else
            {
                I2C_BUS->CR1 = I2C_CR1_PE;                      // Disable Interrupts
                twi_access = TWI_FL_RDY;                   // Transaction complete
            }
        }
        // else WTF ?
    }
    // Read Data
    else if((I2C_BUS->CR1 & I2C_CR1_RXIE) &&                    // Data received
                     (isr & I2C_ISR_RXNE))
    {
        twi_data[twi_pnt++] = I2C_BUS->RXDR;

        if(twi_pnt >= twi_read)
        {
            I2C_BUS->CR1 = I2C_CR1_PE;                          // Disable Interrupts
            twi_access = TWI_FL_RDY;                       // Transaction complete
        }
    }
    // Write Data
    else if((I2C_BUS->CR1 & I2C_CR1_TXIE) &&
                     (isr & I2C_ISR_TXIS))                      // Transmit buffer empty
    {
        I2C_BUS->TXDR = twi_data[twi_pnt++];
    }
    // Write Last Byte and Start Read
    else if(isr & I2C_ISR_TC)                                   // Transfer complete
    {
        uint32_t read = twi_read;
        if(read == 0)
        {
            read = 1;
        }

        I2C_BUS->CR2 = ((uint32_t)(twi_address << 1) |  // Slave address
                                                (read << 16) |  // Bytes to read
                    I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);          // Read request with stop
        I2C_BUS->CR2 |= I2C_CR2_START;                          // Send Repeat Start & Address

        I2C_BUS->CR1 |= I2C_CR1_RXIE;
        I2C_BUS->CR1 &= ~(                                      // Disable Interrupts
                          I2C_CR1_TCIE |                        // Transfer complete
                          I2C_CR1_TXIE);                        // Tx
        twi_pnt = 0;
    }
    // Write Only Access
    else if(isr & I2C_ISR_STOPF)                                // Stop received
    {
        I2C_BUS->ICR = I2C_ICR_STOPCF;                          // Clear Stop Flag
        I2C_BUS->CR1 = I2C_CR1_PE;                              // Disable Interrupts
        twi_access = TWI_FL_RDY;                           // Transaction complete
    }
    // else WTF ?
}
*/
