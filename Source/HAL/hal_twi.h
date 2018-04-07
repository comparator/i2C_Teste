#ifndef _HAL_TWI_H
#define _HAL_TWI_H

#ifdef __cplusplus
extern "C" {
#endif

// TWI Access Flags
#define TWI_FL_WRITE    0x01    // Write access
#define TWI_FL_READ     0x02    // Read access
#define TWI_FL_BUSY     0x08    // Bus busy
#define TWI_FL_RDY      0x10    // Access complete
#define TWI_FL_WD       0x20    // Timeout
#define TWI_FL_SLANACK  0x40    // Slave Addr NACK received
#define TWI_FL_ERROR    0x80    // Unknown error


bool    hal_twi_check_pin(void);
void    hal_twi_enable(void);
void    hal_twi_disable(void);

uint8_t hal_twi_status(void);
uint8_t hal_twi_start(uint8_t addr, uint8_t toWr, uint8_t toRd, uint8_t *pBuf);
uint8_t hal_twi_get_data(uint8_t * pData);

// ISR Section
void I2C1_ER_IRQHandler(void);
void I2C1_EV_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif // _HAL_TWI_H
