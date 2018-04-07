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


bool hal_twi_check_pin(void);

#ifdef __cplusplus
}
#endif

#endif // _HAL_TWI_H
