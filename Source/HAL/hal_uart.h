#ifndef __HAL_UART_H
#define __HAL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

void hal_uart_init_hw(uint32_t baud);
bool hal_uart_datardy(void);
uint8_t hal_uart_get(void);
bool hal_uart_free(void);
void hal_uart_send(uint8_t len, uint8_t * pBuf);

#ifdef __cplusplus
}
#endif

#endif  //  __HAL_H
