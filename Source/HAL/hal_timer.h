#ifndef __HAL_TIMER_H
#define __HAL_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

void hal_timer_init(void);
void hal_timer_start(uint16_t timeout);
bool hal_timer_timeout(void);

#ifdef __cplusplus
}
#endif

#endif  //  __HAL_TIMER_H

