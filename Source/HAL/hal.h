#ifndef __HAL_H
#define __HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx.h"
#include <stdbool.h>

//////////////////////////////////////////////////////////////
// System subroutine
void halEnterCritical(void);
void halLeaveCritical(void);

void hal_system_init(void);

#ifdef __cplusplus
}
#endif

#endif  //  __HAL_H
