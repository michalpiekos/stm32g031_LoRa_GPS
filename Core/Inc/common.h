/*
 * mp_common.h
 *
 *  Created on: Aug 26, 2023
 *      Author: michal
 */

#ifndef INC_MP_COMMON_H_
#define INC_MP_COMMON_H_

/**********************************************
* Global constants
***********************************************/

#if defined(STM32G030xx) || defined(STM32G031xx)
#include "stm32g0xx_hal.h"
#endif
#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#endif
#ifdef STM32F411xE
#include "stm32f4xx_hal.h"
#endif



#define __MP_DEBUG__				/* Debugging enabled */
//#undef __MP_DEBUG__				/* Debugging disabled */

#ifdef __MP_DEBUG__
#define MP_DEBUG 1
#else
#define MP_DEBUG 0
#endif

/**********************************************
* Macros
***********************************************/

#define debug_print(fmt, ...) \
		do { if (MP_DEBUG) fprintf(stderr, "%ld:%s:%d:%s(): " fmt, HAL_GetTick(), __FILE__, \
														__LINE__, __func__, __VA_ARGS__); } while (0)

/**********************************************
* Function prototypes
***********************************************/


#endif /* INC_MP_COMMON_H_ */
