/*
 * hal_timer.h
 *
 *  Created on: Jul 17, 2025
 *      Author: ROJ030
 */

#ifndef HAL_HAL_TIMER_H_
#define HAL_HAL_TIMER_H_

#include <stdint.h>

int hal_timer_init();

void hal_timer_start();

void hal_timer_stop();

void hal_timer_restart();

uint32_t hal_timer_get_ms(void);

#endif /* HAL_HAL_TIMER_H_ */
