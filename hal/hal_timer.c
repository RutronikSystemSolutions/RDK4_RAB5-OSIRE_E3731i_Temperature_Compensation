/*
 * hal_timer.c
 *
 *  Created on: Jul 17, 2025
 *      Author: ROJ030
 */

#include "hal_timer.h"
#include "cyhal_timer.h"

static cyhal_timer_t timer_obj;

int hal_timer_init()
{
	const cyhal_timer_cfg_t timer_cfg =
	{
		.compare_value = 0,                  // Timer compare value, not used
		.period        = 0xFFFFFFFFUL,              	// Timer period set to a large enough value
		//   compared to event being measured
		.direction     = CYHAL_TIMER_DIR_UP, // Timer counts up
		.is_compare    = false,              // Don't use compare mode
		.is_continuous = true,              // Run timer indefinitely
		.value         = 0                   // Initial value of counter
	};


	// Initialize the timer object. Does not use pin output ('pin' is NC) and does not use a
	// pre-configured clock source ('clk' is NULL).
	cyhal_timer_init(&timer_obj, NC, NULL);
	cyhal_timer_configure(&timer_obj, &timer_cfg);
	cyhal_timer_set_frequency(&timer_obj, 1000);

	return 0;
}

void hal_timer_start()
{
	cyhal_timer_start(&timer_obj);
}

void hal_timer_stop()
{
	cyhal_timer_stop(&timer_obj);
	cyhal_timer_reset(&timer_obj);
}

void hal_timer_restart()
{
	hal_timer_stop();
	hal_timer_start();
}

uint32_t hal_timer_get_ms(void)
{
	return cyhal_timer_read(&timer_obj);
}


