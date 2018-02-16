
#include "boards.h"
#include "nrf_gpio.h"

/// Every 200 times you call this, the LED of your choice inverts.
/// Good for watching things move through the system.
#if EVAL_BOARD
#define FLIP_TRACE(led) do { \
	static uint8_t flip_trace = 0; \
	flip_trace++; \
	if(flip_trace >= 200) {\
		flip_trace = 0;\
		uint32_t leds[] = LEDS_LIST; \
		nrf_gpio_pin_toggle(leds[led]); \
	}\
} while(false)
#else
#define FLIP_TRACE(led) do { \
	;\
} while(false)
#endif
