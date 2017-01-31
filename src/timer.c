#include "timer.h"

volatile uint32_t time_var1;

/*
 * Called from systick handler
 */
void timing_handler() {
	if (time_var1) {
		time_var1--;
	}
}

/*
 * Delay a number of systick cycles
 */
void Delay(volatile uint32_t nCount) {
	time_var1 = nCount;

	while(time_var1){};
}

