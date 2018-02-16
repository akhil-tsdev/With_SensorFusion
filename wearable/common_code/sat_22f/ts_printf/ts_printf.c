/*
 * Copyright (c) 2016 TuringSense
 *
 * ts_printf.c
 *
 */
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include "ts_printf.h"
/*******************************************************************************
 * Control application and/or debug output
 ******************************************************************************/

// The following variables can be changed at runtime (e.g. by a debugger)
// to control what output goes to the USB CDC.

bool debug_output = DEBUG_GENERAL;
bool debug_nordic = DEBUG_NORDIC;
bool debug_raw_data = DEBUG_RAW_DATA;
bool debug_sensor_fusion_output = DEBUG_SENSOR_FUSION_OUTPUT;
bool cube_output = OUTPUT_3D_CUBE;
bool debug_log_output = DEBUG_PRINT_LOG;

void cube_debug_print_str(char *p)
{
	static bool first = true;
	int l = strlen(p);
	if (first)
	{
		puts("");
		first = false;
	}
	while (l)
	{
		int i;
		putchar('$');
		putchar(1);
		putchar(0);
		for (i = 3; i <= 21; i++)
		{
			if (l)
			{
				putchar(*(p++));
				l--;
			}
			else
				putchar(0);
		}
	}
}

int cond_printf(bool cond, const char *fmt, ...)
{
	va_list ap;
	int rv = 0;
	static int reentrancy_count = 0;
	static char pbuf[200];

	if (!cond)
		return 0;

	// Use gcc atomic add intrinsic to increment a semaphore and test whether
	// we're being called reentrantly, which should never happen, but I saw it
	// once when debugging. If called reentrantly, skip doing anything.
	if (__sync_fetch_and_add(& reentrancy_count, 1) == 0)
	    {
		va_start(ap, fmt);
		rv = vsnprintf(pbuf, sizeof(pbuf), fmt, ap);
		va_end(ap);
		if (cube_output)
			cube_debug_print_str(pbuf);
		else
		    puts(pbuf);
    }
    __sync_fetch_and_add(& reentrancy_count, -1);
	return rv;
}

int cond_putchar(bool cond, int c)
{
	if (cond)
		putchar(c);
	return c;
}

// macros to prevent the direct use of the underlying functions
// Note that redefining putchar() will cause a warning, which can be ignored.

