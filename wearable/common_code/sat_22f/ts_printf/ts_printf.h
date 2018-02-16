/*
 * Copyright (c) 2016 TuringSense
 *
 * ts_printf.h
 *
 */

#ifndef __TS_PRINTF_H__
#define __TS_PRINTF_H__

/* USB */
void VirtualCom_Deinit(); //virtual_com.h
#include "fsl_debug_console.h"

// The following defines determine what things are output by default to the
// USB CDC. Don't ifdef code based on these, as it will prevent the ability to
// control at runtime by changing the boolean variables (e.g., with a debugger).


#define ENABLE_PRINTF	0	/* Unless you set this to 1, no printf will be enabled.
							 * If you enable this to 1, you must connect the USB to let the printf got out,
							 * otherwise the code will get stuck waiting to print.. */

#define DEBUG_GENERAL   1
#define DEBUG_RAW_DATA  0
#define DEBUG_SENSOR_FUSION_OUTPUT 0
#define DEBUG_NORDIC    0
#define OUTPUT_3D_CUBE  0
#define DEBUG_PRINT_LOG 0

void cube_debug_print_str(char *p);
int cond_printf(bool cond, const char *fmt, ...);
int cond_putchar(bool cond, int c);

bool debug_output;
bool debug_nordic;
bool debug_raw_data;
bool debug_sensor_fusion_output;
bool cube_output;
bool debug_log_output;

#if ENABLE_PRINTF
#define debug_printf(...) cond_printf(debug_output, __VA_ARGS__)

#define debug_raw_data_printf(...) cond_printf(debug_raw_data, __VA_ARGS__)

#define debug_sensor_fusion_output_printf(...) cond_printf(debug_sensor_fusion_output, __VA_ARGS__)

#define debug_nordic_printf(...) cond_printf(debug_nordic, __VA_ARGS__)

#define cube_printf(...) cond_printf(cube_output, __VA_ARGS__)
#define cube_putchar(c) cond_putchar(cube_output, c)

#define debug_log_printf(...) cond_printf(debug_log_output, __VA_ARGS__)

#else

void no_printf(bool cond, const char *fmt, ...) {
	return;
}
#define debug_printf(...) no_printf(debug_output, __VA_ARGS__)
#define debug_raw_data_printf(...) no_printf(debug_raw_data, __VA_ARGS__)
#define debug_sensor_fusion_output_printf(...) no_printf(debug_sensor_fusion_output, __VA_ARGS__)
#define debug_nordic_printf(...) no_printf(debug_nordic, __VA_ARGS__)
#define cube_printf(...) no_printf(cube_output, __VA_ARGS__)
#define cube_putchar(c) no_printf(cube_output, c)
#define debug_log_printf(...) no_printf(debug_log_output, __VA_ARGS__)

#endif /* ENABLE_PRINTF */

#endif /* __TS_PRINTF_H__ */
