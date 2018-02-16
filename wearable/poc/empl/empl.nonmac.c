#include <stdlib.h>
#include <fcntl.h>
#include <assert.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>

#include "empl.h"

static int handle = -1;

void empl_output_initialize(void) {
	char tty_path[200] = {0};
	char command[200] = {0};
	handle = posix_openpt(O_RDWR|O_NOCTTY);
	grantpt(handle);
	int written = ptsname_r(handle,tty_path,200);
	assert(written == 0);
	unlockpt(handle);
	written = snprintf(command,200,"/usr/bin/python eMPL-client.py %s",tty_path);
	assert(written >= 0 && written < 200);
	printf("Run the following command to see output:\n%s\n",command);
}

uint32_t float_reinterpret(float value) {
	union {
		float f;
		uint32_t l;
	} float_to_long;
	float_to_long.f = value;
	return float_to_long.l;
}

void output_position(uint32_t id, float w, float x, float y, float z) {
	char header[3] = {'$', 0x02, id%256};
	uint32_t quaternions[4] = {
		float_reinterpret(w),
		float_reinterpret(x),
		float_reinterpret(y),
		float_reinterpret(z)};
	char footer[4] = {0x0, 0x0, '\r', '\n'};
	write(handle, header, 3);
	write(handle, quaternions, 16);
	write(handle, footer, 4);
	fsync(handle);
}
