#include "empl.h"
#include <math.h>

#ifndef PI
#define PI 3.14
#endif

int main() {
	empl_output_initialize();
	float theta = 0;
	while(1) {
		theta += PI/30;
		if(theta > 2*PI) theta -= 2*PI;
		output_position(1,sin(theta),cos(theta),cos(theta),0);
	}
	return 0;
}
