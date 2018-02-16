/*
 * main.h
 *
 *  Created on: Mar 18, 2015
 *      Author: End User
 */

#ifndef MAIN_H_
#define MAIN_H_

enum _i2c_instances
{
    kMPU9250AuxI2CInstance = 0
};

void mpu9250_reg_int_cb(void (*cb)(void));

#endif /* MAIN_H_ */
