/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * common_err.h
 *
 *  Created on: Mar 7, 2015
 *      Author: cwati
 */

#ifndef COMMON_ERR_H_
#define COMMON_ERR_H_


typedef enum {
	E_SOCK_NOT_READY = -2,
	E_UNUSED = -1,
	E_OK = 0,
	E_INCOMPLETE,
	E_NO_HOST,
	E_NO_MEM,
	E_NO_RESOURCE,
	E_NOT_FOUND,
	E_SOCKET,
	E_TIMEOUT,
	E_UNKNOWN,
} err_t;

#endif /* COMMON_ERR_H_ */
