/*
 * Copyright © 2015 Turingsense, Inc.
 *
 * custom_board.h
 *
 * Main Turingsense custom board definition.
 *
 */
 
#ifndef __CUSTOM_BOARD_H__
#define __CUSTOM_BOARD_H__
 
#if (PRODUCTION1 && EVAL_BOARD)
#error "Hey what board are you? You can't be both!"
#endif
#if (HUB_NORDIC && PRODUCTION1)
#include "hub_production_board.h"
#elif (HUB_NORDIC && EVAL_BOARD)
#include "hub_eval_board.h"
#else
#error "Hey what board are you?"
#endif
 
#endif /* __CUSTOM_BOARD_H__ */
