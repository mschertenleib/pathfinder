/*
EPFL MICRO-315 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
May 2022
*/

#ifndef MOVE_H
#define MOVE_H

#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <motors.h>
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include "main.h"
#include "communications.h"
#include "odometrie.h"
#include "obstacle.h"

#define MAX_MOVES 100

void ReceiveSpeedInstMove(BaseSequentialStream* in,BaseSequentialStream* out);

void stop(void);

void sequence_override(void);

void scan(BaseSequentialStream* out);

void move_index_set(uint8_t index, int16_t lspd, int16_t rspd, int16_t mtime);

void set_move_lenght(uint16_t lght);

void clear_moveinstr(void);

void lauch_move_thd(void);


#endif /* MOVE_H */
