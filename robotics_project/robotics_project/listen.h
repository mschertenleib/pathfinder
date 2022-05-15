/*
EPFL MICRO-315 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
May 2022
*/

#ifndef LISTEN_H
#define LISTEN_H

#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <chprintf.h>
#include <string.h>

#include "main.h"
#include "move.h"
#include "odometrie.h"
#include "process_image.h"
#include "communications.h"

// Listen to commands and executes them.
/*
        !CLR = Sets position and movement sequence.
        !MOVE = Moves according to speed instruction.
        !POS = Sends current position
        !STOP = Stops
        !PIC = Takes a picture.
        !SCAN = Does a "lidar" turn.
        !BEEP = beeps at a frequency for 100ms
*/
void listen( BaseSequentialStream* in, BaseSequentialStream* out);

#endif //LISTEN_H
