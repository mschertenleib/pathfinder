/*
EPFL MICRO-301 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
may 2022
*/

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "ch.h"
#include "hal.h"
#include <motors.h>
#include <sensors\proximity.h>
#include "move.h"
#include "odometrie.h"

#define TIMEBTWM 50         // time between measures in ms.
#define IR_THRESHOLD 450    // limit value for the proximity sensor.

enum ir_sensor{front_right,fRight,right,back_right,back_left,left,fLeft,front_left};

// starts thread for obstacle detection.
void obstacle_detection_init(void);

void obstacle_detection_pause(void);

void obstacle_detection_continue(void);

void go_away(uint8_t sensor);

#endif // OBSTACLE_H
