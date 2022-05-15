/*
EPFL MICRO-301 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
may 2022
*/

#ifndef OBSTACLE_H
#define OBSTACLE_H

#define SAMPLE_FILTER 3     // # of measures to confirm stop conditions.
#define IR_THRESHOLD 350    // limit value for the proximity sensor.
#define RCTPAD 50           // reaction padding
#define TIMEBTWM 50        // time between measures in ms.
#define FLWSPD 100          // follow speed in steps per seconds.

// starts thread for obstacle detection.
void obstacle_detection_init(void);

// speed controller (just kp)
int spdP(int meas);

#endif // OBSTACLE_H
