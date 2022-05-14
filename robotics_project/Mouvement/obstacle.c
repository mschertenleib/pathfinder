/*
EPFL MICRO-301 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
may 2022
*/
#include <obstacle.h>

#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <sensors\proximity.h>

#include "move.h"
#include "odometrie.h"

bool too_close = FALSE;     // condition for stopping the robot.
bool interrupted = FALSE;   // correcting in process.

// Obstacle détection thread.
static THD_WORKING_AREA(obstacleThreadArea,512);
static THD_FUNCTION(obstacleThread,arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

    //variables setup. 
    int front_right,front_left;
    int back_right,back_left;
    uint8_t cumulF = 0;
    uint8_t cumulB = 0;

    //main loop.
	while(TRUE){
        //gets values from IR sensors.
        front_right = get_calibrated_prox(0);
        back_right = get_calibrated_prox(3);
        back_left = get_calibrated_prox(4);
        front_left = get_calibrated_prox(7);

        //detects obstacle condition.
        too_close = FALSE;
        if(front_right > IR_THRESHOLD-RCTPAD) too_close = TRUE;
        if(front_left > IR_THRESHOLD-RCTPAD) too_close = TRUE;
        if(back_right > IR_THRESHOLD-RCTPAD) too_close = TRUE;
        if(back_right > IR_THRESHOLD-RCTPAD) too_close = TRUE;

        //reacts accordingly.
        if(too_close){
            if((back_left + back_right)/2 > IR_THRESHOLD){ //back side.
                cumulF += 1;
                if(cumulF >= SAMPLE_FILTER){
                    interrupted = TRUE;
                    sequence_override();
                    left_motor_set_speed(spdP((back_left + back_right)/2));
                    right_motor_set_speed(spdP((back_left + back_right)/2));
                    //chThdSleepMilliseconds(100);
                    }
            }else{cumulF = 0;} //resets counter if noise.

            if((front_right + front_left)/2 > IR_THRESHOLD){ //front side.
                cumulB += 1;
                if(cumulB >= SAMPLE_FILTER){
                    interrupted = TRUE;
                    sequence_override();
                    left_motor_set_speed(-spdP((front_left + front_right)/2));
                    right_motor_set_speed(-spdP((front_left + front_right)/2));
                    //chThdSleepMilliseconds(100);
                }
            }else{cumulB = 0;} //resets counter if noise.

        // not too close anymore.
        }else if(interrupted){
            left_motor_set_speed(0);
            right_motor_set_speed(0);
            interrupted = FALSE;
        }
        chThdSleepMilliseconds(TIMEBTWM);
    }
}

// simple speed controller
int spdP(int meas){
    int err = (IR_THRESHOLD - meas);
    return -(err);
}

// starts detection thread.
void obstacle_detection_init(void){
	calibrate_ir();
    chThdCreateStatic(obstacleThreadArea,
							  sizeof(obstacleThreadArea),
							  NORMALPRIO+1,
							  obstacleThread,
							  NULL);
}
