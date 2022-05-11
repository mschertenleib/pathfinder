#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <odometrie.h>
#include <sensors\proximity.h>
#include <move.h>
#include <obstacle.h>

bool too_close = FALSE;
bool interrupted = FALSE;
enum side flwside = right;

static THD_WORKING_AREA(obstacleThreadArea,512);
static THD_FUNCTION(obstacleThread,arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

    int front_right,front_left;
    int back_right,back_left;
    uint8_t cumulF = 0;
    uint8_t cumulB = 0;

	while(TRUE){
        front_right = get_calibrated_prox(0);
        back_right = get_calibrated_prox(3);
        back_left = get_calibrated_prox(4);
        front_left = get_calibrated_prox(7);

        too_close = FALSE;
        if(front_right > IR_THRESHOLD-50) too_close = TRUE;
        if(front_left > IR_THRESHOLD-50) too_close = TRUE;
        if(back_right > IR_THRESHOLD-50) too_close = TRUE;
        if(back_right > IR_THRESHOLD-50) too_close = TRUE;
        //chprintf((BaseSequentialStream* ) &SD3,"%i\r\n",right);
        //chprintf((BaseSequentialStream* ) &SD3,"update ir front: %i back: %i\r\n",(front_left+front_right)/2,(back_left + back_right)/2);
        if(too_close){
        
            if((back_left + back_right)/2 > IR_THRESHOLD){
                cumulF += 1;
                if(cumulF >= SAMPLE_FILTER){
                    interrupted = TRUE;
                    sequence_override();
                    left_motor_set_speed(spdPI((back_left + back_right)/2));
                    right_motor_set_speed(spdPI((back_left + back_right)/2));
                    //chThdSleepMilliseconds(100);
                    }
            }else{cumulF = 0;}

            if((front_right + front_left)/2 > IR_THRESHOLD){
                cumulB += 1;
                if(cumulB >= SAMPLE_FILTER){
                    interrupted = TRUE;
                    sequence_override();
                    left_motor_set_speed(-spdPI((front_left + front_right)/2));
                    right_motor_set_speed(-spdPI((front_left + front_right)/2));
                    //chThdSleepMilliseconds(100);
                }
            }else{cumulB = 0;}

        }else if(interrupted){
            left_motor_set_speed(0);
            right_motor_set_speed(0);
            interrupted = FALSE;
        }
        chThdSleepMilliseconds(100);
        
    }
}

int spdPI(int meas){
    int err = (IR_THRESHOLD - meas);
    //chprintf((BaseSequentialStream* ) &SD3,"meas %i err %i\r\n",meas,err);
    return -(err);
}

void obstacle_detection_init(void){
    proximity_start();
    calibrate_ir();
    chThdCreateStatic(obstacleThreadArea,
							  sizeof(obstacleThreadArea),
							  NORMALPRIO,
							  obstacleThread,
							  NULL);
}
