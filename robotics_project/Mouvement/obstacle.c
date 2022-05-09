#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <sensors\proximity.h>
#include "odometrie.h"

void obstacle_detection_init(void){
    proximity_start();
    calibrate_ir();
    
}

static THD_WORKING_AREA(obstacleThreadArea,1024);
static THD_FUNCTION(obstacleThread,arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while(TRUE){

        chThdSleepMilliseconds(200);
    }
}