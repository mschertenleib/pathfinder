/*
EPFL MICRO-301 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
may 2022
*/

#include "odometrie.h"

#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <math.h>
#include <main.h>
#include <motors.h>
#include <usbcfg.h>

// measured position of robot
static float rob_pos[3] = {0,0,0};

//Thread for measuring position.
static THD_WORKING_AREA(odomThdArea,1024);
static THD_FUNCTION(odometrieThd,arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

    //thread loop.
	while(TRUE){
        set_front_led(1); //indicates thread is working.
        //gets motor position since last measure.
        int dl = left_motor_get_pos();
        int dr = right_motor_get_pos();
        
        measure_and_add(rob_pos,dl,dr);

        // resets motors positions
        left_motor_set_pos(0);
        right_motor_set_pos(0);
        set_front_led(0);

        chThdSleepMilliseconds(TIMERES);
	}
}

//returns current angle.
float get_angle(void){
    return rob_pos[2];
}

//returns current horizontal position.
float get_posx(void){
    return rob_pos[0];
}

//returns current vertical position.
float get_posy(void){
    return rob_pos[1];
}

// sets position manually.
void set_pos(float x, float y, float phi){
    rob_pos[0] = x;
    rob_pos[1] = y;
    rob_pos[2] = phi;
}

// starts position measuring thread.
void lauch_odometrie_thd(void){
    left_motor_set_pos(0);
    right_motor_set_pos(0);
	chThdCreateStatic(odomThdArea,
							  sizeof(odomThdArea),
							  NORMALPRIO+2,
							  odometrieThd,
							  NULL);
}

// modifies the position every delta t. 
void measure_and_add(float* pos, int mlc, int mrc){
	float angle_inc =compute_phi(mlc,mrc);
	pos[2] = fmod((pos[2]+angle_inc),M_TWOPI);
	float dist_inc = compute_dist(mlc,mrc,pos[2]);
	pos[0] += dist_inc*cosf(pos[2]);
	pos[1] += dist_inc*sinf(pos[2]);
}

// approximation for angle change.
float compute_phi(short l, short r){
	short diff = l - r;
	return -atan2f((float)diff*CMPSTEP,RBTWIDTHCM);
}

// approximation for distance change.
float compute_dist(short l, short r, float ang){
    if(l==r) return l*CMPSTEP; 
    else{
        return ((l + r)/2.0f)*CMPSTEP;
    }
}
