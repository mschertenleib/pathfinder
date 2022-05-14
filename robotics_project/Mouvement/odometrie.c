#include "odometrie.h"

#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <math.h>
#include <main.h>
#include <motors.h>
#include <usbcfg.h>
#include <chprintf.h>

static float rob_pos[3] = {0,0,0};

static THD_WORKING_AREA(odomThdArea,1024);
static THD_FUNCTION(odometrieThd,arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while(TRUE){
        set_front_led(1);
        int dl = left_motor_get_pos();
        int dr = right_motor_get_pos();
        
        measure_and_add(rob_pos,dl,dr);

        left_motor_set_pos(0);
        right_motor_set_pos(0);
        set_front_led(0);
        //chprintf((BaseSequentialStream *) &SD3,"X%f Y%f P%f\r\n",rob_pos[0],rob_pos[1],(rob_pos[2])*(180/3.14159265359));
        //chprintf((BaseSequentialStream *) &SD3,"dl: %i dr: %i Im at : %.2f %.2f %.2f \r\n",dl,dr,rob_pos[0],rob_pos[1],rob_pos[2]);
        chThdSleepMilliseconds(TIMERES);
	}
}

float get_angle(void){
    return rob_pos[2];
}

float get_posx(void){
    return rob_pos[0];
}

float get_posy(void){
    return rob_pos[1];
}

void set_pos(float x, float y, float phi){
    rob_pos[0] = x;
    rob_pos[1] = y;
    rob_pos[2] = phi;
}

void lauch_odometrie_thd(void){

    left_motor_set_pos(0);
    right_motor_set_pos(0);
	chThdCreateStatic(odomThdArea,
							  sizeof(odomThdArea),
							  NORMALPRIO+2,
							  odometrieThd,
							  NULL);
}

void measure_and_add(float* pos, int mlc, int mrc){


	float angle_inc =compute_phi(mlc,mrc);
	pos[2] = fmod((pos[2]+angle_inc),M_TWOPI);
	float dist_inc = compute_dist(mlc,mrc,pos[2]);
	pos[0] += dist_inc*cosf(pos[2]);
	pos[1] += dist_inc*sinf(pos[2]);
	/*
	float angle_inc =compute_phi(mlc,mrc);
	float rad = compute_rad(mlc,mrc);
	pos[0] += rad*(1-cos(pos[2]+(angle_inc/2)));
	pos[1] += rad*(sin(pos[2]+(angle_inc/2)));
	pos[2] = fmod((pos[2]+angle_inc),M_TWOPI);
	*/
}

// internal functions

short pgdc(short a,short b){
    if(a<0) a = -a;
    if(b<0) b = -b;
    do{
    	if(a==0 || b==0) return 1;
        if(a > b) a = a-b;
        else b = b-a; 
    }while(a != b);
    return a;
}

float compute_phi(short l, short r){

	short diff = l - r;
	return -atan2f((float)diff*CMPSTEP,RBTWIDTHCM);

    // mathieu's method

	//return (r-l)/(RBTWIDTHCM*(1000/13.0f));

}

float compute_dist(short l, short r, float ang){
    if(l==r) return l*CMPSTEP; // pas super precis
    else{
        return ((l + r)/2.0f)*CMPSTEP;
    }
}

float compute_rad(short l, short r){
	//mathieu's method
	return 0.5 * ((l + r) / (r - l)) * RBTWIDTHCM;
}
