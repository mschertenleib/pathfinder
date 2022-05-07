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
							  NORMALPRIO,
							  odometrieThd,
							  NULL);
}

void measure_and_add(float* pos, int mlc, int mrc){
    short cdiv = pgdc(mlc,mrc);
    short dl = mlc/cdiv;
    short dr = mrc/cdiv;
    float dangle =compute_phi(dl,dr);
    pos[2] += dangle;
    float ddist = compute_dist(dl,dr,pos[2]);
    pos[0] += ddist*sinf(pos[2]);
    pos[1] += ddist*cosf(pos[2]);
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
    short op = 0;
    if(l == r){
        return 0;
    }
    else if (l > r){
        op = l-r;
        return atanf( ((float)op*CMPSTEP) / RBTWIDTHCM );
    }
    else if (r > l){
        op = r-l;
        return -atanf( ((float)op*CMPSTEP) / RBTWIDTHCM );
    }
}

float compute_dist(short l, short r, float ang){
	float ratio = 0;
	float radius = 0;
    
    if(l==r) return l*CMPSTEP; // pas super precis
    else{
        return ((l + r)/2.0f)*CMPSTEP;
    }
    /*
    else if(l==0 || r==0){
    	radius = RBTWIDTHCM/2;
    	return 2*radius*sinf(ang/2);
    }else{
    	ratio = ((float)l/(float)r);
		radius = (ratio + RBTWIDTHCM)/(2*(ratio-1));
		return 2*radius*sinf(ang/2);
    }*/
}
