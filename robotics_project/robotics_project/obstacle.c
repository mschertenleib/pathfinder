/*
EPFL MICRO-315 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
May 2022
*/

#include "obstacle.h"

bool obst_active = TRUE;    // is detection active 

// Obstacle detection thread.
static THD_WORKING_AREA(obstacleThreadArea,512);
static THD_FUNCTION(obstacleThread,arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

    //main loop.
	while(TRUE){
        //active/inactive obstacle detection.
		if(!obst_active){
            chThdSleepMilliseconds(500);
            continue;
        }
        //go trough ir sensors values.
        for(uint8_t ir_index = 0 ; ir_index < 8 ; ir_index++){
            //if detect a close encounter, gets away from it;
            if(get_calibrated_prox(ir_index) > IR_THRESHOLD) go_away(ir_index);
        }

        chThdSleepMilliseconds(TIMEBTWM);
    }
}

// simple mapping for speed control
// val going from amin to amax mapped to bmin to bmax.
void spd_control(int measL, int measR, float w){
    int common = (measL + measR)/2;
    int err = common - IR_THRESHOLD;
    int diff = (measL - measR)/4;
    left_motor_set_speed((int)(w*(err + diff)));
    right_motor_set_speed((int)(w*(err - diff)));
}

//pause the obstacle detection.
void obstacle_detection_pause(void){
	obst_active = 0;
}

//re activate obstacle detection.
void obstacle_detection_continue(void){
	obst_active = 1;
}

void go_away(uint8_t sensor){
    stop();
    switch (sensor)
    {
    case front_right:
        move_index_set(0,-308,-308,500);
        set_move_lenght(1);
        chThdSleepMilliseconds(510);
        break;
    case fRight:
        move_index_set(0,236,-236,694);
        move_index_set(1,-308,-308,500);
        set_move_lenght(2);
        chThdSleepMilliseconds(1200);
        break;
    case right:
        move_index_set(0,236,-236,1387);
        move_index_set(1,-308,-308,500);
        set_move_lenght(2);
        chThdSleepMilliseconds(1900);
        break;
    case back_right:
        move_index_set(0,-236,236,650);
        move_index_set(1,308,308,500);
        set_move_lenght(2);
        chThdSleepMilliseconds(1200);
        break;
    case back_left:
        move_index_set(0,236,-236,650);
        move_index_set(1,308,308,500);
        set_move_lenght(2);
        chThdSleepMilliseconds(1200);
        break;
    case left:
        move_index_set(0,-236,236,1387);
        move_index_set(1,-308,-308,500);
        set_move_lenght(2);
        chThdSleepMilliseconds(1900);
        break;
    case fLeft:
        move_index_set(0,-236,236,694);
        move_index_set(1,-308,-308,500);
        set_move_lenght(2);
        chThdSleepMilliseconds(1200);
        break;
    case front_left:
        move_index_set(0,-308,-308,500);
        set_move_lenght(1);
        chThdSleepMilliseconds(510);
        break;
    default:
        break;
    }
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
