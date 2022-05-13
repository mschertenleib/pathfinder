#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <motors.h>
#include <usbcfg.h>
#include <math.h>
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <communications.h>
#include <odometrie.h>
#include <main.h>

#include <move.h>

uint8_t current_move = 0;
int16_t move_sequence[3*MAX_MOVES] = {'\0'};
uint16_t size_move = 0;
bool STOP = FALSE;
bool running_sequence = FALSE;
bool scanning = FALSE;

// semaphore if sequence has been uploaded
static BSEMAPHORE_DECL(sequence_ready_sem, TRUE);

/*
*	Receives int16 values from the computer and fill a uint16 array with 
    3 numbers: left motor speed, right motor speed, runtime(in ms). 
    Size is the number of instructions to be recieved.

*   data = [left speed 0,right speed 0, runtime 0, left speed 1, right speed 1, runtime 1, ...]
*   command example : MOVE nbinstr instructions END
*/
void ReceiveSpeedInstMove(BaseSequentialStream* in, BaseSequentialStream* out)
{
    volatile uint8_t l1,l2,r1,r2,t1,t2;
	volatile uint16_t i=0;
	STOP = FALSE;

	if(!running_sequence && !scanning){
		set_body_led(0);

		size_move = ReceiveUint16FromComputer(out);

		while(1){

			l1 = chSequentialStreamGet(in); //get first byte of leftspeed
			l2 = chSequentialStreamGet(in); //get second byte of leftspeed
			r1 = chSequentialStreamGet(in); //get first byte of rightspeed
			if(l1 == 'E' && l2 == 'N' && r1 == 'D'){
				i++;
				break;
			}
			r2 = chSequentialStreamGet(in); //get second byte of rightspeed
			t1 = chSequentialStreamGet(in); //get first byte of runtime
			t2 = chSequentialStreamGet(in); //get second byte of runtime

			move_sequence[i*3] = (int16_t)((l1 | l2<<8));        // left speed
			move_sequence[(i*3)+1] = (int16_t)((r1 | r2<<8));    // right speed
			move_sequence[(i*3)+2] = (int16_t)((t1 | t2<<8));    // runtime

			i++;
			if(i > MAX_MOVES-2) {
				chprintf(out,"sequence too long !\r\n");
				size_move = MAX_MOVES-2;
				break;
			}
		}

		chBSemSignal(&sequence_ready_sem);

	}else{
		chprintf(out,"cannot load sequence, a sequence being used %d \r\n",chSequentialStreamGet(in));
	}
	set_body_led(0);
}


static THD_WORKING_AREA(moveThreadArea,1024);
static THD_FUNCTION(moveThread,arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while(TRUE){
		chBSemWait(&sequence_ready_sem);
		chThdSleepMilliseconds(500);
		running_sequence = TRUE;
		current_move = 0;
		int16_t lspd = 0;
		int16_t rspd = 0;
		int16_t stime = 0;
		for(uint8_t i = 0; i < (size_move); i++){
			current_move = i;
			lspd = move_sequence[i*3];
			rspd = move_sequence[(i*3)+1];
			stime = move_sequence[(i*3)+2];
			if(STOP) break;
			if(stime == 0) continue;
			left_motor_set_speed(lspd);
			right_motor_set_speed(rspd);
			//chprintf((BaseSequentialStream *) &SD3,"running %d L %d R %d for %d ms \r\n",i,lspd,rspd,stime);
			chThdSleepMilliseconds(stime);
		}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		if(!STOP) current_move = 0;
		//chprintf((BaseSequentialStream *) &SD3,"DONE\r\n");
		chThdSleepMilliseconds(100);
		running_sequence = FALSE;
		set_body_led(1);
		chBSemReset(&sequence_ready_sem,TRUE);
	}
}

void stop(BaseSequentialStream* out){
	sequence_override();
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	STOP = TRUE;
	//chprintf(out,"stopped.\r\n");
}

void sequence_override(void){
	STOP = TRUE;
	running_sequence = FALSE;
}

void scan(BaseSequentialStream* out){
	if(!running_sequence && !scanning){
		set_body_led(0);
		float start_ang = get_angle();
		float ang = start_ang;
		uint16_t dist;
		int turnspd = (int)(M_PI*RBTWIDTHCM*1000)/(13.0f*secscan);
		size_move = 1;
		move_sequence[0] = turnspd;
		move_sequence[1] = -turnspd;
		move_sequence[2] = secscan*1100;
		STOP = FALSE;
		chBSemSignal(&sequence_ready_sem);
		SendFloatToComputer(out,get_posx());
		SendFloatToComputer(out,get_posy());
		SendFloatToComputer(out,get_angle());
		while(!running_sequence  || (ang == start_ang)){
			chThdSleepMilliseconds(100);
		}
		while(running_sequence){
			ang = get_angle();
			dist = VL53L0X_get_dist_mm();
			SendFloatToComputer(out,ang);
			SendUint16ToComputer(out,dist);
			chThdSleepMilliseconds(TIMERES);
		}
		SendUint16ToComputer(out,0xffff);
	}
}

void lauch_move_thd(void){
	chThdCreateStatic(moveThreadArea,
							  sizeof(moveThreadArea),
							  NORMALPRIO+1,
							  moveThread,
							  NULL);
}
