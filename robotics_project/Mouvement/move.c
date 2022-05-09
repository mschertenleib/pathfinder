#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <motors.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <odometrie.h>
#include <main.h>

#include <move.h>

int16_t move_sequence[3*MAX_MOVES] = {'\0'};
uint8_t size_move = 0;
bool running_sequence = FALSE;

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

	if(!running_sequence){
		set_body_led(0);
		l1 = chSequentialStreamGet(in);
		l2 = chSequentialStreamGet(in);

		// The first 2 bytes is the length of the datas
		// -> number of int16_t data
		size_move = (int16_t)((l2 | l1<<8));
		size_move = size_move/3;
		chprintf(out,"Length : %x \r\n",size_move);

		while(1){

			l1 = chSequentialStreamGet(in); //get first byte of leftspeed
			l2 = chSequentialStreamGet(in); //get second byte of leftspeed
			r1 = chSequentialStreamGet(in); //get first byte of rightspeed
			if(l1 == 'E' && l2 == 'N' && r1 == 'D'){
				i++;
				chprintf(out,"STOP Detected after %d moves\r\n",i-1);
				break;
			}
			r2 = chSequentialStreamGet(in); //get second byte of rightspeed
			t1 = chSequentialStreamGet(in); //get first byte of runtime
			t2 = chSequentialStreamGet(in); //get second byte of runtime

			chprintf(out,"loading %d L %d R %d T%d \r\n",i,(int16_t)((l2 | l1<<8)),(int16_t)((r2 | r1<<8)),(int16_t)((t2 | t1<<8)));

			move_sequence[i*3] = (int16_t)((l2 | l1<<8));        // left speed
			move_sequence[(i*3)+1] = (int16_t)((r2 | r1<<8));    // right speed
			move_sequence[(i*3)+2] = (int16_t)((t2 | t1<<8));    // runtime

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
		int16_t lspd = 0;
		int16_t rspd = 0;
		int16_t stime = 0;
		for(uint8_t i = 0; i < (size_move); i++){
			lspd = move_sequence[i*3];
			rspd = move_sequence[(i*3)+1];
			stime = move_sequence[(i*3)+2];
			left_motor_set_speed(lspd);
			right_motor_set_speed(rspd);
			//chprintf((BaseSequentialStream *) &SD3,"running %d L %d R %d for %d ms \r\n",i,lspd,rspd,stime);
			chThdSleepMilliseconds(stime);
		}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		//chprintf((BaseSequentialStream *) &SD3,"DONE\r\n");
		chThdSleepMilliseconds(100);
		running_sequence = FALSE;
		set_body_led(1);
		chBSemReset(&sequence_ready_sem,TRUE);
	}
}

void stop(BaseSequentialStream* out){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	chprintf(out,"stopped.\r\n");
}

void scan(BaseSequentialStream* in ,BaseSequentialStream* out){
	set_body_led(0);
	volatile uint8_t turns = chSequentialStreamGet(in);
	float ang = get_angle();
	int16_t dist;
	int turnspd = (int)(3.14159265359*RBTWIDTHCM*1000)/(13*secscan);
	size_move = 1;
	move_sequence[0] = turnspd;
	move_sequence[1] = -turnspd;
	move_sequence[2] = secscan*turns*1000;
	chBSemSignal(&sequence_ready_sem);
	chprintf(out,"X%f Y%f P%f\r\n",get_posx(),get_posy(),get_angle());
	while(!running_sequence){
		chThdSleepMilliseconds(100);
	}
	while(running_sequence){
		ang = get_angle();
		dist = VL53L0X_get_dist_mm();
		chprintf(out,"P%f D%i\r\n",ang,dist);
		chThdSleepMilliseconds(TIMERES);
	}
	chprintf(out,"MAP\r\n");
}

void lauch_move_thd(void){
	chThdCreateStatic(moveThreadArea,
							  sizeof(moveThreadArea),
							  NORMALPRIO+1,
							  moveThread,
							  NULL);
}
