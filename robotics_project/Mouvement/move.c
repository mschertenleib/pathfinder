#include "ch.h"
#include "hal.h"
#include <main.h>
#include <motors.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <move.h>

uint16_t move_sequence[NB_MOVES*3];
bool running_sequence = FALSE;

// semaphore if sequence has been uploaded
static BSEMAPHORE_DECL(sequence_ready_sem, TRUE);

/*
*	Receives int16 values from the computer and fill a uint16 array with 
    3 numbers: left motor speed, right motor speed, runtime(in ms). 
    Size is the number of instructions to be recieved.
*	=> data should be of size (3 * size)

*   data = [left speed 0,right speed 0, runtime 0, left speed 1, right speed 1, runtime 1, ...]
*   command example : MOVE nbinstr instructions
*/
void ReceiveSpeedInstMove(BaseSequentialStream* in, BaseSequentialStream* out, uint16_t size)
{
    volatile uint8_t l1,l2,r1,r2,t1,t2;
	volatile uint16_t temp_size = 0;
	uint16_t i=0;

	if(!running_sequence){
		uint8_t state = 0;
		while(state != 4){

			l1 = chSequentialStreamGet(in);
			chprintf(out,"ASCII %c, Hex %x, Dec %d.\r\n",l1,l1,l1);
			//State machine to detect the string EOF\0S in order synchronize
			//with the frame received
			switch(state){
				case 0:
					if(l1 == 'M')
						state = 1;
					else
						state = 0;
				case 1:
					if(l1 == 'O')
						state = 2;
					else if(l1 == 'M')
						state = 1;
					else
						state = 0;
				case 2:
					if(l1 == 'V')
						state = 3;
					else if(l1 == 'M')
						state = 1;
					else
						state = 0;
				case 3:
					if(l1 == 'E')
						state = 4;
					else if(l1 == 'M')
						state = 1;
					else
						state = 0;
			}
			
		}

		chprintf(out,"INSTRUCTION MOVE\r\n");

		l1 = chSequentialStreamGet(in);
		l2 = chSequentialStreamGet(in);

		// The first 2 bytes is the length of the datas
		// -> number of int16_t data
		temp_size = (int16_t)((l2 | l1<<8));
		//chprintf((BaseSequentialStream *) &SDU1,"Length : %x \r\n",temp_size);

		if((temp_size/3) == size){
			for(i = 0 ; i < (temp_size/3) ; i++){

				l1 = chSequentialStreamGet(in); //get first byte of leftspeed
				l2 = chSequentialStreamGet(in); //get second byte of leftspeed
				r1 = chSequentialStreamGet(in); //get first byte of rightspeed
				r2 = chSequentialStreamGet(in); //get second byte of rightspeed
				t1 = chSequentialStreamGet(in); //get first byte of runtime
				t2 = chSequentialStreamGet(in); //get second byte of runtime

				move_sequence[i*3] = (int16_t)((l2 | l1<<8));        // left speed
				move_sequence[(i*3)+1] = (int16_t)((r2 | r1<<8));    // right speed
				move_sequence[(i*3)+2] = (int16_t)((t2 | t1<<8));    // runtime

				chprintf(out,"load %d L %d R %d T%d \r\n",i,move_sequence[i*3],move_sequence[(i*3)+1],move_sequence[(i*3)+2]);
			}

			chprintf(out,"M");
			chBSemSignal(&sequence_ready_sem);
		}
	}else{
		chprintf(out,"cannot load sequence, main sequence being used %d \r\n",chSequentialStreamGet(in));
	}
}


static THD_WORKING_AREA(moveThreadArea,1024);
static THD_FUNCTION(moveThread,arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while(TRUE){
		chBSemWait(&sequence_ready_sem);

		running_sequence = TRUE;
		static uint8_t size = sizeof(move_sequence)/(sizeof(uint16_t)*3);
		for(uint8_t i = 0 ; i < size ; i++){
			left_motor_set_speed(move_sequence[i*3]);
			right_motor_set_speed(move_sequence[(i*3)+1]);
			chThdSleepMilliseconds(move_sequence[(i*3)+2]);
			chprintf((BaseSequentialStream *) &SD3,"running: %d L%d R%d T%d \r\n",i,move_sequence[i*3],move_sequence[(i*3)+1],move_sequence[(i*3)+2]);
		}
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		chThdSleepMilliseconds(100);
		chprintf((BaseSequentialStream *) &SD3,"DONE");
		running_sequence = FALSE;
		chBSemReset(&sequence_ready_sem,TRUE);
	}
}

void lauch_move_thd(void){
	chThdCreateStatic(moveThreadArea,
							  sizeof(moveThreadArea),
							  NORMALPRIO-1,
							  moveThread,
							  NULL);
}

void RunSpeedInstSequence(){
    if(!running_sequence){

	}else{
		chprintf((BaseSequentialStream *) &SDU1,"sequence already running\r\n");
	}
}
