/*
EPFL MICRO-301 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
may 2022
*/

#include "move.h"

int16_t move_sequence[3*MAX_MOVES] = {'\0'}; //instruction buffer
uint16_t size_move = 0;						 //# of instructions
bool STOP = FALSE;							 //hard stop
bool running_sequence = FALSE;				 //is it moving ?

// semaphore if a sequence is ready
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

	if(!running_sequence){
		set_body_led(0);

		size_move = ReceiveUint16FromComputer(in);

		while(1){

			l1 = chSequentialStreamGet(in); //get first byte of leftspeed
			l2 = chSequentialStreamGet(in); //get second byte of leftspeed
			r1 = chSequentialStreamGet(in); //get first byte of rightspeed
			if(l1 == 'E' && l2 == 'N' && r1 == 'D'){
				i++;
				size_move = i;
				break;
			}
			r2 = chSequentialStreamGet(in); //get second byte of rightspeed
			t1 = chSequentialStreamGet(in); //get first byte of runtime
			t2 = chSequentialStreamGet(in); //get second byte of runtime

			move_sequence[i*3] = (int16_t)((l1 | l2<<8));        // left speed
			move_sequence[(i*3)+1] = (int16_t)((r1 | r2<<8));    // right speed
			move_sequence[(i*3)+2] = (int16_t)((t1 | t2<<8));    // runtime

			i++;
			if(i > MAX_MOVES) {
				chprintf(out,"sequence too long !\r\n");
				size_move = MAX_MOVES;
				break;
			}
		}

		chBSemSignal(&sequence_ready_sem);

	}else{
		chprintf(out,"cannot load sequence, already running moves\r\n");
	}
}

/*
*	Thread to process moves
*/
static THD_WORKING_AREA(moveThreadArea,1024);
static THD_FUNCTION(moveThread,arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while(TRUE){
		chBSemWait(&sequence_ready_sem);
		running_sequence = TRUE;
		STOP = FALSE;
		int16_t lspd = 0;
		int16_t rspd = 0;
		int16_t stime = 0;

		//Move loop.
		for(uint8_t i = 0; i < (size_move); i++){
			lspd = move_sequence[i*3];
			rspd = move_sequence[(i*3)+1];
			stime = move_sequence[(i*3)+2];
			if(STOP) break;
			if(stime == 0) continue;
			left_motor_set_speed(lspd);
			right_motor_set_speed(rspd);
			chThdSleepMilliseconds(stime);
		}
		//stopping after end of sequence.
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		//chprintf(BTH,"DONE");
		chThdSleepMilliseconds(100);
		running_sequence = FALSE;
		chBSemReset(&sequence_ready_sem,TRUE);
	}
}

//command stop.
void stop(void){
	sequence_override();
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

//to override the sequence without stopping.
void sequence_override(void){
	STOP = TRUE;
	running_sequence = FALSE;
}

//command to do a "lidar turn".
void scan(BaseSequentialStream* out){
	static bool dirturn = 0; //turns clockwise or counterclockwise
	stop();
	obstacle_detection_pause();
	set_body_led(0);
	uint16_t dist = 0;
	float Bang = get_angle();
	float ang = Bang;
	//one move sequence setup.
	if(dirturn){
		move_sequence[0] = -118;
		move_sequence[1] = 118;
		dirturn = 0;
	}else{
		move_sequence[0] = 118;
		move_sequence[1] = -118;
		dirturn = 1;
	}
	size_move = 1;
	move_sequence[2] = 11100;	//Optimized for a clean turn and measure.
	STOP = FALSE;
	chBSemSignal(&sequence_ready_sem);
	//communicates position and direction to computer.
	SendFloatToComputer(out,get_posx());
	SendFloatToComputer(out,get_posy());
	SendFloatToComputer(out,get_angle());
	//waits for sequence to begin.
	while(!running_sequence || ang == Bang){
		chThdSleepMilliseconds(200);
		ang = get_angle();
	}
	//measuring loop
	while(running_sequence){
		ang = get_angle();
		dist = VL53L0X_get_dist_mm();
		SendFloatToComputer(out,ang);
		SendUint16ToComputer(out,dist);
		chThdSleepMilliseconds(TIMERES);
	}
	//end condition for computer, unreachable naturally.
	SendUint16ToComputer(out,0xffff);
	SendUint16ToComputer(out,0xffff);
	SendUint16ToComputer(out,0xffff);
	obstacle_detection_continue();
}

void move_index_set(uint8_t index, int16_t lspd, int16_t rspd, int16_t mtime){
	move_sequence[index*3] = lspd;
	move_sequence[index*3 +1] = rspd;
	move_sequence[index*3 +2] = mtime;
}

void set_move_lenght(uint16_t lght){
	size_move = lght;
    chBSemSignal(&sequence_ready_sem);
}

void clear_moveinstr(void){
	for(uint16_t i = 0 ; i < 3*MAX_MOVES ; i++){
		move_sequence[i] = 0;
	}
}

// create move thread.
void lauch_move_thd(void){
	chThdCreateStatic(moveThreadArea,
							  sizeof(moveThreadArea),
							  NORMALPRIO+1,
							  moveThread,
							  NULL);
}
