#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <communications.h>
#include <arm_math.h>
#include <msgbus/messagebus.h>
#include <move.h>
#include <audio/audio_thread.h>
#include <odometrie.h>
#include <listen.h>
#include "obstacle.h"
#include <sensors/proximity.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <main.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void) {
	static SerialConfig ser_cfg = { 115200, 0, 0, 0, };

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void) {

	halInit();
	chSysInit();
	mpu_init();

	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//quantum sensible inits
	dcmi_start();
	po8030_start();
	motors_init();
	proximity_start();
	VL53L0X_start();
	serial_start();
	usb_start();

	//Lauch threads
	lauch_move_thd();
	lauch_odometrie_thd();	
	process_image_start();
	obstacle_detection_init();

	//setup
	set_pos(0,0,0);

	/* Infinite loop. */
	while (1) {
		listen(BTH,BTH);
		chThdSleepMilliseconds(100);
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
