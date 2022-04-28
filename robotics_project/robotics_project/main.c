#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>
#include <motors.h>
#include <audio/microphone.h>
#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>

#include "main.h"
#include "process_image.h"
#include "communications.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) {
	chSequentialStreamWrite((BaseSequentialStream * )&SD3, (uint8_t* )"START",
			5);
	chSequentialStreamWrite((BaseSequentialStream * )&SD3, (uint8_t* )&size,
			sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream * )&SD3, (uint8_t* )data,
			size);
}

static void serial_start(void) {
	static SerialConfig ser_cfg = { 115200, 0, 0, 0, };

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void) {
	//General Purpose Timer configuration
	//timer 12 is a 16 bit timer so we can measure time
	//to about 65ms with a 1Mhz counter
	static const GPTConfig gpt12cfg = { .frequency = 1000000, /* 1MHz timer clock in order to measure uS.*/
	.callback = NULL, /* Timer callback.*/
	.cr2 = 0, .dier = 0 };

	gptStart(&GPTD12, &gpt12cfg);
	//let the timer count to max value
	gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void) {

	halInit();
	chSysInit();
	mpu_init();

	// Init the messagebus
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	// Start the serial communication
	serial_start();
	// Start the USB communication
	usb_start();
	// Start timer 12
	//timer12_start();
	// Init the motors
	//motors_init();
	// Start the camera
	//dcmi_start();
	//po8030_start();
	process_image_start();
	// Init the proximity sensors
	proximity_start();
	// Calibrate the proximity sensors
	calibrate_ir();
	// Init the TOF sensor
	VL53L0X_start();

	/* Infinite loop. */
	while (1) {

//#define SEND_PROX_VALUES
#ifdef SEND_PROX_VALUES
		int prox_values[8] = {0};

		for (uint8_t prox_sensor_number = 0; prox_sensor_number < 8;
				++prox_sensor_number) {
			prox_values[prox_sensor_number] = get_calibrated_prox(
					prox_sensor_number);
		}
		chprintf((BaseSequentialStream *) &SDU1,
				"Prox. values: {%d, %d, %d, %d, %d, %d, %d, %d}\r\n",
				prox_values[0], prox_values[1], prox_values[2], prox_values[3],
				prox_values[4], prox_values[5], prox_values[6], prox_values[7]);
#endif

#define SEND_TOF_VALUE
#ifdef SEND_TOF_VALUE
		uint16_t dist_mm = VL53L0X_get_dist_mm();
		chprintf((BaseSequentialStream *) &SDU1, "TOF [mm]: %d\r\n", dist_mm);
#endif

		chThdSleepMilliseconds(5);
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
