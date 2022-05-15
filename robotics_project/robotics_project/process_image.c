#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <camera/po8030.h>
#include <communications.h>
#include <string.h>

#include "main.h"
#include "process_image.h"

static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static BSEMAPHORE_DECL(picture, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	// Configure the camera with an RGB565 format, using the parameters defined in process_image.h
	po8030_advanced_config(FORMAT_RGB565, IMAGE_X, IMAGE_Y, IMAGE_WIDTH,
			IMAGE_HEIGHT, IMAGE_SUBSAMPLING, IMAGE_SUBSAMPLING);
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	while (1) {
		// Wait for an image to be requested
		chBSemWait(&picture);
		// Start a capture
		dcmi_capture_start();
		// Wait for the capture to be done
		wait_image_ready();
		// Signal an image has been captured
		chBSemSignal(&image_ready_sem);
	}
}

static THD_WORKING_AREA(waProcessImage, 512);
static THD_FUNCTION(ProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	uint8_t *img_buff_ptr = NULL;

	while (1) {

		// Wait until an image has been captured
		chBSemWait(&image_ready_sem);

		// Get the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		// Send width and height (subsampling taken into account) to the PC
		SendUint16ToComputer((BaseSequentialStream *)&SD3, IMAGE_WIDTH / SUBSAMPLING_VALUE);
		SendUint16ToComputer((BaseSequentialStream *)&SD3, IMAGE_HEIGHT / SUBSAMPLING_VALUE);

		// Send the binary pixel data to the PC
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t *)img_buff_ptr, IMAGE_BUFFER_SIZE);
	}
}

void process_image_start(void) {
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO,
			ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO,
			CaptureImage, NULL);
}

void get_picture(void) {
	// Request an image to be captured
	chBSemSignal(&picture);
}
