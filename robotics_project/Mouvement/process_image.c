#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <camera/po8030.h>

#include <string.h>

#include "main.h"
#include "process_image.h"

static bool picture = 0;

static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 1024);
static THD_FUNCTION(CaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	po8030_advanced_config(FORMAT_RGB565, IMAGE_X, IMAGE_Y, IMAGE_WIDTH,
			IMAGE_HEIGHT, IMAGE_SUBSAMPLING, IMAGE_SUBSAMPLING);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	while (1) {
		if(picture){
			// Start a capture
			dcmi_capture_start();
			// Wait for the capture to be done
			wait_image_ready();
			// Signal an image has been captured
			chBSemSignal(&image_ready_sem);
		}else{
			chThdSleepMilliseconds(1000);
		}
	}
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	uint8_t *img_buff_ptr = NULL;

	uint8_t current_loop = 0;

	while (1) {
		if(picture){
			// Wait until an image has been captured
			chBSemWait(&image_ready_sem);

			// Get the pointer to the array filled with the last image in RGB565
			img_buff_ptr = dcmi_get_last_image_ptr();

			if (current_loop == 0) {
				picture = FALSE;
				chSequentialStreamWrite((BaseSequentialStream * )&SD3, img_buff_ptr, IMAGE_BUFFER_SIZE);
			}
			current_loop = (current_loop + 1) % 16;
		}else{
			chThdSleepMilliseconds(1000);
		}
	}
}

void process_image_start(void) {
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO,
			ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO,
			CaptureImage, NULL);
}

void get_picture(void){
	picture = TRUE;
}
