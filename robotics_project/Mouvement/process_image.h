#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

// Macro to concatenate words
#define CONCATENATE_IMPL(a, b) a ## b
#define CONCATENATE(a, b) CONCATENATE_IMPL(a, b)

// Maximum image dimensions
#define TOTAL_IMAGE_WIDTH 640
#define TOTAL_IMAGE_HEIGHT 480

// Image dimensions used
#define IMAGE_WIDTH 400
#define IMAGE_HEIGHT 384
#define SUBSAMPLING_VALUE 4

#define IMAGE_X (TOTAL_IMAGE_WIDTH - IMAGE_WIDTH) / 2 // Capture a zone in the center of the camera
#define IMAGE_Y (TOTAL_IMAGE_HEIGHT - IMAGE_HEIGHT) / 2 // Capture a zone in the center of the camera
#define IMAGE_SUBSAMPLING CONCATENATE(SUBSAMPLING_X, SUBSAMPLING_VALUE) // Literal subsampling_t value defined in po8030.h
#define BYTES_PER_PIXEL 2
#define IMAGE_BUFFER_SIZE ((IMAGE_WIDTH / SUBSAMPLING_VALUE) * (IMAGE_HEIGHT / SUBSAMPLING_VALUE) * BYTES_PER_PIXEL)
#define MAX_IMAGE_BUFFER_SIZE MAX_BUFF_SIZE // Defined in dcmi_camera.h

// Make sure IMAGE_WIDTH is a multiple of SUBSAMPLING_VALUE
#if (IMAGE_WIDTH % SUBSAMPLING_VALUE != 0)
#error "IMAGE_WIDTH is not a multiple of SUBSAMPLING_VALUE"
#endif

// Make sure IMAGE_HEIGHT is a multiple of SUBSAMPLING_VALUE
#if (IMAGE_HEIGHT % SUBSAMPLING_VALUE != 0)
#error "IMAGE_HEIGHT is not a multiple of SUBSAMPLING_VALUE"
#endif

// Make sure the image buffer is not too large
#if (IMAGE_BUFFER_SIZE > MAX_IMAGE_BUFFER_SIZE)
#error "IMAGE_BUFFER_SIZE is too large"
#endif

// Start the image capture and processing threads
void process_image_start(void);

// Request an image to be captured and sent to the PC
void get_picture(void);

#endif /* PROCESS_IMAGE_H */
