#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define CONCATENATE_IMPL(a, b) a ## b
#define CONCATENATE(a, b) CONCATENATE_IMPL(a, b)

#define TOTAL_IMAGE_WIDTH 640
#define TOTAL_IMAGE_HEIGHT 480

#define IMAGE_WIDTH 300 //640
#define IMAGE_HEIGHT 256 //120
#define SUBSAMPLING_VALUE 4

#define IMAGE_X (TOTAL_IMAGE_WIDTH - IMAGE_WIDTH) / 2 // Capture a zone in the center of the camera
#define IMAGE_Y (TOTAL_IMAGE_HEIGHT - IMAGE_HEIGHT) / 2 // Capture a zone in the center of the camera
#define IMAGE_SUBSAMPLING CONCATENATE(SUBSAMPLING_X, SUBSAMPLING_VALUE)
#define BYTES_PER_PIXEL 2
#define IMAGE_BUFFER_SIZE ((IMAGE_WIDTH / SUBSAMPLING_VALUE) * (IMAGE_HEIGHT / SUBSAMPLING_VALUE) * BYTES_PER_PIXEL)
#define MAX_IMAGE_BUFFER_SIZE (MAX_BUFF_SIZE / 2) // Account for double buffering

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

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
