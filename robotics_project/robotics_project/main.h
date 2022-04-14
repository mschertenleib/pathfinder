#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define SUBSAMPLING 4
#define IMAGE_WIDTH 640 // Needs to be a multiple of SUBSAMPLING
#define IMAGE_HEIGHT 120 // Needs to be a multiple of SUBSAMPLING
#define BYTES_PER_PIXEL 2
#define IMAGE_BUFFER_SIZE ((IMAGE_WIDTH / SUBSAMPLING) * (IMAGE_HEIGHT / SUBSAMPLING) * BYTES_PER_PIXEL)
#define MAX_IMAGE_BUFFER_SIZE (MAX_BUFF_SIZE / 2) // Account for double buffering

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
