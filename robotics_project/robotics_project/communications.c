#include "ch.h"
#include "hal.h"
#include <main.h>

#include <communications.h>

/*
*	Sends floats numbers to the computer
*/
void SendFloatToComputer(BaseSequentialStream* out, float data){
	chSequentialStreamWrite(out, (uint8_t*)&data, sizeof(float));
}

void SendUint16ToComputer(BaseSequentialStream* out, uint16_t data){
	chSequentialStreamWrite(out, (uint8_t*)&data, sizeof(uint16_t));
}


void SendUint8ToComputer(uint8_t* data, uint16_t size) {
	chSequentialStreamWrite((BaseSequentialStream * )&SD3, (uint8_t* )&size,
			sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream * )&SD3, (uint8_t* )data,
			size);
}

float ReceiveFloatFromComputer(BaseSequentialStream* in){
	float f = 0;
	chSequentialStreamRead(in,(uint8_t*)&f,sizeof(float));
	return f;
}

uint16_t ReceiveUint16FromComputer(BaseSequentialStream* in){
	uint16_t i = 0;
	chSequentialStreamRead(in,(uint8_t*)&i,sizeof(uint16_t));
	return i;
}
