#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

void SendFloatToComputer(BaseSequentialStream* out, float data);

void SendUint16ToComputer(BaseSequentialStream* out, uint16_t data);

void SendUint8ToComputer(uint8_t* data, uint16_t size);

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in);

float ReceiveFloatFromComputer(BaseSequentialStream* in);

#endif /* COMMUNICATIONS_H */
