#ifndef MOVE_H
#define MOVE_H

#define NB_MOVES 3
// External functions

void ReceiveSpeedInstMove(BaseSequentialStream* in, uint16_t size);

void RunSpeedInstSequence(void);

void lauch_move_thd(void);

// Internal functions

#endif /* MOVE_H */
