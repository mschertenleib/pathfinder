#ifndef MOVE_H
#define MOVE_H

#define MAX_MOVES 100
#define secscan 6
// External functions

void ReceiveSpeedInstMove(BaseSequentialStream* in,BaseSequentialStream* out);

void stop(BaseSequentialStream* out);

void scan(BaseSequentialStream* in ,BaseSequentialStream* out);

void lauch_move_thd(void);

// Internal functions

#endif /* MOVE_H */
