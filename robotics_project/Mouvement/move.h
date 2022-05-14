#ifndef MOVE_H
#define MOVE_H

#define MAX_MOVES 100
// External functions

void ReceiveSpeedInstMove(BaseSequentialStream* in,BaseSequentialStream* out);

void stop();

void sequence_override(void);

void scan(BaseSequentialStream* out);

void lauch_move_thd(void);

// Internal functions

#endif /* MOVE_H */
