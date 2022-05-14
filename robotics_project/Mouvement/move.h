/*
EPFL MICRO-301 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
may 2022
*/

#ifndef MOVE_H
#define MOVE_H

#define MAX_MOVES 100

void ReceiveSpeedInstMove(BaseSequentialStream* in,BaseSequentialStream* out);

void stop(void);

void sequence_override(void);

void scan(BaseSequentialStream* out);

void lauch_move_thd(void);


#endif /* MOVE_H */
