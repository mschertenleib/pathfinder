#ifndef OBSTACLE_H
#define OBSTACLE_H

#define SAMPLE_FILTER 3
#define IR_THRESHOLD 300
#define TIMEBTWM 100
#define NBMEAS 3
#define FLWSPD 100

void obstacle_detection_init(void);

int spdP(int meas);

#endif // OBSTACLE_H
