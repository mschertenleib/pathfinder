#ifndef ODOM_H
#define ODOM_H

#define TIMERES 20 //time between mesures
#define CMPSTEP (13.0f/1000.0f) //cm per step
#define RBTWIDTHCM 5.389f //robot width in cm

// library functions

void lauch_odometrie_thd(void);

float get_angle(void);
float get_posx(void);
float get_posy(void);

void set_pos(float x, float y, float phi);

// internal funtions 

void measure_and_add(float* pos, int mlc, int mrc);

short pgdc(short a,short b);

float compute_phi(short l, short r);

float compute_dist(short l, short r, float ang);

#endif /* ODOM_H */
