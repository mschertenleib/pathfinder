#ifndef ODOM_H
#define ODOM_H

#define TIMERES 118 //time between mesures
#define CMPSTEP (13.0f/1000.0f) //cm per step
#define RBTWIDTHCM 5.37f //robot width in cm
#define WHLDIAMCM 7.38
#define WHLCIRCCM M_PI*WHLDIAMCM



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

float compute_rad(short l, short r);

#endif /* ODOM_H */
