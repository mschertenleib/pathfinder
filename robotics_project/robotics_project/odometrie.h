/*
EPFL MICRO-301 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
may 2022
*/

#ifndef ODOM_H
#define ODOM_H

#define TIMERES 118                 //time between mesures
#define CMPSTEP (13.0f/1000.0f)     //cm per step
#define RBTWIDTHCM 5.37f            //robot width in cm
#define WHLDIAMCM 7.38              //Diameter of the wheels in cm
#define WHLCIRCCM M_PI*WHLDIAMCM    //Circonference of the wheels in cm

// starts odometrie thread.
void lauch_odometrie_thd(void);

// get functions
float get_angle(void);
float get_posx(void);
float get_posy(void);

//set function
void set_pos(float x, float y, float phi);

// internal funtions 
void measure_and_add(float* pos, int mlc, int mrc);

float compute_phi(short l, short r);

float compute_dist(short l, short r, float ang);

#endif /* ODOM_H */
