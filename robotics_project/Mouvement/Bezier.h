#ifndef BEZIER_H
#define BEZIER_H

#define PATHPRES 20

typedef float[2] point;

typedef struct path{
    int[PATHPRES/2][2] spd_instr;     //speed for each motor for each step
    point[PATHPRES] line;
    uint_8 time;                //time for the path
    point start_p;
    point end_p;
};

//############# PUBLIC FUNCTIONS ####################################

path create_bezier_path(point start,point start_dir,point end, point end_dir,uint_8 runtime);

void run_path(path way);

//############# INTERNAL FUNCTIONS ####################################

point recursive_bezier(point[] points,int size, float t);
point circle_3pts(point p1, point p2, point p3);
float dist_2pts(point p1, point p2);
int len();

#endif /* COMMUNICATIONS_H */