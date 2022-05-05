#include "ch.h"
#include "hal.h"
#include <main.h>

#include <communications.h>
#include <motors.h>
#include <Bezier.h>

void run_path(path way)
{
    //start Thread
    motors_init()
    int time_step_ms = way.time / PATHPRES;

    for(int i = 0, i < PATHPRES/2, i++){

        left_motor_set_speed(way.spd_instr[i][0]);
        right_motor_set_speed(way.spd_instr[i][1]);
        //chThdSleep(time_step_ms)
    }

    //stop Thread
}

path create_bezier_path(point start,point start_dir,point end, point end_dir, uint_8 runtime)
{
    path created_path;
    created_path.start_p = start;
    created_path.end_p = end;
    created_path.time = runtime;

    point[] bez_pts = {start,start_dir,end_dir,end};

    for(int i = 0 ; i < PATHPRES ; i++){
        float x = i/(PATHPRES-1);
        created_path.line[i] = recursive_bezier(bez_pts,4,x);
    }

    //TODO : arc interpolation , distance to speed 
    for(int i = 0, uint8_t j = 0 ; i < PATHPRES-2 ; i += 2 , j++ ){
        point center = circle_3pts(created_path.line[i],created_path.line[i+1],created_path.line[i+2]);

        if(center == created_path.line[i]){
            created_path.spd_instr[j]
        }else{

        }
    
    }

    //normalise to speed

    return created_path;
}

point recursive_bezier(point[] points,int size,float t)
{
    if(size > 1){
        point[size-1] new_points;
        for(int i = 0; i < len(points)-1; i++){
            float px = points[i][0] + t*(points[i+1][0]-points[i][0]);
            float py = points[i][1] + t*(points[i+1][1]-points[i][1]);
            new_points[i] = {px,py};
        }
        return recursive_bezier(new_points,size-1,t)
    }
    else{
        return points[0]
    }
}
