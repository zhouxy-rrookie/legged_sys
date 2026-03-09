#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include "Kinematics.h"


typedef struct {
    float x;
    float y;
    float z;
}Foot_Pos;

typedef struct {
    float p0[3]; 
    float p1[3]; 
    float p2[3]; 
    float p3[3]; 
} BezierCurve_t;

#endif