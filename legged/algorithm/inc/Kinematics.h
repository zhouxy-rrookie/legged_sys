#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "motor.h"

typedef struct 
{
    float FootPos[3];
    float FootVel[3];
    float FootForce[3];
}Leg_Foot;

typedef enum {
    LEG_FL = 0,  
    LEG_FR = 1,  
    LEG_RL = 2,  
    LEG_RR = 3   
} LegID;

typedef struct
{
    const float l_haa;
    const float l_hfe;
    const float l_kfe;
    const float Stand_Foot_Pos[3];

    float HAA_Angle;
    float HFE_Angle;
    float KFE_Angle;
    
    float HAA_Angvel;
    float HFE_Angvel;
    float KFE_Angvel;
}Leg_Param;

typedef struct
{
    LegID ID;
    
    MotorParam HAA_Motor;
    MotorParam HFE_Motor;
    MotorParam KFE_Motor;
    
    Leg_Foot Foot_Exp;
    Leg_Foot Foot_Obs;

    Leg_Param Param_Exp;
    Leg_Param Param_Obs;
}Leg;




#endif