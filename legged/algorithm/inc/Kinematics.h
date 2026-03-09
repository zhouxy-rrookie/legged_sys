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
    
    float HAA_Torque;
    float HFE_Torque;
    float KFE_Torque;
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


extern Leg FL_Leg, FR_Leg, RL_Leg, RR_Leg;

void Pos_To_Angle(Leg *leg);
void Angle_To_Pos(Leg *leg);
void Leg_Forward_Kinematics_Pos(Leg *leg);
int Leg_Inverse_Kinematics(Leg *leg);
void Vel_To_Angvel(Leg *leg);
void Angvel_To_Vel(Leg *leg);
void Jacobi_Compute(Leg *leg, float J[3][3]);
void Leg_Forward_Kinematics_Vel(Leg *leg);
int matrix_invert(float m[3][3], float inv[3][3]);
void Motor_To_Joint_Torque(Leg *leg);
void Joint_To_Motor_Torque(Leg *leg);
void Foot_Force_Estimate(Leg *leg);
void Motor_Torque_Compute(Leg *leg);

#endif