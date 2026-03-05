#include "Kinematics.h"
#include "math.h"

Leg FL_Leg, FR_Leg, RL_Leg, RR_Leg;


void Leg_Forward_Kinematics(Leg *leg)
{
    float HAA_Pos = leg->HAA_Motor.recv.Pos - leg->HAA_Motor.Stand_Pos;
    float HFE_Pos = leg->HFE_Motor.recv.Pos - leg->HFE_Motor.Stand_Pos;
    float KFE_Pos = leg->KFE_Motor.recv.Pos - leg->KFE_Motor.Stand_Pos;
    
    float x_stand = leg->Param.l_hfe * sinf(leg->HFE_Motor.Stand_Pos) + leg->Param.l_kfe * sinf(leg->KFE_Motor.Stand_Pos);
    float y_stand = (leg->ID % 2 == 0) ? leg->Param.l_haa : -leg->Param.l_haa;
    float z_stand = -leg->Param.l_hfe * cosf(leg->HFE_Motor.Stand_Pos) - leg->Param.l_kfe * cosf(leg->KFE_Motor.Stand_Pos);
    
    
}