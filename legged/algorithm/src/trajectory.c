#include "trajectory.h"

void Set_Foot_Pos(Leg *leg, Foot_Pos pos)
{
    leg->Foot_Exp.FootPos[0] = pos.x;
    leg->Foot_Exp.FootPos[1] = pos.y;
    leg->Foot_Exp.FootPos[2] = pos.z;

    if (Leg_Inverse_Kinematics(leg) != 0) {
        return; 
    }

    Angle_To_Pos(leg);
}

void Get_Foot_Pos(Leg *leg, Foot_Pos pos)
{
    Pos_To_Angle(leg);
    
    pos.x = leg->Foot_Obs.FootPos[0];
    pos.y = leg->Foot_Obs.FootPos[1];
    pos.z = leg->Foot_Obs.FootPos[2];
}

void Draw_Foot_Line(Leg *leg, Foot_Pos *cur_pos, Foot_Pos *start_pos, Foot_Pos *des_pos, float during_tim_ms)
{
    
}