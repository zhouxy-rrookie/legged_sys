#include "Kinematics.h"
#include "math.h"

Leg FL_Leg, FR_Leg, RL_Leg, RR_Leg;

//注：运动学解算的坐标，均是以每条腿的髋关节作为坐标原点，x轴向机身前方，y轴向左，z轴向上
//角度定义：髋侧摆，水平为零点，左摆为正；大腿竖直为零点，后摆为正；小腿与大腿共线为零点，后摆为正。


//将电机位置映射到关节角度
//只需要根据不同的机械结构修改此函数的映射关系
void Pos_To_Angle(Leg *leg)
{
    leg->HAA_Motor.Stretch_Pos = leg->HAA_Motor.Init_Pos;
    leg->Param_Obs.HAA_Angle = (leg->HAA_Motor.recv.Pos - leg->HAA_Motor.Stretch_Pos) / leg->HAA_Motor.gear_ratio;

    leg->HFE_Motor.Stretch_Pos = leg->HFE_Motor.Init_Pos - leg->HFE_Motor.Init_Angle * leg->HFE_Motor.gear_ratio; 
    leg->Param_Obs.HFE_Angle = (leg->HFE_Motor.recv.Pos - leg->HFE_Motor.Stretch_Pos) / leg->HFE_Motor.gear_ratio;

    leg->KFE_Motor.Stretch_Pos = leg->KFE_Motor.Init_Pos - leg->KFE_Motor.Init_Angle * leg->KFE_Motor.gear_ratio;
    leg->Param_Obs.KFE_Angle = (leg->KFE_Motor.recv.Pos - leg->KFE_Motor.Stretch_Pos) / leg->KFE_Motor.gear_ratio;
    
    leg->Param_Obs.HAA_Angle = fminf(fmaxf(leg->Param_Obs.HAA_Angle, -1.57f), 1.57f); 
    leg->Param_Obs.HFE_Angle = fminf(fmaxf(leg->Param_Obs.HFE_Angle, 0.0f), 1.57f);  
    leg->Param_Obs.KFE_Angle = fminf(fmaxf(leg->Param_Obs.KFE_Angle, 0.0f), 3.14f);   
}

void Leg_Forward_Kinematics_Pos(Leg *leg)
{
    float theta = leg->Param_Obs.HFE_Angle + leg->Param_Obs.KFE_Angle;
    float x_leg = leg->Param_Obs.l_hfe * sinf(leg->Param_Obs.HFE_Angle) + leg->Param_Obs.l_kfe * sinf(theta);
    float z_leg = -leg->Param_Obs.l_hfe * cosf(leg->Param_Obs.HFE_Angle) - leg->Param_Obs.l_kfe * cosf(theta);

    leg->Foot_Obs.FootPos[0] = x_leg;
    leg->Foot_Obs.FootPos[1] = -z_leg * sinf(leg->Param_Obs.HAA_Angle);
    leg->Foot_Obs.FootPos[2] = z_leg * cosf(leg->Param_Obs.HAA_Angle);
    
    if(leg->ID % 2 == 1)
        leg->Foot_Obs.FootPos[1] = -leg->Foot_Obs.FootPos[1];
}

void Vel_To_Angvel(Leg *leg)
{
    leg->Param_Obs.HAA_Angvel = leg->HAA_Motor.recv.W / leg->HAA_Motor.gear_ratio;
    leg->Param_Obs.HFE_Angvel = leg->HFE_Motor.recv.W / leg->HFE_Motor.gear_ratio;
    leg->Param_Obs.KFE_Angvel = leg->KFE_Motor.recv.W / leg->KFE_Motor.gear_ratio;

    leg->Param_Obs.HAA_Angvel = fminf(fmaxf(leg->Param_Obs.HAA_Angvel, -5.0f), 5.0f);
    leg->Param_Obs.HFE_Angvel = fminf(fmaxf(leg->Param_Obs.HFE_Angvel, -5.0f), 5.0f);
    leg->Param_Obs.KFE_Angvel = fminf(fmaxf(leg->Param_Obs.KFE_Angvel, -5.0f), 5.0f);
}

void Leg_Forward_Kinematics_Vel(Leg *leg)
{
    float haa = leg->Param_Obs.HAA_Angle;
    float hfe = leg->Param_Obs.HFE_Angle;
    float kfe = leg->Param_Obs.KFE_Angle;
    float theta = hfe + kfe;
    float l_hfe = leg->Param_Obs.l_hfe;
    float l_kfe = leg->Param_Obs.l_kfe;
    
    float sin_haa = sinf(haa);
    float cos_haa = cosf(haa);
    float sin_hfe = sinf(hfe);
    float cos_hfe = cosf(hfe);
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    
    float J[3][3] = {0};
    
    J[0][0] = 0.f;
    J[1][0] = (l_hfe * cos_hfe + l_kfe * cos_theta) * cos_haa;
    J[2][0] = -(l_hfe * cos_hfe + l_kfe * cos_theta) * sin_haa;
    
    J[0][1] = l_hfe * cos_hfe + l_kfe * cos_theta;
    J[1][1] = -(l_hfe * sin_hfe + l_kfe * sin_theta) * sin_haa;
    J[2][1] = -(l_hfe * sin_hfe + l_kfe * sin_theta) * cos_haa;
    
    J[0][2] = l_kfe * cos_theta;
    J[1][2] = -l_kfe * sin_theta * sin_haa;
    J[2][2] = -l_kfe * sin_theta * cos_haa;
    
    float d_haa = leg->Param_Obs.HAA_Angvel;
    float d_hfe = leg->Param_Obs.HFE_Angvel;
    float d_kfe = leg->Param_Obs.KFE_Angvel;
    
    leg->Foot_Obs.FootVel[0] = J[0][0]*d_haa + J[0][1]*d_hfe + J[0][2]*d_kfe; 
    leg->Foot_Obs.FootVel[1] = J[1][0]*d_haa + J[1][1]*d_hfe + J[1][2]*d_kfe; 
    leg->Foot_Obs.FootVel[2] = J[2][0]*d_haa + J[2][1]*d_hfe + J[2][2]*d_kfe; 
}

int matrix_invert_3x3(float m[3][3], float inv[3][3])
{
    float det = m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
              - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
              + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);

    if (fabs(det) < 1e-6) return 1;
    float inv_det = 1.0f / det;
    inv[0][0] = (m[1][1]*m[2][2] - m[1][2]*m[2][1]) * inv_det;
    inv[0][1] = (m[0][2]*m[2][1] - m[0][1]*m[2][2]) * inv_det;
    inv[0][2] = (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * inv_det;

    inv[1][0] = (m[1][2]*m[2][0] - m[1][0]*m[2][2]) * inv_det;
    inv[1][1] = (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * inv_det;
    inv[1][2] = (m[0][2]*m[1][0] - m[0][0]*m[1][2]) * inv_det;

    inv[2][0] = (m[1][0]*m[2][1] - m[1][1]*m[2][0]) * inv_det;
    inv[2][1] = (m[0][1]*m[2][0] - m[0][0]*m[2][1]) * inv_det;
    inv[2][2] = (m[0][0]*m[1][1] - m[0][1]*m[1][0]) * inv_det;

    return 0;
}