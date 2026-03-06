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
     
}

void Angle_To_Pos(Leg *leg)
{
    leg->Param_Exp.HAA_Angle = fminf(fmaxf(leg->Param_Obs.HAA_Angle, -1.57f*leg->HAA_Motor.gear_ratio), 1.57f*leg->HFE_Motor.gear_ratio); 
    leg->Param_Exp.HFE_Angle = fminf(fmaxf(leg->Param_Obs.HFE_Angle, 0.0f), 1.57f*leg->HFE_Motor.gear_ratio);  
    leg->Param_Exp.KFE_Angle = fminf(fmaxf(leg->Param_Obs.KFE_Angle, 0.0f), 3.14f*leg->KFE_Motor.gear_ratio);  
    
    leg->HAA_Motor.send.Pos = leg->Param_Exp.HAA_Angle * leg->HAA_Motor.gear_ratio + leg->HAA_Motor.Init_Pos;
    float HFE_Stretch = leg->HFE_Motor.Init_Pos - leg->HFE_Motor.Init_Angle * leg->HFE_Motor.gear_ratio;
    leg->HFE_Motor.send.Pos = leg->Param_Exp.HFE_Angle * leg->HFE_Motor.gear_ratio + HFE_Stretch;
    float KFE_Stretch = leg->KFE_Motor.Init_Pos - leg->KFE_Motor.Init_Angle * leg->KFE_Motor.gear_ratio;
    leg->KFE_Motor.send.Pos = leg->Param_Exp.KFE_Angle * leg->KFE_Motor.gear_ratio + KFE_Stretch;
}

void Leg_Forward_Kinematics_Pos(Leg *leg)
{
    Pos_To_Angle(leg);
    
    float theta = leg->Param_Obs.HFE_Angle + leg->Param_Obs.KFE_Angle;
    float x_leg = leg->Param_Obs.l_hfe * sinf(leg->Param_Obs.HFE_Angle) + leg->Param_Obs.l_kfe * sinf(theta);
    float z_leg = -leg->Param_Obs.l_hfe * cosf(leg->Param_Obs.HFE_Angle) - leg->Param_Obs.l_kfe * cosf(theta);

    leg->Foot_Obs.FootPos[0] = x_leg;
    leg->Foot_Obs.FootPos[1] = -z_leg * sinf(leg->Param_Obs.HAA_Angle);
    leg->Foot_Obs.FootPos[2] = z_leg * cosf(leg->Param_Obs.HAA_Angle);
    
    if(leg->ID % 2 == 1)
        leg->Foot_Obs.FootPos[1] = -leg->Foot_Obs.FootPos[1];
}

int Leg_Inverse_Kinematics(Leg *leg)
{
    float x = leg->Foot_Exp.FootPos[0];
    float y = leg->Foot_Exp.FootPos[1];
    float z = leg->Foot_Exp.FootPos[2];
    float l1 = leg->Param_Obs.l_hfe;
    float l2 = leg->Param_Obs.l_kfe;

    if(leg->ID % 2 == 1) y = -y;
    leg->Param_Exp.HAA_Angle = atan2f(-y, z);

    float z_leg = z / cosf(leg->Param_Exp.HAA_Angle);
    float r_sq = x * x + z_leg * z_leg;
    float r = sqrtf(r_sq);

    float cos_kfe = (r_sq - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    cos_kfe = fminf(fmaxf(cos_kfe, -1.0f), 1.0f);
    leg->Param_Exp.KFE_Angle = acosf(cos_kfe); 

    float alpha = atan2f(x, -z_leg);
    float cos_beta = (l1 * l1 + r_sq - l2 * l2) / (2.0f * l1 * r);
    cos_beta = fminf(fmaxf(cos_beta, -1.0f), 1.0f);
    float beta = acosf(cos_beta);
    
    leg->Param_Exp.HFE_Angle = alpha + beta;
    Angle_To_Pos(leg);

    return 0; 
}

void Vel_To_Angvel(Leg *leg)
{
    leg->Param_Obs.HAA_Angvel = leg->HAA_Motor.recv.W / leg->HAA_Motor.gear_ratio;
    leg->Param_Obs.HFE_Angvel = leg->HFE_Motor.recv.W / leg->HFE_Motor.gear_ratio;
    leg->Param_Obs.KFE_Angvel = leg->KFE_Motor.recv.W / leg->KFE_Motor.gear_ratio;

}

void Angvel_To_Vel(Leg *leg)
{
    leg->Param_Exp.HAA_Angvel = fminf(fmaxf(leg->Param_Exp.HAA_Angvel, -30.0f), 30.0f);
    leg->Param_Exp.HFE_Angvel = fminf(fmaxf(leg->Param_Exp.HFE_Angvel, -30.0f), 30.0f);
    leg->Param_Exp.KFE_Angvel = fminf(fmaxf(leg->Param_Exp.KFE_Angvel, -30.0f), 30.0f);
    
    leg->HAA_Motor.send.W = leg->Param_Exp.HAA_Angvel / leg->HAA_Motor.gear_ratio;
    leg->HFE_Motor.send.W = leg->Param_Exp.HFE_Angvel / leg->HFE_Motor.gear_ratio;
    leg->KFE_Motor.send.W = leg->Param_Exp.KFE_Angvel / leg->KFE_Motor.gear_ratio;

}

void Jacobi_Compute(Leg *leg, float J[3][3])
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
    
    J[0][0] = 0.f;
    J[1][0] = (l_hfe * cos_hfe + l_kfe * cos_theta) * cos_haa;
    J[2][0] = -(l_hfe * cos_hfe + l_kfe * cos_theta) * sin_haa;
    
    J[0][1] = l_hfe * cos_hfe + l_kfe * cos_theta;
    J[1][1] = -(l_hfe * sin_hfe + l_kfe * sin_theta) * sin_haa;
    J[2][1] = -(l_hfe * sin_hfe + l_kfe * sin_theta) * cos_haa;
    
    J[0][2] = l_kfe * cos_theta;
    J[1][2] = -l_kfe * sin_theta * sin_haa;
    J[2][2] = -l_kfe * sin_theta * cos_haa;
}
void Leg_Forward_Kinematics_Vel(Leg *leg)
{   
    float J[3][3] = {0};
    Jacobi_Compute(leg, J);
    
    float d_haa = leg->Param_Obs.HAA_Angvel;
    float d_hfe = leg->Param_Obs.HFE_Angvel;
    float d_kfe = leg->Param_Obs.KFE_Angvel;
    
    leg->Foot_Obs.FootVel[0] = J[0][0]*d_haa + J[0][1]*d_hfe + J[0][2]*d_kfe; 
    leg->Foot_Obs.FootVel[1] = J[1][0]*d_haa + J[1][1]*d_hfe + J[1][2]*d_kfe; 
    leg->Foot_Obs.FootVel[2] = J[2][0]*d_haa + J[2][1]*d_hfe + J[2][2]*d_kfe; 
    
    if(leg->ID % 2 == 1)
        leg->Foot_Obs.FootVel[1] = -leg->Foot_Obs.FootVel[1];
}

int matrix_invert(float m[3][3], float inv[3][3])
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

/***********************关节力矩与电机力矩映射*****************************/
void Motor_To_Joint_Torque(Leg *leg)
{
    leg->Param_Obs.HAA_Torque = leg->HAA_Motor.recv.T * leg->HAA_Motor.gear_ratio;
    leg->Param_Obs.HFE_Torque = leg->HFE_Motor.recv.T * leg->HFE_Motor.gear_ratio;
    leg->Param_Obs.KFE_Torque = leg->KFE_Motor.recv.T * leg->KFE_Motor.gear_ratio;

}

void Joint_To_Motor_Torque(Leg *leg)
{
    leg->Param_Exp.HAA_Torque = fminf(fmaxf(leg->Param_Exp.HAA_Torque, -10.0f), 10.0f);
    leg->Param_Exp.HFE_Torque = fminf(fmaxf(leg->Param_Exp.HFE_Torque, -10.0f), 10.0f);
    leg->Param_Exp.KFE_Torque = fminf(fmaxf(leg->Param_Exp.KFE_Torque, -10.0f), 10.0f);
    
    leg->HAA_Motor.send.T = leg->Param_Exp.HAA_Torque / leg->HAA_Motor.gear_ratio;
    leg->HFE_Motor.send.T = leg->Param_Exp.HFE_Torque / leg->HFE_Motor.gear_ratio;
    leg->KFE_Motor.send.T = leg->Param_Exp.KFE_Torque / leg->KFE_Motor.gear_ratio;
}

void Foot_Force_Estimate(Leg *leg)
{
    float J[3][3] = {0};
    float JT[3][3] = {0};
    float JT_inv[3][3] = {0};
    
    Jacobi_Compute(leg, J);
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            JT[i][j] = J[j][i];
        }
    }
    
    if(matrix_invert(JT, JT_inv) != 0)
    {
        return;
    }
    
    float tau[3];
    Motor_To_Joint_Torque(leg);
    tau[0] = leg->Param_Obs.HAA_Torque;
    tau[1] = leg->Param_Obs.HFE_Torque;
    tau[2] = leg->Param_Obs.KFE_Torque;
    
    leg->Foot_Obs.FootForce[0] = JT_inv[0][0]*tau[0] + JT_inv[0][1]*tau[1] + JT_inv[0][2]*tau[2];
    leg->Foot_Obs.FootForce[1] = JT_inv[1][0]*tau[0] + JT_inv[1][1]*tau[1] + JT_inv[1][2]*tau[2];
    leg->Foot_Obs.FootForce[2] = JT_inv[2][0]*tau[0] + JT_inv[2][1]*tau[1] + JT_inv[2][2]*tau[2];
    
    if(leg->ID % 2 == 1)
        leg->Foot_Obs.FootForce[1] = -leg->Foot_Obs.FootForce[1];
}                                                                                                                           

void Motor_Torque_Compute(Leg *leg)
{
    float J[3][3] = {0};
    Jacobi_Compute(leg, J);
    
    float fx = leg->Foot_Exp.FootForce[0];
    float fy = leg->Foot_Exp.FootForce[1];
    float fz = leg->Foot_Exp.FootForce[2];
    
    if(leg->ID % 2 == 1)
        fy = -fy;
    
    leg->Param_Exp.HAA_Torque = J[0][0]*fx + J[1][0]*fy + J[2][0]*fz;
    leg->Param_Exp.HFE_Torque = J[0][1]*fx + J[1][1]*fy + J[2][1]*fz;
    leg->Param_Exp.KFE_Torque = J[0][2]*fx + J[1][2]*fy + J[2][2]*fz;
    
    Joint_To_Motor_Torque(leg);
}


