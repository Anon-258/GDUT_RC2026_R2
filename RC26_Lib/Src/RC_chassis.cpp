#include "RC_chassis.h"


namespace chassis
{
		extern CAN_HandleTypeDef hcan1;
		extern TIM_HandleTypeDef htim2;
     can::Can can1(hcan1);
     tim::Tim tim2(htim2);
	
    // 初始化4个M3508电机对象
    m3508::M3508 motor1(1, can1, tim2);
    m3508::M3508 motor2(2, can1, tim2);
    m3508::M3508 motor3(3, can1, tim2);
    m3508::M3508 motor4(4, can1, tim2);

    // 运动学参数
    const float WHEEL_RADIUS = 0.152f/2; // 轮子半径，单位m
    const float CHASSIS_D = 0.64;     // 轮子到中心距离，单位m

    // 最大加速度（单位 m/s^2 和 rad/s^2）
    constexpr float MAX_ACC_X = 2.0f;
    constexpr float MAX_ACC_Y = 2.0f;
    constexpr float MAX_ACC_YAW = 1.0f;
   
    // 余弦值
    float COS45=cos(PI/4),SIN45=sin(PI/4);
    float COS30=cos(PI/6),SIN30=sin(PI/6);
    float COS60=cos(PI/3),SIN60=sin(PI/3);

    // 上一周期速度
    static float last_vx = 0, last_vy = 0, last_vyaw = 0;


    RC_Chassis::RC_Chassis()
    {
        motors[0] = &motor1;
        motors[1] = &motor2;
        motors[2] = &motor3;
        motors[3] = &motor4;
    }

    void RC_Chassis::chassis_init()
    {
        //底盘的pid参数
        pid_x.Pid_Mode_Init(false, false, 0);
        pid_y.Pid_Mode_Init(false, false, 0);
        pid_yaw.Pid_Mode_Init(false, false, 0);

        pid_x.Pid_Param_Init(2, 0, 0, 0, 0.001, 0, 1000, 1000, 0, 0, 0);
        pid_y.Pid_Param_Init(2, 0, 0, 0, 0.001, 0, 1000, 1000, 0, 0, 0);
        pid_yaw.Pid_Param_Init(2, 0, 0, 0, 0.001, 0, 1000, 1000, 0, 0, 0);
    }
/**
    void RC_Chassis::Set_Target_pos(float x, float y, float yaw)
    {
        target_x = x;
        target_y = y;
        target_yaw = yaw;
    }
*/
    void RC_Chassis::Set_Target_spd(float vx, float vy, float vyaw)
    {
        target_vx = vx;
        target_vy = vy;
        target_vyaw = vyaw;
    }

    void RC_Chassis::Update(void)
    {
        float dt = 0.001f; // 控制周期
        
        float vx = target_vx;
        float vy = target_vy;
        float vyaw = target_vyaw;

        // 加速度限制
        float acc_x = (vx - last_vx) / dt;
        float acc_y = (vy - last_vy) / dt;
        float acc_yaw = (vyaw - last_vyaw) / dt;

        if(acc_x > MAX_ACC_X)           vx = last_vx + MAX_ACC_X * dt;
        if(acc_x < -MAX_ACC_X)          vx = last_vx - MAX_ACC_X * dt;
        if(acc_y > MAX_ACC_Y)           vy = last_vy + MAX_ACC_Y * dt;
        if(acc_y < -MAX_ACC_Y)          vy = last_vy - MAX_ACC_Y * dt;
        if(acc_yaw > MAX_ACC_YAW)       vyaw = last_vyaw + MAX_ACC_YAW * dt;
        if(acc_yaw < -MAX_ACC_YAW)      vyaw = last_vyaw - MAX_ACC_YAW * dt;

        last_vx = vx;
        last_vy = vy;
        last_vyaw = vyaw;

        // 全向轮逆解
        const float ratio = 19.0f; // 减速比        
        float k = 60.0f * ratio/ (2 * PI * WHEEL_RADIUS); 
        float L = CHASSIS_D;
        float wheel_rpm[4];

        // 左前、右前、右后、左后
        wheel_rpm[0] = k * (vx * COS45  - vy * COS45  + 2 * L * vyaw);
        wheel_rpm[1] = k * (-vx * COS45 - vy * COS45  + 2 * L * vyaw);
        wheel_rpm[2] = k * (-vx * COS45 + vy * COS45  + 2 * L * vyaw);
        wheel_rpm[3] = k * (vx * COS45  + vy * COS45  + 2 * L * vyaw);

        // 6. 设置电机目标
        for(int i=0; i<4; i++)
        {
            motors[i]->Set_Rpm(wheel_rpm[i]);
        }
    }
}