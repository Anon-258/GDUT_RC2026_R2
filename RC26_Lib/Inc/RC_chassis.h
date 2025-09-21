#pragma once
#include "RC_pid.h"
#include "RC_motor.h"
#include "RC_m3508.h"
#include "RC_can.h"
#include "RC_tim.h"
#include "RC_m3508.h"
#include <math.h>
#ifdef __cplusplus

namespace chassis
{
    class RC_Chassis
    {
    public:
        RC_Chassis();
        void chassis_init();

        //设置世界坐标系下的目标位置
          void Set_Target_pos(float x, float y, float yaw);
        //设置世界坐标系下的目标速度
        void Set_Target_spd(float vx, float vy, float vyaw);

        // 更新底盘控制
        void Update(void);

        // 目标位置和速度
        float target_x = 0, target_y = 0, target_yaw = 0;
        float target_vx = 0, target_vy = 0, target_vyaw = 0;

    private:
        m3508::M3508* motors[4]; // 四个电机指针
        pid::Pid pid_x, pid_y, pid_yaw; // 底盘整体PID
    };
}

#endif

