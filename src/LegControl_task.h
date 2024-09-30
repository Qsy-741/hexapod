#ifndef LEGCONTROL_TASK_H
#define LEGCONTROL_TASK_H

#include "my_math.h"
#include "gait_prg.h"
#define leftmotor1 25
#define leftmotor2 23
#define leftmotord1 7
#define leftmotord2 8
#define rightmotor1 27
#define rightmotor2 26
#define rightmotord1 5
#define rightmotord2 6//m1m2

#define LEG_JOINT2_OFFSET 0//PI / 2
#define LEG_JOINT3_OFFSET 0//-2 * PI / 9

#define HEXAPOD_MIN_HEIGHT -70.0f
#define HEXAPOD_MAX_HEIGHT 70.0f
#define HEXAPOD_MIN_X -80.0f
#define HEXAPOD_MAX_X 80.0f
#define HEXAPOD_MIN_Y -80.0f
#define HEXAPOD_MAX_Y 80.0f


#define HEXAPOD_MIN_X_ROTATE -15.0f / 180 * PI // 绕X轴旋转角度最小为-15度
#define HEXAPOD_MAX_X_ROTATE 15.0f / 180 * PI  // 绕X轴旋转角度最大为 15度
#define HEXAPOD_MIN_Y_ROTATE -15.0f / 180 * PI // 绕y轴旋转角度最小为-10度
#define HEXAPOD_MAX_Y_ROTATE 15.0f / 180 * PI  // 绕y轴旋转角度最大为 10度
#define HEXAPOD_MIN_Z_ROTATE -25.0f / 180 * PI // 绕z轴旋转角度最小为-25度
#define HEXAPOD_MAX_Z_ROTATE 25.0f / 180 * PI  // 绕z轴旋转角度最大为 25度

/*PID*/
#define MPU_X_PID_KP 0.015f
#define MPU_X_PID_KI 0.0f
#define MPU_X_PID_KD 0.5f

#define MPU_Y_PID_KP 0.015f
#define MPU_Y_PID_KI 0.0f
#define MPU_Y_PID_KD 0.5f


/*FOF一阶低通滤波参数*/
#define VELOCITY_FOF_K 0.5f
#define BODY_POS_FOF_K 0.5f
#define BODY_ANGLE_FOF_K 0.5f


#define ROTATE_BODY_ANGLE_SENSI 0.002f//控制角度灵敏度
#define ROTATE_BODY_POS_SENSI 0.03f//控制位置灵敏度


typedef enum
{
    HEXAPOD_MOVE,
    HEXAPOD_BODY_ANGEL_CONTROL,
    HEXAPOD_BODY_POS_CONTROL,
} Hexapod_mode_e;

typedef enum
{
    MPU_ON,
    MPU_OFF,
}MPU_SW_e;



class Hexapod
{
public:
    Velocity velocity;           // 机器人速度
    Hexapod_mode_e mode; // 机器人模式
    MPU_SW_e mpu_sw;    //是否由陀螺仪控制
    Position3 body_pos;     //机体位置
    Position3 body_angle;   //机体角度
    Position3 mpu_angle;
    Position3 mpu_angle_set;
    PID mpu_pid_x; //x轴pid
    PID mpu_pid_y; //y轴pid
    PID mpu_pid_z; //y轴pid
    First_order_filter velocity_fof[3];
    First_order_filter body_pos_fof[3];
    First_order_filter body_angle_fof[3];
    bool mpu_flag;
    void Init();          
    void velocity_cal();
    void body_position_cal();
    void body_angle_cal();
    void body_position_cal(int xValue, int yValue, int zValue);
    void velocity_cal(int RXValue, int RYValue, int LXValue);
    void body_angle_cal(int xValue, int yValue, int zValue);
    void mode_select();
    void body_angle_and_pos_zero();
    void move(uint32_t round_time);
    void remote_deal();
    void modeSelect();
};


#endif