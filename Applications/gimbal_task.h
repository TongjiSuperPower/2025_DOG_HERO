#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

#include "struct_typedef.h"
#include "pid.h"

#define RADUCTION_RATIO_1      //底盘减速比：1为19.2 2为13.7

#define KEY_MOD_1               //1为遥控器键鼠，2为图传链路键鼠

//ENCODE控制模式
//yaw轴角度双环外环
#define GimbalMotor4310PosPid1_Yaw_kp 12.0f
#define GimbalMotor4310PosPid1_Yaw_ki 0.0f
#define GimbalMotor4310PosPid1_Yaw_kd 5.0f
//yaw轴角度双环内环
#define GimbalMotor4310PosPid2_Yaw_kp 0.5f
#define GimbalMotor4310PosPid2_Yaw_ki 0.0f
#define GimbalMotor4310PosPid2_Yaw_kd 0.0f
//pitch轴角度双环外环
#define GimbalMotormiPosPid1_Pit_kp 2.0f
#define GimbalMotormiPosPid1_Pit_ki 0.005f
#define GimbalMotormiPosPid1_Pit_kd 0.0f
//pitch轴角度双环内环
#define GimbalMotormiPosPid2_Pit_kp 2.2f
#define GimbalMotormiPosPid2_Pit_ki 0.0f
#define GimbalMotormiPosPid2_Pit_kd 0.0f
//PID输出限制
#define GimbalMotor6020MinOut -10000.0f
#define GimbalMotor6020MaxOut 10000.0f
#define GimbalMotor6020MinIOut -5000.0f
#define GimbalMotor6020MaxIOut 5000.0f

//GYRO控制模式
//Yaw陀螺仪双环外环pid参数-达妙
#define GimbalPid1Yaw_kp 16.0f//12.0f//30.0f
#define GimbalPid1Yaw_ki 0.05f//0.02f//0.1f
#define GimbalPid1Yaw_kd 10.0f//30.0f

#define GimbalPid2Yaw_kp 4.0f//6.0f//1.8f
#define GimbalPid2Yaw_ki 0.0f
#define GimbalPid2Yaw_kd 0.0f

//Pitch陀螺仪双环外环pid参数-小米
#define GimbalPid1Pitch_kp 25.0f//45.0f
#define GimbalPid1Pitch_ki 0.2f//0.02f
#define GimbalPid1Pitch_kd 0.0f//50.0f

#define GimbalPid2Pitch_kp 3.0f
#define GimbalPid2Pitch_ki 0.0f
#define GimbalPid2Pitch_kd 0.0f

//自瞄控制模式
//自瞄Yaw陀螺仪双环外环pid参数-达妙
#define GimbalPid1YawAutoaim_kp 16.0f//30.0f
#define GimbalPid1YawAutoaim_ki 0.05f//0.1f
#define GimbalPid1YawAutoaim_kd 10.0f//30.0f

#define GimbalPid2YawAutoaim_kp 4.0f//1.8f
#define GimbalPid2YawAutoaim_ki 0.0f
#define GimbalPid2YawAutoaim_kd 0.0f

//自瞄Pitch陀螺仪双环外环pid参数-小米
#define GimbalPid1PitchAutoaim_kp 25.0f//45.0f
#define GimbalPid1PitchAutoaim_ki 0.2f//0.02f
#define GimbalPid1PitchAutoaim_kd 0.0f

#define GimbalPid2PitchAutoaim_kp 3.0f//1.8f
#define GimbalPid2PitchAutoaim_ki 0.0f
#define GimbalPid2PitchAutoaim_kd 0.0f

//达妙外环最大输出
#define GimbalPid1YawMinOut -10.0f
#define GimbalPid1YawMaxOut 10.0f
#define GimbalPid1YawMinIOut -0.5f
#define GimbalPid1YawMaxIOut 0.5f

//达妙内环最大输出
#define GimbalPid2YawMinOut -10.0f
#define GimbalPid2YawMaxOut 10.0f
#define GimbalPid2YawMinIOut -3.0f
#define GimbalPid2YawMaxIOut 3.0f

//小米电机外环最大输出
#define GimbalPid1PitMinOut -12.0f
#define GimbalPid1PitMaxOut 12.0f
#define GimbalPid1PitMinIOut -0.4f
#define GimbalPid1PitMaxIOut 0.4f


//小米内环最大输出
#define GimbalPid2PitMinOut -12.0f
#define GimbalPid2PitMaxOut 12.0f
#define GimbalPid2PitMinIOut -4.0f
#define GimbalPid2PitMaxIOut 4.0f
	
//遥控器模式云台数据
#define WMax 0.004f                     //rad/ms
#define PitAngleMax 0.53//0.32f                //Pitch轴限位    最大角度
#define PitAngleMin -0.28//-0.62f               //Pitch轴限位    最小角度
#define OffsetAngle 0.06142f  //小米电机相对角度 relative angle = -陀螺仪欧拉角INS_eulers[1] - offset


//键鼠模式鼠标系数数据
#define WCoef 0.00006f                  //联盟赛0.00006f
#define low_W 0.0003f                   //QE微调速度

//掉头冷却时间
#define TurnOverColdTime 800            //ms

//重力补偿力矩
#define Tor_param 1.4f
//重心偏移角
#define OFFSET_ANGLE -0.05f               //rad

//云台状态机参数
typedef enum{

    GIMBAL_ZERO_FORCE,
    GIMBAL_INIT,
    GIMBAL_GYRO,
    GIMBAL_ENCONDE,
    GIMBAL_AUTO,
    GIMBAL_PASS,

}gimbal_behaviour_e;

//云台电机控制模式
typedef enum{

    GIMBAL_Motor_DOWN,    // 电机电流发零
	GIMBAL_MOTOR_GYRO,	  // 电机陀螺仪角度控制
	GIMBAL_MOTOR_ENCONDE, // 电机编码值角度控制
    GIMBAL_MOTOR_PASS,

} gimbal_motor_mode_e;

typedef struct{
	
    gimbal_behaviour_e gimbal_behaviour;
    gimbal_behaviour_e last_gimbal_behaviour;

    gimbal_motor_mode_e gimbal_motor_yaw_mode;
    gimbal_motor_mode_e gimbal_motor_pit_mode;

    pid_type_def GimbalIMUYawPid1;                       //Yaw PID控制陀螺仪外环数据
    fp32 f_GimbalYawPidMid;
    pid_type_def GimbalIMUYawPid2;                       //Yaw PID控制陀螺仪内环数据

    pid_type_def GimbalIMUPitPid1;                       //Pit PID控制陀螺仪外环数据
    fp32 f_GimbalPitPidMid;
    pid_type_def GimbalIMUPitPid2;                       //Pit PID控制陀螺仪内环数据

    pid_type_def GimbalIMUYawAutoaimPid1;                //Yaw 自瞄PID控制陀螺仪外环数据
    fp32 f_GimbalYawAutoaimPidMid;
    pid_type_def GimbalIMUYawAutoaimPid2;                //Yaw 自瞄PID控制陀螺仪内环数据

    pid_type_def GimbalIMUPitAutoaimPid1;                //Pit 自瞄PID控制陀螺仪外环数据
    fp32 f_GimbalPitAutoaimPidMid;
    pid_type_def GimbalIMUPitAutoaimPid2;                //Pit 自瞄PID控制陀螺仪内环数据

    fp32 gyro_yaw_angle_add;                             //Yaw轴控制角度增量     rad
    fp32 gyro_pit_angle_add;                             //Pitch轴控制角度增量   rad

    fp32 gyro_yaw_target_angle;
    fp32 gyro_pit_target_angle;


}gimbal_data_t;

extern gimbal_data_t gimbal_data;

//云台数据初始化
void fn_GimbalInit(void);

//云台电机初始化函数
void fn_GimbalMotorInit(void);

//云台状态选择
void fn_GimbalMode(void);

//电机模式选择
void fn_MotorMode(void);

//云台电流解算
void fn_GimbalMove(void);


#endif
