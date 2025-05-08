#ifndef _CALIBRATE_TASK_H_
#define _CALIBRATE_TASK_H_

#include "struct_typedef.h"
#include "gimbal_task.h"


#define GyroXZero 0.00989371073f
#define GyroYZero 0.0044168164f
#define GyroZZero 0.0040072063f


extern uint8_t calibrate_flag;
extern fp32 gyro_x_zero;
extern fp32 gyro_y_zero;
extern fp32 gyro_z_zero;

//初始化函数
void fn_CalibrationInit(void);

//右拨为下,遥感拨到"/ \"累计2s开始陀螺仪校准,校准开始有提示音,20s校准完毕有同样提示音
void fn_CalibrationStart(void);

//陀螺仪均值校准
void fn_GyroCalibration(void);

#endif
