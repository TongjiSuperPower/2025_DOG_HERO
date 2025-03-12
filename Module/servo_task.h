#ifndef SERVO_TASK_H
#define SERVO_TASK_H

#include "struct_typedef.h"

/*开倍镜的pwm值，机械限位*/
#define OCULAR_PWM_OPEN 1700 
#define OCULAR_PWM_CLOSE 2000

/*副pitch的上下限位*/
#define SECONDPITCH_PWM_MAX 1500    //上限位
#define SECONDPITCH_PWM_MIN 1200    //下限位
#define SECONDPITCH_PWM_NORMAL 1500     //正常与枪管平行
#define SECONDPITCH_PWM_STATICLOB 1300  //吊射与地面平行


typedef enum
{
    OCULAR_CLOSE,
    OCULAR_OPEN
}ocular_mode_e;


typedef struct 
{
    uint16_t pwm_set;
    ocular_mode_e mode;
    uint16_t open_pwm;  //开倍镜的pwm值
    uint16_t close_pwm;
    uint8_t channel;    //倍镜舵机为0，tim8的channel 2
}ocular_servo_t;

typedef enum
{
    NO_LOB,
    STATIC_LOB,
    DYNAMIC_LOB,
    LOB_LOCK
}secondpitch_mode_e;


typedef struct
{
    secondpitch_mode_e mode;
    uint16_t max_pwm;   //上限位
    uint16_t min_pwm;   //下限位
    uint16_t pwm_set;
    uint8_t channel;    //副pitch舵机为2，tim1的channel 4
    uint16_t normal_pwm;    //不进吊射模式的pwm值
    uint16_t staticlob_pwm;   //进吊射与地面平行的pwm

    uint16_t last_v_key;

}secondpitch_servo_t;



extern ocular_servo_t ocular_servo;
void fn_ocular_servo_init(void);
void fn_setpwm_ocular(void);

extern secondpitch_servo_t secondpitch_servo;  //副pitch任务在gimbal_task中执行
void fn_secondpitch_servo_init(void);
void fn_setpwm_secondpitch(void);
void fn_setmode_secondpitch(void);

#endif
