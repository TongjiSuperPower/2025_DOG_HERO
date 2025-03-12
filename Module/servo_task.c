#include "servo_task.h"
#include "bsp_tim.h"
#include "main.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "chassis_task.h"

ocular_servo_t ocular_servo;    //  倍镜舵机任务在buzzer_task中执行
secondpitch_servo_t secondpitch_servo;  //副pitch任务在gimbal_task中执行

int16_t last_q_key = 0;
int16_t last_e_key = 0;

#define MOUSE_SEN 0.6


void fn_ocular_servo_init(void)
{
    ocular_servo.channel = 0;   //倍镜舵机为0，tim8的channel 2
    ocular_servo.close_pwm = OCULAR_PWM_CLOSE;
    ocular_servo.open_pwm = OCULAR_PWM_OPEN;
    ocular_servo.mode = OCULAR_CLOSE;    
}

void fn_setpwm_ocular(void)
{
    if(ocular_servo.mode == OCULAR_OPEN)
    {
        ocular_servo.pwm_set = ocular_servo.open_pwm;
    }
    else if(ocular_servo.mode == OCULAR_CLOSE)
    {
        ocular_servo.pwm_set = ocular_servo.close_pwm;
    }
}


void fn_secondpitch_servo_init(void)
{
    secondpitch_servo.channel = 2;
    secondpitch_servo.mode = NO_LOB;
    secondpitch_servo.max_pwm = SECONDPITCH_PWM_MAX;
    secondpitch_servo.min_pwm = SECONDPITCH_PWM_MIN;
    secondpitch_servo.normal_pwm = SECONDPITCH_PWM_NORMAL;
    secondpitch_servo.staticlob_pwm = SECONDPITCH_PWM_STATICLOB;
    secondpitch_servo.last_v_key = 0;
}

void fn_setpwm_secondpitch(void)
{
    if(secondpitch_servo.mode == NO_LOB)
    {
        secondpitch_servo.pwm_set = secondpitch_servo.normal_pwm;
    }
    else if(secondpitch_servo.mode == STATIC_LOB)
    {
        secondpitch_servo.pwm_set = secondpitch_servo.staticlob_pwm;
    }
    else if(secondpitch_servo.mode == DYNAMIC_LOB)
    {
        if (IF_KEY_PRESSED_Q && last_q_key == 0)
        {
            secondpitch_servo.pwm_set += 10;
            last_q_key = 1;
        }
        else if (IF_KEY_PRESSED_E && last_e_key == 0)
        {
            secondpitch_servo.pwm_set -= 10;
            last_e_key = 1;
        }
        else if (!IF_KEY_PRESSED_E && last_e_key == 1)
        {
            last_e_key = 0;
        }
        else if (!IF_KEY_PRESSED_Q && last_q_key == 1)
        {
            last_q_key = 0;
        }
        // secondpitch_servo.pwm_set -= ctl.mouse.y * MOUSE_SEN * 0.3 ;
        if(secondpitch_servo.pwm_set > secondpitch_servo.max_pwm)
        {
            secondpitch_servo.pwm_set = secondpitch_servo.max_pwm;
        }
        else if(secondpitch_servo.pwm_set < secondpitch_servo.min_pwm)
        {
            secondpitch_servo.pwm_set = secondpitch_servo.min_pwm;
        }
    }
    else if(secondpitch_servo.mode == LOB_LOCK)
    {
        secondpitch_servo.pwm_set = secondpitch_servo.pwm_set;
    }
}


void fn_setmode_secondpitch(void)
{
    /*遥控器模式*/
    if(!IF_RC_SW2_MID)
    {
        secondpitch_servo.mode = NO_LOB;
    }
    /*键鼠模式*/
    if(IF_RC_SW2_MID)
    {
        /*陀螺仪控制主云台*/
        if(gimbal_data.gimbal_behaviour == GIMBAL_GYRO)
        {
            /*底盘不跟随，即进入吊射模式*/
            if(chassis_move_data.chassis_mode == chassis_not_follow)
            {
                if(secondpitch_servo.mode == NO_LOB)
                {
                    secondpitch_servo.mode = STATIC_LOB;
                }
                if(IF_KEY_PRESSED_V && secondpitch_servo.last_v_key == 0 && (secondpitch_servo.mode == STATIC_LOB || secondpitch_servo.mode == LOB_LOCK))
                {
                    secondpitch_servo.mode = DYNAMIC_LOB;
                    secondpitch_servo.last_v_key = 1;
                }
                else if(IF_KEY_PRESSED_V && secondpitch_servo.last_v_key == 0 && secondpitch_servo.mode == DYNAMIC_LOB)
                {
                    secondpitch_servo.mode = LOB_LOCK;
                    secondpitch_servo.last_v_key = 1;
                }
                else if(!IF_KEY_PRESSED_V && secondpitch_servo.last_v_key == 1)
                {
                    secondpitch_servo.last_v_key = 0;
                }
            }
            else
            {
                secondpitch_servo.mode = NO_LOB;
            }
        }
    }
}




