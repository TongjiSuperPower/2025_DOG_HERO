#include "bsp_tim.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

void fn_servo_pwm_set(uint16_t pwm, uint8_t i)
{
    switch(i)
    {
        case 0:
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm);
        }break;
        case 1:
        {
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
        }break;
        case 2:
        {
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
        }break;
        case 3:
        {
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwm);
        }break;
        case 4:
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);//别用这个，这个没配
        }break;
    }
}//图传舵机是2，倍镜舵机是0
