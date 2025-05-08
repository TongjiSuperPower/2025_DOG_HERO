#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_

#include "struct_typedef.h"
#include "gimbal_task.h"

//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��


//������3508���PID����
//�Ƕ�˫���⻷����
#define TriggerMotor3508PosPid1_ID205_kp 30.0f//45.0f//10.0f//20.0f//40.0f//35.0f//25.0f//30.0f
#define TriggerMotor3508PosPid1_ID205_ki 1.0f//0.7f//0.5f//0.005f//0.0f//0.5f//0.01f//0.08f//0.1f//0.01f//0.07f
#define TriggerMotor3508PosPid1_ID205_kd 1200.0f//0.0f//0.0f//300.0f//450.0f
//�Ƕ�˫���ڻ�����
#define TriggerMotor3508PosPid2_ID205_kp 1200.0f//3000.0f//4800.0f//6000.0f//400.0f//800.0f//600.0f
#define TriggerMotor3508PosPid2_ID205_ki 0.0f//0.0f
#define TriggerMotor3508PosPid2_ID205_kd 0.0f//30000.0f//55000.0f//10.0f
//���ٶȻ�����
#define TriggerMotor3508PosPid3_ID205_kp 800.0f//320.0f
#define TriggerMotor3508PosPid3_ID205_ki 50.0f//2.0f
#define TriggerMotor3508PosPid3_ID205_kd 10.0f//100.0f

#define TriggerMotor3508SpeedMinOut -10.0f
#define TriggerMotor3508SpeedMaxOut 10.0f
#define TriggerMotor3508SpeedMinIOut -0.2f
#define TriggerMotor3508SpeedMaxIOut 0.2f

#define TriggerMotor3508MinOut -16000.0f
#define TriggerMotor3508MaxOut 16000.0f
#define TriggerMotor3508MinIOut -1500.0f
#define TriggerMotor3508MaxIOut 1500.0f

//Ħ����3508���PID����
//���ٶȻ�PID����
#define GimbalMotor3508PosPid3_ID201_kp 230.0f//220.0f//205.0f
#define GimbalMotor3508PosPid3_ID201_ki 0.015f
#define GimbalMotor3508PosPid3_ID201_kd 10.0f

#define GimbalMotor3508PosPid3_ID202_kp 230.0f
#define GimbalMotor3508PosPid3_ID202_ki 0.015f
#define GimbalMotor3508PosPid3_ID202_kd 10.0f

#define GimbalMotor3508PosPid3_ID203_kp 40.0f
#define GimbalMotor3508PosPid3_ID203_ki 0.0015f
#define GimbalMotor3508PosPid3_ID203_kd 10.0f

#define GimbalMotor3508MinOut -16000.0f
#define GimbalMotor3508MaxOut 16000.0f
#define GimbalMotor3508MinIOut -100.0f
#define GimbalMotor3508MaxIOut 100.0f



//ÿ�������ȴʱ��
#define ShootColdTime 400             //ms
//��������ģʽת����ȴʱ��
#define ShootModeTime 1000            //ms
//ÿ�������������ת�Ƕ�
#define ShootAngleAdd 1.0471975511965f         //rad   0.92��52.7��
//一次射击为了保证射出暂时多转的角度
#define ShootAngleAdd_addition 0.01f
//�����ֶ�ת��ת�Ƕ�
#define TriggerBackAngle 0.3f         //rad    ��10��
//��߸����ٳ���ʱ��
#define ShootTimeUpper 7000
//��͸����ٳ���ʱ��
#define ShootTimeLower 1000
//Ħ����ת��rad/s
#define FricSpeed 406.0f//435.0f//570.0f//520.0f//f//500.0f//520.0f
//˫���ж�һ�����Ħ���ֽ�����ֵ
#define FricSpeedReduce 30.0f
//˫��������ʱ��
#define DetectTime 600


typedef enum{
    
    SHOOT_DOWN,                 //Ħ����δ�򿪣����ģ��down
    SHOOT_READY_SINGLE,         //Ħ�����Ѵ򿪣����Ե������ 
    SHOOT_READY_COUNTINUE,      //Ħ�����Ѵ򿪣��Ա�ģʽ
    SHOOT_INIT,                 //��ת���Զ������תģʽ��ʹ�����ֻ�תһ���Ƕ�  ��ʱ���ã���Ϊ�л�е��λ
    SHOOT_CLEAR,                //���������嵯

}shoot_mode_e;

typedef enum{

	FRIC_OFF = 0,
	FRIC_ON = 1,
    FRIC_DOWN = 2,
    FRIC_WARMING = 3

}fric_state_e;

typedef struct{

    shoot_mode_e shoot_mode;

    fric_state_e fric_state;

    float fric_speed;
    uint16_t shoot_cold_time;

    float infact_shoot_speed;                 //������ٶ�
    float last_infact_shoot_speed;            //�ϴ�����ĵ�����ٶ�

    //uint8_t shoot_over_back_flag;             //单发一次后回转多于角度,1代表需要回转，0代表不需要
    uint8_t first_shoot_flag;                 //是否打出过第一发，是为0，否为1
    uint8_t trigger_back_flag;                //Ϊ1���ڲ�����init����ת��ģʽ�У�Ϊ0����������
    uint8_t shoot_permission_flag;            //Ϊ1ϵͳ�Ż������������Ȩ�ޣ�����ģʽ�����Ʋ���������
    uint8_t fric_state_change_flag;           //Ϊ1����Ըı�Ħ����״̬��Ϊ0���У�Ϊ�˷�ֹ��ҡ��һֱ�������浼��״̬һֱ��
    uint8_t shoot_over_flag;                  //Ϊ1����һ������Ѿ���ɣ���ʼ������ȴ��Ϊ0������һ����������У����ɼ�����ȴ
    uint8_t shoot_single_or_countinue_flag;   //0：单发，1：自爆
    uint8_t double_shoot_flag;                //˫����ʶ�� Ϊ0��û�н��м�⣬Ϊ1�����ڼ��

    uint8_t  last_key_pressed_g;                 //上一次按键G是否按下

}shoot_data_t;


extern shoot_data_t shoot_data;

//�����ֳ�ʼ��
void fn_ShootMotorInit(void);
//Ħ���ֳ�ʼ��
void fn_shoot_motor3508_init(void);
//���ģ��״̬��ʼ��
void fn_shoot_init(void);
//���ģ��ģʽѡ��
void fn_fric_state(void);
//���ģ���������
void fn_ShootMove(void);

//����һ������жϼ�����
extern uint16_t shoot_single_time_count;
//��������жϼ�����
extern uint16_t shooting_single_count;

#endif
