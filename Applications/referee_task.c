#include "referee_task.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "struct_typedef.h"
#include "can.h"
#include "main.h"
#include "bsp_usart.h"
#include "protocol.h"
#include "usart.h"
#include "shoot_task.h"
#include "can_task.h"
#include "string.h"
#include "fifo.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "stdbool.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "supercap_task.h"
#include "power_control.h"
#include "math.h"

void init_referee_struct_data(void);
void referee_unpack_fifo_data(void);
void referee_data_solve(uint8_t *frame);
void referee_data_solve(uint8_t *frame);
void send_multi_static_graphic(void);
void send_sight_static_graphic(void);//发送准星静态图形
void send_multi_dynamic_graphic(void);
void referee_send_multi_graphic(ext_id_t target_id, ext_interaction_figure_seven_t* graphic_draw);
void referee_send_five_graphic(ext_id_t target_id, ext_interaction_figure_five_t* graphic_draw);
void send_double_text(void);
void send_pitch_angle(int x, int y, int color, int type);
void send_direction(int x, int y, int color, int type);
void send_capvol_graphic(int capvols);
void send_sentry_HP_static(void);
void send_sentry_HP_graphic(uint16_t red_7_robot_HP);
void send_energy_static(void);
void send_energy_graphic(int energy);
void referee_send_client_graphic(ext_id_t target_id, graphic_data_struct_t* graphic_draw);
void send_rub_graphic(char graphname[3], int x, int y, int color, int type);
void send_string(char* str, char* name, int x, int y, int upd, int colour);
void send_line_graphic(char name[3], uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
void send_rectangle_graphic(char name[3], uint32_t x, uint32_t y, uint32_t end_x, uint32_t end_y);
void send_circle_graphic(char name[3], uint32_t x, uint32_t y, uint32_t radius);
void send_spinning_graphic(char name[3], int x, int y, int color, int type);
void send_shoot_state(void);
void send_autoaim_state(void);
void get_robot_id(void);
bool capdraw = true;
bool energy_draw = true;
bool auto_state = true;
bool shoot = true;
bool number_7_robot_HP_draw = true;

ext_id_t MY_CLIENT_ID = clientid_blue_hero;
int MY_ROBOT_ID = clientid_blue_hero;

uint16_t cmd_id;  // 数据包ID

ext_game_status_t                          ext_game_status;// 比赛状态数据（0x0001）
ext_game_result_t                          ext_game_result;//比赛结果数据(0x0002)
ext_game_robot_HP_t                 	   ext_game_robot_HP;//机器人血量数据（0x0003）


ext_event_data_t                           ext_event_data;//场地事件数据（0x0101）
ext_supply_projectile_action_t             ext_supply_projectile_action;//补给站动作标识数据（0x0102）

ext_referee_warning_t                      ext_referee_warning;//裁判警告数据(0x0104)
ext_dart_info_t                  		   ext_dart_info;//飞镖发射相关数据(0x0105)


ext_robot_status_t                    	   ext_robot_status;//机器人性能体系数据(0x0201)
ext_power_heat_data_t                      ext_power_heat_data;//实时底盘功率和枪口热量数据（0x0202）
ext_robot_pos_t                       	   ext_robot_pos;//机器人位置数据（0x0203）
ext_buff_t                                 ext_buff;//机器人增益数据（0x0204）
ext_air_support_data_t                     ext_air_support_data;//空中支援时间数据（0x0205）
ext_hurt_data_t                            ext_hurt_data;//伤害状态数据（0x0206）
ext_robot_shoot_data_t                     ext_robot_shoot_data;//实时射击数据（0x0207）
ext_projectile_allowance_t                 ext_projectile_allowance;//允许发弹量(0x0208)
ext_rfid_status_t                          ext_rfid_status;//机器人 RFID 模块状态(0x0209)
ext_dart_client_cmd_t                      ext_dart_client_cmd;//飞镖选手端指令数据(0x020A)
ext_ground_robot_position_t                ext_ground_robot_position;//地面机器人位置数据(0x020B)
ext_radar_mark_data_t                      ext_radar_mark_data;//雷达标记进度数据(0x020C)
ext_sentry_info_t                          ext_sentry_info;//哨兵自主决策信息同步(0x020D)
ext_radar_info_t                      	   ext_radar_info;//雷达自主决策信息同步(0x020E)

//-------------0x0301部分开始-------------------
ext_robot_interactive_header_data_t        ext_robot_interactive_header_data;//机器人交互数据（0x0301）
robot_interactive_data_t                   robot_interactive_data;//机器人间交互数据，内容 ID:0x0200~0x02FF
ext_interaction_layer_delete_t             ext_interaction_layer_delete;//客户端删除图形，内容 ID:0x0100;
graphic_data_struct_t                      graphic_data_struct;//图形数据
ext_interaction_figure_t         		   ext_interaction_figure;//客户端绘制一个图形
ext_interaction_figure_double_t            ext_interaction_figure_double;//客户端绘制两个图形
ext_interaction_figure_five_t              ext_interaction_figure_five;//客户端绘制五个图形
ext_interaction_figure_seven_t             ext_interaction_figure_seven;//客户端绘制七个图形
ext_client_custom_character_t              ext_client_custom_character;//客户端绘制字符
//-------------0x0301部分结束-------------------


ext_custom_robot_data_t                    ext_custom_robot_data;//自定义控制器与机器人交互数据(0x0302)
ext_map_command_t                          ext_map_command; //选手端小地图交互数据(0x0303)/*发送频率：触发时发送.*/
ext_remote_control_t                       ext_remote_control;//键鼠遥控数据(0x0304)
ext_client_map_command_t                   ext_client_map_command; //选手端小地图接收雷达数据(0x0305)
ext_custom_client_data_t                   ext_custom_client_data; //自定义控制器与选手端交互数据(0x0306)
ext_map_data_t                             ext_map_data; //选手端小地图接收哨兵数据(0x0307)
ext_custom_info_t                          ext_custom_info; //选手端小地图接收机器人数据(0x0308)

frame_header_struct_t ext_referee_receive_header;
frame_header_struct_t ext_referee_send_header;

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

uint8_t USART6_dma[80];		//DMA接收数据
uint8_t Personal_Data[128];	//DMA发送数据


void Referee_Task(void const * argument){
    
    init_referee_struct_data();//为用于盛装数据的各个结构体分配空间。
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);//初始化用于交换裁判系统信息的队列缓存
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);//通过6号串口收发来自电源管理模块的数据，送至主控模块通过WIFI与服务器通信。
	//USART6通过串口中断接收数据。
	
	uint16_t UI_PushUp_Counter = 0;
	uint16_t UI_static_Counter = 0;
	/* 裁判系统初始化 */
	vTaskDelay(300);
	int op_type;
	while(1){
		referee_unpack_fifo_data();//接收数据

	    vTaskDelay(10);
		get_robot_id();

		UI_PushUp_Counter++;
				
		if(UI_PushUp_Counter > 20){
			UI_PushUp_Counter = 0;
		}
		
		UI_static_Counter++;
		
		if(UI_static_Counter >= 100){
			UI_static_Counter = 0;
		}
		
		if(UI_static_Counter == 20)
		{
			op_type = 1;
		}
		else{
			op_type = 2;
		}
		
		if(IF_KEY_PRESSED_R && (capdraw != true || auto_state != true || shoot != true || energy_draw != true)){
		    //send_multi_graphic();
		    capdraw = true;
		    auto_state = true;
			shoot = true;
			energy_draw = true;
			number_7_robot_HP_draw = true;
		    continue;
		}

		//staitic rectangle
		if(UI_static_Counter == 57)
		{
			send_energy_static();
			send_sentry_HP_static();
			continue;
		}

        if(UI_static_Counter == 29 && IF_KEY_PRESSED_R){
		    send_multi_static_graphic();//七个
			//send_energy_static();
			continue;
	    }

		//剩余能量数据
		if(UI_PushUp_Counter == 19){
			send_energy_graphic(sum_energy);
			if(ext_robot_status.robot_id == robotid_red_hero)
			{
				send_sentry_HP_graphic(ext_game_robot_HP.red_7_robot_HP);
			}
			else
			{
				send_sentry_HP_graphic(ext_game_robot_HP.blue_7_robot_HP);
			}
			continue;
		}

		//电容数据
		if(UI_PushUp_Counter == 16){
			//REST ENERGY
            send_capvol_graphic(cap_data.Capacity);
			continue;
		}

		//dynamic string
		if(UI_PushUp_Counter == 13)
		{
			send_pitch_angle(100, 900, 8, op_type);
			send_direction(1350, 700, 5, op_type);
			continue;
		}

		//其余动态数据UI
		if(UI_PushUp_Counter == 7){
			send_multi_dynamic_graphic();//七个
			continue;
		}
	}
}



void init_referee_struct_data(void)
{
    memset(&ext_referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&ext_game_status, 0, sizeof(ext_game_status_t));
    memset(&ext_game_result, 0, sizeof(ext_game_result_t));
    memset(&ext_game_robot_HP, 0, sizeof(ext_game_robot_HP_t));

    memset(&ext_event_data, 0, sizeof(ext_event_data_t));
    memset(&ext_supply_projectile_action, 0, sizeof(ext_supply_projectile_action_t));
    memset(&ext_referee_warning, 0, sizeof(ext_referee_warning_t));
	memset(&ext_dart_info,             0, sizeof(ext_dart_info));

    memset(&ext_robot_status, 0, sizeof(ext_robot_status_t));
    memset(&ext_power_heat_data, 0, sizeof(ext_power_heat_data_t));
	ext_power_heat_data.buffer_energy = 15;
    memset(&ext_robot_pos, 0, sizeof(ext_robot_pos_t));
    memset(&ext_buff, 0, sizeof(ext_buff));
    memset(&ext_air_support_data, 0, sizeof(air_support_data));
    memset(&ext_hurt_data, 0, sizeof(ext_hurt_data_t));
    memset(&ext_robot_shoot_data, 0, sizeof(ext_robot_shoot_data_t));
    memset(&ext_projectile_allowance, 0, sizeof(ext_projectile_allowance_t));
	memset(&ext_rfid_status,                     0, sizeof(ext_rfid_status));
	memset(&ext_dart_client_cmd,                 0, sizeof(ext_dart_client_cmd));	
	memset(&ext_ground_robot_position,         0, sizeof(ext_ground_robot_position));
	memset(&ext_radar_mark_data,                 0, sizeof(ext_radar_mark_data));
	memset(&ext_sentry_info,                 0, sizeof(ext_sentry_info));
	memset(&ext_radar_info,                 0, sizeof(ext_radar_info));
				

    memset(&ext_robot_interactive_header_data, 0, sizeof(ext_robot_interactive_header_data_t));
	memset(&robot_interactive_data, 0, sizeof(robot_interactive_data));
	memset(&ext_custom_robot_data, 0, sizeof(ext_custom_robot_data));
	memset(&ext_map_command, 0, sizeof(ext_map_command));
	memset(&ext_remote_control, 0, sizeof(ext_remote_control));
}


void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&ext_referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&ext_game_status, frame + index, sizeof(ext_game_status_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&ext_game_result, frame + index, sizeof(ext_game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&ext_game_robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;

        case EVENTS_DATA_CMD_ID:
        {
            memcpy(&ext_event_data, frame + index, sizeof(ext_event_data_t));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&ext_supply_projectile_action, frame + index, sizeof(ext_supply_projectile_action_t));
        }
        break;

        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&ext_referee_warning, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATUS_CMD_ID:
        {
            memcpy(&ext_robot_status, frame + index, sizeof(ext_robot_status_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&ext_power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&ext_robot_pos, frame + index, sizeof(ext_robot_pos_t));
        }
        break;
        case BUFF_CMD_ID:
        {
            memcpy(&ext_buff, frame + index, sizeof(ext_buff_t));
        }
        break;
        case AIR_SUPPORT_DATA_CMD_ID:
        {
            memcpy(&ext_air_support_data, frame + index, sizeof(ext_air_support_data_t));
        }
        break;
        case HURT_DATA_CMD_ID:
        {
            memcpy(&ext_hurt_data, frame + index, sizeof(ext_hurt_data_t));
        }
        break;
        case ROBOT_SHOOT_DATA_CMD_ID:
        {
            memcpy(&ext_robot_shoot_data, frame + index, sizeof(ext_robot_shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&ext_projectile_allowance, frame + index, sizeof(ext_projectile_allowance_t));
        }
        break;
        case RFID_STATUS_CMD_ID:
        {
            memcpy(&ext_rfid_status, frame + index, sizeof(ext_rfid_status_t));
        }
        break;
				case GROUND_ROBOT_POSITION_CMD_ID: // 0x020B
        {
            memcpy(&ext_ground_robot_position, frame + index, sizeof(ext_ground_robot_position_t));
        }
        break;
				case RADAR_MARK_DATA_CMD_ID: // 0x020C
        {
            memcpy(&ext_radar_mark_data, frame + index, sizeof(ext_radar_mark_data_t));
        }
        break;
        case SENTRY_INFO_CMD_ID: // 0x020D
        {
            memcpy(&ext_sentry_info, frame + index, sizeof(ext_sentry_info_t));
        }
        break;
				case RADAR_INFO_CMD_ID: // 0x020E
        {
            memcpy(&ext_radar_info, frame + index, sizeof(ext_radar_info_t));
        }
        break;
				case ROBOT_INTERACTIVE_DATA_CMD_ID: // 0x0301
        {
            memcpy(&ext_robot_interactive_header_data, frame + index, sizeof(ext_robot_interactive_header_data_t));
        }
				case CUSTOM_ROBOT_DATA_CMD_ID: // 0x0302
        {
            memcpy(&ext_custom_robot_data , frame + index, sizeof(ext_custom_robot_data_t));
        }
        break;
				case MAP_COMMAND_CMD_ID: // 0x0303
        {
            memcpy(&ext_map_command, frame + index, sizeof(ext_map_command_t));
        }
        break;
				case ROBOT_COMMAND_CMD_ID: // 0x0304
        {
            memcpy(&ext_remote_control, frame + index, sizeof(ext_remote_control_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

void get_robot_id()
{
		switch(ext_robot_status.robot_id)
		{
			case robotid_red_hero:{
					MY_CLIENT_ID = clientid_red_hero;
					MY_ROBOT_ID = robotid_red_hero;	
				  break;
			}
			case robotid_red_engineer:{
					MY_CLIENT_ID = clientid_red_engineer;
					MY_ROBOT_ID = robotid_red_engineer;	
				  break;
			}
			case robotid_red_infantry_1:{
					MY_CLIENT_ID = clientid_red_infantry_1;
					MY_ROBOT_ID = robotid_red_infantry_1;	
				  break;
			}
			case robotid_red_infantry_2:{
					MY_CLIENT_ID = clientid_red_infantry_2;
					MY_ROBOT_ID = robotid_red_infantry_2;	
				  break;					
			}
			case robotid_red_infantry_3:{
					MY_CLIENT_ID = clientid_red_infantry_3;
					MY_ROBOT_ID = robotid_red_infantry_3;
				  break;					
			}
			case robotid_red_aerial:{
					MY_CLIENT_ID = clientid_red_aerial;
					MY_ROBOT_ID = robotid_red_aerial;
				  break;					
			} 
		
			case robotid_blue_hero:{
					MY_CLIENT_ID = clientid_blue_hero;
					MY_ROBOT_ID = robotid_blue_hero;	
				  break;
			}
				case robotid_blue_engineer:{
					MY_CLIENT_ID = clientid_blue_engineer;
					MY_ROBOT_ID = robotid_blue_engineer;	
				  break;
			}
			case robotid_blue_infantry_1:{
					MY_CLIENT_ID = clientid_blue_infantry_1;
					MY_ROBOT_ID = robotid_blue_infantry_1;	
				  break;
			}
			case robotid_blue_infantry_2:{
					MY_CLIENT_ID = clientid_blue_infantry_2;
					MY_ROBOT_ID = robotid_blue_infantry_2;
				  break;					
			}
			case robotid_blue_infantry_3:{
					MY_CLIENT_ID = clientid_blue_infantry_3;
					MY_ROBOT_ID = robotid_blue_infantry_3;
				  break;					
			}
			case robotid_blue_aerial:{
					MY_CLIENT_ID = clientid_blue_aerial;
					MY_ROBOT_ID = robotid_blue_aerial;
				  break;					
			} 
 
		}
}

void send_multi_static_graphic(void)//向客户端发送七个个图形的数据。每个图形包含起点终点、操作类型、颜色、线宽等属性。
{

	ext_interaction_figure_seven_t graphic_draw;
	switch(ext_robot_status.robot_id){
		case robotid_red_hero:{
			MY_CLIENT_ID = clientid_red_hero;
			MY_ROBOT_ID = robotid_red_hero;	
			break;
		}
		case robotid_red_infantry_1:{
			MY_CLIENT_ID = clientid_red_infantry_1;
			MY_ROBOT_ID = robotid_red_infantry_1;	
			break;
		}
		case robotid_red_infantry_2:{
			MY_CLIENT_ID = clientid_red_infantry_2;
			MY_ROBOT_ID = robotid_red_infantry_2;	
			break;					
		}
		case robotid_red_infantry_3:{
			MY_CLIENT_ID = clientid_red_infantry_3;
			MY_ROBOT_ID = robotid_red_infantry_3;
			break;					
		}
	
		case robotid_blue_hero:{
			MY_CLIENT_ID = clientid_blue_hero;
			MY_ROBOT_ID = robotid_blue_hero;	
			break;
		}
		case robotid_blue_infantry_1:{
			MY_CLIENT_ID = clientid_blue_infantry_1;
			MY_ROBOT_ID = robotid_blue_infantry_1;	
			break;
		}
		case robotid_blue_infantry_2:{
			MY_CLIENT_ID = clientid_blue_infantry_2;
			MY_ROBOT_ID = robotid_blue_infantry_2;
			break;					
		}
		case robotid_blue_infantry_3:{
			MY_CLIENT_ID = clientid_blue_infantry_3;
			MY_ROBOT_ID = robotid_blue_infantry_3;
			break;					
		}
	}
	
	//准星坐标 (FRONT_SLIGHT_POSITION_X,FRONT_SLIGHT_POSITION_X)
	//准星竖线
	graphic_draw.graphic_data_struct[0].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[0].graphic_name[1] = '4';
	graphic_draw.graphic_data_struct[0].graphic_name[2] = '1';

	graphic_draw.graphic_data_struct[0].operate_tpye = 1;

	graphic_draw.graphic_data_struct[0].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[0].layer = 2;

	graphic_draw.graphic_data_struct[0].color = 2;
	graphic_draw.graphic_data_struct[0].width = 2;

    graphic_draw.graphic_data_struct[0].start_x = FRONT_SLIGHT_POSITION_X;//竖线x轴起始位置
	graphic_draw.graphic_data_struct[0].start_y = 900;
	graphic_draw.graphic_data_struct[0].details_d = FRONT_SLIGHT_POSITION_X;//竖线x轴结束位置
	graphic_draw.graphic_data_struct[0].details_e = 120;

	//准星横线
    graphic_draw.graphic_data_struct[1].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[1].graphic_name[1] = '4';
	graphic_draw.graphic_data_struct[1].graphic_name[2] = '2';

	graphic_draw.graphic_data_struct[1].operate_tpye = 1;

	graphic_draw.graphic_data_struct[1].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[1].layer = 2;

	graphic_draw.graphic_data_struct[1].color = 2;
	graphic_draw.graphic_data_struct[1].width = 2;

    graphic_draw.graphic_data_struct[1].start_x = FRONT_SLIGHT_POSITION_X - 75;
	graphic_draw.graphic_data_struct[1].start_y = FRONT_SLIGHT_POSITION_Y;
	graphic_draw.graphic_data_struct[1].details_d = FRONT_SLIGHT_POSITION_X + 75;
	graphic_draw.graphic_data_struct[1].details_e = FRONT_SLIGHT_POSITION_Y;


	//圆框
	graphic_draw.graphic_data_struct[2].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[2].graphic_name[1] = '3';
	graphic_draw.graphic_data_struct[2].graphic_name[2] = '1';

	graphic_draw.graphic_data_struct[2].operate_tpye = 1;

	graphic_draw.graphic_data_struct[2].graphic_tpye = 2;
	graphic_draw.graphic_data_struct[2].layer = 0;

	graphic_draw.graphic_data_struct[2].color = 8;
	graphic_draw.graphic_data_struct[2].width = 2;

    graphic_draw.graphic_data_struct[2].start_x = 1800;
	graphic_draw.graphic_data_struct[2].start_y = 600;
	graphic_draw.graphic_data_struct[2].details_c = 33;

    //圆框
	graphic_draw.graphic_data_struct[3].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[3].graphic_name[1] = '3';
	graphic_draw.graphic_data_struct[3].graphic_name[2] = '2';

	graphic_draw.graphic_data_struct[3].operate_tpye = 1;

	graphic_draw.graphic_data_struct[3].graphic_tpye = 2;
	graphic_draw.graphic_data_struct[3].layer = 0;

	graphic_draw.graphic_data_struct[3].color = 8;
	graphic_draw.graphic_data_struct[3].width = 2;

    graphic_draw.graphic_data_struct[3].start_x = 1800;
	graphic_draw.graphic_data_struct[3].start_y = 800;
	graphic_draw.graphic_data_struct[3].details_c = 33;

	//电容框
	graphic_draw.graphic_data_struct[4].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[4].graphic_name[1] = '3';
	graphic_draw.graphic_data_struct[4].graphic_name[2] = '3';

	graphic_draw.graphic_data_struct[4].operate_tpye = 1;

	graphic_draw.graphic_data_struct[4].graphic_tpye = 1;
	graphic_draw.graphic_data_struct[4].layer = 0;

	graphic_draw.graphic_data_struct[4].color = 8;
	graphic_draw.graphic_data_struct[4].width = 1;

    graphic_draw.graphic_data_struct[4].start_x = 564;//760
	graphic_draw.graphic_data_struct[4].start_y = 106;
	graphic_draw.graphic_data_struct[4].details_d = 885;
	graphic_draw.graphic_data_struct[4].details_e = 125;

    //示宽线右
	graphic_draw.graphic_data_struct[5].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[5].graphic_name[1] = '8';
	graphic_draw.graphic_data_struct[5].graphic_name[2] = '1';

	graphic_draw.graphic_data_struct[5].operate_tpye = 1;

	graphic_draw.graphic_data_struct[5].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[5].layer = 0;

	graphic_draw.graphic_data_struct[5].color = 2;
	graphic_draw.graphic_data_struct[5].width = 2;

    graphic_draw.graphic_data_struct[5].start_x = 1300;
	graphic_draw.graphic_data_struct[5].start_y = 0;
	graphic_draw.graphic_data_struct[5].details_d = 1142;
	graphic_draw.graphic_data_struct[5].details_e = 255;

    //示宽线左
	graphic_draw.graphic_data_struct[6].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[6].graphic_name[1] = '8';
	graphic_draw.graphic_data_struct[6].graphic_name[2] = '2';

	graphic_draw.graphic_data_struct[6].operate_tpye = 1;

	graphic_draw.graphic_data_struct[6].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[6].layer = 0;

	graphic_draw.graphic_data_struct[6].color = 2;
	graphic_draw.graphic_data_struct[6].width = 2;

    graphic_draw.graphic_data_struct[6].start_x = 620;
	graphic_draw.graphic_data_struct[6].start_y = 0;
	graphic_draw.graphic_data_struct[6].details_d = 778;
	graphic_draw.graphic_data_struct[6].details_e = 255;

	referee_send_multi_graphic(MY_CLIENT_ID, &graphic_draw);
}

//绘制象征密位的6条横线，以及自瞄框
void send_sight_static_graphic()
{

	graphic_data_struct_t graphic_draw;	
	//自瞄框
	graphic_draw.graphic_name[0] = '2';
	graphic_draw.graphic_name[1] = '5';
	graphic_draw.graphic_name[2] = '7';

	graphic_draw.operate_tpye = 1;

	graphic_draw.graphic_tpye = 1;
	graphic_draw.layer = 0;

	graphic_draw.color = 1;
	graphic_draw.width = 3;

    graphic_draw.start_x = 750;//760
	graphic_draw.start_y = 180;
	graphic_draw.details_d = 1100;
	graphic_draw.details_e = 530;
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_multi_dynamic_graphic(void)//向客户端发送七个动态图形的数据。
{

	ext_interaction_figure_seven_t graphic_draw;
	switch(ext_robot_status.robot_id){
		case robotid_red_hero:{
			MY_CLIENT_ID = clientid_red_hero;
			MY_ROBOT_ID = robotid_red_hero;	
			break;
		}
		case robotid_red_infantry_1:{
			MY_CLIENT_ID = clientid_red_infantry_1;
			MY_ROBOT_ID = robotid_red_infantry_1;	
			break;
		}
		case robotid_red_infantry_2:{
			MY_CLIENT_ID = clientid_red_infantry_2;
			MY_ROBOT_ID = robotid_red_infantry_2;	
			break;					
		}
		case robotid_red_infantry_3:{
			MY_CLIENT_ID = clientid_red_infantry_3;
			MY_ROBOT_ID = robotid_red_infantry_3;
			break;					
		}
			
		case robotid_blue_hero:{
			MY_CLIENT_ID = clientid_blue_hero;
			MY_ROBOT_ID = robotid_blue_hero;	
			break;
		}
		case robotid_blue_infantry_1:{
			MY_CLIENT_ID = clientid_blue_infantry_1;
			MY_ROBOT_ID = robotid_blue_infantry_1;	
			break;
		}
		case robotid_blue_infantry_2:{
			MY_CLIENT_ID = clientid_blue_infantry_2;
			MY_ROBOT_ID = robotid_blue_infantry_2;
			break;					
		}
		case robotid_blue_infantry_3:{
			MY_CLIENT_ID = clientid_blue_infantry_3;
			MY_ROBOT_ID = robotid_blue_infantry_3;
			break;					
		}
	}

    //舵机数据
	if(1 == 0){
		graphic_draw.graphic_data_struct[0].operate_tpye = 1;
	}
	else{
		graphic_draw.graphic_data_struct[0].operate_tpye = 3;
	}
	graphic_draw.graphic_data_struct[0].graphic_name[0] = '0';
	graphic_draw.graphic_data_struct[0].graphic_name[1] = '1';
	graphic_draw.graphic_data_struct[0].graphic_name[2] = '3';
	
	graphic_draw.graphic_data_struct[0].color = 4;
	graphic_draw.graphic_data_struct[0].graphic_tpye = 2;
	
	graphic_draw.graphic_data_struct[0].layer = 0;
	graphic_draw.graphic_data_struct[0].width = 20; //线条宽度
	
	graphic_draw.graphic_data_struct[0].start_x = 1800;
	graphic_draw.graphic_data_struct[0].start_y = 500;
	
	graphic_draw.graphic_data_struct[0].details_c = 20;
        


	//自瞄数据 相机自瞄视场角
	graphic_draw.graphic_data_struct[1].graphic_name[0] = '4';
	graphic_draw.graphic_data_struct[1].graphic_name[1] = '0';
	graphic_draw.graphic_data_struct[1].graphic_name[2] = '9';
	if(shoot){
	    graphic_draw.graphic_data_struct[1].operate_tpye = 1;
	    shoot = false;
	}
	else
	{
	    graphic_draw.graphic_data_struct[1].operate_tpye = 2;
	}
	if(gimbal_data.gimbal_behaviour == GIMBAL_AUTO){
		if(autoaim_measure.vision_state == 1){
		    graphic_draw.graphic_data_struct[1].color = 1;
		}
		else{
		    graphic_draw.graphic_data_struct[1].color = 3;
		}
	}
	else{
		graphic_draw.graphic_data_struct[1].color = 8;
	}

	graphic_draw.graphic_data_struct[1].graphic_tpye = 1;
	graphic_draw.graphic_data_struct[1].layer = 0;

	graphic_draw.graphic_data_struct[1].width = 3;

    graphic_draw.graphic_data_struct[1].start_x = 750;
	graphic_draw.graphic_data_struct[1].start_y = 180;
	graphic_draw.graphic_data_struct[1].details_d = 1100;
	graphic_draw.graphic_data_struct[1].details_e = 530;


    //摩擦轮是否开启
	if (shoot_data.fric_state == FRIC_ON)
	{
		graphic_draw.graphic_data_struct[2].operate_tpye = 1;
	}
	else
	{
		graphic_draw.graphic_data_struct[2].operate_tpye = 3;
	}
	graphic_draw.graphic_data_struct[2].graphic_name[0] = '0';
	graphic_draw.graphic_data_struct[2].graphic_name[1] = '0';
	graphic_draw.graphic_data_struct[2].graphic_name[2] = '4';
	
	graphic_draw.graphic_data_struct[2].color = 2;
	graphic_draw.graphic_data_struct[2].graphic_tpye = 2;
	
	graphic_draw.graphic_data_struct[2].layer = 0;
	graphic_draw.graphic_data_struct[2].width = 20; //线条宽度
	
	graphic_draw.graphic_data_struct[2].start_x = 1800;
	graphic_draw.graphic_data_struct[2].start_y = 600;
	
	graphic_draw.graphic_data_struct[2].details_c = 20;
        
	//是否在小陀螺
	if (chassis_move_data.chassis_mode == chassis_spin){
		graphic_draw.graphic_data_struct[3].color = 2;
		graphic_draw.graphic_data_struct[3].operate_tpye = 1;
	}
	else{
		graphic_draw.graphic_data_struct[3].color = 3;
		graphic_draw.graphic_data_struct[3].operate_tpye = 3;
	}
	graphic_draw.graphic_data_struct[3].graphic_name[0] = '0';
	graphic_draw.graphic_data_struct[3].graphic_name[1] = '0';
	graphic_draw.graphic_data_struct[3].graphic_name[2] = '3';
	
	graphic_draw.graphic_data_struct[3].graphic_tpye = 2;
	
	graphic_draw.graphic_data_struct[3].layer = 0;
	graphic_draw.graphic_data_struct[3].width = 20; //线条宽度
	
	graphic_draw.graphic_data_struct[3].start_x = 1800;
	graphic_draw.graphic_data_struct[3].start_y = 800;
	
	graphic_draw.graphic_data_struct[3].details_c = 20;

	//射击模式
	graphic_draw.graphic_data_struct[4].graphic_name[0] = '1';
	graphic_draw.graphic_data_struct[4].graphic_name[1] = '0';
	graphic_draw.graphic_data_struct[4].graphic_name[2] = '9';
	if(auto_state){
	    graphic_draw.graphic_data_struct[4].operate_tpye = 1;
	    auto_state = false;
	}
	else
	{
	    graphic_draw.graphic_data_struct[4].operate_tpye = 2;
	}

	graphic_draw.graphic_data_struct[4].graphic_tpye = 1;
	graphic_draw.graphic_data_struct[4].layer = 0;

	if(shoot_data.shoot_mode == SHOOT_READY_COUNTINUE){
		graphic_draw.graphic_data_struct[4].color = 2;
	}
    else if(shoot_data.shoot_mode == SHOOT_READY_SINGLE){
		graphic_draw.graphic_data_struct[4].color = 3;
    }
 	else{
		graphic_draw.graphic_data_struct[4].color = 8;
	}
	
	graphic_draw.graphic_data_struct[4].width = 40; 
	graphic_draw.graphic_data_struct[4].start_x = 1775;
	graphic_draw.graphic_data_struct[4].start_y = 400;
	
	graphic_draw.graphic_data_struct[4].details_d = 1825;
	graphic_draw.graphic_data_struct[4].details_e = 400;


	graphic_draw.graphic_data_struct[5].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[5].graphic_name[1] = '7';
	graphic_draw.graphic_data_struct[5].graphic_name[2] = '3';

	graphic_draw.graphic_data_struct[5].operate_tpye = 1;

	graphic_draw.graphic_data_struct[5].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[5].layer = 0;

	graphic_draw.graphic_data_struct[5].color = 8;
	graphic_draw.graphic_data_struct[5].width = 1;

    graphic_draw.graphic_data_struct[5].start_x = 760;
	graphic_draw.graphic_data_struct[5].start_y = 80;
	graphic_draw.graphic_data_struct[5].details_d = 1172;
	graphic_draw.graphic_data_struct[5].details_e = 60;


	graphic_draw.graphic_data_struct[6].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[6].graphic_name[1] = '7';
	graphic_draw.graphic_data_struct[6].graphic_name[2] = '4';

	graphic_draw.graphic_data_struct[6].operate_tpye = 1;

	graphic_draw.graphic_data_struct[6].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[6].layer = 0;

	graphic_draw.graphic_data_struct[6].color = 8;
	graphic_draw.graphic_data_struct[6].width = 1;

    graphic_draw.graphic_data_struct[6].start_x = 760;
	graphic_draw.graphic_data_struct[6].start_y = 80;
	graphic_draw.graphic_data_struct[6].details_d = 1172;
	graphic_draw.graphic_data_struct[6].details_e = 60;

	referee_send_multi_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_pitch_angle(int x, int y, int color, int type)
{
	
	char data[30];
	bool sign;
	fp32 pitch_angle = gimbal_motormi_data[0].relative_raw_angle;
	if(pitch_angle > 0)
		sign =true;
	else
	{
		sign = false;
		pitch_angle = -pitch_angle;
	}

	int32_t one = ((int)pitch_angle)%10;
	int32_t ten = ((((int)pitch_angle) - one)/10)%10;
	int32_t tenth = ((int)(pitch_angle*10))%10;
	int32_t hunth = ((int)(pitch_angle*100))%10;

	char pitchname[3]="210";
	

	data[0] = 'p';
	data[1] = 'i';
	data[2] = 't';
	data[3] = 'c';
	data[4] = 'h';
	data[5] = ':';
	if (sign)
		data[6] = ' ';
	else
		data[6] = '-';
	data[7] = ten+'0';
	data[8] = one+'0';
	data[9] = '.';
	data[10] = tenth+'0';
	data[11] = hunth+'0';
	data[12] = ' ';
	data[13] = ' ';
	data[14] = ' ';
	data[15] = ' ';
	data[16] = ' ';
	data[17] = ' ';
	data[18] = ' ';
	data[19] = ' ';
	data[20] = ' ';
	data[21] = ' ';
	data[22] = ' ';
	data[23] = ' ';
	data[24] = ' ';
	data[25] = ' ';
	data[26] = ' ';
	data[27] = ' ';
	data[28] = ' ';
	data[29] = ' ';	

	send_string(data, pitchname, x, y, type, color);

}

void send_direction(int x, int y, int color, int type)
{
	
	char data[30];
	bool sign;
	char name[3]="211";
	
	if(gimbal_motor4310_data[0].raw_position <= -2.08f || gimbal_motor4310_data[0].raw_position >= 1.06f )
	{
		sign =true;

	}
	else if(gimbal_motor4310_data[0].raw_position <= 1.06f && gimbal_motor4310_data[0].raw_position >= -2.08f )
	{
		sign =false;
	}

	if(sign == true)
	{		
		data[0] = 'f';
		data[1] = 'r';
		data[2] = 'o';
		data[3] = 'n';
		data[4] = 't';
	}
	else
	{
		data[0] = 'b';
		data[1] = 'a';
		data[2] = 'c';
		data[3] = 'k';
		data[4] = ' ';
	}
	
	data[5] = ' ';
	data[6] = ' ';
	data[7] = ' ';
	data[8] = ' ';
	data[9] = ' ';
	data[10] = ' ';
	data[11] = ' ';
	data[12] = ' ';
	data[13] = ' ';
	data[14] = ' ';
	data[15] = ' ';
	data[16] = ' ';
	data[17] = ' ';
	data[18] = ' ';
	data[19] = ' ';
	data[20] = ' ';
	data[21] = ' ';
	data[22] = ' ';
	data[23] = ' ';
	data[24] = ' ';
	data[25] = ' ';
	data[26] = ' ';
	data[27] = ' ';
	data[28] = ' ';
	data[29] = ' ';	

	send_string(data,name, x, y, type, color);

}

/*//电容框
	graphic_draw.graphic_data_struct[4].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[4].graphic_name[1] = '3';
	graphic_draw.graphic_data_struct[4].graphic_name[2] = '3';

	graphic_draw.graphic_data_struct[4].operate_tpye = 1;

	graphic_draw.graphic_data_struct[4].graphic_tpye = 1;
	graphic_draw.graphic_data_struct[4].layer = 0;

	graphic_draw.graphic_data_struct[4].color = 8;
	graphic_draw.graphic_data_struct[4].width = 1;

    graphic_draw.graphic_data_struct[4].start_x = 564;//760
	graphic_draw.graphic_data_struct[4].start_y = 95;
	graphic_draw.graphic_data_struct[4].details_d = 876;
	graphic_draw.graphic_data_struct[4].details_e = 115;
*/
//剩余能量框
void send_energy_static(void)
{
	graphic_data_struct_t graphic_draw;
	char name[3] = "310";
	graphic_draw.graphic_name[0] = name[0];
	graphic_draw.graphic_name[1] = name[1];
	graphic_draw.graphic_name[2] = name[2];
	
	graphic_draw.operate_tpye = 1;
	graphic_draw.graphic_tpye = 1;
	graphic_draw.layer = 0;

	graphic_draw.color = 8;
	graphic_draw.width = 1;

	graphic_draw.start_x = 999;
	graphic_draw.start_y = 125;
	graphic_draw.details_d = 1301;
	graphic_draw.details_e = 106;

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}


void send_energy_graphic(int energy)
{
	
	graphic_data_struct_t graphic_draw;
	char capname[3] = "311";
	graphic_draw.graphic_name[0] = capname[0];
	graphic_draw.graphic_name[1] = capname[1];
	graphic_draw.graphic_name[2] = capname[2];
	if(energy_draw || IF_KEY_PRESSED_R){
	    graphic_draw.operate_tpye = 1;
	    energy_draw = false;
	}
	else
	{
	    graphic_draw.operate_tpye = 2;
	}

	graphic_draw.color = 6;

	graphic_draw.graphic_tpye = 0;	//直线
	graphic_draw.layer = 1;

	graphic_draw.width = 18; //线条宽度
	graphic_draw.start_x = 1000;
	graphic_draw.start_y = 115;
	
	graphic_draw.details_d  = 1000 + (20000 - energy) * 0.015 ;
	graphic_draw.details_e  = 115;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//rectangle of sentry HP
void send_sentry_HP_static(void)
{
	graphic_data_struct_t graphic_draw;
	char name[3] = "990";
	graphic_draw.graphic_name[0] = name[0];
	graphic_draw.graphic_name[1] = name[1];
	graphic_draw.graphic_name[2] = name[2];
	
	graphic_draw.operate_tpye = 1;
	graphic_draw.graphic_tpye = 1;
	graphic_draw.layer = 0;

	graphic_draw.color = 8;
	graphic_draw.width = 1;

	graphic_draw.start_x = 1400;
	graphic_draw.start_y = 375;
	graphic_draw.details_d = 1700;
	graphic_draw.details_e = 351;

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_sentry_HP_graphic(uint16_t red_7_robot_HP)
{
	
	graphic_data_struct_t graphic_draw;
	char capname[3] = "312";
	graphic_draw.graphic_name[0] = capname[0];
	graphic_draw.graphic_name[1] = capname[1];
	graphic_draw.graphic_name[2] = capname[2];
	if(number_7_robot_HP_draw || IF_KEY_PRESSED_R){
	    graphic_draw.operate_tpye = 1;
	    number_7_robot_HP_draw = false;
	}
	else
	{
	    graphic_draw.operate_tpye = 2;
	}

	if(red_7_robot_HP <= 150)
	{
		graphic_draw.color = 5;
	}
	else{
		graphic_draw.color = 6;
	}

	graphic_draw.graphic_tpye = 0;	//直线
	graphic_draw.layer = 1;

	graphic_draw.width = 18; //线条宽度
	graphic_draw.start_x = 1400;
	graphic_draw.start_y = 363;
	
	graphic_draw.details_d  = 1400 + red_7_robot_HP  * 0.75f ;
	graphic_draw.details_e  = 363;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_capvol_graphic(int capvols)
{
	graphic_data_struct_t graphic_draw;
	char capname[3] = "209";
	graphic_draw.graphic_name[0] = capname[0];
	graphic_draw.graphic_name[1] = capname[1];
	graphic_draw.graphic_name[2] = capname[2];
	if(capdraw || IF_KEY_PRESSED_R){
	    graphic_draw.operate_tpye = 1;
	    capdraw = false;
	}
	else
	{
	    graphic_draw.operate_tpye = 2;
	}

	if(cap_FSM == cap_auto_charge){
		graphic_draw.color = 2;	//绿色
	}
	else{
		graphic_draw.color = 1;	//黄色
	}

	graphic_draw.graphic_tpye = 0;	//直线
	graphic_draw.layer = 1;

	graphic_draw.width = 18; //线条宽度
	graphic_draw.start_x = 565;
	graphic_draw.start_y = 115;
	
	graphic_draw.details_d  = 565 + (capvols - 6) * 14;
	graphic_draw.details_e  = 115;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//发送7个图形
void referee_send_multi_graphic(ext_id_t target_id, ext_interaction_figure_seven_t* graphic_draw){

	static ext_robot_seven_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 7 *15;
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0104;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//将定制好的图形数据传输出去

}

void referee_send_five_graphic(ext_id_t target_id, ext_interaction_figure_five_t* graphic_draw){

	static ext_robot_five_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 5 *15;
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0103;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//将定制好的图形数据传输出去

}

void referee_send_two_graphic(ext_id_t target_id, ext_interaction_figure_double_t* graphic_draw) {

	static ext_robot_two_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 2 * 15;
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0102;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//将定制好的图形数据传输出去
}

void send_double_text(void)
{
	ext_interaction_figure_double_t graphic_draw;
	char name1[3] = "221";
	graphic_draw.graphic_data_struct[0].graphic_name[0] = name1[0];
	graphic_draw.graphic_data_struct[0].graphic_name[1] = name1[1];
	graphic_draw.graphic_data_struct[0].graphic_name[2] = name1[2];
	graphic_draw.graphic_data_struct[0].operate_tpye = 1;
	graphic_draw.graphic_data_struct[0].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[0].layer = 0;
	graphic_draw.graphic_data_struct[0].color = 2;
	graphic_draw.graphic_data_struct[0].width =2;
	graphic_draw.graphic_data_struct[0].start_x =556;
	graphic_draw.graphic_data_struct[0].start_y =0;
	graphic_draw.graphic_data_struct[0].details_d =706;
	graphic_draw.graphic_data_struct[0].details_e =240;
	
	char name2[3] = "223";
	graphic_draw.graphic_data_struct[1].graphic_name[0] = name2[0];
	graphic_draw.graphic_data_struct[1].graphic_name[1] = name2[1];
	graphic_draw.graphic_data_struct[1].graphic_name[2] = name2[2];
	graphic_draw.graphic_data_struct[1].operate_tpye = 1;
	graphic_draw.graphic_data_struct[1].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[1].layer = 0;
	graphic_draw.graphic_data_struct[1].color = 2;
	graphic_draw.graphic_data_struct[1].width =2;
	graphic_draw.graphic_data_struct[1].start_x =1364;
	graphic_draw.graphic_data_struct[1].start_y =0;
	graphic_draw.graphic_data_struct[1].details_d =1214;
	graphic_draw.graphic_data_struct[1].details_e =240;
	
	referee_send_two_graphic(MY_CLIENT_ID, &graphic_draw);
	
}


void referee_send_client_graphic(ext_id_t target_id, graphic_data_struct_t* graphic_draw) 
{
	static ext_robot_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0101;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;
	
	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));//帧头CRC校验

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));
	osDelay(33);
}

void referee_send_client_character(ext_id_t target_id, ext_client_custom_character_t* character_data) {


	static ext_robot_character_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));

	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0110;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;
	
	robot_data.character_data = *character_data;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));
	osDelay(33);

}

//字符发送函数
void  send_string(char* str, char* name, int x, int y, int upd, int colour)
{
	int len = strlen(str) + 1;
	ext_client_custom_character_t character_data;
	character_data.graphic_data_struct.graphic_tpye = 7;//string

	character_data.graphic_data_struct.operate_tpye = upd;//modify

	character_data.graphic_data_struct.layer = 0;

	character_data.graphic_data_struct.color = colour;//green

	character_data.graphic_data_struct.width = 2;
	character_data.graphic_data_struct.details_a = 20;//size
	character_data.graphic_data_struct.details_b = len;//length
	character_data.graphic_data_struct.start_x = x;
	character_data.graphic_data_struct.start_y = y;

	memcpy(character_data.data, (uint8_t*)str, len);
	memcpy(character_data.graphic_data_struct.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_character(MY_CLIENT_ID, &character_data);
	osDelay(33);
}

void send_rub_graphic(char graphname[3], int x, int y, int color, int type)
{
	
	graphic_data_struct_t graphic_draw;
	graphic_draw.graphic_name[0] = graphname[0];
	graphic_draw.graphic_name[0] = graphname[1];
	graphic_draw.graphic_name[0] = graphname[2];
	
	graphic_draw.operate_tpye = type;
	
	graphic_draw.color = color;
	graphic_draw.graphic_tpye = 2;
	
	graphic_draw.layer = 0;
	graphic_draw.width = 20; //线条宽度
	
	graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	
	graphic_draw.details_c = 20;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//画一条直线
void send_line_graphic(char name[3], uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y){

    graphic_data_struct_t graphic_draw;

	graphic_draw.graphic_name[0] = name[0];
	graphic_draw.graphic_name[1] = name[1];
	graphic_draw.graphic_name[2] = name[2];

	graphic_draw.operate_tpye = 1;

	graphic_draw.graphic_tpye = 0;
	graphic_draw.layer = 2;

	graphic_draw.color = 8;
	graphic_draw.width = 2;

    graphic_draw.start_x = start_x;
	graphic_draw.start_y = start_y;
	graphic_draw.details_d = end_x;
	graphic_draw.details_e = end_y;

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//矩形
void send_rectangle_graphic(char name[3], uint32_t x, uint32_t y, uint32_t end_x, uint32_t end_y){
	graphic_data_struct_t graphic_draw;

	graphic_draw.graphic_name[0] = name[0];
	graphic_draw.graphic_name[1] = name[1];
	graphic_draw.graphic_name[2] = name[2];

	graphic_draw.operate_tpye = 1;

	graphic_draw.graphic_tpye = 1;
	graphic_draw.layer = 0;

	graphic_draw.color = 8;
	graphic_draw.width = 1
	;

    graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	graphic_draw.details_d = end_x;
	graphic_draw.details_e = end_y;

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//细圆
void send_circle_graphic(char name[3], uint32_t x, uint32_t y, uint32_t radius){

	graphic_data_struct_t graphic_draw;

	graphic_draw.graphic_name[0] = name[0];
	graphic_draw.graphic_name[1] = name[1];
	graphic_draw.graphic_name[2] = name[2];

	graphic_draw.operate_tpye = 1;

	graphic_draw.graphic_tpye = 2;
	graphic_draw.layer = 0;

	graphic_draw.color = 8;
	graphic_draw.width = 2;

    graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	graphic_draw.details_c = radius;

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//粗圆
void send_spinning_graphic(char name[3], int x, int y, int color, int type)
{
	
	graphic_data_struct_t graphic_draw;
	
	graphic_draw.graphic_name[0] =name[0];
	graphic_draw.graphic_name[1] =name[1];
	graphic_draw.graphic_name[2] =name[2];
	
	
	graphic_draw.operate_tpye = type;
	
	graphic_draw.color = color;
	graphic_draw.graphic_tpye = 2;
	
	graphic_draw.layer = 0;
	graphic_draw.width = 20; //线条宽度
	
	graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	
	graphic_draw.details_c = 20;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}


void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
        }
    }
}


void send_autoaim_state(void){
	graphic_data_struct_t graphic_draw;
	char capname[3] = "409";
	graphic_draw.graphic_name[0] = capname[0];
	graphic_draw.graphic_name[1] = capname[1];
	graphic_draw.graphic_name[2] = capname[2];
	if(shoot){
	graphic_draw.operate_tpye = 1;
	shoot = false;
	}
	else
	{
	graphic_draw.operate_tpye = 2;
	}
	graphic_draw.graphic_tpye = 1;
	graphic_draw.layer = 0;
	if(gimbal_data.gimbal_behaviour == GIMBAL_AUTO){
		if(autoaim_measure.vision_state == 1){
		  graphic_draw.color = 2;
		}
		else{
		  graphic_draw.color = 3;
		}
	}
	else{
		graphic_draw.color = 8;
	}
	graphic_draw.width = 40; 
	graphic_draw.start_x = 1775;
	graphic_draw.start_y = 700;
	
	graphic_draw.details_d = 1825;
	graphic_draw.details_e = 700;
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_shoot_state(void){
	graphic_data_struct_t graphic_draw;
	char capname[3] = "109";
	graphic_draw.graphic_name[0] = capname[0];
	graphic_draw.graphic_name[1] = capname[1];
	graphic_draw.graphic_name[2] = capname[2];
	if(auto_state){
	graphic_draw.operate_tpye = 1;
	auto_state = false;
	}
	else
	{
	graphic_draw.operate_tpye = 2;
	}
	graphic_draw.graphic_tpye = 1;
	graphic_draw.layer = 0;
	if(shoot_data.shoot_mode == SHOOT_READY_COUNTINUE){
		  graphic_draw.color = 2;
	}
  else if(shoot_data.shoot_mode == SHOOT_READY_SINGLE){
		  graphic_draw.color = 3;
  }
	else{
		graphic_draw.color = 8;
	}
	graphic_draw.width = 40; 
	graphic_draw.start_x = 1775;
	graphic_draw.start_y = 400;
	
	graphic_draw.details_d = 1825;
	graphic_draw.details_e = 400;
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//读取底盘功率
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    //*power = ext_power_heat_data.chassis_power;
	*power = cap_data.Cell_Power;
    *buffer = ext_power_heat_data.buffer_energy;
}

//读取枪口射速
void get_shoot_speed(float *speed){
	*speed = ext_robot_shoot_data.initial_speed;;
}
