#ifndef __DATA_JUDGE_H
#define __DATA_JUDGE_H



#include "config.h"



//uart通信配置，波特率115200,数据位8,停止位1，检验位无，流控制无

//通信协议格式
//FrameHeader(4-Byte) 
//CmdID(2-Byte) 
//Data(n-Byte) 
//FrameTail(2-Byte, CRC16)


#define JudgeBufferLength       50



#define HEADER_LEN   5
#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes




typedef __packed struct{
	uint8_t		 start_byte;
	uint16_t  data_length;
	uint8_t       seq_num;
	uint8_t          crc8;
}frame_header_t;




typedef __packed struct {
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
} ext_game_state_t;//比赛状态数据0x0001    1Hz



typedef __packed struct {
uint8_t winner; 
} ext_game_result_t;//比赛结果数据0x0002		结束发送

typedef __packed struct {
uint16_t robot_legion; 
} ext_game_robot_survivors_t;//机器人存活数据0x0003		1Hz


typedef __packed struct { 
uint32_t event_type;
} ext_event_data_t;//场地事件数据0x0101  事件改变发送

typedef __packed struct { 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id; 
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;//补给站动作标识：0x0102。发送频率：动作改变后发送



typedef __packed struct { 
	uint8_t supply_projectile_id; 
	uint8_t supply_robot_id;
	uint8_t supply_num; 
} ext_supply_projectile_booking_t;//请求补给站补弹子弹：cmd_id (0x0103)。发送频率：上限10Hz。RM对抗赛尚未开放

typedef __packed struct { 
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP; 
	uint16_t max_HP; 
	uint16_t shooter_heat0_cooling_rate;
	uint16_t shooter_heat0_cooling_limit;
	uint16_t shooter_heat1_cooling_rate; 
	uint16_t shooter_heat1_cooling_limit; 
	uint8_t mains_power_gimbal_output : 1; 
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_state_t;//比赛机器人状态：0x0201。发送频率：10Hz
typedef __packed struct { 
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_heat0;
	uint16_t shooter_heat1; 
} ext_power_heat_data_t;//实时功率热量数据：0x0202。发送频率：50Hz

typedef __packed struct {
float x; 
float y;
float z; 
float yaw;
} ext_game_robot_pos_t;
//机器人位置：0x0203。发送频率：10Hz


typedef __packed struct { 
uint8_t power_rune_buff; 
}ext_buff_musk_t;
//机器人增益：0x0204。发送频率：状态改变后发送



typedef __packed struct {
	uint8_t energy_point; 
	uint8_t attack_time; 
} aerial_robot_energy_t;
//空中机器人能量状态：0x0205。发送频率：10Hz


typedef __packed struct { 
	uint8_t armor_id : 4; 
  uint8_t hurt_type : 4;
} ext_robot_hurt_t;
//伤害状态：0x0206。发送频率：伤害发生后发送



typedef __packed struct { 
	uint8_t bullet_type; 
	uint8_t bullet_freq; 
  float bullet_speed; 
} ext_shoot_data_t;

//实时射击信息：0x0207。发送频率：射击后发送



//机器人间交互数据
//数据段最大为128 - 9 - 6 = 113字节


typedef __packed struct { 
	uint16_t data_cmd_id; //数据段的内容ID
	uint16_t send_id; 		//发送者ID
	uint16_t receiver_id;	//接收者ID
}ext_student_interactive_header_data_t;
//交互数据接收信息：0x0301。发送频率：上限10Hz


typedef __packed struct
{
float data1;
float data2;
float data3;
uint8_t masks;
} client_custom_data_t;
//客户端自定义数据：cmd_id:0x0301。内容ID:0xD180。发送频率：上限10Hz



typedef __packed struct
{
	uint8_t data[113];//n最大为113
} robot_interactive_data_t;





//先全都写进来再说，到时候把全部变量都看看有没有用，再考虑要不要把没用的东西删了
typedef struct
{
	ext_game_state_t 										GameState;//比赛状态数据，1Hz周期发送
	ext_game_result_t									 GameResult;//比赛结果数据，比赛结束后发送
	ext_game_robot_survivors_t 					Survivors;//比赛机器人存活数据，1Hz周期发送
	ext_event_data_t 										EventData;//场地事件数据，事件改变后发送
	ext_supply_projectile_action_t 	 SupplyAction;//场地补给站动作标识数据，动作改变后发送
	ext_supply_projectile_booking_t SupplyRequest;//请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM对抗赛尚未开放）
	ext_game_robot_state_t 						 RobotState;//机器人状态数据，10Hz周期发送
	ext_power_heat_data_t			 	 	 PowerHeatState;//实时功率热量数据，50Hz周期发送
	ext_game_robot_pos_t 								 RobotPos;//机器人位置数据，10Hz发送
	ext_buff_musk_t											 BuffMusk;//机器人增益数据，增益状态改变后发送
	aerial_robot_energy_t						AirRobotState;//空中机器人能量状态数据，10Hz周期发送，只有空中机器人主控发送
	ext_robot_hurt_t 										RobotHurt;//伤害状态数据，伤害发生后发送
	ext_shoot_data_t										ShootData;//实时射击数据，子弹发射后发送
}Info_Rc_Judge_t;


typedef enum
{
  GAMESTATE_DATA_ID       = 0x0001,  
  GAMERESULT_DATA_ID 			= 0x0002,
  SURVIVORS_DATA_ID 			= 0x0003,
  EVENT_DATA_ID 					= 0x0101,  
  SUPPLYACTION_DATA_ID 		= 0x0102,  
  SUPPLYREQUEST_DATA_ID   = 0x0103,
  ROBOTSTATE_DATA_ID      = 0x0201,
  POWER_HEAT_STATE_DATA_ID= 0x0202,
	ROBOTPOS_DATA_ID				= 0x0203,
	BUFFMUSK_DATA_ID				= 0x0204,
	AIRROBOTSTATE_DATA_ID		= 0x0205,
	ROBOTHURT_DATA_ID				= 0x0206,
	SHOOT_DATA_ID						= 0x0207,
  CLIENT_INTER_DATA_ID    = 0x0301, 
} judge_data_id_e;





typedef struct
{
	frame_header_t 											 			Frameheader;
	ext_student_interactive_header_data_t	InterDataHeader;
	client_custom_data_t											 ClientData;
	robot_interactive_data_t										InterData;
}Info_Inter_Judge_t;//这一部分是交互的，机器人间交互数据，发送方触发发送，上限10Hz















#endif




