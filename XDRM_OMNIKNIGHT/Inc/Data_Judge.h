#ifndef __DATA_JUDGE_H
#define __DATA_JUDGE_H



#include "config.h"



//uartͨ�����ã�������115200,����λ8,ֹͣλ1������λ�ޣ���������

//ͨ��Э���ʽ
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
} ext_game_state_t;//����״̬����0x0001    1Hz



typedef __packed struct {
uint8_t winner; 
} ext_game_result_t;//�����������0x0002		��������

typedef __packed struct {
uint16_t robot_legion; 
} ext_game_robot_survivors_t;//�����˴������0x0003		1Hz


typedef __packed struct { 
uint32_t event_type;
} ext_event_data_t;//�����¼�����0x0101  �¼��ı䷢��

typedef __packed struct { 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id; 
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;//����վ������ʶ��0x0102������Ƶ�ʣ������ı����



typedef __packed struct { 
	uint8_t supply_projectile_id; 
	uint8_t supply_robot_id;
	uint8_t supply_num; 
} ext_supply_projectile_booking_t;//���󲹸�վ�����ӵ���cmd_id (0x0103)������Ƶ�ʣ�����10Hz��RM�Կ�����δ����

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
} ext_game_robot_state_t;//����������״̬��0x0201������Ƶ�ʣ�10Hz
typedef __packed struct { 
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_heat0;
	uint16_t shooter_heat1; 
} ext_power_heat_data_t;//ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz

typedef __packed struct {
float x; 
float y;
float z; 
float yaw;
} ext_game_robot_pos_t;
//������λ�ã�0x0203������Ƶ�ʣ�10Hz


typedef __packed struct { 
uint8_t power_rune_buff; 
}ext_buff_musk_t;
//���������棺0x0204������Ƶ�ʣ�״̬�ı����



typedef __packed struct {
	uint8_t energy_point; 
	uint8_t attack_time; 
} aerial_robot_energy_t;
//���л���������״̬��0x0205������Ƶ�ʣ�10Hz


typedef __packed struct { 
	uint8_t armor_id : 4; 
  uint8_t hurt_type : 4;
} ext_robot_hurt_t;
//�˺�״̬��0x0206������Ƶ�ʣ��˺���������



typedef __packed struct { 
	uint8_t bullet_type; 
	uint8_t bullet_freq; 
  float bullet_speed; 
} ext_shoot_data_t;

//ʵʱ�����Ϣ��0x0207������Ƶ�ʣ��������



//�����˼佻������
//���ݶ����Ϊ128 - 9 - 6 = 113�ֽ�


typedef __packed struct { 
	uint16_t data_cmd_id; //���ݶε�����ID
	uint16_t send_id; 		//������ID
	uint16_t receiver_id;	//������ID
}ext_student_interactive_header_data_t;
//�������ݽ�����Ϣ��0x0301������Ƶ�ʣ�����10Hz


typedef __packed struct
{
float data1;
float data2;
float data3;
uint8_t masks;
} client_custom_data_t;
//�ͻ����Զ������ݣ�cmd_id:0x0301������ID:0xD180������Ƶ�ʣ�����10Hz



typedef __packed struct
{
	uint8_t data[113];//n���Ϊ113
} robot_interactive_data_t;





//��ȫ��д������˵����ʱ���ȫ��������������û���ã��ٿ���Ҫ��Ҫ��û�õĶ���ɾ��
typedef struct
{
	ext_game_state_t 										GameState;//����״̬���ݣ�1Hz���ڷ���
	ext_game_result_t									 GameResult;//����������ݣ�������������
	ext_game_robot_survivors_t 					Survivors;//���������˴�����ݣ�1Hz���ڷ���
	ext_event_data_t 										EventData;//�����¼����ݣ��¼��ı����
	ext_supply_projectile_action_t 	 SupplyAction;//���ز���վ������ʶ���ݣ������ı����
	ext_supply_projectile_booking_t SupplyRequest;//���󲹸�վ�������ݣ��ɲ����ӷ��ͣ����� 10Hz����RM�Կ�����δ���ţ�
	ext_game_robot_state_t 						 RobotState;//������״̬���ݣ�10Hz���ڷ���
	ext_power_heat_data_t			 	 	 PowerHeatState;//ʵʱ�����������ݣ�50Hz���ڷ���
	ext_game_robot_pos_t 								 RobotPos;//������λ�����ݣ�10Hz����
	ext_buff_musk_t											 BuffMusk;//�������������ݣ�����״̬�ı����
	aerial_robot_energy_t						AirRobotState;//���л���������״̬���ݣ�10Hz���ڷ��ͣ�ֻ�п��л��������ط���
	ext_robot_hurt_t 										RobotHurt;//�˺�״̬���ݣ��˺���������
	ext_shoot_data_t										ShootData;//ʵʱ������ݣ��ӵ��������
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
}Info_Inter_Judge_t;//��һ�����ǽ����ģ������˼佻�����ݣ����ͷ��������ͣ�����10Hz















#endif




