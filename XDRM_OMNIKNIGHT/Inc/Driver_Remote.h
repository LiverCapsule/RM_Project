#ifndef __REMOTE_H
#define __REMOTE_H


#include "stdint.h"



#define  RC_FRAME_LENGTH                       18u
#define  REMOTE_CONTROLLER_STICK_OFFSET      1024u   

#define NORMAL_TIME_2 1500
#define NORMAL_TIME_3 3000


#define REMOTE_SWITCH_CHANGE_1TO3TO1TO3			(uint8_t)((STICK_UP << 6) | (STICK_CENTRAL << 4) | (STICK_UP << 2) | STICK_CENTRAL)//01110111    119
#define REMOTE_SWITCH_CHANGE_1TO3TO2TO3			(uint8_t)((STICK_UP << 6) | (STICK_CENTRAL << 4) | (STICK_DOWN << 2) | STICK_CENTRAL)//01111011   123
#define REMOTE_SWITCH_CHANGE_2TO3TO2TO3			(uint8_t)((STICK_DOWN << 6) | (STICK_CENTRAL << 4) | (STICK_DOWN << 2) | STICK_CENTRAL)//10111011  187
#define REMOTE_SWITCH_CHANGE_2TO3TO1TO3			(uint8_t)((STICK_DOWN << 6) | (STICK_CENTRAL << 4) | (STICK_UP << 2) | STICK_CENTRAL)//10110111    183
#define REMOTE_SWITCH_CHANGE_3TO2TO3TO2			(uint8_t)((STICK_CENTRAL << 6) | (STICK_DOWN << 4) | (STICK_CENTRAL << 2) | STICK_DOWN)//11101110		238
#define REMOTE_SWITCH_CHANGE_3TO2TO3TO1			(uint8_t)((STICK_CENTRAL << 6) | (STICK_DOWN << 4) | (STICK_CENTRAL << 2) | STICK_UP)//11101101			237
#define REMOTE_SWITCH_CHANGE_3TO1TO3TO1			(uint8_t)((STICK_CENTRAL << 6) | (STICK_UP << 4) | (STICK_CENTRAL << 2) | STICK_UP)//11011101   221
#define REMOTE_SWITCH_CHANGE_3TO1TO3TO2			(uint8_t)((STICK_CENTRAL << 6) | (STICK_UP << 4) | (STICK_CENTRAL << 2) | STICK_DOWN)//11011110    222
//动三下改状态
//动两下和以前一样只弄拨盘和摩擦轮，用以前的索引函数
#define REMOTE_SWITCH_CHANGE_1TO3      (uint8_t)((REMOTE_SWITCH_VALUE_UP << 2) | REMOTE_SWITCH_VALUE_CENTRAL)   
#define REMOTE_SWITCH_CHANGE_2TO3      (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 2) | REMOTE_SWITCH_VALUE_CENTRAL)  
#define REMOTE_SWITCH_CHANGE_3TO1      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_UP)
#define REMOTE_SWITCH_CHANGE_3TO2      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_DOWN)

#define REMOTE_SWITCH_CHANGE_1TO3TO2   (uint8_t)((REMOTE_SWITCH_VALUE_UP << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_DOWN))   

#define REMOTE_SWITCH_CHANGE_2TO3TO1   (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_UP)) 

#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u


#define KEY_B		0x8000
#define KEY_V		0x4000
#define KEY_C		0x2000
#define KEY_X		0x1000
#define KEY_Z		0x0800
#define KEY_G		0x0400
#define KEY_F		0x0200
#define KEY_R		0x0100
#define KEY_E		0x0080
#define KEY_Q		0x0040
#define KEY_CTRL	0x0020
#define KEY_SHIFT	0x0010
#define KEY_D		0x0008
#define KEY_A		0x0004
#define KEY_S		0x0002
#define KEY_W		0x0001

typedef __packed struct
{
	int8_t  s1;
  int8_t  s2;
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
}Remote;


typedef __packed struct
{
 uint16_t v;
}Key;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;

typedef __packed struct//__packed 在struct和union中不添加填充字节
{
	Remote rc;
	Key key;
	Mouse mouse;
}RC_Ctrl;

typedef enum
{
	STICK_UP = 1,
	STICK_CENTRAL = 3,
	STICK_DOWN = 2,
}Stick_Position_e;

typedef enum
{	
	NO_CHANGE = 0,
	ONE_CHANGE = 1 ,
	TWO_CHANGE = 2,
	THREE_CHANGE = 3,
	FOUR_CHANGE = 4,
}change_e;


typedef enum
{
	REMOTE_INPUT = 0,
	KEYBOARD_INPUT = 1,
	STOP =3,
}InputMode_e;


uint8_t Remote_CheckJumpKey(uint16_t Key);
void RemoteDataProcess(uint8_t *pdata);
uint8_t stick1_change(void);
uint8_t stick1_action(void);
void RemoteTaskInit(void);


extern uint8_t Action_stick;
extern RC_Ctrl RC_CtrlData;

extern uint8_t stick_store;




#endif

