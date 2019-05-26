#include "StatusMachine.h"
#include "SuperviseTask.h"
#include "ControlTask.h"
#include "Driver_Manipulator.h"
#include "Driver_Chassis.h"
#include "Driver_GuideWheel.h"
#include "Driver_Remote.h"
//#include "Driver_Beltraise.h"
#include "Driver_LiftMechanism.h"
#include "imu.h"

uint16_t MovementEndFlag = 0;

#define MOV_UPISLAND     1
#define MOV_DOWNISLAND   2
#define MOV_FETCH_EGG    3
#define MOV_FETCH_EGGS   4
#define MOV_FETCH_I_EGG  5
#define MOV_FETCH_I_EGGS 6
#define MOV_PULL_I_EGGS  7
#define MOV_GIVE_EGG		 8
//#define MOV_FETCH_EGG 9
//#define MOV_FETCH_EGG 10
//#define MOV_FETCH_EGG 11

//届时动作完成将把一定的标志位置一，跳出该动作
//还有一个按键功能应当是强制停止动作





WorkState_e 	WorkState;
WorkState_e		LastWorkState = STOP_STATE;

OperateMode_e OperateMode;
AutoMovement_e AutoMovement;
//UpIslandStateTypeDef 			UpIslandState;
//DownIslandStateTypeDef  DownIslandState;
extern uint32_t time_tick_1ms;

extern float CM_SPEED_C;
extern float CM_OMEGA_C;

extern float LM_SPEED_C;
extern float AM_SPEED_C;

extern int16_t LiftAngleRef;
uint8_t key_b_cnt = 0;
uint8_t key_b_cnt_last;


static void WorkStateSwitchProcess(void)
{
	//如果从其他模式切换到prapare模式，要将一系列参数初始化
	if((LastWorkState != WorkState) && (WorkState == PREPARE_STATE))  
	{
		ControlLoopTaskInit();
		RemoteTaskInit();
	}
}

void SetWorkState(WorkState_e state)
{
    WorkState = state;
}


WorkState_e GetWorkState(void)
{
	return WorkState;
}

void WorkStateUpdate(void)
{
  //几种直接换状态的特殊情况
	LastWorkState = WorkState;
	if(Is_Lost_Error_Set(LOST_ERROR_RC) || InputMode == STOP)
  //存疑，直接使用InputMode和使用函数GetInputMode()有什么区别，我暂时认为没有区别
  {
    WorkState = STOP_STATE;
    return;
  }//还没有加陀螺仪以及校准功能，所以目前只有遥控器失控这一种特殊状况
  switch (WorkState)
  {
    case PREPARE_STATE:
    {
      if(time_tick_1ms > PREPARE_TIME_TICK_MS)
      {
        if(InputMode == REMOTE_INPUT)
        {
         WorkState = NORMAL_RC_STATE;
        }
        if(InputMode == KEYBOARD_INPUT)
        {
          WorkState = KEYBOARD_RC_STATE;
        }
      }
    }break;
  
    case NORMAL_RC_STATE:
    {
			if(InputMode == KEYBOARD_INPUT)
			{
				WorkState = KEYBOARD_RC_STATE;
			}
      else if(InputMode == STOP)
      {
        WorkState = STOP_STATE;
      }
    }break;

    case KEYBOARD_RC_STATE:
    {
      if(InputMode == REMOTE_INPUT)
			{
				WorkState = NORMAL_RC_STATE;
			}
      else if(InputMode == STOP)
      {
        WorkState = STOP_STATE;
      }
    }break;

    case STOP_STATE:
    {
      if(InputMode != STOP)
      {
        WorkState = PREPARE_STATE;
      }
    }break;
  }
	WorkStateSwitchProcess();
}


extern float ARM_LiftMotorRefAngle;
void InputMode_Select(void)
{
  if(RC_CtrlData.rc.s2 == STICK_UP)
  {
    InputMode = REMOTE_INPUT;
  }
  if(RC_CtrlData.rc.s2 == STICK_CENTRAL)
  {
    InputMode = KEYBOARD_INPUT;
  }
  if(RC_CtrlData.rc.s2 == STICK_DOWN)
  {
    InputMode = STOP;
  }
}
InputMode_e GetInputMode(void)
{
	return InputMode;
}
extern uint8_t lift_flag_again1;
extern uint8_t lift_flag_again;
extern uint8_t arm_move_i;
void OperateModeSelect(void)//车总的运动状态选择
{
  switch (WorkState)
  {
    case PREPARE_STATE:
    {
      OperateMode = Stop_Mode;
    }break;

    case NORMAL_RC_STATE:
    {
        OperateMode = NormalRC_Mode;
				AutoMovement = Auto_NoMovement;

			
			if(RC_CtrlData.rc.s1 == STICK_UP) //在上减慢速度，通道3控制的是机械臂链条，通道2控制的是机械臂平移电机
      {
				 OperateMode = NormalRC_Mode;
				 AutoMovement = Auto_NoMovement;				
				 CM_SPEED_C = 1.5;
				 CM_OMEGA_C	= 1;		
				
				 LM_SPEED_C = 1;//下面两个参数虽然写为1,但是实际上有一部分可能在驱动函数中并未用到,我这样写只是图个方便,懒得去管他
				 AM_SPEED_C = 1;

			}
			
			if(RC_CtrlData.rc.s1 == STICK_CENTRAL) 
      {
				CM_SPEED_C = 1;
				CM_OMEGA_C = 0;//此时没有旋转量,此常数为0	
				
			  LM_SPEED_C = 1;
				AM_SPEED_C = 1;
				
				
				OperateMode = NormalRC_Mode;
				AutoMovement = Auto_NoMovement;	

				
				
						

			}
      if (RC_CtrlData.rc.s1 == STICK_DOWN) //这里的通道可以用绝对值大于600时做自动动作的启动
      {
				 OperateMode = NormalRC_Mode;
				 AutoMovement = Auto_NoMovement;	
				 CM_SPEED_C = 0;
				 CM_OMEGA_C	= 0;			
				 LM_SPEED_C = 0;//全为零,除了ch2用作机械臂平移电机外,所有电机速度不受通道直接控制,而由动作命令控制
				 AM_SPEED_C = 0;
				
				
				
				if(RC_CtrlData.rc.ch0 > 600)
				{
					AutoMovement = Auto_Get_Egg;
				}
				else if(RC_CtrlData.rc.ch0 < - 600)
				{
					AutoMovement = Auto_Get_I_Egg;//Auto_Get_I_Egg;
				}
				
				if(RC_CtrlData.rc.ch1 > 600)//这里的通道1,2,3会影响到很多电机的转动,还需要改
				{
					AutoMovement = Auto_Pull_Eggs;
				}
				else if(RC_CtrlData.rc.ch1 < - 600)//通道1 3还有问题
				{
					AutoMovement = Auto_Get_I_Egg;
				}
				
					
				if(RC_CtrlData.rc.ch2 > 600)
				{
					AutoMovement = Auto_Up_Island;
				}
				else if(RC_CtrlData.rc.ch2 < - 600)
				{
					AutoMovement = Auto_Down_Island;
				}
				
				if(RC_CtrlData.rc.ch3 > 600)
				{
					AutoMovement = Auto_Get_Eggs;
				}
				else if(RC_CtrlData.rc.ch3 < - 600)
				{
					AutoMovement = Auto_Get_I_Eggs;
				}
				
				
				
				
				
			}
			
			
			
    }break;
		
    case STOP_STATE:
    {
      OperateMode = Stop_Mode;
    }break;	
    case KEYBOARD_RC_STATE://键盘下机械臂抬升会自己上去,到时候解决
    {
			
			OperateMode = KeyMouse_Mode;

			if(AutoMovement != Auto_NoMovement)//其实这些东西可以考虑加在mode下
			{
			//不变
			}
			/********以上是保持动作不变**********/
			else if(Remote_CheckJumpKey(KEY_Z))//通过按键与通道值改变状态
			{
				AutoMovement = Auto_Up_Island;
				arm_move_i = 0;
			}
			else if(Remote_CheckJumpKey(KEY_X))
			{
				AutoMovement = Auto_Down_Island;
				arm_move_i = 0;

			}	
//			else if(Remote_CheckJumpKey(KEY_C))//这个先不写把,这个键留给我来切换图传角度
//			{
//				arm_move_i = 0;
//				AutoMovement = Auto_Up_Step;//上一层
//			}
			else if(Remote_CheckJumpKey(KEY_E)&&RC_CtrlData.mouse.press_l ==0)
			{
				arm_move_i = 0;

				AutoMovement = Auto_Get_Egg;
			}
			else if(Remote_CheckJumpKey(KEY_Q)&&RC_CtrlData.mouse.press_l ==1)
			{
					arm_move_i = 0;
			//AutoMovement = Auto_Get_I_Egg;
				AutoMovement = Auto_Get_I_Egg;
			}
			else if(Remote_CheckJumpKey(KEY_E) && RC_CtrlData.mouse.press_l == 1)
			{
				arm_move_i = 0;
//				AutoMovement = Auto_Get_Egg;
			}
			else if(Remote_CheckJumpKey(KEY_Q) && RC_CtrlData.mouse.press_r == 1)
			{
				arm_move_i = 0;
				AutoMovement = Auto_Pull_Eggs;
			}
			else if(Remote_CheckJumpKey(KEY_R))
			{
//				arm_move_i = 0;
//				AutoMovement = Auto_Cali_For_Egg;
			}
//			else if(Remote_CheckJumpKey(KEY_G))
//			{
//				AutoMovement = Auto_NoMovement;
//				arm_move_i = 0;
//				
////				TIM8->CCR1 = 2400;
//				
//				
//			}

			if(Remote_CheckJumpKey(KEY_B))
			{
				arm_move_i = 0;
				AutoMovement = Auto_NoMovement;
				lift_flag_again1 =lift_flag_again =0;
				
				ARM_UNPINCH;//新加未试
				
				ARM_LiftMotorRefAngle = 0;
				ARM_RotateMotorRefAngle = 0;
				ARM_TransMotorRefAngle = 0;
				ARM_BACK;
				LiftAngleRef = 0;
				
				
			}
			
			
    }break;

    default:
    {
      OperateMode = Stop_Mode;
    }break;
  }
}

void DriversModeSelect(void)
{
  switch (OperateMode)
  {
    case Stop_Mode:
    {
      LiftMechanismMode =		 Lift_Locked;
      ChassisMode = 			Chassis_Locked;
      GuideWheelMode = GuideWheel_Locked;
			Arm_OperateMode  = 			Arm_Locked;
			//TIM8->CCR1 = 1200;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);

    }break;
  
    case NormalRC_Mode:
    {	
      LiftMechanismMode =		 Lift_NormalRCMode;
      ChassisMode =				Chassis_NormalRCMode;
      GuideWheelMode = GuideWheel_NormalRCMode;
			Arm_OperateMode  = 			Arm_NormalRCMode;

			//遥控器模式下的自动动作未完成
			
			
			//在图传转动的时候加一个标志位,用标志位改变cm_speed为-1,omega_speed也是,应该就可以了
			
			
			
			if(AutoMovement != Auto_NoMovement)//如果没有特殊动作，就保持原来的遥控模式
			{
				switch (AutoMovement)
				{
					case Auto_Get_Eggs:
					{
						LiftMechanismMode =		 Lift_NormalRCMode;//为了使其左右平移取弹时不受后面辅助轮影响
						ChassisMode = Chassis_Auto_CaliForEgg;//何时进行校准,需要视稳定性改变
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Get_Eggs;
					
					}break;
					case Auto_Get_Egg:
					{
						LiftMechanismMode =		 Lift_Locked;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Get_Egg;
					
					}break;
					case Auto_Get_I_Eggs:
					{
						LiftMechanismMode =		 Lift_NormalRCMode;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Get_I_Eggs;				
					}break;
					case Auto_Get_I_Egg:
					{
						LiftMechanismMode =		 Lift_Locked;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Get_I_Egg;				
					}break;
					case Auto_Pull_Eggs:
					{
						LiftMechanismMode =		 Lift_Locked;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Pull_Eggs;				
					}break;	
					case Auto_Give_Egg:
					{
						LiftMechanismMode =		 Lift_Locked;
						ChassisMode = Chassis_KeyMouseMode;
						CM_SPEED_C = 0.8;//这个需要注意，应该是没写全的
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Give_Egg;				
					}break;	
					case Auto_Up_Island:
					{
						LiftMechanismMode =		 Lift_Auto_UpIsland;
						ChassisMode = Chassis_Auto_UpIsland;
						GuideWheelMode = GuideWheel_Auto_UpIsland;
						Arm_OperateMode  = Arm_Locked;				

					}break;
					
					case Auto_Down_Island:
					{
						LiftMechanismMode =		 Lift_Auto_DownIsland;
						ChassisMode = Chassis_Auto_DownIsland;
						GuideWheelMode = GuideWheel_Auto_DownIsland;
						Arm_OperateMode  = Arm_Locked;
					}break;
					case Auto_Cali_For_Egg:
					{
						LiftMechanismMode =		 Lift_Locked;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Locked;
					}break;

			
					default:
						//不变
						break;
					}
				}
				else
				{
					
					//没有动作时,再看拨杆
						if(RC_CtrlData.rc.s1 == 1)//拨杆1 3 时肯定是只有通道控制的连续值
						{
							ChassisMode =				Chassis_NormalRCMode;//此时通道0 1控制底盘前后左右移动 通道2控制旋转
							LiftMechanismMode =		 Lift_NormalRCMode;//通道3控制抬升链条
							GuideWheelMode = GuideWheel_NormalRCMode;//通道1也控制导轮//这样方便遥控器手动上岛//这里导轮代码里需要改
							Arm_OperateMode  = 						Arm_Locked;
						}
						else if(RC_CtrlData.rc.s1 == 3)
						{
							LiftMechanismMode =	         Lift_Locked;
							ChassisMode =				Chassis_NormalRCMode;//底盘正常移动,但此时不可转动,通道1同时控制
							GuideWheelMode =			 GuideWheel_Locked;
							Arm_OperateMode  = 			Arm_NormalRCMode;//此时通道3作为机械臂抬升,通道2机械臂平移电机
						}
						
						//目前4.24 机械臂和导轮的driver层中遥控器控制函数还需要改动
						
				
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);

			//			TIM8->CCR1 = 1200;//关闭定时器，关弹舱
						arm_move_i = 0;
				}
			

    }break;
    
		case KeyMouse_Mode:
    {
      LiftMechanismMode =		 Lift_KeyMouseMode;
      ChassisMode = 			Chassis_KeyMouseMode;
      GuideWheelMode = GuideWheel_KeyMouseMode;
			Arm_OperateMode  = 			Arm_KeyMouseMode;
			
			if(AutoMovement != Auto_NoMovement)//如果没有特殊动作，就保持原来的键鼠模式
			{
				switch (AutoMovement)
				{
					case Auto_Get_Eggs:
					{
						LiftMechanismMode =		 Lift_KeyMouseMode;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Get_Eggs;
					
					}break;
					case Auto_Get_Egg:
					{
						LiftMechanismMode =		 Lift_Locked;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Get_Egg;
					
					}break;
					case Auto_Get_I_Eggs:
					{
						LiftMechanismMode =		 Lift_KeyMouseMode;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Get_I_Eggs;				
					}break;
					case Auto_Get_I_Egg:
					{
						LiftMechanismMode =		 Lift_Locked;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Get_I_Egg;				
					}break;
					case Auto_Pull_Eggs:
					{
						LiftMechanismMode =		 Lift_Locked;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Pull_Eggs;				
					}break;	
					case Auto_Give_Egg:
					{
						LiftMechanismMode =		 Lift_Locked;
						ChassisMode = Chassis_KeyMouseMode;
						CM_SPEED_C = 0.8;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Auto_Give_Egg;
						
					}break;	
					case Auto_Up_Island:
					{
						LiftMechanismMode =		 Lift_Auto_UpIsland;
						ChassisMode = Chassis_Auto_UpIsland;
						GuideWheelMode = GuideWheel_Auto_UpIsland;
						Arm_OperateMode  = Arm_Locked;				

					}break;
					
					case Auto_Down_Island:
					{
						LiftMechanismMode =		 Lift_Auto_DownIsland;
						ChassisMode = Chassis_Auto_DownIsland;
						GuideWheelMode = GuideWheel_Auto_DownIsland;
						Arm_OperateMode  = Arm_Locked;
					}break;
					case Auto_Cali_For_Egg:
					{
						LiftMechanismMode =		 Lift_Locked;
						ChassisMode = Chassis_Auto_CaliForEgg;
						GuideWheelMode = GuideWheel_Locked;
						Arm_OperateMode  = Arm_Locked;
					}break;

			
					default:
						//不变
						break;
					}
				}
				else
				{
					
						LiftMechanismMode =		 Lift_KeyMouseMode;
						ChassisMode = Chassis_KeyMouseMode;
						GuideWheelMode = GuideWheel_KeyMouseMode;
						Arm_OperateMode  = Arm_KeyMouseMode;///视觉识别时一般需要抬升，所以不锁住
						

						arm_move_i = 0;
//						ARM_LiftMotorRefAngle = 0;
			
				}
			}break;
			case Auto_Mode:
			{
			
			}break;
			
			default:
				{
					LiftMechanismMode =		 Lift_Locked;
					ChassisMode = Chassis_Locked;
					GuideWheelMode = GuideWheel_Locked;
					Arm_OperateMode  = Arm_Locked;
				}break;
			}					
		
}





void UpIsland_Init(void)//上下岛都初始化了
{
  ChassisMode = Chassis_Locked;
  GuideWheelMode = GuideWheel_Locked;
	LiftMechanismMode =	Lift_Locked;
}

void StatusMachine_Init(void)
{
  WorkState = PREPARE_STATE;
	Arm_OperateMode = Arm_Locked;
	AutoMovement = Auto_NoMovement;
  UpIsland_Init();

}



void StatusMachine_Update(void)
{
  InputMode_Select();
  WorkStateUpdate();
  OperateModeSelect();
	DriversModeSelect();
}
	

void StatusMachine(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {

			
			StatusMachine_Update();

			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//此时处于阻塞态
		
		
  }
  /* USER CODE END Can_Send_Task */
}





