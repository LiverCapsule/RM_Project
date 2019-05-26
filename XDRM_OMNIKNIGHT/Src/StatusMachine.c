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

//��ʱ������ɽ���һ���ı�־λ��һ�������ö���
//����һ����������Ӧ����ǿ��ֹͣ����





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
	//���������ģʽ�л���prapareģʽ��Ҫ��һϵ�в�����ʼ��
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
  //����ֱ�ӻ�״̬���������
	LastWorkState = WorkState;
	if(Is_Lost_Error_Set(LOST_ERROR_RC) || InputMode == STOP)
  //���ɣ�ֱ��ʹ��InputMode��ʹ�ú���GetInputMode()��ʲô��������ʱ��Ϊû������
  {
    WorkState = STOP_STATE;
    return;
  }//��û�м��������Լ�У׼���ܣ�����Ŀǰֻ��ң����ʧ����һ������״��
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
void OperateModeSelect(void)//���ܵ��˶�״̬ѡ��
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

			
			if(RC_CtrlData.rc.s1 == STICK_UP) //���ϼ����ٶȣ�ͨ��3���Ƶ��ǻ�е��������ͨ��2���Ƶ��ǻ�е��ƽ�Ƶ��
      {
				 OperateMode = NormalRC_Mode;
				 AutoMovement = Auto_NoMovement;				
				 CM_SPEED_C = 1.5;
				 CM_OMEGA_C	= 1;		
				
				 LM_SPEED_C = 1;//��������������ȻдΪ1,����ʵ������һ���ֿ��������������в�δ�õ�,������дֻ��ͼ������,����ȥ����
				 AM_SPEED_C = 1;

			}
			
			if(RC_CtrlData.rc.s1 == STICK_CENTRAL) 
      {
				CM_SPEED_C = 1;
				CM_OMEGA_C = 0;//��ʱû����ת��,�˳���Ϊ0	
				
			  LM_SPEED_C = 1;
				AM_SPEED_C = 1;
				
				
				OperateMode = NormalRC_Mode;
				AutoMovement = Auto_NoMovement;	

				
				
						

			}
      if (RC_CtrlData.rc.s1 == STICK_DOWN) //�����ͨ�������þ���ֵ����600ʱ���Զ�����������
      {
				 OperateMode = NormalRC_Mode;
				 AutoMovement = Auto_NoMovement;	
				 CM_SPEED_C = 0;
				 CM_OMEGA_C	= 0;			
				 LM_SPEED_C = 0;//ȫΪ��,����ch2������е��ƽ�Ƶ����,���е���ٶȲ���ͨ��ֱ�ӿ���,���ɶ����������
				 AM_SPEED_C = 0;
				
				
				
				if(RC_CtrlData.rc.ch0 > 600)
				{
					AutoMovement = Auto_Get_Egg;
				}
				else if(RC_CtrlData.rc.ch0 < - 600)
				{
					AutoMovement = Auto_Get_I_Egg;//Auto_Get_I_Egg;
				}
				
				if(RC_CtrlData.rc.ch1 > 600)//�����ͨ��1,2,3��Ӱ�쵽�ܶ�����ת��,����Ҫ��
				{
					AutoMovement = Auto_Pull_Eggs;
				}
				else if(RC_CtrlData.rc.ch1 < - 600)//ͨ��1 3��������
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
    case KEYBOARD_RC_STATE://�����»�е��̧�����Լ���ȥ,��ʱ����
    {
			
			OperateMode = KeyMouse_Mode;

			if(AutoMovement != Auto_NoMovement)//��ʵ��Щ�������Կ��Ǽ���mode��
			{
			//����
			}
			/********�����Ǳ��ֶ�������**********/
			else if(Remote_CheckJumpKey(KEY_Z))//ͨ��������ͨ��ֵ�ı�״̬
			{
				AutoMovement = Auto_Up_Island;
				arm_move_i = 0;
			}
			else if(Remote_CheckJumpKey(KEY_X))
			{
				AutoMovement = Auto_Down_Island;
				arm_move_i = 0;

			}	
//			else if(Remote_CheckJumpKey(KEY_C))//����Ȳ�д��,��������������л�ͼ���Ƕ�
//			{
//				arm_move_i = 0;
//				AutoMovement = Auto_Up_Step;//��һ��
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
				
				ARM_UNPINCH;//�¼�δ��
				
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

			//ң����ģʽ�µ��Զ�����δ���
			
			
			//��ͼ��ת����ʱ���һ����־λ,�ñ�־λ�ı�cm_speedΪ-1,omega_speedҲ��,Ӧ�þͿ�����
			
			
			
			if(AutoMovement != Auto_NoMovement)//���û�����⶯�����ͱ���ԭ����ң��ģʽ
			{
				switch (AutoMovement)
				{
					case Auto_Get_Eggs:
					{
						LiftMechanismMode =		 Lift_NormalRCMode;//Ϊ��ʹ������ƽ��ȡ��ʱ���ܺ��渨����Ӱ��
						ChassisMode = Chassis_Auto_CaliForEgg;//��ʱ����У׼,��Ҫ���ȶ��Ըı�
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
						CM_SPEED_C = 0.8;//�����Ҫע�⣬Ӧ����ûдȫ��
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
						//����
						break;
					}
				}
				else
				{
					
					//û�ж���ʱ,�ٿ�����
						if(RC_CtrlData.rc.s1 == 1)//����1 3 ʱ�϶���ֻ��ͨ�����Ƶ�����ֵ
						{
							ChassisMode =				Chassis_NormalRCMode;//��ʱͨ��0 1���Ƶ���ǰ�������ƶ� ͨ��2������ת
							LiftMechanismMode =		 Lift_NormalRCMode;//ͨ��3����̧������
							GuideWheelMode = GuideWheel_NormalRCMode;//ͨ��1Ҳ���Ƶ���//��������ң�����ֶ��ϵ�//���ﵼ�ִ�������Ҫ��
							Arm_OperateMode  = 						Arm_Locked;
						}
						else if(RC_CtrlData.rc.s1 == 3)
						{
							LiftMechanismMode =	         Lift_Locked;
							ChassisMode =				Chassis_NormalRCMode;//���������ƶ�,����ʱ����ת��,ͨ��1ͬʱ����
							GuideWheelMode =			 GuideWheel_Locked;
							Arm_OperateMode  = 			Arm_NormalRCMode;//��ʱͨ��3��Ϊ��е��̧��,ͨ��2��е��ƽ�Ƶ��
						}
						
						//Ŀǰ4.24 ��е�ۺ͵��ֵ�driver����ң�������ƺ�������Ҫ�Ķ�
						
				
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);

			//			TIM8->CCR1 = 1200;//�رն�ʱ�����ص���
						arm_move_i = 0;
				}
			

    }break;
    
		case KeyMouse_Mode:
    {
      LiftMechanismMode =		 Lift_KeyMouseMode;
      ChassisMode = 			Chassis_KeyMouseMode;
      GuideWheelMode = GuideWheel_KeyMouseMode;
			Arm_OperateMode  = 			Arm_KeyMouseMode;
			
			if(AutoMovement != Auto_NoMovement)//���û�����⶯�����ͱ���ԭ���ļ���ģʽ
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
						//����
						break;
					}
				}
				else
				{
					
						LiftMechanismMode =		 Lift_KeyMouseMode;
						ChassisMode = Chassis_KeyMouseMode;
						GuideWheelMode = GuideWheel_KeyMouseMode;
						Arm_OperateMode  = Arm_KeyMouseMode;///�Ӿ�ʶ��ʱһ����Ҫ̧�������Բ���ס
						

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





void UpIsland_Init(void)//���µ�����ʼ����
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

			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//��ʱ��������̬
		
		
  }
  /* USER CODE END Can_Send_Task */
}





