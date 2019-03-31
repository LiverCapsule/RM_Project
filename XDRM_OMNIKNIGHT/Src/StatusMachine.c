#include "StatusMachine.h"
#include "SuperviseTask.h"
#include "ControlTask.h"

#include "Driver_Beltraise.h"
#include "Driver_Chassis.h"
#include "Driver_GuideWheel.h"
#include "Driver_Manipulator.h"
#include "Driver_Remote.h"
#include "Driver_Sensor.h"
#include "DriverLib_Ramp.h"



InputMode_e	InputMode;


//��Ҫ��һ��mode��Ϊ���ϲ㣬Ȼ��state����������������������Ϊ����һ��������ʹ�����
//ң�������ݺ�֡�ʼ�ⲿ����Ϊstate�����룬ң������state��Ϊ����ģʽ������
WorkState_e 	WorkState;
WorkState_e		LastWorkState = STOP_STATE;
OperateMode_e OperateMode;
Auto_Manual_Act_e Auto_Manual_Act;

//CarMoveModeTypeDef CarMoveMode;
//UpIslandStateTypeDef UpIslandState;
//DownIslandStateTypeDef   DownIslandState;


uint16_t Up_Island_Flag;
uint16_t Down_Island_Flag;
extern uint32_t time_tick_1ms;


void SetWorkState(WorkState_e state)
{
    WorkState = state;
}

WorkState_e GetWorkState(void)
{
	return WorkState;
}


InputMode_e GetInputMode(void)
{
	return InputMode;
}


void InputMode_Switch(void)
{
  if(RC_CtrlData.rc.s2 == STICK_UP)
  {
    InputMode = REMOTE_INPUT;
  }
  if(RC_CtrlData.rc.s2 == STICK_CENTRAL)
  {
    InputMode = KEY_MOUSE_INPUT;
  }
  if(RC_CtrlData.rc.s2 == STICK_DOWN)
  {
    InputMode = STOP;
  }
}





static void WorkStateSwitchProcess(void)
{
	//���������ģʽ�л���prapareģʽ��Ҫ��һϵ�в�����ʼ��
	if((LastWorkState != WorkState) && (WorkState == PREPARE_STATE))  
	{
		StatusMachine_Init();
		ControlLoopTaskInit();
			//б�³�ʼ��
//	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
//	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
//	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
//	FBSpeedRamp.ResetCounter(&FBSpeedRamp);

	ChassisData.ChassisSpeedRef.Y = 0.0f;
	ChassisData.ChassisSpeedRef.X = 0.0f;
	ChassisData.ChassisSpeedRef.Omega = 0.0f;

	}
}




void WorkState_Update(void)
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
        if(InputMode == KEY_MOUSE_INPUT)
        {
          WorkState = KEY_MOUSE_STATE;
        }
      }
    }break;
  
    case NORMAL_RC_STATE:
    {
			if(InputMode == KEY_MOUSE_INPUT)
			{
				WorkState = KEY_MOUSE_STATE;
			}
      else if(InputMode == STOP)
      {
        WorkState = STOP_STATE;
      }
    }break;

    case KEY_MOUSE_STATE:
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



void OperateMode_Select(void)//���ܵ��˶�״̬ѡ��
{
  switch (WorkState)
  {
    case PREPARE_STATE:
    {
      OperateMode = Stop_Mode;//
    }break;

    case NORMAL_RC_STATE:
    {
      if (RC_CtrlData.rc.s1 == STICK_UP) 
      {
        OperateMode = Test_Mode;//ĿǰӦ��ûʲô����
      }
      if (RC_CtrlData.rc.s1 == STICK_CENTRAL) 
      {
				OperateMode = Manual_Mode;
        if(RC_CtrlData.rc.ch3 > 600)
        {
					
        }
        else if(RC_CtrlData.rc.ch3 < -600)
				{
					
        }
      }
      if (RC_CtrlData.rc.s1 == STICK_DOWN) 
      {
				OperateMode = Auto_Mode;//�Զ�ģʽ�µ��̹���״̬���ͳ���ģʽ��ͬ
				if(RC_CtrlData.rc.ch3 > 600)
        {
					Auto_Manual_Act = Up_Island;
        }
        else if(RC_CtrlData.rc.ch3 < -600)
				{
					Auto_Manual_Act = Down_Island;
        } 
				else if(RC_CtrlData.rc.ch1 > 600)
        {
					Auto_Manual_Act = Fetch_Egg;
        }
        else if(RC_CtrlData.rc.ch1 < -600)
				{
					Auto_Manual_Act = Fetch_Eggs;
        }
				else if(RC_CtrlData.rc.ch2 > 600)
        {
					Auto_Manual_Act = Fetch_I_Egg;
        }
        else if(RC_CtrlData.rc.ch2 < -600)
				{
					Auto_Manual_Act = Fetch_I_Eggs;
        }
      }
    }break;
    case STOP_STATE:
    {
      OperateMode = Stop_Mode;
    }break;
		case KEY_MOUSE_STATE://���ĸ�����ǿ���˳��κ�״̬��������ͨģʽ
		{
			//���е�driver
			//����
			//����1
			//����2
			//����1
			//����2
			//��е�۵��
			
			//�����������������仯
			
			//
			//�ֶ�ģʽ�µ���д�Ķ���
			//1.̧���ϵ� �Ҽ���W S?
			//2.
			
			
			if(Remote_CheckJumpKey(KEY_X) == 1)
			{
				if(OperateMode == Manual_Mode)
				{
					OperateMode = Auto_Mode;
				}
				else if(OperateMode == Auto_Mode)
				{
					OperateMode = Manual_Mode;
				}
			}
			if(Remote_CheckJumpKey(KEY_CTRL) == 1)
			{
			
			
			}

		}
    default:
    {
      OperateMode = Stop_Mode;
    }break;
  }
}
/*
void CarMoveFSM(void)
{
  switch (CarMoveMode)
  {
    case Stop_Move_Mode:
    {
      BeltMode = BeltMove_Stop;
      ChassisMode = ChassisMove_Stop;
      GuideWheelMode = GuideWheelMove_Stop;
    }break;
  
    case Normal_Move_Mode:
    {
      BeltMode = Normal_Rc_BeltMove;
      ChassisMode = Normal_Rc_ChassisMove;
      GuideWheelMode = Normal_Rc_GuideWheelMove;
    }break;

    case Auto_Up_Island_Mode:
    {
      switch (UpIslandState)
      {
        case Up_Island_Prepare:
        {
          UpIslandState = Up_Island_BeltDown_First;
        }break;

        case Up_Island_BeltDown_First://Ƥ���½�ʱ�ѳ�����������
        {
          BM_AngelGet = 1;
          BeltMode = Belt_Down;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        case Up_Island_GuideWheelAdvance_First:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheel_Advance;
        }break;
				
        case Up_Island_BeltUp_First:
        {
          BM_AngelGet = 1;
          BeltMode = Belt_Up;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        case Up_Island_ChassisAdvance_First:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = Chassis_Advance;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        case Up_Island_BeltDown_Twice:
        {
          BM_AngelGet = 1;
          BeltMode = Belt_Down;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        case Up_Island_GuideWheelAdvance_Twice:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheel_Advance;
        }break;
				
        case Up_Island_BeltUp_Twice:
        {
          BM_AngelGet = 1;
          BeltMode = Belt_Up;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        case Up_Island_Stop:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        default:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;
      }
    }break;

    case Auto_Down_Island_Mode:
    {
      switch (DownIslandState)
      {
        case Down_Island_Prepare:
        {
          DownIslandState = Down_Island_ChassisBack_First;
        }break;
        
        case Down_Island_ChassisBack_First:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = Chassis_Back;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        case Down_Island_BeltDown_First:
        {
          BM_AngelGet = 1;
          BeltMode = Belt_Down;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        case Down_Island_GuideWheelBack_First:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheel_Back;
        }break;

        case Down_Island_BeltUp_First:
        {
          BM_AngelGet = 1;
          BeltMode = Belt_Up;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        case Down_Island_ChassisBack_Twice:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = Chassis_Back;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        case Down_Island_BeltDown_Twice:
        {
          BM_AngelGet = 1;
          BeltMode = Belt_Down;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;


        case Down_Island_GuideWheelBack_Twice:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheel_Back;
        }break;

        case Down_Island_BeltUp_Twice:
        {
          BM_AngelGet = 1;
          BeltMode = Belt_Up;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;
      
        case Down_Island_stop:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;

        default:
        {
          BeltMode = BeltMove_Stop;
          ChassisMode = ChassisMove_Stop;
          GuideWheelMode = GuideWheelMove_Stop;
        }break;
      }
    }break;
    default:
    {
      BeltMode = BeltMove_Stop;
      ChassisMode = ChassisMove_Stop;
      GuideWheelMode = GuideWheelMove_Stop;
    }break;
  }
}
*/









void DriversMode_Select(void)
{
	switch(OperateMode)
	{
		case Stop_Mode:
		{
			ChassisMode = Chassis_Locked;
			LongChainMode = 
			ShortChainMode = 
			
			
			//all driver_mode
		
		
		}break;
		case Auto_Mode:
		{//all situation
		 //all driver_mode
			if()
		
		
		}break;
		case Manual_Mode:
		{
			
		
		
		}break;
		case Test_Mode:
		{
			
		
		
		}break;
		default:
				
		break;
	}
	
}




void StatusMachine_Init(void)
{
  WorkState = PREPARE_STATE;

  ChassisMode = ChassisMove_Stop;
  GuideWheelMode = GuideWheelMove_Stop;
  BeltMode = BeltMove_Stop;
  Up_Island_Flag = 0;
  Down_Island_Flag = 0;
  UpIslandState = Up_Island_Prepare;
  DownIslandState = Down_Island_Prepare;


}


void StatusMachine_Update(void)
{
  InputMode_Switch();
  WorkState_Update();
  OperateMode_Select();
	DriversMode_Select();
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





