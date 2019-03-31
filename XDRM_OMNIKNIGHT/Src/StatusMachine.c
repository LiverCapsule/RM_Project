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


//需要另一中mode作为最上层，然后state和他并列甚至高于他，作为他的一部分输入和错误检测
//遥控器数据和帧率检测部分作为state的输入，遥控器和state作为运行模式的输入
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
	//如果从其他模式切换到prapare模式，要将一系列参数初始化
	if((LastWorkState != WorkState) && (WorkState == PREPARE_STATE))  
	{
		StatusMachine_Init();
		ControlLoopTaskInit();
			//斜坡初始化
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



void OperateMode_Select(void)//车总的运动状态选择
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
        OperateMode = Test_Mode;//目前应该没什么意义
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
				OperateMode = Auto_Mode;//自动模式下底盘工作状态不和常规模式相同
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
		case KEY_MOUSE_STATE://按哪个键，强制退出任何状态，进入普通模式
		{
			//所有的driver
			//导轮
			//链条1
			//链条2
			//气缸1
			//气缸2
			//机械臂电机
			
			//用鼠标滚轮做电机输出变化
			
			//
			//手动模式下单独写的东西
			//1.抬升上岛 右键下W S?
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

        case Up_Island_BeltDown_First://皮带下降时把车给撑起来了
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
			
			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//此时处于阻塞态
		
		
  }
  /* USER CODE END Can_Send_Task */
}





