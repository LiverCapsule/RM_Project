#include "StatusMachine.h"
#include "SuperviseTask.h"
#include "ControlTask.h"

WorkState_e 	WorkState;
WorkState_e		LastWorkState = STOP_STATE;
extern uint32_t time_tick_1ms;

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

void WorkStateFSM(void)
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
      if(InputMode == STOP)
      {
        WorkState = STOP_STATE;
      }
    }break;

    case KEYBOARD_RC_STATE:
    {
      if(InputMode == STOP)
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



void Input_Mode_Select(void)
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


void StatusMachine_Init(void)
{
  WorkState = PREPARE_STATE;
}


void StatusMachine_Update(void)
{
  Input_Mode_Select();
  WorkStateFSM();

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





