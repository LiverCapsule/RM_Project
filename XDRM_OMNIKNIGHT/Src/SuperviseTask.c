#include "main.h"
#include "SuperviseTask.h"
#include "cmsis_os.h"
#include "task.h"

uint32_t *GetLostCounter(uint32_t index);
uint16_t GetLostCounter_max(uint32_t index);
uint32_t lost_err = 0;     //每一位代表一个错误
uint32_t lost_counter[40] = {0};			//看门狗计数

Lost_Counter_Max lost_counter_max = {	
										.LOST_ERROR_RC_MAX 			= 20,
										.LOST_ERROR_MOTOR1_MAX 		= 500,
										.LOST_ERROR_MOTOR2_MAX 		= 500,
										.LOST_ERROR_MOTOR3_MAX 		= 500,
										.LOST_ERROR_MOTOR4_MAX  	= 500,
										.LOST_ERROR_MOTOR_LBELT_MAX = 500,
										.LOST_ERROR_MOTOR_RBELT_MAX = 500,
										.LOST_ERROR_MOTOR_ARM_MAX	= 500,
                    .LOST_ERROR_CAN_TX_MAX          = 1000,
                    .LOST_ERROR_CAN_RX_MAX          = 1000,
                                    };

//标志错误位
void Set_Error_Flag(uint32_t index)
{
	lost_err |= (uint32_t)(1<<index);
}

//重置错误位
void Reset_Error_Flag(uint32_t index)
{
	lost_err &= (uint32_t)~(1<<index);
}                                    

//获取计数的地址
uint32_t *GetLostCounter(uint32_t index)
{
	return (uint32_t *)&lost_counter[index];
}
//获取计数上限
uint16_t GetLostCounter_max(uint32_t index)
{
	uint16_t *pmax =(uint16_t *)(&lost_counter_max + index*8*sizeof(uint16_t));
	return *pmax;
}
//是否出错
uint8_t Is_Lost_Error_Set(uint32_t index)
{
	if((lost_err &(1 << index)) != 0)return 1;
	else							return 0;
}

/*
uint8_t Is_Serious_Error()
{
	#if (XDRM_CONVERTER == 0)
	return Is_Lost_Error_Set(LOST_ERROR_IMU);
	
	#else
	return Is_Lost_Error_Set(LOST_ERROR_XDRMBLACK);

	#endif
}
不知道2019年的工程会不会用到，所以先注释掉*/

/**
  * @brief  用于监控线程的占用情况
  * @param  obj   Mode(THREAD_IN放在线程开头 THREAD_OUT放在结尾,即可)
  * @retval None
  */
void ThreadMonitor(ThreadMonitor_t* obj,uint8_t Mode)
{
	uint32_t currentTime = xTaskGetTickCount();//18年步兵用的是自己写的Get_Time_Micros()，我暂时还不知道区别
	if(Mode == THREAD_IN)
	{
		obj->deltaTime = xTaskGetTickCount() - obj->previousTime;	//距离距离上次运行的时间
		obj->previousTime = currentTime;						//线程上次进入的时间
	}
	else if(Mode == THREAD_OUT)
	{
		obj->excutionTime = currentTime - obj->previousTime;	//线程的执行时间
	}	
}

uint32_t lost_err;
uint32_t lost_counter[40];


uint16_t DBUSFrameRate = 0;
uint16_t DBUSFrameCounter = 0;

uint16_t ChassisFrameRate[4] = {0};
uint16_t ChassisFrameCounter[4] = {0};

uint16_t LbeltFrameRate = 0;
uint16_t LbeltFrameCounter = 0;

uint16_t RbeltFrameRate = 0;
uint16_t RbeltFrameCounter = 0;

uint16_t MotorArmFrameRate = 0;
uint16_t MotorArmFrameCounter = 0;

uint16_t CAN_Send_FrameRate = 0;
uint16_t CAN_Send_FrameCounter = 0;

uint16_t CAN_Res_FrameRate = 0;
uint16_t CAN_Res_FrameCounter = 0;

uint16_t UART2FrameRate = 0;
uint16_t UART2FrameCounter = 0;

portBASE_TYPE Control_Stack = 1;
portBASE_TYPE CANSend_Stack = 1;
portBASE_TYPE SuperviseTask_Stack = 1;

extern osThreadId Led_ToggleHandle;
extern osThreadId Can_SendHandle;
extern osThreadId Task_ControlHandle;
extern osThreadId Status_UpdateHandle;
extern osThreadId Superviese_TaskHandle;

void Task_Monitor(void)
{
	portTickType CurrentTaskMonitor_Tick = 0;
	while(1)
	{		
		osDelayUntil(&CurrentTaskMonitor_Tick, 500);
		//******************  ↓   数据帧率统计   ↓  **************************/
			//DBUS帧率统计
			DBUSFrameRate = DBUSFrameCounter*2;	
			DBUSFrameCounter = 0;
			
			//底盘电机帧率统计
			ChassisFrameRate[0] = ChassisFrameCounter[0]*2;
			ChassisFrameCounter[0] = 0;
			ChassisFrameRate[1] = ChassisFrameCounter[1]*2;
			ChassisFrameCounter[1] = 0;
			ChassisFrameRate[2] = ChassisFrameCounter[2]*2;
			ChassisFrameCounter[2] = 0;
			ChassisFrameRate[3] = ChassisFrameCounter[3]*2;
			ChassisFrameCounter[3] = 0;
			
            //伸缩腿电机帧率统计
			LbeltFrameRate = LbeltFrameCounter*2;
			LbeltFrameCounter = 0;
		
			RbeltFrameRate = RbeltFrameCounter*2;
			RbeltFrameCounter = 0;
			
            //机械臂电机帧率统计
			MotorArmFrameRate = MotorArmFrameCounter*2;
			MotorArmFrameCounter = 0;
			
            //CAN收发帧率统计
			CAN_Send_FrameRate = CAN_Send_FrameCounter*2;
			CAN_Send_FrameCounter = 0;

			CAN_Res_FrameRate = CAN_Res_FrameCounter*2;
			CAN_Res_FrameCounter = 0;
		
		//串口帧率检测
		UART2FrameRate = UART2FrameCounter*2;
		UART2FrameCounter = 0;
		
//---------------------------------------------------------------------		
		//遥控器帧率过低
		if(DBUSFrameRate < 10)
		{
			Set_Error_Flag(LOST_ERROR_RC);
		}
		else
		{
			Reset_Error_Flag(LOST_ERROR_RC);
		}
		//底盘电机帧率过低
		if(ChassisFrameRate[0] < 200)
		{
			Set_Error_Flag(LOST_ERROR_MOTOR1);
		}
		else
		{
			Reset_Error_Flag(LOST_ERROR_MOTOR1);
		}
		
		if(ChassisFrameRate[1] < 200)
		{
			Set_Error_Flag(LOST_ERROR_MOTOR2);
		}
		else
		{
			Reset_Error_Flag(LOST_ERROR_MOTOR2);
		}
		
		if(ChassisFrameRate[2] < 200)
		{
			Set_Error_Flag(LOST_ERROR_MOTOR3);
		}
		else
		{
			Reset_Error_Flag(LOST_ERROR_MOTOR3);
		}
		
		if(ChassisFrameRate[3] < 200)
		{
			Set_Error_Flag(LOST_ERROR_MOTOR4);
		}
		else
		{
			Reset_Error_Flag(LOST_ERROR_MOTOR4);
		}
		
        //伸缩腿电机帧率过低
		if(RbeltFrameRate < 200)
		{
			Set_Error_Flag(LOST_ERROR_MOTOR4);
		}
		else
		{
			Reset_Error_Flag(LOST_ERROR_MOTOR4);
		}
		
		if(LbeltFrameRate < 200)
		{
			Set_Error_Flag(LOST_ERROR_MOTOR_LBELT);
		}
		else
		{
			Reset_Error_Flag(LOST_ERROR_MOTOR_RBELT);
		}
		
        //机械臂电机帧率过低
		if(MotorArmFrameRate < 200)
		{
			Set_Error_Flag(LOST_ERROR_MOTOR_ARM);
		}
		else
		{
			Reset_Error_Flag(LOST_ERROR_MOTOR4);
		}
        //CAN收发帧率过低
        if(CAN_Send_FrameRate < 400)
		{
			Set_Error_Flag(LOST_ERROR_CAN_TX);
		}
		else
		{
			Reset_Error_Flag(LOST_ERROR_CAN_TX);
		}

        if(CAN_Res_FrameRate < 400)
		{
			Set_Error_Flag(LOST_ERROR_CAN_RX);
		}
		else
		{
			Reset_Error_Flag(LOST_ERROR_CAN_RX);
		}
		
		
//-----------------------------------操作系统堆栈监测------------------------------------
		CANSend_Stack=uxTaskGetStackHighWaterMark(Can_SendHandle);
		Control_Stack=uxTaskGetStackHighWaterMark(Task_ControlHandle);
		SuperviseTask_Stack=uxTaskGetStackHighWaterMark(Superviese_TaskHandle);
	}
        
}		

void Superviese_Task(void const * argument)
{

  
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {

			
			Task_Monitor();
			
			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//此时处于阻塞态

  }

}
		


