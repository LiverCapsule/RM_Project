#include "Driver_Manipulator.h"
#include "CanBusTask.h"
#include "Driver_Remote.h"
#include "DriverLib_PID.h"

#define DELAY_MOVEMENT 1


extern TIM_HandleTypeDef htim8;

#define ROTATE_M_ANGLE 1400//1600			  //岛上为1000,岛下1700,岛上惯性大,所以反而更小
#define ROTATE_ANGLE 3250//3200//         //3500,岛上会给足够的时间到达目标值,水平3300
#define ROTATE_M_ANGLE_DELTA -0
#define ROTATE_ANGLE_DELTA 100
#define ROTATE_ANGLE_MAX 3400

#define ROTATE_ANGLE_AWAY 2700
#define MOVE_ANGLE_R 0//25000
#define MOVE_ANGLE_M 0//现在去了左右自由度14000


#define LIFT_ANGLE 6450//6700//6100//左边目标为-值 指抬升到中间
#define LIFT_ANGLE_DELTA1 -0
#define LIFT_H_ANGLE 8800
#define LIFT_ANGLE_DELTA 2250//2500

//岛上时整车高度一改,抬升高度和翻转角度都得改


Arm_OperateMode_e Arm_OperateMode;

uint8_t arm_move_i = 0;//机械臂的实时动作


uint8_t Arm_Fetch_Egg[40] = {ARM_RAISE_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_AWAY_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD,ARM_MOVE_END_CMD};//MIDLE//定义为全局变量，此时除已赋值单元外，其余为单元为0

																																																		//这里MIDDLE先删掉了						
uint8_t Arm_Fetch_Egg2[40] ={ARM_RAISE_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_FALL_CMD,\
														 ARM_WAIT_CM_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_FALL_CMD,\
														 ARM_WAIT_CM_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD,ARM_MOVE_END_CMD};//取两个，当弹舱不够大，但是能脱离围栏(不一定)								//这个是平移是被卡住了
//取完第一个后在机械臂中延时等待,底盘水平移动一段恒定距离

uint8_t Arm_Pull_I_Egg[40]	={ARM_RAISE_CMD,ARM_ROTATE_MAX_CMD,ARM_AHEAD_CMD,ARM_PINCH_CMD,\
	ARM_BACK_CMD,ARM_DELAY_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD,ARM_MOVE_END_CMD};													 
																														//先翻了，气缸等待

	uint8_t Arm_Fetch_I_Egg[40] = {ARM_RAISE_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_AWAY_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD,ARM_MOVE_END_CMD};
	
//这里的eggs指的是两个弹药箱


//uint8_t
//通过状态机获得

float ARM_RotateMotorRefAngle = 0;
float ARM_LiftMotorRefAngle = 0;
float ARM_TransMotorRefAngle = 0;

#include "CanbusTask.h"
float AM_SPEED_C = 1;

//ARM_PINCH   	  
//ARM_UNPINCH   

//ARM_AHEAD				
//ARM_BACK		
extern AutoMovement_e AutoMovement;


float Motor_EcdAngleSet(float Target,Measure *mea)
{
	Target += mea->ecd_angle;
	return	Target;
}

extern float ALM_SPEED_C;
extern float AM_MOVE_C;

float ARM_LiftMotorRefSpeed = 0;
float ARM_MoveMotorRefSpeed = 0;

//状态机中，如何检测退出该动作
uint8_t Arm_Move[40] = {0};

void Arm_Movement_Split(void)//步骤拆分开
{
	switch(Arm_OperateMode)
	{
		case Arm_Auto_Get_Egg:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = Arm_Fetch_Egg[move_n];
			}
		}break;
		case Arm_Auto_Get_Eggs:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
					Arm_Move[move_n] = 0;

				Arm_Move[move_n] = Arm_Fetch_Egg2[move_n];
			}
		}break;
		case Arm_Auto_Get_I_Egg:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = Arm_Fetch_I_Egg[move_n];
			}
		}break;
		case Arm_Auto_Get_I_Eggs:
		{
				for(uint8_t move_n = 0;move_n < 40;move_n++)
			{				
				Arm_Move[move_n] = 0;

//				Arm_Move[move_n] = Arm_Fetch_I_Egg2[move_n];
			}
		}break;
		case Arm_Auto_Pull_Eggs:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
					Arm_Move[move_n] = 0;

				Arm_Move[move_n] = Arm_Pull_I_Egg[move_n];
			}
		}break;

		case Arm_Locked:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = 0;
			}
		}break;
		case Arm_NormalRCMode:
		{
			ARM_LiftMotorRefSpeed = RC_CtrlData.rc.ch3*AM_SPEED_C;//待测
			ARM_MoveMotorRefSpeed = RC_CtrlData.rc.ch2;//*AM_SPEED_C;//这个先留着,方便调
			
			
			
		}break;
		case Arm_KeyMouseMode:
		{//键鼠模式下机械臂平移电机不可手动控制,并且需要在很多地方对机械臂的水平位置进行归位//按B时是否需要不知
			if(Remote_CheckJumpKey(KEY_W)&& RC_CtrlData.mouse.press_r == 1)
			{
				ARM_LiftMotorRefAngle = LIFT_ANGLE;
			}
			else if(Remote_CheckJumpKey(KEY_S)&& RC_CtrlData.mouse.press_r == 1)
			{
				ARM_LiftMotorRefAngle = 0;//应该没问题
			}
			
			ARM_RotateMotorRefAngle = 0;
			ARM_TransMotorRefAngle = 0;
			
			if(Remote_CheckJumpKey(KEY_R)&&RC_CtrlData.mouse.press_r == 0&&RC_CtrlData.mouse.press_l == 1)
			{
				static uint32_t egg_mark2 = 0;
				if(ARM_LiftMotorRefAngle ==0)//实际中可能会有一些问题
				{
					egg_mark2 = xTaskGetTickCount();
				
				}
				ARM_LiftMotorRefAngle =  LIFT_H_ANGLE+1800;//10600
				
				if(xTaskGetTickCount() - egg_mark2 >1000)
				{
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
				}
				
//				TIM8->CCR1 = 2400;
			
			}
			if(Remote_CheckJumpKey(KEY_R)&&RC_CtrlData.mouse.press_r == 1&&RC_CtrlData.mouse.press_l == 0)
			{
	//			static uint32_t egg_mark = 0;
	//			if(TIM8->CCR1 == 2400)//就是一个延时
	//			{
	//				egg_mark = xTaskGetTickCount();
	//			
	//			}
	//			TIM8->CCR1 = 1200;
	//			if(xTaskGetTickCount() - egg_mark >1000)
	//			{
	//				ARM_LiftMotorRefAngle = 0;
	//			}
				
				
				static uint32_t egg_mark = 0;
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == 1)//就是一个延时
				{
					egg_mark = xTaskGetTickCount();
				
				}
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
				if(xTaskGetTickCount() - egg_mark >400)
				{
				  ARM_LiftMotorRefAngle = 0;
				}
			}
			
			
		}
		default:
			break;
	}
}

uint32_t CM_AngleMark = 0;
uint32_t time_mark_fb = 0;
uint32_t time_mark_bf = 0;

float arm_average_angle = 0;
uint16_t time_tick_arm = 0;
uint16_t pinstate1 = 0;

uint8_t Egg_Box_Held = 0;

uint8_t last_arm_move_i = 0;

uint32_t arm_time_mark = 0;

void ArmPart_Get_Movement(void)
{//解算
	arm_average_angle = (abs(LiftChain_Motor6_Measure.ecd_angle)+abs(LiftChain_Motor5_Measure.ecd_angle))/2;
	
	if(Arm_OperateMode != Arm_NormalRCMode && Arm_OperateMode != Arm_KeyMouseMode &&AutoMovement != Auto_NoMovement)//其实这样写不是我本意。。
	{
		switch (Arm_Move[arm_move_i])
		{
			case ARM_PINCH_CMD:
			{
				ARM_PINCH;			//控制这个的电磁阀和弹弹药箱的电磁阀用同一个
				Egg_Box_Held++;
				
				if(xTaskGetTickCount() - arm_time_mark > 500)
				{
					arm_move_i++;		
				}
				
			}break;
			case ARM_UNPINCH_CMD:
			{
				ARM_UNPINCH;
				Egg_Box_Held --;
				
				if(Arm_OperateMode == Arm_Auto_Get_I_Egg)
				{
						if(xTaskGetTickCount() - arm_time_mark > 0)
						{
							arm_move_i++;		
						}
				}
				else
				{
					if(xTaskGetTickCount() - arm_time_mark > 200)
						{
							arm_move_i++;		
						}
					}
			}break;
			
			
			
			case ARM_AHEAD_CMD://气缸前进时速度较慢，需要在进入下一个动作之前有一个延时
			{
				ARM_AHEAD;
				
				if(xTaskGetTickCount() - arm_time_mark > 800)//这个可以正常使用
				{
					arm_move_i++;		
				}

				
			}break;
			case ARM_BACK_CMD:
			{
				ARM_BACK;
				
				if(xTaskGetTickCount() - arm_time_mark > 800)
				{
					arm_move_i++;		
				}
				

		
				
			}break;
			case ARM_RAISE_CMD:
			{
				CM_AngleMark =(abs(Chassis_Motor1_Measure.ecd_angle) + abs(Chassis_Motor2_Measure.ecd_angle) +abs(Chassis_Motor3_Measure.ecd_angle)+abs(Chassis_Motor4_Measure.ecd_angle))/4;
				ARM_LiftMotorRefAngle = LIFT_ANGLE;
				if(Arm_OperateMode == Arm_Auto_Get_I_Eggs||Arm_OperateMode == Arm_Auto_Pull_Eggs)//Arm_OperateMode == Arm_Auto_Get_I_Egg||
				{
					ARM_LiftMotorRefAngle += LIFT_ANGLE_DELTA;
				}
				
				if(Arm_OperateMode == Arm_Auto_Get_I_Egg)
				{
					ARM_LiftMotorRefAngle += LIFT_ANGLE_DELTA1;
				}
				if(arm_average_angle >ARM_LiftMotorRefAngle - 250)//减一个值是为了提早进入下一动作，而这一动作又能继续进行
				{
					arm_move_i++;
				}	
			}break;
			case ARM_HRAISE_CMD:
			{
				ARM_LiftMotorRefAngle =LIFT_H_ANGLE;
				if(arm_average_angle >LIFT_H_ANGLE -300)//
				{
				//	if(xTaskGetTickCount() - arm_time_mark > 200)
			//		{
						arm_move_i++;		
			//		}
				
				}	
			}break;
			case ARM_I_LOW_CMD://机械臂拉回后翻出时把角度拉低一点在夹取
			{
				ARM_LiftMotorRefAngle = LIFT_ANGLE - 200;
				if(arm_average_angle <LIFT_ANGLE - 100)
				{
					arm_move_i++;
				}	
			}break;
			case ARM_LFALL_CMD:
			{
				ARM_LiftMotorRefAngle = 0;
				if(arm_average_angle <300)
				{
					arm_move_i++;
				}	
			}break;
			case ARM_FALL_CMD:
			{
				ARM_LiftMotorRefAngle  = LIFT_ANGLE;
				if(arm_average_angle <LIFT_ANGLE+300)
				{
					arm_move_i++;	
				}	
			}break;
			case ARM_LEFT_CMD:
			{
//				ARM_TransMotorRefAngle  = 0;//最左端为0值，以后写入flash，或开机自检
//				if(MoveArm_Motor_Measure.ecd_angle <2000)
//				{
//					arm_move_i++;
//				}	
				
			}break;
			case ARM_MIDDLE_CMD:
			{
				ARM_TransMotorRefAngle = MOVE_ANGLE_M;
				if(MoveArm_Motor_Measure.ecd_angle >MOVE_ANGLE_M - 200)//从左边过来就写小的//MoveArm_Motor_Measure.ecd_angle <MOVE_ANGLE_M + 200)//提早停下
				{
					arm_move_i++;
				}	
				
			}break;
			case ARM_RIGHT_CMD:
			{
//				ARM_TransMotorRefAngle = MOVE_ANGLE_R;
//				if(MoveArm_Motor_Measure.ecd_angle > MOVE_ANGLE_R - 400)
//				{
//					arm_move_i++;
//				}	
//				
			}break;
			case ARM_ROTATE_I_CMD:
			{
				
				ARM_RotateMotorRefAngle = 0;
					
				
				if(FlipArm_Motor_Measure.ecd_angle < 300)
				{
					arm_move_i++;
				}	

			}break;
				case ARM_ROTATE_O_CMD:
			{
				ARM_RotateMotorRefAngle = ROTATE_ANGLE;
				
				if(Arm_OperateMode == Arm_Auto_Get_I_Egg||Arm_OperateMode == Arm_Auto_Get_I_Eggs)
				{
					ARM_RotateMotorRefAngle += ROTATE_ANGLE_DELTA;
				}
				

					if(FlipArm_Motor_Measure.ecd_angle > ARM_RotateMotorRefAngle-100)
					{
						if(xTaskGetTickCount() - arm_time_mark > 600)
						{
							arm_move_i++;		
						}
					}	
				
	
					
			}break;
			
			case ARM_ROTATE_AWAY_CMD:
			{						
				ARM_RotateMotorRefAngle = ROTATE_ANGLE_AWAY;
				
		
				
//				if(Egg_Box_Held == 1)//这一部是为了将弹药箱甩出去//完全可以用延时写
//				{
//					if(FlipArm_Motor_Measure.ecd_angle > ROTATE_ANGLE-500)//没达成目的
//					{
//							arm_move_i++;		
//					}	
//				}
//				else 
//				{
					if(FlipArm_Motor_Measure.ecd_angle > ROTATE_ANGLE_AWAY-50)
					{
						if(xTaskGetTickCount() - arm_time_mark > 400)
						{
							arm_move_i++;		
						}
					}	
//				}
				
	
					
			}break;
			
					case ARM_ROTATE_MAX_CMD:
			{
				ARM_RotateMotorRefAngle = ROTATE_ANGLE_MAX;
				
//				if(Arm_OperateMode == Arm_Auto_Get_I_Egg||Arm_OperateMode == Arm_Auto_Get_I_Eggs)
//				{
//					ARM_RotateMotorRefAngle += ROTATE_ANGLE_DELTA;
//				}
				

					if(FlipArm_Motor_Measure.ecd_angle > ROTATE_ANGLE_MAX-50)
					{
						if(xTaskGetTickCount() - arm_time_mark > 600)
						{
							arm_move_i++;		
						}
					}	

					
			}break;
			
			case ARM_ROTATE_M_CMD:
			{
				ARM_RotateMotorRefAngle = ROTATE_M_ANGLE;
				if(Arm_OperateMode == Arm_Auto_Get_I_Egg||Arm_OperateMode == Arm_Auto_Get_I_Eggs)
				{
					ARM_RotateMotorRefAngle += ROTATE_M_ANGLE_DELTA;
				}
				if(FlipArm_Motor_Measure.ecd_angle < ROTATE_M_ANGLE+100)
				{
		
					if(xTaskGetTickCount() - arm_time_mark > 500)
					{
						arm_move_i++;		
					}
				}	
			}break;
			case ARM_GIVE_CMD:
			{
				//舵机角度控制
				//这个目前不需要，到时候如果想要写的整齐一点再在这里加

				
			}break;
			case ARM_MOVE_END_CMD:
			{
					arm_move_i = 0;
					AutoMovement = Auto_NoMovement;//新增未测试
					Arm_OperateMode = Arm_KeyMouseMode;
			}break;
			case ARM_DELAY_CMD:
			{

				//单纯的给延时用的,目前是给底盘用
				if(xTaskGetTickCount() - arm_time_mark > 450)//
				{
					arm_move_i++;		
				}
				
			}break;
			case ARM_WAIT_CM_CMD:
			{
				//等待底盘编码器
			}break;
			default:
			{			
			}
			break;
		}
			if(last_arm_move_i != arm_move_i)//每一次状态变化,记录时间节点,在需要延时的动作中进行时延
			{
				arm_time_mark = xTaskGetTickCount();
			}
	
		last_arm_move_i = arm_move_i;

	}

}

float chm_kp = 22;//60

extern PID_Regulator_t LCM5PositionPID;
extern PID_Regulator_t LCM6PositionPID;
extern PID_Regulator_t AMRotatePositionPID;
extern PID_Regulator_t AMMovePositionPID;


float pos_kp = 10;



float arm_output_c =  1;

float rotate_pos_kp[5] = {4,8,5,5,10};
float rotate_speed_kp[5] = {2.5,30,20,10,5};
void Arm_Motor_Get_PID_Para(void)//以后变量命名以其用途位置等命名，不要以他是什么东西命名
{
	//以下是没有特殊动作时的PID参数
	
	LCM5SpeedPID.kp = LCM6SpeedPID.kp =chm_kp;//22;//chm_kp;
	LCM5PositionPID.kp = LCM6PositionPID.kp = 8;//pos_kp;调完了

	
	
	AMMovePositionPID.kp = rotate_pos_kp[2];
	AMMovePID.kp = rotate_speed_kp[2];//

	AMRotatePID.kp = 2;
	AMRotatePositionPID.kp = 8;//30;

	arm_output_c = 1;
	if(Arm_OperateMode == Arm_Auto_Get_Egg||Arm_OperateMode == Arm_Auto_Get_Eggs)
	{
		if(Arm_Move[arm_move_i] == ARM_ROTATE_I_CMD && Egg_Box_Held > 0)
		{
			AMRotatePID.kp = rotate_speed_kp[1];
			AMRotatePositionPID.kp = rotate_pos_kp[1];//30;
			arm_output_c = 2;
		}
		else if(Arm_Move[arm_move_i] == ARM_ROTATE_I_CMD && Egg_Box_Held == 0)
		{
			AMRotatePID.kp = 5;//
			AMRotatePositionPID.kp = 12;
			arm_output_c = 1;
		
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_AWAY_CMD)//抓取后翻回时，速度变慢，力量要大
		{
			
			AMRotatePID.kp = 5;
			arm_output_c = 1;
			AMRotatePositionPID.kp = 8;
		}
		
		if(Arm_Move[arm_move_i] == ARM_ROTATE_H_CMD&&Egg_Box_Held == 1)
		{
			AMRotatePID.kp = 5;//
			AMRotatePositionPID.kp = 14;
			arm_output_c = 2;
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_M_CMD)//抓取后翻回时，速度变慢，力量要大
		{
			
			AMRotatePID.kp = rotate_speed_kp[3];
			AMRotatePositionPID.kp = rotate_pos_kp[3];
			arm_output_c = 2;
		}
		
		if(Arm_Move[arm_move_i] == ARM_ROTATE_O_CMD &&Egg_Box_Held == 0)//出去夹时要慢
		{
			AMRotatePID.kp = rotate_speed_kp[0];
			AMRotatePositionPID.kp = rotate_pos_kp[0];//角度大于目标值时,有可能是kp过大超调
			arm_output_c = 1;
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_O_CMD&&Egg_Box_Held > 0)//出去翻时要用力
		{
			AMRotatePID.kp = 5;
			arm_output_c = 1;
			AMRotatePositionPID.kp = 8;
		}
		
	}
	else if(Arm_OperateMode == Arm_Auto_Get_I_Egg)//还未调
	{
		//以下为20个时pid等参数
		if(Arm_Move[arm_move_i] == ARM_ROTATE_I_CMD && Egg_Box_Held > 0)
		{
			AMRotatePID.kp = rotate_speed_kp[1];
			AMRotatePositionPID.kp = rotate_pos_kp[1];//30;
			arm_output_c = 2;
		}
		else if(Arm_Move[arm_move_i] == ARM_ROTATE_I_CMD && Egg_Box_Held == 0)
		{
			AMRotatePID.kp = 5;//
			AMRotatePositionPID.kp = 12;
			arm_output_c = 1;
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_M_CMD)//抓取后翻回时，速度变慢，力量要大
		{
			
			AMRotatePID.kp = 8;//18;
			AMRotatePositionPID.kp = 20;//40;
			arm_output_c = 9;
			
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_AWAY_CMD)//抓取后翻回时，速度变慢，力量要大
		{
			
			AMRotatePID.kp = 5;
			arm_output_c = 1;
			AMRotatePositionPID.kp = 8;
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_O_CMD &&Egg_Box_Held == 0)//出去夹时要慢
		{
			AMRotatePID.kp = rotate_speed_kp[0];
			AMRotatePositionPID.kp = rotate_pos_kp[0];//角度大于目标值时,有可能是kp过大超调
			arm_output_c = 1;
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_O_CMD&&Egg_Box_Held > 0)//出去翻时要用力
		{
			AMRotatePID.kp = 7;
			arm_output_c = 2;
			AMRotatePositionPID.kp = 15;
		}
	
	
	}
	
	if((Arm_Move[arm_move_i] == ARM_ROTATE_O_CMD || Arm_Move[arm_move_i] == ARM_ROTATE_MAX_CMD) &&Egg_Box_Held == 0)
	{
		AMRotatePID.kp = rotate_speed_kp[0];
		AMRotatePositionPID.kp = rotate_pos_kp[0];//角度大于目标值时,有可能是kp过大超调
		arm_output_c = 1;
	
	}
	
	
	AMMovePID.ki = LCM5SpeedPID.ki = LCM6SpeedPID.ki =AMRotatePID.ki = 0;
	AMMovePID.kd = LCM5SpeedPID.kd = LCM6SpeedPID.kd = AMRotatePID.kd = 0;
}



uint32_t  ref_nnn = 0;

void Arm_Cali_Output(void)
{
	if(Arm_OperateMode == Arm_KeyMouseMode)
	{
		PID_Task(&LCM5PositionPID,-ARM_LiftMotorRefAngle,LiftChain_Motor5_Measure.ecd_angle);//
		PID_Task(&LCM5SpeedPID,LCM5PositionPID.output,LiftChain_Motor5_Measure.speed_rpm/3);//其实这里都应当除以减速比的,现在这样不太好调PID

		PID_Task(&LCM6PositionPID, ARM_LiftMotorRefAngle,LiftChain_Motor6_Measure.ecd_angle);//
		PID_Task(&LCM6SpeedPID,LCM6PositionPID.output,LiftChain_Motor6_Measure.speed_rpm/3);//
		
		PID_Task(&AMRotatePositionPID,0,FlipArm_Motor_Measure.ecd_angle);//都为0
		PID_Task(&AMRotatePID,AMRotatePositionPID.output,FlipArm_Motor_Measure.speed_rpm/3);//
		
		PID_Task(&AMMovePositionPID,0,MoveArm_Motor_Measure.ecd_angle);//应当是锁定在0的位置,这样写不知会不会出错
		PID_Task(&AMMovePID,AMMovePositionPID.output,MoveArm_Motor_Measure.speed_rpm/3);//

	

	
	
	}
	else if(Arm_OperateMode == Arm_NormalRCMode)//此时速度环,目标值直接为速度
	{
		PID_Task(&LCM5SpeedPID,ARM_LiftMotorRefSpeed,LiftChain_Motor5_Measure.speed_rpm/3);//

		PID_Task(&LCM6SpeedPID,-ARM_LiftMotorRefSpeed,LiftChain_Motor6_Measure.speed_rpm/3);
		
		PID_Task(&AMMovePID,ARM_MoveMotorRefSpeed,MoveArm_Motor_Measure.speed_rpm/3);//

		PID_Task(&AMRotatePID,0,FlipArm_Motor_Measure.speed_rpm/10);//此时输出为零

		
		
		
	}
	else 
	{
		
		PID_Task(&LCM5PositionPID,-ARM_LiftMotorRefAngle,LiftChain_Motor5_Measure.ecd_angle);//
		PID_Task(&LCM5SpeedPID,LCM5PositionPID.output,LiftChain_Motor5_Measure.speed_rpm/3);//

		PID_Task(&LCM6PositionPID, ARM_LiftMotorRefAngle,LiftChain_Motor6_Measure.ecd_angle);//
		PID_Task(&LCM6SpeedPID,LCM6PositionPID.output,LiftChain_Motor6_Measure.speed_rpm/3);
		
		
		PID_Task(&AMRotatePositionPID,ARM_RotateMotorRefAngle,FlipArm_Motor_Measure.ecd_angle);//
		PID_Task(&AMRotatePID,AMRotatePositionPID.output,FlipArm_Motor_Measure.speed_rpm/3);//
		
		PID_Task(&AMMovePositionPID,ARM_TransMotorRefAngle,MoveArm_Motor_Measure.ecd_angle);//
		PID_Task(&AMMovePID,AMMovePositionPID.output,MoveArm_Motor_Measure.speed_rpm/3);//
	}

	

}


float arm_output = 0;
void Arm_Set_Output(void)
{
	arm_output = AMRotatePID.output*arm_output_c;
	if(arm_output>15000)//限制电调输出电流不得大于16384，大于时会不正常
	{
		arm_output = 15000;
	
	}
	else if(arm_output < -15000)
	{
		arm_output = -15000;
	
	}
	if(WorkState == STOP_STATE||WorkState == PREPARE_STATE||OperateMode == Stop_Mode || Arm_OperateMode== Arm_Locked)
	{
		CAN2_Send_LM(0,0,0,0);
	}
	else 
		CAN2_Send_LM(LCM5SpeedPID.output,LCM6SpeedPID.output,arm_output,AMMovePID.output);//AMRotatePID.output*2,AMMovePID.output);
}

uint32_t channel=0;

void Manipulator_Control(void)
{
	Arm_Movement_Split();
	ArmPart_Get_Movement();

	Arm_Motor_Get_PID_Para();//把pid赋值放在划定动作之后
	Arm_Cali_Output();
	Arm_Set_Output();
//调机械臂 调speed.kp pos.kp 和 pos.output speed.output

}

