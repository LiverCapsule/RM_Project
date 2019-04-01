#include "Driver_Manipulator.h"
#include "CanBusTask.h"
#include "Driver_Remote.h"
#include "DriverLib_PID.h"

typedef enum
{
	Arm_Locked,
	Arm_Get_Egg,
	Arm_Get_Eggs,
	Arm_Pull_Eggs,//岛上取弹两种策略，一种是先把全部拉回来，一种是拉回来就取
	Arm_Get_I_Egg,	
	Arm_Get_I_Eggs,
	Arm_Give_Egg,
}Arm_OperateMode_e;
Arm_OperateMode_e Arm_OperateMode;
#define ARM_PINCH_CMD 	1
#define ARM_UNPINCH_CMD 2
#define ARM_AHEAD_CMD 	3
#define ARM_BACK_CMD 		4
#define ARM_RAISE_CMD	  5//
#define ARM_HRAISE_CMD  6//抬到最高
#define ARM_LFALL_CMD		7//降到最低
#define ARM_FALL_CMD    8//
#define ARM_ROTATE_CMD  9//刚开始默认为收回状态
#define ARM_LEFT_CMD   10
#define ARM_RIGHT_CMD  11
#define ARM_MIDDLE_CMD 12
#define ARM_GIVE_CMD   13
uint8_t arm_move_i = 0;//机械臂的实时动作
uint8_t Arm_Fetch_Egg[40] = {ARM_MIDDLE_CMD,ARM_RAISE_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_RAISE_CMD,ARM_ROTATE_CMD,ARM_UNPINCH_CMD,ARM_LFALL_CMD};//定义为全局变量，此时除已赋值单元外，其余为单元为0
uint8_t Arm_Fetch_Eggs[40] ={ARM_MIDDLE_CMD,ARM_RAISE_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_CMD,\
ARM_UNPINCH_CMD,ARM_LEFT_CMD,ARM_FALL_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_MIDDLE_CMD,ARM_ROTATE_CMD,ARM_UNPINCH_CMD,\
ARM_RIGHT_CMD,ARM_FALL_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_MIDDLE_CMD,ARM_ROTATE_CMD,ARM_UNPINCH_CMD,\
ARM_LFALL_CMD};//未写完
uint8_t Arm_Fetch_I_Egg[40] = {ARM_MIDDLE_CMD,ARM_RAISE_CMD,ARM_AHEAD_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_BACK_CMD,\
ARM_ROTATE_CMD,ARM_UNPINCH_CMD,ARM_LFALL_CMD};


uint8_t Arm_Fetch_I_Eggs[40] ={ARM_MIDDLE_CMD,ARM_RAISE_CMD,ARM_AHEAD_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_BACK_CMD,\
ARM_ROTATE_CMD,ARM_UNPINCH_CMD,ARM_LEFT_CMD,ARM_AHEAD_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_BACK_CMD,\
ARM_MIDDLE_CMD,ARM_ROTATE_CMD,ARM_UNPINCH_CMD,ARM_RIGHT_CMD,ARM_FALL_CMD,ARM_AHEAD_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,\
ARM_HRAISE_CMD,ARM_BACK_CMD,ARM_MIDDLE_CMD,ARM_ROTATE_CMD,ARM_UNPINCH_CMD,ARM_LFALL_CMD};
uint8_t Arm_Pull_I_Eggs[40] = {ARM_MIDDLE_CMD,ARM_RAISE_CMD,ARM_AHEAD_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_BACK_CMD,\
ARM_HRAISE_CMD,ARM_LEFT_CMD,ARM_AHEAD_CMD,ARM_FALL_CMD,ARM_PINCH_CMD,ARM_BACK_CMD,ARM_HRAISE_CMD,ARM_RIGHT_CMD,ARM_AHEAD_CMD,\
ARM_FALL_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_BACK_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_CMD,ARM_MIDDLE_CMD};
	
	




//通过状态机获得

float ARM_RotateMotorRefAngle = 0;
float ARM_LiftMotorRefAngle = 0;
float ARM_TransMotorRefAngle = 0;

#include "CanbusTask.h"

//ARM_PINCH   	  
//ARM_UNPINCH   

//ARM_AHEAD				
//ARM_BACK		



float Motor_EcdAngleSet(float Target,Measure *mea)
{
	Target += mea->ecd_angle;
	return	Target;
}


////这两个翻转的函数包好了
//void Arm_Rotate_Pinch(void)//机械臂翻转并夹取
//{
//	if(FlipArm_Motor_Measure.angle/19 < 10)
//	{
//		ARM_PINCH;
//	}
//	else 
//	{
//		Motor_EcdAngleSet(-120,&FlipArm_Motor_Measure);//一定的角度
//	}
//	
//}


//void Arm_Rotate_Unpinch(void)//机械臂翻转并施放弹药箱
//{
//	if(FlipArm_Motor_Measure.angle/19 >110)
//	{
//		ARM_UNPINCH;
//	}
//	else 
//	{
//		Motor_EcdAngleSet(120,&FlipArm_Motor_Measure);//一定的角度
//	}

//}




//状态机中，如何检测退出该动作
uint8_t Arm_Move[40] = {0};
void Arm_Movement_Split(void)//步骤拆分开
{
	switch(Arm_OperateMode)
	{
		case Arm_Get_Egg:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = Arm_Fetch_Egg[move_n];
			}
		}break;
		case Arm_Get_Eggs:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = Arm_Fetch_Eggs[move_n];
			}
		}break;
		case Arm_Get_I_Egg:
		{
				for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = Arm_Fetch_I_Egg[move_n];
			}
		}break;
		case Arm_Get_I_Eggs:
		{
				for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = Arm_Fetch_I_Eggs[move_n];
			}
		}break;
		case Arm_Pull_Eggs:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = Arm_Pull_I_Eggs[move_n];
			}
		}break;
	
		case Arm_Give_Egg:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
//				Arm_Move[move_n] = ARM_GIVE_CMD;
			}
		
		}break;
		
		case Arm_Locked:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = 0;
			}
		}break;
		default:
			break;
	}
}


uint16_t time_tick_arm = 0;

void ArmPart_Get_Movement(void)
{//解算
	switch (Arm_Move[arm_move_i])
	{
		case ARM_PINCH_CMD:
		{
			ARM_PINCH;			//控制这个的电磁阀和弹弹药箱的电磁阀用同一个
			arm_move_i++;//夹住后不需要等待
		}break;
		case ARM_UNPINCH_CMD:
		{
			ARM_UNPINCH;
			arm_move_i++;//松开后也不需要等待
		}break;
		case ARM_AHEAD_CMD:
		{
			ARM_AHEAD;
			arm_move_i++;
		}break;
		case ARM_BACK_CMD:
		{
			ARM_BACK;
			arm_move_i++;
		//需要
		}break;
		case ARM_RAISE_CMD:
		{

			ARM_LiftMotorRefAngle = 600;
			if(LiftChain_Motor5_Measure.ecd_angle >400)
			{
				arm_move_i++;
			}	
		}break;
		case ARM_HRAISE_CMD:
		{
			ARM_LiftMotorRefAngle =1200;
			if(LiftChain_Motor5_Measure.ecd_angle >1000)
			{
				arm_move_i++;
			}	
		}break;
		case ARM_LFALL_CMD:
		{
			ARM_LiftMotorRefAngle = 0;
			if(LiftChain_Motor5_Measure.ecd_angle <200)
			{
				arm_move_i++;
			}	
		}break;
		case ARM_FALL_CMD:
		{
			ARM_LiftMotorRefAngle  = 600;
			if(LiftChain_Motor5_Measure.ecd_angle <800)
			{
				arm_move_i++;
			}	
		}break;
		case ARM_LEFT_CMD:
		{
			ARM_TransMotorRefAngle  = 0;//最左端为0值，以后写入flash，或开机自检
			if(MoveArm_Motor_Measure.ecd_angle <50)
			{
				arm_move_i++;
			}	
			
		}break;
		case ARM_MIDDLE_CMD:
		{
			ARM_LiftMotorRefAngle = 600;
			if(MoveArm_Motor_Measure.ecd_angle >500||MoveArm_Motor_Measure.ecd_angle <300)
			{
				arm_move_i++;
			}	
			
		}break;
		case ARM_RIGHT_CMD:
		{
			ARM_LiftMotorRefAngle = 1200;
			if(MoveArm_Motor_Measure.ecd_angle > 1000)
			{
				arm_move_i++;
			}	
			
		}break;
		case ARM_ROTATE_CMD:
		{
			if(FlipArm_Motor_Measure.ecd_angle > 120)
			{			
				ARM_RotateMotorRefAngle = 0;
			}
			else if(FlipArm_Motor_Measure.ecd_angle < 20)
				ARM_RotateMotorRefAngle = 120;

			if(FlipArm_Motor_Measure.ecd_angle < 30||(FlipArm_Motor_Measure.ecd_angle >100))
			{
				arm_move_i++;
			}
		}break;
		case ARM_GIVE_CMD:
		{
			//舵机角度控制
			
		}break;
		default:
		{			
			ARM_UNPINCH;
			ARM_LiftMotorRefAngle = 0;
			ARM_TransMotorRefAngle = 800;
			ARM_RotateMotorRefAngle = 0;
			ARM_BACK;
			
		}
		break;
	}
}

float chm_kp = 5;//60


void Arm_Motor_Get_PID_Para(void)//以后变量命名以其用途位置等命名，不要以他是什么东西命名
{
	AMMovePID.kp = LCM5SpeedPID.kp = LCM6SpeedPID.kp =chm_kp;
	AMRotatePID.kp = 10;//60
	AMMovePID.ki = LCM5SpeedPID.ki = LCM6SpeedPID.ki =AMRotatePID.ki = 0;
	AMMovePID.kd = LCM5SpeedPID.kd = LCM6SpeedPID.kd = AMRotatePID.kd = 0;
}
	
void Arm_Cali_Output(void)
{
	PID_Task(&LCM5SpeedPID,ARM_LiftMotorRefAngle,LiftChain_Motor5_Measure.speed_rpm/10.0);//float /10.0
	PID_Task(&LCM6SpeedPID,-ARM_LiftMotorRefAngle,LiftChain_Motor6_Measure.speed_rpm/10.0);
	PID_Task(&AMMovePID,ARM_TransMotorRefAngle,MoveArm_Motor_Measure.speed_rpm/10.0);
	PID_Task(&AMRotatePID,ARM_RotateMotorRefAngle,FlipArm_Motor_Measure.speed_rpm/10.0);


	if(LCM5SpeedPID.output>-300 && LCM5SpeedPID.output<300 ) LCM5SpeedPID.output=0;//我们这样是不是就不用加ki就可以停在斜坡上了
	if(LCM6SpeedPID.output>-300 && LCM6SpeedPID.output<300 ) LCM6SpeedPID.output=0;
	if(AMMovePID.output>-300 && AMMovePID.output<300 ) AMMovePID.output=0;
	if(AMRotatePID.output>-300 && AMRotatePID.output<300 ) AMRotatePID.output=0;
}



void Arm_Set_Output(void)
{
//	if(
	CAN1_Send_LM(LCM5SpeedPID.output,LCM6SpeedPID.output,AMRotatePID.output,AMMovePID.output);



}

void Manipulator_Control(void)
{
	
	Arm_Motor_Get_PID_Para();
	Arm_Movement_Split();
	ArmPart_Get_Movement();
	Arm_Cali_Output();
	Arm_Set_Output();
}


