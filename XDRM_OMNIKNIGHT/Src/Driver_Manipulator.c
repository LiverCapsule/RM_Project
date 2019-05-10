#include "Driver_Manipulator.h"
#include "CanBusTask.h"
#include "Driver_Remote.h"
#include "DriverLib_PID.h"



extern TIM_HandleTypeDef htim8;

#define ROTATE_M_ANGLE 1600			  //岛上为1000,岛下1700,岛上惯性大,所以反而更小
#define ROTATE_ANGLE 2700         //岛上为3100,岛下为2700,岛上会给足够的时间到达目标值,水平3300
#define ROTATE_M_ANGLE_DELTA -600
#define ROTATE_ANGLE_DELTA 400
//岛上取弹有足够的时间稳定//2500//这里也改了,未知原因2700//实际机械臂在电机编码器3300位置时达到水平,但是由于惯性会超前转动,所以在这里将目标值改小
#define ROTATE_H_ANGLE 2300

#define MOVE_ANGLE_R 0//25000
#define MOVE_ANGLE_M 0//现在去了左右自由度14000


#define LIFT_ANGLE 6800//左边目标为-值 指抬升到中间
#define LIFT_H_ANGLE 10400//8400//8300为岛上高度,此时气缸减震没气,若加了需要在调低//10400
#define LIFT_ANGLE_DELTA 1600

//岛上时整车高度一改,抬升高度和翻转角度都得改


Arm_OperateMode_e Arm_OperateMode;
#define ARM_PINCH_CMD 	1
#define ARM_UNPINCH_CMD 2
#define ARM_AHEAD_CMD 	3
#define ARM_BACK_CMD 		4
#define ARM_RAISE_CMD	  5//
#define ARM_HRAISE_CMD  6//抬到最高
#define ARM_LFALL_CMD		7//降到最低
#define ARM_FALL_CMD    8//
#define ARM_ROTATE_I_CMD  9//刚开始默认为收回状态,朝内
#define ARM_ROTATE_O_CMD  10//
#define ARM_ROTATE_M_CMD  11
#define ARM_LEFT_CMD   12
#define ARM_RIGHT_CMD  13
#define ARM_MIDDLE_CMD 14
#define ARM_GIVE_CMD   15
#define ARM_OPEN_CMD   16
#define ARM_ROTATE_H_CMD 17 //为了抬高弹药箱所作的翻转
uint8_t arm_move_i = 0;//机械臂的实时动作


//uint8_t Arm_Fetch_I_Egg2[40] = {ARM_MIDDLE_CMD,ARM_RAISE_CMD,ARM_AHEAD_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_BACK_CMD,\
//ARM_ROTATE_I_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD};//拉回来翻，不抬到最高
	
//uint8_t Arm_Pull_I_Eggs[40] = {ARM_MIDDLE_CMD,ARM_RAISE_CMD,ARM_AHEAD_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_BACK_CMD,\
//ARM_HRAISE_CMD,ARM_LEFT_CMD,ARM_AHEAD_CMD,ARM_FALL_CMD,ARM_PINCH_CMD,ARM_BACK_CMD,ARM_HRAISE_CMD,ARM_RIGHT_CMD,ARM_AHEAD_CMD,\
//ARM_FALL_CMD,ARM_ROTATE_CMD,ARM_PINCH_CMD,ARM_BACK_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_CMD,ARM_MIDDLE_CMD};


uint8_t Arm_Fetch_Egg[40] = {ARM_MIDDLE_CMD,ARM_RAISE_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD};//MIDLE//定义为全局变量，此时除已赋值单元外，其余为单元为0

																																																		//这里MIDDLE先删掉了						
uint8_t Arm_Fetch_Egg2[40] ={ARM_RIGHT_CMD,ARM_RAISE_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD,\
														 ARM_LEFT_CMD,ARM_RAISE_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_H_CMD,ARM_MIDDLE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD,\
														 ARM_LEFT_CMD};//取两个，当弹舱不够大，但是能脱离围栏(不一定)								//这个是平移是被卡住了
//夹取两个目前因为车高(不想给底部气缸充气，懒得试了)及机械臂抬升高度不够所以无法将弹药箱脱离围栏,往往是机械臂被卡住,底盘反方向动,还有一个解绝方案是夹第二个时,底盘跟着动
														 //现在不太想写了
//有一个问题是,机械臂的位置,1是重启问题,2是操作手夹取时,他的原位置3是操作手夹取时,他取了弹药箱后的翻转位置


														 
																														//先翻了，气缸等待
uint8_t Arm_Fetch_I_Egg[40] = {ARM_MIDDLE_CMD,ARM_RAISE_CMD,ARM_ROTATE_O_CMD,ARM_AHEAD_CMD,ARM_PINCH_CMD,\
	ARM_BACK_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD};//加了平移电机后MIDDLE_CMD需要注意一下
//这里的eggs指的是两个弹药箱
	
uint8_t Arm_Give_Egg[40] = {ARM_RAISE_CMD,ARM_OPEN_CMD,ARM_HRAISE_CMD};//


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

//				Arm_Move[move_n] = Arm_Pull_I_Eggs[move_n];
			}
		}break;
	
		case Arm_Auto_Give_Egg:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
					Arm_Move[move_n] = 0;
			}
			ARM_LiftMotorRefAngle = LIFT_ANGLE;
			TIM8->CCR1 = 1850;
			
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
				ARM_LiftMotorRefAngle = 0;
			}
			
			ARM_RotateMotorRefAngle = 0;
			ARM_TransMotorRefAngle = 0;
		}
		default:
			break;
	}
}


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
				arm_move_i++;//夹住后不需要等待
			}break;
			case ARM_UNPINCH_CMD:
			{
				ARM_UNPINCH;
				Egg_Box_Held --;
				arm_move_i++;//松开后也不需要等待
			}break;
			
			
			
			case ARM_AHEAD_CMD://气缸前进时速度较慢，需要在进入下一个动作之前有一个延时
			{
				ARM_AHEAD;
				
				if(xTaskGetTickCount() - arm_time_mark > 1000)//这个可以正常使用
				{
					arm_move_i++;		
				}

				
			}break;
			case ARM_BACK_CMD:
			{
				ARM_BACK;
				
				if(xTaskGetTickCount() - arm_time_mark > 1000)
				{
					arm_move_i++;		
				}
				

		
				
			}break;
			case ARM_RAISE_CMD:
			{

				ARM_LiftMotorRefAngle = LIFT_ANGLE;
				if(Arm_OperateMode == Arm_Auto_Get_I_Egg||Arm_OperateMode == Arm_Auto_Get_I_Eggs)
				{
					ARM_LiftMotorRefAngle += LIFT_ANGLE_DELTA;
				}

				if(arm_average_angle >LIFT_ANGLE - 300)//减一个值是为了提早进入下一动作，而这一动作又能继续进行
				{
					arm_move_i++;
				}	
			}break;
			case ARM_HRAISE_CMD:
			{
				ARM_LiftMotorRefAngle =LIFT_H_ANGLE;
				if(arm_average_angle >LIFT_H_ANGLE -300)//
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
				ARM_TransMotorRefAngle  = 0;//最左端为0值，以后写入flash，或开机自检
				if(MoveArm_Motor_Measure.ecd_angle <2000)
				{
					arm_move_i++;
				}	
				
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
				ARM_TransMotorRefAngle = MOVE_ANGLE_R;
				if(MoveArm_Motor_Measure.ecd_angle > MOVE_ANGLE_R - 400)
				{
					arm_move_i++;
				}	
				
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
				
//				if(Egg_Box_Held == 1)//这一部是为了将弹药箱甩出去//完全可以用延时写
//				{
//					if(FlipArm_Motor_Measure.ecd_angle > ROTATE_ANGLE-500)//没达成目的
//					{
//							arm_move_i++;		
//					}	
//				}
//				else 
//				{
					if(FlipArm_Motor_Measure.ecd_angle > ROTATE_ANGLE-50)
					{
							arm_move_i++;		
					}	
//				}
				
	
			}break;
			case ARM_ROTATE_H_CMD:
			{
				ARM_RotateMotorRefAngle = ROTATE_H_ANGLE;
				
				if(FlipArm_Motor_Measure.ecd_angle < ROTATE_H_ANGLE+100)
				{
					arm_move_i++;
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
		
					arm_move_i++;
				}	
			}break;
			case ARM_GIVE_CMD:
			{
				//舵机角度控制
				//这个目前不需要，到时候如果想要写的整齐一点再在这里加

				
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

float rotate_pos_kp[5] = {10,8,5,10,10};
float rotate_speed_kp[5] = {5,30,30,20,5};
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
			arm_output_c = 4;
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
			AMRotatePositionPID.kp = 15;
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
			AMRotatePositionPID.kp = 15;
		}
	
	
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
		PID_Task(&LCM5SpeedPID,LCM5PositionPID.output,LiftChain_Motor5_Measure.speed_rpm/3);//

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
	if(arm_output>15000)
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

