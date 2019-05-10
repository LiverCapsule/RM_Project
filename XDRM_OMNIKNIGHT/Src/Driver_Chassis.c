#include "Driver_Chassis.h"
#include "CanBusTask.h"
#include "DriverLib_PID.h"
#include "Driver_Remote.h"
#include "math.h"
#include "config.h"
#include "Driver_Sensor.h"
#include "StatusMachine.h"
#include "DriverLib_Ramp.h"
#include "imu.h"

//#define M_PI  (float)3.1415926535f
#define Ang2Rad(m)  (m/180.0f*M_PI)

ChassisDataTypeDef ChassisData;
ChassisMode_e ChassisMode;


float CM_SPEED_C = 1;
float CM_OMEGA_C = 1;

extern WorkState_e 	WorkState;

uint8_t Foward_C_flag = 0;
uint8_t Back_C_flag = 0;
extern uint8_t BM_AngleGet;
extern uint8_t steps_down;

uint8_t kk1 = 60;
uint8_t kk2 = 60;
uint8_t kk3 = 60;
uint8_t kk4 = 60;


void CM_Get_PID(void)
{
	CMRotatePID.kp =15;

	CM1SpeedPID.kp = kk1;
	CM2SpeedPID.kp = kk2;
	CM3SpeedPID.kp = kk3;
	CM4SpeedPID.kp = kk4;

	CM1SpeedPID.ki = CM2SpeedPID.ki = CM3SpeedPID.ki = CM4SpeedPID.ki = 0;
	CM1SpeedPID.kd = CM2SpeedPID.kd = CM3SpeedPID.kd = CM4SpeedPID.kd = 0;//

	CM1SpeedPID.err[2] =  CM2SpeedPID.err[2] = CM3SpeedPID.err[2] = CM4SpeedPID.err[2] = 0;
	CM1SpeedPID.ki = CM2SpeedPID.ki = CM3SpeedPID.ki = CM4SpeedPID.ki = 0;

}



uint32_t time_X = 0;
void 	CM_Calc_Output(void)
{
	PID_Task(&CM1SpeedPID,ChassisData.ChassisWheelSpeedRef[0],Chassis_Motor1_Measure.speed_rpm/10.0);//float /10.0
	PID_Task(&CM2SpeedPID,ChassisData.ChassisWheelSpeedRef[1],Chassis_Motor2_Measure.speed_rpm/10.0);
	PID_Task(&CM3SpeedPID,ChassisData.ChassisWheelSpeedRef[2],Chassis_Motor3_Measure.speed_rpm/10.0);
	PID_Task(&CM4SpeedPID,ChassisData.ChassisWheelSpeedRef[3],Chassis_Motor4_Measure.speed_rpm/10.0);


	if(CM1SpeedPID.output>-700 && CM1SpeedPID.output<700 ) CM1SpeedPID.output=0;//���������ǲ��ǾͲ��ü�ki�Ϳ���ͣ��б������
	if(CM2SpeedPID.output>-700 && CM2SpeedPID.output<700 ) CM2SpeedPID.output=0;
	if(CM3SpeedPID.output>-700 && CM3SpeedPID.output<700 ) CM3SpeedPID.output=0;
	if(CM4SpeedPID.output>-700 && CM4SpeedPID.output<700 ) CM4SpeedPID.output=0;
	
}



/**
  * @brief  ����3���ٶ�ֵ�������ӵ��ٶ�
  * @param  float vx X�����ϵ��ٶ�
  * @param  float vy Y�����ϵ��ٶ�
  * @param  float omega ��ת���ٶ�
  * @param  float radian ��ʱ���������ĽǶ�(����)��˳ʱ��Ϊ����
  * @param  int16_t maxspped  ����ٶ�
  * @retval None
  */
static void CalculateWheelSpeed(float vx, float vy, float omega, float radian, uint16_t maxspeed)//
{
	float   fWheelSpd[5];
	float	Chassis_forward_back_ref;		//��ս������ϵ�Ĳο��ٶ�
	float	Chassis_left_right_ref;
	float 	fMaxSpd = 0;
	int16_t s16_WheelSpd[5];
	Chassis_forward_back_ref = vy*cos(radian)+vx*sin(radian);		
	Chassis_left_right_ref   = vx*cos(radian)-vy*sin(radian);
	//���ֽ���
	fWheelSpd[0] = -Chassis_forward_back_ref + Chassis_left_right_ref + omega;	//���Ͻǵ����ʼ��ʱ����
	fWheelSpd[1] =  Chassis_forward_back_ref + Chassis_left_right_ref + omega;
	fWheelSpd[2] =  Chassis_forward_back_ref - Chassis_left_right_ref + omega;
	fWheelSpd[3] = -Chassis_forward_back_ref - Chassis_left_right_ref + omega;

	  
	//���� �ҵ��ٶ����ֵ
	fMaxSpd = fabs(fWheelSpd[0]);		
	if(fabs(fWheelSpd[1]) > fMaxSpd)
		fMaxSpd = fabs(fWheelSpd[1]);
	if(fabs(fWheelSpd[2]) > fMaxSpd)
		fMaxSpd = fabs(fWheelSpd[2]);
	if(fabs(fWheelSpd[3]) > fMaxSpd)
		fMaxSpd = fabs(fWheelSpd[3]);
  
	//�����������ٶ�
	if(fMaxSpd > maxspeed)
	{
		s16_WheelSpd[0]   = (int16_t)(fWheelSpd[0]*(maxspeed/fMaxSpd));
		s16_WheelSpd[1]   = (int16_t)(fWheelSpd[1]*(maxspeed/fMaxSpd));
		s16_WheelSpd[2]   = (int16_t)(fWheelSpd[2]*(maxspeed/fMaxSpd));
		s16_WheelSpd[3]   = (int16_t)(fWheelSpd[3]*(maxspeed/fMaxSpd));
	}
	else
	{
		s16_WheelSpd[0]   = (int16_t)fWheelSpd[0]; 
		s16_WheelSpd[1]   = (int16_t)fWheelSpd[1];
		s16_WheelSpd[2]   = (int16_t)fWheelSpd[2];
		s16_WheelSpd[3]   = (int16_t)fWheelSpd[3];
	}
	memcpy((void*)ChassisData.ChassisWheelSpeedRef, (void*)s16_WheelSpd, 8);
	
	
	
	
	
}


void Key2Speed(int16_t FB, int16_t LR)
{
		//����ģʽ
	int16_t tmp_FB = 0;
	int16_t tmp_LR = 0;

	tmp_FB = FB ;
	tmp_LR = LR ;


	if(Remote_CheckJumpKey(KEY_W) == 1&&RC_CtrlData.mouse.press_r == 0 &&RC_CtrlData.mouse.press_l == 0)
	{
		ChassisData.ChassisSpeedRef.Y = -tmp_FB*FBSpeedRamp.Calc(&FBSpeedRamp);
	}
	else if(Remote_CheckJumpKey(KEY_S) == 1&&RC_CtrlData.mouse.press_r == 0 &&RC_CtrlData.mouse.press_l == 0)
	{
		ChassisData.ChassisSpeedRef.Y = tmp_FB*FBSpeedRamp.Calc(&FBSpeedRamp);
	}
	else
	{
		FBSpeedRamp.ResetCounter(&FBSpeedRamp);
		FBSpeedRamp.SetCounter(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT *0.2);
		ChassisData.ChassisSpeedRef.Y = 0;
	}

	if(Remote_CheckJumpKey(KEY_D) == 1&&RC_CtrlData.mouse.press_r == 0 &&RC_CtrlData.mouse.press_l == 0)
	{
		ChassisData.ChassisSpeedRef.X = tmp_LR*LRSpeedRamp.Calc(&LRSpeedRamp);
	}
	else if(Remote_CheckJumpKey(KEY_A) == 1&&RC_CtrlData.mouse.press_r == 0 &&RC_CtrlData.mouse.press_l == 0)
	{
		ChassisData.ChassisSpeedRef.X = -tmp_LR*LRSpeedRamp.Calc(&LRSpeedRamp);
	}
	else
	{
		LRSpeedRamp.ResetCounter(&LRSpeedRamp);
		LRSpeedRamp.SetCounter(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT *0.2);
		ChassisData.ChassisSpeedRef.X = 0;
	}
	
	
	
	
}


extern AutoMovement_e AutoMovement;
#include "Data_MiniPC.h"
float angle_save = 0;
extern Mini_PC_Data_t Mini_Pc_Data;
uint32_t tick_c_1 = 0;
uint32_t cm_timetick =0;
uint16_t underpan_acc = 30;
extern float angle_average;
uint8_t lift_flag_again = 0;
extern int8_t flag_gcm;//���ֺ͵��̵��ǰ������flag
int8_t flag_gcm_l = 0;
uint8_t lift_flag_again1 = 0;
void CM_Get_SpeedRef(void)
{
	if(ChassisMode == Chassis_Locked)
	{
		ChassisData.ChassisSpeedRef.Y = 0;
		ChassisData.ChassisSpeedRef.X = 0;
		ChassisData.ChassisSpeedRef.Omega  = 0;
		ChassisData.ChassisAngle = 0;
	}
	else if(ChassisMode == Chassis_NormalRCMode)
	{
		ChassisData.ChassisSpeedRef.Y = RC_CtrlData.rc.ch1;
		ChassisData.ChassisSpeedRef.X = RC_CtrlData.rc.ch0;
		ChassisData.ChassisSpeedRef.Omega  = RC_CtrlData.rc.ch2/2;
		ChassisData.ChassisAngle = 0;
		
		static int16_t Y_temp = 0;
		static int16_t X_temp = 0;
		//����ң�������ٶ�
		if(abs(RC_CtrlData.rc.ch1 - Y_temp) > underpan_acc)
		{
			if(RC_CtrlData.rc.ch1 >Y_temp && RC_CtrlData.rc.ch1 > 0)
				Y_temp+=underpan_acc;
			else if (RC_CtrlData.rc.ch1 <Y_temp && RC_CtrlData.rc.ch1 < 0)
				Y_temp-=underpan_acc;	
			else
				Y_temp = RC_CtrlData.rc.ch1;
		}
		else
		{
			Y_temp = RC_CtrlData.rc.ch1;
		}
		
		if(abs(RC_CtrlData.rc.ch0 - X_temp) > underpan_acc)
		{
			if(RC_CtrlData.rc.ch0 >X_temp && RC_CtrlData.rc.ch0 > 0)
				X_temp+=underpan_acc;
			else if (RC_CtrlData.rc.ch0 <X_temp && RC_CtrlData.rc.ch0 < 0)
				X_temp-=underpan_acc;	
			else
				X_temp = RC_CtrlData.rc.ch0;
		}
		else
		{
			X_temp = RC_CtrlData.rc.ch0;
		}
		ChassisData.ChassisSpeedRef.Y		= -Y_temp * CM_SPEED_C;
		ChassisData.ChassisSpeedRef.X   	= X_temp * CM_SPEED_C; 
		ChassisData.ChassisSpeedRef.Omega  = RC_CtrlData.rc.ch2/2 * CM_OMEGA_C;


	}
	else if(ChassisMode == Chassis_KeyMouseMode)
	{
		
		Key2Speed(NORMAL_FORWARD_BACK_SPEED, NORMAL_LEFT_RIGHT_SPEED);
		if(Remote_CheckJumpKey(KEY_SHIFT))
		{
				Key2Speed(HIGH_FORWARD_BACK_SPEED, HIGH_LEFT_RIGHT_SPEED);
		}
		//Key2Speed(LOW_FORWARD_BACK_SPEED, LOW_LEFT_RIGHT_SPEED);
		ChassisData.ChassisSpeedRef.Omega = RC_CtrlData.mouse.x*KEY_FACTOR_NORMAL;
		
	}
	else if(ChassisMode == 	Chassis_Auto_CaliForEgg)
	{
		ChassisData.ChassisSpeedRef.Y = -40;//�����赲�˳���,ϣ���ڻ�е��̧��ʱ�ܹ�ǰ��,�����ټ�һ��ǰ�ƶ��ٶ�
		if(Mini_Pc_Data.left_or_right < 112&&Mini_Pc_Data.left_or_right > 30)//-30 to 30 ��Ϊ�����м�ķ�Χ
		{
			ChassisData.ChassisSpeedRef.X = 70;
		}
		else if(Mini_Pc_Data.left_or_right < -30 &&Mini_Pc_Data.left_or_right > -112)
		{
		ChassisData.ChassisSpeedRef.X = -70;
		}
		else if(Mini_Pc_Data.left_or_right == 0)
		{
			ChassisData.ChassisSpeedRef.X = 0;
		}
	
		//������λ���ش�����Ϣ����������ǰ��ƽ��
	
	
	}
	else if(ChassisMode == Chassis_Auto_UpIsland)
	{
//		//���̺͵���һ����ǰ��
		
		if(InfraredState_back&& angle_average >11000)//��������⵽����̧������һ����������ʱ
		{
			
			angle_save = pitch_angle;//�����ϵ�ʱ�����ǵĽǶȣ��µ�ʱ�Զ����
			cm_timetick  = xTaskGetTickCount();
		}

		if(flag_gcm == 1)//���ʱ����ֵָ�Ӻ�ߺ��������ƽ��ʼ��ʱһ��ʱ��
		{								 //���̵�ʱ��Ҳ�þ�����
				ChassisData.ChassisSpeedRef.Y = -200;//
				ChassisData.ChassisSpeedRef.X = 0;
		}
		
		time_X = xTaskGetTickCount() - cm_timetick;
		if(time_X > 2000&&cm_timetick!=0)//������Ǻ����Ƿ�ֹʱ��տ�ʼ�ͼӣ�����4000
		{
			flag_gcm = 0;
			lift_flag_again = 1;
		}
		flag_gcm_l = flag_gcm;

	}
	else if(ChassisMode == Chassis_Auto_DownIsland)
	{
		
			ChassisData.ChassisSpeedRef.Omega = PID_Task(&CMRotatePID, angle_save, pitch_angle);//����ǵ�ʱ���ע���
			ChassisData.ChassisSpeedRef.Y = -200;//
			ChassisData.ChassisSpeedRef.X = 0;

			
			if(flag_gcm == -1)
			{
				ChassisData.ChassisSpeedRef.Y = 100;//
			}
			else if(flag_gcm == 0)
			{
				ChassisData.ChassisSpeedRef.Y = 0;//
			}

		
	}
	else
	{
		ChassisData.ChassisSpeedRef.Y = 0;
		ChassisData.ChassisSpeedRef.X = 0;
		ChassisData.ChassisSpeedRef.Omega  = 0;
		ChassisData.ChassisAngle = 0;
	
	}

	CalculateWheelSpeed(ChassisData.ChassisSpeedRef.X,\
										ChassisData.ChassisSpeedRef.Y,\
										ChassisData.ChassisSpeedRef.Omega,\
										Ang2Rad(ChassisData.ChassisAngle),30000);
}


void CM_Set_Current(void)
{
	if(ChassisMode == Chassis_Locked||WorkState == STOP_STATE||WorkState == PREPARE_STATE)//��д����Ϊ�˱���
	{
		CAN2_Send_CM(0,0,0,0);
	}
	else
		CAN2_Send_CM( CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, \
								 CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output, \
								 CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output, \
								 CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);		

}





void Chassis_Control(void)
{
	CM_Get_PID();
	CM_Get_SpeedRef();
	CM_Calc_Output();
	CM_Set_Current();

}



