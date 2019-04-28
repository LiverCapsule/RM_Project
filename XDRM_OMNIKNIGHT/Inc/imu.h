#ifndef __IMU_H
#define __IMU_H

#include "main.h"
#include "test_imu.h"
#include "cmsis_os.h"
#include "config.h"

void Init_Quaternion(void);
void IMU_getYawPitchRoll(volatile float * ypr); //������̬
void GetPitchYawGxGyGz(void);
void IMU_Task(void);
void IMU_Cali(void);



extern float MPUGyroX_Offset;
extern float MPUGyroY_Offset;
extern float MPUGyroZ_Offset;
extern int16_t MPU6050_FIFO[6][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
//extern int16_t HMC5883_FIFO[3][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ ע���Ŵ������Ĳ���Ƶ���������Ե����г�

extern volatile float angle[3];
extern volatile float yaw_angle,pitch_angle,roll_angle; //ʹ�õ��ĽǶ�ֵ
extern volatile float mygetqval[9];   

#endif


