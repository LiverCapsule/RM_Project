/**
  *@file test_imu.c
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
#include "test_imu.h"
#include "SuperviseTask.h"

#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)


float MPUGyroX_Offset = 0;
float MPUGyroY_Offset = 0;
float MPUGyroZ_Offset = 0;

MagMaxMinData_t MagMaxMinData;
volatile MPU6050_RAW_DATA    MPU6050_Raw_Data;    //原始数据
volatile MPU6050_REAL_DATA   MPU6050_Real_Data;
int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值
uint8_t MPU_id = 0;
uint8_t isMPU6050_is_DRY = 0;

IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};

IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg&0x7f;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 10);
  MPU_Tx = data;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 10);
  
  MPU6500_NSS_High();
  return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg|0x80;

  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 10);
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 10);
  
  MPU6500_NSS_High();
  return MPU_Rx;
}

//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
	
  static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
  MPU6500_NSS_Low();

  MPU_Tx = regAddr|0x80;
  MPU_Tx_buff[0] = MPU_Tx;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 10);
  HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, pData, len, 10);

  MPU6500_NSS_High();
  return 0;
}

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  HAL_Delay(10);
}

//Write IST8310 register through MPU6500
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  HAL_Delay(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  return data;
}

//Initialize the MPU6500 I2C Slave0 for I2C reading
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  HAL_Delay(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  HAL_Delay(7);
}

//Initialize the IST8310
uint8_t IST8310_Init(void)
{
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  HAL_Delay(100);
  return 0;
}



extern uint8_t imu_flag;
//Get 6 axis data from MPU6500
void IMU_Get_Data()
{
  uint8_t mpu_buff[14];
  
  MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
  
  imu_data.ax = mpu_buff[0]<<8 |mpu_buff[1];
  imu_data.ay = mpu_buff[2]<<8 |mpu_buff[3];
  imu_data.az = mpu_buff[4]<<8 |mpu_buff[5];
  
  imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
  
  imu_data.gx = (mpu_buff[8]<<8 |mpu_buff[9]) - MPUGyroX_Offset;
  imu_data.gy = (mpu_buff[10]<<8 |mpu_buff[11]) - MPUGyroY_Offset;
  imu_data.gz = (mpu_buff[12]<<8 |mpu_buff[13]) - MPUGyroZ_Offset;
  if(imu_flag == 1)
	{
//		imu_data.gx = KalmanFilter(p,imu_data.gx);
	}		
  
}

void MPU6050_AverageFilter(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]为最近10次数据 [10]为10次数据的平均值
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++)
	{
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	
	MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
	
}


void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) 
{
	
	if(isMPU6050_is_DRY)
	{
		isMPU6050_is_DRY = 0;
		IMU_Get_Data();
//		HMC58X3_ReadData(&(mpu_buf[14]));  //14-19为陀螺仪数据
		MPU6050_AverageFilter(imu_data.ax, imu_data.ay, imu_data.az, imu_data.gx, imu_data.gy, imu_data.gz);  		
		*ax  =MPU6050_FIFO[0][10];
		*ay  =MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];
		*gx  =MPU6050_FIFO[3][10];
		*gy = MPU6050_FIFO[4][10];
		*gz = MPU6050_FIFO[5][10];
		
	} 
	else
	{
		
		*ax =MPU6050_FIFO[0][10];
		*ay =MPU6050_FIFO[1][10];
		*az =MPU6050_FIFO[2][10];
		*gx =MPU6050_FIFO[3][10];
		*gy =MPU6050_FIFO[4][10];
		*gz =MPU6050_FIFO[5][10];
	}
	
}

//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}



//Initialize the MPU6500
uint8_t MPU6500_Init(void)
{	//初始化NSS引脚
	GPIO_InitTypeDef gpio;
	gpio.Pin = GPIO_PIN_6;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOF, &gpio);
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[11][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x10},      // +-1000dps
//	{MPU6500_GYRO_CONFIG,   0x08},      // +-500dps
    {MPU6500_ACCEL_CONFIG,  0x01},      // +-2G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    {MPU6500_USER_CTRL,     0x20},      // Enable AUX
	{MPU6500_INT_PIN_CFG,	0x02},		
	{MPU6500_INT_ENABLE,	0x01}
  };
  
  HAL_Delay(100);
  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
  
  for(index = 0; index < 10; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
	HAL_Delay(1);//这个Delay很重要
  }

  return 0;
}


