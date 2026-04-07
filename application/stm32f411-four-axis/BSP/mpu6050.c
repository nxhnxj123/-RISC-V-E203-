#include "mpu6050.h"
#include "i2c.h"

/**
  * @brief   д���ݵ�MPU6050�Ĵ���
  * @param   
  * @retval  
  */
void MPU6050_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
	uint8_t ack;
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	ack=i2c_WaitAck();
	i2c_SendByte(reg_add);
	ack=i2c_WaitAck();
	i2c_SendByte(reg_dat);
	ack=i2c_WaitAck();
	i2c_Stop();
}

/**
  * @brief   ��MPU6050�Ĵ�����ȡ����
  * @param   
  * @retval  
  */
void MPU6050_ReadData(uint8_t reg_add,unsigned char*Read,uint8_t num)
{
	unsigned char i;
	uint8_t ack;
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	ack=i2c_WaitAck();
	i2c_SendByte(reg_add);
	ack=i2c_WaitAck();
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS | 0x01);
	ack=i2c_WaitAck();
	
	for(i=0;i<(num-1);i++){
		*Read=i2c_ReadByte(1);
		Read++;
	}
	*Read=i2c_ReadByte(0);
	i2c_Stop();
}

/**
  * @brief   ��ʼ��MPU6050оƬ
  * @param   
  * @retval  
  */
void MPU6050_Init(void)
{
	i2c_GPIO_Config();
	    delay_ms_volatile(100); // 确保上电复位时间足够

	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);      // 解除休眠
	delay_ms_volatile(50); // 【修正】使用正确的函数名״̬
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //�����ǲ����ʣ�1KHz
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	        //��ͨ�˲���20hz
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);	  //���ü��ٶȴ����������ڡ�4gģʽ�����Լ�
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //�������Լ켰������Χ,2000deg/s
}

/**
  * @brief   ��ȡMPU6050��ID
  * @param   
  * @retval  
  */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
  MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //��������ַ
	/*
	if(Re != 0x68)
	{
		return 0;
	}
	else
	{
		return 1;
	}*/
	return Re;
}

/**
  * @brief   ��ȡMPU6050�ļ��ٶ�����
  * @param   
  * @retval  
  */
void MPU6050ReadAcc(short *accData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (short)((buf[0] << 8) | buf[1]);
    accData[1] = (short)((buf[2] << 8) | buf[3]);
    accData[2] = (short)((buf[4] << 8) | buf[5]);
}

/**
  * @brief   ��ȡMPU6050�ĽǼ��ٶ�����
  * @param   
  * @retval  
  */
void MPU6050ReadGyro(short *gyroData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (short)((buf[0] << 8) | buf[1]);
    gyroData[1] = (short)((buf[2] << 8) | buf[3]);
    gyroData[2] = (short)((buf[4] << 8) | buf[5]);
}

/**
  * @brief   ��ȡMPU6050��ԭʼ�¶�����
  * @param   
  * @retval  
  */
void MPU6050ReadTemp(short *tempData)
{
	uint8_t buf[2];
    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
    *tempData = (buf[0] << 8) | buf[1];
}

/**
  * @brief   ��ȡMPU6050���¶����ݣ�ת�������϶�
  * @param   
  * @retval  
  */
void MPU6050_ReturnTemp(float *Temperature)
{
	short temp3;
	uint8_t buf[2];
	
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
	temp3= (buf[0] << 8) | buf[1];	
	*Temperature=((double) temp3/340.0)+36.53;

}
void delay_ms_volatile(int ms) {
    for(volatile int i = 0; i < ms; i++) {
        for(volatile int j = 0; j < 5000; j++); // 根据主频调整
    }
}
