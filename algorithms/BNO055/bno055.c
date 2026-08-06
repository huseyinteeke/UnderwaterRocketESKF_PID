/*
 * bno055.c
 *
 *  Created on: Nov 21, 2025
 *      Author: husey
 */


#include "bno055_func_struct.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"



/*
 * Private functions
 */

/*-----------------------------------------------------------------------------------------------------------------------------*/

static HAL_StatusTypeDef BNO_WriteReg(BNO055Init_TypeDef_t *dev, uint8_t reg,
		uint8_t value) {
    HAL_I2C_Mem_Write(dev->i2cHandler, dev->i2cAddress, reg ,
      1 , &value , 1 , dev->i2cTimeout );
    return HAL_OK;
}

static uint8_t BNO_ReadReg(BNO055Init_TypeDef_t *dev, uint8_t reg) {
	static uint8_t value = 0;
	HAL_I2C_Mem_Read(dev->i2cHandler, dev->i2cAddress, reg , 1,
	    &value, 1 , dev->i2cTimeout);
	return value;
}

static HAL_StatusTypeDef BNO_WriteMulti(BNO055Init_TypeDef_t *dev, uint8_t reg,
		uint8_t *data, uint8_t len) {

    HAL_I2C_Mem_Write(dev->i2cHandler, dev->i2cAddress, reg, 1, data,
      len , dev->i2cTimeout);
    return HAL_OK;
    }


static HAL_StatusTypeDef BNO_ReadMulti(BNO055Init_TypeDef_t *dev, uint8_t reg,
		uint8_t *data, uint8_t len){
    return HAL_I2C_Mem_Read_DMA(dev->i2cHandler , dev->i2cAddress , reg ,
        1 , data , len );
}


static void BNO_SetPage(BNO055Init_TypeDef_t *dev, uint8_t page) {
	BNO_WriteReg(dev, BNO055_PAGE_ID0, page); // 0x07
}






/*----------------------------------------------------------------------------------------------------------------------------*/
/*
 * remap eksenleri için register değerleri
 */
static const uint8_t REMAP_CONFIG_TABLE[8][2] = {
		{ 0x21, 0x04 }, // P0 (Default)
		{ 0x24, 0x00 }, // P1
		{ 0x24, 0x06 }, // P2
		{ 0x21, 0x02 }, // P3
		{ 0x24, 0x03 }, // P4
		{ 0x21, 0x01 }, // P5
		{ 0x21, 0x07 }, // P6
		{ 0x24, 0x05 }  // P7
};
//********DELAY WRAPPER**************//
static inline void BNO_Delay(BNO055Init_TypeDef_t *dev , uint32_t ms){
  if(dev->delayCallback != NULL) dev->delayCallback(ms);
  else HAL_Delay(ms);
}


/*------------------------------------------------------------------------------------------------------------------------*/
/*										SENSOR INIT FUNCTION															  */
/*------------------------------------------------------------------------------------------------------------------------*/
/*
 * @brief Initialize the BNO055 sensor
 * @param device specific definition structure
 * @retrieval BNO_Status_t
 */

BNO_Status_t BNO055_Init(BNO055Init_TypeDef_t *dev) {
  HAL_StatusTypeDef status = HAL_ERROR;


  #ifdef HW_PCB
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, RESET);
  BNO_Delay(dev , 20); 
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, SET);
  BNO_Delay(dev , 700);
  #endif

  BNO_Delay(dev , 650);

  status = HAL_I2C_IsDeviceReady(dev->i2cHandler, dev->i2cAddress , 3, 100);

	// 1.2. Goto page 0
 	BNO_SetPage(dev, 0);
  BNO_Delay(dev, 10);
	// 1.3. Chip ID check (0xA0)
	
  uint8_t id = BNO_ReadReg(dev, BNO055_CHIP_ID);
	BNO_Delay(dev , 20);

	for(int i = 0 ; i < 10 ; i ++){
		if(id == 0xA0) break;

		id = BNO_ReadReg(dev, BNO055_CHIP_ID);
		BNO_Delay(dev , 30);
	}
    

	if(id != 0xA0) return BNO_ID_ERROR;

	BNO_WriteReg(dev, BNO055_SYS_TRIGGER, 0x20);
	BNO_Delay(dev , 800);

	BNO_WriteReg(dev, BNO055_OPR_MODE, BNO_MODE_CONFIG);
	BNO_Delay(dev , 20);
	BNO_WriteReg(dev, BNO055_PWR_MODE, dev->powerMode);
	BNO_Delay(dev , 10);

	uint8_t units_reg = 0;
	units_reg |= (dev->accelUnit << 0);
	units_reg |= (dev->gyroUnit << 1);
	units_reg |= (dev->eulerUnit << 2);
	units_reg |= (dev->tempUnit << 4);
	units_reg |= (1 << 7);

	BNO_WriteReg(dev, BNO055_UNIT_SEL, units_reg);


	uint8_t p_idx = dev->axisRemap;
	p_idx &= 0x07; // Hata koruması

	BNO_WriteReg(dev, BNO055_AXIS_MAP_CNFG, REMAP_CONFIG_TABLE[p_idx][0]);
	BNO_WriteReg(dev, BNO055_AXIS_MAP_SIGN, REMAP_CONFIG_TABLE[p_idx][1]);

	uint8_t trigger = BNO_ReadReg(dev, BNO055_SYS_TRIGGER);
	trigger &= ~0x80;

	if (dev->externalCrystal == BNO_CLK_EXTERNAL) {
		trigger |= 0x80;
	}
	BNO_WriteReg(dev, BNO055_SYS_TRIGGER, trigger);

	BNO_Delay(dev ,10);
	if (dev->useStoredCalibration) {
		BNO_WriteMulti(dev, BNO055_ACC_OFFSETX_LSB, dev->calibrationData, 22);
		BNO_Delay(dev ,100);
	}
	BNO_WriteReg(dev, BNO055_OPR_MODE, BNO_MODE_CONFIG);
	BNO_Delay(dev ,20);
	BNO_WriteReg(dev, BNO055_OPR_MODE, dev->operationMode);
	BNO_Delay(dev , 30);
	uint8_t sys_stat = BNO_ReadReg(dev, BNO055_SYS_STAT);
	if (sys_stat == 0x01) {

		uint8_t err =BNO_ReadReg(dev , BNO055_SYS_ERR);
		dev->lastError = err;
		return BNO_SYS_ERROR;
	}
	return BNO_OK;
}







/*----------------------------------SENSOR DATA FUNCTIONS---------------------------*/




/*
 * Read accelometer function
 * @param device struct addr
 * @param data struct addr
 * @retrieval none
 */

BNO_Status_t BNO055_ReadAccel(BNO055Init_TypeDef_t *dev, BNO055_AccelData_t *data) {
    uint8_t buffer[6];

    // X_LSB (0x08) adresinden başlayarak 6 byte oku
    if(!BNO_ReadMulti(dev, BNO055_ACC_DATAX_LSB, buffer, 6)) return BNO_I2C_ERROR;
    BNO055_ParseAccelBuffer(dev, buffer, data);
    return BNO_OK;
}


/*
 * Read gyroscope function
 * @param device struct addr
 * @param data struct addr
 * @retrieval none
 */

BNO_Status_t BNO055_ReadGyro(BNO055Init_TypeDef_t *dev, BNO055_GyroData_t *data){
	uint8_t buffer[6];

	//X_LSB to Z_MSB read
	if(!BNO_ReadMulti(dev, BNO055_GYRO_DATAX_LSB, buffer, 6)) return BNO_I2C_ERROR;
  BNO055_ParseGyroBuffer(dev, buffer, data);
	return BNO_OK;
}



/*
 * Read gyroscope function
 * @param device struct addr
 * @param data struct addr
 * @retrieval none
 */

BNO_Status_t BNO055_ReadMag(BNO055Init_TypeDef_t *dev, BNO055_MagData_t*data){
	uint8_t buffer[6];

	//X_LSB to Z_MSB read

	if(!BNO_ReadMulti(dev, BNO055_MAG_DATAX_LSB , buffer, 6)) return BNO_I2C_ERROR;
  BNO055_ParseMagBuffer(dev, buffer, data);

	return BNO_OK;
}



/*
 * Read euler angles function
 * @param device struct addr
 * @param data struct addr
 * @retrieval none
 */

BNO_Status_t BNO055_ReadEuler(BNO055Init_TypeDef_t *dev, BNO055_EulerData_t *data){
	uint8_t buffer[6];

	//X_LSB to Z_MSB read
	if(!BNO_ReadMulti(dev, BNO055_EUL_HEADING_LSB , buffer, 6)) return BNO_I2C_ERROR;
	BNO055_ParseEulerBuffer(dev, buffer, data);
	return BNO_OK;
	}

/*
 * Read quaternion angles function
 * @param device struct addr
 * @param data struct addr
 * @retrieval none
 */



BNO_Status_t BNO055_ReadQuaternion(BNO055Init_TypeDef_t *dev, BNO055_QuatData_t *data){
	uint8_t buffer[8];

	//X_LSB to Z_MSB read
	if(BNO_ReadMulti(dev, BNO055_QUA_DATAW_LSB , buffer, 8)) return BNO_I2C_ERROR;
	BNO055_ParseQuatBuffer(dev, buffer, data);
	return BNO_OK;
}




/*
 * Read calibration data function
 * @param device struct addr
 * @param data struct addr
 * @retrieval none
 */

BNO_Status_t BNO055_ReadCalibration(BNO055Init_TypeDef_t *dev, BNO055_CalibData_t *data){
  uint8_t calib = BNO_ReadReg(dev, BNO055_CALIB_STAT);

  data->calib_mag = (calib >> 0) & 0x03;
  data->calib_acc = (calib >> 2) & 0x03;
  data->calib_gyro = (calib >> 4) & 0x03;
  data->calib_sys = (calib >> 6) & 0x03;

  return BNO_OK;
}


/*Non-Blocking DMA read functions - non-blocking */
BNO_Status_t BNO055_ReadEuler_DMA(BNO055Init_TypeDef_t *dev, uint8_t *buffer){
  return
      (HAL_I2C_Mem_Read_DMA(dev->i2cHandler ,
      dev->i2cAddress,
      BNO055_EUL_HEADING_LSB,
      1,
      buffer,
      6) == HAL_OK) ? BNO_OK : BNO_I2C_ERROR;
}
BNO_Status_t BNO055_ReadAccel_DMA(BNO055Init_TypeDef_t *dev, uint8_t *buffer){
  return
      (HAL_I2C_Mem_Read_DMA(dev->i2cHandler ,
      dev->i2cAddress,
      BNO055_ACC_DATAX_LSB,
       1,
       buffer,
       6) == HAL_OK) ? BNO_OK : BNO_I2C_ERROR;
}
BNO_Status_t BNO055_ReadGyro_DMA(BNO055Init_TypeDef_t *dev, uint8_t *buffer){
  return
        (HAL_I2C_Mem_Read_DMA(dev->i2cHandler ,
        dev->i2cAddress,
        BNO055_GYRO_DATAX_LSB,
         1,
         buffer,
         6) == HAL_OK) ? BNO_OK : BNO_I2C_ERROR;
}
BNO_Status_t BNO055_ReadMag_DMA(BNO055Init_TypeDef_t *dev, uint8_t *buffer){
  return
          (HAL_I2C_Mem_Read_DMA(dev->i2cHandler ,
          dev->i2cAddress,
          BNO055_MAG_DATAX_LSB,
           1,
           buffer,
           6) == HAL_OK) ? BNO_OK : BNO_I2C_ERROR;
}
BNO_Status_t BNO055_ReadQuaternion_DMA(BNO055Init_TypeDef_t *dev, uint8_t *buffer){
  return
          (HAL_I2C_Mem_Read_DMA(dev->i2cHandler ,
          dev->i2cAddress,
          BNO055_QUA_DATAW_LSB,
           1,
           buffer,
           8) == HAL_OK) ? BNO_OK : BNO_I2C_ERROR;
}
//************PARSE DATA FUNCTIONS**************************//

void BNO055_ParseEulerBuffer(BNO055Init_TypeDef_t *dev, const uint8_t *buffer,
                            BNO055_EulerData_t *data) {
   int16_t heading_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
   int16_t roll_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
   int16_t pitch_raw = (int16_t)((buffer[5] << 8) | buffer[4]);

   if (dev->eulerUnit == BNO_EULER_UNIT_DEG) {
       data->heading = heading_raw / 16.0f;
       data->roll = roll_raw / 16.0f;
       data->pitch = pitch_raw / 16.0f;
   } else {
       data->heading = heading_raw / 900.0f;
       data->roll = roll_raw / 900.0f;
       data->pitch = pitch_raw / 900.0f;
   }
}

void BNO055_ParseAccelBuffer(BNO055Init_TypeDef_t *dev, const uint8_t *buffer,
                            BNO055_AccelData_t *data) {
   int16_t x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
   int16_t y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
   int16_t z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);

   if (dev->accelUnit == BNO_ACC_UNIT_MS2) {
       data->acc_x = x_raw / 100.0f;
       data->acc_y = y_raw / 100.0f;
       data->acc_z = z_raw / 100.0f;
   } else {
       data->acc_x = (float)x_raw;
       data->acc_y = (float)y_raw;
       data->acc_z = (float)z_raw;
   }
}

void BNO055_ParseGyroBuffer(BNO055Init_TypeDef_t *dev, const uint8_t *buffer,
                           BNO055_GyroData_t *data) {
   int16_t x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
   int16_t y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
   int16_t z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);

   if (dev->gyroUnit == BNO_GYRO_UNIT_DPS) {
       data->gyro_x = x_raw / 16.0f;
       data->gyro_y = y_raw / 16.0f;
       data->gyro_z = z_raw / 16.0f;
   } else {
       data->gyro_x = x_raw / 900.0f;
       data->gyro_y = y_raw / 900.0f;
       data->gyro_z = z_raw / 900.0f;

   }
}




void BNO055_ParseQuatBuffer(BNO055Init_TypeDef_t *dev, const uint8_t *buffer, BNO055_QuatData_t *data)
{
    int16_t w_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
     int16_t x_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
     int16_t y_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
     int16_t z_raw = (int16_t)((buffer[7] << 8) | buffer[6]);


     data->quat_w = w_raw / 16384.0f;
      data->quat_x = x_raw / 16384.0f;
      data->quat_y = y_raw / 16384.0f;
      data->quat_z = z_raw / 16384.0f;

}




void BNO055_ParseMagBuffer(BNO055Init_TypeDef_t *dev, const uint8_t *buffer, BNO055_MagData_t *data)
{
    data->mag_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->mag_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->mag_z = (int16_t)((buffer[5] << 8) | buffer[4]);
}