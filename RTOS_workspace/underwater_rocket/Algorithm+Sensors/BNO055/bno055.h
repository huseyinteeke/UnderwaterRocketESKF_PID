/*
 * bno055.h
 *
 *  Created on: Nov 20, 2025
 *      Author: husey
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "stdint.h"

#define BNO055_I2C_ADDR_LOW		  (0x28 << 1)				           /*ADR -> GND			*/
#define BNO055_I2C_ADDR_HIGH		(0x29 << 1)		               /*ADR -> VCC			*/

/******************************ONLY READ********************************/

/*																	 			*/
/* 																				*/
/* ---------------------------------PAGE 0--------------------------------------*/
/* 																				*/
/*																				*/

#define BNO055_CHIP_ID		0x00		/*Default Value 	0xA0*/
#define BNO055_ACC_ID			0x01		/*Default Value 	0xFB*/
#define BNO055_MAG_ID			0x02		/*Default Value 	0x32*/
#define BNO055_GYR_ID			0x03		/*Default Value 	0x0F*/
/*
 * SW_RV and BL_REV registers used to understand software and bootloader version
 */
#define BNO055_SW_REV_ID_LSB	0x04		/*Default Value 	0x11*/
#define BNO055_SW_REV_ID_MSF	0x05		/*Default Value 	0x03*/
#define BNO055_BL_REV_ID		0x06		/*Default Value 	0x15*/

/*
 * PAGE ID reg.  used for different register addressing (0x00 - 0x7F yetmemiş , ekstra koymuşlar)
 */
#define BNO055_PAGE_ID0			0x07		/*Default Value 	0x00*/

/*
 * ACC data registers
 */

#define BNO055_ACC_DATAX_LSB	0x08		/*Default Value 	0x00*/
#define BNO055_ACC_DATAX_MSB	0x09		/*Default Value 	0x00*/
#define BNO055_ACC_DATAY_LSB	0x0A		/*Default Value 	0x00*/
#define BNO055_ACC_DATAY_MSB	0x0B		/*Default Value 	0x00*/
#define BNO055_ACC_DATAZ_LSB	0x0C		/*Default Value 	0x00*/
#define BNO055_ACC_DATAZ_MSB	0x0D		/*Default Value 	0x00*/

/*
 * MAG data registers
 */
#define BNO055_MAG_DATAX_LSB	0x0E		/*Default Value 	0x00*/
#define BNO055_MAG_DATAX_MSB	0x0F		/*Default Value 	0x00*/
#define BNO055_MAG_DATAY_LSB	0x10		/*Default Value 	0x00*/
#define BNO055_MAG_DATAY_MSB	0x11		/*Default Value 	0x00*/
#define BNO055_MAG_DATAZ_LSB	0x12		/*Default Value 	0x00*/
#define BNO055_MAG_DATAZ_MSB	0x13		/*Default Value 	0x00*/

/*
 * GYRO data registers
 */

#define BNO055_GYRO_DATAX_LSB	0x14		/*Default Value 	0x00*/
#define BNO055_GYRO_DATAX_MSB	0x15		/*Default Value 	0x00*/
#define BNO055_GYRO_DATAY_LSB	0x16		/*Default Value 	0x00*/
#define BNO055_GYRO_DATAY_MSB	0x17		/*Default Value 	0x00*/
#define BNO055_GYRO_DATAZ_LSB	0x18		/*Default Value 	0x00*/
#define BNO055_GYRO_DATAZ_MSB	0x19		/*Default Value 	0x00*/

/*
 * EULER angles registers
 */

#define BNO055_EUL_HEADING_LSB 	0x1A		/*Default Value 	0x00*/
#define BNO055_EUL_HEADING_MSB 	0x1B		/*Default Value 	0x00*/
#define BNO055_EUL_ROLL_LSB 	  0x1C		/*Default Value 	0x00*/
#define BNO055_EUL_ROLL_MSB 	  0x1D		/*Default Value 	0x00*/
#define BNO055_EUL_PITCH_LSB 	  0x1E		/*Default Value 	0x00*/
#define BNO055_EUL_PITCH_MSB 	  0x1F		/*Default Value 	0x00*/

/*
 * QUATERNATION data registers (Uzaydaki yönelim)
 *
 * q = w + xi + yj + zk
 */
#define BNO055_QUA_DATAW_LSB 	0x20		/*Default Value 	0x00*/
#define BNO055_QUA_DATAW_MSB 	0x21		/*Default Value 	0x00*/
#define BNO055_QUA_DATAX_LSB 	0x22		/*Default Value 	0x00*/
#define BNO055_QUA_DATAX_MSB 	0x23		/*Default Value 	0x00*/
#define BNO055_QUA_DATAY_LSB 	0x24		/*Default Value 	0x00*/
#define BNO055_QUA_DATAY_MSB 	0x25		/*Default Value 	0x00*/
#define BNO055_QUA_DATAZ_LSB 	0x26		/*Default Value 	0x00*/
#define BNO055_QUA_DATAZ_MSB 	0x27		/*Default Value 	0x00*/

/*
 * Linear Acceleration data registers
 */
#define BNO055_LIA_DATAX_LSB 	0x28		/*Default Value 	0x00*/
#define BNO055_LIA_DATAX_MSB 	0x29		/*Default Value 	0x00*/
#define BNO055_LIA_DATAY_LSB 	0x2A		/*Default Value 	0x00*/
#define BNO055_LIA_DATAY_MSB 	0x2B		/*Default Value 	0x00*/
#define BNO055_LIA_DATAZ_LSB 	0x2C		/*Default Value 	0x00*/
#define BNO055_LIA_DATAZ_MSB 	0x2D		/*Default Value 	0x00*/

/*
 * Gravity Vector data registers
 */
#define BNO055_GRV_DATAX_LSB 	0x2E		/*Default Value 	0x00*/
#define BNO055_GRV_DATAX_MSB 	0x2F		/*Default Value 	0x00*/
#define BNO055_GRV_DATAY_LSB 	0x30		/*Default Value 	0x00*/
#define BNO055_GRV_DATAY_MSB 	0x31		/*Default Value 	0x00*/
#define BNO055_GRV_DATAZ_LSB 	0x32		/*Default Value 	0x00*/
#define BNO055_GRV_DATAZ_MSB 	0x33		/*Default Value 	0x00*/

/*
 * Tempature register
 */
#define BNO055_TEMP				0x34		/*Default Value 	0x00*/

/*
 * Calibration Status register
 */
#define BNO055_CALIB_STAT		0x35		/*Default Value 	0x00*/

/*
 *BNO 055 self test result register , sensor checks ACC ;MAG ; GYR; MCU 1 means OK
 */
#define BNO055_ST_RESULT		0x36		/*Default Value 	0x0F*/

/*
 *Olay yeri inceleme gibi , hareket algılandığında , yüksek g lerde falan sensörün
 *Ben bir şey gördüm dediği  yerdir !!!! ÖNEMLİ
 */
#define BNO055_INT_STA			0x37		/*Default Value 	0x00*/

/*
 * Sensörün çalışma kontrolünde önemli , kullanılıyor !!! ÖNEMLİ
 */

#define BNO055_SYS_CLK_STAT		0x38		/*Default Value 	0x00*/
#define BNO055_SYS_STAT			0x39		/*Default Value 	0x00*/
#define BNO055_SYS_ERR			0x3A		/*Default Value 	0x00*/

/****************READ AND WRITE BITS***************************/
/*
 * Unit config register
 */
#define BNO055_UNIT_SEL			0x3B		/*Default Value		0x80*/

/*
 *OPR Mode Register
 */
#define BNO055_OPR_MODE			0x3D		/*Default Value		0x10*/

/*
 * PWR Mode Register
 */
#define BNO055_PWR_MODE			0x3E		/*Default Value		0x00*/

/*
 * SYS trigger Register Initte kullanılması gerekli - Sensörün interruptı
 * gibi çalışıyor
 */
#define BNO055_SYS_TRIGGER		0x3F		/*Default Value		0x00*/

/*
 * Temp Source Register
 */
#define BNO055_TEMP_SOURCE		0x40		/*Default Value		0x00*/

/*
 * AXIS MAP CONFIGURATION
 */
#define BNO055_AXIS_MAP_CNFG	0x41		/*Default Value		0x24*/
#define BNO055_AXIS_MAP_SIGN	0x42		/*Default Value		0x00*/

/*
 * Soft Iron Correction registers - manyetormetre kalibrasyonunda kullanılıyor
 * BNO kendi ayarlar , bizim bu ayarı stm32 nin kendi hafızasına kaydetmemiz
 * gerekli
 */
#define BNO055_SIC_MATRIX_LSB0	0x43		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_MSB0	0x44		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_LSB1	0x45		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_MSB1	0x46		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_LSB2	0x47		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_MSB2	0x48		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_LSB3	0x49		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_MSB3	0x4A		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_LSB4	0x4B		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_MSB4	0x4C		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_LSB5	0x4D		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_MSB5	0x4E		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_LSB6	0x4F		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_MSB6	0x50		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_LSB7	0x51		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_MSB7	0x52		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_LSB8	0x53		/*Default Value		0x00*/
#define BNO055_SIC_MATRIX_MSB8	0x54		/*Default Value		0x00*/

/*
 * ACC Offset config registers
 */
#define BNO055_ACC_OFFSETX_LSB	0x55		/*Default Value		0x00*/
#define BNO055_ACC_OFFSETX_MSB	0x56		/*Default Value		0x00*/
#define BNO055_ACC_OFFSETY_LSB	0x57		/*Default Value		0x00*/
#define BNO055_ACC_OFFSETY_MSB	0x58		/*Default Value		0x00*/
#define BNO055_ACC_OFFSETZ_LSB	0x59		/*Default Value		0x00*/
#define BNO055_ACC_OFFSETZ_MSB	0x5A		/*Default Value		0x00*/

/*
 * Magnetometer Offset Config registers
 */
#define BNO055_MAG_OFFSETX_LSB	0x5B		/*Default Value		0x00*/
#define BNO055_MAG_OFFSETX_MSB	0x5C		/*Default Value		0x00*/
#define BNO055_MAG_OFFSETY_LSB	0x5D		/*Default Value		0x00*/
#define BNO055_MAG_OFFSETY_MSB	0x5E		/*Default Value		0x00*/
#define BNO055_MAG_OFFSETZ_LSB	0x5F		/*Default Value		0x00*/
#define BNO055_MAG_OFFSETZ_MSB	0x60		/*Default Value		0x00*/

/*
 * Gyro Offset Config Registers
 */
#define BNO055_GYRO_OFFSETX_LSB	0x61		/*Default Value		0x00*/
#define BNO055_GYRO_OFFSETX_MSB	0x62		/*Default Value		0x00*/
#define BNO055_GYRO_OFFSETY_LSB	0x63		/*Default Value		0x00*/
#define BNO055_GYRO_OFFSETY_MSB	0x64		/*Default Value		0x00*/
#define BNO055_GYRO_OFFSETZ_LSB	0x65		/*Default Value		0x00*/
#define BNO055_GYRO_OFFSETZ_MSB	0x66		/*Default Value		0x00*/

/*
 * Accelometer and Magnetometer radius config registers
 */
#define BNO055_ACC_RADIUS_LSB	0x67		/*Default Value		0x00*/
#define BNO055_ACC_RADIUS_MSB	0x68		/*Default Value		0x00*/
#define BNO055_MAG_RADIUS_LSB	0x69		/*Default Value		0x00*/
#define BNO055_MAG_RADIUS_MSB	0x6A		/*Default Value		0x00*/

/*																	 			*/
/* 																				*/
/* ---------------------------------PAGE 1--------------------------------------*/
/* 																				*/
/*																				*/

#define BNO055_PAGE_ID1			0x07		/*Default Value 	0x00*/

/*
 * Accelometer , Gyro , Mag range-bandwith-power config
 */
#define BNO055_ACC_CNFG			0x08		/*Default Value		0x0D*/
#define BNO055_MAG_CNFG			0x09		/*Default Value		0x0B*/
#define BNO055_GYRO_CNFG0		0x0A		/*Default Value		0x38*/
#define BNO055_GYRO_CNFG1		0x0B		/*Default Value		0x00*/

/*
 * Sleep configuration registers
 */
#define BNO055_ACC_SLEEP_CNFG	0x0C		/*Default Value		0x00*/
#define BNO055_GYRO_SLEEP_CNFG	0x0D		/*Default Value		0x00*/

/*
 * BNO055 Interrupt Enable and Disable registers
 */
#define BNO055_INT_MSK			0x0F		/*Default Value		0x00*/
#define BNO055_INT_EN 			0x10		/*Default Value		0x00*/

/*
 * Interrupt için hassasiyet ayarları
 * HG = High G force
 * HR = High Roll
 * AM = Any Motion
 */

#define BNO055_ACC_AM_THRES		0x11		/*Default Value		0x14*/
#define BNO055_ACC_INT_SETTINGS	0x12		/*Default Value		0x03*/
#define BNO055_ACC_HG_DURATION	0x13		/*Default Value		0x0F*/
#define BNO055_ACC_HG_THRES		0x14		/*Default Value		0xC0*/
#define BNO055_ACC_NM_THRES		0x15		/*Default Value		0x0A*/
#define BNO055_ACC_NM_SET		0x16		/*Default Value		0x0B*/

#define BNO055_GYRO_INT_SETTING 0x17		/*Default Value		0x00*/
#define BNO055_GYRO_HRX_SET		0x18		/*Default Value		0x01*/
#define BNO055_GYRO_DURX_SET	0x19		/*Default Value		0x19*/
#define BNO055_GYRO_HRY_SET		0x1A		/*Default Value		0x01*/
#define BNO055_GYRO_DURY_SET	0x1B		/*Default Value		0x19*/
#define BNO055_GYRO_HRZ_SET		0x1C		/*Default Value		0x01*/
#define BNO055_GYRO_DURZ_SET	0x1D		/*Default Value		0x19*/

#define BNO055_GYRO_AM_THRES	0x1E		/*Default Value		0x04*/
#define BNO055_GYRO_AM_SET		0x1F		/*Default Value		0x0A*/

/*
 *
 * ----------------------------BNO UNIQUE ID----------------------
 *
 */
#define BNO055_UNIQUE_ID		0x50	/*0x50 to 0x5F*/

#endif /* INC_BNO055_H_ */
