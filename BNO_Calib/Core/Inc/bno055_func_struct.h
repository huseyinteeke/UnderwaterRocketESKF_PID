/*
 * bno055_func_struct.h
 *
 *  Created on: Nov 21, 2025
 *      Author: husey
 */

#ifndef INC_BNO055_FUNC_STRUCT_H_
#define INC_BNO055_FUNC_STRUCT_H_

#include "stm32f4xx_hal.h"
#include "bno055.h"

typedef void (*BNO_DelayCallBack_t)(uint32_t ms);
typedef void (*BNO_DMA_RxCallback_t)(void);
typedef void (*BNO_DMA_ErrorCallback_t)(void);


/*
 * ENUM definitions
 */

/*
 * Temperature measurement active or not
 */
typedef enum {
  TEMPOK = 1, TEMPNO = 0
} BNO_TempMode_t;

/*
 * NONFUSION Modes = ACCONLY , ACCONLY ,MAGONLY ,GYROONLY ,ACCMAG ,ACCGYRO ,MAGGYRO ,AMG
 * FUSION Mode = IMU ,COMPASS ,M4G ,NDOF_FMC_OFF ,NDOF
 * CONFIGURATION Mode = CONFIG
 */

typedef enum {
  BNO_MODE_CONFIG = 0x00,
  BNO_MODE_ACCONLY = 0x01,
  BNO_MODE_MAGONLY = 0x02,
  BNO_MODE_GYROONLY = 0x03,
  BNO_MODE_ACCMAG = 0x04,
  BNO_MODE_ACCGYRO = 0x05,
  BNO_MODE_MAGGYRO = 0x06,
  BNO_MODE_AMG = 0x07,
  BNO_MODE_IMU = 0x08,
  BNO_MODE_COMPASS = 0x09,
  BNO_MODE_M4G = 0x0A,
  BNO_MODE_NDOF_FMC_OFF = 0x0B,
  BNO_MODE_NDOF = 0x0C
}BNO_OperatingMode_t;

/*
 * Power mode definitions
 */
typedef enum {
  BNO_PWR_MODE_NORMAL = 0x00,
  BNO_PWR_MODE_LOW = 0x01,
  BNO_PWR_MODE_SUSPEND = 0x02
}BNO_PowerMode_t;

/*
 * BNO axis remapping enum
 */
typedef enum {
  //      [YUKARI BAKAN]   [İLERİ BAKAN]      [AÇIKLAMA]
  BNO_AXIS_REMAP_P0 = 0, //      +Z              +Y         (Varsayılan - Yatay Montaj)
  BNO_AXIS_REMAP_P1 = 1, //      +X              +Z         (Dikey - Konnektörler Arkada)
  BNO_AXIS_REMAP_P2 = 2, //      -X              -Z         (Dikey - P1'in Tersi)
  BNO_AXIS_REMAP_P3 = 3, //      +Y              -X         (Yan Yatmış)
  BNO_AXIS_REMAP_P4 = 4, //      -Y              +X         (Yan Yatmış - P3'ün Tersi)
  BNO_AXIS_REMAP_P5 = 5, //      -X              +Z         (Dikey - Farklı Yön)
  BNO_AXIS_REMAP_P6 = 6, //      -Z              +Y         (Ters Dönmüş - Baş Aşağı)
  BNO_AXIS_REMAP_P7 = 7 //      -Z              -Y         (Baş Aşağı - P6'nın Tersi)
}BNO_AxisRemap_t;

/*
 * BNO clock enum
 */
typedef enum {
  BNO_CLK_INTERNAL = 0x00, // Dahili Osilatör
  BNO_CLK_EXTERNAL = 0x01  // Harici Kristal (Daha stabil)
} BNO_Clock_t;

/*
 * Acceleration unit
 */
typedef enum {
  BNO_ACC_UNIT_MS2 = 0x00,  // m/s²
  BNO_ACC_UNIT_MG = 0x01    // mg (miligravite)
} BNO_AccUnit_t;

/*
 * Gyroscope unit
 */
typedef enum {
  BNO_GYRO_UNIT_DPS = 0x00, // Derece/saniye
  BNO_GYRO_UNIT_RPS = 0x01  // Radyan/saniye
} BNO_GyroUnit_t;

/*
 * Euler angle unit
 */
typedef enum {
  BNO_EULER_UNIT_DEG = 0x00, // Derece
  BNO_EULER_UNIT_RAD = 0x01  // Radyan
} BNO_EulerUnit_t;

/*
 * Temperature unit
 */
typedef enum {
  BNO_TEMP_UNIT_C = 0x00,  // Celsius
  BNO_TEMP_UNIT_F = 0x01   // Fahrenheit
} BNO_TempUnit_t;

/*
 * BNO ERROR~STATUS typedef
 */

typedef enum {
  BNO_ERR_NONE = 0x00, // Her şey yolunda
  BNO_ERR_PERIPH_INIT = 0x01, // Çevresel birim başlatma hatası
  BNO_ERR_SYS_INIT = 0x02, // Sistem başlatma hatası
  BNO_ERR_SELF_TEST = 0x03, // Self-test başarısız (Donanım bozuk olabilir)
  BNO_ERR_REG_VAL_RANGE = 0x04, // Yazdığın değer register limitleri dışında
  BNO_ERR_REG_ADDR_RANGE = 0x05, // Yazdığın adres yanlış
  BNO_ERR_REG_WRITE = 0x06, // Yazma hatası
  BNO_ERR_LOW_POWER = 0x07, // Düşük güç modu hatası
  BNO_ERR_ACC_PWR = 0x08, // İvmeölçer güç hatası
  BNO_ERR_FUSION_CONFIG = 0x09, // Füzyon algoritma hatası
  BNO_ERR_SENSOR_CONFIG = 0x0A  // Sensör konfigürasyon hatası
} BNO_SystemError_t;

/*
 * BNO055 System Status Register (0x39) Values
 * Sensörün o anki çalışma durumu
 */
typedef enum {
  BNO_SYS_IDLE = 0x00, // Boşta
  BNO_SYS_ERR = 0x01, // Hata var! (SYS_ERR'e bakılmalı)
  BNO_SYS_INIT_PERIPH = 0x02, // Sensörleri başlatıyor
  BNO_SYS_INIT_SYSTEM = 0x03, // Sistemi başlatıyor
  BNO_SYS_EXECUTING_SELFTEST = 0x04, // Test yapıyor
  BNO_SYS_FUSION_RUNNING = 0x05, // MÜKEMMEL! Füzyon çalışıyor (Veri hazır)
  BNO_SYS_RUNNING_NO_FUSION = 0x06  // Füzyonsuz çalışıyor (Sadece ham veri)
} BNO_SystemStatus_t;

/*
 * Driver Return Codes
 * Fonksiyonların dönüş tipi bu olacak.
 */
typedef enum {
  BNO_OK = 0x00, // İşlem başarılı
  BNO_I2C_ERROR = 0x01, // I2C hattında sorun var (HAL hatası)
  BNO_ID_ERROR = 0x02, // Chip ID uyuşmuyor (Bağlantı hatası / Yanlış sensör)
  BNO_SYS_ERROR = 0x03, // Sensörün kendisi hata veriyor (SYS_ERR oku)
  BNO_TIMEOUT = 0x04  // Zaman aşım
} BNO_Status_t;

/*
 * USER Configuration , BNO-055 Init type definition
 */

typedef struct {
  I2C_HandleTypeDef *i2cHandler;
  uint8_t i2cAddress;
  uint32_t i2cTimeout;

  /* DMA callbacks (NULL if not using DMA) */
  BNO_DMA_RxCallback_t dmaRxCallback;
  BNO_DMA_ErrorCallback_t dmaErrorCallback;
  BNO_DelayCallBack_t delayCallback;

  BNO_PowerMode_t powerMode;
  BNO_OperatingMode_t operationMode;
  BNO_Clock_t externalCrystal;

  //AXIS CONFIG
  BNO_AxisRemap_t axisRemap;

  // Unit selections
  BNO_AccUnit_t accelUnit;
  BNO_GyroUnit_t gyroUnit;
  BNO_EulerUnit_t eulerUnit;
  BNO_TempUnit_t tempUnit;
  BNO_TempMode_t temperature;

  uint8_t useStoredCalibration;  // 1 = kullan, 0 = kullanma
  uint8_t calibrationData[22];   // Önceden kaydedilmiş kalibrasyon

  BNO_SystemStatus_t currentStatus; // SYS_STAT'tan okunan son durum
  uint8_t lastError;
} BNO055Init_TypeDef_t;

/*
 * BNO055 Output Data Structure
 * Sensörden okunan işlenmiş veriler burada tutulacak.
 */
typedef struct {
  // Euler Açıları (Derece veya Radyan)
  float heading;
  float roll;
  float pitch;
}BNO055_EulerData_t;
  // Quaternion (Birim yok, normalize edilmiş)

typedef struct {
  float quat_w;
  float quat_x;
  float quat_y;
  float quat_z;
} BNO055_QuatData_t;

typedef struct{
  float acc_x;
  float acc_y;
  float acc_z;
}BNO055_AccelData_t;
  // Jiroskop (dps veya rps)
typedef struct{
  float gyro_x;
  float gyro_y;
  float gyro_z;
}BNO055_GyroData_t;
  // Manyetometre (uT)
typedef struct{
  float mag_x;
  float mag_y;
  float mag_z;
}BNO055_MagData_t;



typedef struct{
  uint8_t calib_sys;
  uint8_t calib_gyro;
  uint8_t calib_acc;
  uint8_t calib_mag;
} BNO055_CalibData_t;

// Function Prototypes
BNO_Status_t BNO055_ReadAccel(BNO055Init_TypeDef_t *dev, BNO055_AccelData_t *data);
BNO_Status_t BNO055_ReadGyro(BNO055Init_TypeDef_t *dev, BNO055_GyroData_t *data);
BNO_Status_t BNO055_ReadMag(BNO055Init_TypeDef_t *dev, BNO055_MagData_t *data);
BNO_Status_t BNO055_ReadEuler(BNO055Init_TypeDef_t *dev, BNO055_EulerData_t *data);
BNO_Status_t BNO055_ReadQuaternion(BNO055Init_TypeDef_t *dev, BNO055_QuatData_t *data);
//BNO_Status_t BNO055_ReadTemp(BNO055Init_TypeDef_t *dev, int8_t *data);
BNO_Status_t BNO055_ReadCalibration(BNO055Init_TypeDef_t *dev, BNO055_CalibData_t *data);
BNO_Status_t BNO055_Init(BNO055Init_TypeDef_t *deviceStruct);
BNO_Status_t BNO055_GetCalibrationOffsets(BNO055Init_TypeDef_t *dev, uint8_t *buffer);
BNO_Status_t BNO055_SetCalibrationOffsets(BNO055Init_TypeDef_t *dev, const uint8_t *buffer);
uint8_t BNO055_IsFullyCalibrated(BNO055Init_TypeDef_t *dev);

/* Status Functions */
BNO_Status_t BNO055_GetSystemStatus(BNO055Init_TypeDef_t *dev, BNO_SystemStatus_t *status);
BNO_Status_t BNO055_GetSystemError(BNO055Init_TypeDef_t *dev, BNO_SystemError_t *error);

/* Mode Control */
BNO_Status_t BNO055_SetOperationMode(BNO055Init_TypeDef_t *dev, BNO_OperatingMode_t mode);
BNO_Status_t BNO055_SetPowerMode(BNO055Init_TypeDef_t *dev, BNO_PowerMode_t mode);


/*Non-Blocking DMA read functions - non-blocking */
BNO_Status_t BNO055_ReadEuler_DMA(BNO055Init_TypeDef_t *dev, uint8_t *buffer);
BNO_Status_t BNO055_ReadAccel_DMA(BNO055Init_TypeDef_t *dev, uint8_t *buffer);
BNO_Status_t BNO055_ReadGyro_DMA(BNO055Init_TypeDef_t *dev, uint8_t *buffer);
BNO_Status_t BNO055_ReadMag_DMA(BNO055Init_TypeDef_t *dev, uint8_t *buffer);
BNO_Status_t BNO055_ReadQuaternion_DMA(BNO055Init_TypeDef_t *dev, uint8_t *buffer);

/* Parse DMA buffer to data structures */
void BNO055_ParseEulerBuffer(BNO055Init_TypeDef_t *dev, const uint8_t *buffer, BNO055_EulerData_t *data);
void BNO055_ParseAccelBuffer(BNO055Init_TypeDef_t *dev, const uint8_t *buffer, BNO055_AccelData_t *data);
void BNO055_ParseGyroBuffer(BNO055Init_TypeDef_t *dev, const uint8_t *buffer, BNO055_GyroData_t *data);
void BNO055_ParseMagBuffer(BNO055Init_TypeDef_t *dev, const uint8_t *buffer, BNO055_MagData_t *data);
void BNO055_ParseQuatBuffer(BNO055Init_TypeDef_t *dev, const uint8_t *buffer, BNO055_QuatData_t *data);

#endif /* INC_BNO055_FUNC_STRUCT_H_ */
