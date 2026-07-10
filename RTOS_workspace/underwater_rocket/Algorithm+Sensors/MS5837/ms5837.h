/*
 * ms5837.h
 *
 *  Created on: Jan 2, 2026
 *      Author: husey
 */

#ifndef INC_MS5837_H_
#define INC_MS5837_H_

/*Read calibration(C1 - C6) u16
  Read digital pressure , tempature data(D1 , D2) u32
  calculate temp(dT , TEMP)     signed 32   //According to this value(temp/100 < 20) -> LOW or HIGH
  //Update C values after first tmp
  calculate temp and cmnpensated press(OFF , SENS , P) signed 64(OFF , SENS) ; signed 32 P
*/
#include "stm32f4xx_hal.h"

#define MS5837_ADDR                     (0x76 << 1)

#define MS5837_RESET_CMD                (0x1E) //Write addr use
#define MS5837_PROM_READ_BASE           (0xA0) //Or with 1 for Ad2 , Ad1 , Ad0 selections
#define MS5837_ADC_READ                 (0x00)
#define MS5837_CONVERT_BASE             (0x40)

typedef enum{
  OSR_256  = 0x00,
  OSR_512  = 0x02,
  OSR_1024 = 0x04,
  OSR_2048 = 0x06,
  OSR_4096 = 0x08,
  OSR_8192 = 0x0A
}MS5837_Osr_t;

typedef enum{
  D1      =  0x00,
  D2      =  0x10
}MS5837_DType_t;



typedef struct {
    uint32_t D1_Pres_Raw;
    uint32_t D2_Temp_Raw;
    int32_t TEMP;     // Gerçek Sıcaklık (Derece * 100)
    int32_t P;        // Gerçek Basınç (mbar * 10)
    int32_t surface_pressure;
    uint16_t C[7];
    float depth;
    I2C_HandleTypeDef *i2cHandle;
    void (*Delay)(uint32_t ms);
    uint8_t rx_buffer[3];
    uint8_t tx_buffer[1];
}MS5837_t;

HAL_StatusTypeDef MS5837_Init(MS5837_t *dev, I2C_HandleTypeDef *i2c_bus);
void MS5837_Calculate(MS5837_t *dev); // Matematiği yapıp P ve TEMP'i günceller

// DMA Helpers
void MS5837_Send_Command_DMA(MS5837_t *dev, uint8_t cmd);
void MS5837_Read_ADC_DMA(MS5837_t *dev);
uint32_t MS5837_Parse_ADC(MS5837_t *dev);

// ISR Callback

#endif /* INC_MS5837_H_ */
