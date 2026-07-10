/*
 * ms5837.c
 *
 *  Created on: Jan 2, 2026
 *      Author: husey
 */

#include "ms5837.h"

// Private Helper
static uint8_t MS5837_CheckCRC(MS5837_t *dev);
static void MS5837_ReadPROM(MS5837_t *dev);
static void MS5837_Reset(MS5837_t *dev);


static uint8_t MS5837_CheckCRC(MS5837_t *dev){
    uint16_t n_prom[8];
    uint8_t crc_read;
    uint8_t crc_calc;

    for (int i = 0; i < 7; i++) {
            n_prom[i] = dev->C[i];
    }

    n_prom[7] = 0;
    crc_read = (n_prom[0] >> 12) & 0x000F;
    n_prom[0] = n_prom[0] & 0x0FFF;

    uint16_t n_rem = 0;

    for (int cnt = 0; cnt < 16; cnt++){
            if (cnt % 2 == 1)
                n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
            else
                n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);

            for (uint8_t n_bit = 8; n_bit > 0; n_bit--)
            {
                if (n_rem & (0x8000))
                    n_rem = (n_rem << 1) ^ 0x3000;
                else
                    n_rem = (n_rem << 1);
            }
      }
    n_rem = ((n_rem >> 12) & 0x000F);
    crc_calc = (n_rem ^ 0x00);
    if (crc_calc == crc_read) {
        return 1; // Başarılı, veriler sağlam
    } else {
        return 0; // Hata, veriler bozuk!
    }
    }

void MS5837_ReadPROM(MS5837_t *dev){
    uint8_t cmd    = MS5837_PROM_READ_BASE;
    uint8_t rx_buffer[2];
    for(int i = 0 ; i < 7 ; i++){
        cmd = MS5837_PROM_READ_BASE + (i * 2);
       HAL_I2C_Master_Transmit(dev->i2cHandle,
           MS5837_ADDR ,&cmd , 1 , 100);

       HAL_I2C_Master_Receive(dev->i2cHandle,
           MS5837_ADDR , rx_buffer , 2 , 100);

      dev->C[i] = ((rx_buffer[0] << 8) | rx_buffer[1]);
    }
}

void MS5837_StartConversion(MS5837_t *dev, MS5837_DType_t type, MS5837_Osr_t osr){
    uint8_t cmd= MS5837_CONVERT_BASE + type + osr;
    HAL_I2C_Master_Transmit(dev->i2cHandle , MS5837_ADDR ,
        &cmd , 1 , 100);
    dev->Delay(10);

}


HAL_StatusTypeDef MS5837_Init(MS5837_t *dev, I2C_HandleTypeDef *i2c_bus){
        dev->i2cHandle = i2c_bus;
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(dev->i2cHandle, MS5837_ADDR, 5, 100);

            if (status != HAL_OK)
            {
                uint8_t cmd = MS5837_RESET_CMD;
                HAL_I2C_Master_Transmit(dev->i2cHandle, MS5837_ADDR, &cmd, 1, 100);
                dev->Delay(10);
                status = HAL_I2C_IsDeviceReady(dev->i2cHandle, MS5837_ADDR, 5, 100);
                if (status != HAL_OK)
                {
                    return HAL_ERROR;
                }
            }


        MS5837_Reset(dev);

        if(dev->Delay != NULL) dev->Delay(10);
        else HAL_Delay(10);

        MS5837_ReadPROM(dev);

        if (MS5837_CheckCRC(dev) != 1)
            {
                return HAL_ERROR;
            }
        dev->surface_pressure = 0;
        return HAL_OK;
}

void MS5837_Send_Command_DMA(MS5837_t *dev, uint8_t cmd)
{
  dev->tx_buffer[0] = cmd;
  HAL_I2C_Master_Transmit_DMA(dev->i2cHandle,
                              MS5837_ADDR,
                              dev->tx_buffer,
                              1);
}


void MS5837_Reset(MS5837_t *dev){
    uint8_t cmd = MS5837_RESET_CMD;
    HAL_I2C_Master_Transmit(dev->i2cHandle , MS5837_ADDR ,
            &cmd , 1 , 100);

}

/*
 * Read ADC command -> turns as a callback function
 */
void MS5837_Read_ADC_DMA(MS5837_t *dev)
{
  HAL_I2C_Master_Receive_DMA( dev->i2cHandle,
                              MS5837_ADDR,
                              dev->rx_buffer,
                              3);
}


uint32_t MS5837_Parse_ADC(MS5837_t *dev)
{
    return (dev->rx_buffer[0] << 16) | (dev->rx_buffer[1] << 8) | dev->rx_buffer[2];
}


/*
 * We need raw tmp and raw press , don't forget to call them in the task function
 */
void MS5837_Calculate(MS5837_t *dev){
    int32_t dT          = (int32_t)dev->D2_Temp_Raw - ((int32_t)(dev->C[5] << 8));
    int32_t TEMP        = 2000 + (int32_t)(dT*dev->C[6] >> 23);
    int64_t OFF         = (int64_t)(dev->C[2] << 16) + (((int64_t)(dev->C[4]*dT)) >> 7);
    int64_t SENS        = (dev->C[1] << 15) + ((dev->C[3]*dT) >> 8);
    int64_t pressure    = (((((int64_t)dev->D1_Pres_Raw * SENS) >> 21) - OFF) >> 13);

    dev->P    = (int32_t)pressure;  //mbar * 10
    dev->TEMP = TEMP;               //degree * 100

    //If it is the first calculation == Take start as zero
    if(dev->surface_pressure == 0) {
            dev->surface_pressure = dev->P;
    }

    if (dev->P > dev->surface_pressure) {
        dev->depth = (dev->P - dev->surface_pressure) / (1029 * 9.80665) * 100; //Başlangıç noktasını 0 derinlik alıyoruz
    } else {
        dev->depth = 0; // Basınç düşükse hala havadayız demektir.
    }

}
