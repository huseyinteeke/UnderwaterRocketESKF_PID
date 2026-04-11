/*
 * maestro.c
 *
 *  Created on: Nov 21, 2025
 *      Author: husey
 */


#include "maestro.h"



/*
 *
 * -------------------------------------PRIVATE-----------------------------------------------------------
 *
 */
extern void My_RTOS_Delay_Func(uint32_t period_ms);

uint16_t degreetoUs(int8_t degrees){

	if(degrees < SERVO_MIN_DEG){
		degrees = SERVO_MIN_DEG;
	}else if(degrees > SERVO_MAX_DEG){
		degrees = SERVO_MAX_DEG;
	}

	uint8_t degreeRange = SERVO_MAX_DEG - SERVO_MIN_DEG;
	uint16_t usRange = SERVO_MAX_US - SERVO_MIN_US;
	uint8_t shifted_degree = (uint8_t)(degrees - SERVO_MIN_DEG);

	uint16_t result_us = SERVO_MIN_US + (((uint32_t)shifted_degree * usRange) / degreeRange);

	return result_us;

}

/*
 *
 * ------------------------------------PUBLIC-------------------------------------------------------------
 *
 */




/*
 * Assigning UART bus to the controller
 * @param device struct
 * @param uart address
 * @retrieval None
 */

static uint8_t command[4];

void MaestroInit(Maestro_Handler_t *dev, UART_HandleTypeDef *huart , MaestroAccel_Speed_TypeDef_t speed , MaestroAccel_Speed_TypeDef_t accel) {
    dev->huart = huart;
    dev->accel = accel;
    dev->accel = speed;
    Maestro_SetAccel(dev , ALLSERVOS);
    Maestro_SetSpeed(dev , ALLSERVOS);
    Maestro_SetTarget(dev ,ALLSERVOS , 0);
}


/*
 * Maestro set target function
 * @param device pointer
 * @param channels
 * @param degrees
 * @retrieval NONE
 */


void Maestro_SetTarget(Maestro_Handler_t *dev, MaestroChannel_TypeDef_t channels, int8_t degrees) {
	  uint16_t target_quarter_us = degreetoUs(degrees) * 4;
	  command[0] = MAESTRO_CMD_SET_TARGET;
		command[1]  = 	0; //adjusted in the loop
    command[2] = target_quarter_us & 0x7F;
		command[3] = 	(target_quarter_us >> 7) & 0x7F;


    for (uint8_t ch = 0; ch < MAESTRO_MAX_CHANNEL; ch++) {

        // for pitch -> channels = 0b0011 ,
    		// ** ch = 1 ==== channels & 0010 = 0010
    		// ** ch = 3 ==== channels & 1000 = 0000
            if ((channels) & (1 << ch)) {

                command[1] = ch; // Kanal numarasını pakete koy

                // Paketi Gönder
                HAL_UART_Transmit_DMA(dev->huart, command, 4);

                My_RTOS_Delay_Func(5);
                // (Opsiyonel) Maestro'nun bufferı dolmasın diye minik bir bekleme
            }
        }
    }


/*
 * Maestro set speed function
 * @param device pointer
 * @param channels
 * @retrieval NONE
 */


void Maestro_SetSpeed(Maestro_Handler_t *dev , MaestroChannel_TypeDef_t channels){


	//set speed function

  command[0] = MAESTRO_CMD_SET_SPEED;
  command[1] = 0; //applied to all channel after
  command[2] = dev->speed & 0x7F;
  command[3] = (dev->speed >> 7) & 0x7F;




	for (uint8_t ch = 0; ch < MAESTRO_MAX_CHANNEL; ch++) {

		if((channels) & (1 << ch)){

		command[1] = ch;
		HAL_UART_Transmit_DMA(dev->huart, command, 4);
		My_RTOS_Delay_Func(5);

		}
	   }
	  }



/*
 * Maestro set accel function
 * @param device pointer
 * @retrieval NONE
 */


void Maestro_SetAccel(Maestro_Handler_t *dev , MaestroChannel_TypeDef_t channels){


	//set accel function

		command[0] = 	MAESTRO_CMD_SET_ACCEL;
		command[1] =	0; //applied to all channel after
		command[2] = 	dev->accel & 0x7F;
		command[3] = 	 (dev->accel >> 7) & 0x7F;




	for (uint8_t ch = 0; ch < MAESTRO_MAX_CHANNEL; ch++) {
	    if((channels) & (1 << ch)){
	    	command[1] = ch; // Kanal numarasını pakete koy
			// Paketi Gönder
			HAL_UART_Transmit_DMA(dev->huart, command, 4);
			My_RTOS_Delay_Func(5);
			// (Opsiyonel) Maestro'nun bufferı dolmasın diye minik bir bekleme
	    }
	      }
	   }









































