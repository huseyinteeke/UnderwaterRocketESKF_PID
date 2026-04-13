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
	uint32_t range_us = SERVO_MAX_US - SERVO_MIN_US;
  uint32_t range_deg = SERVO_MAX_DEG - SERVO_MIN_DEG;
  uint32_t result_us = SERVO_MIN_US + ((uint32_t)(degrees - SERVO_MIN_DEG) * range_us) / range_deg;
	return result_us * 4;

}

/*
 *
 * ------------------------------------PUBLIC-------------------------------------------------------------
 *
 */



/*
 * Maestro set target function
 * @param device pointer
 * @param channels
 * @param degrees
 * @retrieval NONE
 */


void Maestro_SetTarget(Maestro_Handler_t *dev, MaestroChannel_TypeDef_t channels, int8_t degrees , uint8_t* command) {
	  uint16_t target_quarter_us = degreetoUs(degrees);
	  command[0] = MAESTRO_CMD_SET_TARGET;
		command[1]  = 	channels;
    command[2] = target_quarter_us & 0x7F;
		command[3] = 	(target_quarter_us >> 7) & 0x7F;

    }


/*
 * Maestro set speed function
 * @param device pointer
 * @param channels
 * @retrieval NONE
 */


void Maestro_SetSpeed(Maestro_Handler_t *dev , MaestroChannel_TypeDef_t channels , uint8_t* command){


	//set speed function

  command[0] = MAESTRO_CMD_SET_SPEED;
  command[1] = 0; //applied to all channel after
  command[2] = dev->speed & 0x7F;
  command[3] = (dev->speed >> 7) & 0x7F;




	for (uint8_t ch = 0; ch < MAESTRO_MAX_CHANNEL; ch++) {

		if((channels) & (1 << ch)){

		command[1] = ch;
		}
	   }
	  }



/*
 * Maestro set accel function
 * @param device pointer
 * @retrieval NONE
 */


void Maestro_SetAccel(Maestro_Handler_t *dev , MaestroChannel_TypeDef_t channels , uint8_t* command){




		command[0] = 	MAESTRO_CMD_SET_ACCEL;
		command[1] =	0; //applied to all channel after
		command[2] = 	dev->accel & 0x7F;
		command[3] = 	 (dev->accel >> 7) & 0x7F;




	for (uint8_t ch = 0; ch < MAESTRO_MAX_CHANNEL; ch++) {
	    if((channels) & (1 << ch)){
	    	command[1] = ch;
	    }
	      }
	   }









































