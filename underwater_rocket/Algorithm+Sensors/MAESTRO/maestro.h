/*
 * maestro.h
 *
 *  Created on: Nov 21, 2025
 *      Author: husey
 */

#ifndef INC_MAESTRO_H_
#define INC_MAESTRO_H_

#include "stm32f4xx_hal.h"

/*
 * Servo minimum and maximum us settings
 */
/*
 * Mechanical limits
 */
#define SERVO_MIN_US	((uint16_t)(1000))	//0 degrees
#define SERVO_MAX_US	((uint16_t)(2000))  //180 degrees

#define SERVO_MAX_DEG	((int8_t)(130))
#define SERVO_MIN_DEG	((int8_t)(0))

#define MAESTRO_ACCEL_MAX (255U)
#define MAESTRO_ACCEL_MIN (0U)

#define MAESTRO_SPEED_MAX (255U)
#define MAESTRO_SPEED_MIN (0U)
#define SERVO_CENTER_DEG  (65U)
/*
 * SERIAL SERVO COMMANDS
 * Compact protocol: 0x84, channel number, target low bits, target high bits
 *
 */

#define MAESTRO_CMD_SET_TARGET    0x84
#define MAESTRO_CMD_SET_SPEED     0x87
#define MAESTRO_CMD_SET_ACCEL     0x89


#define MAESTRO_MAX_CHANNEL			4U
/*
 * Maestro channel numbers enum
 */

typedef enum{
	CH0 = 1,
	CH1 = 2,
	CH2 = 3,
	CH3 = 4
}MaestroChannel_TypeDef_t;

/*
 * Maestro acceleration-speed rate modes enumerating
 * ****SLOW modes are better for underwater scenarios*****
 */

typedef enum{
	SLOWEST = 20U,
	SLOW = 70U,
	MID = 120U,
	FAST = 170U,
	FASTEST = 250U
}MaestroAccel_Speed_TypeDef_t;

#define ALLSERVOS (CH0|CH1|CH2|CH3)

/*
 * UART Bus  , motion adjust declerations
 */

typedef struct{
	UART_HandleTypeDef *huart;
	MaestroAccel_Speed_TypeDef_t accel;
	MaestroAccel_Speed_TypeDef_t speed;
}Maestro_Handler_t;




/*
 * Function Prototypes
 */
void MaestroInit(Maestro_Handler_t *dev , UART_HandleTypeDef *huart , MaestroAccel_Speed_TypeDef_t speed , MaestroAccel_Speed_TypeDef_t accel);
void Maestro_SetTarget(Maestro_Handler_t *dev , MaestroChannel_TypeDef_t channel , int8_t degrees , uint8_t* command);
void Maestro_SetSpeed(Maestro_Handler_t *dev ,  MaestroChannel_TypeDef_t channels , uint8_t* command);
void Maestro_SetAccel(Maestro_Handler_t *dev ,  MaestroChannel_TypeDef_t channels , uint8_t* command);


#endif /* INC_MAESTRO_H_ */
