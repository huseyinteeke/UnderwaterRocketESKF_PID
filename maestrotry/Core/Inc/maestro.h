/*
 * maestro.h
 *
 *  Created on: Nov 21, 2025
 *      Author: husey
 */

#ifndef INC_MAESTRO_H_
#define INC_MAESTRO_H_

#include "stm32f4xx_hal.h"

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Protocol Configuration
#define MAESTRO_DEVICE_NUMBER_DEFAULT   255  // Use Compact Protocol
#define MAESTRO_NO_RESET_PIN            255  // No reset pin

// Command Bytes (Compact Protocol)
#define MAESTRO_CMD_MINI_SSC            0xFF
#define MAESTRO_CMD_SET_TARGET          0x84
#define MAESTRO_CMD_SET_SPEED           0x87
#define MAESTRO_CMD_SET_ACCELERATION    0x89
#define MAESTRO_CMD_GET_POSITION        0x90
#define MAESTRO_CMD_GET_MOVING_STATE    0x93
#define MAESTRO_CMD_GET_ERRORS          0xA1
#define MAESTRO_CMD_GO_HOME             0xA2
#define MAESTRO_CMD_STOP_SCRIPT         0xA4
#define MAESTRO_CMD_RESTART_SCRIPT      0xA7
#define MAESTRO_CMD_RESTART_WITH_PARAM  0xA8
#define MAESTRO_CMD_GET_SCRIPT_STATUS   0xAE

// Mini Maestro specific commands
#define MAESTRO_CMD_SET_PWM             0x8A
#define MAESTRO_CMD_SET_MULTIPLE_TARGET 0x9F

// Protocol bytes
#define MAESTRO_BAUD_RATE_INDICATION    0xAA
#define MAESTRO_CRC7_POLYNOMIAL         0x91

// Limits
#define MAESTRO_MAX_CHANNELS            24
#define MAESTRO_MAX_TARGET              16383
#define MAESTRO_MAX_SPEED               16383
#define MAESTRO_MAX_ACCELERATION        16383

/**
 * @brief Maestro configuration structure
 */
typedef struct {
    UART_HandleTypeDef *huart;      // UART handle
    uint8_t deviceNumber;            // Device number (255 = Compact Protocol)
    bool crcEnabled;                 // Enable CRC7 calculation
    GPIO_TypeDef *resetPort;         // Reset pin GPIO port (NULL if not used)
    uint16_t resetPin;               // Reset pin number (0 if not used)
    uint32_t txTimeout;              // Transmission timeout in ms
} Maestro_Config_t;

/**
 * @brief Maestro handle structure
 */
typedef struct {
    Maestro_Config_t config;
    uint8_t crcByte;                 // CRC accumulator
} Maestro_Handle_t;

// Initialization and Reset
HAL_StatusTypeDef Maestro_Init(Maestro_Handle_t *hmaestro, Maestro_Config_t *config);
HAL_StatusTypeDef Maestro_Reset(Maestro_Handle_t *hmaestro);

// Basic Servo Control (Compact Protocol)
HAL_StatusTypeDef Maestro_SetTarget(Maestro_Handle_t *hmaestro, uint8_t channel, uint16_t target);
HAL_StatusTypeDef Maestro_SetTargetMiniSSC(Maestro_Handle_t *hmaestro, uint8_t channel, uint8_t target);
HAL_StatusTypeDef Maestro_SetSpeed(Maestro_Handle_t *hmaestro, uint8_t channel, uint16_t speed);
HAL_StatusTypeDef Maestro_SetAcceleration(Maestro_Handle_t *hmaestro, uint8_t channel, uint16_t acceleration);

// Position and Status Queries (require RX)
HAL_StatusTypeDef Maestro_GetPosition(Maestro_Handle_t *hmaestro, uint8_t channel, uint16_t *position);
HAL_StatusTypeDef Maestro_GetMovingState(Maestro_Handle_t *hmaestro, uint8_t *moving);
HAL_StatusTypeDef Maestro_GetScriptStatus(Maestro_Handle_t *hmaestro, uint8_t *status);
HAL_StatusTypeDef Maestro_GetErrors(Maestro_Handle_t *hmaestro, uint16_t *errors);

// Script Control
HAL_StatusTypeDef Maestro_GoHome(Maestro_Handle_t *hmaestro);
HAL_StatusTypeDef Maestro_StopScript(Maestro_Handle_t *hmaestro);
HAL_StatusTypeDef Maestro_RestartScript(Maestro_Handle_t *hmaestro, uint8_t subroutineNumber);
HAL_StatusTypeDef Maestro_RestartScriptWithParameter(Maestro_Handle_t *hmaestro, uint8_t subroutineNumber, uint16_t parameter);

// Mini Maestro Specific Commands
HAL_StatusTypeDef Maestro_SetPWM(Maestro_Handle_t *hmaestro, uint16_t onTime, uint16_t period);
HAL_StatusTypeDef Maestro_SetMultiTarget(Maestro_Handle_t *hmaestro, uint8_t numberOfTargets, uint8_t firstChannel, uint16_t *targetList);

// DMA versions (non-blocking)
HAL_StatusTypeDef Maestro_SetTarget_DMA(Maestro_Handle_t *hmaestro, uint8_t channel, uint16_t target, uint8_t *buffer);
HAL_StatusTypeDef Maestro_SetSpeed_DMA(Maestro_Handle_t *hmaestro, uint8_t channel, uint16_t speed, uint8_t *buffer);

#endif /* INC_MAESTRO_H_ */
