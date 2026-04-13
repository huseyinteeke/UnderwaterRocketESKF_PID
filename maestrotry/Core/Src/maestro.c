/*
 * maestro.c
 *
 *  Created on: Nov 21, 2025
 *      Author: husey
 */


#include "maestro.h"


// Private function prototypes
static HAL_StatusTypeDef Maestro_WriteByte(Maestro_Handle_t *hmaestro, uint8_t byte);
static HAL_StatusTypeDef Maestro_WriteCRC(Maestro_Handle_t *hmaestro);
static HAL_StatusTypeDef Maestro_WriteCommand(Maestro_Handle_t *hmaestro, uint8_t command);
static HAL_StatusTypeDef Maestro_Write7BitData(Maestro_Handle_t *hmaestro, uint8_t data);
static HAL_StatusTypeDef Maestro_Write14BitData(Maestro_Handle_t *hmaestro, uint16_t data);
static HAL_StatusTypeDef Maestro_ReceiveByte(Maestro_Handle_t *hmaestro, uint8_t *byte);
static void Maestro_UpdateCRC(Maestro_Handle_t *hmaestro, uint8_t byte);

/**
 * @brief Initialize Maestro handle
 */
HAL_StatusTypeDef Maestro_Init(Maestro_Handle_t *hmaestro, Maestro_Config_t *config)
{
    if (hmaestro == NULL || config == NULL || config->huart == NULL)
    {
        return HAL_ERROR;
    }

    // Copy configuration
    memcpy(&hmaestro->config, config, sizeof(Maestro_Config_t));

    // Initialize CRC
    hmaestro->crcByte = 0;

    // Set default timeout if not specified
    if (hmaestro->config.txTimeout == 0)
    {
        hmaestro->config.txTimeout = 100; // 100ms default
    }

    return HAL_OK;
}

/**
 * @brief Reset Maestro via hardware reset pin
 */
HAL_StatusTypeDef Maestro_Reset(Maestro_Handle_t *hmaestro)
{
    if (hmaestro == NULL)
    {
        return HAL_ERROR;
    }

    // Check if reset pin is configured
    if (hmaestro->config.resetPort != NULL && hmaestro->config.resetPin != 0)
    {
        // Drive reset pin low
        HAL_GPIO_WritePin(hmaestro->config.resetPort, hmaestro->config.resetPin, GPIO_PIN_RESET);
        HAL_Delay(1);

        // Return to high (Maestro has internal pull-up)
        HAL_GPIO_WritePin(hmaestro->config.resetPort, hmaestro->config.resetPin, GPIO_PIN_SET);
        HAL_Delay(200); // Wait for Maestro to boot
    }

    return HAL_OK;
}

/**
 * @brief Set target position using Compact or Pololu protocol
 * @param channel Servo channel (0-127)
 * @param target Target position (0-16383) in quarter-microseconds
 */
HAL_StatusTypeDef Maestro_SetTarget(Maestro_Handle_t *hmaestro, uint8_t channel, uint16_t target)
{
    if (hmaestro == NULL || channel > 127 || target > MAESTRO_MAX_TARGET)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_SET_TARGET);
    if (status != HAL_OK) return status;

    status = Maestro_Write7BitData(hmaestro, channel);
    if (status != HAL_OK) return status;

    status = Maestro_Write14BitData(hmaestro, target);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);

    return status;
}

/**
 * @brief Set target using Mini SSC protocol
 * @param channel Channel (0-254)
 * @param target Target (0-254)
 */
HAL_StatusTypeDef Maestro_SetTargetMiniSSC(Maestro_Handle_t *hmaestro, uint8_t channel, uint8_t target)
{
    if (hmaestro == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t cmd[3] = {MAESTRO_CMD_MINI_SSC, channel, target};

    return HAL_UART_Transmit(hmaestro->config.huart, cmd, 3, hmaestro->config.txTimeout);
}

/**
 * @brief Set speed limit
 * @param channel Servo channel (0-127)
 * @param speed Speed (0-16383)
 */
HAL_StatusTypeDef Maestro_SetSpeed(Maestro_Handle_t *hmaestro, uint8_t channel, uint16_t speed)
{
    if (hmaestro == NULL || channel > 127 || speed > MAESTRO_MAX_SPEED)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_SET_SPEED);
    if (status != HAL_OK) return status;

    status = Maestro_Write7BitData(hmaestro, channel);
    if (status != HAL_OK) return status;

    status = Maestro_Write14BitData(hmaestro, speed);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);

    return status;
}

/**
 * @brief Set acceleration limit
 * @param channel Servo channel (0-127)
 * @param acceleration Acceleration (0-16383)
 */
HAL_StatusTypeDef Maestro_SetAcceleration(Maestro_Handle_t *hmaestro, uint8_t channel, uint16_t acceleration)
{
    if (hmaestro == NULL || channel > 127 || acceleration > MAESTRO_MAX_ACCELERATION)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_SET_ACCELERATION);
    if (status != HAL_OK) return status;

    status = Maestro_Write7BitData(hmaestro, channel);
    if (status != HAL_OK) return status;

    status = Maestro_Write14BitData(hmaestro, acceleration);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);

    return status;
}

/**
 * @brief Get current position (requires RX)
 */
HAL_StatusTypeDef Maestro_GetPosition(Maestro_Handle_t *hmaestro, uint8_t channel, uint16_t *position)
{
    if (hmaestro == NULL || position == NULL || channel > 127)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t lowerByte, upperByte;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_GET_POSITION);
    if (status != HAL_OK) return status;

    status = Maestro_Write7BitData(hmaestro, channel);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);
    if (status != HAL_OK) return status;

    // Receive 2 bytes
    status = Maestro_ReceiveByte(hmaestro, &lowerByte);
    if (status != HAL_OK) return status;

    status = Maestro_ReceiveByte(hmaestro, &upperByte);
    if (status != HAL_OK) return status;

    *position = ((uint16_t)upperByte << 8) | lowerByte;

    return HAL_OK;
}

/**
 * @brief Get moving state (requires RX)
 */
HAL_StatusTypeDef Maestro_GetMovingState(Maestro_Handle_t *hmaestro, uint8_t *moving)
{
    if (hmaestro == NULL || moving == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_GET_MOVING_STATE);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);
    if (status != HAL_OK) return status;

    status = Maestro_ReceiveByte(hmaestro, moving);

    return status;
}

/**
 * @brief Get script status (requires RX)
 */
HAL_StatusTypeDef Maestro_GetScriptStatus(Maestro_Handle_t *hmaestro, uint8_t *status_out)
{
    if (hmaestro == NULL || status_out == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_GET_SCRIPT_STATUS);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);
    if (status != HAL_OK) return status;

    status = Maestro_ReceiveByte(hmaestro, status_out);

    return status;
}

/**
 * @brief Get error register (requires RX)
 */
HAL_StatusTypeDef Maestro_GetErrors(Maestro_Handle_t *hmaestro, uint16_t *errors)
{
    if (hmaestro == NULL || errors == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t lowerByte, upperByte;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_GET_ERRORS);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);
    if (status != HAL_OK) return status;

    // Receive 2 bytes
    status = Maestro_ReceiveByte(hmaestro, &lowerByte);
    if (status != HAL_OK) return status;

    status = Maestro_ReceiveByte(hmaestro, &upperByte);
    if (status != HAL_OK) return status;

    *errors = ((uint16_t)upperByte << 8) | lowerByte;

    return HAL_OK;
}

/**
 * @brief Send all servos to home position
 */
HAL_StatusTypeDef Maestro_GoHome(Maestro_Handle_t *hmaestro)
{
    if (hmaestro == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_GO_HOME);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);

    return status;
}

/**
 * @brief Stop running script
 */
HAL_StatusTypeDef Maestro_StopScript(Maestro_Handle_t *hmaestro)
{
    if (hmaestro == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_STOP_SCRIPT);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);

    return status;
}

/**
 * @brief Restart script at subroutine
 */
HAL_StatusTypeDef Maestro_RestartScript(Maestro_Handle_t *hmaestro, uint8_t subroutineNumber)
{
    if (hmaestro == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_RESTART_SCRIPT);
    if (status != HAL_OK) return status;

    status = Maestro_Write7BitData(hmaestro, subroutineNumber);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);

    return status;
}

/**
 * @brief Restart script with parameter
 */
HAL_StatusTypeDef Maestro_RestartScriptWithParameter(Maestro_Handle_t *hmaestro, uint8_t subroutineNumber, uint16_t parameter)
{
    if (hmaestro == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_RESTART_WITH_PARAM);
    if (status != HAL_OK) return status;

    status = Maestro_Write7BitData(hmaestro, subroutineNumber);
    if (status != HAL_OK) return status;

    status = Maestro_Write14BitData(hmaestro, parameter);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);

    return status;
}

/**
 * @brief Set PWM (Mini Maestro only)
 */
HAL_StatusTypeDef Maestro_SetPWM(Maestro_Handle_t *hmaestro, uint16_t onTime, uint16_t period)
{
    if (hmaestro == NULL || onTime > 16320 || period < 4 || period > 16384)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_SET_PWM);
    if (status != HAL_OK) return status;

    status = Maestro_Write14BitData(hmaestro, onTime);
    if (status != HAL_OK) return status;

    status = Maestro_Write14BitData(hmaestro, period);
    if (status != HAL_OK) return status;

    status = Maestro_WriteCRC(hmaestro);

    return status;
}

/**
 * @brief Set multiple targets (Mini Maestro only)
 */
HAL_StatusTypeDef Maestro_SetMultiTarget(Maestro_Handle_t *hmaestro, uint8_t numberOfTargets,
                                          uint8_t firstChannel, uint16_t *targetList)
{
    if (hmaestro == NULL || targetList == NULL || numberOfTargets > MAESTRO_MAX_CHANNELS)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = Maestro_WriteCommand(hmaestro, MAESTRO_CMD_SET_MULTIPLE_TARGET);
    if (status != HAL_OK) return status;

    status = Maestro_Write7BitData(hmaestro, numberOfTargets);
    if (status != HAL_OK) return status;

    status = Maestro_Write7BitData(hmaestro, firstChannel);
    if (status != HAL_OK) return status;

    for (int i = 0; i < numberOfTargets; i++)
    {
        status = Maestro_Write14BitData(hmaestro, targetList[i]);
        if (status != HAL_OK) return status;
    }

    status = Maestro_WriteCRC(hmaestro);

    return status;
}

/**
 * @brief Set target using DMA (non-blocking)
 * @param buffer User-provided buffer (minimum 4 bytes for Compact, 6 for Pololu)
 */
HAL_StatusTypeDef Maestro_SetTarget_DMA(Maestro_Handle_t *hmaestro, uint8_t channel,
                                        uint16_t target, uint8_t *buffer)
{
    if (hmaestro == NULL || buffer == NULL || channel > 127 || target > MAESTRO_MAX_TARGET)
    {
        return HAL_ERROR;
    }

    uint8_t idx = 0;

    // Pololu protocol
    if (hmaestro->config.deviceNumber != MAESTRO_DEVICE_NUMBER_DEFAULT)
    {
        buffer[idx++] = MAESTRO_BAUD_RATE_INDICATION;
        buffer[idx++] = hmaestro->config.deviceNumber & 0x7F;
        buffer[idx++] = MAESTRO_CMD_SET_TARGET & 0x7F;
    }
    else // Compact protocol
    {
        buffer[idx++] = MAESTRO_CMD_SET_TARGET;
    }

    buffer[idx++] = channel & 0x7F;
    buffer[idx++] = target & 0x7F;
    buffer[idx++] = (target >> 7) & 0x7F;

    // CRC is not supported in DMA mode (would need to calculate manually)

    return HAL_UART_Transmit_DMA(hmaestro->config.huart, buffer, idx);
}

/**
 * @brief Set speed using DMA (non-blocking)
 */
HAL_StatusTypeDef Maestro_SetSpeed_DMA(Maestro_Handle_t *hmaestro, uint8_t channel,
                                       uint16_t speed, uint8_t *buffer)
{
    if (hmaestro == NULL || buffer == NULL || channel > 127 || speed > MAESTRO_MAX_SPEED)
    {
        return HAL_ERROR;
    }

    uint8_t idx = 0;

    // Pololu protocol
    if (hmaestro->config.deviceNumber != MAESTRO_DEVICE_NUMBER_DEFAULT)
    {
        buffer[idx++] = MAESTRO_BAUD_RATE_INDICATION;
        buffer[idx++] = hmaestro->config.deviceNumber & 0x7F;
        buffer[idx++] = MAESTRO_CMD_SET_SPEED & 0x7F;
    }
    else // Compact protocol
    {
        buffer[idx++] = MAESTRO_CMD_SET_SPEED;
    }

    buffer[idx++] = channel & 0x7F;
    buffer[idx++] = speed & 0x7F;
    buffer[idx++] = (speed >> 7) & 0x7F;

    return HAL_UART_Transmit_DMA(hmaestro->config.huart, buffer, idx);
}

// ============== PRIVATE FUNCTIONS ==============

/**
 * @brief Write a single byte and update CRC
 */
static HAL_StatusTypeDef Maestro_WriteByte(Maestro_Handle_t *hmaestro, uint8_t byte)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit(hmaestro->config.huart, &byte, 1, hmaestro->config.txTimeout);

    if (status == HAL_OK && hmaestro->config.crcEnabled)
    {
        Maestro_UpdateCRC(hmaestro, byte);
    }

    return status;
}

/**
 * @brief Write CRC byte if enabled
 */
static HAL_StatusTypeDef Maestro_WriteCRC(Maestro_Handle_t *hmaestro)
{
    if (hmaestro->config.crcEnabled)
    {
        HAL_StatusTypeDef status = HAL_UART_Transmit(hmaestro->config.huart,
                                                      &hmaestro->crcByte, 1,
                                                      hmaestro->config.txTimeout);
        hmaestro->crcByte = 0; // Reset CRC
        return status;
    }

    return HAL_OK;
}

/**
 * @brief Write command byte (with protocol header if Pololu protocol)
 */
static HAL_StatusTypeDef Maestro_WriteCommand(Maestro_Handle_t *hmaestro, uint8_t command)
{
    HAL_StatusTypeDef status;

    // Pololu protocol
    if (hmaestro->config.deviceNumber != MAESTRO_DEVICE_NUMBER_DEFAULT)
    {
        status = Maestro_WriteByte(hmaestro, MAESTRO_BAUD_RATE_INDICATION);
        if (status != HAL_OK) return status;

        status = Maestro_Write7BitData(hmaestro, hmaestro->config.deviceNumber);
        if (status != HAL_OK) return status;

        status = Maestro_Write7BitData(hmaestro, command);
    }
    else // Compact protocol
    {
        status = Maestro_WriteByte(hmaestro, command);
    }

    return status;
}

/**
 * @brief Write 7-bit data (mask to 7 bits)
 */
static HAL_StatusTypeDef Maestro_Write7BitData(Maestro_Handle_t *hmaestro, uint8_t data)
{
    return Maestro_WriteByte(hmaestro, data & 0x7F);
}

/**
 * @brief Write 14-bit data as two 7-bit bytes
 */
static HAL_StatusTypeDef Maestro_Write14BitData(Maestro_Handle_t *hmaestro, uint16_t data)
{
    HAL_StatusTypeDef status;

    status = Maestro_WriteByte(hmaestro, data & 0x7F);
    if (status != HAL_OK) return status;

    status = Maestro_WriteByte(hmaestro, (data >> 7) & 0x7F);

    return status;
}

/**
 * @brief Receive a single byte with timeout
 */
static HAL_StatusTypeDef Maestro_ReceiveByte(Maestro_Handle_t *hmaestro, uint8_t *byte)
{
    return HAL_UART_Receive(hmaestro->config.huart, byte, 1, hmaestro->config.txTimeout);
}

/**
 * @brief Update CRC7 calculation
 */
static void Maestro_UpdateCRC(Maestro_Handle_t *hmaestro, uint8_t byte)
{
    hmaestro->crcByte ^= byte;

    for (uint8_t j = 0; j < 8; j++)
    {
        if (hmaestro->crcByte & 1)
        {
            hmaestro->crcByte ^= MAESTRO_CRC7_POLYNOMIAL;
        }
        hmaestro->crcByte >>= 1;
    }
}
