/*
 * tasks.h
 *
 *  Created on: Jan 23, 2026
 *      Author: husey
 */

#ifndef TASKS_CONFIG_H_
#define TASKS_CONFIG_H_

#include <eskf_c_wrapper.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "bno055_func_struct.h"
#include "ms5837.h"
#include "pid.h"
#include <stdbool.h>
#include "maestro.h"

/*
###############TASK PRIORITIES################################
*/
#define TASK_PRIORITY_BNO_READ        (tskIDLE_PRIORITY + 2)
#define TASK_PID_MSG                  (tskIDLE_PRIORITY + 3)

#define TASK_PRIORITY_YAWROLL_CONTROL (tskIDLE_PRIORITY + 3)
#define TASK_PRIORITY_PITCH_CONTROL   (tskIDLE_PRIORITY + 3)
#define TASK_PRIORITY_MS5837_READ     (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_Comm            (tskIDLE_PRIORITY + 3)
#define TASK_PRIORITY_VELOCITY        (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_ESKF            (tskIDLE_PRIORITY + 3)

/*
 ############TASK STACK SIZES#################################3
 */
#define TASK_STACK_BNO_READ             1024
#define TASK_STACK_YAWROLL_CONTROL      1024
#define TASK_STACK_PITCH_CONTROL        1024
#define TASK_STACK_SPEED_CONTROL        1024
#define TASK_STACK_MS5837_READ          1024
#define TASK_STACK_Comm                  512
#define TASK_STACK_VELOCITY              512
#define TASK_STACK_ESKF                 5096
#define TASK_STACK_ENGINE               1024

/**************************************************************************
 * CONTROL FREQUENCIES
 ***************************************************************************/
#define BNO_READ_RATE_HZ                30      // 50Hz sensor reading
#define YAW_CONTROL_RATE_HZ             30      // 10Hz yaw control
#define PITCH_CONTROL_RATE_HZ           50      // 20Hz pitch control
#define MS5837_READ_RATE_HZ             30      // 10Hz depth reading
#define VELOCITY_RATE_HZ                50
#define DEPTH_CONTROL_RATE_HZ           20
#define ENGINE_CONTROL_RATE_HZ           5


#define ENGINE_READ_PERIOD_MS       pdMS_TO_TICKS(1000 / ENGINE_CONTROL_RATE_HZ)
#define BNO_READ_PERIOD_MS          pdMS_TO_TICKS(1000 / BNO_READ_RATE_HZ)
#define YAW_CONTROL_PERIOD_MS       pdMS_TO_TICKS(1000 / YAW_CONTROL_RATE_HZ)
#define PITCH_CONTROL_PERIOD_MS     pdMS_TO_TICKS(1000 / PITCH_CONTROL_RATE_HZ)
#define MS5837_READ_PERIOD_MS       pdMS_TO_TICKS(1000 / MS5837_READ_RATE_HZ)
#define VELOCITY_PERIOD_MS          pdMS_TO_TICKS(1000 / VELOCITY_RATE_HZ)
#define DEPTH_CONTROL_PERIOD_MS     pdMS_TO_TICKS(1000 / DEPTH_CONTROL_RATE_HZ)
/*************************************************************************
 * QUEUE SIZES
 ***************************************************************************/
#define QUEUE_SIZE_YAW              10   // Yaw controller queue
#define QUEUE_SIZE_PITCHROLL        10   // Pitch controller queue
#define QUEUE_SIZE_DEPTH            10   // Depth sensor queue
#define QUEUE_SIZE_BT               10  // Bluetooth command queue


/*************************************************************************
 * DATA STRUCTURES FOR QUEUES
 ***************************************************************************/

/* Yaw data (heading only) */
typedef struct {
    float heading;      // Yaw angle (degrees or radians)
    float roll;         // Roll angle
    uint32_t timestamp; // System tick count
} YawRollData_t;

/* Pitch/Roll data */
typedef struct {
    float pitch;        // Pitch angle
    uint32_t timestamp;
} PitchData_t;


/* Depth sensor data */
typedef struct {
    float depth;        // Depth in meters
    float pressure;     // Prssure data
    uint32_t timestamp;
} DepthData_t;


typedef struct
{
  MaestroChannel_TypeDef_t channel;
  float target;
}MaestroMsg_t;

typedef struct __attribute((packed))
{
  uint16_t header;
  uint32_t timestamp;
  float depth, ax, ay, az, pitch, roll, yaw , velocityx , velocityy , velocityz , distancex , distancey , distancez , rpm , rudderangle , sternangle;
  uint16_t footer;
}TelemetryData_t;


typedef enum __attribute((packed))
{
    CMD_NONE = 0, 
    TURN,
    DEPTH, 
    GO_TO, 
    SYSTEM_RESET, 
    ARM, 
    DISARM, 
    YUNUSLAMA
} Command_t;

typedef struct __attribute((packed))
{
  Command_t command;
  int16_t value;
}CommandData_t;

extern volatile uint8_t g_IMU_OK;

//GLOBAL Functions
void System_Tasks_Init(void);
void Comm_Send_Response(const char* msg);
#endif /* TASKS_CONFIG_H_ */
