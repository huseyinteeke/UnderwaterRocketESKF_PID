#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>

#define POS_TOLERANCE_M      0.5f
#define YAW_TOLERANCE_DEG    2.0f
#define MAX_PITCH_ANGLE_DEG  25.0f
#define SURFACE_DEPTH_M      0.3f

typedef enum {
    CMD_GO_TO = 0,
    CMD_TURN,
    CMD_DEPTH,
    CMD_YUNUSLAMA
} CommandType_t;

typedef struct {
    CommandType_t command;
    float value;
} CommandData_t;

// Global RTOS Nesneleri ve Bayraklar
extern QueueHandle_t xMissionQueue;
extern SemaphoreHandle_t xEskfMutex;
extern volatile bool g_MissionAbort;
extern volatile float lastUpdatedDistancex;
extern volatile float lastUpdatedYaw;
extern volatile float lastUpdatedDepth;

void Mission_Init(void);
void vMissionExecTask(void *pvParameters);

#endif