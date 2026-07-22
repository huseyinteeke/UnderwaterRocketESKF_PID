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
#define TASK_PID_MSG                  (tskIDLE_PRIORITY + 4)

#define TASK_PRIORITY_YAWROLL_CONTROL (tskIDLE_PRIORITY + 3)
#define TASK_PRIORITY_PITCH_CONTROL   (tskIDLE_PRIORITY + 3)
#define TASK_PRIORITY_MS5837_READ     (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_Comm            (tskIDLE_PRIORITY + 4)
#define TASK_PRIORITY_VELOCITY        (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_ESKF            (tskIDLE_PRIORITY + 4)

/*
 ############TASK STACK SIZES#################################3
 */
#define TASK_STACK_BNO_READ             1024
#define TASK_STACK_YAWROLL_CONTROL      1024
#define TASK_STACK_PITCH_CONTROL        1024
#define TASK_STACK_MS5837_READ          1024
#define TASK_STACK_Comm                  512
#define TASK_STACK_VELOCITY              512
#define TASK_STACK_ESKF                 5096


/**************************************************************************
 * CONTROL FREQUENCIES
 ***************************************************************************/
#define BNO_READ_RATE_HZ                30      // 50Hz sensor reading
#define YAW_CONTROL_RATE_HZ             30      // 10Hz yaw control
#define PITCH_CONTROL_RATE_HZ           50      // 20Hz pitch control
#define MS5837_READ_RATE_HZ             30      // 10Hz depth reading
#define VELOCITY_RATE_HZ                50
#define DEPTH_CONTROL_RATE_HZ           20

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

typedef struct __attribute((packed))__
{
  uint16_t header;
  uint32_t timestamp;
  float depth, ax, ay, az, pitch, roll, yaw , velocity , distance;
  uint8_t mission_state;
  uint16_t footer;
}TelemetryData_t;

/*************************************************************************
 * GÖREV DURUM MAKİNESİ (Mission FSM)
 ***************************************************************************/
typedef enum {
    MISSION_IDLE = 0,         // Araç bekliyor, ARM bekleniyor
    MISSION_INIT,             // Başlangıç ZUPT, sensör stabilizasyonu
    MISSION_DIVE,             // Hedef derinliğe dalış
    MISSION_CRUISE_OUT,       // 50m ileri seyir (sabit PWM)
    MISSION_DECEL_OUT,        // Hedefe yaklaşma, yavaşlama
    MISSION_STOP_OUT,         // Motor kapalı, sürüklenme ile durma
    MISSION_ZUPT_OUT,         // Dönüş noktasında ZUPT (hız sıfırlama)
    MISSION_TURN,             // 180° U dönüşü (Yaw PID ile)
    MISSION_CRUISE_BACK,      // 50m geri seyir
    MISSION_DECEL_BACK,       // Başlangıca yaklaşma, yavaşlama
    MISSION_STOP_BACK,        // Motor kapalı, sürüklenme ile durma
    MISSION_ZUPT_BACK,        // Varış noktasında ZUPT
    MISSION_SURFACE,          // Yüzeye çıkış
    MISSION_COMPLETE          // Görev tamamlandı
} MissionState_t;

/*************************************************************************
 * NAVİGASYON PARAMETRELERİ
 ***************************************************************************/
#define NAV_TARGET_DISTANCE       30.0f
#define NAV_DECEL_DISTANCE        25.0f    // Yavaşlamaya başlama mesafesi
#define NAV_ARRIVAL_TOLERANCE     2.0f     // Varış toleransı (metre)

#define NAV_CRUISE_PWM            1500     // Seyir PWM değeri (kalibrasyon sonrası güncelle)
#define NAV_DECEL_PWM             1200     // Yavaşlama PWM değeri
#define NAV_STOP_PWM              1000     // Motor kapalı

#define NAV_ZUPT_WAIT_MS          3000     // ZUPT öncesi bekleme süresi (ms)
#define NAV_ZUPT_VELOCITY_THRESH  0.05f    // ZUPT için hız eşiği (m/s)

#define NAV_TURN_HEADING_DELTA    180.0f   // U dönüşü açısı
#define NAV_TURN_TOLERANCE        5.0f     // Dönüş tamamlanma toleransı (derece)

#define NAV_MODEL_DVL_STABLE_MS   2000     // Model-DVL için PWM kararlılık süresi (ms)
#define NAV_DECEL_PROFILE_MAX_S   5.0f     // Yavaşlama profili maksimum süresi (saniye)

#define NAV_TARGET_DEPTH          1.0f     // Hedef derinlik (metre)
#define NAV_DEPTH_TOLERANCE       0.3f     // Derinlik toleransı (metre)

/*************************************************************************
 * MISSION TASK PARAMETRELERİ
 ***************************************************************************/
#define TASK_PRIORITY_MISSION     (tskIDLE_PRIORITY + 3)
#define TASK_STACK_MISSION        1024
#define MISSION_CONTROL_RATE_HZ   20
#define MISSION_CONTROL_PERIOD_MS pdMS_TO_TICKS(1000 / MISSION_CONTROL_RATE_HZ)


extern volatile uint8_t g_IMU_OK;

//GLOBAL Functions
void System_Tasks_Init(void);
void Comm_Send_Response(const char* msg);
#endif /* TASKS_CONFIG_H_ */
