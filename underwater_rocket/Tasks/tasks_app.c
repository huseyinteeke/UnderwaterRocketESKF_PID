
/*
 * tasks.c
 *
 * Created on: Jan 23, 2026
 * Author: husey
 *
 * REVIEWED / CORRECTED VERSION
 * All changes are marked with "// FIX:" comments so they're easy to find.
 */
 
#include "main.h"
#include "tasks_config.h"
#include <stdio.h>
#include <math.h>    
 
void Callback_BNO_Error(void);
void My_RTOS_Delay_Func(uint32_t period_ms);
void Callback_BNO_DMA_Rx(void);
 
void MS5837_DMA_Callback(void);
void MS5837_DMA_Error_Callback(void);
 
// Sadece fonksiyonları tanıtıyoruz
extern void Pitch_Relay_AutoTune_Task(void *pvParameters);
extern void Yaw_Relay_AutoTune_Task(void *pvParameters);
 
/*
 * ********HAL COMMUNICATION LAYERS******************
 */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart6;
 
Maestro_Handler_t ServoDriver = { &huart4 , FAST  , FAST};
 
static float lastUpdatedPressure;
static float lastUpdatedDepth;
static float lastUpdatedPitch;
static float lastUpdatedYaw;
static float lastUpdatedRoll;
static float lastUpdatedAccelx;
static float lastUpdatedAccely;
static float lastUpdatedAccelz;
static float lastUpdatedVelocity;
static float lastUpdatedDistance;
static float lastUpdatedPWM;
 
/*
 * ************GLOBAL PID VARIABLES******************
 */
 
PID_Config_t g_YawPID = {
    .Kp = 3.0f,
    .Ki = 0.0f,
    .Kd = 0.02f,
    .dt = 0.02f,
 
    .setpoint      = 270.0f,
    .lastError     = 0.0f,
    .integralError = 0.0f,
 
    .outputLimit   = 40.0f,
    .integralLimit = 10.0f
};
 
PID_Config_t g_DepthPID = {
    .Kp = 80.0f,
    .Ki = 0.0f,
    .Kd = 4.0f,
    .dt = 0.05f,
    .setpoint      = 1.0f,
    .lastError     = 0.0f,
    .integralError = 0.0f,
    .outputLimit   = 20.0f,
    .integralLimit = 20.0f
};
 
PID_Config_t g_PitchPID = {
    .Kp = 80.0f,
    .Ki = 0.0f,
    .Kd = 4.0f,
    .dt = 0.05f,
    .setpoint      = 1.0f,
    .lastError     = 0.0f,
    .integralError = 0.0f,
    .outputLimit   = 20.0f,
    .integralLimit = 20.0f
};
 
volatile uint8_t g_ARM_STATUS = 0;
 
/*************************************************************************
 * PRIVATE HANDLES
 ***************************************************************************/
 
/* Queue Handles */
static QueueHandle_t xMaestroCmdQueue;
static SemaphoreHandle_t xEskfMutex;
/* Semaphore Handles */
static SemaphoreHandle_t xMS5837_BinarySem;
/* Task Handles */
static TaskHandle_t xTaskBNO_Read;
static TaskHandle_t xTaskYawRollControl;
static TaskHandle_t xTaskPitchControl;
static TaskHandle_t xTaskDepthControl;
static TaskHandle_t xTaskMS5837;
static TaskHandle_t xMaestroGateKeeper;
static TaskHandle_t xCommRxTask;
static TaskHandle_t xEngineTask;
static TaskHandle_t xCommTxTask;
static TaskHandle_t xEskfPredictTask;
static TaskHandle_t xEskfUpdateTask;
 
static void vBNOTask(void *pvParameters);
static void vPitchPidTask(void *pvParameters);
static void vDepthPidTask(void *pvParameters);
static void vYawPidTask(void *pvParameters);
static void vMS5837Task(void *pvParameters);
static void vMaestroGatekeeperTask(void* pvParameters);
static void vCommRxTask(void * parameters);
static void vCommTxTask(void* parameters);
static void vEngineTask(void* parameters);
static void vEskfPredictTask(void* parameters);
static void vEskfUpdateTask(void* parameters);
 

#define TUNE_MODE_PITCH 1
#define TUNE_MODE_YAW   2
 
// ŞU AN YAPILACAK TEST AUTOTUNE İÇİN (Burayı 1 veya 2 yapmanız yeterli):
#define ACTIVE_TUNE_MODE TUNE_MODE_PITCH
 
// --- KONTROL PARAMETRELERİ ---
#define AUTO_TUNE_TARGET_PWM 1200      // %20 motor gücü
#define AUTO_TUNE_RELAY_AMP  15.0f     // Kanatçıklara verilecek röle genliği (+15 / -15 derece)
#define AUTO_TUNE_CROSSINGS  7         // Toplam 0'dan geçiş sayısı (7 geçiş = 3.5 periyot yapar)
#define AUTO_TUNE_IGNORE     2         // İlk 2 geçişi (1 tam periyot) kararsız bölge olarak yoksay
#define ABORT_ANGLE          30.0f     // GÜVENLİK FRENİ: 30 dereceyi geçerse motoru kapat!
 
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
 
// =========================================================================
// FIX: Removed the duplicate vTempAutoTuneStarter definition that existed
// at the bottom of the original file (it's a redefinition -> compile error).
// This single copy now suspends Depth too (the first copy did, the second
// duplicate at the bottom didn't), and its logic is unchanged otherwise.
// =========================================================================
static void vTempAutoTuneStarter(void *pvParameters) {
    // 1. Aracı suya bırakmak ve yerleşmesi için 15 saniye bekle
    vTaskDelay(pdMS_TO_TICKS(15000));
 
    // 2. TÜM PID VE MOTOR GÖREVLERİNİ ASKIYA AL
    if(xTaskPitchControl != NULL)   vTaskSuspend(xTaskPitchControl);
    if(xTaskYawRollControl != NULL) vTaskSuspend(xTaskYawRollControl);
    if(xTaskDepthControl != NULL)   vTaskSuspend(xTaskDepthControl);
    if(xEngineTask != NULL)         vTaskSuspend(xEngineTask);
 
    // 3. SEÇİM: Pitch mi, Yaw mu?
    #if ACTIVE_TUNE_MODE == TUNE_MODE_PITCH
        printf("\n*** PITCH TESTI BASLATILIYOR - TUM SISTEM SUSPENDED ***\n");
        xTaskCreate(Pitch_Relay_AutoTune_Task, "PitchTune", 512, NULL, TASK_PRIORITY_PITCH_CONTROL, NULL);
    #elif ACTIVE_TUNE_MODE == TUNE_MODE_YAW
        printf("\n*** YAW TESTI BASLATILIYOR - PITCH PID UYANIK ***\n");
        // Yaw testi yaparken Pitch düz durmalı, o yüzden Pitch PID'yi geri uyandırıyoruz.
        if(xTaskPitchControl != NULL) vTaskResume(xTaskPitchControl);
        xTaskCreate(Yaw_Relay_AutoTune_Task, "YawTune", 512, NULL, TASK_PRIORITY_YAWROLL_CONTROL, NULL);
    #endif
 
    // 4. Bu başlatıcı görevin işi bitti
    vTaskDelete(NULL);
}
 
 
/*
 * ################GLOBAL SYSTEM INIT FUNCTION######################
 */
void System_Tasks_Init(void){
 
    xMS5837_BinarySem   = xSemaphoreCreateBinary();
    xMaestroCmdQueue    = xQueueCreate(10 , sizeof(MaestroMsg_t));
    xEskfMutex = xSemaphoreCreateMutex();
 
 
    xTaskCreate(vBNOTask ,
                "BNO_READ" ,
                TASK_STACK_BNO_READ ,
                NULL,
                TASK_PRIORITY_BNO_READ ,
                &xTaskBNO_Read);
 
     if(xMS5837_BinarySem){
         xTaskCreate(vMS5837Task,
                        "MS5837_READ",
                        TASK_STACK_MS5837_READ,
                        NULL,
                        TASK_PRIORITY_MS5837_READ,
                        &xTaskMS5837);
        }
 
        xTaskCreate(vYawPidTask ,
                    "Yaw_RolController",
                    TASK_STACK_YAWROLL_CONTROL,
                    NULL,
                    TASK_PRIORITY_YAWROLL_CONTROL,
                    &xTaskYawRollControl);
 
        xTaskCreate(vPitchPidTask ,
                    "Pitch_Controller",
                    TASK_STACK_PITCH_CONTROL,
                    NULL,
                    TASK_PRIORITY_PITCH_CONTROL,
                    &xTaskPitchControl);
 
        xTaskCreate(vDepthPidTask ,
                    "Depth_Controller",
                    TASK_STACK_PITCH_CONTROL,
                    NULL,
                    TASK_PRIORITY_PITCH_CONTROL ,
                    &xTaskDepthControl
                    );
 
        xTaskCreate(vMaestroGatekeeperTask ,
                    "Maestro gate keeper",
                    TASK_STACK_PITCH_CONTROL,
                    NULL,
                    TASK_PID_MSG,
                    &xMaestroGateKeeper);
 
       xTaskCreate(vCommRxTask ,
                    "COMM rx task",
                    TASK_STACK_Comm,
                    NULL,
                    TASK_PRIORITY_Comm,
                    &xCommRxTask
        );
 
       xTaskCreate(vCommTxTask ,
                   "COMM tx task",
                   TASK_STACK_Comm,
                   NULL,
                   TASK_PRIORITY_Comm,
                   &xCommTxTask
       );
 
       xTaskCreate(vEngineTask,
                    "Engine Task",
                    1024,
                    NULL,
                    TASK_PID_MSG,
                    &xEngineTask);
 
       xTaskCreate(vEskfPredictTask ,
                   "ESKF Predict",
                   TASK_STACK_ESKF,
                   NULL,
                   TASK_PRIORITY_ESKF,
                   &xEskfPredictTask
       );
 
       xTaskCreate(vEskfUpdateTask ,
                          "ESKF update",
                          TASK_STACK_ESKF,
                          NULL,
                          TASK_PRIORITY_ESKF,
                          &xEskfUpdateTask
              );
 
       SubESKF_Init();
 
       // --- GEÇİCİ AUTO TUNE BAŞLATICISINI SİSTEME EKLE ---
       xTaskCreate(vTempAutoTuneStarter,
                   "AutoTuneStart",
                   256,
                   NULL,
                   1, // Düşük öncelik yeterli, sadece bekleyecek
                   NULL);
 
    vTaskStartScheduler();
}
 
static void vEskfPredictTask(void* parameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  const float dt = 0.01f;
  float currentpwm;
 
  for(;;)
  {
    portENTER_CRITICAL();
    currentpwm = lastUpdatedPWM;
    portEXIT_CRITICAL();
 
    if (xSemaphoreTake(xEskfMutex, portMAX_DELAY) == pdTRUE) {
        SubESKF_Predict(currentpwm, dt);
        xSemaphoreGive(xEskfMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
 
static void vEskfUpdateTask(void* parameters)
{
  static float accel_x;
  static float current_pwm;
 
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    portENTER_CRITICAL();
    accel_x = lastUpdatedAccelx;
    current_pwm = lastUpdatedPWM;
    portEXIT_CRITICAL();
 
    if (xSemaphoreTake(xEskfMutex, portMAX_DELAY) == pdTRUE) {
          SubESKF_UpdateIMU(accel_x, current_pwm);
          xSemaphoreGive(xEskfMutex);
      }
  }
}
 
 
/******************************************************************************
 * ************************BNO READ TASK***************************************
 ******************************************************************************/
static void vBNOTask(void *pvParameters)
{
  BNO_Status_t status;
  BNO055Init_TypeDef_t localBNO = {
      .i2cHandler = &hi2c2,
      .i2cAddress = BNO055_I2C_ADDR_LOW,
      .i2cTimeout = 10,
      .dmaRxCallback    = Callback_BNO_DMA_Rx,
      .dmaErrorCallback = Callback_BNO_Error,
      .delayCallback    = My_RTOS_Delay_Func,
      .powerMode     = BNO_PWR_MODE_NORMAL,
      .operationMode = BNO_MODE_NDOF,
      .externalCrystal = 0,
      .axisRemap = BNO_AXIS_REMAP_P1,
      .accelUnit = BNO_ACC_UNIT_MS2,
      .gyroUnit  = BNO_GYRO_UNIT_DPS,
      .eulerUnit = BNO_EULER_UNIT_DEG,
      .tempUnit  = BNO_TEMP_UNIT_C,
      .useStoredCalibration = 1,
      .calibrationData = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x01}
  };
 
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  status = BNO055_Init(&localBNO);
 
 
  if(status != BNO_OK)
  {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
      vTaskDelete(NULL);
  }
  portENTER_CRITICAL();
  PID_Reset(&g_YawPID);
  portEXIT_CRITICAL();
  BNO055_EulerData_t tmp;
  BNO055_AccelData_t acctmp;
 
#define BNO_BURST_READ_SIZE  20
  static uint8_t burst_buffer[BNO_BURST_READ_SIZE];
 
  float yawOffset = 0.0f;
  uint8_t isOffsetSet = 0;
  uint8_t initCounter = 0;
 
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(BNO_READ_PERIOD_MS);
 
  for(;;){
    HAL_I2C_Mem_Read_DMA(localBNO.i2cHandler, localBNO.i2cAddress, BNO055_EUL_HEADING_LSB, 1 , burst_buffer, BNO_BURST_READ_SIZE);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    BNO055_ParseEulerBuffer(&localBNO, &burst_buffer[0] , &tmp);
    BNO055_ParseAccelBuffer(&localBNO, &burst_buffer[14], &acctmp);
    float raw_heading = 450.0f - tmp.heading;
    while (raw_heading >= 360.0f) raw_heading -= 360.0f;
    while (raw_heading < 0.0f)    raw_heading += 360.0f;
 
    if (!isOffsetSet) {
        if (initCounter < 50) {
            initCounter++;
        } else {
            yawOffset = raw_heading - 270.0f;
            isOffsetSet = 1;
        }
    }
 
    float finalYaw = raw_heading - yawOffset;
 
    // 0-360 normalizasyonu
    while (finalYaw >= 360.0f) finalYaw -= 360.0f;
    while (finalYaw < 0.0f)    finalYaw += 360.0f;
 
 
    portENTER_CRITICAL();
    lastUpdatedYaw = finalYaw;
    lastUpdatedRoll = tmp.roll;
    lastUpdatedPitch = tmp.pitch;
    lastUpdatedAccelx = acctmp.acc_x;
    lastUpdatedAccely = acctmp.acc_y;
    lastUpdatedAccelz = acctmp.acc_z;
    portEXIT_CRITICAL();
 
 
    if(xEskfUpdateTask != NULL)
    {
      xTaskNotifyGive(xEskfUpdateTask);
    }
 
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
 
/********************************************************************************
 **********************************vMS5837 Read Task***********************
 ******************************************************************************/
static void vMS5837Task(void *pvParameters){
    static MS5837_t localMS5837;
    localMS5837.Delay = My_RTOS_Delay_Func;
 
    if(MS5837_Init(&localMS5837, &hi2c1) != HAL_OK){
            SEGGER_SYSVIEW_Error("MS5837 INIT FAIL");
            vTaskDelete(NULL);
    }
 
    DepthData_t tx_Depth;
    uint8_t cmd_d1   = MS5837_CONVERT_BASE | D1 | OSR_4096;
    uint8_t cmd_d2   = MS5837_CONVERT_BASE | D2 | OSR_4096;
    uint8_t cmd_read = MS5837_ADC_READ;
 
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(MS5837_READ_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();
 
    for(;;){
 
      // --- D1 (Basınç) Convert Başlat ---
      MS5837_Send_Command_DMA(&localMS5837, cmd_d1);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // DMA TX bittiyse uyan
 
      vTaskDelay(pdMS_TO_TICKS(20)); // Sensöre hesaplama yapması için 20ms süre ver
 
      // --- D1 Read Komutu Gönder ---
      MS5837_Send_Command_DMA(&localMS5837, cmd_read);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
 
      // --- D1 Ham Veriyi Oku (RX) ---
      MS5837_Read_ADC_DMA(&localMS5837);
 
      if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
          localMS5837.D1_Pres_Raw = MS5837_Parse_ADC(&localMS5837);
      }
 
      // --- D2 (Sıcaklık) Convert Başlat ---
      MS5837_Send_Command_DMA(&localMS5837, cmd_d2);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
 
      vTaskDelay(pdMS_TO_TICKS(20)); // Hesaplama için 20ms süre ver
 
      // --- D2 Read Komutu Gönder ---
      MS5837_Send_Command_DMA(&localMS5837, cmd_read);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
 
      // --- D2 Ham Veriyi Oku (RX) ---
      MS5837_Read_ADC_DMA(&localMS5837);
      if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
         localMS5837.D2_Temp_Raw = MS5837_Parse_ADC(&localMS5837);
      }
 
      // --- Matematik ve Değişken Güncelleme ---
      MS5837_Calculate(&localMS5837);
 
      tx_Depth.depth    = localMS5837.depth;
      tx_Depth.pressure = localMS5837.P;
 
      portENTER_CRITICAL();
      lastUpdatedDepth = tx_Depth.depth;
      lastUpdatedPressure = tx_Depth.pressure;
      portEXIT_CRITICAL();
 
      vTaskDelayUntil(&xLastWakeTime , xFrequency);
    }
}
 
static void vPitchPidTask(void *pvParameters){
  static MaestroMsg_t msg1;
  static MaestroMsg_t msg2;
  float current_pitch;
  float servo_cmd;
 
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(PITCH_CONTROL_PERIOD_MS);
 
  for(;;){
     portENTER_CRITICAL();
     current_pitch = lastUpdatedPitch;
     portEXIT_CRITICAL();
 
     servo_cmd = PID_Calculate(&g_PitchPID, current_pitch);
 
     msg1.channel = CH3;
     msg1.target = servo_cmd + SERVO_CENTER_DEG;
     xQueueSend(xMaestroCmdQueue, &msg1, 0);
 
     msg2.channel = CH4;
     msg2.target = SERVO_CENTER_DEG - servo_cmd;
     xQueueSend(xMaestroCmdQueue, &msg2, 0);
     vTaskDelayUntil(&xLastWakeTime , xFrequency);
  }
}
 
 
 
static void vDepthPidTask(void *pvParameters)
{
 
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(DEPTH_CONTROL_PERIOD_MS);
  float desiredPitch;
  float current_depth;
  xLastWakeTime = xTaskGetTickCount();
  for(;;){
 
    portENTER_CRITICAL();
    current_depth = lastUpdatedDepth;
    portEXIT_CRITICAL();
 
    desiredPitch = PID_Calculate(&g_DepthPID , current_depth);
 
    portENTER_CRITICAL();
     g_PitchPID.setpoint = desiredPitch;
     portEXIT_CRITICAL();
 
 
     vTaskDelayUntil(&xLastWakeTime , xFrequency);
}
}
 
static void vYawPidTask(void *pvParameters){
  static MaestroMsg_t msg1;
  static MaestroMsg_t msg2;
 
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(PITCH_CONTROL_PERIOD_MS);
 
  float servo_cmd;
  xLastWakeTime = xTaskGetTickCount();
 
  for(;;){
    servo_cmd = PID_Calculate(&g_YawPID , lastUpdatedYaw);
 
    msg1.channel = CH1;
    msg1.target = servo_cmd + SERVO_CENTER_DEG;
    xQueueSend(xMaestroCmdQueue, &msg1, 0);
 
    msg2.channel = CH0;
    msg2.target = SERVO_CENTER_DEG - servo_cmd;
    xQueueSend(xMaestroCmdQueue, &msg2, 0);
 
    vTaskDelayUntil(&xLastWakeTime , xFrequency);
  }
}
 
#define MAX_BATCH_SIZE  10
#define CMD_LEN         4
 
 
static void vMaestroGatekeeperTask(void *pvParameters) {
    MaestroMsg_t msg;
    static uint8_t command[CMD_LEN];
 
    for(;;) {
        while (xQueueReceive(xMaestroCmdQueue, &msg, portMAX_DELAY) == pdPASS) {
            Maestro_SetTarget(&ServoDriver, msg.channel, msg.target, command);
            if(ServoDriver.huart->gState == HAL_UART_STATE_READY) {
                HAL_UART_Transmit_DMA(ServoDriver.huart, command, CMD_LEN);
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            }
            vTaskDelay(2);
        }
    }
}
 
 
 
static uint32_t g_CurrentThrottle = 1000;
static void vEngineTask(void* parameters)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
  portENTER_CRITICAL();
    lastUpdatedPWM = 1000.0f;
    portEXIT_CRITICAL();
  vTaskDelay(pdMS_TO_TICKS(2000));
 
  vTaskDelay(pdMS_TO_TICKS(1000));
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE  , portMAX_DELAY);
    for(int i = 0 ; i < 80 ; i++){
      g_CurrentThrottle += 5;
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g_CurrentThrottle);
 
      portENTER_CRITICAL();
      lastUpdatedPWM = (float)g_CurrentThrottle;
      portEXIT_CRITICAL();
 
      vTaskDelay(pdMS_TO_TICKS(20));
  }
  vTaskDelay(pdMS_TO_TICKS(13000));
  for(int i = 0 ; i < 80 ; i++){
        g_CurrentThrottle -= 5;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g_CurrentThrottle);
        portENTER_CRITICAL();
        lastUpdatedPWM = (float)g_CurrentThrottle;
        portEXIT_CRITICAL();
        vTaskDelay(pdMS_TO_TICKS(1));
  }
  }
}
 
 
// FIX: Simple range/NaN/Inf sanity check used both for UART-received gains
// and for autotune-computed gains before they are ever written into a live
// PID struct that drives real actuators. Tune the max bounds to your rig.
static uint8_t IsGainSetSafe(float Kp, float Ki, float Kd, float setpoint)
{
    if (isnan(Kp) || isnan(Ki) || isnan(Kd) || isnan(setpoint)) return 0;
    if (isinf(Kp) || isinf(Ki) || isinf(Kd) || isinf(setpoint)) return 0;
    if (Kp < 0.0f || Ki < 0.0f || Kd < 0.0f)                    return 0;
    if (Kp > 500.0f || Ki > 500.0f || Kd > 100.0f)              return 0; // FIX: adjust to your real expected ranges
    return 1;
}
 
static uint8_t msg[33];
static void vCommRxTask(void * parameters)
{
  uint8_t command = 0;
  for(;;)
  {
    // FIX: check the return value. If the DMA is still busy from a previous
    // call, HAL_UART_Receive_DMA fails silently and the task then blocks
    // forever on ulTaskNotifyTake because no completion IRQ will ever come.
    if (HAL_UART_Receive_DMA(&huart6 , msg , 33) != HAL_OK) {
        vTaskDelay(pdMS_TO_TICKS(5));
        continue;
    }
    ulTaskNotifyTake(pdTRUE , portMAX_DELAY);
    command = msg[0];
 
    switch (command){
      case 0x01:
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        if(eTaskGetState(xEngineTask) == eSuspended) {
           vTaskResume(xEngineTask);
        } else {
          xTaskNotify(xTaskBNO_Read , 0 , eNoAction);
          vTaskDelay(2000);
          xTaskNotify(xEngineTask, 0 , eNoAction);
        }
 
        break;
 
      case 0x02: // DISARM / SUSPEND
        vTaskSuspend(xEngineTask);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000); // Motoru hemen durdur
        break;
 
      case 0x03:
        HAL_NVIC_SystemReset();
        break;
 
      case 0x04:
      {
        // FIX: validate the incoming gains/setpoints before committing them
        // to the live PID structs. A corrupted UART packet (bit flip, ESP
        // buffering glitch, partial frame) could previously write NaN/Inf
        // or absurd gains directly into the controller with no check at all.
        float newDepthKp = *(float*)&msg[1];
        float newDepthKi = *(float*)&msg[5];
        float newDepthKd = *(float*)&msg[9];
        float newYawKp   = *(float*)&msg[13];
        float newYawKi   = *(float*)&msg[17];
        float newYawKd   = *(float*)&msg[21];
        float newDepthSp = *(float*)&msg[25];
        float newYawSp   = *(float*)&msg[29];
 
        uint8_t depthOk = IsGainSetSafe(newDepthKp, newDepthKi, newDepthKd, newDepthSp);
        uint8_t yawOk   = IsGainSetSafe(newYawKp,   newYawKi,   newYawKd,   newYawSp);
 
        if (depthOk && yawOk) {
            portENTER_CRITICAL();
            g_DepthPID.Kp       = newDepthKp;
            g_DepthPID.Ki       = newDepthKi;
            g_DepthPID.Kd       = newDepthKd;
            g_YawPID.Kp         = newYawKp;
            g_YawPID.Ki         = newYawKi;
            g_YawPID.Kd         = newYawKd;
            g_DepthPID.setpoint = newDepthSp;
            g_YawPID.setpoint   = newYawSp;
            portEXIT_CRITICAL();
        } else {
            SEGGER_SYSVIEW_Error("REJECTED bad PID packet (0x04)");
        }
        break;
      }
      default:
        break;
    }
  }
}
 
 
 
 
 
static void vCommTxTask(void* parameters)
{
  static TelemetryData_t toSendlist;
 
  for(;;)
  {
 
    toSendlist.timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
    toSendlist.header    = 0xAABB;
    toSendlist.footer    = 0xCCDD;
    portENTER_CRITICAL();
    toSendlist.depth = lastUpdatedDepth;
    toSendlist.ax = lastUpdatedAccelx;
    toSendlist.ay = lastUpdatedAccely;
    toSendlist.az = lastUpdatedAccelz;
    toSendlist.pitch = lastUpdatedPitch;
    toSendlist.roll = lastUpdatedRoll;
    toSendlist.yaw = lastUpdatedYaw;
    toSendlist.velocity = lastUpdatedVelocity;
    toSendlist.distance = lastUpdatedDistance;
    portEXIT_CRITICAL();
 
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&toSendlist , sizeof(TelemetryData_t));
    ulTaskNotifyTake(pdTRUE , portMAX_DELAY);
 
 
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
 
 
 
 
 
/*****************************************************************************
 *****************************BNO DMA HELPERS*********************************
 *****************************************************************************/
void Callback_BNO_DMA_Rx(void)
{
  if(xTaskBNO_Read != NULL){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xTaskBNO_Read , &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  }
 
void Callback_BNO_Error(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    SEGGER_SYSVIEW_Error("BNO DMA ERROR");
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
 
 
void My_RTOS_Delay_Func(uint32_t period_ms)
{
    // EĞER RTOS Scheduler çalışıyorsa (Tasklar dönüyorsa)
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
    else
    {
        HAL_Delay(period_ms);
    }
 
}
 
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C2)
    {
        Callback_BNO_DMA_Rx();
    }
}
 
 
void Maestro_CallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(xMaestroGateKeeper , &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
 
 
void CommRx_CallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(xCommRxTask , &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
 
 
void CommTx_CallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(xCommTxTask , &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
 
 
// Error callbacks
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C1)
    {
        Callback_BNO_Error();
    }else if(hi2c->Instance == I2C3)
    {
        MS5837_DMA_Error_Callback();
    }
}
//##########################################################################################################
 
/*################################################################################
 * #################### MS5837 DMA Callback functions #########################
 *##################################################################################*/
void MS5837_DMA_Callback()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(xTaskMS5837 , &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
 
void MS5837_DMA_Error_Callback(){
     SEGGER_SYSVIEW_Print("MS5837 DMA EXPLODED");
}
 
// =========================================================================
// --- GELİŞMİŞ AUTO-TUNE GÖREVLERİ (Seçmeli Mod) ---
// FIX SUMMARY for both Pitch_Relay_AutoTune_Task and Yaw_Relay_AutoTune_Task:
//   1) All servo targets now include SERVO_CENTER_DEG, matching how the
//      normal PID tasks command the same servos. Previously the autotune
//      tasks sent raw +/-15 deg (and 0.0f on abort) with no center offset,
//      which likely drove the fins toward a mechanical extreme instead of
//      a small symmetric wiggle around neutral -- including on the SAFETY
//      ABORT path, which is the worst place for that bug to be.
//   2) lastUpdatedPWM is now updated during the throttle ramp loops, so the
//      ESKF predict task isn't working off a stale throttle value while
//      autotune is running.
//   3) fabs() -> fabsf() since everything here is float, not double.
//   4) Before writing the computed Ku/Tu-derived gains into the live PID
//      struct, they're checked for NaN/Inf/zero/negative via IsGainSetSafe().
//      If the check fails, the gains are NOT applied and it's reported over
//      the log instead -- otherwise a bad oscillation reading (e.g. near-zero
//      amplitude) could divide-toward-Inf and get written straight into the
//      live controller.
//valla claude sorun var dedi yaptı bişeyler üstte de açıklaması
// =========================================================================
 
void Pitch_Relay_AutoTune_Task(void *pvParameters) {
    float local_max_amp = 0.0f;
    float sum_amplitudes = 0.0f;
    uint32_t baslangic_zamani = 0;
    int salinim_sayaci = 0;
 
    portENTER_CRITICAL();
    float onceki_pitch = lastUpdatedPitch;
    portEXIT_CRITICAL();
 
    MaestroMsg_t msg1, msg2;
    float servo_hedef_acisi = 0.0f;
 
    for(int i = 1000; i <= AUTO_TUNE_TARGET_PWM; i += 5) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i);
        portENTER_CRITICAL();                    // FIX: keep lastUpdatedPWM in sync
        lastUpdatedPWM = (float)i;
        portEXIT_CRITICAL();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
 
    printf("--- PITCH AUTO TUNE BASLADI ---\n");
 
    while(salinim_sayaci < AUTO_TUNE_CROSSINGS) {
        portENTER_CRITICAL();
        float guncel_pitch = lastUpdatedPitch;
        portEXIT_CRITICAL();
 
        if (fabsf(guncel_pitch) >= ABORT_ANGLE) {
            printf("!!! TEHLIKE: PITCH 30 DERECEYI GECTI. TEST IPTAL EDILIYOR !!!\n");
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
            portENTER_CRITICAL();
            lastUpdatedPWM = 1000.0f;
            portEXIT_CRITICAL();
            // FIX: return fins to SERVO_CENTER_DEG (neutral), not 0.0f (which
            // is only "neutral" if SERVO_CENTER_DEG happens to be 0 -- if
            // your servo driver takes an absolute angle with e.g. 90 deg
            // as center, sending 0.0f here would drive toward a hard limit
            // exactly when you've just detected a dangerous pitch angle).
            msg1.channel = CH3; msg1.target = SERVO_CENTER_DEG; xQueueSend(xMaestroCmdQueue, &msg1, 0);
            msg2.channel = CH4; msg2.target = SERVO_CENTER_DEG; xQueueSend(xMaestroCmdQueue, &msg2, 0);
            vTaskDelete(NULL);
        }
 
        if(guncel_pitch > 0.0f) servo_hedef_acisi = -AUTO_TUNE_RELAY_AMP;
        else if (guncel_pitch < 0.0f) servo_hedef_acisi = AUTO_TUNE_RELAY_AMP;
 
        // FIX: add SERVO_CENTER_DEG, same convention as vPitchPidTask
        msg1.channel = CH3; msg1.target = SERVO_CENTER_DEG + servo_hedef_acisi;
        xQueueSend(xMaestroCmdQueue, &msg1, 0);
        msg2.channel = CH4; msg2.target = SERVO_CENTER_DEG - servo_hedef_acisi;
        xQueueSend(xMaestroCmdQueue, &msg2, 0);
 
        if (fabsf(guncel_pitch) > local_max_amp) local_max_amp = fabsf(guncel_pitch);
 
        if ((onceki_pitch <= 0.0f && guncel_pitch > 0.0f) || (onceki_pitch >= 0.0f && guncel_pitch < 0.0f)) {
            salinim_sayaci++;
            if (salinim_sayaci == AUTO_TUNE_IGNORE) {
                baslangic_zamani = HAL_GetTick();
                local_max_amp = 0.0f;
            }
            else if (salinim_sayaci > AUTO_TUNE_IGNORE) {
                sum_amplitudes += local_max_amp;
                local_max_amp = 0.0f;
            }
        }
        onceki_pitch = guncel_pitch;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
 
    for(int i = AUTO_TUNE_TARGET_PWM; i >= 1000; i -= 5) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i);
        portENTER_CRITICAL();                    // FIX: keep lastUpdatedPWM in sync
        lastUpdatedPWM = (float)i;
        portEXIT_CRITICAL();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
 
    // FIX: bring fins back to neutral once the test throttle-down completes
    msg1.channel = CH3; msg1.target = SERVO_CENTER_DEG; xQueueSend(xMaestroCmdQueue, &msg1, 0);
    msg2.channel = CH4; msg2.target = SERVO_CENTER_DEG; xQueueSend(xMaestroCmdQueue, &msg2, 0);
 
    int valid_half_periods = AUTO_TUNE_CROSSINGS - AUTO_TUNE_IGNORE;
    float valid_full_periods = valid_half_periods / 2.0f;
    float ortalama_genlik = sum_amplitudes / valid_half_periods;
    float gecen_sure_sn = (HAL_GetTick() - baslangic_zamani) / 1000.0f;
 
    float Ku = (4.0f * AUTO_TUNE_RELAY_AMP) / (M_PI * ortalama_genlik);
    float Tu = gecen_sure_sn / valid_full_periods;
 
    float newKp = 0.6f * Ku;
    float newKi = (1.2f * Ku) / Tu;
    float newKd = 0.075f * Ku * Tu;
 
    // FIX: sanity-check before writing into the live controller
    if (ortalama_genlik > 0.5f && Tu > 0.01f && IsGainSetSafe(newKp, newKi, newKd, g_PitchPID.setpoint)) {
        portENTER_CRITICAL();
        g_PitchPID.Kp = newKp;
        g_PitchPID.Ki = newKi;
        g_PitchPID.Kd = newKd;
        portEXIT_CRITICAL();
 
        printf("\n===================================\n");
        printf("   PITCH AUTO-TUNE BASARIYLA BITTI!\n");
        printf("===================================\n");
        printf("Ortalama Genlik (A) : %.2f Derece\n", ortalama_genlik);
        printf("Kritik Periyot (Tu) : %.2f Saniye\n", Tu);
        printf("Kritik Kazanc (Ku)  : %.2f\n", Ku);
        printf("-----------------------------------\n");
        printf("  >> BULUNAN PID DEGERLERI <<\n");
        printf("  Kp = %.3f\n", g_PitchPID.Kp);
        printf("  Ki = %.3f\n", g_PitchPID.Ki);
        printf("  Kd = %.3f\n", g_PitchPID.Kd);
        printf("===================================\n\n");
    } else {
        // FIX: never silently write garbage gains -- report and bail instead
        printf("\n!!! PITCH AUTO-TUNE REJECTED - bad measurement (amp=%.3f, Tu=%.3f) !!!\n",
               ortalama_genlik, Tu);
    }
 
    vTaskDelete(NULL);
}
 
void Yaw_Relay_AutoTune_Task(void *pvParameters) {
    float local_max_amp = 0.0f;
    float sum_amplitudes = 0.0f;
    uint32_t baslangic_zamani = 0;
    int salinim_sayaci = 0;
 
    portENTER_CRITICAL();
    float onceki_yaw = lastUpdatedYaw;
    portEXIT_CRITICAL();
 
    MaestroMsg_t msg1, msg2;
    float servo_hedef_acisi = 0.0f;
 
    for(int i = 1000; i <= AUTO_TUNE_TARGET_PWM; i += 5) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i);
        portENTER_CRITICAL();                    // FIX: keep lastUpdatedPWM in sync
        lastUpdatedPWM = (float)i;
        portEXIT_CRITICAL();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
 
    printf("--- YAW AUTO TUNE BASLADI ---\n");
 
    while(salinim_sayaci < AUTO_TUNE_CROSSINGS) {
        portENTER_CRITICAL();
        float guncel_yaw = lastUpdatedYaw;
        portEXIT_CRITICAL();
 
        if (fabsf(guncel_yaw) >= ABORT_ANGLE) {
            printf("!!! TEHLIKE: YAW 30 DERECEYI GECTI. TEST IPTAL EDILIYOR !!!\n");
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
            portENTER_CRITICAL();
            lastUpdatedPWM = 1000.0f;
            portEXIT_CRITICAL();
            // FIX: neutral = SERVO_CENTER_DEG, not 0.0f (see note in Pitch task above)
            msg1.channel = CH1; msg1.target = SERVO_CENTER_DEG; xQueueSend(xMaestroCmdQueue, &msg1, 0);
            msg2.channel = CH0; msg2.target = SERVO_CENTER_DEG; xQueueSend(xMaestroCmdQueue, &msg2, 0);
            vTaskDelete(NULL);
        }
 
        if(guncel_yaw > 0.0f) servo_hedef_acisi = -AUTO_TUNE_RELAY_AMP;
        else if (guncel_yaw < 0.0f) servo_hedef_acisi = AUTO_TUNE_RELAY_AMP;
 
        // FIX: add SERVO_CENTER_DEG, same convention as vYawPidTask
        msg1.channel = CH1; msg1.target = SERVO_CENTER_DEG + servo_hedef_acisi;
        xQueueSend(xMaestroCmdQueue, &msg1, 0);
        msg2.channel = CH0; msg2.target = SERVO_CENTER_DEG - servo_hedef_acisi;
        xQueueSend(xMaestroCmdQueue, &msg2, 0);
 
        if (fabsf(guncel_yaw) > local_max_amp) local_max_amp = fabsf(guncel_yaw);
 
        if ((onceki_yaw <= 0.0f && guncel_yaw > 0.0f) || (onceki_yaw >= 0.0f && guncel_yaw < 0.0f)) {
            salinim_sayaci++;
            if (salinim_sayaci == AUTO_TUNE_IGNORE) {
                baslangic_zamani = HAL_GetTick();
                local_max_amp = 0.0f;
            }
            else if (salinim_sayaci > AUTO_TUNE_IGNORE) {
                sum_amplitudes += local_max_amp;
                local_max_amp = 0.0f;
            }
        }
        onceki_yaw = guncel_yaw;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
 
    for(int i = AUTO_TUNE_TARGET_PWM; i >= 1000; i -= 5) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i);
        portENTER_CRITICAL();                    // FIX: keep lastUpdatedPWM in sync
        lastUpdatedPWM = (float)i;
        portEXIT_CRITICAL();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
 
    // FIX: bring fins back to neutral once the test throttle-down completes
    msg1.channel = CH1; msg1.target = SERVO_CENTER_DEG; xQueueSend(xMaestroCmdQueue, &msg1, 0);
    msg2.channel = CH0; msg2.target = SERVO_CENTER_DEG; xQueueSend(xMaestroCmdQueue, &msg2, 0);
 
    int valid_half_periods = AUTO_TUNE_CROSSINGS - AUTO_TUNE_IGNORE;
    float valid_full_periods = valid_half_periods / 2.0f;
    float ortalama_genlik = sum_amplitudes / valid_half_periods;
    float gecen_sure_sn = (HAL_GetTick() - baslangic_zamani) / 1000.0f;
 
    float Ku = (4.0f * AUTO_TUNE_RELAY_AMP) / (M_PI * ortalama_genlik);
    float Tu = gecen_sure_sn / valid_full_periods;
 
    float newKp = 0.6f * Ku;
    float newKi = (1.2f * Ku) / Tu;
    float newKd = 0.075f * Ku * Tu;
 
    // FIX: sanity-check before writing into the live controller
    if (ortalama_genlik > 0.5f && Tu > 0.01f && IsGainSetSafe(newKp, newKi, newKd, g_YawPID.setpoint)) {
        portENTER_CRITICAL();
        g_YawPID.Kp = newKp;
        g_YawPID.Ki = newKi;
        g_YawPID.Kd = newKd;
        portEXIT_CRITICAL();
 
        printf("\n===================================\n");
        printf("    YAW AUTO-TUNE BASARIYLA BITTI!\n");
        printf("===================================\n");
        printf("Ortalama Genlik (A) : %.2f Derece\n", ortalama_genlik);
        printf("Kritik Periyot (Tu) : %.2f Saniye\n", Tu);
        printf("Kritik Kazanc (Ku)  : %.2f\n", Ku);
        printf("-----------------------------------\n");
        printf("  >> BULUNAN PID DEGERLERI <<\n");
        printf("  Kp = %.3f\n", g_YawPID.Kp);
        printf("  Ki = %.3f\n", g_YawPID.Ki);
        printf("  Kd = %.3f\n", g_YawPID.Kd);
        printf("===================================\n\n");
    } else {
        // FIX: never silently write garbage gains -- report and bail instead
        printf("\n!!! YAW AUTO-TUNE REJECTED - bad measurement (amp=%.3f, Tu=%.3f) !!!\n",
               ortalama_genlik, Tu);
    }
 
    vTaskDelete(NULL);
}
