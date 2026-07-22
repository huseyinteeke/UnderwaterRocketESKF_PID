
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
 
extern void MS5837_DMA_Callback(void);
extern void MS5837_DMA_Error_Callback(void);

#include "eskf_c_wrapper.h"

 
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
volatile MissionState_t g_MissionState = MISSION_IDLE;
 
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
static TaskHandle_t xMissionTask;
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
static void vMissionTask(void* parameters);
static void vEskfPredictTask(void* parameters);
static void vEskfUpdateTask(void* parameters);
 
 
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
 
       xTaskCreate(vMissionTask,
                    "Mission Task",
                    TASK_STACK_MISSION,
                    NULL,
                    TASK_PRIORITY_MISSION,
                    &xMissionTask);
 
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
        float p = SubESKF_GetPosition();
        float v = SubESKF_GetVelocity();
        xSemaphoreGive(xEskfMutex);
        
        portENTER_CRITICAL();
        lastUpdatedDistance = p;
        lastUpdatedVelocity = v;
        portEXIT_CRITICAL();
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
 
static void vEskfUpdateTask(void* parameters)
{
  static float accel_x;
  static float current_pwm;
  
  TickType_t last_pwm_change = 0;
  float previous_pwm = 0;
  
  TickType_t cutoff_time = 0;
  float v0_cutoff = 0.0f;
  uint8_t is_decelerating = 0;
 
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    portENTER_CRITICAL();
    accel_x = lastUpdatedAccelx;
    current_pwm = lastUpdatedPWM;
    portEXIT_CRITICAL();
 
    if (xSemaphoreTake(xEskfMutex, portMAX_DELAY) == pdTRUE) {
          SubESKF_UpdateIMU(accel_x, current_pwm);
          
          switch(g_MissionState) {
              case MISSION_CRUISE_OUT:
              case MISSION_CRUISE_BACK:
                  if (current_pwm != previous_pwm) {
                      last_pwm_change = xTaskGetTickCount();
                      previous_pwm = current_pwm;
                  } else if (pdTICKS_TO_MS(xTaskGetTickCount() - last_pwm_change) >= NAV_MODEL_DVL_STABLE_MS) {
                      SubESKF_UpdateModelDVL(current_pwm);
                  }
                  is_decelerating = 0;
                  break;
                  
              case MISSION_TURN:
                  // BUG-4 FIX: Dönüş sırasında da Model-DVL aktif (1100 PWM sabit)
                  if (current_pwm != previous_pwm) {
                      last_pwm_change = xTaskGetTickCount();
                      previous_pwm = current_pwm;
                  } else if (pdTICKS_TO_MS(xTaskGetTickCount() - last_pwm_change) >= NAV_MODEL_DVL_STABLE_MS) {
                      SubESKF_UpdateModelDVL(current_pwm);
                  }
                  is_decelerating = 0;
                  break;
                  
              case MISSION_STOP_OUT:
              case MISSION_STOP_BACK:
                  // Optimizasyonla bu durumlar kaldırıldı
                  is_decelerating = 0;
                  break;
                  
              case MISSION_ZUPT_OUT:
              case MISSION_ZUPT_BACK:
              case MISSION_INIT:
                  // Sadece INIT'te ZUPT aktif (Araç suda başta hareketsiz)
                  SubESKF_UpdateZUPT();
                  is_decelerating = 0;
                  break;
                  
              default:
                  is_decelerating = 0;
                  break;
          }
          
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
      .operationMode = BNO_MODE_IMU, // Motor manyetik alanından etkilenmemesi için IMU (Sadece Gyro+İvme) modu
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
  const TickType_t xFrequency = BNO_READ_PERIOD_MS;//2 kez pdms to tick yapılıyor 
 
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
 
  TickType_t xLastWakeTime = get_tick_count();//çöp değer alıyodu artık tick count u alıyor

  const TickType_t xFrequency = PITCH_CONTROL_PERIOD_MS; // 2 kez pdms to tick yapıyo
 
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
  const TickType_t xFrequency = DEPTH_CONTROL_PERIOD_MS; // 2 kez pdms to tick yapıyo
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
  const TickType_t xFrequency = YAW_CONTROL_PERIOD_MS;
 
  float servo_cmd;
  float current_yaw;
  xLastWakeTime = xTaskGetTickCount();
 
  for(;;){

    portENTER_CRITICAL();
    current_yaw =  lastUpdatedYaw; //crticical a aldım (abi sövme lütfen awdjklwadl)
    portEXIT_CRITICAL();

    servo_cmd = PID_Calculate_Angle(&g_YawPID , current_yaw);//PID calculate angle a geçirdim çünkü her ne kadar normalizasyon yapılsa da
                                                                // 359 dan 1 e gelirken 358 derece dönücekti bu ayar olmadan 
 
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
 
 
 static uint32_t mission_start_time = 0;
static uint32_t state_start_time = 0;
static float initial_yaw = 0.0f;
static float cruise_pwm = NAV_CRUISE_PWM;
static float turn_position = 0.0f;
static uint32_t current_motor_pwm = NAV_STOP_PWM;  // Kademeli PWM rampalama için

static void vMissionTask(void* parameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = MISSION_CONTROL_PERIOD_MS;
    
    // Motor başlangıç
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, NAV_STOP_PWM);
    
    SubESKF_SetThrustCoeffs(1250.0f, 0.0f, 0.0f); // 1200 PWM -> ~50N
    
    for(;;)
    {
        switch(g_MissionState)
        {
            case MISSION_IDLE:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, NAV_STOP_PWM);
                portENTER_CRITICAL(); lastUpdatedPWM = NAV_STOP_PWM; portEXIT_CRITICAL();
                // ARM bekleniyor... Şimdilik test için direkt INIT'e geçiyoruz:
                // g_MissionState = MISSION_INIT;
                // state_start_time = pdTICKS_TO_MS(xTaskGetTickCount());
                break;
                
            case MISSION_INIT:
                if(pdTICKS_TO_MS(xTaskGetTickCount()) - state_start_time > 5000) {
                    g_MissionState = MISSION_CRUISE_OUT;
                    cruise_pwm = NAV_CRUISE_PWM;
                    initial_yaw = lastUpdatedYaw;
                    current_motor_pwm = NAV_STOP_PWM; // Görev baştan başlarsa rampayı sıfırla
                    
                    portENTER_CRITICAL();
                    g_YawPID.setpoint = initial_yaw;
                    portEXIT_CRITICAL();
                }
                break;
                
            case MISSION_CRUISE_OUT:
                // Kademeli PWM rampalaması: Her FSM döngüsünde (50ms) 10 birim artır
                if(current_motor_pwm < (uint32_t)cruise_pwm) {
                    current_motor_pwm += 10;
                    if(current_motor_pwm > (uint32_t)cruise_pwm) current_motor_pwm = (uint32_t)cruise_pwm;
                }
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_motor_pwm);
                portENTER_CRITICAL(); lastUpdatedPWM = (float)current_motor_pwm; portEXIT_CRITICAL();
                
                if(lastUpdatedDistance >= NAV_DECEL_DISTANCE) {
                    g_MissionState = MISSION_DECEL_OUT;
                }
                break;
                
            case MISSION_DECEL_OUT:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, NAV_DECEL_PWM);
                portENTER_CRITICAL(); lastUpdatedPWM = NAV_DECEL_PWM; portEXIT_CRITICAL();
                
                // Continuous Turn Optimizasyonu: Durmadan dönüşe geç
                if(lastUpdatedDistance >= NAV_TARGET_DISTANCE) {
                    g_MissionState = MISSION_TURN;
                    
                    // Sola dönüşü GARANTİLEMEK için hedefi önce 90 derece veriyoruz
                    float target_yaw = initial_yaw + 90.0f;
                    if(target_yaw >= 360.0f) target_yaw -= 360.0f;
                    portENTER_CRITICAL(); g_YawPID.setpoint = target_yaw; portEXIT_CRITICAL();
                }
                break;
                
            case MISSION_STOP_OUT:
            case MISSION_ZUPT_OUT:
                // Bu durumlar atlandı
                break;
                
            case MISSION_TURN:
            {
                // U dönüşü esnasında motor 1100 PWM'de çalışsın ki kanatçıklara su çarpsın
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1100);
                portENTER_CRITICAL(); lastUpdatedPWM = 1100; portEXIT_CRITICAL();
                
                // Asıl 180 derecelik hedef
                float final_target = initial_yaw + 180.0f;
                if(final_target >= 360.0f) final_target -= 360.0f;
                
                // Şu anki PID hedefine olan hata (ilk aşamada 90, sonra 180'e dönecek)
                float err_to_current_sp = fabsf(lastUpdatedYaw - g_YawPID.setpoint);
                if(err_to_current_sp > 180.0f) err_to_current_sp = 360.0f - err_to_current_sp;
                
                // Araç sola dönüşe başladıysa ve 90 derecelik hedefin yarısını geçtiyse 
                // artık yönü tam 180'e kitleyelim.
                if (err_to_current_sp < 45.0f) {
                    portENTER_CRITICAL(); g_YawPID.setpoint = final_target; portEXIT_CRITICAL();
                }
                
                // Dönüşün bitip bitmediğini ASIL hedefe göre kontrol et
                float err_to_final = fabsf(lastUpdatedYaw - final_target);
                if(err_to_final > 180.0f) err_to_final = 360.0f - err_to_final;
                
                if(err_to_final < NAV_TURN_TOLERANCE) {
                    turn_position = lastUpdatedDistance;
                    current_motor_pwm = 1100;  // Dönüşten çıkarken motor 1100'den rampalanacak
                    g_MissionState = MISSION_CRUISE_BACK;
                }
                break;
            }
                
            case MISSION_CRUISE_BACK:
            {
                // Kademeli PWM rampalaması: Dönüş sonrası da yavaşça hızlan
                if(current_motor_pwm < (uint32_t)cruise_pwm) {
                    current_motor_pwm += 10;
                    if(current_motor_pwm > (uint32_t)cruise_pwm) current_motor_pwm = (uint32_t)cruise_pwm;
                }
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_motor_pwm);
                portENTER_CRITICAL(); lastUpdatedPWM = (float)current_motor_pwm; portEXIT_CRITICAL();
                
                float dist_back = lastUpdatedDistance - turn_position;
                if(dist_back >= NAV_DECEL_DISTANCE) {
                    g_MissionState = MISSION_DECEL_BACK;
                }
                break;
            }
                
            case MISSION_DECEL_BACK:
            {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, NAV_DECEL_PWM);
                portENTER_CRITICAL(); lastUpdatedPWM = NAV_DECEL_PWM; portEXIT_CRITICAL();
                
                // Dönüş yolu bitince doğrudan yüzeye çık (Continuous Turn Optimizasyonu)
                float dist_back2 = lastUpdatedDistance - turn_position;
                if(dist_back2 >= NAV_TARGET_DISTANCE) {
                    g_MissionState = MISSION_SURFACE;
                }
                break;
            }
                
            case MISSION_STOP_BACK:
            case MISSION_ZUPT_BACK:
                // Bu durumlar atlandı
                break;
                
            case MISSION_SURFACE:
                // Araç kendi yoğunluğuyla yüzeye çıkacak, motoru kapat
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, NAV_STOP_PWM);
                portENTER_CRITICAL();
                lastUpdatedPWM = NAV_STOP_PWM;
                g_DepthPID.setpoint = 0.0f;
                portEXIT_CRITICAL();
                
                if(lastUpdatedDepth <= 0.15f) {
                    g_MissionState = MISSION_COMPLETE;
                }
                break;
                
            case MISSION_COMPLETE:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, NAV_STOP_PWM);
                portENTER_CRITICAL(); lastUpdatedPWM = NAV_STOP_PWM; portEXIT_CRITICAL();
                break;
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
        if(eTaskGetState(xMissionTask) == eSuspended) {
           vTaskResume(xMissionTask);
        } else {
          xTaskNotify(xTaskBNO_Read , 0 , eNoAction);
          vTaskDelay(2000);
          
          g_MissionState = MISSION_INIT;
          state_start_time = pdTICKS_TO_MS(xTaskGetTickCount());
        }
 
        break;
 
      case 0x02: // DISARM / SUSPEND
        vTaskSuspend(xMissionTask);
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
    // BUG-2 FIX: Mission state telemetriye eklendi
    toSendlist.mission_state = (uint8_t)g_MissionState;
 
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
 
void MS5837_DMA_Error_Callback()
{
    SEGGER_SYSVIEW_Print("MS5837 DMA EXPLODED");
}
