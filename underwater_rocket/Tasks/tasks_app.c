/*
 * tasks.c
 *
 *  Created on: Jan 23, 2026
 *      Author: husey
 */

#include "main.h"
#include "tasks_config.h"
#include "stdio.h"

void Callback_BNO_Error(void);
void My_RTOS_Delay_Func(uint32_t period_ms);
void Callback_BNO_DMA_Rx(void);

void MS5837_DMA_Callback(void);
void MS5837_DMA_Error_Callback(void);

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


/*
 * ************GLOBAL PID VARIABLES******************
 */

/*
 * ************GLOBAL PID VARIABLES******************
 */

PID_Config_t g_PitchPID = {
    .Kp = 1.0f,
    .Ki = 0.2f,
    .Kd = 0.2f,
    .dt = 0.02f,

    .setpoint      = 0.0f, // Araç burnunu düz (0 derece) tutsun
    .lastError     = 0.0f, // Başlangıçta 0
    .integralError = 0.0f, // Başlangıçta birikmiş hata 0

    // --- Güvenlik ve Limitler ---
    .outputLimit   = 65.0f, // Maestro'ya gidecek max sapma (Açıklamayı oku)
    .integralLimit = 10.0f,  // Anti-Windup limiti (Çıkış limitinin %20'si iyi bir başlangıçtır)
};

PID_Config_t g_YawPID = {
    // --- Ayarlanabilir Katsayılar ---
    .Kp = 3.0f,
    .Ki = 0.2f,
    .Kd = 0.2f,
    .dt = 0.02f,

    .setpoint      = 270.0f,
    .lastError     = 0.0f,
    .integralError = 0.0f,

    // --- Güvenlik ve Limitler ---
    .outputLimit   = 50.0f,
    .integralLimit = 10.0f,

};



PID_Config_t g_DepthPID = {
    .Kp = 12.0f,
    .Ki = 0.1f,
    .Kd = 0.1f,
    .dt = 0.02f,

    .setpoint      = 1.0f,
    .lastError     = 0.0f,
    .integralError = 0.0f,

    .outputLimit   = 65.0f,
    .integralLimit = 20.0f,

};


volatile uint8_t g_ARM_STATUS = 0;


/*************************************************************************
 * PRIVATE HANDLES
 ***************************************************************************/

/* Queue Handles */

static QueueHandle_t xMaestroCmdQueue;
/* Semaphore Handles */
static SemaphoreHandle_t xBNO_DMA_Semaphore;  // BNO055 DMA complete signal
static SemaphoreHandle_t xMS5837_BinarySem;   // NS5837 DMA complete signal
/* Task Handles */
static TaskHandle_t xTaskBNO_Read;
static TaskHandle_t xTaskYawRollControl;
static TaskHandle_t xTaskPitchControl;
static TaskHandle_t xTaskMS5837;
static TaskHandle_t xMaestroGateKeeper;
static TaskHandle_t xCommRxTask;
static TaskHandle_t xEngineTask;
static TaskHandle_t xCommTxTask;
static TaskHandle_t xVelocityTask;
/*
 * BT message handle
 */
static char* volatile g_latest_BT_Msg_Ptr = NULL;
static void vBNOTask(void *pvParameters);
static void vPitchPidTask(void *pvParameters);
static void vYawRollPidTask(void *pvParameters);
static void vMS5837Task(void *pvParameters);
static void vMaestroGatekeeperTask(void* pvParameters);
static void vCommRxTask(void * parameters);
static void vCommTxTask(void* parameters);
static void vEngineTask(void* parameters);
static void vVelocityTask(void* parameters);

/*
 * ################GLOBAL SYSTEM INIT FUNCTION######################
 */

void System_Tasks_Init(void){

    xMS5837_BinarySem   = xSemaphoreCreateBinary();
    xMaestroCmdQueue    = xQueueCreate(10 , sizeof(MaestroMsg_t));



    uint8_t status;
    status = xTaskCreate(vBNOTask ,
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

        xTaskCreate(vYawRollPidTask ,
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

       xTaskCreate(vVelocityTask,
                    "Velocity_ZUPT",
                    TASK_STACK_VELOCITY,
                    NULL,
                    TASK_PRIORITY_VELOCITY,
                    &xVelocityTask);

    vTaskStartScheduler();
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
      .axisRemap = BNO_AXIS_REMAP_P0,
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
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(PITCH_CONTROL_PERIOD_MS);
    float servo_cmd;
    float current_pitch;
    float current_depth;
    float desired_pitch;


    xLastWakeTime = xTaskGetTickCount();
  for(;;){

    portENTER_CRITICAL();
    current_pitch = lastUpdatedPitch;
    current_depth = lastUpdatedDepth;
    portEXIT_CRITICAL();

    servo_cmd = PID_Calculate(&g_DepthPID , current_depth);

    msg1.channel = CH3;
    msg1.target = servo_cmd + SERVO_CENTER_DEG;
    xQueueSend(xMaestroCmdQueue, &msg1, 0);

    msg2.channel = CH4;
    msg2.target = SERVO_CENTER_DEG - servo_cmd;
    xQueueSend(xMaestroCmdQueue, &msg2, 0);
    vTaskDelayUntil(&xLastWakeTime , xFrequency);
  }
}





static void vYawRollPidTask(void *pvParameters){
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
    TickType_t xLastWakeTime = xTaskGetTickCount();

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
  vTaskDelay(pdMS_TO_TICKS(2000));

  vTaskDelay(pdMS_TO_TICKS(1000));
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE  , portMAX_DELAY);
    for(int i = 0 ; i < 140 ; i++){
      g_CurrentThrottle += 5;
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g_CurrentThrottle);
      vTaskDelay(pdMS_TO_TICKS(20));
  }
  vTaskDelay(pdMS_TO_TICKS(13000));

  //portENTER_CRITICAL();
  //g_YawPID.setpoint = 90.0f;
  //portEXIT_CRITICAL();

  //vTaskDelay(pdMS_TO_TICKS(13000));

  for(int i = 0 ; i < 140 ; i++){
        g_CurrentThrottle -= 5;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g_CurrentThrottle);
        vTaskDelay(pdMS_TO_TICKS(1));
  }
  }
}
static uint8_t msg[33];

/******************************************************************************
 * **********************VELOCITY / ZUPT TASK**********************************
 * Runs at 10 Hz.  Reads the latest linear acceleration on the X-axis
 * (forward axis) published by vBNOTask, integrates to velocity, and resets
 * velocity to zero whenever acceleration has been below the threshold for
 * ZUPT_WINDOW_SAMPLES consecutive cycles (~300 ms at 10 Hz).
 ******************************************************************************/
static void vVelocityTask(void* parameters)
{
    /* ZUPT parameters */
    const float ZUPT_THRESHOLD   = 0.05f; /* m/s^2 – accel below this = stationary */
    const int   ZUPT_WIN_SAMPLES = 25;     /* 5 × 100 ms = 500 ms window            */
    const float dt               = 1.0f / (float)VELOCITY_RATE_HZ; /* 0.1 s */
    const float LPF_ALPHA        = 0.2f;


    int   zupt_count = 0;
    float velocity   = 0.0f;
    float distance   = 0.0f;


    float filtered_accel   = 0.0f;
    float prev_filt_accel  = 0.0f;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / VELOCITY_RATE_HZ);

    for(;;)
    {
        /* Grab the latest accel sample published by vBNOTask (Y = forward axis) */
        portENTER_CRITICAL();
        float raw_accel = lastUpdatedAccely;
        portEXIT_CRITICAL();

        filtered_accel = (LPF_ALPHA * raw_accel) + ((1.0f - LPF_ALPHA) * prev_filt_accel);
        /* ZUPT detection: count consecutive near-zero accel samples */
        if (filtered_accel > -ZUPT_THRESHOLD && filtered_accel < ZUPT_THRESHOLD) {
            zupt_count++;
        } else {
            zupt_count = 0;
        }

        /* Integrate or zero depending on ZUPT state */
        if (zupt_count >= ZUPT_WIN_SAMPLES) {
            velocity = 0.0f;  /* Zero Velocity Update */
        } else {
          velocity += ((filtered_accel + prev_filt_accel) * 0.5f) * dt;
        }

        /* Integrate velocity to distance */
        distance += velocity * dt;

        prev_filt_accel = filtered_accel;
        /* Publish results */
        portENTER_CRITICAL();
        lastUpdatedVelocity = velocity;
        lastUpdatedDistance = distance;
        portEXIT_CRITICAL();

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

static void vCommRxTask(void * parameters)
{
  uint8_t command = 0;
  for(;;)
  {
    HAL_UART_Receive_DMA(&huart6 , msg , 33);
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
        portENTER_CRITICAL();
        g_DepthPID.Kp       = *(float*)&msg[1];
        g_DepthPID.Ki       = *(float*)&msg[5];
        g_DepthPID.Kd       = *(float*)&msg[9];
        g_YawPID.Kp         = *(float*)&msg[13];
        g_YawPID.Ki         = *(float*)&msg[17];
        g_YawPID.Kd         = *(float*)&msg[21];
        g_DepthPID.setpoint = *(float*)&msg[25];
        g_YawPID.setpoint   = *(float*)&msg[29];
        portEXIT_CRITICAL();
        break;
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


    vTaskDelay(pdMS_TO_TICKS(50));
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


// 2. MASTER OKUMA (MS5837 Veri Okuma İçin) - BUNU EKLE!

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



