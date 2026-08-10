/*
 * tasks.c
 *
 *  Created on: Jan 23, 2026
 *      Author: husey
 */
 #include "bno055.h"
#include "bno055_func_struct.h"
#include "cmsis_gcc.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
 #include "stm32f4xx_hal_i2c.h"

#include "pid.h"
#include "tasks_config.h"
#include "eskf_c_wrapper.h"
#include "main.h"
#include "portmacro.h"
#include "projdefs.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_tim.h"
#include "stdio.h"
#include "math.h"
#include "config.h"



void Callback_BNO_Error(void);
void My_RTOS_Delay_Func(uint32_t period_ms);
void Callback_BNO_DMA_Rx(void);

void MS5837_DMA_Callback(void);
void MS5837_DMA_Error_Callback(void);
void Notify_wrapper(void);
/*
 * ********HAL COMMUNICATION LAYERS******************
 */

#ifdef HW_PERTINAKS
  extern I2C_HandleTypeDef hi2c1;
  extern I2C_HandleTypeDef hi2c2;
  extern UART_HandleTypeDef huart4;
  extern UART_HandleTypeDef huart5;
  extern TIM_HandleTypeDef htim2;
  extern UART_HandleTypeDef huart6;
  BNO055Init_TypeDef_t localBNO = {
      .i2cHandler = &hi2c2,
      .i2cAddress = BNO055_I2C_ADDR_LOW,
      .i2cTimeout = 10,
      .dmaRxCallback    = Callback_BNO_DMA_Rx,
      .dmaErrorCallback = Callback_BNO_Error,
      .delayCallback    = My_RTOS_Delay_Func,
      .powerMode     = BNO_PWR_MODE_NORMAL,
      .operationMode = BNO_MODE_IMU,
      .externalCrystal = 0,
      .axisRemap = BNO_AXIS_REMAP_P1,
      .accelUnit = BNO_ACC_UNIT_MS2,
      .gyroUnit  = BNO_GYRO_UNIT_DPS,
      .eulerUnit = BNO_EULER_UNIT_DEG,
      .tempUnit  = BNO_TEMP_UNIT_C,
      .useStoredCalibration = 1,
      .calibrationData = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x01}
  };
Maestro_Handler_t ServoDriver = { &huart4 , FAST  , FAST};



#elif defined(HW_PCB)
  extern I2C_HandleTypeDef hi2c1;	//BNO
  extern I2C_HandleTypeDef hi2c3; //MS
  extern UART_HandleTypeDef huart3;
  extern TIM_HandleTypeDef htim2;
  extern UART_HandleTypeDef huart6;
  #define BUZZER_PIN  GPIO_PIN_7
  #define BUZZER_PORT GPIOE
  #define ACIL_BUTON      GPIO_PIN_2
  #define ACIL_BUTON_PORT GPIOA
  BNO055Init_TypeDef_t localBNO = {
    .i2cHandler = &hi2c1,
    .i2cAddress = BNO055_I2C_ADDR_LOW,
    .i2cTimeout = 10,
    .dmaRxCallback    = Callback_BNO_DMA_Rx,
    .dmaErrorCallback = Callback_BNO_Error,
    .delayCallback    = My_RTOS_Delay_Func,
    .powerMode     = BNO_PWR_MODE_NORMAL,
    .notifyCallback = Notify_wrapper,
    .operationMode = BNO_MODE_IMU,
    .externalCrystal = 0,
    .axisRemap = BNO_AXIS_REMAP_P1,
    .accelUnit = BNO_ACC_UNIT_MS2,
    .gyroUnit  = BNO_GYRO_UNIT_DPS,
    .eulerUnit = BNO_EULER_UNIT_DEG,
    .tempUnit  = BNO_TEMP_UNIT_C,
    .useStoredCalibration = 0,
    .calibrationData = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x01}
  };
  Maestro_Handler_t ServoDriver = { &huart3 , FAST  , FAST};

#elif defined(HW_FOTA)

#endif


#define STM_ESP_RX_BUFFER_SIZE  128






static float lastUpdatedPressure;
static float lastUpdatedDepth;
static float lastUpdatedPitch;
static float lastUpdatedYaw;
static float lastUpdatedRoll;
static float lastUpdatedAccelx;
static float lastUpdatedAccely;
static float lastUpdatedAccelz;
static float lastUpdatedVelocityx;
static float lastUpdatedVelocityy;
static float lastUpdatedVelocityz;

static float lastUpdatedDistancex;
static float lastUpdatedDistancey;
static float lastUpdatedDistancez;

static float lastUpdatedStern;
static float lastUpdatedRudder;
static float lastUpdatedPWM;
static float lastUpdatedRPM;


/*
 * ************GLOBAL PID VARIABLES******************
 */

/*
 * ************GLOBAL PID VARIABLES******************
 */

PID_Config_t g_YawPID = {
    .Kp = 3.0f,
    .Ki = 0.0f,
    .Kd = 0.05f,
    .dt = 0.02f,

    .setpoint      = 270.0f,
    .lastError     = 0.0f,
    .integralError = 0.0f,

    .outputLimit   = 55.0f,
    .integralLimit = 10.0f

};



PID_Config_t g_DepthPID = {
    .Kp = 10.0f,
    .Ki = 0.0f,
    .Kd = 1.0f,
    .dt = 0.05f,

    .setpoint      = 1.0f,
    .lastError     = 0.0f,
    .integralError = 0.0f,

    .outputLimit   = 35.0f,
    .integralLimit = 30.0f

};


PID_Config_t g_PitchPID = {
    .Kp = 3.0f,
    .Ki = 0.0f,
    .Kd = 0.05f,
    .dt = 0.02f,

    .setpoint      = 1.0f,
    .lastError     = 0.0f,
    .integralError = 0.0f,

    .outputLimit   = 40.0f,
    .integralLimit = 30.0f
};

PID_Config_t g_EnginePID = 
{
  .Kp = 20,
  .Ki = 1,
  .Kd = 5, 
  .dt = 0.2f,
  .setpoint = 0.0f,
  .lastError     = 0.0f,
  .integralError = 0.0f,

  .outputLimit   = 800.0f,
  .integralLimit = 100.0f
};





volatile uint8_t g_ARM_STATUS = 0;


/*************************************************************************
 * PRIVATE HANDLES
 ***************************************************************************/

/* Queue Handles */

static QueueHandle_t xMaestroCmdQueue;
static QueueHandle_t xCmdQueue;
static QueueHandle_t xEngineControlQueue;
static SemaphoreHandle_t xEskfMutex;
/* Semaphore Handles */
static SemaphoreHandle_t xMS5837_BinarySem;   // NS5837 DMA complete signal
/* Task Handles */
static TaskHandle_t xTaskBNO_Read;
static TaskHandle_t xTaskYawRollControl;
static TaskHandle_t xTaskPitchControl;
static TaskHandle_t xTaskDepthControl;
static TaskHandle_t xTaskMS5837;
static TaskHandle_t xMaestroGateKeeper;
static TaskHandle_t xCommRxTask;
static TaskHandle_t xCommandHandler;
static TaskHandle_t xCommTxTask;
static TaskHandle_t xEskfTask;
static TaskHandle_t xEngineControlTask;
static TaskHandle_t xEnginePIDTask;


static void vBNOTask(void *pvParameters);

static void vPitchPidTask(void *pvParameters);
static void vDepthPidTask(void *pvParameters);
static void vYawPidTask(void *pvParameters);
static void vEngineControlTask(void *parameters);
static void vMS5837Task(void *pvParameters);
static void vMaestroGatekeeperTask(void* pvParameters);
static void vCommRxTask(void * parameters);
static void vCommTxTask(void* parameters);
static void vCommandHandler(void* parameters);
static void vEskfTask(void* parameters);
static void vEnginePidTask(void *pvParameters);

/*
 * ################GLOBAL SYSTEM INIT FUNCTION######################
 */

void System_Tasks_Init(void){

    xMS5837_BinarySem   = xSemaphoreCreateBinary();
    xMaestroCmdQueue    = xQueueCreate(10 , sizeof(MaestroMsg_t));
    xCmdQueue           = xQueueCreate(10 , sizeof(CommandData_t));
    xEngineControlQueue = xQueueCreate(10,  sizeof(uint16_t));
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

       
       
       xTaskCreate(vCommandHandler,
        "Engine Task",
        1024,
        NULL,
        TASK_PID_MSG,
        &xCommandHandler);
        
        
        xTaskCreate(vEskfTask ,
          "ESKF Predict",
          TASK_STACK_ESKF,
          NULL,
          TASK_PRIORITY_ESKF,
          &xEskfTask
        );
        

        xTaskCreate(vEngineControlTask , 
          "Engine Control", 
          TASK_STACK_ENGINE,
          NULL ,
          TASK_PRIORITY_VELOCITY, 
          &xEngineControlTask);

        xTaskCreate(vEnginePidTask, 
          "Engine PID", 
          TASK_STACK_SPEED_CONTROL,
          NULL ,
          TASK_PRIORITY_VELOCITY, 
          &xEnginePIDTask);
          
       SubESKF_Init();

    vTaskStartScheduler();
}

static void vEnginePidTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PITCH_CONTROL_PERIOD_MS);
    
    uint16_t target_pwm = 1000;

    for(;;)
    {
        float current_dist;

        // X eksenindeki mesafeyi güvenli bir şekilde al
        portENTER_CRITICAL();
        current_dist = lastUpdatedDistancex;
        portEXIT_CRITICAL();

        // 40 metre hedefine ulaşıldı mı kontrolü
        if(current_dist < 40.0f)
        {
            target_pwm = 1700; // 40 metrenin altındaysa ileri gitmeye devam et
        }
        else
        {
            target_pwm = 1000; // 40 metreye ulaşıldı, motorları tamamen durdur
            portENTER_CRITICAL();
            g_YawPID.setpoint = 90.0f;
            portEXIT_CRITICAL();
        }

        // Komutu motor kuyruğuna gönder
        xQueueSend(xEngineControlQueue, &target_pwm, 0);

        // Görev periyodunu bekle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void vEngineControlTask(void *parameters)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);

    uint16_t currentPWM = 1000; 
    uint16_t targetPWM  = 1000;
    
    const uint16_t min_signal = 1000;
    const uint16_t max_signal = 2000;
    const uint16_t step_size  = 10;

    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for(;;)
    {
        uint16_t new_command;
        
        if(xQueueReceive(xEngineControlQueue, &new_command, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            if(new_command <= max_signal && new_command >= min_signal)
            {
                targetPWM = new_command; 
            }
        }

       
        if (currentPWM != targetPWM)
        {
            if (currentPWM < targetPWM)
            {
                if (targetPWM - currentPWM < step_size) currentPWM = targetPWM;
                else currentPWM += step_size;
            }
            else 
            {
                if (currentPWM - targetPWM < step_size) currentPWM = targetPWM;
                else currentPWM -= step_size;
            }

            portENTER_CRITICAL();
            lastUpdatedPWM = currentPWM; 
            portEXIT_CRITICAL();

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, currentPWM);
        }
    }
}


static void vEskfTask(void* parameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz loop
    const float dt = 0.01f;

    static float model_velocity = 0.0f; 

    float currentaccel;
    float current_pwm;

    for(;;)
    {
        // 1. İhtiyacımız olan son güncel verileri (İvme ve PWM) güvenli bölgede alıyoruz
        portENTER_CRITICAL();
        currentaccel = lastUpdatedAccelx;
        current_pwm = lastUpdatedPWM;
        portEXIT_CRITICAL();

        // 2. Fizik modelini işlet (dt = 0.01 ile 100 Hz'de güncelleniyor)
        pwm_to_velocity(current_pwm, &model_velocity, dt);

        SubESKF_Step(model_velocity, currentaccel, dt);

        portENTER_CRITICAL();
        SubESKF_GetVelocity(&lastUpdatedVelocityx);
        SubESKF_GetPosition(&lastUpdatedDistancex);
        portEXIT_CRITICAL();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}



void I2C_Clear(GPIO_TypeDef *SCL_Port, uint16_t SCL_Pin, GPIO_TypeDef *SDA_Port, uint16_t SDA_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = SCL_Pin | SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(SCL_Port, &GPIO_InitStruct); // Not: Aynı porttalarsa tek seferde, farklı porttalarsa ayrı ayrı init edilmeli
    
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_SET);
    vTaskDelay(2);

    for (int i = 0; i < 9; i++) 
    {
        HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_RESET);
        vTaskDelay(2); // vTaskDelay milisaniye cinsinden gecikme sağlar
        HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
        vTaskDelay(2);
        
        if (HAL_GPIO_ReadPin(SDA_Port, SDA_Pin) == GPIO_PIN_SET) {
            break;
        }
    }

    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_RESET);
    vTaskDelay(2);
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    vTaskDelay(2);
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_SET);
    vTaskDelay(2);
}








/******************************************************************************
 * ************************BNO READ TASK***************************************
 ******************************************************************************/
static void vBNOTask(void *pvParameters)
{
  BNO_Status_t status = BNO_TIMEOUT;
  vTaskDelay(2000);
  status = BNO055_Init(&localBNO);

  while(status != BNO_OK)
  {
      
      vTaskDelay(50);

      status = BNO055_Init(&localBNO);
  }
  
  portENTER_CRITICAL();
  PID_Reset(&g_YawPID);
  portEXIT_CRITICAL();
  
  BNO055_EulerData_t tmp;
  BNO055_AccelData_t acctmp;

  #define BNO_BURST_READ_SIZE  20
  static uint8_t burst_buffer[BNO_BURST_READ_SIZE];

  #define MY_ABS(x) ((x) < 0.0f ? -(x) : (x))

  float yawOffset = 0.0f;
  uint8_t isOffsetSet = 0;
  uint8_t initCounter = 0;

  // Glitch koruması için bayrak ve değişkenler
  float lastValidRoll = 0.0f;
  float lastValidPitch = 0.0f;
  uint8_t isFirstValidRead = 0;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(BNO_READ_PERIOD_MS);

  for(;;) {
    HAL_I2C_Mem_Read_DMA(localBNO.i2cHandler, localBNO.i2cAddress, BNO055_EUL_HEADING_LSB, 1, burst_buffer, BNO_BURST_READ_SIZE);  
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));
        
    BNO055_ParseEulerBuffer(&localBNO, &burst_buffer[0], &tmp);
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
    while (finalYaw >= 360.0f) finalYaw -= 360.0f;
    while (finalYaw < 0.0f)    finalYaw += 360.0f;



    portENTER_CRITICAL();
    lastUpdatedYaw = finalYaw;
    lastUpdatedRoll = tmp.roll;
    lastUpdatedPitch = tmp.pitch;
    lastUpdatedAccelx = acctmp.acc_x - SubESKF_GetBias(); 
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

    #ifdef HW_PCB
    if(MS5837_Init(&localMS5837, &hi2c3) != HAL_OK){
            SEGGER_SYSVIEW_Error("MS5837 INIT FAIL");
            vTaskDelete(NULL);
    }
    #elif defined(HW_PERTINAKS)
    if(MS5837_Init(&localMS5837, &hi2c1) != HAL_OK){
            SEGGER_SYSVIEW_Error("MS5837 INIT FAIL");
            vTaskDelete(NULL);
    }
    #endif

    DepthData_t tx_Depth;
    uint8_t cmd_d1   = MS5837_CONVERT_BASE | D1 | OSR_4096;
    uint8_t cmd_d2   = MS5837_CONVERT_BASE | D2 | OSR_4096;
    uint8_t cmd_read = MS5837_ADC_READ;

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(MS5837_READ_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for(;;){

      MS5837_Send_Command_DMA(&localMS5837, cmd_d1);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
      vTaskDelay(pdMS_TO_TICKS(20)); 
      MS5837_Send_Command_DMA(&localMS5837, cmd_read);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      MS5837_Read_ADC_DMA(&localMS5837);
      if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
          localMS5837.D1_Pres_Raw = MS5837_Parse_ADC(&localMS5837);
      }
      MS5837_Send_Command_DMA(&localMS5837, cmd_d2);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(20)); 
      MS5837_Send_Command_DMA(&localMS5837, cmd_read);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      MS5837_Read_ADC_DMA(&localMS5837);
      if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
         localMS5837.D2_Temp_Raw = MS5837_Parse_ADC(&localMS5837);
      }

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

  #define PITCHCHANNELS 0b1100
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(PITCH_CONTROL_PERIOD_MS);

  for(;;){
     portENTER_CRITICAL();
     current_pitch = lastUpdatedPitch;   
     portEXIT_CRITICAL();

     servo_cmd = PID_Calculate(&g_PitchPID, current_pitch);

     msg1.channel = CH4;
     msg1.target = servo_cmd + SERVO_CENTER_DEG;
     xQueueSend(xMaestroCmdQueue, &msg1, 0);
   
     msg2.channel = CH3;
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
     g_PitchPID.setpoint = -1 * desiredPitch;
     portEXIT_CRITICAL();


     vTaskDelayUntil(&xLastWakeTime , xFrequency);
}
}

static void vYawPidTask(void *pvParameters){
  static MaestroMsg_t msg1;
  static MaestroMsg_t msg2;
#define YAWCHANNELS 0b0011
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(PITCH_CONTROL_PERIOD_MS);

  float servo_cmd;
  xLastWakeTime = xTaskGetTickCount();

  for(;;){
    servo_cmd = PID_Calculate(&g_YawPID , lastUpdatedYaw);

    msg1.channel = CH0;
    msg1.target = servo_cmd + SERVO_CENTER_DEG;
    xQueueSend(xMaestroCmdQueue, &msg1, 0);

    msg2.channel = CH1;
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

                if(msg.channel == CH0 ){
                  lastUpdatedRudder = SERVO_CENTER_DEG - msg.target;
                }

                if(msg.channel == CH3){
                  lastUpdatedStern = SERVO_CENTER_DEG - msg.target;
                }
              
                
             

                HAL_UART_Transmit_DMA(ServoDriver.huart, command, CMD_LEN);
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            }
            vTaskDelay(2);
        }
    }
}


static uint32_t g_CurrentThrottle = 1000;

static void vCommandHandler(void* parameters)
{
    static CommandData_t command;
    static uint16_t target = 1600;
    xQueueSend(xEngineControlQueue, &target, 100);
    for(;;)
    {
        xQueueReceive(xCmdQueue, &command, portMAX_DELAY);
    
        switch (command.command) 
        {
            case ARM:
                if(xTaskBNO_Read != NULL)
                {
                    xTaskNotify(xTaskBNO_Read, 0, eNoAction);
                }
                
                if(xEngineControlTask != NULL)
                {
                  if(eTaskGetState(xEngineControlTask) == eBlocked)
                    xTaskNotify(xEngineControlTask, 0, eNoAction);
                }else {
                    vTaskResume(xEngineControlTask);
                }
                break; 

            case DISARM:
                if(xEngineControlTask != NULL)
                {
                    vTaskSuspend(xEngineControlTask); 
                }
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
                portENTER_CRITICAL();
                lastUpdatedPWM = 1000;
                portEXIT_CRITICAL();

                HAL_GPIO_WritePin(GPIOA , GPIO_PIN_2 , SET);
                break;

            case YUNUSLAMA:
                portENTER_CRITICAL();
                g_PitchPID.Kp = 8.0f;
                g_PitchPID.Ki = 0.0f;
                g_PitchPID.Kd = 0.2f;
                g_PitchPID.dt = 0.02f;
                g_PitchPID.setpoint      = 1.0f;
                g_PitchPID.lastError     = 0.0f;
                g_PitchPID.integralError = 0.0f;
                g_PitchPID.outputLimit   = 50.0f;
                g_PitchPID.integralLimit = 30.0f;          
                g_DepthPID.Kp = 10.0f;
                g_DepthPID.Ki = 0.0f;
                g_DepthPID.Kd = 1.0f;
                g_DepthPID.dt = 0.05f;
                g_DepthPID.setpoint      = -1.0f;
                g_DepthPID.lastError     = 0.0f;
                g_DepthPID.integralError = 0.0f;
                g_DepthPID.outputLimit   = 50.0f;
                g_DepthPID.integralLimit = 30.0f;
                portEXIT_CRITICAL();
                break;

            case TURN:
                portENTER_CRITICAL(); 
                g_YawPID.setpoint = command.value; 
                lastUpdatedDistancex = 5; 
                g_EnginePID.setpoint = 0;
                portEXIT_CRITICAL();
                break;

            case DEPTH:
                portENTER_CRITICAL(); 
                g_DepthPID.setpoint = command.value;  
                portEXIT_CRITICAL();
                break;

            case GO_TO:
                portENTER_CRITICAL(); 
                g_EnginePID.setpoint = command.value;  
                portEXIT_CRITICAL();
                break;

            case SYSTEM_RESET:
                HAL_NVIC_SystemReset();
                break;
                
            default:
                break;
        }
    }
}











static void vCommRxTask(void * parameters)
{

  for(;;)
  {
    static CommandData_t cmd;
    HAL_UART_Receive_DMA(&huart6 , (uint8_t *)&cmd , sizeof(CommandData_t));
    ulTaskNotifyTake(pdTRUE , portMAX_DELAY);
    xQueueSend(xCmdQueue , &cmd , 0);

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
    toSendlist.velocityx = lastUpdatedVelocityx;
    toSendlist.velocityy = lastUpdatedVelocityy;
    toSendlist.velocityz = lastUpdatedVelocityz;
    toSendlist.distancex = lastUpdatedDistancex;
    toSendlist.distancey = lastUpdatedDistancey;
    toSendlist.distancez = lastUpdatedDistancez;
    toSendlist.rpm       = lastUpdatedPWM;
    toSendlist.rudderangle  = lastUpdatedRudder;
    toSendlist.sternangle   = lastUpdatedStern;
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



void Notify_wrapper(void)
{
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}