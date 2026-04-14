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
extern UART_HandleTypeDef huart3;
//extern UART_HandleTypeDef huart4;

Maestro_Handler_t ServoDriver = { &huart3 , FAST  , FAST};

static float lastUpdatedDepth;
static float lastUpdatedPitch;
static float lastUpdatedYaw;
static float lastUpdatedRoll;

/*
 * ************GLOBAL PID VARIABLES******************
 */
/*
 * ************GLOBAL PID VARIABLES******************
 */

PID_Config_t g_PitchPID = {
    .Kp = 1.0f,
    .Ki = 0.0f,   // Telefondan BT ile güncellenecek
    .Kd = 0.0f,   // Telefondan BT ile güncellenecek
    .dt = 0.1f,   // Task periyodun 100ms ise 0.1 saniye

    .setpoint      = 0.0f, // Araç burnunu düz (0 derece) tutsun
    .lastError     = 0.0f, // Başlangıçta 0
    .integralError = 0.0f, // Başlangıçta birikmiş hata 0

    // --- Güvenlik ve Limitler ---
    .outputLimit   = 50.0f, // Maestro'ya gidecek max sapma (Açıklamayı oku)
    .integralLimit = 10.0f  // Anti-Windup limiti (Çıkış limitinin %20'si iyi bir başlangıçtır)
};

PID_Config_t g_YawPID = {
    // --- Ayarlanabilir Katsayılar ---
    .Kp = 1.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .dt = 0.1f,

    // --- Durum Değişkenleri ---
    .setpoint      = 0.0f, // Pusulada istenen baş açısı (Hedef)
    .lastError     = 0.0f,
    .integralError = 0.0f,

    // --- Güvenlik ve Limitler ---
    .outputLimit   = 50.0f,
    .integralLimit = 10.0f
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
/*
 * BT message handle
 */
static char* volatile g_latest_BT_Msg_Ptr = NULL;
static void vBNOTask(void *pvParameters);
static void vPitchPidTask(void *pvParameters);
static void vYawRollPidTask(void *pvParameters);
static void vMS5837Task(void *pvParameters);
static void vMaestroGatekeeperTask(void* pvParameters);

/*
 * ################GLOBAL SYSTEM INIT FUNCTION######################
 */

void System_Tasks_Init(void){
    xMS5837_BinarySem   = xSemaphoreCreateBinary();
    xMaestroCmdQueue    = xQueueCreate(5 , sizeof(MaestroMsg_t));



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
      .useStoredCalibration = 0,
      .calibrationData = {0}
  };

  status = BNO055_Init(&localBNO);
  if(status != BNO_OK)
      {
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
          vTaskDelete(NULL);
      }
  BNO055_EulerData_t tmp;
  static uint8_t DMA_rx_buffer[6];

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(BNO_READ_PERIOD_MS);

  //char msg[50];
  for(;;){
    HAL_I2C_Mem_Read_DMA(localBNO.i2cHandler ,
          localBNO.i2cAddress,
          BNO055_EUL_HEADING_LSB,
          1,
          DMA_rx_buffer,
          6);
       ulTaskNotifyTake(pdTRUE , portMAX_DELAY);
       BNO055_ParseEulerBuffer(&localBNO, DMA_rx_buffer, &tmp);
       float mapped_heading = 450.0f - tmp.heading;
       while (mapped_heading >= 360.0f) mapped_heading -= 360.0f;
       while (mapped_heading < 0.0f)    mapped_heading += 360.0f;
       lastUpdatedYaw = mapped_heading;

       lastUpdatedRoll    = tmp.roll;
       lastUpdatedPitch    = tmp.pitch;
       vTaskDelayUntil(&xLastWakeTime , xFrequency);

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

      // D1 convert
      MS5837_Send_Command_DMA(&localMS5837, cmd_d1);
      xSemaphoreTake(xMS5837_BinarySem, pdMS_TO_TICKS(10));
      vTaskDelay(pdMS_TO_TICKS(10));

      //D1 Read
      MS5837_Send_Command_DMA(&localMS5837, cmd_read);
      xSemaphoreTake(xMS5837_BinarySem, pdMS_TO_TICKS(10));

      MS5837_Read_ADC_DMA(&localMS5837);
      // Okuma bitince ISR semaforu verecek
      if(xSemaphoreTake(xMS5837_BinarySem, pdMS_TO_TICKS(10)) == pdTRUE) {
          localMS5837.D1_Pres_Raw = MS5837_Parse_ADC(&localMS5837);
      }

      //D2 Convert
      MS5837_Send_Command_DMA(&localMS5837, cmd_d2);
      xSemaphoreTake(xMS5837_BinarySem, pdMS_TO_TICKS(10));
      vTaskDelay(pdMS_TO_TICKS(10));

      //D2 Read
      MS5837_Send_Command_DMA(&localMS5837, cmd_read);
      xSemaphoreTake(xMS5837_BinarySem, pdMS_TO_TICKS(10));

      MS5837_Read_ADC_DMA(&localMS5837);
      if(xSemaphoreTake(xMS5837_BinarySem, pdMS_TO_TICKS(10)) == pdTRUE) {
         localMS5837.D2_Temp_Raw = MS5837_Parse_ADC(&localMS5837);
      }

      MS5837_Calculate(&localMS5837);

      tx_Depth.depth    = localMS5837.depth;
      tx_Depth.pressure = localMS5837.P;

      portENTER_CRITICAL();
      lastUpdatedDepth = tx_Depth.depth;
      portEXIT_CRITICAL();

      //xQueueSend(xQueueDepth, &tx_Depth, 0);
      //We already waited for 20 seconds during tasks.
      vTaskDelayUntil(&xLastWakeTime , xFrequency);
    }
}




static void vPitchPidTask(void *pvParameters){
    MaestroMsg_t msg = {.channel = CH0 ,.target = 0};
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(PITCH_CONTROL_PERIOD_MS);
    float servo_cmd;
    xLastWakeTime = xTaskGetTickCount();
  for(;;){
    servo_cmd = PID_Calculate(&g_PitchPID , lastUpdatedDepth);
    msg.channel = 0;
    msg.target = servo_cmd;
    xQueueSend(xMaestroCmdQueue, &msg, 0);

    msg.channel = 1;
    msg.target = servo_cmd;
    xQueueSend(xMaestroCmdQueue, &msg, 0);
    vTaskDelayUntil(&xLastWakeTime , xFrequency);
  }
}





static void vYawRollPidTask(void *pvParameters){
  MaestroMsg_t msg = {.channel = CH2 ,.target = 0};
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(PITCH_CONTROL_PERIOD_MS);
  float servo_cmd;
  xLastWakeTime = xTaskGetTickCount();
  for(;;){
    servo_cmd = PID_Calculate(&g_YawPID , lastUpdatedYaw);
    msg.target = servo_cmd;
    xQueueSend(xMaestroCmdQueue , &msg , 0);
    vTaskDelayUntil(&xLastWakeTime , xFrequency);
  }
}
uint8_t message = 0x55;
static void vMaestroGatekeeperTask(void *pvParameters) {
    MaestroMsg_t msg;
    static uint8_t command[4];
    HAL_StatusTypeDef status;
    for(;;) {
        if (xQueueReceive(xMaestroCmdQueue, &msg, portMAX_DELAY) == pdPASS) {
            Maestro_SetTarget(&ServoDriver, msg.channel, msg.target, command);
            if(ServoDriver.huart->gState == HAL_UART_STATE_READY){
              status = HAL_UART_Transmit_DMA(ServoDriver.huart, message, 1);
            }
           ulTaskNotifyTake(pdFALSE , portMAX_DELAY);

        }
    }
}

/*****************************************************************************
 *****************************BNO DMA HELPERS*********************************
 *****************************************************************************/
void Callback_BNO_DMA_Rx(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xTaskBNO_Read , &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
    xSemaphoreGiveFromISR(xMS5837_BinarySem , &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void MS5837_DMA_Error_Callback(){
     SEGGER_SYSVIEW_Print("MS5837 DMA EXPLODED");
}



