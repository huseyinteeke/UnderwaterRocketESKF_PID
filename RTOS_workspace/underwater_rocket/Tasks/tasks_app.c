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
extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

/*
 * ************GLOBAL PID VARIABLES******************
 */
volatile PID_Config_t g_PitchPID = {1.0f , 0.0f , 0.0f , 0.1f , 0.0f , 0.0f , 0.0f , 0.0f};
volatile PID_Config_t g_YawPID =   {1.0f , 0.0f , 0.0f , 0.1f , 0.0f , 0.0f , 0.0f , 0.0f};
volatile uint8_t g_ARM_STATUS = 0;


/*************************************************************************
 * PRIVATE HANDLES
 ***************************************************************************/

/* Queue Handles */

static QueueHandle_t xQueueYawRoll;                   // Yaw data queue
static QueueHandle_t xQueuePitch;                // Pitch/Roll data queue
static QueueHandle_t xQueueDepth;               // Depth data queue

/* Semaphore Handles */
static SemaphoreHandle_t xBNO_DMA_Semaphore;  // BNO055 DMA complete signal
static SemaphoreHandle_t xMS5837_BinarySem;   // NS5837 DMA complete signal
/* Task Handles */
static TaskHandle_t xTaskBNO_Read;
static TaskHandle_t xTaskYawRollControl;
static TaskHandle_t xTaskPitchControl;
static TaskHandle_t xTaskMS5837;
static TaskHandle_t xTaskBluetooth;

/*
 * BT message handle
 */
static char* volatile g_latest_BT_Msg_Ptr = NULL;
static void vBNOTask(void *pvParameters);
static void vPitchPidTask(void *pvParameters);
static void vYawRollPidTask(void *pvParameters);
static void vMS5837Task(void *pvParameters);
static void vBTTask(void *pvParameters);

/*
 * ################GLOBAL SYSTEM INIT FUNCTION######################
 */

void System_Tasks_Init(void){
    xQueueYawRoll       = xQueueCreate(QUEUE_SIZE_YAW       , sizeof(YawRollData_t));
    xQueuePitch         = xQueueCreate(QUEUE_SIZE_PITCHROLL , sizeof(PitchData_t));
    xQueueDepth         = xQueueCreate(QUEUE_SIZE_DEPTH     , sizeof(DepthData_t));
    xBNO_DMA_Semaphore  = xSemaphoreCreateBinary();
    xMS5837_BinarySem   = xSemaphoreCreateBinary();

    vQueueAddToRegistry(xQueueYawRoll, "Q_YawRoll");
    vQueueAddToRegistry(xQueueDepth,   "Q_Depth");
    vQueueAddToRegistry(xQueuePitch,   "Q_Pitch");
    if(xBNO_DMA_Semaphore){
        xTaskCreate(vBNOTask ,
                    "BNO_READ" ,
                    TASK_STACK_BNO_READ ,
                    NULL,
                    TASK_PRIORITY_BNO_READ ,
                    &xTaskBNO_Read);
    }
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
        xTaskCreate(vBTTask,
                "BT task",
                TASK_STACK_BT,
                NULL,
                TASK_PRIORITY_BT,
                &xTaskBluetooth);
    vTaskStartScheduler();
}



/******************************************************************************
 * ************************BT ISR HANDLER***************************************
 ******************************************************************************/
void BT_ISR_Data_Handler(char* message)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    g_latest_BT_Msg_Ptr = message;
    vTaskNotifyGiveFromISR(xTaskBluetooth , &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}




/******************************************************************************
 * ************************BT TASK***************************************
 ******************************************************************************/
static void vBTTask(void *pvParameters) {
  ParsedCommand_t cmd;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    SEGGER_SYSVIEW_Print("BT TASK ACTIVE");

    if (Comm_Get_Parsed_Command(&cmd)) {
      switch (cmd.type) {
      case CMD_ARM:
        g_ARM_STATUS = 1;
        //xTaskNotify __ ENGINE task
        Comm_Send_Response("ARMED\n");
        break;

      case CMD_DISARM:
        g_ARM_STATUS = 0;
        //xTaskNotify __ ENGINE task
        Comm_Send_Response("DISARMED\n");
        break;
      case CMD_SET_PID_PITCH:
        g_PitchPID.Kp = cmd.val1;
        g_PitchPID.Kd = cmd.val2;
        g_PitchPID.Ki = cmd.val3;
        Comm_Send_Response("Done Pitch PID");
        break;
      case CMD_SET_PID_YAW:
        g_YawPID.Kp = cmd.val1;
        g_YawPID.Kd = cmd.val2;
        g_YawPID.Ki = cmd.val3;
        Comm_Send_Response("Done Yaw PID");
        break;
      case CMD_LED:
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        Comm_Send_Response("Done LED");
        break;
      default:
        break;
      }

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
      .i2cHandler = &hi2c1,
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
      .useStoredCalibration = 0,
      .calibrationData = {0}
  };

  status = BNO055_Init(&localBNO);
  if(status != BNO_OK)
      {
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
          vTaskDelete(NULL);
      }
  YawRollData_t tx_YawRoll;
  PitchData_t   tx_Pitch;
  BNO055_EulerData_t tmp;
  static uint8_t DMA_rx_buffer[6];

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(BNO_READ_PERIOD_MS);

  //char msg[50];
  for(;;){
       BNO055_ReadEuler_DMA(&localBNO, DMA_rx_buffer);

       if(xSemaphoreTake(xBNO_DMA_Semaphore , pdMS_TO_TICKS(20)) == pdTRUE){
           BNO055_ParseEulerBuffer(&localBNO, DMA_rx_buffer, &tmp);
           tx_YawRoll.heading = tmp.heading;
           tx_YawRoll.roll    = tmp.roll;
           tx_Pitch.pitch     = tmp.pitch;
           xQueueSend(xQueuePitch   , &tx_Pitch     , 0);
           xQueueSend(xQueueYawRoll , &tx_YawRoll   , 0);
       }

       vTaskDelayUntil(&xLastWakeTime , xFrequency);

  }

}

/********************************************************************************
 **********************************vMS5837 Read Task***********************
 ******************************************************************************/
static void vMS5837Task(void *pvParameters){
    static MS5837_t localMS5837;
    localMS5837.Delay = My_RTOS_Delay_Func;
    if(MS5837_Init(&localMS5837, &hi2c3) != HAL_OK){
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

      xQueueSend(xQueueDepth, &tx_Depth, 0);
      //We already waited for 20 seconds during tasks.
      vTaskDelayUntil(&xLastWakeTime , xFrequency);
    }
}




static void vPitchPidTask(void *pvParameters){
  for(;;){
        vTaskDelay(portMAX_DELAY);
      }
}





static void vYawRollPidTask(void *pvParameters){
  for(;;){
          vTaskDelay(portMAX_DELAY);
        }
}


/********************************************************************************
 **********************************STACK OVERFLOW CHECKER***********************
 ******************************************************************************/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // Hangi taskın patladığını anlamak için buraya Breakpoint koy!
    // pcTaskName değişkeni sana patlayan taskın adını ("BT_Task" gibi) söyler.

    Comm_Send_Response("FATAL: STACK OVERFLOW -> ");
    Comm_Send_Response(pcTaskName); // Hangi task öldü?

    // Motorları acil durdur!
    //Motor_Disarm();

    for(;;); // Sonsuz döngüde bekle
}

/*****************************************************************************
 *****************************BNO DMA HELPERS*********************************
 *****************************************************************************/
void Callback_BNO_DMA_Rx(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xBNO_DMA_Semaphore, &xHigherPriorityTaskWoken);
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
    // BNO055 I2C1 hattında ise:
    if(hi2c->Instance == I2C1)
    {
        // Direkt dosya içindeki fonksiyonu çağırıyoruz
        Callback_BNO_DMA_Rx();
    }
}

// 2. MASTER OKUMA (MS5837 Veri Okuma İçin) - BUNU EKLE!
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C3) // MS5837
    {
        MS5837_DMA_Callback();
    }
}

// 3. MASTER YAZMA (MS5837 Komut Gönderme İçin) - BUNU EKLE!
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C3) // MS5837
    {
        MS5837_DMA_Callback();
    }
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
    xSemaphoreGiveFromISR(xMS5837_BinarySem , &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void MS5837_DMA_Error_Callback(){
     SEGGER_SYSVIEW_Print("MS5837 DMA EXPLODED");
}



