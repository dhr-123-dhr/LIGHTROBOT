/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  *
  * 任务架构 (参照 Robot):
  *   defaultTask: 任务调度/指令编排 (Normal)
  *   gyroTask:    陀螺仪 1ms 实时轮询 (High) + 底盘 yaw 注入
  *   printTask:   100ms 串口打印位姿 & 目标线速度 (Low)
  *
  * 注意:
  *   - MotorCtrl_Update() 在 TIM14 1kHz 硬件中断回调中调用 (最高实时)
  *   - chassis_tick() 在 TIM14 回调中紧接 MotorCtrl_Update 调用
  *   - gyroTask 异步: MPU6050_DMP_Get_YawPitchRoll() 后注入底盘
  *   - printTask: 100ms 串口发送, 低位姿数据量 (~200B/次), 安全
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Chassis.h"
#include "mpu6050.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gyroTask */
osThreadId_t gyroTaskHandle;
const osThreadAttr_t gyroTask_attributes = {
  .name = "gyroTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for printTask */
osThreadId_t printTaskHandle;
const osThreadAttr_t printTask_attributes = {
  .name = "printTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartGyroTask(void *argument);
void StartPrintTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* gyroTask: 陀螺仪 1ms 轮询 (高优先级) */
  gyroTaskHandle = osThreadNew(StartGyroTask, NULL, &gyroTask_attributes);
  /* printTask: 100ms 串口日志 (低优先级) */
  printTaskHandle = osThreadNew(StartPrintTask, NULL, &printTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* 等待陀螺仪初始化完成 (首次 yaw 数据就绪) */
  osDelay(500);

  /* 演示指令序列 */
  enum { MOVE1, ROT1, MOVE2, ROT2, END };
  uint8_t demo_state = MOVE1;

  for(;;)
  {
    switch(demo_state)
    {
      case MOVE1:
        /* 直线前进 500mm, 陀螺仪锁头 */
        Chassis_Move(500.0f);
        while (Chassis_IsBusy()) { osDelay(10); }
        demo_state = ROT1;
        break;
      case ROT1:
        /* 原地左转 90°, 不锁头 */
        Chassis_Rotate(90.0f);
        while (Chassis_IsBusy()) { osDelay(10); }
        demo_state = MOVE2;
        break;
      case MOVE2:
        /* 直线后退 300mm, 陀螺仪锁头 */
        Chassis_Move(500.0f);
        while (Chassis_IsBusy()) { osDelay(10); }
        demo_state = ROT2;
        break;
      case ROT2:
        /* 原地右转 90°, 不锁头 */
        Chassis_Rotate(90.0f);
        while (Chassis_IsBusy()) { osDelay(10); }
        demo_state = MOVE1;
        break;
      case END:
      default:
        osDelay(1000);
        break;
    }
    osDelay(500);  /* 指令间间隔 */
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartGyroTask */
/**
  * @brief  陀螺仪轮询任务 (1ms 周期, 对齐 Robot)
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGyroTask */
void StartGyroTask(void *argument)
{
  /* USER CODE BEGIN StartGyroTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();

  /* 等待 MPU6050 DMP 初始化完成 (最多 3 秒) */
  {
    uint8_t retry = 0;
    while (mpu_dmp_init() != 0) {
      osDelay(100);
      if (++retry > 30) {
        /* 超时, 放弃 (底盘将用编码器仅推算) */
        vTaskDelete(NULL);
      }
    }
  }

  for(;;)
  {
    /* 严格 1ms 节拍 (对齐 Robot 300us 任务精度, FreeRTOS 1ms tick) */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));

    /* 读取 DMP 姿态: 非阻塞流水线 < 50us */
    float yaw, pitch, roll;
    if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
    {
      /* 注入底盘: 直线运动 yaw PD 航向锁定, 旋转模式只读不控 */
      /* Robot 调用: chassis_feed_gyro(yaw); */
      chassis_feed_gyro(yaw);
    }

    /* 显式让出 CPU: 确保 printTask/defaultTask 获得调度
     * 即使本任务 < 1ms 完成, 也让低优先级任务有机会运行 */
    taskYIELD();
  }
  /* USER CODE END StartGyroTask */
}

/* USER CODE BEGIN Header_StartPrintTask */
/**
  * @brief  100ms 串口打印位姿 & 目标线速度 (上位机观测)
  *
  * 打印内容:
  *   位姿 (X, Y, Theta)  |  目标线速度 V_cmd  |  左右轮实际速度 (VL, VR)
  *   X/Y 单位 mm, Theta 单位 deg, 速度单位 mm/s
  *
  * 波特率: 115200 8N1 (ST-Link VCP USART2: PD5 TX)
  * 数据量: ~80 字节/次, 100ms 周期, ~1KB/s, 不阻塞
  *
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPrintTask */
void StartPrintTask(void *argument)
{
  /* USER CODE BEGIN StartPrintTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();

  /* 等待系统就绪 (陀螺仪校准 ~500ms, 底盘初始化) */
  vTaskDelay(pdMS_TO_TICKS(2000));

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

    /* 读取偏航角 */
    float theta;
    chassis_get_pose(NULL, NULL, &theta);

    /* 读取目标线速度 */
    float v_cmd = chassis_get_cmd_speed();

    /* 读取左右轮实际线速度 */
    float vL, vR;
    chassis_get_wheel_speed(&vL, &vR);

    /* 格式化输出: 简洁一行
     * 格式: "TH:%7.1f V:%6.1f VL:%6.1f VR:%6.1f\r\n"
     * 示例: "TH:   90.0 V: 340.0 VL: 335.0 VR: 345.0" */
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
                       "TH:%7.1f V:%6.1f VL:%6.1f VR:%6.1f\r\n",
                       theta, v_cmd, vL, vR);

    /* 非阻塞发送 (100ms 超时保护, 防止 DMA 冲突) */
    if (len > 0 && len < (int)sizeof(buf)) {
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 100);
    }
  }
  /* USER CODE END StartPrintTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */