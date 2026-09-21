/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS application code for Steady Spoon
  ******************************************************************************
  */
/* USER CODE END Header */


/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"


/* -------------------------------------------------------------------------- */
/* Functions implemented in main.c                                            */
/* -------------------------------------------------------------------------- */

extern void App_IMU_TaskStep(void);
extern void App_Control_TaskStep(void);


/* -------------------------------------------------------------------------- */
/* Task function prototypes                                                   */
/* -------------------------------------------------------------------------- */

void StartImuTask(void *argument);
void StartControlTask(void *argument);


/* -------------------------------------------------------------------------- */
/* Task handles                                                               */
/* -------------------------------------------------------------------------- */

osThreadId_t imuTaskHandle;
osThreadId_t controlTaskHandle;


/* -------------------------------------------------------------------------- */
/* IMU TASK CONFIGURATION                                                     */
/*                                                                            */
/* Stack size = 256 words                                                     */
/* 256 words x 4 bytes = 1024 bytes                                           */
/*                                                                            */
/* Priority is Above Normal because we want fresh IMU data before the          */
/* control task calculates the servo correction.                              */
/* -------------------------------------------------------------------------- */

const osThreadAttr_t imuTask_attributes =
{
    .name = "imuTask",

    .stack_size = 256 * 4,

    .priority =
        (osPriority_t) osPriorityAboveNormal
};


/* -------------------------------------------------------------------------- */
/* CONTROL TASK CONFIGURATION                                                 */
/*                                                                            */
/* This task uses the newest IMU information to calculate Servo1 and Servo2    */
/* corrections.                                                               */
/* -------------------------------------------------------------------------- */

const osThreadAttr_t controlTask_attributes =
{
    .name = "controlTask",

    .stack_size = 256 * 4,

    .priority =
        (osPriority_t) osPriorityNormal
};


/* -------------------------------------------------------------------------- */
/* FreeRTOS Initialization                                                    */
/*                                                                            */
/* main.c calls this after osKernelInitialize().                               */
/*                                                                            */
/* This creates both RTOS tasks.                                              */
/* -------------------------------------------------------------------------- */

void MX_FREERTOS_Init(void)
{
    /*
     * Create IMU task.
     */
    imuTaskHandle =
        osThreadNew(
            StartImuTask,
            NULL,
            &imuTask_attributes
        );


    /*
     * Create control task.
     */
    controlTaskHandle =
        osThreadNew(
            StartControlTask,
            NULL,
            &controlTask_attributes
        );
}


/* -------------------------------------------------------------------------- */
/* IMU TASK                                                                   */
/*                                                                            */
/* Frequency = 200 Hz                                                         */
/* Period    = 5 ms                                                           */
/*                                                                            */
/* Job:                                                                       */
/*   1. Read MPU6050 #1                                                       */
/*   2. Read MPU6050 #2                                                       */
/*   3. Update pitch and roll complementary filters                           */
/*                                                                            */
/* vTaskDelayUntil() keeps the timing consistent instead of simply waiting    */
/* 5 ms after each calculation finishes.                                      */
/* -------------------------------------------------------------------------- */

void StartImuTask(void *argument)
{
    /*
     * Remember when this task starts.
     */
    TickType_t lastWakeTime =
        xTaskGetTickCount();


    for (;;)
    {
        /*
         * Read both MPU6050 sensors.
         *
         * The actual function is implemented
         * in main.c.
         */
        App_IMU_TaskStep();


        /*
         * Run this task once every 5 ms.
         *
         * 1000 ms / 5 ms = 200 Hz
         */
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(5)
        );
    }
}


/* -------------------------------------------------------------------------- */
/* CONTROL TASK                                                               */
/*                                                                            */
/* Frequency = 100 Hz                                                         */
/* Period    = 10 ms                                                          */
/*                                                                            */
/* Job:                                                                       */
/*   1. Read the latest filtered angles                                       */
/*   2. Calculate Servo1 correction                                           */
/*   3. Calculate Servo2 correction                                           */
/*   4. Update PWM outputs                                                    */
/* -------------------------------------------------------------------------- */

void StartControlTask(void *argument)
{
    /*
     * Remember when this task starts.
     */
    TickType_t lastWakeTime =
        xTaskGetTickCount();


    for (;;)
    {
        /*
         * Update both stabilization axes.
         *
         * The actual control function is
         * implemented in main.c.
         */
        App_Control_TaskStep();


        /*
         * Run this task once every 10 ms.
         *
         * 1000 ms / 10 ms = 100 Hz
         */
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(10)
        );
    }
}
