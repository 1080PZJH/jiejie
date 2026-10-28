/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"
#include "dmmotor.h"
#include "ma600_task.h"

#include "bsp_log.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
osThreadId ma600TaskHandle;



void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);
void StartMA600TASK(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    // // 后续修改为读取传感器数据准备好的中断处理,

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(ma600task, StartMA600TASK, osPriorityNormal, 0, 1024);
    ma600TaskHandle = osThreadCreate(osThread(ma600task), NULL);

    // osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    // uiTaskHandle = osThreadCreate(osThread(uitask), NULL);

    //HTMotorControlInit(); // 没有注册HT电机则不会执行
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    uint64_t ins_start;
    uint32_t ins_dt_us;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;)
    {
        // 1kHz
        ins_start = DWT_GetTimeline_us();
        INS_Task();
        ins_dt_us = (uint32_t)(DWT_GetTimeline_us() - ins_start);
        if (ins_dt_us > 1000)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%u]us", ins_dt_us);
        VisionSend(); // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMA600TASK(void const *argument)
{
    uint64_t ma600_start;
    uint32_t ma600_dt_us;
    TickType_t last_wakeup;
    LOGINFO("[freeRTOS] MA600 Task Start");
    last_wakeup = xTaskGetTickCount();
    for (;;)
    {
        // 500Hz(2ms),与底盘电机控制周期一致
        // 必须用vTaskDelayUntil: osDelay(2)是从"调用时刻"再等2个tick,
        // 会把MA600Task自身的耗时叠加进周期,实际变成 2ms+耗时,且随负载抖动
        // dt用us整数统计: DWT_GetTimeline_ms返回float,开机2.8小时后ULP已达1ms,
        // 差值会退化成0/1/2的量化值,报警阈值失效;%f本框架也不支持(见bsp_log.h)
        ma600_start = DWT_GetTimeline_us();
        MA600Task();
        ma600_dt_us = (uint32_t)(DWT_GetTimeline_us() - ma600_start);
        if (ma600_dt_us > 1500) // 4路阻塞读约780us,留出到2ms周期的余量
            LOGERROR("[freeRTOS] MA600 Task is being DELAY! dt = [%u]us", ma600_dt_us);
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(2));
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    uint64_t motor_start;
    uint32_t motor_dt_us;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        motor_start = DWT_GetTimeline_us();
        MotorControlTask();
        motor_dt_us = (uint32_t)(DWT_GetTimeline_us() - motor_start);
        if (motor_dt_us > 1000)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%u]us", motor_dt_us);
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    uint64_t daemon_start;
    uint32_t daemon_dt_us;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_us();
        DaemonTask();
        BuzzerTask();
        daemon_dt_us = (uint32_t)(DWT_GetTimeline_us() - daemon_start);
        if (daemon_dt_us > 10000)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%u]us", daemon_dt_us);
        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    uint64_t robot_start;
    uint32_t robot_dt_us;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        robot_start = DWT_GetTimeline_us();
        RobotTask();
        robot_dt_us = (uint32_t)(DWT_GetTimeline_us() - robot_start);
        if (robot_dt_us > 5000)
            LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%u]us", robot_dt_us);
        osDelay(5);
    }
}

// __attribute__((noreturn)) void StartUITASK(void const *argument)
// {
//     LOGINFO("[freeRTOS] UI Task Start");
//     MyUIInit();
//     LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
//     for (;;)
//     {
//         // 每给裁判系统发送一包数据会挂起一次,详见UITask函数的refereeSend()
//         UITask();
//         osDelay(1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
//     }
// }
