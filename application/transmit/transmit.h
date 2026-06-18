#ifndef __TRANSMIT_H__
#define __TRANSMIT_H__

/**
 * @brief 初始化传输任务,会被RobotInit()调用
 * 
 */
void TransmitInit(void);

/**
 * @brief 传输任务
 * 
 */
void TransmitTask(void);

#endif