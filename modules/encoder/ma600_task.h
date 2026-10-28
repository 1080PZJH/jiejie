/**
 * @file    ma600_task.h
 * @brief   MA600 磁编码器读取任务
 * @note    驱动层(ma600.c)只负责"注册实例 + 读一次";本文件负责按固定周期把所有
 *          已注册的磁编刷一遍,并对外提供角度查询接口,其他应用(chassis等)只需
 *          include 本文件即可拿到最新角度,不必关心SPI细节。
 */

#ifndef MA600_TASK_H
#define MA600_TASK_H

#include "ma600.h"

/**
 * @brief 挂载的磁编数量,必须与 ma600_task.c 中 ma600_config[] 的行数严格一致
 * @note  修改此值后需要把 Makefile 无需改动,但配置表必须同步增删
 */
#define MA600_CNT 4

/**
 * @brief 注册所有磁编实例,需在实时系统启动后、使用角度数据之前调用
 * @note  只做注册(配置SPI实例+CS引脚),不发起传输,可以在RobotInit中安全调用
 */
void MA600TaskInit(void);

/**
 * @brief 读取全部磁编一次,由磁编任务按500Hz(2ms)周期调用
 */
void MA600Task(void);

/**
 * @brief 获取磁编实例指针,调试或需要访问原始值时使用
 * @param idx 磁编编号,0 ~ MA600_CNT-1
 * @return 实例指针,越界或未注册返回NULL
 */
MA600Instance *MA600GetInstance(uint8_t idx);

/**
 * @brief 获取某路磁编的单圈角度
 * @param idx 磁编编号,0 ~ MA600_CNT-1
 * @return 角度值,范围0~2π(已减去零位偏置),越界返回0
 */
float MA600GetAngleRad(uint8_t idx);

/**
 * @brief 获取某路磁编的单圈角度(单位度)
 * @param idx 磁编编号,0 ~ MA600_CNT-1
 * @return 角度值,范围0~360°,越界返回0
 */
float MA600GetAngleDeg(uint8_t idx);

/**
 * @brief 在线设置某路磁编的零位偏置(机械零位标定用)
 * @param idx 磁编编号,0 ~ MA600_CNT-1
 * @note  传入"当前这个位置希望的读数",例如当前机械零位读数为1.2rad而希望它是0,
 *        就传1.2;下次更新后该位置读数即为0
 */
void MA600SetZeroOffset(uint8_t idx, float offset_rad);

#endif // !MA600_TASK_H
