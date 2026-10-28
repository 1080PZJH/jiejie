/**
 * @file    ma600.h
 * @brief   MA600 (MPS MagAlpha, Gen6) 16bit TMR磁编码器SPI驱动
 * @note    MA600的SPI是流式协议: CS拉低后, 每16个时钟MISO直接移出一个16bit角度值
 *          (0~65535对应0~360°), 无需先发寄存器地址; 同一CS内连续第二个16bit是
 *          多圈/速度计数(寄存器0x0001), 不要误当成角度解析(官方Gen6库同此协议,
 *          SPI支持mode1/mode3, 此处用mode3: CPOL=1,CPHA=1)
 */

#ifndef MA600_H
#define MA600_H

#include "bsp_spi.h"
#include "general_def.h"

#define MA600_SPI_BUFF_SIZE 2u  // CS拉低后第一个16bit即角度, 2字节即可完成一次读取

typedef struct ma600_instance
{
    SPIInstance *spi_ins;                   // 底层SPI实例
    uint16_t raw_angle;                     // 原始角度值, 0~65535
    float angle_rad;                        // 单圈角度, 已减去零位偏置, 范围0~2π
    float angle_deg;                        // 单圈角度, 范围0~360°
    float zero_offset_rad;                  // 零位偏置, 单位rad
    uint8_t tx_buffer[MA600_SPI_BUFF_SIZE]; // 发送缓冲: 2字节寄存器地址 + 2字节dummy
    uint8_t rx_buffer[MA600_SPI_BUFF_SIZE]; // 接收缓冲: 前2字节为命令阶段无效数据, 后2字节为角度
} MA600Instance;

typedef struct
{
    SPI_HandleTypeDef *spi_handle; // 使用的SPI总线, 如&hspi1
    GPIO_TypeDef *cs_gpiox;        // 片选信号GPIO组, 如GPIOA
    uint16_t cs_pin;               // 片选信号引脚, 如GPIO_PIN_4
    SPI_TXRX_MODE_e spi_work_mode; // SPI工作模式(阻塞/中断/DMA)
    float zero_offset_rad;         // 零位偏置, 单位rad, 读取结果会自动减去该值
} MA600_Init_Config_s;

/**
 * @brief 注册一个MA600编码器实例
 *
 * @param config MA600初始化配置
 * @return MA600Instance* MA600实例指针, 之后通过该指针读取角度
 */
MA600Instance *MA600Register(MA600_Init_Config_s *config);

/**
 * @brief 发起一次角度读取并更新实例中的角度数据
 * @note  阻塞模式: 传输完成后立即解析; IT/DMA模式: 传输完成后在回调中解析,
 *        两种模式下调用此函数后直接读取angle_rad/angle_deg即可
 *
 * @param ma600 MA600实例指针
 */
void MA600Update(MA600Instance *ma600);

/**
 * @brief 一路角度读取完成后的钩子(仅 IT/DMA 模式会被调用)
 * @note  ma600.c 里给了一个空实现(__weak),上层可以在自己的源文件里重写它,
 *        用来做"读完一路立刻发起下一路"的链式读取。阻塞模式不会调用。
 *
 * @param ma600 刚刚完成读取的实例指针
 */
void MA600SampleDone(MA600Instance *ma600);

#endif // !MA600_H