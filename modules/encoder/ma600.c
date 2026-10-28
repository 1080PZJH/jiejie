#include "ma600.h"
#include "memory.h"
#include "stdlib.h"

/**
 * @brief 解析rx_buffer中的角度数据并更新实例
 * @note  MA600流式协议: CS拉低后第一个16bit(即rx_buffer[0]:[1], 大端序)就是角度,
 *        同一CS内若继续读取,第二个16bit是多圈/速度计数而非角度
 */
static void MA600Decode(MA600Instance *ma600)
{
    ma600->raw_angle = ((uint16_t)ma600->rx_buffer[0] << 8) | ma600->rx_buffer[1];
    ma600->angle_rad = (float)ma600->raw_angle / 65536.f * PI2 - ma600->zero_offset_rad;
    // 限幅到0~2π
    while (ma600->angle_rad < 0.f)
        ma600->angle_rad += PI2;
    while (ma600->angle_rad >= PI2)
        ma600->angle_rad -= PI2;
    ma600->angle_deg = ma600->angle_rad * 57.29578f;
}

/**
 * @brief 读取完成钩子的空实现,上层可用同名函数覆盖(见ma600_task.c的链式读取)
 * @note  这里必须保留__weak,否则上层不覆盖时链接会报未定义
 */
__weak void MA600SampleDone(MA600Instance *ma600)
{
    (void)ma600;
}

/**
 * @brief SPI传输完成回调,仅工作在IT/DMA模式下被bsp层调用,通过id找回实例并解析数据
 */
static void MA600SPICallback(SPIInstance *spi_ins)
{
    MA600Instance *ma600 = (MA600Instance *)spi_ins->id;
    MA600Decode(ma600);
    MA600SampleDone(ma600); // 链式读取:在此发起下一路(空实现时什么也不做)
}

MA600Instance *MA600Register(MA600_Init_Config_s *config)
{
    MA600Instance *ma600 = (MA600Instance *)malloc(sizeof(MA600Instance));
    memset(ma600, 0, sizeof(MA600Instance));

    ma600->zero_offset_rad = config->zero_offset_rad;

    // MA600流式协议读取角度无需发寄存器地址, 发送内容不影响MISO输出, 填0即可
    ma600->tx_buffer[0] = 0x00u;
    ma600->tx_buffer[1] = 0x00u;

    // 填充SPI配置并注册,id指向ma600实例,传输完成回调中通过id找回实例(父指针)
    SPI_Init_Config_s spi_config = {
        .spi_handle = config->spi_handle,
        .GPIOx = config->cs_gpiox,
        .cs_pin = config->cs_pin,
        .spi_work_mode = config->spi_work_mode,
        .callback = MA600SPICallback,
        .id = ma600,
    };
    ma600->spi_ins = SPIRegister(&spi_config);

    return ma600;
}

void MA600Update(MA600Instance *ma600)
{
    // CS拉低期间MISO流式移出角度, 2字节收发即可; 阻塞模式传输完成后立即解析
    SPITransRecv(ma600->spi_ins, ma600->rx_buffer, ma600->tx_buffer, MA600_SPI_BUFF_SIZE);
    // 阻塞模式不会触发回调,传输完成后直接解析; IT/DMA模式在回调中解析
    if (ma600->spi_ins->spi_work_mode == SPI_BLOCK_MODE)
        MA600Decode(ma600);
}