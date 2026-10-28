#include "ma600_task.h"
#include "bsp_log.h"
#include "spi.h"
#include "gpio.h"

/*
 *  ★ 新增一路磁编需要同时做四件事:
 *      1. CubeMX 里多配一根 GPIO_Output,并设置 User Label
 *      2. 在下面数组里照抄一行,只改 cs_gpiox / cs_pin(引用生成的宏)
 *      3. 把 ma600_task.h 中的 MA600_CNT 改成实际数量
 *      4. 确认 bsp_spi.h 的 MX_SPI_BUS_SLAVE_CNT 足够大(BMI088已占2个名额)
*/


static MA600_Init_Config_s ma600_config[MA600_CNT] = {
    {
        .spi_handle = &hspi2,             // 磁编挂在SPI2
        .cs_gpiox = MA600_CS1_GPIO_Port,  // CS引脚端口,由CubeMX的User Label生成
        .cs_pin = MA600_CS1_Pin,          // CS引脚号,由CubeMX的User Label生成
        .spi_work_mode = SPI_BLOCK_MODE,  // 阻塞模式最简单,也不会和IMU抢总线
        .zero_offset_rad = 0.0f,          // 机械零位偏置,单位rad
    },
    {
        .spi_handle = &hspi2,
        .cs_gpiox = MA600_CS2_GPIO_Port,
        .cs_pin = MA600_CS2_Pin,
        .spi_work_mode = SPI_BLOCK_MODE,
        .zero_offset_rad = 0.0f,
    },
    {
        .spi_handle = &hspi2,
        .cs_gpiox = MA600_CS3_GPIO_Port,
        .cs_pin = MA600_CS3_Pin,
        .spi_work_mode = SPI_BLOCK_MODE,
        .zero_offset_rad = 0.0f,
    },
    {
        .spi_handle = &hspi2,
        .cs_gpiox = MA600_CS4_GPIO_Port,
        .cs_pin = MA600_CS4_Pin,
        .spi_work_mode = SPI_BLOCK_MODE,
        .zero_offset_rad = 0.0f,
    }
};

static MA600Instance *ma600[MA600_CNT] = {NULL};



void MA600TaskInit(void)
{
    for (uint8_t i = 0; i < MA600_CNT; i++)
    {
        ma600[i] = MA600Register(&ma600_config[i]);
    }
    LOGINFO("[ma600] %d encoder(s) registered on SPI2", MA600_CNT);
}

void MA600Task(void)
{
    for (uint8_t i = 0; i < MA600_CNT; i++)
    {
        if (ma600[i] != NULL)
            MA600Update(ma600[i]);
    }
}

MA600Instance *MA600GetInstance(uint8_t idx)
{
    if (idx >= MA600_CNT)
        return NULL;
    return ma600[idx];
}

float MA600GetAngleRad(uint8_t idx)
{
    if (idx >= MA600_CNT || ma600[idx] == NULL)
        return 0.0f;
    return ma600[idx]->angle_rad;
}

float MA600GetAngleDeg(uint8_t idx)
{
    if (idx >= MA600_CNT || ma600[idx] == NULL)
        return 0.0f;
    return ma600[idx]->angle_deg;
}

void MA600SetZeroOffset(uint8_t idx, float offset_rad)
{
    if (idx >= MA600_CNT || ma600[idx] == NULL)
        return;
    ma600[idx]->zero_offset_rad = offset_rad;
}
