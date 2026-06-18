#include "transmit.h"
#include "usart.h"
#include "ins_task.h"
#include <stdio.h>

static UART_HandleTypeDef *transmit_uart = NULL;
static attitude_t *ins_data = NULL;

void TransmitInit(void)
{
    transmit_uart = &huart6;
    ins_data = INS_Init();
}

void TransmitTask(void)
{
    if (ins_data == NULL || transmit_uart == NULL) return;

    static char send_buffer[200];
    int len = sprintf(send_buffer,
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
        (double)ins_data->Gyro[0],
        (double)ins_data->Gyro[1],
        (double)ins_data->Gyro[2],
        (double)ins_data->Accel[0],
        (double)ins_data->Accel[1],
        (double)ins_data->Accel[2],
        (double)ins_data->Roll,
        (double)ins_data->Pitch,
        (double)ins_data->Yaw);

    if (transmit_uart->gState == HAL_UART_STATE_READY)
    {
        HAL_UART_Transmit_DMA(transmit_uart, (uint8_t*)send_buffer, len);
    }
}