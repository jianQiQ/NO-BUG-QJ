#include "main.h"
#include "i2c.h"

#define I2C_ADDRESS (0x20 << 1)  // 7位地址左移1位

uint8_t rxData = 0;
uint8_t txData = 0;

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // 主机要发送数据给我们，准备接收
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rxData, 1, I2C_FIRST_AND_LAST_FRAME);
    } else {
        // 主机要从我们这里读数据，准备发送
        txData = 0;  // 发送数据0
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &txData, 1, I2C_FIRST_AND_LAST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // 接收完成，处理接收到的数据
    if (rxData == 1) {
        // 收到数据1，可以做相应处理
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // 例如切换LED
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // 发送完成
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    // 错误处理
    HAL_I2C_Slave_Listen_IT(hi2c);  // 重新开始监听
}

// 在main函数中调用
void I2C_Slave_Init(void)
{
    HAL_I2C_Slave_Listen_IT(&hi2c1);  // 开始监听I2C通信
}