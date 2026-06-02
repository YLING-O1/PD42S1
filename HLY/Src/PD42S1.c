//
// Created by 34285 on 2026/5/21.
//

#include "PD42S1.h"

//电机旋转方向：0：正转；1：反转
uint8_t Rotation_Dir = 0;
//加减速度，取值范围0~200，数值越大加减速度越大（注意：0表示直接启动）
uint8_t Acceleration = 0;
//速度，取值范围0~6000 RPM（类型uint16_t）
uint16_t Speed = 0;
//绝对位置（注意51200为一圈）（类型uint32_t）
uint32_t Absolute_Position = 0;

uint8_t Command_Result_Motor_0 = 0;
uint8_t Command_Result_Motor_1 = 0;
uint8_t Command_Result_Motor_2 = 0;
uint8_t Command_Result_Motor_3 = 0;

uint8_t PD42_Motor_Rx_Buffer[20];
uint8_t PD42_0x01_Rx_Data[20];
uint8_t PD42_0x02_Rx_Data[20];
uint8_t PD42_0x03_Rx_Data[20];
uint8_t PD42_0x04_Rx_Data[20];

Struct_RS485_Manage_Object USART1_RS485_Manage_Object;
Struct_RS485_Manage_Object USART6_RS485_Manage_Object;

unsigned int floatToUint(float value)
{
    // 如果是负数，根据业务需求通常置为 0，防止转换为无符号整型时发生下溢(Wrap-around)
    if (value <= 0.0f)
    {
        return 0;
    }
    // 强制类型转换会自动丢弃小数部分（向零取整）
    return (unsigned int)value;
}

//初始化RS485总线
void RS485_Init(UART_HandleTypeDef *huart, RS485_Call_Back CallBack_Function)
{
    RS485_RX_ENABLE();
    HAL_UARTEx_ReceiveToIdle_DMA(huart, PD42_Motor_Rx_Buffer, sizeof(PD42_Motor_Rx_Buffer));
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    if (huart->Instance == USART1)
    {
        // __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        USART1_RS485_Manage_Object.UART_Handler = huart;
        USART1_RS485_Manage_Object.CallBack_Function = CallBack_Function;
    }
    else if (huart->Instance == USART6)
    {
        // __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
        USART6_RS485_Manage_Object.UART_Handler = huart;
        USART6_RS485_Manage_Object.CallBack_Function = CallBack_Function;
    }
}

//发送数据至RS485总线
void RS485_Send_Data_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    while (huart->gState != HAL_UART_STATE_READY) {}
    RS485_TX_ENABLE();
    HAL_UART_Transmit_DMA(huart, pData, Size);
}

//设置位置环PID参数指令
uint8_t RS485_Tx_Data_Position_PID[17];
void Position_PID_Packet(uint8_t addr, uint32_t KP, uint32_t KI, uint32_t KD)
{
    RS485_Tx_Data_Position_PID[0] = 0xC5;
    RS485_Tx_Data_Position_PID[1] = addr;
    RS485_Tx_Data_Position_PID[2] = 0x63;
    RS485_Tx_Data_Position_PID[3] = (KP & 0xFF000000) >> 24;
    RS485_Tx_Data_Position_PID[4] = (KP & 0xFF0000) >> 16;
    RS485_Tx_Data_Position_PID[5] = (KP & 0xFF00) >> 8;
    RS485_Tx_Data_Position_PID[6] = KP & 0xFF;
    RS485_Tx_Data_Position_PID[7] = (KI & 0xFF000000) >> 24;
    RS485_Tx_Data_Position_PID[8] = (KI & 0xFF0000) >> 16;
    RS485_Tx_Data_Position_PID[9] = (KI & 0xFF00) >> 8;
    RS485_Tx_Data_Position_PID[10] = KI & 0xFF;
    RS485_Tx_Data_Position_PID[11] = (KD & 0xFF000000) >> 24;
    RS485_Tx_Data_Position_PID[12] = (KD & 0xFF0000) >> 16;
    RS485_Tx_Data_Position_PID[13] = (KD & 0xFF00) >> 8;
    RS485_Tx_Data_Position_PID[14] = KD & 0xFF;
    uint8_t CHECKSUM = 0;
    for(int i = 0; i <= 14; i++)
    {
        CHECKSUM += RS485_Tx_Data_Position_PID[i];
    }
    RS485_Tx_Data_Position_PID[15] = CHECKSUM;
    RS485_Tx_Data_Position_PID[16] = 0x5C;
}

//设置速度环PID参数指令
uint8_t RS485_Tx_Data_Speed_PID[17];
void Speed_PID_Packet(uint8_t addr, uint32_t KP, uint32_t KI, uint32_t KD)
{
    RS485_Tx_Data_Speed_PID[0] = 0xC5;
    RS485_Tx_Data_Speed_PID[1] = addr;
    RS485_Tx_Data_Speed_PID[2] = 0x73;
    RS485_Tx_Data_Speed_PID[3] = (KP & 0xFF000000) >> 24;
    RS485_Tx_Data_Speed_PID[4] = (KP & 0xFF0000) >> 16;
    RS485_Tx_Data_Speed_PID[5] = (KP & 0xFF00) >> 8;
    RS485_Tx_Data_Speed_PID[6] = KP & 0xFF;
    RS485_Tx_Data_Speed_PID[7] = (KI & 0xFF000000) >> 24;
    RS485_Tx_Data_Speed_PID[8] = (KI & 0xFF0000) >> 16;
    RS485_Tx_Data_Speed_PID[9] = (KI & 0xFF00) >> 8;
    RS485_Tx_Data_Speed_PID[10] = KI & 0xFF;
    RS485_Tx_Data_Speed_PID[11] = (KD & 0xFF000000) >> 24;
    RS485_Tx_Data_Speed_PID[12] = (KD & 0xFF0000) >> 16;
    RS485_Tx_Data_Speed_PID[13] = (KD & 0xFF00) >> 8;
    RS485_Tx_Data_Speed_PID[14] = KD & 0xFF;
    uint8_t CHECKSUM = 0;
    for(int i = 0; i <= 14; i++)
    {
        CHECKSUM += RS485_Tx_Data_Speed_PID[i];
    }
    RS485_Tx_Data_Speed_PID[15] = CHECKSUM;
    RS485_Tx_Data_Speed_PID[16] = 0x5C;
}

//绝对位置模式控制指令
uint8_t RS485_Tx_Data_Absolute_Position_Mode[13];
void Absolute_Position_Mode_Packet(uint8_t addr, uint8_t rotation_dir, uint8_t acceleration,
                                   float Omega, float Angle)
{
    RS485_Tx_Data_Absolute_Position_Mode[0] = 0xC5;
    RS485_Tx_Data_Absolute_Position_Mode[1] = addr;
    RS485_Tx_Data_Absolute_Position_Mode[2] = 0xF2;

    uint16_t speed = floatToUint(Omega * RADPERS_To_RPM);
    uint32_t absolute_position = floatToUint(Angle * RAD_To_Value);
    RS485_Tx_Data_Absolute_Position_Mode[3] = rotation_dir;
    RS485_Tx_Data_Absolute_Position_Mode[4] = acceleration;
    RS485_Tx_Data_Absolute_Position_Mode[5] = (speed & 0xFF00) >> 8;
    RS485_Tx_Data_Absolute_Position_Mode[6] = speed & 0xFF;
    RS485_Tx_Data_Absolute_Position_Mode[7] = (absolute_position & 0xFF000000) >> 24;
    RS485_Tx_Data_Absolute_Position_Mode[8] = (absolute_position & 0xFF0000) >> 16;
    RS485_Tx_Data_Absolute_Position_Mode[9] = (absolute_position & 0xFF00) >> 8;
    RS485_Tx_Data_Absolute_Position_Mode[10] = absolute_position & 0xFF;
    uint8_t CHECKSUM = 0;
    for(int i = 0; i <= 10; i++)
    {
        CHECKSUM += RS485_Tx_Data_Absolute_Position_Mode[i];
    }
    RS485_Tx_Data_Absolute_Position_Mode[11] = CHECKSUM;
    RS485_Tx_Data_Absolute_Position_Mode[12] = 0x5C;
}

//将当前的位置角度清零指令
uint8_t RS485_Tx_Data_Reset_Current_Position[5];
void Reset_Current_Position_Packet(uint8_t addr)
{
    RS485_Tx_Data_Reset_Current_Position[0] = 0xC5;
    RS485_Tx_Data_Reset_Current_Position[1] = addr;
    RS485_Tx_Data_Reset_Current_Position[2] = 0xF8;
    RS485_Tx_Data_Reset_Current_Position[3] = 0xBE;
    RS485_Tx_Data_Reset_Current_Position[4] = 0x5C;
}

//电机使能控制指令,0：使能电机；1：失能电机
uint8_t RS485_Tx_Data_Motor_Enable[6];
void Motor_Enable_Packet(uint8_t addr, uint8_t Cmd)
{
    RS485_Tx_Data_Motor_Enable[0] = 0xC5;
    RS485_Tx_Data_Motor_Enable[1] = addr;
    RS485_Tx_Data_Motor_Enable[2] = 0xFA;
    RS485_Tx_Data_Motor_Enable[3] = Cmd;
    uint8_t CHECKSUM = 0;
    for(int i = 0; i <= 3; i++)
    {
        CHECKSUM += RS485_Tx_Data_Motor_Enable[i];
    }
    RS485_Tx_Data_Motor_Enable[4] = CHECKSUM;
    RS485_Tx_Data_Motor_Enable[5] = 0x5C;
}

//清除状态指令
uint8_t RS485_Tx_Data_Cleared_Condition[5];
void Cleared_Condition_Packet(uint8_t addr)
{
    RS485_Tx_Data_Cleared_Condition[0] = 0xC5;
    RS485_Tx_Data_Cleared_Condition[1] = addr;
    RS485_Tx_Data_Cleared_Condition[2] = 0xFB;
    RS485_Tx_Data_Cleared_Condition[3] = 0xC1;
    RS485_Tx_Data_Cleared_Condition[4] = 0x5C;
}

//PD42S1接收回调处理函数
void PD42S1_Call_Back(uint8_t *PD42_Motor_Rx_Buffer)
{
    if (PD42_Motor_Rx_Buffer[0] == 0xC5)
    {
        switch (PD42_Motor_Rx_Buffer[1])
        {
        case (0x01):
            {
                Command_Result_Motor_0 = PD42_Motor_Rx_Buffer[3];
                for (int i = 0; i < 20; i++)
                {
                    PD42_0x01_Rx_Data[i] = PD42_Motor_Rx_Buffer[i];
                }
            }
        break;
        case (0x02):
            {
                Command_Result_Motor_1 = PD42_Motor_Rx_Buffer[3];
                for (int i = 0; i < 20; i++)
                {
                    PD42_0x02_Rx_Data[i] = PD42_Motor_Rx_Buffer[i];
                }
            }
        break;
        case (0x03):
            {
                Command_Result_Motor_2 = PD42_Motor_Rx_Buffer[3];
                for (int i = 0; i < 20; i++)
                {
                    PD42_0x03_Rx_Data[i] = PD42_Motor_Rx_Buffer[i];
                }
            }
        break;
        case (0x04):
            {
                Command_Result_Motor_3 = PD42_Motor_Rx_Buffer[3];
                for (int i = 0; i < 20; i++)
                {
                    PD42_0x04_Rx_Data[i] = PD42_Motor_Rx_Buffer[i];
                }
            }
        break;
        default: ;
        }
    }
}

//RS485的发送回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1 || huart->Instance == USART6)
    {
        while(__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET);
        RS485_RX_ENABLE();
    }
}

//RS485的接收回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    RS485_RX_ENABLE();
    if (huart->Instance == USART1)
    {
        if (USART1_RS485_Manage_Object.CallBack_Function != NULL)
        {
            USART1_RS485_Manage_Object.CallBack_Function(PD42_Motor_Rx_Buffer);
        }
    }
    else if (huart->Instance == USART6)
    {
        if (USART6_RS485_Manage_Object.CallBack_Function != NULL)
        {
            USART6_RS485_Manage_Object.CallBack_Function(PD42_Motor_Rx_Buffer);
        }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, PD42_Motor_Rx_Buffer, sizeof(PD42_Motor_Rx_Buffer));
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

//设置位置环PID参数
void Set_Position_PID(UART_HandleTypeDef *huart, uint8_t addr, uint32_t KP, uint32_t KI, uint32_t KD)
{
    Position_PID_Packet(addr, KP, KI, KD);
    RS485_Send_Data_DMA(huart, RS485_Tx_Data_Position_PID, 17);
}

//设置速度环PID参数
void Set_Speed_PID(UART_HandleTypeDef *huart, uint8_t addr, uint32_t KP, uint32_t KI, uint32_t KD)
{
    Speed_PID_Packet(addr, KP, KI, KD);
    RS485_Send_Data_DMA(huart, RS485_Tx_Data_Speed_PID, 17);
}

/**
 * @brief 绝对位置模式控制函数
 *
 * @param huart 串口句柄
 * @param addr 舵机地址
 * @param rotation_dir 旋转方向，0：正转；1：反转
 * @param acceleration 加减速度，取值范围0~200，数值越大加减速度越大（注意：0表示直接启动）
 * @param Omega 速度，取值范围0~6000 RPM（类型uint16_t）
 * @param Angle 绝对位置（注意51200为一圈）（类型uint32_t）
 */
void PD42S1_Absolute_Position_Mode(UART_HandleTypeDef *huart, uint8_t addr, uint8_t rotation_dir,
                                   uint8_t acceleration, float Omega, float Angle)
{
    Absolute_Position_Mode_Packet(addr, rotation_dir, acceleration, Omega, Angle);
    RS485_Send_Data_DMA(huart, RS485_Tx_Data_Absolute_Position_Mode, 13);
}

//将当前的位置角度清零
void Reset_Current_Position(UART_HandleTypeDef *huart, uint8_t addr)
{
    Reset_Current_Position_Packet(addr);
    RS485_Send_Data_DMA(huart, RS485_Tx_Data_Reset_Current_Position, 5);
}

//使能电机,0：使能电机；1：失能电机
void Motor_Enable(UART_HandleTypeDef *huart, uint8_t addr, uint8_t Cmd)
{
    Motor_Enable_Packet(addr, Cmd);
    RS485_Send_Data_DMA(huart, RS485_Tx_Data_Motor_Enable, 6);
}

//清除状态
void Cleared_Condition(UART_HandleTypeDef *huart, uint8_t addr)
{
    Cleared_Condition_Packet(addr);
    RS485_Send_Data_DMA(huart, RS485_Tx_Data_Cleared_Condition, 5);
}
