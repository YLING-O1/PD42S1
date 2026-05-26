//
// Created by 34285 on 2026/5/20.
//

#ifndef PD42S1_PD42S1_H
#define PD42S1_PD42S1_H

#include "stm32f4xx_hal.h"

#define PI 3.14159265358979323846

#define RADPERS_To_RPM (60.0f / (2 * PI))
#define RAD_To_Value (51200.0f / (2 * PI))

#define RS485_DE_GPIO_PORT GPIOB
#define RS485_DE_GPIO_PIN  GPIO_PIN_12

#define RS485_TX_ENABLE() HAL_GPIO_WritePin(RS485_DE_GPIO_PORT, RS485_DE_GPIO_PIN, GPIO_PIN_SET)
#define RS485_RX_ENABLE() HAL_GPIO_WritePin(RS485_DE_GPIO_PORT, RS485_DE_GPIO_PIN, GPIO_PIN_RESET)

//RS485接收回调函数数据类型
typedef void (*RS485_Call_Back)(uint8_t *PD42_Motor_Rx_Buffer);

//RS485处理接受到的信息的结构体
typedef struct
{
    UART_HandleTypeDef *UART_Handler;
    RS485_Call_Back CallBack_Function;
}Struct_RS485_Manage_Object;

unsigned int floatToUint(float value);

void RS485_Init(UART_HandleTypeDef *huart, RS485_Call_Back CallBack_Function);
void RS485_Send_Data_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

void Position_PID_Packet(uint8_t addr, uint32_t KP, uint32_t KI, uint32_t KD);
void Speed_PID_Packet(uint8_t addr, uint32_t KP, uint32_t KI, uint32_t KD);
void Absolute_Position_Mode_Packet(uint8_t addr, uint8_t rotation_dir, uint8_t acceleration,
                            float Omega, float Angle);
void Reset_Current_Position_Packet(uint8_t addr);
void Motor_Enable_Packet(uint8_t addr, uint8_t Cmd);
void Cleared_Condition_Packet(uint8_t addr);

void PD42S1_Call_Back(uint8_t *PD42_Motor_Rx_Buffer);
void Set_Position_PID(UART_HandleTypeDef *huart, uint8_t addr, uint32_t KP, uint32_t KI, uint32_t KD);
void Set_Speed_PID(UART_HandleTypeDef *huart, uint8_t addr, uint32_t KP, uint32_t KI, uint32_t KD);
void PD42S1_Absolute_Position_Mode(UART_HandleTypeDef *huart, uint8_t addr, uint8_t rotation_dir,
                                   uint8_t acceleration, float Omega, float Angle);
void Reset_Current_Position(UART_HandleTypeDef *huart, uint8_t addr);
void Motor_Enable(UART_HandleTypeDef *huart, uint8_t addr, uint8_t Cmd);
void Cleared_Condition(UART_HandleTypeDef *huart, uint8_t addr);

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

extern Struct_RS485_Manage_Object USART1_RS485_Manage_Object;
extern Struct_RS485_Manage_Object USART6_RS485_Manage_Object;

#endif //PD42S1_PD42S1_H
