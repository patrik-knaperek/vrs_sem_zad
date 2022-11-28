
/**
  ******************************************************************************
  * @file    CAN2022.c
  * @author  SGT Generated by tool
  * @version V0.1.0 (generator)
  * @date    8-September-2022
  * @brief   CAN protocol application/PROFILE layer for use in SGT formula student electric 2022
  ******************************************************************************
  */
#include "stm32f1xx_hal.h"
#include "CAN2022.h"
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;


#if defined(Tx_MCU_IMU_angular_velocity) || defined(Rx_MCU_IMU_angular_velocity)
    MCU_IMU_angular_velocity_TypeDef MCU_IMU_angular_velocity_Data = {
        .gyrX = 0,
        .gyrY = 0,
        .gyrZ = 0
     };
#endif
    
#ifdef Tx_MCU_IMU_angular_velocity
void Tx_MCU_IMU_angular_velocity_Data(CAN_HandleTypeDef* hcan, MCU_IMU_angular_velocity_TypeDef* MCU_IMU_angular_velocity_Data)
{
    //hcan->pTxMsg = &CanTxMsg;
    TxHeader.StdId = ID_MCU_IMU_angular_velocity;
    TxHeader.DLC = DLC_MCU_IMU_angular_velocity;
    TxHeader.ExtId = 0x0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
        
    TxData[0] = (uint8_t)((MCU_IMU_angular_velocity_Data->gyrX >> 12)&0b11111111);
    TxData[1] = (uint8_t)((MCU_IMU_angular_velocity_Data->gyrX >> 4)&0b1111111111111111);
    TxData[2] = (uint8_t)((MCU_IMU_angular_velocity_Data->gyrX << 4)&0b111111111111111111111111);
    TxData[2] |= (uint8_t)((MCU_IMU_angular_velocity_Data->gyrY >> 16)&0b1111);
    TxData[3] = (uint8_t)((MCU_IMU_angular_velocity_Data->gyrY >> 8)&0b111111111111);
    TxData[4] = (uint8_t)((MCU_IMU_angular_velocity_Data->gyrY)&0b11111111111111111111);
    TxData[5] = (uint8_t)((MCU_IMU_angular_velocity_Data->gyrZ >> 12)&0b11111111);
    TxData[6] = (uint8_t)((MCU_IMU_angular_velocity_Data->gyrZ >> 4)&0b1111111111111111);
    TxData[7] = (uint8_t)((MCU_IMU_angular_velocity_Data->gyrZ << 4)&0b111111111111111111111111);
                        
    HAL_CAN_ActivateNotification(hcan,0xff);
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
}
#endif
#ifdef Rx_MCU_IMU_angular_velocity
void Rx_MCU_IMU_angular_velocity_Data(CAN_HandleTypeDef* hcan, MCU_IMU_angular_velocity_TypeDef* MCU_IMU_angular_velocity_Data)
{
    MCU_IMU_angular_velocity_Data->gyrX = (int32_t)(((hcan->pRxMsg->Data[0]&0b11111111) << 12) | ((hcan->pRxMsg->Data[1]&0b1111111111111111) << 4) | ((hcan->pRxMsg->Data[2]&0b111111111111111111111111) >> 4));
    MCU_IMU_angular_velocity_Data->gyrY = (int32_t)(((hcan->pRxMsg->Data[2]&0b1111) << 16) | ((hcan->pRxMsg->Data[3]&0b111111111111) << 8) | (hcan->pRxMsg->Data[4]&0b11111111111111111111));
    MCU_IMU_angular_velocity_Data->gyrZ = (int32_t)(((hcan->pRxMsg->Data[5]&0b11111111) << 12) | ((hcan->pRxMsg->Data[6]&0b1111111111111111) << 4) | ((hcan->pRxMsg->Data[7]&0b111111111111111111111111) >> 4));
}
#endif


#if defined(Tx_MCU_IMU_acceleration) || defined(Rx_MCU_IMU_acceleration)
    MCU_IMU_acceleration_TypeDef MCU_IMU_acceleration_Data = {
        .accX = 0,
        .accY = 0,
        .accZ = 0
     };
#endif
    
#ifdef Tx_MCU_IMU_acceleration
void Tx_MCU_IMU_acceleration_Data(CAN_HandleTypeDef* hcan, MCU_IMU_acceleration_TypeDef* MCU_IMU_acceleration_Data)
{
    //hcan->pTxMsg = &CanTxMsg;
    TxHeader.StdId = ID_MCU_IMU_acceleration;
    TxHeader.DLC = DLC_MCU_IMU_acceleration;
    TxHeader.ExtId = 0x0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
        
    TxData[0] = (uint8_t)((MCU_IMU_acceleration_Data->accX >> 12)&0b11111111);
    TxData[1] = (uint8_t)((MCU_IMU_acceleration_Data->accX >> 4)&0b1111111111111111);
    TxData[2] = (uint8_t)((MCU_IMU_acceleration_Data->accX << 4)&0b111111111111111111111111);
    TxData[2] |= (uint8_t)((MCU_IMU_acceleration_Data->accY >> 16)&0b1111);
    TxData[3] = (uint8_t)((MCU_IMU_acceleration_Data->accY >> 8)&0b111111111111);
    TxData[4] = (uint8_t)((MCU_IMU_acceleration_Data->accY)&0b11111111111111111111);
    TxData[5] = (uint8_t)((MCU_IMU_acceleration_Data->accZ >> 12)&0b11111111);
    TxData[6] = (uint8_t)((MCU_IMU_acceleration_Data->accZ >> 4)&0b1111111111111111);
    TxData[7] = (uint8_t)((MCU_IMU_acceleration_Data->accZ << 4)&0b111111111111111111111111);
                        
    HAL_CAN_ActivateNotification(hcan,0xff);
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
}
#endif
#ifdef Rx_MCU_IMU_acceleration
void Rx_MCU_IMU_acceleration_Data(CAN_HandleTypeDef* hcan, MCU_IMU_acceleration_TypeDef* MCU_IMU_acceleration_Data)
{
    MCU_IMU_acceleration_Data->accX = (int32_t)(((hcan->pRxMsg->Data[0]&0b11111111) << 12) | ((hcan->pRxMsg->Data[1]&0b1111111111111111) << 4) | ((hcan->pRxMsg->Data[2]&0b111111111111111111111111) >> 4));
    MCU_IMU_acceleration_Data->accY = (int32_t)(((hcan->pRxMsg->Data[2]&0b1111) << 16) | ((hcan->pRxMsg->Data[3]&0b111111111111) << 8) | (hcan->pRxMsg->Data[4]&0b11111111111111111111));
    MCU_IMU_acceleration_Data->accZ = (int32_t)(((hcan->pRxMsg->Data[5]&0b11111111) << 12) | ((hcan->pRxMsg->Data[6]&0b1111111111111111) << 4) | ((hcan->pRxMsg->Data[7]&0b111111111111111111111111) >> 4));
}
#endif


#if defined(Tx_MCU_IMU_euler_angles) || defined(Rx_MCU_IMU_euler_angles)
    MCU_IMU_euler_angles_TypeDef MCU_IMU_euler_angles_Data = {
        .roll = 0,
        .pitch = 0,
        .yaw = 0
     };
#endif
    
#ifdef Tx_MCU_IMU_euler_angles
void Tx_MCU_IMU_euler_angles_Data(CAN_HandleTypeDef* hcan, MCU_IMU_euler_angles_TypeDef* MCU_IMU_euler_angles_Data)
{
    //hcan->pTxMsg = &CanTxMsg;
    TxHeader.StdId = ID_MCU_IMU_euler_angles;
    TxHeader.DLC = DLC_MCU_IMU_euler_angles;
    TxHeader.ExtId = 0x0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
        
    TxData[0] = (uint8_t)((MCU_IMU_euler_angles_Data->roll >> 12)&0b11111111);
    TxData[1] = (uint8_t)((MCU_IMU_euler_angles_Data->roll >> 4)&0b1111111111111111);
    TxData[2] = (uint8_t)((MCU_IMU_euler_angles_Data->roll << 4)&0b111111111111111111111111);
    TxData[2] |= (uint8_t)((MCU_IMU_euler_angles_Data->pitch >> 16)&0b1111);
    TxData[3] = (uint8_t)((MCU_IMU_euler_angles_Data->pitch >> 8)&0b111111111111);
    TxData[4] = (uint8_t)((MCU_IMU_euler_angles_Data->pitch)&0b11111111111111111111);
    TxData[5] = (uint8_t)((MCU_IMU_euler_angles_Data->yaw >> 12)&0b11111111);
    TxData[6] = (uint8_t)((MCU_IMU_euler_angles_Data->yaw >> 4)&0b1111111111111111);
    TxData[7] = (uint8_t)((MCU_IMU_euler_angles_Data->yaw << 4)&0b111111111111111111111111);
                        
    HAL_CAN_ActivateNotification(hcan,0xff);
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
}
#endif
#ifdef Rx_MCU_IMU_euler_angles
void Rx_MCU_IMU_euler_angles_Data(CAN_HandleTypeDef* hcan, MCU_IMU_euler_angles_TypeDef* MCU_IMU_euler_angles_Data)
{
    MCU_IMU_euler_angles_Data->roll = (int32_t)(((hcan->pRxMsg->Data[0]&0b11111111) << 12) | ((hcan->pRxMsg->Data[1]&0b1111111111111111) << 4) | ((hcan->pRxMsg->Data[2]&0b111111111111111111111111) >> 4));
    MCU_IMU_euler_angles_Data->pitch = (int32_t)(((hcan->pRxMsg->Data[2]&0b1111) << 16) | ((hcan->pRxMsg->Data[3]&0b111111111111) << 8) | (hcan->pRxMsg->Data[4]&0b11111111111111111111));
    MCU_IMU_euler_angles_Data->yaw = (int32_t)(((hcan->pRxMsg->Data[5]&0b11111111) << 12) | ((hcan->pRxMsg->Data[6]&0b1111111111111111) << 4) | ((hcan->pRxMsg->Data[7]&0b111111111111111111111111) >> 4));
}
#endif


#if defined(Tx_MCU_IMU_gps_position) || defined(Rx_MCU_IMU_gps_position)
    MCU_IMU_gps_position_TypeDef MCU_IMU_gps_position_Data = {
        .lat = 0,
        .longitude = 0
     };
#endif

#ifdef Tx_MCU_IMU_gps_position
void Tx_MCU_IMU_gps_position_Data(CAN_HandleTypeDef* hcan, MCU_IMU_gps_position_TypeDef* MCU_IMU_gps_position_Data)
{
    //hcan->pTxMsg = &CanTxMsg;
    TxHeader.StdId = ID_MCU_IMU_gps_position;
    TxHeader.DLC = DLC_MCU_IMU_gps_position;
    TxHeader.ExtId = 0x0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;

	TxData[0] = (uint8_t)((MCU_IMU_gps_position_Data->lat >> 24)&0b11111111);
    TxData[1] = (uint8_t)((MCU_IMU_gps_position_Data->lat >> 16)&0b1111111111111111);
    TxData[2] = (uint8_t)((MCU_IMU_gps_position_Data->lat >> 8)&0b111111111111111111111111);
    TxData[3] = (uint8_t)((MCU_IMU_gps_position_Data->lat)&0b11111111111111111111111111111111);
    TxData[4] = (uint8_t)((MCU_IMU_gps_position_Data->longitude >> 24)&0b11111111);
    TxData[5] = (uint8_t)((MCU_IMU_gps_position_Data->longitude >> 16)&0b1111111111111111);
    TxData[6] = (uint8_t)((MCU_IMU_gps_position_Data->longitude >> 8)&0b111111111111111111111111);
    TxData[7] = (uint8_t)((MCU_IMU_gps_position_Data->longitude)&0b11111111111111111111111111111111);

    HAL_CAN_ActivateNotification(hcan,0xff);
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
}
#endif
#ifdef Rx_MCU_IMU_gps_position
void Rx_MCU_IMU_gps_position_Data(CAN_HandleTypeDef* hcan, MCU_IMU_gps_position_TypeDef* MCU_IMU_gps_position_Data)
{
    MCU_IMU_gps_position_Data->lat = (uint32_t)(((hcan->pRxMsg->Data[0]&0b11111111) << 24) | ((hcan->pRxMsg->Data[1]&0b1111111111111111) << 16) | ((hcan->pRxMsg->Data[2]&0b111111111111111111111111) << 8) | (hcan->pRxMsg->Data[3]&0b11111111111111111111111111111111));
    MCU_IMU_gps_position_Data->longitude = (uint32_t)(((hcan->pRxMsg->Data[4]&0b11111111) << 24) | ((hcan->pRxMsg->Data[5]&0b1111111111111111) << 16) | ((hcan->pRxMsg->Data[6]&0b111111111111111111111111) << 8) | (hcan->pRxMsg->Data[7]&0b11111111111111111111111111111111));
}
#endif


#if defined(Tx_MCU_IMU_gps_speed) || defined(Rx_MCU_IMU_gps_speed)
    MCU_IMU_gps_speed_TypeDef MCU_IMU_gps_speed_Data = {
        .gps_velocity = 0
     };
#endif

#ifdef Tx_MCU_IMU_gps_speed
void Tx_MCU_IMU_gps_speed_Data(CAN_HandleTypeDef* hcan, MCU_IMU_gps_speed_TypeDef* MCU_IMU_gps_speed_Data)
{
    //hcan->pTxMsg = &CanTxMsg;
    TxHeader.StdId = ID_MCU_IMU_gps_speed;
    TxHeader.DLC = DLC_MCU_IMU_gps_speed;
    TxHeader.ExtId = 0x0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;

    TxData[0] = (uint8_t)((MCU_IMU_gps_speed_Data->gps_velocity >> 8)&0b11111111);
    TxData[1] = (uint8_t)((MCU_IMU_gps_speed_Data->gps_velocity)&0b1111111111111111);

    HAL_CAN_ActivateNotification(hcan,0xff);
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
}
#endif
#ifdef Rx_MCU_IMU_gps_speed
void Rx_MCU_IMU_gps_speed_Data(CAN_HandleTypeDef* hcan, MCU_IMU_gps_speed_TypeDef* MCU_IMU_gps_speed_Data)
{
    MCU_IMU_gps_speed_Data->gps_velocity = (uint16_t)(((hcan->pRxMsg->Data[0]&0b11111111) << 8) | (hcan->pRxMsg->Data[1]&0b1111111111111111));
}
#endif


/** \brief Zmaze CAN error ak je
 *
 * \param hcan CAN_HandleTypeDef*
 * \return void
 *
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    hcan->ErrorCode = HAL_CAN_ERROR_NONE;
    hcan->Instance->MSR &= 0x1C;
}
/**
* @brief Event for CAN Rx message
* @param Can controller message structure
*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
    switch (RxHeader.StdId)
    {

#ifdef Rx_MCU_IMU_angular_velocity
        case ID_MCU_IMU_angular_velocity:
            Rx_MCU_IMU_angular_velocity_Data(hcan, &MCU_IMU_angular_velocity_Data);
            break;
#endif

#ifdef Rx_MCU_IMU_acceleration
        case ID_MCU_IMU_acceleration:
            Rx_MCU_IMU_acceleration_Data(hcan, &MCU_IMU_acceleration_Data);
            break;
#endif

#ifdef Rx_MCU_IMU_euler_angles
        case ID_MCU_IMU_euler_angles:
            Rx_MCU_IMU_euler_angles_Data(hcan, &MCU_IMU_euler_angles_Data);
            break;
#endif

#ifdef Rx_MCU_IMU_gps_position
        case ID_MCU_IMU_gps_position:
            Rx_MCU_IMU_gps_position_Data(hcan, &MCU_IMU_gps_position_Data);
            break;
#endif

#ifdef Rx_MCU_IMU_gps_speed
        case ID_MCU_IMU_gps_speed:
            Rx_MCU_IMU_gps_speed_Data(hcan, &MCU_IMU_gps_speed_Data);
            break;
#endif

        default:
            break;
    }
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	HAL_CAN_RxCpltCallback(hcan);
}

