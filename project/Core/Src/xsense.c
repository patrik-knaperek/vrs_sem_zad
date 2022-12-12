/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"
#include "xsense.h"
#include "main.h"
#include "CAN2022.h"
#include "math.h"

//void send_sgt_CAN(void);

/* Variables -----------------------------------------------------------------*/

uint8_t XSENSE_rx_buffer[XSENSE_rx_buffer_size];

struct XDI_TemperatureDataType XDI_Temperature;
struct XDI_UtcTimeDataType XDI_UtcTime;
struct XDI_PacketCounterDataType XDI_PacketCounter;
struct XDI_SampleTimeFineDataType XDI_SampleTimeFine;
struct XDI_SampleTimeCoarseDataType XDI_SampleTimeCoarse;
struct XDI_RotationMatrixDataType XDI_RotationMatrix;
struct XDI_EulerAnglesDataType XDI_EulerAngles;
struct XDI_AccelerationDataType XDI_Acceleration;
struct XDI_GPSPositionDataType XDI_GPSPosition;
struct XDI_RateOfTurnDataType XDI_RateOfTurn;
struct XDI_VelocityDataType XDI_Velocity;

extern CAN_HandleTypeDef hcan1;
extern MCU_IMU_angular_velocity_TypeDef MCU_IMU_angular_velocity_Data;
//extern MCU_IMU_angular_velocity1_TypeDef MCU_IMU_angular_velocity1_Data;

extern MCU_IMU_acceleration_TypeDef MCU_IMU_acceleration_Data;
//extern MCU_IMU_acceleration1_TypeDef MCU_IMU_acceleration1_Data;

extern MCU_IMU_euler_angles_TypeDef MCU_IMU_euler_angles_Data;
//extern MCU_IMU_euler_angles1_TypeDef MCU_IMU_euler_angles1_Data;

//extern MCU_IMU_gps_position_TypeDef MCU_IMU_gps_position_Data;
extern MCU_IMU_gps_speed_TypeDef MCU_IMU_gps_speed_Data;


/* UART RX Idle line interrupt callback --------------------------------------*/

void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
	if(is_sgt_CAN_busy())
		return;

	/* DMA restart */
	DMA_HandleTypeDef *hdma = huart->hdmarx;
	__HAL_DMA_DISABLE(hdma);									// Disable the channel
	huart->RxXferCount = 0;										// Clear transfer counter
	__HAL_DMA_CLEAR_FLAG(hdma, DMA_FLAG_TE6);				    // Clear interupt flag
	hdma->Instance->CMAR = (uint32_t)XSENSE_rx_buffer[0];   	// Set memory address for DMA again
	hdma->Instance->CNDTR = XSENSE_rx_buffer_size;   			// Set number of bytes to receive
	hdma->Instance->CCR |= DMA_CCR_EN;            				// Start DMA transfer

	/* Variables */
	static uint8_t msg_counter;
	uint16_t buff_indx = 0;

	struct XsenseDataType
	{
		uint8_t preamble;				// Indicator of start of message
		uint8_t bid;					// Bus identifier or Address
		uint8_t mid;					// Message ID
		uint8_t len;					// For standard length message: Value equals number of bytes in DATA field.
										// For extended length message: Field value is always 255 (0xFF)
		uint16_t ext_len;				// 16 bit value representing the number of data bytes for extended length messages
		uint16_t data_ID;				// Packet data identifier
		uint8_t data_LEN;				// Packet data length
		uint8_t checksum;				// Checksum of message
	} MTD_Message;

	XDI_PacketCounter.PacketCounter = 0;
	XDI_SampleTimeFine.SampleTimeFine = 0;

	union
	{
		uint16_t u;
		unsigned char c[2];
	} tmp_u16;

	union
	{
		uint32_t u;
		unsigned char c[4];
	} tmp_u32;

	union
	{
		float f;
		unsigned char c[4];
	} tmp_f;


	/* START OF DATA PARSE */
	/* Steps:
	 *  1. Check Xbus header - PREAMBLE, BID, MID
	 *  2. Check data length - Standard or Extended length
	 *  3. Check data_ID and data_LEN and jump to appropriate case function to process data
	 *  4. Repeat (do) step 3. until (while) buffer index reach end of the buffer
	 */

	if(XSENSE_rx_buffer[0] == PREAMBLE)				// Check PREAMBLE
	{
	if(XSENSE_rx_buffer[1] == BID)					// Check BID
	{
	if(XSENSE_rx_buffer[2] == MID)					// Check MID
	{
	if(XSENSE_rx_buffer[3] < 0xFF)					// Standard data length
	{
		MTD_Message.len = XSENSE_rx_buffer[3];
		buff_indx = 4;
		do											// Repeat (do) until buffer index reach end of buffer
		{
			tmp_u16.c[1] = XSENSE_rx_buffer[buff_indx];
			buff_indx++;
			tmp_u16.c[0] = XSENSE_rx_buffer[buff_indx];
			buff_indx++;
			MTD_Message.data_ID = tmp_u16.u;

			MTD_Message.data_LEN = XSENSE_rx_buffer[buff_indx];												// Check data_LEN
			buff_indx++;

			switch(MTD_Message.data_ID)				// Jump to appropriate case function to process data
			{
				case DID_PacketCounter:
				{
					tmp_u16.c[1] = XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_u16.c[0] = XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_PacketCounter.PacketCounter = tmp_u16.u;
				}break;

				case DID_SampleTimeFine:
				{
					tmp_u32.c[3] = XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_u32.c[2] = XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_u32.c[1] = XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_u32.c[0] = XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_SampleTimeFine.SampleTimeFine = tmp_u32.u;
				} break;

				case DID_EulerAngles:
				{
					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_EulerAngles.roll = tmp_f.f;


					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_EulerAngles.pitch = tmp_f.f;


					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_EulerAngles.yaw = tmp_f.f;
				} break;


				case DID_Acceleration:
				{
					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_Acceleration.accx = tmp_f.f;


					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_Acceleration.accy = tmp_f.f;


					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_Acceleration.accz = tmp_f.f;
				} break;

				case DID_RateOfTurn:
				{
					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_RateOfTurn.gyrX = tmp_f.f;


					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_RateOfTurn.gyrY = tmp_f.f;


					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_RateOfTurn.gyrZ = tmp_f.f;
				} break;

				case DID_LatLon:
				{
					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_GPSPosition.latitude = tmp_f.f;


					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_GPSPosition.longitude = tmp_f.f;
				} break;

				case DID_VelocityXYZ:
				{
					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_Velocity.velX = tmp_f.f;


					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_Velocity.velY = tmp_f.f;


					tmp_f.c[3] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[2] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[1] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					tmp_f.c[0] = (unsigned char)XSENSE_rx_buffer[buff_indx];
					buff_indx++;
					XDI_Velocity.velZ = tmp_f.f;

					XDI_Velocity.velSum = sqrt(XDI_Velocity.velX * XDI_Velocity.velX
							+ XDI_Velocity.velY * XDI_Velocity.velY
							+ XDI_Velocity.velZ * XDI_Velocity.velZ);
				} break;

				default:
				{
					// skip packet
					buff_indx = buff_indx + MTD_Message.data_LEN;
				}
			}
		} while(buff_indx <= (MTD_Message.len + 4));
	}

	else	// Extended data length
	{
	// 	TODO data parse for extended data length
		MTD_Message.len = XSENSE_rx_buffer[4];
		buff_indx = 5;
	//	do
	//	{
	//	switch - case
	//	}
	//	while
	}
	}
	}
	}


	/* Blink IMU LED every 10 received messages */
	if (msg_counter > 10)
	{
		msg_counter = 0;
		HAL_GPIO_TogglePin(USART_RX_L_GPIO_Port, USART_RX_L_Pin);
	}
	else
	{
		msg_counter++;
	}

	set_MCU_IMU_data();
	HAL_UART_Receive_DMA(huart,XSENSE_rx_buffer,XSENSE_rx_buffer_size);		// enable DMA Rx cplt interrupt		// na test iba tx complete IT
}
