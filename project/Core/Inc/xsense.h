/* Define section ------------------------------------------------------------*/

#define XSENSE_rx_buffer_size	64
#define PREAMBLE			  0xFA			// Indicator of start of packet
#define BID					  0xFF			// XBus Master identifier
#define MID 				  0x36			// MTData2 massage identifier

/* Variables -----------------------------------------------------------------*/

extern uint8_t msg_counter;
extern uint8_t XSENSE_rx_buffer[XSENSE_rx_buffer_size];

/* Available MTData2 DATA_IDs with Float32 + ENU output format ---------------*/

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_TemperatureGroup		  	0x08x0
#define	DID_Temperature			0x0810			// °C
extern struct XDI_TemperatureDataType XDI_Temperature;

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_TimestampGroup		  	0x10x0
#define DID_UtcTime				0x1010			// N/A
extern struct XDI_UtcTimeDataType XDI_UtcTime;

#define	DID_PacketCounter		0x1020			// N/A
extern struct XDI_PacketCounterDataType XDI_PacketCounter;

#define	DID_SampleTimeFine		0x1060			// N/A
extern struct XDI_SampleTimeFineDataType XDI_SampleTimeFine;

#define DID_SampleTimeCoarse 	0x1070			// s
extern struct XDI_SampleTimeCoarseDataType XDI_SampleTimeCoarse;

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_OrientationGroup		  	0x20xy
#define	DID_Quaternion			0x2010			// N/A
typedef struct XDI_QuaternionDataType
{
	float q1;
	float q2;
	float q3;
	float q4;
}XDI_Quaternion;

#define	DID_RotationMatrix		0x2020			// N/A
extern struct XDI_RotationMatrixDataType XDI_RotationMatrix;

#define DID_EulerAngles			0x2030			// deg
extern struct XDI_EulerAnglesDataType XDI_EulerAngles;

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_PressureGroup			0x30xy
#define	XDI_BaroPressure		0x3010			// Pa

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_AccelerationGroup	  	0x40xy
#define	DID_DeltaV				0x4010			// m/s
#define	DID_Acceleration		0x4020			// m/s^2
extern struct XDI_AccelerationDataType XDI_Acceleration;

#define	DID_FreeAcceleration	0x4030			// m/s^2
#define	DID_AccelerationHR		0x4040			// m/s^2

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_PositionGroup 		  	0x50xy
#define DID_AltitudeEllipsoid	0x5020			// m
#define DID_PositionEcef		0x5030			// m
#define DID_LatLon				0x5040			// m
extern struct XDI_GPSPositionDataType XDI_GPSPosition;

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_GnssGroup			  	0x70x0
#define DID_GnssPvtData			0x7010			// N/A
#define DID_GnssSatInfo			0x7020			// N/A
#define DID_GnssPvtPulse		0x7030			// s

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_AngularVelocityGroup	  	0x80xy
#define	DID_RateOfTurn			0x8020			// rad/s
extern struct XDI_RateOfTurnDataType XDI_RateOfTurn;

#define	DID_DeltaQ				0x8030			// N/A
#define	DID_RateOfTurnHR		0x8040			// rad/s

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_RawSensorGroup 		  	0xA0x0
#define DID_RawAccGyrMagTemp	0xA010			// N/A
#define DID_RawGyroTemp			0xA020			// �C

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_MagneticGroup		  	0xC0xy
#define	DID_MagneticField		0xC020			// a.u

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_VelocityGroup 		  	0xD0xy
#define DID_VelocityXYZ			0xD010			// m/s
extern struct XDI_VelocityDataType XDI_Velocity;

/* -------------------------------------------------------------*/
/* 		NAME					DATA_ID			  Unit			*/
/*  ------------------------------------------------------------*/
// XDI_StatusGroup 			  	0xE0x0
#define DID_StatusByte			0xE010			// N/A
#define DID_StatusWord			0xE020			// N/A
#define DID_DeviceId			0xE080			// N/A
#define DID_LocationId			0xE090			// N/A
