/**************************************************************************//**
  \file app.h

  \brief Usart-Anwendung Headerdatei.

  \author
    Markus Krauﬂe

******************************************************************************/

#ifndef _APP_H
#define _APP_H

#define SPS30_DATA_SIZE 30
#define SPS30_DEVICE_ADDRESS 0x69
//#define SPS30_REQUEST_LOOP 2000		//02 seconds
#define SPS30_REQUEST_LOOP 10000	//10 seconds
//#define SPS30_REQUEST_LOOP 60000	//1 minute
#define SPS30_WAIT_MES 10
#define TRANSMIT_WAIT 2
#define SPS30_HIGH_REAP 0x00

#define SPS30_START_MES 0x00, 0x10, 0x05, 0x00, 0xF6
#define SPS30_READ_MES 0x03, 0x00
#define SPS30_STOP_MES 0x01, 0x04


#define ZIGBEE_MODEL_ID 6
#define PM25_MASS_CON 61
#define PM10_MASS_CON 62
#define PM25_NUM_CON 63
#define PM10_NUM_CON 64

typedef enum{
	SPS30_INIT_STATE,
	SPS30_REQ_MES_STATE,
	SPS30_WRITE_MES_STATE,
	SPS30_READ_MES_STATE,
	APP_TRANSMIT_1,
	APP_TRANSMIT_2,
	APP_TRANSMIT_3,
	APP_TRANSMIT_4,
	APP_NOTHING_STATE,
	SPS30_STOP_MES_STATE,
	APP_INIT_STATE,
	APP_STARTJOIN_NETWORK,
	APP_INIT_ENDPOINT,
	APP_INIT_TRANSMITDATA
} AppState_t;
#endif

BEGIN_PACK
typedef struct _AppMessage_t{
	uint8_t header[APS_ASDU_OFFSET]; //APS header
	uint8_t data[6];
	uint8_t footer[APS_AFFIX_LENGTH - APS_ASDU_OFFSET]; // Footer
} PACK AppMessage_t;
END_PACK
// eof app.h