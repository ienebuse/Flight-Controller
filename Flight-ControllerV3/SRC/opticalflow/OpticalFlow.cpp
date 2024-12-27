/*
 * OpticalFlow.cpp
 *
 *  Created on: Aug 9, 2024
 *      Author: Ikenna
 */

#include <OpticalFlow.h>
#include <string.h>
#include <usart.h>
#include <stdio.h>
#include <Configurator.h>
#include "usbd_cdc_if.h"


#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8


const uint8_t OpticalFlow::MAX_BUFFER_SIZE;

//typedef struct __attribute__((packed)) {
//	uint64_t time_us;
//	uint8_t id;
//	int16_t flow_x;			// dpix
//	int16_t flow_y;			// dpix
//	float flow_comp_m_x;	// m/s
//	float flow_comp_m_y;	// m/s
//	uint8_t quality;
//	float gDistance;		// m
//}MAV_FLOW_Data_t;

typedef struct __attribute__((packed)) {
	uint64_t time_us;
	float flow_comp_m_x;	// m/s
	float flow_comp_m_y;	// m/s
	float gDistance;		// m
	int16_t flow_x;			// dpix
	int16_t flow_y;			// dpix
	uint8_t id;
	uint8_t quality;
}MAV_FLOW_Data_t;

typedef struct __attribute__ ((packed)) {
	uint32_t time_us;		// ms
	uint16_t min_dist;		// cm
	uint16_t max_dist;		// cm
	uint16_t dist;			// cm
	uint8_t sensor_type;
	uint8_t id;
	uint8_t orientation;
	uint8_t cov;			// cm^2
}MAV_RANGE_Data_t;

typedef struct __attribute__((packed)) {
	uint16_t distance; 		// mm
	uint8_t distStrength;	// 0-255
	uint8_t reserved1;
	int16_t flow_x;			// dpix
	int16_t flow_y;			// dpix
	uint16_t timespan;
	uint8_t valid;		// 1-valid; 0-invalid
	uint8_t version;
}UPIXEL_FLOW_Data_t;


typedef struct __attribute__((packed)) {
	uint32_t time_us;		// ms
	uint32_t gDistance;		// mm
	uint8_t distStrength;	// 0-255
	uint8_t distPrecision;	// 0-255; 0 > 255
	uint8_t distStatus;		// 1-valid, 0-invalid
	uint8_t reserved1;
	int16_t flow_x;			// cm/sec @ 1m; speed(cm/s) = flow_vel * distance(m)
	int16_t flow_y;			// cm/sec @ 1m; speed(cm/s) = flow_vel * distance(m)
	uint8_t flw_qlty;		// flow quality 0-255; 255 > 0
	uint8_t flw_status;		// 1-valid; 0-invalid
	uint16_t reserved2;
}MICRO_LINK_Data_t;

#ifndef HAVE_CRC_ACCUMULATE
/**
 * @brief Accumulate the CRC16_MCRF4XX checksum by adding one char at a time.
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new char to hash
 * @param crcAccum the already accumulated checksum
 **/
static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
        /*Accumulate one byte of data into the CRC*/
        uint8_t tmp;

        tmp = data ^ (uint8_t)(*crcAccum &0xff);
        tmp ^= (tmp<<4);
        *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}
#endif


/**
 * @brief Initialize the buffer for the MCRF4XX CRC16
 *
 * @param crcAccum the 16 bit MCRF4XX CRC16
 */
static inline void crc_init(uint16_t* crcAccum)
{
        *crcAccum = X25_INIT_CRC;
}


/**
 * @brief Accumulate the MCRF4XX CRC16 by adding an array of bytes
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new bytes to hash
 * @param crcAccum the already accumulated checksum
 **/
static inline void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
	const uint8_t *p = (const uint8_t *)pBuffer;
	while (length--) {
                crc_accumulate(*p++, crcAccum);
        }
}


/**
 * @brief Calculates the CRC16_MCRF4XX checksum on a byte buffer
 *
 * @param  pBuffer buffer containing the byte array to hash
 * @param  length  length of the byte array
 * @return the checksum over the buffer bytes
 **/
static inline uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length, uint8_t crc_extra)
{
        uint16_t crcTmp;
        crc_init(&crcTmp);
	while (length--) {
                crc_accumulate(*pBuffer++, &crcTmp);
        }

	crc_accumulate(crc_extra, &crcTmp);
    return crcTmp;
}

static inline uint8_t checksum(uint8_t *data, size_t length) {
    unsigned char checksum = 0;

    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }

    return checksum;
}

OpticalFlow::OpticalFlow() {
	// TODO Auto-generated constructor stub

}

OpticalFlow::~OpticalFlow() {
	// TODO Auto-generated destructor stub
}

void OpticalFlow::init(UART_HandleTypeDef* huart, AHRS* ahrs) {
//	m_uartRx.init(huart, MAX_BUFFER_SIZE);
	m_uart = huart;
	m_ahrs = ahrs;

//	xFilt.init(0.5, 50);
//	yFilt.init(0.5, 50);
//	zFilt.init(10, 50);

	xFilt.init(0.5, 50);
	yFilt.init(0.5, 50);
	zFilt.init(20, 50);

	HAL_UART_AbortReceive(huart);

	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);  	// Enable RXNE interrupt
	HAL_NVIC_EnableIRQ(LPUART1_IRQn);              	// Enable the USART2 interrupt in the NVIC

	__HAL_UART_ENABLE(huart);

	startOptFlw();
}



static void sendData(uint8_t* data, uint16_t len) {
	CDC_Transmit_FS(data, len);
}

bool OpticalFlow::parseOptFlwData() {
	static Config config = Configurator::getConfig();
	volatile uint8_t qlty;
	uint32_t xflw, yflw;
	static uint8_t payload[40];
	uint8_t len, flag;

	static  uint8_t remLen = 1;
	static uint16_t payloadSize = 0;
	static bool first = true;
	uint8_t data = 0;

	uint8_t indx =  0;
//	}

	newEvent = false;

	if(config.settings.OptFlwUseUPixel) {
		switch(m_state) {
			case 0:
				data = buffer[0];
				if(data == 0xDF) {
					payload[0] = data;
					m_state = 1;
					remLen = 1;
				}
				break;
			case 1:
				data = buffer[0];
				if(data == 0x15) {
					payload[1] = data;
					m_state = 2;
					remLen = 1;
				}
				else {
					m_state = 0;
					remLen = 1;
				}
				break;
			case 2:
				data = buffer[0];
				if(data == 0x00) {
					payload[2] = data;
					m_state = 3;
					remLen = 3;
				}
				else {
					m_state = 0;
					remLen = 1;
				}
				break;
			case 3:
				memcpy(&payload[3], buffer, remLen);
				if(payload[3] == 0x55) {
					payloadSize = payload[5];
					remLen = payloadSize + 1;
					m_state = 4;
				}
				else {
					m_state = 0;
					remLen = 1;
				}
				break;
			case 4:
				memcpy(&payload[6], buffer, remLen);
				uint8_t crc = (uint16_t)payload[payloadSize + 6];
				uint8_t crc_eval = checksum(payload, payloadSize + 6);

				if(crc == crc_eval){
					UPIXEL_FLOW_Data_t* flowData = (UPIXEL_FLOW_Data_t*)(payload + 6);
					if(flowData->valid > 5) {
						m_xFlwSum -= flowData->flow_x;
						m_yFlwSum -= flowData->flow_y;
						flowRdy = true;
						++flowCount;
					};
					if(flowData->distStrength >= 10) {
						m_hLidar = (float)flowData->distance;
						lidarRdy = true;
					}
				}
				m_state = 0;
				remLen = 1;
				payloadSize = 0;
				break;
		}
	}
	else if(config.settings.OptFlwUseMSP) {

		switch(m_state) {
			case 0:
				data = buffer[0];
				if((char)data == '$') {
					m_state = 1;
					remLen = 1;
				}
				break;
			case 1:
				data = buffer[0];
				if((char)data == 'X') {
					m_state = 2;
					remLen = 1;
				}
				else {
					m_state = 0;
					remLen = 1;
				}
				break;
			case 2:
				data = buffer[0];
				if((char)data == '<') {
					m_state = 3;
					remLen = 5;
					indx = 0;
				}
				else {
					m_state = 0;
					remLen = 1;
				}
				break;
			case 3:
				memcpy(&payload[0], buffer, remLen);
					 if(payload[0] == 0) {
						payloadSize = le16(&payload[3]);	// Get payload size
						if(payloadSize == 5 || payloadSize == 9) {
							m_state = 4;
							remLen = payloadSize+1;
						}
						else {
							m_state = 0;
							remLen = 1;
						}
					}
					else {
						m_state = 0;
						remLen = 1;
					}
				break;
			case 4:
				memcpy(&payload[5], buffer, remLen);
				uint8_t crc = payload[payloadSize+5];
				uint8_t crc_eval = checkSum(payload, payloadSize + 5);

				if(crc == crc_eval) {
					uint16_t funct = le16(&payload[1]);
					qlty = payload[5];
					if(funct == FUNC_LIDAR && payloadSize == 5) {
						m_hLidar = le32(&payload[6]);
						lidarRdy = true;
					}
					else if(funct == FUNC_FLOW && payloadSize == 9) {
						xflw = le32(&payload[6]);
						yflw = le32(&payload[10]);

						m_xFlwSum += (int32_t)xflw;
						m_yFlwSum += (int32_t)yflw;
						m_state = 0;
						++flowCount;
						flowRdy = true;
						remLen = 1;
					}
				}
				m_state = 0;
				payloadSize = 0;
				remLen = 1;
				break;
		}
	}
	else if(config.settings.OptFlwUseMavLink) {

		static uint16_t funct = 0;

		switch(m_state) {
			case 0:
				data = buffer[0];
				if(data == 0xFE) {
					m_state = 1;
					remLen = 5;
				}
				break;
			case 1:
				memcpy(&payload[0], buffer, remLen);
				if(payload[2] == 0x01 && payload[3] == 0x58) {
					payloadSize = payload[0];
					remLen = payloadSize + 2;
					if(payload[4] == 0x64){
						funct = FUNC_FLOW;
						m_state = 2;
					}
					else if(payload[4] == 0x84){
						funct = FUNC_LIDAR;
						m_state = 2;
					}
					else {
						m_state = 0;
						remLen = 1;
					}
				}
				else {
					m_state = 0;
					remLen = 1;
				}
				break;
			case 2:
				memcpy(&payload[5], buffer, remLen);
				if(payload[0] == 0x1A) {
					volatile uint8_t x = 0;
					x++;
				}
				uint8_t CRC_EXTRA = 0;
				if(funct == FUNC_FLOW) {
					CRC_EXTRA = 0xAF;
				}
				else if(funct == FUNC_LIDAR) {
					CRC_EXTRA = 0x55;
				}
				else {
					m_state = 0;
					remLen = 1;
					payloadSize = 0;
					break;
				}
				uint16_t crc = (uint16_t)payload[payloadSize + 6] << 8 | (uint16_t)payload[payloadSize + 5];
				uint16_t crc_eval = crc_calculate(payload, payloadSize + 5, CRC_EXTRA);

				if(crc == crc_eval){
					if(funct == FUNC_FLOW) {
						MAV_FLOW_Data_t* flowData = (MAV_FLOW_Data_t*)(payload + 5);
						m_xFlwSum += flowData->flow_x;
						m_yFlwSum += flowData->flow_y;
						++flowCount;
						flowRdy = true;
					}
					else if(funct == FUNC_LIDAR) {
						MAV_RANGE_Data_t* rangeData = (MAV_RANGE_Data_t*)(payload + 5);
						m_hLidar = (float)rangeData->dist;
						lidarRdy = true;
					}
				}
				m_state = 0;
				remLen = 1;
				payloadSize = 0;
				break;
		}
	}
	else if(config.settings.OptFlwUseMicrolink) {


		switch(m_state) {
			case 0:
				data = buffer[0];
				if(data == 0xEF) {
					payload[0] = data;
					m_state = 1;
					remLen = 1;
				}
				break;
			case 1:
				data = buffer[0];
				if(data == 0x0F) {
					payload[1] = data;
					m_state = 2;
					remLen = 1;
				}
				else {
					m_state = 0;
					remLen = 1;
				}
				break;
			case 2:
				data = buffer[0];
				if(data == 0x00) {
					payload[2] = data;
					m_state = 3;
					remLen = 3;
				}
				else {
					m_state = 0;
					remLen = 1;
				}
				break;
			case 3:
				memcpy(&payload[3], buffer, remLen);
				if(payload[3] == 0x51) {
					payloadSize = payload[5];
					remLen = payloadSize + 1;
					m_state = 4;
				}
				else {
					m_state = 0;
					remLen = 1;
				}
				break;
			case 4:
				memcpy(&payload[6], buffer, remLen);
				uint8_t crc = (uint16_t)payload[payloadSize + 6];
				uint8_t crc_eval = checksum(payload, payloadSize + 6);

				if(crc == crc_eval){
					MICRO_LINK_Data_t* flowData = (MICRO_LINK_Data_t*)(payload + 6);
					if(flowData->flw_status == 1) {
						m_xFlwSum += flowData->flow_x;
						m_yFlwSum += flowData->flow_y;
						flowRdy = true;
						++flowCount;
					};
					if(flowData->distStatus == 1) {
						m_hLidar = (float)flowData->gDistance;
						lidarRdy = true;
					}
				}
				m_state = 0;
				remLen = 1;
				payloadSize = 0;
				break;
		}
	}

	HAL_UART_Receive_IT(m_uart, buffer, remLen);

	return (flowRdy || lidarRdy);
}

uint16_t OpticalFlow::le16(const uint8_t* const data) {
	uint16_t le = (uint16_t)data[1] << 8 | (uint16_t)data[0];
	return le;
}

uint32_t OpticalFlow::le32(const uint8_t* const data) {
	uint32_t le = (uint32_t)data[3] << 24 | (uint32_t)data[2] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[0];
	return le;
}

uint8_t OpticalFlow::checkSum(uint8_t* data, uint8_t len) {
	uint8_t ck2 = 0; // initialise CRC

	for (int i = 0; i < len; i++) {
	    ck2 = crc8_dvb_s2(ck2, data[i]);
	}
	return ck2;
}

uint8_t OpticalFlow::crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

void OpticalFlow::taskFunc(timetick_us currenTimeUs) {
//	static char buff[60];
//	static float z = 0;
	float vwx = 0;
	float vwy = 0;
	if(lidarRdy) {
		currentFlowData.z = zFilt.apply((float)m_hLidar);
		lidarRdy = false;
	}

	if(flowRdy) {
		currentFlowData.x = xFilt.apply((float)(m_xFlwSum * currentFlowData.z/1000));
		currentFlowData.y = yFilt.apply((float)(m_yFlwSum * currentFlowData.z/1000));

		float tmp;
		switch(OPTICAL_FLOW_ROT_CCW) {
			case 90:
				tmp = currentFlowData.y;
				currentFlowData.y = -currentFlowData.x;
				currentFlowData.x = tmp;
				break;
			case 180:
				currentFlowData.x = -currentFlowData.x;
				currentFlowData.y = -currentFlowData.y;
				break;
			case 270:
				tmp = currentFlowData.y;
				currentFlowData.y = currentFlowData.x;
				currentFlowData.x = -tmp;
				break;
		}

		if(OPTICAL_FLOW_INVERT_X) {
			currentFlowData.x = -currentFlowData.x;
		}

		if(OPTICAL_FLOW_INVERT_Y) {
			currentFlowData.y = -currentFlowData.y;
		}

		Quat q = m_ahrs->getCurrentAttitude().quat;
		float q02 = q.q0*q.q0;
		float q12 = q.q1*q.q1;
		float q22 = q.q2*q.q2;
		float q32 = q.q3*q.q3;

		float _2q1q2 = 2*q.q1*q.q2;
		float _2q0q3 = 2*q.q0*q.q3;



		if(currentFlowData.z > 100) {
			vwx = currentFlowData.x*(q02 + q12 - q22 - q32) + currentFlowData.y*(_2q1q2 - _2q0q3);
			vwy = currentFlowData.x*(_2q1q2 + _2q0q3) + currentFlowData.y*(q02 - q12 + q22 - q32);
//
			wPos.x += vwx * 0.02f;
			wPos.y += vwy * 0.02f;

//			lPos.x += currentFlowData.x * 0.02f;
//			lPos.y += currentFlowData.y * 0.02f;


		}
		else {
			wPos.x = 0;
			wPos.y = 0;
//			lPos.x = 0;
//			lPos.y = 0;
		}

//		float h1 = currentFlowData.z * cos(roll) * cos(pitch);
//		float h2 = currentFlowData.z * (q02 - q12 - q22 + q32);//(q02 + q12 - q22 + q32);

		totalTime += currenTimeUs - lastTime;
		lastTime = currenTimeUs;

//		snprintf(buff, 60, "%.1f,%.1f,%.1f,%.1f\r\n",vEst.x,vEst.y,vwx,vwy);
//		snprintf(buff, 60, "%.1f,%.1f,%.3f\r\n",wPos.x,wPos.y,yaw);
//		snprintf(buff, 60, "%.3f,%.3f,%.3f\r\n",h1,h2,currentFlowData.z);
//		snprintf(buff, 60, "%.3f,%.3f,%.3f\r\n",currentFlowData.x,currentFlowData.y, currentFlowData.z);
//		sendData((uint8_t*)buff, strlen(buff));

		m_xFlwSum = 0;
		m_yFlwSum = 0;
		flowCount = 0;
		totalTime = 0;
		flowRdy = false;

		float h = currentFlowData.z * (q02 - q12 - q22 + q32);
		m_osd->setOptFlwAltitude(h);
	}

}

