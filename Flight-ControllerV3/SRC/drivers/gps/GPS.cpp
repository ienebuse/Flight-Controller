/*
 * M8N.cpp
 *
 *  Created on: Jul 29, 2024
 *      Author: Ikenna
 */

#include <GPS.h>

GPS::GPS() {
	// TODO Auto-generated constructor stub

}

GPS::~GPS() {
	// TODO Auto-generated destructor stub
}

//uint8_t UBX_CFG_PRT[] = {
//	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
//	0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x01, 0x00,
//	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79
//};
//
//uint8_t UBX_CFG_NMEA410[]={0xB5,0x62,0x06,0x17,0x14,0x00,0x00,0x41,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x75,0x57};
//
////Activation of navigation system: Galileo, Glonass, GPS, SBAS, IMES
////uint8_t UBX_CFG_GNSS[]={0xB5,0x62,0x06,0x3E,0x24,0x00,0x00,0x00,0x20,0x04,0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x01,0x00,0x01,0x01,0x02,0x04,0x08,0x00,0x01,0x00,0x01,0x01,0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01,0xDF,0xFB};
//
//uint8_t UBX_CFG_ENABLE_PVT[]={
//	    0xB5, 0x62, // Header
//	    0x06, 0x3E, // Class and ID (CFG-MSG)
//	    0x08, 0x00, // Length
//	    0x01, 0x07, // Message Class and ID
//	    0x00, 0x00, // Rate (default)
//	    0x00, 0x00, // Reserved
//	    0xD7, 0x54  // Checksum
//	};
//
//uint8_t UBX_CFG_MSG[] = {
//	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01,
//	0x00, 0x00, 0x00, 0x00, 0x13, 0xBE
//};
//
////uint8_t UBX_CFG_RATE[] = {
////	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
////	/*0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, */
////	0x64 0x00 0x01 0x00 0x01 0x00,
////	0xDE, 0x6A
////};
//
////uint8_t UBX_CFG_RATE[] = {
////	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
////	0x64, 0x00, 0x01, 0x00, 0x01, 0x00,
////	0x66, 0x5E
////};
//
//uint8_t UBX_CFG_RATE[] = {
//    0xB5, 0x62, // Header
//    0x06, 0x3E, // Class and ID (CFG-MSG)
//    0x08, 0x00, // Length
//    0x01, 0x07, // Message Class and ID
//    0x0A, 0x00, // Rate (10 Hz)
//    0x00, 0x00, // Reserved
//    0xD9, 0x5E  // Checksum
//};
//
//uint8_t UBX_CFG_CFG[] = {
//	0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
//	0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31,
//	0xBF
//};
//
//uint8_t UBX_POSLLH_Data[] = {
//		0xB5,0x62,0x01,0x02,0x00,0x00,0x03,0x0A
//};
//
//uint8_t UBX_PVT_Data[] = {
//		0xB5,0x62,0x01,0x07,0x00,0x00,0x08,0x19
//};
//
////uint8_t UBX_PVT_Data[] = {
////		0xB5,0x62,0x06,0x01,0x03,0x00,
////		0x01,0x07,0x01,
////		0x09, 0x11
////};
//
////uint8_t UBX_PVT_Data[] = {
////    0xB5, 0x62,  // Header
////    0x06, 0x01,  // Class and ID
////    0x03, 0x00,  // Length
////    0x01, 0x07, 0x01,  // Payload
////    0x09, 0x11  // Checksum
////};


uint8_t UBX_CFG_PRT[] = {
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
	0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x01, 0x00,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79
};

uint8_t UBX_CFG_MSG[] = {
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x13, 0xBE
};

uint8_t UBX_CFG_RATE[] = {
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00,
	0x01, 0x00, 0xDE, 0x6A
};

uint8_t UBX_CFG_CFG[] = {
	0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31,
	0xBF
};

uint8_t UBX_POSLLH_Data[] = {
		0xB5,0x62,0x01,0x02,0x00,0x00,0x03,0x0A
};

uint8_t UBX_PVT_Data[] = {
		0xB5,0x62,0x01,0x07,0x00,0x00,0x08,0x19
};

void GPS::transmitData(uint8_t* data, uint16_t len)
{
	HAL_UART_Transmit_DMA(m_uart, data, len);
}

//void GPS::init(UART_HandleTypeDef* huart)
//{
//	m_uart = huart;
//
//	transmitData(UBX_CFG_PRT, sizeof(UBX_CFG_PRT));
//	HAL_Delay(250);
////	transmitData(UBX_CFG_NMEA410, sizeof(UBX_CFG_NMEA410));
////	HAL_Delay(250);
//	transmitData(UBX_CFG_ENABLE_PVT, sizeof(UBX_CFG_ENABLE_PVT));
//	HAL_Delay(250);
////	transmitData(UBX_CFG_MSG, sizeof(UBX_CFG_MSG));
////	HAL_Delay(250);
//	transmitData(UBX_CFG_RATE, sizeof(UBX_CFG_RATE));
//	HAL_Delay(250);
////	transmitData(UBX_CFG_CFG, sizeof(UBX_CFG_CFG));
////	HAL_Delay(250);
////	M8N_TransmitData(pvtRequest(), sizeof(UBX_POSLLH_Data));
////	HAL_UART_Receive_DMA(m_uart, &m_rxData, 1);
//	pvtRequest();
//}


//void GPS::init(UART_HandleTypeDef* huart)
//{
//	m_uart = huart;
//
//	transmitData(UBX_CFG_PRT, sizeof(UBX_CFG_PRT));
//	HAL_Delay(250);
////	transmitData(UBX_CFG_NMEA410, sizeof(UBX_CFG_NMEA410));
////	HAL_Delay(250);
////	transmitData(UBX_CFG_ENABLE_PVT, sizeof(UBX_CFG_ENABLE_PVT));
////	HAL_Delay(250);
//	transmitData(UBX_CFG_MSG, sizeof(UBX_CFG_MSG));
//	HAL_Delay(250);
//	transmitData(UBX_CFG_RATE, sizeof(UBX_CFG_RATE));
//	HAL_Delay(250);
//	transmitData(UBX_CFG_CFG, sizeof(UBX_CFG_CFG));
//	HAL_Delay(250);
////	M8N_TransmitData(pvtRequest(), sizeof(UBX_POSLLH_Data));
////	HAL_UART_Receive_DMA(m_uart, &m_rxData, 1);
////	pvtRequest();
//	posllhRequest();
//}


void GPS::init(UART_HandleTypeDef* huart)
{
	m_uart = huart;

	transmitData(UBX_CFG_PRT, sizeof(UBX_CFG_PRT));
	HAL_Delay(250);
//	transmitData(UBX_CFG_NMEA410, sizeof(UBX_CFG_NMEA410));
//	HAL_Delay(250);
//	transmitData(UBX_CFG_ENABLE_PVT, sizeof(UBX_CFG_ENABLE_PVT));
//	HAL_Delay(250);
	transmitData(UBX_CFG_MSG, sizeof(UBX_CFG_MSG));
	HAL_Delay(250);
	transmitData(UBX_CFG_RATE, sizeof(UBX_CFG_RATE));
	HAL_Delay(250);
	transmitData(UBX_CFG_CFG, sizeof(UBX_CFG_CFG));
	HAL_Delay(250);
//	M8N_TransmitData(pvtRequest(), sizeof(UBX_POSLLH_Data));
//	HAL_UART_Receive_DMA(m_uart, &m_rxData, 1);
//	pvtRequest();
	posllhRequest();
}


bool GPS::chkSumCheck(unsigned char* data, unsigned char len)
{
	unsigned char CK_A = 0, CK_B = 0;

	for(int i=2;i<len-2;i++)
	{
		CK_A = CK_A + data[i];
		CK_B = CK_B + CK_A;
	}

	return ((CK_A == data[len-2]) && (CK_B == data[len-1]));
}

void GPS::parseNavPOSLLH()
{
	const uint8_t *data = gps_rx_buf;
	navData.CLASS = data[2];
	navData.ID = data[3];
	navData.length = data[4] | data[5]<<8;

	navData.iTOW = data[6] | data[7]<<8 | data[8]<<16 | data[9]<<24;
	navData.lon = data[10] | data[11]<<8 | data[12]<<16 | data[13]<<24;
	navData.lat = data[14] | data[15]<<8 | data[16]<<16 | data[17]<<24;
	navData.height = data[18] | data[19]<<8 | data[20]<<16 | data[21]<<24;
	navData.hMSL = data[22] | data[23]<<8 | data[24]<<16 | data[25]<<24;
	navData.hAcc = data[26] | data[27]<<8 | data[28]<<16 | data[29]<<24;
	navData.vAcc = data[30] | data[31]<<8 | data[32]<<16 | data[33]<<24;

//	posllh->lon_f64 = posllh->lon / 10000000.;
//	posllh->lat_f64 = posllh->lat / 10000000.;

}

void GPS::pvtRequest() {
	if(m_rxState != 0){
		return;
	}
	transmitData(UBX_PVT_Data, sizeof(UBX_PVT_Data));
	HAL_UART_Receive_DMA(m_uart, &m_rxData, 1);
}

void GPS::parseNavPVT()
{
	const uint8_t *data = gps_rx_buf;
	navData.CLASS = data[2];
	navData.ID = data[3];
	navData.length = data[4] | data[5]<<8;

	navData.iTOW = data[6] | data[7]<<8 | data[8]<<16 | data[9]<<24;
	navData.lon = data[30] | data[31]<<8 | data[32]<<16 | data[33]<<24;
	navData.lat = data[34] | data[35]<<8 | data[36]<<16 | data[37]<<24;
	navData.height = data[38] | data[39]<<8 | data[40]<<16 | data[41]<<24;
	navData.hMSL = data[42] | data[43]<<8 | data[44]<<16 | data[45]<<24;
	navData.hAcc = data[46] | data[47]<<8 | data[48]<<16 | data[49]<<24;
	navData.vAcc = data[50] | data[51]<<8 | data[52]<<16 | data[53]<<24;
	navData.vx = data[54] | data[55]<<8 | data[56]<<16 | data[57]<<24;
	navData.vy = data[58] | data[59]<<8 | data[60]<<16 | data[61]<<24;
	navData.vz = data[62] | data[63]<<8 | data[64]<<16 | data[65]<<24;

	navData.lon_f64 = navData.lon / 10000000.;
	navData.lat_f64 = navData.lat / 10000000.;

}

void GPS::posllhRequest() {
	if(m_rxState != 0){
		return;
	}
	transmitData(UBX_POSLLH_Data, sizeof(UBX_POSLLH_Data));
	HAL_UART_Receive_DMA(m_uart, &m_rxData, 1);
}

GPS_UBX_NAV GPS::getData() {
	return navData;
}

UART_HandleTypeDef* GPS::getUart() {
	return m_uart;
}

void GPS::rxHandler() {

	switch(m_rxState)
	{
		case 0:
			if(m_rxData == 0xb5)
			{
				gps_rx_buf[m_rxState] = m_rxData;
				m_rxState++;
				m_startTime = TimeTick::getTimeUs();
			}
			break;
		case 1:
			if(m_rxData == 0x62)
			{
				gps_rx_buf[m_rxState] = m_rxData;
				m_rxState++;
			}
			else {
				m_rxState = 0;
				m_rxError = true;
			}
			break;
		case 2:
			if(m_rxData == 0x01)	//NAV
			{
				gps_rx_buf[m_rxState] = m_rxData;
				m_rxState++;
//				m_startTime = TimeTick::getTimeUs();
			}
			else {
				m_rxState = 0;
				m_rxError = true;
			}
			break;
		case 3:
			if(m_rxData == 0x02)	// POSLLH
			{
				gps_rx_buf[m_rxState] = m_rxData;
				m_rxState++;
				m_dataLength = POSLLH;
			}
			else if(m_rxData == 0x07)	// PVT
			{
				gps_rx_buf[m_rxState] = m_rxData;
				m_rxState++;
				m_dataLength = PVT;
			}
			else {
				m_rxState = 0;
				m_rxError = true;
			}
			break;
		case 35:
			gps_rx_buf[m_rxState] = m_rxData;
			m_rxState++;
			if(m_dataLength == POSLLH) {
				m_rxState = 0;
				gps_rx_cplt_flag = true;
			}
			break;
		case 99:
			gps_rx_buf[m_rxState] = m_rxData;
			m_rxState = 0;
			if(m_dataLength == PVT) {
				m_rxState = 0;
				gps_rx_cplt_flag = true;
			}
			break;
		default:
			if(m_rxState >= 150) {
				m_rxState = 0;
				m_rxError = true;
				break;
			}
			gps_rx_buf[m_rxState] = m_rxData;
			m_rxState++;
			break;
	}

	HAL_UART_Receive_DMA(m_uart, &m_rxData, 1);
}

void GPS::taskFunc(timetick_us currenTimeUs) {
	navData.new_data = false;
	if(gps_rx_cplt_flag) {
		gps_rx_cplt_flag = false;
		if(m_dataLength == POSLLH) {
			if(chkSumCheck(gps_rx_buf, 36)) {
				parseNavPOSLLH();
				navData.new_data = true;
				navData.lastDataTime = currenTimeUs;
			}
		}
		else if(m_dataLength == PVT) {
			if(chkSumCheck(gps_rx_buf, 100)) {
				parseNavPVT();
				navData.new_data = true;
				navData.lastDataTime = currenTimeUs;
			}
		}
	}
	else if(m_rxError) {
		m_rxError = false;
	}
	else if(currenTimeUs - m_startTime > cm_TIMEOUT_US) {
		m_rxState = 0;
	}
	posllhRequest();
//	pvtRequest();

}

