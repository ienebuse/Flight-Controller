/*
 * Configurator.cpp
 *
 *  Created on: Nov 13, 2024
 *      Author: Ikenna
 */

#include <wifi/Configurator.h>
#include <string.h>
#include <Application.h>
#include <Buzzer.h>


Config Configurator::m_config, Configurator::m_defaultConfig;
Settings Configurator::m_settings;
Pid Configurator::m_pid;

bool Configurator::m_canSendLog{true};
Log_Packet Configurator::logPacket;
bool Configurator::m_sendNewLog{false};


static uint8_t checksum(uint8_t *data, size_t length) {
    unsigned char checksum = 0;

    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }

    return checksum;
}

Configurator::Configurator(AHRS* ahrs, FlightControl* mtor, Meter* meter, GPS* gps, Barometer* baro, OpticalFlow* optFlw, BlackBok* bbox) :
		m_ahrs(ahrs),
		m_motor(mtor),
		m_meter(meter),
		m_gps(gps),
		m_barometer(baro),
		m_optFlw(optFlw),
		m_bBox(bbox)
{
	// TODO Auto-generated constructor stub

}

Configurator::~Configurator() {
	// TODO Auto-generated destructor stub
}

void Configurator::init(UART_HandleTypeDef* huart) {
	m_uart = huart;
	loadConfig();
	HAL_UART_Receive_IT(m_uart, &m_buffer[m_state], 1);
}

bool Configurator::sendLog(Log_Data log) {
	if(!m_canSendLog) {
		return false;
	}
	logPacket.logData = log;
	m_sendNewLog = true;
	m_canSendLog = false;
	return true;
}

void Configurator::loadConfig() {
	bool settingsState = loadSettings();
	bool pidState = loadPid();

	if(!(settingsState && pidState)) {
		if(m_bBox->eraseConfig()) {
			saveSettings();
			savePid();
		}
	}
}

void Configurator::loadDefaultConfig() {
	m_config.settings = m_defaultConfig.settings;
}

bool Configurator::loadSettings() {
	Settings_Packet settingsPckt;

	m_bBox->readConfig(0, (uint8_t*)&settingsPckt, sizeof(Settings_Packet));
	if(settingsPckt.Header == CONFIG_HEADER) {
		uint8_t crc = settingsPckt.crc;
		uint8_t crc_eval = checksum((uint8_t*)&settingsPckt, sizeof(Settings_Packet)-1);

		if(crc == crc_eval) {
			m_config.settings = settingsPckt.settings;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
	return true;
}

bool Configurator::loadPid() {
	Pid_Packet pidPckt;

	m_bBox->readConfig(1, (uint8_t*)&pidPckt, sizeof(Pid_Packet));
	if(pidPckt.Header == CONFIG_HEADER) {
		uint8_t crc = pidPckt.crc;
		uint8_t crc_eval = checksum((uint8_t*)&pidPckt, sizeof(Pid_Packet)-1);

		if(crc == crc_eval) {
			m_config.pid = pidPckt.pid;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
	return true;
}

void Configurator::saveSettings() {
	Settings_Packet settingsPckt;
	settingsPckt.settings = m_config.settings;
	uint8_t crc = checksum((uint8_t*)&settingsPckt, sizeof(Settings_Packet)-1);
	settingsPckt.crc = crc;

	m_bBox->writeConfig(0, (uint8_t*)&settingsPckt, sizeof(Settings_Packet));
}

void Configurator::savePid() {
	Pid_Packet pidPckt;
	pidPckt.pid = m_config.pid;
	uint8_t crc = checksum((uint8_t*)&pidPckt, sizeof(Pid_Packet)-1);
	pidPckt.crc = crc;

	m_bBox->writeConfig(1, (uint8_t*)&pidPckt, sizeof(Pid_Packet));
}


void Configurator::handleRxInterrupt(bool reset) {
	static uint8_t remLen = 1;
	static uint16_t payloadSize = 0;
	static uint8_t cmd;

	if(reset) {
		m_state = 0;
	}

	switch(m_state) {
		case 0:
			if(m_buffer[0] == 0xCE) {
				m_state = 1;
				remLen = 1;
			}
			break;
		case 1:
			if(m_buffer[1] == 0xFA) {
				m_state = 2;
				remLen = 1;
			}
			else {
				m_state = 0;
				remLen = 1;
			}
			break;
		case 2:
			if(m_buffer[2] == 0xAD) {
				m_state = 3;
				remLen = 1;
			}
			else {
				m_state = 0;
				remLen = 1;
			}
			break;
		case 3:
			if(m_buffer[3] == 0xDE) {
				m_state = 4;
				remLen = 2;
			}
			else {
				m_state = 0;
				remLen = 1;
			}
			break;
		case 4:
			cmd = m_buffer[4];
			payloadSize = m_buffer[5];
			m_state = 6;
			remLen = payloadSize;
			break;
		case 6:
			uint8_t crc = m_buffer[payloadSize + 5];
			uint8_t crc_eval = checksum(m_buffer, payloadSize + 5);

			if(crc == crc_eval){
				if(cmd == CMD_PID_RESP) {
					Pid* pids = (Pid*)(&m_buffer[6]);
					m_motor->setPIDGains(ROLL, pids->roll.p, pids->roll.i, pids->roll.d, pids->roll.rp);
					m_motor->setPIDGains(PITCH, pids->pitch.p, pids->pitch.i, pids->pitch.d, pids->pitch.rp);
					m_motor->setPIDGains(YAW, pids->yaw.p, pids->yaw.i, pids->yaw.d, pids->yaw.rp);
					m_motor->setPIDGains(PX, pids->px.p, pids->px.i, pids->px.d, pids->px.rp);
					m_motor->setPIDGains(PY, pids->py.p, pids->py.i, pids->py.d, pids->py.rp);
					m_motor->setPIDGains(THROTTLE, pids->alt.p, pids->alt.i, pids->alt.d, pids->alt.rp);

					memcpy((uint8_t*)&(m_config.pid), (uint8_t*)pids, sizeof(Pid));
					m_eraseConfig = true;
				}
				else if(cmd == CMD_CONF_REQ) {
					m_sendConfig = true;
				}
				else if(cmd == CMD_CONF_RESP) {
					memcpy((uint8_t*)&m_config, (uint8_t*)&m_buffer[6], sizeof(Config));
					m_eraseConfig = true;
				}
				else if(cmd == CMD_CONFIG_DEFAULT) {
					loadDefaultConfig();
					m_sendConfig = true;
					m_eraseConfig = true;
				}

				else if(cmd == CMD_RESET) {
//					Application::buzzerOn();
//					TimeTick::delay_ms(300);
//					Application::buzzerOff();
					Buzzer::getInstance()->buzz();
					NVIC_SystemReset();
				}
			}
			m_state = 0;
			remLen = 1;
			payloadSize = 0;
			break;
	}

	HAL_UART_Receive_IT(m_uart, &m_buffer[m_state], remLen);
}

void Configurator::taskFunc(timetick_us currenTimeUs) {
	static bool sending = false;
	static Config_Packet config;

	if(m_eraseConfig) {
		if(m_bBox->eraseConfig()) {
			m_saveNewSettings = true;
			m_saveNewPid = true;
			m_eraseConfig = false;
		}
	}
	else if(m_saveNewSettings) {
		saveSettings();
		m_saveNewSettings = false;
	}
	else if(m_saveNewPid) {
		savePid();
		m_saveNewPid = false;
	}
	else if(m_sendConfig && m_txReady && (sendingState == SendingConfig || sendingState == SendingNone)) {
		if(!sending) {
			sendingState = SendingConfig;
			sending = true;
			memcpy((uint8_t*)&(config.config), (uint8_t*)&m_config, sizeof(Config));
			config.Cmd = CMD_CONF_REQ;
			uint8_t crc = checksum((uint8_t*)&config, sizeof(Config_Packet)-1);
			config.crc = crc;

			m_txReady = false;
			HAL_UART_Transmit_DMA(m_uart, (uint8_t*)&config, sizeof(Config_Packet)/2);
		}
		else {
			m_txReady = false;
			HAL_UART_Transmit_DMA(m_uart, (uint8_t*)&config + sizeof(Config_Packet)/2, sizeof(Config_Packet) - sizeof(Config_Packet)/2);
			m_sendConfig = false;
			sending = false;
			sendingState = SendingNone;
		}
	}
	else if(m_sendNewLog && m_txReady && (sendingState == SendingLog || sendingState == SendingNone)) {
		if(!sending) {
			sendingState = SendingLog;
			sending = true;
			uint8_t crc = checksum((uint8_t*)&logPacket, sizeof(Log_Packet)-1);
			logPacket.crc = crc;

			m_txReady = false;
			HAL_UART_Transmit_DMA(m_uart, (uint8_t*)&logPacket, sizeof(Log_Packet)/2);
		}
		else {
			m_txReady = false;
			HAL_UART_Transmit_DMA(m_uart, (uint8_t*)&logPacket + sizeof(Log_Packet)/2, sizeof(Log_Packet) - sizeof(Log_Packet)/2);
			m_sendConfig = false;
			sending = false;
			sendingState = SendingNone;
			m_canSendLog = true;
		}
	}

}

