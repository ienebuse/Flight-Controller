/*
 * Application.cpp
 *
 *  Created on: Jun 29, 2024
 *      Author: Ikenna
 */

#include "Application.h"
#include "string.h"
#include <stdio.h>
#include <TimeTick.h>
#include <DShot.h>
#include <Task.h>

//#include <usbd_cdc_if.h>
//#include <string.h>
//#include <stdio.h>

#define THROTTLE_MAX		192.0f
#define THROTTLE_MIN		1792.0f
#define THROTTLE_RANGE 		THROTTLE_MAX - THROTTLE_MIN

#define SW_MIN		192.0f
#define SW_MAX		1792.0f
#define SW_RANGE 		SW_MAX - SW_MIN


#define RPY_MAX_ANGLE		45.0f
#define RPY_MIN_ANGLE		-45.0f
#define RPY_MAX				1552.0f
#define RPY_MIN				432.0f
#define RPY_RANGE 			RPY_MAX - RPY_MIN

#define R_MAX_ANGLE			45.0f
#define R_MIN_ANGLE			-45.0f
#define R_MIN				1552.0f
#define R_MAX				432.0f
#define R_RANGE 			RPY_MAX - RPY_MIN



#define THROTTLE(t) ((t-THROTTLE_MIN)*100/(THROTTLE_RANGE))
#define RPY(rpy) ((rpy-RPY_MIN)*100/(RPY_RANGE))	//(((rpy-RPY_MIN)*(RPY_MAX_ANGLE-RPY_MIN_ANGLE)/(RPY_RANGE)) + RPY_MIN_ANGLE)
#define RTF(r) ((r-R_MIN)*100/(R_RANGE))
#define SW(t) ((t-SW_MIN)*100/(SW_RANGE))

Attitude view;
float dt = 0;

volatile bool Application::sUartTxReady = true;

//typedef struct {
//	volatile float roll = 0;
//	volatile float pitch = 0;
//	volatile float throttle = 0;
//	volatile float yaw = 0;
//	volatile int16_t SL1 = 0;
//	volatile int16_t SL2 = 2000;
//	volatile int16_t SR2 = 0;
//	volatile int16_t SR1 = 0;
//}Channel;



Application::Application() :
		m_motorControl(&m_ahrs, &m_optflw, &m_rxCh),
		m_blackBox(&m_ahrs, &m_motorControl, &m_meter, &m_gps, &m_barometer, &m_optflw),
		m_log(&m_ahrs, &m_motorControl, &m_meter, &m_gps, &m_barometer, &m_optflw)
{
	// TODO Auto-generated constructor stub

}

Application::~Application() {
	// TODO Auto-generated destructor stub
}

//uint32_t Application::getTickUs() {
//	return __HAL_TIM_GET_COUNTER(m_devices->appTmr);
//}


static float getIncreament(float scale, float input) {
	return 2*(input - 50) * 0.001 * scale;
}

//FUSION_TASK,
//MOTOR_CONTROL_TASK,
//MAG_TASK,
//GPS_TASK,
//BAROMETER_TASK,
//OPTICAL_FLOW_TASK,
//CMD_RX_TASK,
//TELEMETRY_TASK,
//BB_TASK,
//OSD_TASK,
//NUM_TASK,

//uint8_t taskId;
//uint8_t staticPriority;
//uint8_t priority;
//uint8_t eventPriority;
//timetick_us period;
//timetick_us lastRunTime;
//timetick_us nextRuntTime;
//bool newEvent = false;

void Application::init(HAL_Devices_t *devices) {

	m_devices = devices;
	TimeTick::init(m_devices->appTmr);
//	HAL_TIM_Base_Start_IT(m_devices->appTmr);

	m_i2cBus.init(devices->i2cBus);

	SPI_Config imuConfig1 = {
			.hspi = devices->imu2SPI,
			.csPort = IMU2_CS_GPIO_Port,
			.csPin = IMU2_CS_Pin,
			.SPI_HS_CLK = 24000000
	};

	SPI_Config imuConfig2 = {
			.hspi = devices->imu1SPI,
			.csPort = IMU1_CS_GPIO_Port,
			.csPin = IMU1_CS_Pin,
			.SPI_HS_CLK = 24000000
	};

	I2C_config magConfig = {
			.i2cBus = &m_i2cBus,
			.devAddr = MAG_DEV_ADDR,
	};

	SPI_Config blkBxConfig = {
			.hspi = devices->bbxSPI,
			.csPort = BB_CS_GPIO_Port,
			.csPin = BB_CS_Pin
	};

	m_blackBox.init(devices->logUart, blkBxConfig);
	m_blackBox.setTaskInfo(BLKBOX_TASK, PRIORITY_MEDIUM, PRIORITY_MEDIUM, 0, 5000);
	m_scheduler.addTask(&m_blackBox);

	m_ahrs.init(imuConfig1, imuConfig2, magConfig);
	m_ahrs.setTaskInfo(FUSION_TASK, PRIORITY_REALTIME, PRIORITY_REALTIME, 0, IMU_SAMPLING_PERIOD_US);
	m_scheduler.addTask(&m_ahrs);

	int i = 0;
	uint8_t resp;

#if (USE_BAROMETER == 1)
	m_barometer.init(&m_i2cBus);
	m_barometer.setTaskInfo(BAROMETER_TASK, PRIORITY_MEDIUM, PRIORITY_MEDIUM, 0, 500000);
	m_scheduler.addTask(&m_barometer);
#endif

	m_sbusRx.init(devices->sBus, this);
	m_sbusRx.setTaskInfo(CMD_RX_TASK, PRIORITY_HIGH, PRIORITY_HIGH, 0, 20000);
	m_scheduler.addTask(&m_sbusRx);

	m_motorControl.init(devices->motorTmr);
	m_motorControl.setTaskInfo(MOTOR_CONTROL_TASK, PRIORITY_REALTIME, PRIORITY_REALTIME, 0, FLIGHT_CONTROL_PERIOD_US);
	m_scheduler.addTask(&m_motorControl);

	m_meter.init();
	m_meter.setTaskInfo(METER_TASK, PRIORITY_HIGH, PRIORITY_HIGH, 0, 1000000);
	m_scheduler.addTask(&m_meter);

#if (USE_GPS == 1)
	m_gps.init(devices->gpsUart);
	m_log.setTaskInfo(GPS_TASK, PRIORITY_LOW, PRIORITY_LOW, 0, 1000000);
	m_scheduler.addTask(&m_gps);
#endif

#if (ENABLE_DEBUG == 1)
	m_log.init(devices->logUart);
	m_log.setTaskInfo(LOG_TASK, PRIORITY_LOW, PRIORITY_LOW, 0, 30000);
	m_scheduler.addTask(&m_log);
#endif

	m_hrtBt.setTaskInfo(HEARTBEAT_TASK, PRIORITY_MEDIUM, PRIORITY_MEDIUM, 0, 500000);
	m_scheduler.addTask(&m_hrtBt);

	m_optflw.init(devices->optflwUart, &m_ahrs);
	m_log.setTaskInfo(OPTICAL_FLOW_TASK, PRIORITY_HIGH, PRIORITY_HIGH, 0, 20000);
	m_scheduler.addTask(&m_optflw);

}

void Application::run() {

	m_sbusRx.startSBus();

	TimeTick::delay_ms(2000);
//	batteryVoltage = m_meter.getBatteryVoltage();

	while (1) {
		m_scheduler.run();
	}
}

void Application::checkPIDRequest(uint16_t cmd) {
	static timetick_us lastTime;
	static uint16_t lastCmd = 0;
	static uint8_t state = 0;



	timetick_us currentTime = TimeTick::getTimeUs();
	if(currentTime-lastTime > 500000) {
		state = 0;
		lastCmd = cmd;
		lastTime = currentTime;
		return;
	}

	if(cmd == lastCmd) return;

	switch(state) {
	case 0:
		state = (cmd < 40) ? 1 : 0;
		break;
	case 1:
		state = (cmd > 40 && cmd < 60) ? 2 : 0;
		break;
	case 2:
		state = (cmd > 60) ? 3 : 0;
		break;
	case 3:
		state = (cmd > 40 && cmd < 60) ? 4 : 0;
		break;
	case 4:
		state = (cmd < 40) ? 5 : 0;
		break;
	case 5:
		state = (cmd > 40 && cmd < 60) ? 6 : 0;
		break;
	case 6:
		state = (cmd > 60) ? 7 : 0;
		break;
	case 7:
		state = (cmd > 40 && cmd < 60) ? 8 : 0;
		break;
	case 8:
		state = (cmd < 40) ? 9 : 0;
		break;
	case 9:
		if(cmd > 40 && cmd < 60){
			m_PidMode = true;
			state = 10;
		}
		else {
			state = 0;
		}
		break;
	case 10:
		state = (cmd > 60) ? 11 : 0;
		break;
	case 11:
		state = (cmd > 40 && cmd < 60) ? 12 : 0;
		break;
	case 12:
		state = (cmd < 40) ? 13 : 0;
		break;
	case 13:
		if(cmd > 40 && cmd < 60){
			m_PidMode = false;
		}
		state = 0;
		break;
	}
	lastCmd = cmd;
	lastTime = currentTime;
}

void Application::checkBlackBloxErase(uint16_t cmd) {
	static timetick_us lastTime;
	static uint16_t lastCmd = 0;
	static uint8_t state = 0;



	timetick_us currentTime = TimeTick::getTimeUs();
	if(currentTime-lastTime > 500000) {
		state = 0;
		lastCmd = cmd;
		lastTime = currentTime;
		return;
	}

	if(cmd == lastCmd) return;

	switch(state) {
	case 0:
		state = (cmd < 40) ? 1 : 0;
		break;
	case 1:
		state = (cmd > 40 && cmd < 60) ? 2 : 0;
		break;
	case 2:
		state = (cmd > 60) ? 3 : 0;
		break;
	case 3:
		state = (cmd > 40 && cmd < 60) ? 4 : 0;
		break;
	case 4:
		state = (cmd < 40) ? 5 : 0;
		break;
	case 5:
		state = (cmd > 40 && cmd < 60) ? 6 : 0;
		break;
	case 6:
		state = (cmd > 60) ? 7 : 0;
		break;
	case 7:
		state = (cmd > 40 && cmd < 60) ? 8 : 0;
		break;
	case 8:
		state = (cmd < 40) ? 9 : 0;
		break;
	case 9:
		if(cmd > 40 && cmd < 60){
			m_blackBox.eraseData();
			state = 10;
		}
		else {
			state = 0;
		}
		break;
	case 10:
		state = (cmd > 60) ? 11 : 0;
		break;
	case 11:
		state = (cmd > 40 && cmd < 60) ? 12 : 0;
		break;
	case 12:
		state = (cmd < 40) ? 13 : 0;
		break;
	case 13:
		if(cmd > 40 && cmd < 60){
			m_blackBox.fullErase();
		}
		state = 0;
		break;
	}
	lastCmd = cmd;
	lastTime = currentTime;
}

void Application::checkBlackBloxRead(uint16_t cmd) {
	static timetick_us lastTime;
	static uint16_t lastCmd = 0;
	static uint8_t state = 0;



	timetick_us currentTime = TimeTick::getTimeUs();
	if(currentTime-lastTime > 500000) {
		state = 0;
		lastCmd = cmd;
		lastTime = currentTime;
		return;
	}

	if(cmd == lastCmd) return;

	switch(state) {
	case 0:
		state = (cmd < 40) ? 1 : 0;
		break;
	case 1:
		state = (cmd > 40 && cmd < 60) ? 2 : 0;
		break;
	case 2:
		state = (cmd < 40) ? 3 : 0;
		break;
	case 3:
		state = (cmd > 40 && cmd < 60) ? 4 : 0;
		break;
	case 4:
		state = (cmd > 60) ? 5 : 0;
		break;
	case 5:
		state = (cmd > 40 && cmd < 60) ? 6 : 0;
		break;
	case 6:
		state = (cmd > 60) ? 7 : 0;
		break;
	case 7:
		if(cmd > 40 && cmd < 60){
			m_blackBox.setBlkBoxReq();
			state = 8;
		}
		else {
			state = 0;
		}
		break;
	case 8:
		state = (cmd < 40) ? 9 : 0;
		break;
	case 9:
		if(cmd > 40 && cmd < 60){
//			m_blackBox.eraseData();
			state = 10;
		}
		else {
			state = 0;
		}
		break;
	case 10:
		state = (cmd > 60) ? 11 : 0;
		break;
	case 11:
		state = (cmd > 40 && cmd < 60) ? 12 : 0;
		break;
	case 12:
		state = (cmd < 40) ? 13 : 0;
		break;
	case 13:
		if(cmd > 40 && cmd < 60){
//			m_blackBox.fullErase();
		}
		state = 0;
		break;
	}
	lastCmd = cmd;
	lastTime = currentTime;
}

void Application::handleChannelData(SbusData sbusData) {
	m_rxCh.roll = RPY(sbusData.ch[0]);
	m_rxCh.pitch = RPY(sbusData.ch[1]);
	m_rxCh.throttle = THROTTLE(sbusData.ch[2]);
	m_rxCh.yaw = RPY(sbusData.ch[3]);
	m_rxCh.SL1 = SW(sbusData.ch[4]);
	m_rxCh.SL2 = SW(sbusData.ch[5]);
	m_rxCh.SR2 = SW(sbusData.ch[6]);
	m_rxCh.SR1 = SW(sbusData.ch[7]);

	if(m_rxCh.SL2 < 40 && m_rxCh.throttle < 2 && m_rxCh.yaw < 2 && m_rxCh.roll > 98 && m_rxCh.pitch < 2) {
		m_motorControl.armMotors(true);
	}
	else if(m_rxCh.SL2 > 40) {
		m_motorControl.armMotors(false);
	}

	checkPIDRequest(m_rxCh.SR1);
	checkBlackBloxErase(m_rxCh.SL1);
	checkBlackBloxRead(m_rxCh.SL1);
}


