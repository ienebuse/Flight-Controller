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



Attitude view;
float dt = 0;

volatile bool Application::sUartTxReady = true;
bool Application::buzz_state = false;

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
		m_flightControl(&m_ahrs, &m_optflw, &m_rxCh),
		m_blackBox(&m_ahrs, &m_flightControl, &m_meter, &m_gps, &m_barometer, &m_optflw),
		m_log(&m_ahrs, &m_flightControl, &m_meter, &m_gps, &m_barometer, &m_optflw),
		m_configurator(&m_ahrs, &m_flightControl, &m_meter, &m_gps, &m_barometer, &m_optflw, &m_blackBox),
		altFilt(&m_barometer, &m_optflw),
		m_osd(&m_meter)
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

//	__disable_irq();

	m_devices = devices;
	TimeTick::init(m_devices->appTmr);



	SPI_Config blkBxConfig = {
			.hspi = devices->bbxSPI,
			.csPort = BB_CS_GPIO_Port,
			.csPin = BB_CS_Pin
	};

	m_blackBox.init(blkBxConfig);
	m_blackBox.setTaskInfo(BLKBOX_TASK, PRIORITY_MEDIUM, PRIORITY_MEDIUM, 0, BLACKBOX_UPDATEPERIOD_US);
	m_scheduler.addTask(&m_blackBox);

	m_configurator.init(devices->configUart);
	m_configurator.setTaskInfo(CONFIG_TAX, PRIORITY_MEDIUM, PRIORITY_MEDIUM, 0, CONFIG_CHECK_PERIOD_US);
	m_scheduler.addTask(&m_configurator);

	Config config = Configurator::getConfig();

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

	SPI_Config osdConfig = {
			.hspi = devices->osdSPI,
			.csPort = OSD_CS_GPIO_Port,
			.csPin = OSD_CS_Pin
	};


	I2C_config magConfig = {
			.i2cBus = &m_i2cBus,
			.devAddr = Configurator::getConfig().settings.MagDevAddr,
	};

	m_ahrs.init(imuConfig1, imuConfig2, magConfig);
	m_ahrs.setTaskInfo(FUSION_TASK, PRIORITY_REALTIME, PRIORITY_REALTIME, 0, IMU_SAMPLING_PERIOD_US);
	m_ahrs.registerOSD(&m_osd);
	m_scheduler.addTask(&m_ahrs);

	if(config.settings.UseBarometer) {
		m_barometer.init(&m_i2cBus);
		m_barometer.setTaskInfo(BAROMETER_TASK, PRIORITY_MEDIUM, PRIORITY_MEDIUM, 0, BAROMETER_PERIOD_US);
		m_barometer.registerOSD(&m_osd);
		m_scheduler.addTask(&m_barometer);
	}

	m_sbusRx.init(devices->sBus, this);
	m_sbusRx.setTaskInfo(CMD_RX_TASK, PRIORITY_HIGH, PRIORITY_HIGH, 0, RECEIVER_PERIOD_US);
	m_scheduler.addTask(&m_sbusRx);

	m_flightControl.init(devices->motorTmr);
	m_flightControl.setTaskInfo(FLIGHT_CONTROL_TASK, PRIORITY_REALTIME, PRIORITY_REALTIME, 0, FLIGHT_CONTROL_PERIOD_US);
	m_scheduler.addTask(&m_flightControl);

	m_meter.init();
	m_meter.setTaskInfo(METER_TASK, PRIORITY_HIGH, PRIORITY_HIGH, 0, METER_PERIOD_US);
	m_meter.registerOSD(&m_osd);
	m_scheduler.addTask(&m_meter);

	if(config.settings.UseGPS) {
		m_gps.init(devices->gpsUart);
		m_gps.setTaskInfo(GPS_TASK, PRIORITY_LOW, PRIORITY_LOW, 0, GPS_PERIOD_US);
		m_gps.registerOSD(&m_osd);
		m_scheduler.addTask(&m_gps);
	}

	if(config.settings.EnableLogging) {
		m_log.init();
		m_log.setTaskInfo(LOG_TASK, PRIORITY_LOW, PRIORITY_LOW, 0, LOGGER_PERIOD_US);
		m_scheduler.addTask(&m_log);
	}

	m_hrtBt.setTaskInfo(HEARTBEAT_TASK, PRIORITY_MEDIUM, PRIORITY_MEDIUM, 0, HEARTBEAT_PERIOD_US);
	m_scheduler.addTask(&m_hrtBt);

	m_optflw.init(devices->optflwUart, &m_ahrs);
	m_log.setTaskInfo(OPTICAL_FLOW_TASK, PRIORITY_HIGH, PRIORITY_HIGH, 0, OPT_FLW_PERIOD_US);
	m_optflw.registerOSD(&m_osd);
	m_scheduler.addTask(&m_optflw);

	m_buzzer.init();
	m_buzzer.setTaskInfo(BUZZER_TASK, PRIORITY_MEDIUM, PRIORITY_MEDIUM, 0, BUZZER_PERIOD_US);
	m_scheduler.addTask(&m_buzzer);

#if ENABLE_OSD
	m_osd.init(osdConfig);
	m_osd.setTaskInfo(OSD_TASK, PRIORITY_MEDIUM, PRIORITY_MEDIUM, 0, OSD_PERIOD_US);
	m_scheduler.addTask(&m_osd);
#endif

//	__enable_irq();

}

void Application::run() {

	m_sbusRx.startSBus();

	buzzerOn();
	TimeTick::delay_ms(1000);
	buzzerOff();

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

static bool isSwitchStable(int16_t* sw, uint8_t size) {
	int16_t val = sw[0];
	for(uint8_t i = 1; i < size; i++) {
		if(sw[i] != val) {
			return false;
		}
	}
	return true;
}

static float stickAvg(float* sw, uint8_t size) {
	float sum = 0;
	for(uint8_t i = 0; i < size; i++) {
		sum += sw[i];
	}
	return sum/size;
}

void Application::handleChannelData(SbusData sbusData) {
	static timetick_us disarmTimer = 0;
	static bool disarmStarted = false;
	timetick_us currentTime = TimeTick::getTimeUs();
	static const int BUFF_SIZE = 5;

	static float roll[BUFF_SIZE], pitch[BUFF_SIZE], throttle[BUFF_SIZE], yaw[BUFF_SIZE];
	static int16_t sl1[BUFF_SIZE], sl2[BUFF_SIZE], sr1[BUFF_SIZE], sr2[BUFF_SIZE];
	static uint8_t index = 0;

	roll[index] = RPY(sbusData.ch[0]);
	pitch[index] = RPY(sbusData.ch[1]);
	throttle[index] = THROTTLE(sbusData.ch[2]);
	yaw[index] = RPY(sbusData.ch[3]);
	sl1[index] = (int16_t)SW(sbusData.ch[4]);
	sl2[index] = (int16_t)SW(sbusData.ch[5]);
	sr2[index] = (int16_t)SW(sbusData.ch[6]);
	sr1[index] = (int16_t)SW(sbusData.ch[7]);
	index  = (index + 1) % BUFF_SIZE;

	m_rxCh.roll = stickAvg(roll, BUFF_SIZE);
	m_rxCh.pitch = stickAvg(pitch, BUFF_SIZE);
	m_rxCh.throttle = stickAvg(throttle, BUFF_SIZE);
	m_rxCh.yaw = stickAvg(yaw, BUFF_SIZE);
	m_rxCh.SL1 = isSwitchStable(sl1, BUFF_SIZE) ? sl1[0] : m_rxCh.SL1;
	m_rxCh.SL2 = isSwitchStable(sl2, BUFF_SIZE) ? sl2[0] : m_rxCh.SL2;
	m_rxCh.SR2 = isSwitchStable(sr2, BUFF_SIZE) ? sr2[0] : m_rxCh.SR2;
	m_rxCh.SR1 = isSwitchStable(sr1, BUFF_SIZE) ? sr1[0] : m_rxCh.SR1;

	if(m_rxCh.throttle < 2 && m_rxCh.yaw < 2 && m_rxCh.roll > 98 && m_rxCh.pitch < 2) {
		if(m_rxCh.SL2 < 40) {
			m_flightControl.armMotors(true);
			disarmStarted = false;
			disarmTimer = 0;
		}
		else {
			NVIC_SystemReset();
		}
	}
	else if(m_rxCh.SL2 > 40) {
		if(disarmStarted && (currentTime - disarmTimer > 400000)) {
			disarmStarted = false;
			disarmTimer = 0;
		}
		else if(disarmStarted && (currentTime - disarmTimer > 300000)) {
			m_flightControl.armMotors(false);
			disarmStarted = false;
			disarmTimer = 0;
		}
		else if(!disarmStarted) {
			disarmStarted = true;
			disarmTimer = currentTime;
		}


	}

	checkPIDRequest(m_rxCh.SR1);
	checkBlackBloxErase(m_rxCh.SL1);
	checkBlackBloxRead(m_rxCh.SL1);
}

//void Application::handleChannelData(SbusData sbusData) {
//	static timetick_us disarmTimer = 0;
//	static bool disarmStarted = false;
//	timetick_us currentTime = TimeTick::getTimeUs();
//
//	m_rxCh.roll = RPY(sbusData.ch[0]);
//	m_rxCh.pitch = RPY(sbusData.ch[1]);
//	m_rxCh.throttle = THROTTLE(sbusData.ch[2]);
//	m_rxCh.yaw = RPY(sbusData.ch[3]);
//	m_rxCh.SL1 = SW(sbusData.ch[4]);
//	m_rxCh.SL2 = SW(sbusData.ch[5]);
//	m_rxCh.SR2 = SW(sbusData.ch[6]);
//	m_rxCh.SR1 = SW(sbusData.ch[7]);
//
//	if(m_rxCh.throttle < 2 && m_rxCh.yaw < 2 && m_rxCh.roll > 98 && m_rxCh.pitch < 2) {
//		if(m_rxCh.SL2 < 40) {
//			m_flightControl.armMotors(true);
//			disarmStarted = false;
//			disarmTimer = 0;
//		}
//		else {
//			NVIC_SystemReset();
//		}
//	}
//	else if(m_rxCh.SL2 > 40) {
//		if(disarmStarted && (currentTime - disarmTimer > 400000)) {
//			disarmStarted = false;
//			disarmTimer = 0;
//		}
//		else if(disarmStarted && (currentTime - disarmTimer > 300000)) {
//			m_flightControl.armMotors(false);
//			disarmStarted = false;
//			disarmTimer = 0;
//		}
//		else if(!disarmStarted) {
//			disarmStarted = true;
//			disarmTimer = currentTime;
//		}
//
//
//	}
//
//	checkPIDRequest(m_rxCh.SR1);
//	checkBlackBloxErase(m_rxCh.SL1);
//	checkBlackBloxRead(m_rxCh.SL1);
//}


