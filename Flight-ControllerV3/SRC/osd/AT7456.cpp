/*
 * AT7456.cpp
 *
 *  Created on: Jul 19, 2024
 *      Author: Ikenna
 */

#include <osd/AT7456.h>
#include <string>
#include <string.h>
#include "symbols.h"

//using namespace time_literals;

static constexpr uint32_t OSD_UPDATE_RATE{50000};	// 20 Hz
static uint8_t NUM_ROWS;

AT7456::AT7456()
{
}


void AT7456::init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
	m_hspi = hspi;
	m_csPort = csPort;
	m_csPin = csPin;

	reset();

	init_osd();

	// clear the screen
	NUM_ROWS = (OSD_USE_NTSC_PAL == 1 ? OSD_NUM_ROWS_NTSC : OSD_NUM_ROWS_PAL);

	for (int i = 0; i < OSD_CHARS_PER_ROW; i++) {
		for (int j = 0; j < NUM_ROWS; j++) {
			add_character_to_screen(' ', i, j);
		}
	}
}

void AT7456::start()
{
//	ScheduleOnInterval(OSD_UPDATE_RATE, 10000);
}

bool AT7456::probe()
{
	uint8_t data = 0;

	writeRegister(0x00, 0x01); //disable video output
	readRegister(0x00, &data, 1);

	if (data != 1) {
		return false;;
	}

	return true;
}

void AT7456::init_osd()
{
	uint8_t data = OSD_ZERO_BYTE;

	if (OSD_USE_NTSC_PAL == 2) {
		data |= OSD_PAL_TX_MODE;
	}

	writeRegister(0x00, data);
	writeRegister(0x04, OSD_ZERO_BYTE);

	enable_screen();
}

void AT7456::readRegister(unsigned reg, uint8_t *data, unsigned count)
{
//	uint8_t cmd[5] {}; // read up to 4 bytes

	uint8_t cmd = DIR_READ(reg);

//	_read(cmd[0], &cmd[1], count + 1);
	_read(cmd, data, count);

//	memcpy(&data[0], &cmd[1], count);
}

void AT7456::writeRegister(unsigned reg, uint8_t data)
{
	uint8_t cmd[2] {}; // write 1 byte

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;

	_write(&cmd[0], 2);
}

void AT7456::add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y)
{
	uint16_t position = (OSD_CHARS_PER_ROW * pos_y) + pos_x;
	uint8_t position_lsb = 0;

	if (position > 0xFF) {
		position_lsb = static_cast<uint8_t>(position) - 0xFF;
		writeRegister(0x05, 0x01); //DMAH

	} else {
		position_lsb = static_cast<uint8_t>(position);
		writeRegister(0x05, 0x00); //DMAH
	}

	writeRegister(0x06, position_lsb); //DMAL

	writeRegister(0x07, c);
}

void AT7456::add_string_to_screen_centered(const char *str, uint8_t pos_y, int max_length)
{
	int len = strlen(str);

	if (len > max_length) {
		len = max_length;
	}

	int pos = (OSD_CHARS_PER_ROW - max_length) / 2;
	int before = (max_length - len) / 2;

	for (int i = 0; i < before; ++i) {
		add_character_to_screen(' ', pos++, pos_y);
	}

	for (int i = 0; i < len; ++i) {
		add_character_to_screen(str[i], pos++, pos_y);
	}

	while (pos < (OSD_CHARS_PER_ROW + max_length) / 2) {
		add_character_to_screen(' ', pos++, pos_y);
	}
}

void AT7456::clear_line(uint8_t pos_x, uint8_t pos_y, int length)
{
	for (int i = 0; i < length; ++i) {
		add_character_to_screen(' ', pos_x + i, pos_y);
	}
}

int AT7456::add_battery_info(float battVoltage, float battCapacity, uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];

	// TODO: show battery symbol based on battery fill level
	snprintf(buf, sizeof(buf), "%c%5.2f", OSD_SYMBOL_BATT_3, (double)battVoltage);
	buf[sizeof(buf) - 1] = '\0';

	pos_x = OSD_CHARS_PER_ROW - strlen(buf) - 1;

	for (int i = 0; buf[i] != '\0'; i++) {
		add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	add_character_to_screen('V', pos_x + 5, pos_y);

	pos_y++;
	pos_x++;

	snprintf(buf, sizeof(buf), "%5d", (int)battCapacity);
	buf[sizeof(buf) - 1] = '\0';

	pos_x = OSD_CHARS_PER_ROW - strlen(buf) - 2;

	for (int i = 0; buf[i] != '\0'; i++) {
		add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	add_character_to_screen(OSD_SYMBOL_MAH, pos_x + 5, pos_y);
}

void AT7456::add_altitude(float altitude, uint8_t pos_x, int8_t pos_y)
{
	char buf[16];

	if(pos_y < 0) {
		pos_y = NUM_ROWS + pos_y;
	}

	snprintf(buf, sizeof(buf), "%c%10.2f%c", OSD_SYMBOL_ARROW_NORTH, (double)altitude, OSD_SYMBOL_M);
	buf[sizeof(buf) - 1] = '\0';

	pos_x = OSD_CHARS_PER_ROW - strlen(buf) - 2;

	for (int i = 0; buf[i] != '\0'; i++) {
		add_character_to_screen(buf[i], pos_x + i, pos_y);
	}
}

void AT7456::add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];

	snprintf(buf, sizeof(buf), "%c%5.1f", OSD_SYMBOL_FLIGHT_TIME, (double)flight_time);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		add_character_to_screen(buf[i], pos_x + i, pos_y);
	}
}

void AT7456::add_heading(float heading, uint8_t pos_x, uint8_t pos_y) {
	char buf[10];

	snprintf(buf, sizeof(buf), "%c%5.1f", OSD_SYMBOL_FLIGHT_TIME, (double)heading);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		add_character_to_screen(buf[i], pos_x + i, pos_y);
	}
}

void AT7456::add_gps_info(float lon, float lat, float height, uint8_t pos_x, uint8_t pos_y) {
	char buf[20];

	add_character_to_screen(OSD_SYMBOL_SAT_L, 0, NUM_ROWS - 4);
	add_character_to_screen(OSD_SYMBOL_SAT_R, 1, NUM_ROWS - 4);

	snprintf(buf, sizeof(buf), "%5.3f", (double)lon);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		add_character_to_screen(buf[i], pos_x + i, NUM_ROWS - 3);
	}

	snprintf(buf, sizeof(buf), "%5.3f", (double)lat);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		add_character_to_screen(buf[i], pos_x + i, NUM_ROWS - 2);
	}

	snprintf(buf, sizeof(buf), "%5.1f", (double)height);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		add_character_to_screen(buf[i], pos_x + i, NUM_ROWS - 1);
	}
}

void AT7456::enable_screen()
{
	uint8_t data = 0;

	readRegister(0x00, &data, 1);
	writeRegister(0x00, data | 0x48);
}

void AT7456::disable_screen()
{
	uint8_t data = 0;

	readRegister(0x00, &data, 1);
	writeRegister(0x00, data & 0xF7);
}

void AT7456::update_topics()
{
//	/* update battery subscription */
//	if (_battery_sub.updated()) {
//		battery_status_s battery{};
//		_battery_sub.copy(&battery);
//
//		if (battery.connected) {
//			_battery_voltage_v = battery.voltage_v;
//			_battery_discharge_mah = battery.discharged_mah;
//			_battery_valid = true;
//
//		} else {
//			_battery_valid = false;
//		}
//	}
//
//	/* update vehicle local position subscription */
//	if (_local_position_sub.updated()) {
//		vehicle_local_position_s local_position{};
//		_local_position_sub.copy(&local_position);
//
//		_local_position_valid = local_position.z_valid;
//
//		if (_local_position_valid) {
//			_local_position_z = -local_position.z;
//		}
//	}
//
//	/* update vehicle status subscription */
//	if (_vehicle_status_sub.updated()) {
//		vehicle_status_s vehicle_status{};
//		_vehicle_status_sub.copy(&vehicle_status);
//
//		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
//		    _arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
//			// arming
//			_arming_timestamp = hrt_absolute_time();
//
//		} else if (vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED &&
//			   _arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
//			// disarming
//		}
//
//		_arming_state = vehicle_status.arming_state;
//		_nav_state = vehicle_status.nav_state;
//	}

}

const char* AT7456::get_flight_mode(uint8_t nav_state)
{
	const char *flight_mode = "UNKNOWN";
//
//	switch (nav_state) {
//	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
//		flight_mode = "MANUAL";
//		break;
//
//	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
//		flight_mode = "ALTITUDE";
//		break;
//
//	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
//		flight_mode = "POSITION";
//		break;
//
//	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
//		flight_mode = "RETURN";
//		break;
//
//	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
//		flight_mode = "MISSION";
//		break;
//
//	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
//	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
//	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
//	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
//	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
//	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
//		flight_mode = "AUTO";
//		break;
//
//	case vehicle_status_s::NAVIGATION_STATE_ACRO:
//		flight_mode = "ACRO";
//		break;
//
//	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
//		flight_mode = "TERMINATE";
//		break;
//
//	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
//		flight_mode = "OFFBOARD";
//		break;
//
//	case vehicle_status_s::NAVIGATION_STATE_STAB:
//		flight_mode = "STABILIZED";
//		break;
//	}

	return flight_mode;
}

void AT7456::update_screen()
{
	if (_battery_valid) {
//		add_battery_info(1, 1);

	} else {
		clear_line(1, 1, 10);
		clear_line(1, 2, 10);
	}

	if (_local_position_valid) {
//		add_altitude(1, 3);

	} else {
		clear_line(1, 3, 10);
	}

	const char *flight_mode = "";

//	if (_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
//		float flight_time_sec = static_cast<float>((hrt_absolute_time() - _arming_timestamp) / (1e6f));
//		add_flighttime(flight_time_sec, 1, 14);
//
//	} else {
//		flight_mode = get_flight_mode(_nav_state);
//	}

	add_string_to_screen_centered(flight_mode, 12, 10);
}

void AT7456::reset()
{
	writeRegister(0x00, 0x02);
	HAL_Delay(1);
}

void AT7456::RunImpl()
{
//	if (should_exit()) {
//		exit_and_cleanup();
//		return;
//	}

	update_topics();

	update_screen();
}

void
AT7456::print_usage()
{
//	PRINT_MODULE_DESCRIPTION(
//		R"DESCR_STR(
//### Description
//OSD driver for the ATXXXX chip that is mounted on the OmnibusF4SD board for example.
//
//It can be enabled with the OSD_ATXXXX_CFG parameter.
//)DESCR_STR");
//
//	PRINT_MODULE_USAGE_NAME("atxxxx", "driver");
//	PRINT_MODULE_USAGE_COMMAND("start");
//	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
//	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

//int
//atxxxx_main(int argc, char *argv[])
//{
//	using ThisDriver = AT7456;
//	BusCLIArguments cli{false, true};
//	cli.spi_mode = SPIDEV_MODE0;
//	cli.default_spi_frequency = OSD_SPI_BUS_SPEED;
//
//	const char *verb = cli.parseDefaultArguments(argc, argv);
//
//	if (!verb) {
//		ThisDriver::print_usage();
//		return -1;
//	}
//
//	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_OSD_DEVTYPE_ATXXXX);
//
//	if (!strcmp(verb, "start")) {
//		return ThisDriver::module_start(cli, iterator);
//	}
//
//	if (!strcmp(verb, "stop")) {
//		return ThisDriver::module_stop(iterator);
//	}
//
//	if (!strcmp(verb, "status")) {
//		return ThisDriver::module_status(iterator);
//	}
//
//	ThisDriver::print_usage();
//	return -1;
//}

void AT7456::_read(uint8_t reg, uint8_t* data, uint16_t len) {
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(m_hspi,&reg,1,1);
	HAL_SPI_Receive(m_hspi,data,len,10);
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}
void AT7456::_write(uint8_t* data, uint16_t len) {
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(m_hspi,data,len,10);
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}
