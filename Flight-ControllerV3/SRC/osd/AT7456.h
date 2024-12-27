/*
 * AT7456.h
 *
 *  Created on: Jul 19, 2024
 *      Author: Ikenna
 */

#ifndef OSD_AT7456_H_
#define OSD_AT7456_H_

#include <stm32h7xx.h>
#include <spi.h>
#include <typedefs.h>
#include <string>



#define OSD_SPI_BUS_SPEED (2000000L) /*  2 MHz  */

#define DIR_READ(a) ((a) | (1 << 7))
#define DIR_WRITE(a) ((a) & 0x7f)

#define OSD_CHARS_PER_ROW	30
#define OSD_NUM_ROWS_PAL	16
#define OSD_NUM_ROWS_NTSC	13
#define OSD_ZERO_BYTE 0x00
#define OSD_PAL_TX_MODE 0x40

//extern "C" __EXPORT int atxxxx_main(int argc, char *argv[]);

class AT7456
{
public:
	AT7456();
	virtual ~AT7456() = default;

	static void print_usage();

	void init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);

	int add_battery_info(float battVoltage, float battCapacity, uint8_t pos_x = 0, uint8_t pos_y = 0);
	void add_altitude(float altitude, uint8_t pos_x = 0, int8_t pos_y = 0);
	void add_flighttime(float flight_time, uint8_t pos_x = 0, uint8_t pos_y = 0);
	void add_heading(float heading, uint8_t pos_x = 0, uint8_t pos_y = 0);
	void add_gps_info(float lon, float lat, float height, uint8_t pos_x = 0, uint8_t pos_y = 0);

	void RunImpl();

protected:
	bool probe();

private:

	void start();

	void reset();

	void init_osd();

	void readRegister(unsigned reg, uint8_t *data, unsigned count);
	void writeRegister(unsigned reg, uint8_t data);

	void add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y);
	void add_string_to_screen_centered(const char *str, uint8_t pos_y, int max_length);
	void clear_line(uint8_t pos_x, uint8_t pos_y, int length);



	static const char* get_flight_mode(uint8_t nav_state);

	void enable_screen();
	void disable_screen();

	void update_topics();
	void update_screen();

	void _read(uint8_t reg, uint8_t* data, uint16_t len);
	void _write(uint8_t* data, uint16_t len);

	SPI_HandleTypeDef* m_hspi;
	GPIO_TypeDef *m_csPort;
	uint16_t m_csPin;

	// battery
	float _battery_voltage_v{0.f};
	float _battery_discharge_mah{0.f};
	bool _battery_valid{false};

	// altitude
	float _local_position_z{0.f};
	bool _local_position_valid{false};

	// flight time
	uint8_t _arming_state{0};
	uint64_t _arming_timestamp{0};

	// flight mode
	uint8_t _nav_state{0};
};


#endif /* OSD_AT7456_H_ */
