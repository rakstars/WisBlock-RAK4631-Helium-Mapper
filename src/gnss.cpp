/**
 * @file gnss.cpp
 * @author Johan Sebastian Macias (johan.macias@rakwireless.com)
 * @brief GNSS functions and task
 * @version 0.1
 * @date 2020-07-24
 * 
 * Based on: GNSS functions and task
 * https://github.com/beegee-tokyo/RAK4631-Kit-2-RAK1910-RAK1904-RAK1906/blob/main/src/gnss.cpp
 * By: Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * Date: 2020-07-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "main.h"

// The GNSS object
TinyGPSPlus my_gnss;

/** GNSS polling function */
bool poll_gnss(void);

/** Location data as byte array */
mapper_data_s g_mapper_data;

/** Latitude/Longitude value converter */
latLong_s pos_union;

/** Flag if location was found */
bool last_read_ok = false;

/**
 * @brief Initialize the GNSS
 * 
 */
bool init_gnss(void)
{
	bool gnss_found = false;
	// // Give the module some time to power up
	// delay(2000);

	// Power on the GNSS module
	// pinMode(WB_IO2, OUTPUT);
	// delay(100);
	digitalWrite(WB_IO2, HIGH);

	// Give the module some time to power up
	delay(500);

	Serial1.begin(9600);
	while (!Serial1);

	gnss_found = true;

	return gnss_found;
}

/**
 * @brief Check GNSS module for position
 * 
 * @return true Valid position found
 * @return false No valid position
 */
bool poll_gnss(void)
{
	time_t time_out = millis();
	bool has_pos = false;
	int64_t latitude = 0;
	int64_t longitude = 0;
	int32_t altitude = 0;

	bool hasAlt = false;

	MYLOG("GNSS", "Polling RAK1910");
	if (ble_uart_is_connected) {
		ble_uart.print("Polling RAK1910\n");
	}

	while ((millis() - time_out) < 10000) {

		while (Serial1.available() > 0)
		{
			// if (my_gnss.encode(ss.read()))
			if (my_gnss.encode(Serial1.read()))
			{
				digitalToggle(LED_BUILTIN);
				if (my_gnss.location.isUpdated() && my_gnss.location.isValid())
				{
					has_pos = true;
					latitude = my_gnss.location.lat() * 100000;
					longitude = my_gnss.location.lng() * 100000;
				}
				else if (my_gnss.altitude.isUpdated() && my_gnss.altitude.isValid())
				{
					hasAlt = true;
					altitude = my_gnss.altitude.meters();
				}
			}
			// if (has_pos && hasAlt)
			if (has_pos && hasAlt)
			{
				break;
			}
		}
		if (has_pos && hasAlt)
		{
			break;
		}
	}

	if (has_pos)
	{
		MYLOG("GNSS", "Lat: %.4f Lon: %.4f", latitude / 10000000.0, longitude / 10000000.0);
		MYLOG("GNSS", "Alt: %.2f", altitude / 1000.0);

		if (ble_uart_is_connected)
		{
			ble_uart.printf("Lat: %.4f Lon: %.4f\n", latitude / 100000.0, longitude / 100000.0);
			ble_uart.printf("Alt: %.2f\n", altitude / 1000.0);
		}
		pos_union.val32 = latitude;
		g_mapper_data.lat_1 = pos_union.val8[0];
		g_mapper_data.lat_2 = pos_union.val8[1];
		g_mapper_data.lat_3 = pos_union.val8[2];
		g_mapper_data.lat_4 = pos_union.val8[3];

		pos_union.val32 = longitude;
		g_mapper_data.long_1 = pos_union.val8[0];
		g_mapper_data.long_2 = pos_union.val8[1];
		g_mapper_data.long_3 = pos_union.val8[2];
		g_mapper_data.long_4 = pos_union.val8[3];

		pos_union.val32 = altitude;
		g_mapper_data.alt_1 = pos_union.val8[0];
		g_mapper_data.alt_2 = pos_union.val8[1];

	}
	else
	{
		delay(1000);
	}

	if (has_pos)
	{
		// my_gnss.powerSaveMode(true);
		// my_gnss.setMeasurementRate(10000);
		return true;
	}

	MYLOG("GNSS", "No valid location found");
	if (ble_uart_is_connected)
	{
		ble_uart.print("No valid location found\n");
	}
	last_read_ok = false;
	// my_gnss.setMeasurementRate(1000);
	return false;
}