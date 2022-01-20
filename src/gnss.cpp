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
#include "app.h"

// The GNSS object
TinyGPSPlus my_rak1910_gnss;	 // RAK1910_GNSS
SFE_UBLOX_GNSS my_rak12500_gnss; // RAK12500_GNSS

/** GNSS polling function */
bool poll_gnss(void);

/** Location data as byte array */
mapper_data_s g_mapper_data;

/** Latitude/Longitude value converter */
latLong_s pos_union;

/** Flag if location was found */
bool last_read_ok = false;

/**
 * @brief Detect and initialize a connected GNSS module. Supports RAK12500 and RAK1910.
 * 
 * @return RAK1910 or RAK12500 (uint8_t)
 */
uint8_t init_gnss(void)
{
	// Give the module some time to power up
	// delay(2000);

	// Power on the GNSS module
	// pinMode(WB_IO2, OUTPUT);
	// delay(100);
	digitalWrite(WB_IO2, HIGH);

	// Give the module some time to power up
	delay(500);

	// Initialize RAK12500 if present, otherwise initialize RAK1910
	Wire.begin();
	bool rak12500_present = my_rak12500_gnss.begin();

	MYLOG("GNSS", "Trying to initialize RAK12500");

	if (rak12500_present)
	{
		my_rak12500_gnss.setI2COutput(COM_TYPE_UBX);				 // Set the I2C port to output UBX only (turn off NMEA noise)
		my_rak12500_gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to flash and BBR
		MYLOG("GNSS", "Detected and initialized RAK12500");
		return RAK12500_GNSS;
	}
	else
	{
		MYLOG("GNSS", "RAK12500 not detected at default I2C address");
		Wire.end();

		MYLOG("GNSS", "Trying to initialize RAK1910");
		Serial1.begin(9600);
		while (!Serial1)
			;
		MYLOG("GNSS", "Initialized RAK1910");
		return RAK1910_GNSS;
	}
}

/**
 * @brief Check GNSS module for position
 * 
 * @return Is valid position found (bool)
 */
bool poll_gnss(uint8_t gnss_option)
{
	time_t time_out = millis();
	bool has_pos = false;
	int64_t latitude = 0;
	int64_t longitude = 0;
	int32_t altitude = 0;
	int32_t accuracy = 0;
	uint32_t polling_seconds;
	uint32_t polling_miliseconds;

	bool has_alt = false;

	digitalWrite(LED_BUILTIN, HIGH);

	// Poll the GPS according to the initialized module
	switch (gnss_option)
	{
	case RAK1910_GNSS:
		MYLOG("GNSS", "Polling RAK1910");
		if (g_ble_uart_is_connected)
		{
			g_ble_uart.print("Polling RAK1910\n");
		}

		while ((millis() - time_out) < 10000)
		{
			polling_miliseconds = millis() - time_out;

			if ( (polling_miliseconds % 1000 == 0) && (polling_miliseconds / 1000 != polling_seconds)) 
			{
				polling_seconds = polling_miliseconds / 1000;
				MYLOG("GNSS", "Polling elapsed time: %d s", polling_seconds);
				if (g_ble_uart_is_connected)
				{
					g_ble_uart.printf("Polling elapsed time: %d s", polling_seconds);
				}
			} 

			while (Serial1.available() > 0)
			{
				// if (my_rak1910_gnss.encode(ss.read()))
				if (my_rak1910_gnss.encode(Serial1.read()))
				{
					digitalToggle(LED_BUILTIN);
					if (my_rak1910_gnss.location.isUpdated() && my_rak1910_gnss.location.isValid())
					{
						has_pos = true;
						latitude = my_rak1910_gnss.location.lat() * 100000;
						longitude = my_rak1910_gnss.location.lng() * 100000;
					}
					else if (my_rak1910_gnss.altitude.isUpdated() && my_rak1910_gnss.altitude.isValid())
					{
						has_alt = true;
						altitude = my_rak1910_gnss.altitude.meters();
					}
					else if (my_rak1910_gnss.hdop.isUpdated() && my_rak1910_gnss.hdop.isValid())
					{
						accuracy = my_rak1910_gnss.hdop.hdop() * 100;
					}
				}
				if (has_pos && has_alt)
				{
					break;
				}
			}
			if (has_pos && has_alt)
			{
				break;
			}
		}
		break;

	case RAK12500_GNSS:
		MYLOG("GNSS", "Polling RAK12500");
		if (g_ble_uart_is_connected)
		{
			g_ble_uart.print("Polling RAK12500\n");
		}

		if (my_rak12500_gnss.getGnssFixOk())
		{
			latitude = my_rak12500_gnss.getLatitude() / 100;
			longitude = my_rak12500_gnss.getLongitude() / 100;
			altitude = my_rak12500_gnss.getAltitude() / 1000;
			accuracy = my_rak12500_gnss.getHorizontalDOP();
			has_pos = true;
		}
		break;

	default:
		MYLOG("GNSS", "No valid gnss_option provided");
		if (g_ble_uart_is_connected)
		{
			g_ble_uart.print("No valid gnss_option provided\n");
		}
	}

	digitalWrite(LED_BUILTIN, LOW);
	delay(10);

	if (has_pos)
	{
		MYLOG("GNSS", "Lat: %.4fº Lon: %.4fº", latitude / 100000.0, longitude / 100000.0);
		MYLOG("GNSS", "Alt: %d m", altitude);
		MYLOG("GNSS", "Acy: %.2f ", accuracy / 100.0);

		if (g_ble_uart_is_connected)
		{
			g_ble_uart.printf("Lat: %.4fº Lon: %.4fº\n", latitude / 100000.0, longitude / 100000.0);
			g_ble_uart.printf("Alt: %d m\n", altitude);
			g_ble_uart.printf("Acy: %.2f\n", accuracy / 100.0);
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

		pos_union.val32 = accuracy;
		g_mapper_data.acy_1 = pos_union.val8[0];
		g_mapper_data.acy_2 = pos_union.val8[1];
	}
	else
	{
		delay(1000);
	}

	if (has_pos)
	{
		// my_rak1910_gnss.powerSaveMode(true);
		// my_rak1910_gnss.setMeasurementRate(10000);
		return true;
	}

	last_read_ok = false;
	// my_rak1910_gnss.setMeasurementRate(1000);
	return false;
}