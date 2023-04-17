/**
 * @file app.h
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief For application specific includes and definitions
 *        Will be included from main.h
 * @version 0.1
 * @date 2021-04-23
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef APP_H
#define APP_H

#include <Arduino.h>
/** Add you required includes after Arduino.h */

/** Include the WisBlock-API */
#include <WisBlock-API-V2.h> // Click to install library: http://librarymanager/All#WisBlock-API

// Debug output set to 0 to disable app debug output
#ifndef MY_DEBUG
#define MY_DEBUG 1
#endif

#if MY_DEBUG > 0
#define MYLOG(tag, ...)           \
	do                            \
	{                             \
		if (tag)                  \
			PRINTF("[%s] ", tag); \
		PRINTF(__VA_ARGS__);      \
		PRINTF("\n");             \
	} while (0)
#else
#define MYLOG(...)
#endif

/** Application function definitions */
void setup_app(void);
bool init_app(void);
void app_event_handler(void);
void ble_data_handler(void) __attribute__((weak));
void lora_data_handler(void);
void lora_tx_finished(bool success);
void lora_rx_failed(void);

/** Examples for application events */
#define ACC_TRIGGER 0b1000000000000000
#define N_ACC_TRIGGER 0b0111111111111111

/** Application stuff */
extern BaseType_t g_higher_priority_task_woken;

// GNSS options
#define RAK1910_GNSS 1
#define RAK12500_GNSS 2

// GNSS functions
#include "TinyGPS++.h"
#include <SoftwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // RAK12500_GNSS
uint8_t init_gnss(void);
bool poll_gnss(uint8_t gnss_option);

/** Accelerometer stuff */
#include <SparkFunLIS3DH.h>
#define INT1_PIN WB_IO5
bool init_acc(void);
void clear_acc_int(void);
void read_acc(void);

// LoRaWan functions
struct mapper_data_s
{
	uint8_t lat_1 = 0;	// 1
	uint8_t lat_2 = 0;	// 2
	uint8_t lat_3 = 0;	// 3
	uint8_t lat_4 = 0;	// 4
	uint8_t long_1 = 0; // 5
	uint8_t long_2 = 0; // 6
	uint8_t long_3 = 0; // 7
	uint8_t long_4 = 0; // 8
	uint8_t alt_1 = 0;	// 9
	uint8_t alt_2 = 0;	// 10
	uint8_t acy_1 = 0;	// 11
	uint8_t acy_2 = 0;	// 12
	uint8_t batt_1 = 0; // 13
	uint8_t batt_2 = 0; // 14
};
extern mapper_data_s g_mapper_data;
#define MAPPER_DATA_LEN 14 // sizeof(g_mapper_data)

/** Battery level uinion */
union batt_s
{
	uint16_t batt16 = 0;
	uint8_t batt8[2];
};
/** Latitude/Longitude value union */
union latLong_s
{
	uint32_t val32;
	uint8_t val8[4];
};

#endif