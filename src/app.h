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

/** Application function definitions */
void setup_app(void);
bool init_app(void);
void app_event_handler(void);
void ble_data_handler(void) __attribute__((weak));
void lora_data_handler(void);
void lora_tx_finished(bool success);
void lora_rx_failed(void);

/** Examples for application events */
#define PIR_TRIGGER   0b1000000000000000
#define N_PIR_TRIGGER 0b0111111111111111
#define BUTTON        0b0100000000000000
#define N_BUTTON      0b1011111111111111

/** Application stuff */
extern BaseType_t g_higher_priority_task_woken;

// GNSS functions
#include "TinyGPS++.h"
#include <SoftwareSerial.h>
bool init_gnss(void);
bool poll_gnss(void);

// LoRaWan functions
struct mapper_data_s
{
	uint8_t lat_1 = 0;			// 1
	uint8_t lat_2 = 0;			// 2
	uint8_t lat_3 = 0;			// 3
	uint8_t lat_4 = 0;			// 4
	uint8_t long_1 = 0;			// 5
	uint8_t long_2 = 0;			// 6
	uint8_t long_3 = 0;			// 7
	uint8_t long_4 = 0;			// 8
	uint8_t alt_1 = 0;			// 9
	uint8_t alt_2 = 0;			// 10
};
extern mapper_data_s g_mapper_data;
#define MAPPER_DATA_LEN 10 // sizeof(g_mapper_data)

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