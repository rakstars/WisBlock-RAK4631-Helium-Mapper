/**
 * @file app.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Application specific functions. Mandatory to have init_app(), 
 *        app_event_handler(), ble_data_handler(), lora_data_handler()
 *        and lora_tx_finished()
 * @version 0.1
 * @date 2021-04-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "app.h"

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "WB-Mapper";

/** Flag showing if TX cycle is ongoing */
bool lora_busy = false;

/** Timer since last position message was sent */
time_t last_pos_send = 0;
/** Timer for delayed sending to keep duty cycle */
SoftwareTimer delayed_sending;
/** Required for give semaphore from ISR */
BaseType_t g_higher_priority_task_woken = pdTRUE;

/** Battery level uinion */
batt_s batt_level;

/** Flag if delayed sending is already activated */
bool delayed_active = false;

/** Minimum delay between sending new locations, set to 45 seconds */
time_t min_delay = 45000;

/** The GPS module to use */
uint8_t gnss_option;

// Forward declaration
void send_delayed(TimerHandle_t unused);

/**
 * @brief Application specific setup functions
 * 
 */
void setup_app(void)
{
		// Called in the very beginning of setup
	/**************************************************************/
	/**************************************************************/
	/// \todo set g_enable_ble to true if you want to enable BLE
	/// \todo if true, BLE is enabled and advertise 60 seconds
	/// \todo after reset or power-up
	/**************************************************************/
	/**************************************************************/
	g_enable_ble = true;
	api_set_version(1, 0, 2);
}

/**
 * @brief Application specific initializations
 * 
 * @return true Initialization success
 * @return false Initialization failure
 */
bool init_app(void)
{
	// Add your application specific initialization here
	bool init_result = true;

	MYLOG("APP", "Application initialization");
	if (g_ble_uart_is_connected)
	{
		g_ble_uart.print("Application initialization\n");
	}

	pinMode(WB_IO2, OUTPUT);
	digitalWrite(WB_IO2, HIGH);

	// Initialize GNSS module
	gnss_option = init_gnss();

	// Initialize ACC sensor
	init_result |= init_acc();

	if (g_lorawan_settings.send_repeat_time != 0)
	{
		// Set delay for sending to 1/2 of scheduled sending
		//min_delay = g_lorawan_settings.send_repeat_time / 2;
		min_delay = 15000;
	}
	else
	{
		// Send repeat time is 0, set delay to 30 seconds
		min_delay = 30000;
	}
	// Set to 1/2 of programmed send interval or 30 seconds
	delayed_sending.begin(min_delay, send_delayed, NULL, false);

	// Power down GNSS module
	// pinMode(WB_IO2, OUTPUT);
	// digitalWrite(WB_IO2, LOW);
	return init_result;
}

/**
 * @brief Application specific event handler
 *        Requires as minimum the handling of STATUS event
 *        Here you handle as well your application specific events
 */
void app_event_handler(void)
{
	// Timer triggered event
	if ((g_task_event_type & STATUS) == STATUS)
	{
		g_task_event_type &= N_STATUS;

		MYLOG("APP", "Timer wakeup");
		if (g_ble_uart_is_connected)
		{
			g_ble_uart.print("Timer wakeup\n");
		}

		clear_acc_int();

		// If BLE is enabled, restart Advertising
		if (g_enable_ble)
		{
			restart_advertising(15);
		}

		if (lora_busy)
		{
			MYLOG("APP", "LoRaWAN TX cycle not finished, skip this event");
			if (g_ble_uart_is_connected)
			{
				g_ble_uart.print("LoRaWAN TX cycle not finished, skip this event\n");
			}
		}
		else
		{
			// Get battery level
			batt_level.batt16 = read_batt();
			g_mapper_data.batt_1 = batt_level.batt8[0];
			g_mapper_data.batt_2 = batt_level.batt8[1];
			
			MYLOG("APP", "Battery level %d", batt_level.batt16);
			MYLOG("APP", "Trying to poll GNSS position");
			if(g_ble_uart_is_connected)
			{
				g_ble_uart.printf("Battery: %.2f V\n", batt_level.batt16 / 1000.0);
				g_ble_uart.print("Trying to poll GNSS position\n");
			}
			
			if (poll_gnss(gnss_option))
			{
				MYLOG("APP", "Valid GNSS position acquired");
				if (g_ble_uart_is_connected)
				{
					g_ble_uart.print("Valid GNSS position acquired\n");
				}

				MYLOG("APP", "Lat 1: %02X", g_mapper_data.lat_1);
				MYLOG("APP", "Lat 2: %02X", g_mapper_data.lat_2);
				MYLOG("APP", "Lat 3: %02X", g_mapper_data.lat_3);
				MYLOG("APP", "Lat 4: %02X", g_mapper_data.lat_4);
				MYLOG("APP", "Long 1: %02X", g_mapper_data.long_1);
				MYLOG("APP", "Long 2: %02X", g_mapper_data.long_2);
				MYLOG("APP", "Long 3: %02X", g_mapper_data.long_3);
				MYLOG("APP", "Long 4: %02X", g_mapper_data.long_4);
				MYLOG("APP", "Alt 1: %02X", g_mapper_data.alt_1);
				MYLOG("APP", "Alt 2: %02X", g_mapper_data.alt_2);
				MYLOG("APP", "Acy 1: %02X", g_mapper_data.acy_1);
				MYLOG("APP", "Acy 2: %02X", g_mapper_data.acy_2);
				MYLOG("APP", "Batt 1: %02X", g_mapper_data.batt_1);
				MYLOG("APP", "Batt 2: %02X", g_mapper_data.batt_2);

				if (g_ble_uart_is_connected) {
					g_ble_uart.printf("Lat 1: %02X\n", g_mapper_data.lat_1);
					g_ble_uart.printf("Lat 2: %02X\n", g_mapper_data.lat_2);
					g_ble_uart.printf("Lat 3: %02X\n", g_mapper_data.lat_3);
					g_ble_uart.printf("Lat 4: %02X\n", g_mapper_data.lat_4);
					g_ble_uart.printf("Long 1: %02X\n", g_mapper_data.long_1);
					g_ble_uart.printf("Long 2: %02X\n", g_mapper_data.long_2);
					g_ble_uart.printf("Long 3: %02X\n", g_mapper_data.long_3);
					g_ble_uart.printf("Long 4: %02X\n", g_mapper_data.long_4);
					g_ble_uart.printf("Alt 1: %02X\n", g_mapper_data.alt_1);
					g_ble_uart.printf("Alt 2: %02X\n", g_mapper_data.alt_2);
					g_ble_uart.printf("Acy 1: %02X\n", g_mapper_data.acy_1);
					g_ble_uart.printf("Acy 2: %02X\n", g_mapper_data.acy_2);
					g_ble_uart.printf("Batt 1: %02X\n", g_mapper_data.batt_1);
					g_ble_uart.printf("Batt 2: %02X\n", g_mapper_data.batt_2);
				}

				lmh_error_status result = send_lora_packet((uint8_t *)&g_mapper_data, MAPPER_DATA_LEN);
				switch (result)
				{
				case LMH_SUCCESS:
					MYLOG("APP", "Packet enqueued");
					if (g_ble_uart_is_connected)
					{
						g_ble_uart.print("Packet enqueued\n");
					}
					/// \todo set a flag that TX cycle is running
					lora_busy = true;
					
					break;
				case LMH_BUSY:
					MYLOG("APP", "LoRa transceiver is busy");
					if (g_ble_uart_is_connected)
					{
						g_ble_uart.print("LoRa transceiver is busy\n");
					}
					break;
				case LMH_ERROR:
					MYLOG("APP", "Packet error, too big to send with current DR");
					if (g_ble_uart_is_connected)
					{
						g_ble_uart.print("Packet error, too big to send with current DR\n");
					}
					break;
				}

			}
			else
			{
				MYLOG("APP", "No valid GNSS position");
				if (g_ble_uart_is_connected)
				{
					g_ble_uart.print("No valid GNSS position\n");
				}
			}

			// Remember last time sending
			last_pos_send = millis();
			// Just in case
			delayed_active = false;


		}
	}

	// ACC trigger event
	if ((g_task_event_type & ACC_TRIGGER) == ACC_TRIGGER &&  g_lpwan_has_joined)
	{
		g_task_event_type &= N_ACC_TRIGGER;
		MYLOG("APP", "ACC triggered");
		if (g_ble_uart_is_connected)
		{
			g_ble_uart.print("ACC triggered\n");
		}

		// Check time since last send
		bool send_now = true;
		if (g_lorawan_settings.send_repeat_time != 0)
		{
			if ((millis() - last_pos_send) < min_delay)
			{
				send_now = false;
				if (!delayed_active)
				{
					delayed_sending.stop();
					MYLOG("APP", "Expired time %d", (int)(millis() - last_pos_send));
					MYLOG("APP", "Max delay time %d", (int)min_delay);
					if (g_ble_uart_is_connected)
					{
						g_ble_uart.printf("Expired time %d\n", (millis() - last_pos_send));
						g_ble_uart.printf("Max delay time %d\n", min_delay);
					}
					time_t wait_time = abs(min_delay - (millis() - last_pos_send) >= 0) ? (min_delay - (millis() - last_pos_send)) : min_delay;
					MYLOG("APP", "Wait time %ld", (long)wait_time);
					if (g_ble_uart_is_connected)
					{
						g_ble_uart.printf("Wait time %d\n", wait_time);
					}

					MYLOG("APP", "Only %lds since last position message, send delayed in %lds", (long)((millis() - last_pos_send) / 1000), (long)(wait_time / 1000));
					if (g_ble_uart_is_connected)
					{
						g_ble_uart.printf("Only %ds since last pos msg, delay by %ds\n", ((millis() - last_pos_send) / 1000), (wait_time / 1000));
					}
					delayed_sending.setPeriod(wait_time);
					delayed_sending.start();
					delayed_active = true;
				}
			}
		}
		if (send_now)
		{
			// Remember last send time
			last_pos_send = millis();

			// Trigger a GNSS reading and packet sending
			g_task_event_type |= STATUS;
		}

		// Reset the standard timer
		if (g_lorawan_settings.send_repeat_time != 0)
		{
			api_timer_restart(g_lorawan_settings.send_repeat_time);
		}
	}

}

/**
 * @brief Handle BLE UART data
 * 
 */
void ble_data_handler(void)
{
	if (g_enable_ble)
	{
		// BLE UART data handling
		if ((g_task_event_type & BLE_DATA) == BLE_DATA)
		{
			/**************************************************************/
			/**************************************************************/
			/// \todo BLE UART data arrived
			/// \todo parse them here
			/**************************************************************/
			/**************************************************************/
			MYLOG("AT", "Received BLE");
			/** BLE UART data arrived */
			g_task_event_type &= N_BLE_DATA;

			while (g_ble_uart.available() > 0)
			{
				at_serial_input(uint8_t(g_ble_uart.read()));
				delay(5);
			}
			at_serial_input(uint8_t('\n'));
		}
	}
}

/**
 * @brief Handle received LoRa Data
 * 
 */
void lora_data_handler(void)
{
	// LoRa data handling
	if ((g_task_event_type & LORA_DATA) == LORA_DATA)
	{
		/**************************************************************/
		/**************************************************************/
		/// \todo LoRa data arrived
		/// \todo parse them here
		/**************************************************************/
		/**************************************************************/
		g_task_event_type &= N_LORA_DATA;
		MYLOG("APP", "Received package over LoRa");
		if (g_ble_uart_is_connected)
		{
			g_ble_uart.print("Received package over LoRa\n");
		}

		char log_buff[g_rx_data_len * 3] = {0};
		uint8_t log_idx = 0;
		for (int idx = 0; idx < g_rx_data_len; idx++)
		{
			sprintf(&log_buff[log_idx], "%02X ", g_rx_lora_data[idx]);
			log_idx += 3;
		}
		lora_busy = false;

		MYLOG("APP", "%s", log_buff);
		if (g_ble_uart_is_connected)
		{
			g_ble_uart.printf("%s", log_buff);
		}

		/**************************************************************/
		/**************************************************************/
		/// \todo Just an example, if BLE is enabled and BLE UART
		/// \todo is connected you can send the received data
		/// \todo for debugging
		/**************************************************************/
		/**************************************************************/
		if (g_ble_uart_is_connected && g_enable_ble)
		{
			for (int idx = 0; idx < g_rx_data_len; idx++)
			{
				g_ble_uart.printf("%02X ", g_rx_lora_data[idx]);
			}
			g_ble_uart.print("");
		}
	}

	// LoRa TX finished handling
	if ((g_task_event_type & LORA_TX_FIN) == LORA_TX_FIN)
	{
		/**************************************************************/
		/**************************************************************/
		/// \todo LoRaWAN TX cycle (including RX1 and RX2 window) finished
		/// \todo can be used to enable next sending
		/// \todo if confirmed packet sending, g_rx_fin_result holds the result of the transmission
		/**************************************************************/
		/**************************************************************/
		g_task_event_type &= N_LORA_TX_FIN;

		MYLOG("APP", "LPWAN TX cycle %s", g_rx_fin_result ? "finished ACK" : "failed NAK");
		if (g_ble_uart_is_connected)
		{
			g_ble_uart.printf("LPWAN TX cycle %s\n", g_rx_fin_result ? "finished ACK" : "failed NAK");
		}

		/// \todo reset flag that TX cycle is running
		lora_busy = false;
	}
}

void tud_cdc_rx_cb(uint8_t itf)
{
	g_task_event_type |= AT_CMD;
	if (g_task_sem != NULL)
	{
		xSemaphoreGiveFromISR(g_task_sem, pdFALSE);
	}
}

/**
 * @brief Timer function used to avoid sending packages too often.
 * 			Delays the next package by 10 seconds
 * 
 * @param unused 
 * 			Timer handle, not used
 */
void send_delayed(TimerHandle_t unused)
{
	g_task_event_type |= STATUS;
	xSemaphoreGiveFromISR(g_task_sem, &g_higher_priority_task_woken);
}
