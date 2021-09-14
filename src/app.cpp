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

#include "main.h"

/** Set the device name, max length is 10 characters */
char ble_dev_name[10] = "WB-Mapper";

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
	/// \todo set enable_ble to true if you want to enable BLE
	/// \todo if true, BLE is enabled and advertise 60 seconds
	/// \todo after reset or power-up
	/**************************************************************/
	/**************************************************************/
	enable_ble = true;
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
	MYLOG("APP", "init_app");

	pinMode(WB_IO2, OUTPUT);
	digitalWrite(WB_IO2, HIGH);

	// Initialize GNSS module
	init_result = init_gnss();

	if (g_lorawan_settings.send_repeat_time != 0)
	{
		// Set delay for sending to 1/2 of scheduled sending
		min_delay = g_lorawan_settings.send_repeat_time / 2;
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

		// If BLE is enabled, restart Advertising
		if (enable_ble)
		{
			restart_advertising(15);
		}

		if (lora_busy)
		{
			MYLOG("APP", "LoRaWAN TX cycle not finished, skip this event");
			if (ble_uart_is_connected)
			{
				ble_uart.print("LoRaWAN TX cycle not finished, skip this event\n");
			}
		}
		else
		{
			MYLOG("APP", "Trying to poll GNSS position");
			if (poll_gnss())
			{
				MYLOG("APP", "Valid GNSS position");
				if (ble_uart_is_connected)
				{
					ble_uart.print("Valid GNSS position\n");
				}

				Serial.printf("%02X", g_mapper_data.lat_1);
				Serial.printf("%02X", g_mapper_data.lat_2);
				Serial.printf("%02X", g_mapper_data.lat_3);
				Serial.printf("%02X", g_mapper_data.lat_4);
				Serial.printf("%02X", g_mapper_data.long_1);
				Serial.printf("%02X", g_mapper_data.long_2);
				Serial.printf("%02X", g_mapper_data.long_3);
				Serial.printf("%02X", g_mapper_data.long_4);
				Serial.printf("%02X", g_mapper_data.alt_1);
				Serial.printf("%02X", g_mapper_data.alt_2);
				Serial.println("");

				lmh_error_status result = send_lora_packet((uint8_t *)&g_mapper_data, MAPPER_DATA_LEN);
				switch (result)
				{
				case LMH_SUCCESS:
					MYLOG("APP", "Packet enqueued");
					/// \todo set a flag that TX cycle is running
					lora_busy = true;
					if (ble_uart_is_connected)
					{
						ble_uart.print("Packet enqueued\n");
					}
					break;
				case LMH_BUSY:
					MYLOG("APP", "LoRa transceiver is busy");
					if (ble_uart_is_connected)
					{
						ble_uart.print("LoRa transceiver is busy\n");
					}
					break;
				case LMH_ERROR:
					MYLOG("APP", "Packet error, too big to send with current DR");
					if (ble_uart_is_connected)
					{
						ble_uart.print("Packet error, too big to send with current DR\n");
					}
					break;
				}

			}
			else
			{
				MYLOG("APP", "No valid GNSS position");
				if (ble_uart_is_connected)
				{
					ble_uart.print("No valid GNSS position\n");
				}
			}

			// Remember last time sending
			last_pos_send = millis();
			// Just in case
			delayed_active = false;


		}
	}

}

/**
 * @brief Handle BLE UART data
 * 
 */
void ble_data_handler(void)
{
	if (enable_ble)
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
			MYLOG("AT", "RECEIVED BLE");
			/** BLE UART data arrived */
			g_task_event_type &= N_BLE_DATA;

			while (ble_uart.available() > 0)
			{
				at_serial_input(uint8_t(ble_uart.read()));
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
		char log_buff[g_rx_data_len * 3] = {0};
		uint8_t log_idx = 0;
		for (int idx = 0; idx < g_rx_data_len; idx++)
		{
			sprintf(&log_buff[log_idx], "%02X ", g_rx_lora_data[idx]);
			log_idx += 3;
		}
		lora_busy = false;
		MYLOG("APP", "%s", log_buff);

		/**************************************************************/
		/**************************************************************/
		/// \todo Just an example, if BLE is enabled and BLE UART
		/// \todo is connected you can send the received data
		/// \todo for debugging
		/**************************************************************/
		/**************************************************************/
		if (ble_uart_is_connected && enable_ble)
		{
			for (int idx = 0; idx < g_rx_data_len; idx++)
			{
				ble_uart.printf("%02X ", g_rx_lora_data[idx]);
			}
			ble_uart.print("");
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
		if (ble_uart_is_connected)
		{
			ble_uart.printf("LPWAN TX cycle %s\n", g_rx_fin_result ? "finished ACK" : "failed NAK");
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