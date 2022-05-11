#pragma once

#include "cmain.h"
#include "perif/uart.h"
#include <functional>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "usart.h"
#include "util/fifo.h"

#include "FreeRTOS.h"
#include "task.h"


#define AT_NUMBER_OF_REPLIES 5
#define AT_LENGTH_OF_REPLIES 50

#define WIFI_RX_STATE_SCOUT 0
#define WIFI_RX_STATE_CONNECT 1
#define WIFI_RX_STATE_CLOSED 2
#define WIFI_RX_STATE_RECEIVE 3
#define WIFI_RX_STATE_SEND_READY 4
#define WIFI_RX_STATE_SEND_RECV 5

#define WIFI_TX_STATE_PENDING 0
#define WIFI_TX_STATE_RECV 1

namespace quad
{

class ESP
{
private:
	UART m_ESPcomm;

	util::string_fifo<WIFI_INDATA_LENGTH> esp_inData;
	util::string_fifo<WIFI_OUTDATA_LENGTH> esp_outData;

	util::string_fifo<TOPIC_INDATA_LENGTH> ros_inData;

	/* AT commands */
	char at_command_cipstart[46]	= "AT+CIPSTART=\"TCP\",\"192.168.4.2\",11411\r\n";

	/* AT command replies */
	char *at_command_reply_vector[AT_NUMBER_OF_REPLIES] = {	"CONNECT",
															"CLOSED",
															"+IPD,",
															">",
															//"Recv"};
															"SEND OK"};
															//"OK"};
	uint8_t wifi_rx_state = WIFI_RX_STATE_SCOUT;
	uint8_t wifi_tx_state = WIFI_TX_STATE_PENDING;
	char throw_away = '0';						/* Variable to read unnecessary characters into from rx messages */
	bool wifi_connected = false;

#if DEBUG_CODE_ACTIVE
	volatile char wifi_throw_away_debug[50] = "";
	uint8_t wifi_rx_state_debug[10];
	uint8_t wifi_rx_state_counter = 0u;
	uint8_t wifi_rx_state_debug_previus = 0;
	uint8_t wifi_tx_state_debug[10];
	uint8_t wifi_tx_state_counter = 0u;
	uint8_t wifi_tx_state_debug_previus = 0;
	util::string_fifo<40000> esp_inData_debug;
#endif

	volatile bool wifi_send_ready = false;
	volatile bool wifi_send_recv = false;
	volatile bool wifi_tx_process_done = false;

	int32_t wifi_send_size = 0;

	void Rx_Process();
	void Tx_Process();

	int32_t Tx_SetupSend();

public:
	ESP(USART_TypeDef *uart);
	UART& getUART();

	util::string_fifo<TOPIC_INDATA_LENGTH>& GetRos_inData();
	util::string_fifo<WIFI_OUTDATA_LENGTH>& GetRos_outData();

	void Start();
	void Process();
};

}
