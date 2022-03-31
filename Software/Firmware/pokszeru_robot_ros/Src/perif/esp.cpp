#include "perif/esp.h"

#define WIFI_UART_BOOTUP_RESPONSE 1000
#define WIFI_UART_RESPONSE 50
#define WIFI_UART_CONNECTION 1000
#define WIFI_TCP_RETRY 3000
#define WIFI_1_MS_WAIT 1

namespace quad
{
ESP::ESP(USART_TypeDef *uart):
		m_ESPcomm(uart)
{

}

UART& ESP::getUART()
{
	return m_ESPcomm;
}

util::string_fifo<TOPIC_INDATA_LENGTH>& ESP::GetRos_inData()
{
	return ros_inData;
}

util::string_fifo<WIFI_OUTDATA_LENGTH>& ESP::GetRos_outData()
{
	return esp_outData;
}

void ESP::Start()
{
	char response[35];
	char at_command_at[] = "AT\r\r\n\r\nOK\r\n";
	char at_command_tcp_error[] = "\r\nERROR\r\nCLOSED\r\n";

	//char at_command_uart_def[]		= "AT+UART_DEF=115200,8,1,0,0\r\n";
	//char at_command_uart_cur[]		= "AT+UART_CUR=115200,8,1,0,0\r\n";
	//char at_command_cwmode[]		= "AT+CWMODE_CUR=3\r\n";
	char at_command_cwmode[]		= "AT+CWMODE_CUR=2\r\n";			//AP mode (server)
	//char at_command_cipmux[]		= "AT+CIPMUX=1\r\n";
	//char at_command_cipserver[]		= "AT+CIPSERVER=1\r\n";
	//char at_command_cwjap[]			= "AT+CWJAP_CUR=\"Limbo\",\"k3r3v3tLG\"\r\n";
	char at_command_ate[]			= "ATE0\r\n";
	//char at_command_cipsend[]		= "AT+CIPSEND=0,0\r\n";
	//char at_command_cifsr[]			= "AT+CIFSR\r\n";
	/* char at_command_cipstart[]		= "AT+CIPSTART=\"TCP\",\"192.168.4.2\",11411\r\n"; */
	//char at_command_cipstatus[]		= "AT+CIPSTATUS\r\n";

	/* Try ESP-01 reset */
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
	vTaskDelay( WIFI_UART_RESPONSE / portTICK_PERIOD_MS );
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);

	vTaskDelay( WIFI_UART_BOOTUP_RESPONSE / portTICK_PERIOD_MS );

	/* Delete unwanted boot-up message from ESP */
	m_ESPcomm.ClearInFifo();
	/* Send test message to check baudrate */
	m_ESPcomm.SendString("AT\r\n");
	while(!m_ESPcomm.ReadBuffer(response, 11))
	{
		vTaskDelay( WIFI_1_MS_WAIT / portTICK_PERIOD_MS );
		//todo: watchog, or fix wait time (previus todo)
	}

	/* Set last charachter to string end */
	response[11] = '\0';

	if(0 == strcmp((char*)response, (char*)at_command_at))
	{
		/* return; */
	}
	else
	{
		MX_USART3_UART_Init((uint32_t)115200);
	}

	m_ESPcomm.ClearInFifo();

	m_ESPcomm.SendString(at_command_ate);
	vTaskDelay( WIFI_UART_RESPONSE / portTICK_PERIOD_MS );

	m_ESPcomm.SendString(at_command_cwmode);
	vTaskDelay( WIFI_UART_RESPONSE / portTICK_PERIOD_MS );

	m_ESPcomm.ClearInFifo();

	/*m_ESPcomm.SendString(at_command_cipmux);
	LL_mDelay(WIFI_UART_RESPONSE);

	m_ESPcomm.SendString(at_command_cipserver);
	LL_mDelay(WIFI_UART_RESPONSE);

	m_ESPcomm.SendString(at_command_cwjap);
	while(!m_ESPcomm.ReadBuffer(response, 35))
	{
		//todo: watchog, or fix wait time (previus todo)
	}

	m_ESPcomm.ClearInFifo();*/

	while(1)
	{
		m_ESPcomm.SendString(at_command_cipstart);
		while(!m_ESPcomm.ReadBuffer(response, 17))
		{
			vTaskDelay( WIFI_1_MS_WAIT / portTICK_PERIOD_MS );
		}

		/* Set last charachter to string end */
		response[17] = '\0';

		if(0 == strcmp((char*)response, (char*)at_command_tcp_error))
		{
			m_ESPcomm.ClearInFifo();
			vTaskDelay( WIFI_TCP_RETRY / portTICK_PERIOD_MS );
		}
		else
		{
			wifi_connected = true;
			break;
		}
	}

	vTaskDelay( WIFI_UART_RESPONSE / portTICK_PERIOD_MS );
}

void ESP::Process()
{
	Rx_Process();
	if(wifi_connected)
	{
		Tx_Process();
	}
	else
	{
		/* Try to reconnect */
		m_ESPcomm.SendString(at_command_cipstart);
		vTaskDelay( WIFI_TCP_RETRY / portTICK_PERIOD_MS );
	}

#if DEBUG_CODE_ACTIVE
	if(wifi_rx_state_debug[wifi_rx_state_counter] != wifi_rx_state)
	{
		wifi_rx_state_debug_previus = wifi_rx_state_debug[wifi_rx_state_counter];
		if(9 != wifi_rx_state_counter)
		{
			wifi_rx_state_counter++;
		}
		else
		{
			wifi_rx_state_counter = 0;
		}
		wifi_rx_state_debug[wifi_rx_state_counter] = wifi_rx_state;
	}
	if(wifi_tx_state_debug[wifi_tx_state_counter] != wifi_tx_state)
	{
		wifi_tx_state_debug_previus = wifi_tx_state_debug[wifi_tx_state_counter];
		if(9 != wifi_tx_state_counter)
		{
			wifi_tx_state_counter++;
		}
		else
		{
			wifi_tx_state_counter = 0;
		}
		wifi_tx_state_debug[wifi_tx_state_counter] = wifi_tx_state;
	}
#endif
}

void ESP::Rx_Process()
{
	volatile uint8_t next_state = WIFI_RX_STATE_SCOUT;		/* The next state of the Rx_Process */
	volatile int16_t next_state_priority = -1;				/* The position of the found string in the rx_inData */

	m_ESPcomm.CopyInFifo(esp_inData);

	/* Insert string termination after the data to prevent searching algorithms to find valid matches in invalid buffer spaces */
	esp_inData.insertTermination();

#if DEBUG_CODE_ACTIVE
	for(uint8_t i=0; i<esp_inData.size(); i++)
	{
		esp_inData_debug.push(esp_inData.buffer()[esp_inData.begin()+i]);
		if(40000 == esp_inData_debug.size())
		{
			esp_inData_debug.clear();
		}
	}
#endif

	switch(wifi_rx_state)
	{
	case(WIFI_RX_STATE_SCOUT):
	{
		/* Iterate through the possible replies */
		for(uint16_t reply_vector_looper = 0; reply_vector_looper < AT_NUMBER_OF_REPLIES; reply_vector_looper++)
		{
			int16_t tmp_priority;

			/* Search for a specific possible reply */
			tmp_priority = esp_inData.compare(at_command_reply_vector[reply_vector_looper]);

			/* Valid command was found for the first time || Valid command was found before the command that was already found*/
			if( (-1 == next_state_priority && -1 != tmp_priority) || (-1 != next_state_priority && -1 != tmp_priority && tmp_priority < next_state_priority) )
			{
				next_state_priority = tmp_priority;
				/* Set the next state, the states specified so they are equal to the looper count + 1 */
				next_state = reply_vector_looper + 1;
			}
		}

		/* Throw away unused characters if valid command was found */
		for(uint16_t looper = 0; looper < next_state_priority && WIFI_RX_STATE_SCOUT != next_state; looper++)
		{
			esp_inData.pop(throw_away);

#if DEBUG_CODE_ACTIVE
			//strcat((char*)wifi_throw_away_debug, &throw_away);
			//wifi_throw_away_debug[looper] = throw_away;
			//if(NULL != strstr((char*)wifi_throw_away_debug, "SE"))
			//{
			//	uint8_t zup;
			//	zup = 0;
			//}
#endif
		}

		wifi_rx_state = next_state;
		break;
	}
	case(WIFI_RX_STATE_CONNECT):
	{
		wifi_connected = true;
		esp_inData.pop(throw_away);				// TODO throw away CONNECT string
		wifi_rx_state = WIFI_RX_STATE_SCOUT;
		break;
	}
	case(WIFI_RX_STATE_CLOSED):
	{
		wifi_connected = false;
		esp_inData.pop(throw_away);				// TODO throw away CLOSED string
		wifi_rx_state = WIFI_RX_STATE_SCOUT;
		break;
	}
	case(WIFI_RX_STATE_RECEIVE):
	{
		/* Variable that stores the message length */
		static int32_t size;
		char tmp_rx_inData[WIFI_INDATA_LENGTH];
		/* Flag to signal if invalid data has arrived in the size (not a number) */
		volatile bool invalid_character_found = false;

		/* Throw away "+IPD," */
		for(uint16_t looper = 0; looper < 5; looper++)
		{
			esp_inData.pop(throw_away);
		}

		/* Wait for all the data to arrive to process the message size */
		while(false == esp_inData.esp_size_arrived())
		{
			vTaskDelay( WIFI_1_MS_WAIT / portTICK_PERIOD_MS );
			m_ESPcomm.CopyInFifo(esp_inData);
		}

		/* Read in the size of the in data */
		for(uint32_t looper = 0; esp_inData.pop(tmp_rx_inData[looper]);looper++)
		{
			/* Check if all the characters of the size information was processed */
			if(':' == tmp_rx_inData[looper])
			{
				tmp_rx_inData[looper] = '\0';
				break;
			}
			/* Check if invalid data is contained in the size information */
			else if('0' > tmp_rx_inData[looper] || '9' < tmp_rx_inData[looper])
			{
				invalid_character_found = true;
				break;
			}
		}

		/* If invalid data was found in the size information finish processing */
		if(invalid_character_found)
		{
			wifi_rx_state = WIFI_RX_STATE_SCOUT;
			break;
		}

		/* Convert the read in size string to integer */
		size = atoi(tmp_rx_inData);

		/* Wait for all the data to arrive to process the message */
		while(esp_inData.size() < size)
		{
			vTaskDelay( WIFI_1_MS_WAIT / portTICK_PERIOD_MS );
			m_ESPcomm.CopyInFifo(esp_inData);
		}

		for(int32_t looper = 0; looper < size; looper++)
		{
			// TODO copy to ros fifo
			char tmp;
			esp_inData.pop(tmp);
			ros_inData.push(tmp);
		}

		wifi_rx_state = WIFI_RX_STATE_SCOUT;

		break;
	}
	case(WIFI_RX_STATE_SEND_READY):
	{
		if(WIFI_TX_STATE_PENDING == wifi_tx_state)
		{
			wifi_send_ready = true;
		}

		esp_inData.pop(throw_away);
		wifi_rx_state = WIFI_RX_STATE_SCOUT;

		break;
	}
	case(WIFI_RX_STATE_SEND_RECV):
	{
		if(WIFI_TX_STATE_RECV == wifi_tx_state)
		{
			wifi_send_recv = true;
		}

		esp_inData.pop(throw_away);
		wifi_rx_state = WIFI_RX_STATE_SCOUT;

		break;
	}
	}
}

void ESP::Tx_Process()
{
	if(esp_outData.size())
	{
		wifi_tx_process_done = false;

		while(false == wifi_tx_process_done)
		{
			wifi_tx_state = WIFI_TX_STATE_PENDING;
			/* Send AT+CIPSEND command to ESP, and save the promised size */
			wifi_send_size = Tx_SetupSend();

			/* Wait for response from the ESP */
			while(false == wifi_send_ready)
			{
				/* Ask for Rx message processing */
				Rx_Process();

				/* Note: This delay is not used to let the wifi sendig to be executed as fast as possible */
				/* vTaskDelay( WIFI_1_MS_WAIT / portTICK_PERIOD_MS ); */
			}
			wifi_send_ready = false;
			wifi_tx_state = WIFI_TX_STATE_RECV;

			/* Send the amount of data, that was promised to send */
			int32_t buffer_space = 0u;
			char tmp;
			while(wifi_send_size)
			{
				/* Check how much space is in the uart out buffer */
				buffer_space = UART_OUTDATA_LENGTH-m_ESPcomm.GetOutSize();

				if(buffer_space > wifi_send_size)
				{
					buffer_space = wifi_send_size;
				}

				/* Put into the amount that is possible to fit */
				for(uint16_t looper = 0; looper < buffer_space; looper++)
				{
					esp_outData.pop(tmp);
					m_ESPcomm.SendData(tmp);
				}

				/* Update the remaining data size */
				wifi_send_size -= buffer_space;

				vTaskDelay( WIFI_1_MS_WAIT / portTICK_PERIOD_MS );//todo testing this
			}

			/* Wait for response from the ESP */
			while(false == wifi_send_recv)
			{
				/* Ask for Rx message processing */
				Rx_Process();

				/* Note: This delay is not used to let the wifi sendig to be executed as fast as possible */
				/* vTaskDelay( WIFI_1_MS_WAIT / portTICK_PERIOD_MS ); */
			}
			wifi_send_recv = false;

			/* Check if there is more data to send */
			if(esp_outData.size())
			{
				/* More data to be sent, next iteration */
			}
			/* If not, finish sending */
			else
			{
				wifi_tx_process_done = true;
			}
		}
	}
}

int32_t ESP::Tx_SetupSend()
{
	/* String, long enough to store the entire Tx buffer (of uart) */
	char message[UART_OUTDATA_MESSAGE_LENGTH];
	int32_t size;

	/* Get size information of Tx data, and convert it to string */
	if(ESP_BUFFER_MAX_LENTH > (uint32_t)esp_outData.size())
	{
		size = esp_outData.size();
	}
	else
	{
		size = ESP_BUFFER_MAX_LENTH;
	}
	char sizeStr[ESP_BUFFER_MAX_LENTH_DIGITS];
	itoa(size, sizeStr, 10);

	/* Assemble AT+CIPSEND message */
	strcpy(message, "AT+CIPSEND=");
	strcat(message, sizeStr);
	strcat(message, "\r\n");

	while((UART_OUTDATA_LENGTH-m_ESPcomm.GetOutSize()) < strlen(message))
	{
		/* Wait until enough space is available in the out buffer */
	}

	/* Copy to Tx fifo */
	m_ESPcomm.SendString(message);

	return size;
}
}
