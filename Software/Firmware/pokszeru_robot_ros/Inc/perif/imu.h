#pragma once

#include "cmain.h"
#include "perif/uart.h"
#include <functional>
#include "usart.h"
#include "util/fifo.h"

#include "FreeRTOS.h"
#include "task.h"

namespace quad
{

class IMU
{
private:
	UART m_BNOcomm;

	ImuRequest imu_inRequestDebug = IMU_REQUEST_DEFAULT;
	ImuRequest imu_outRequestDebug = IMU_REQUEST_DEFAULT;
	ImuRequest imu_outRequestQuaternion = IMU_REQUEST_DEFAULT;

	const ImuRequest imu_inRequestQuaternion = {
													.header = IMUREQUEST_HEADER_START,
													.direction = IMUREQUEST_DIRECTION_READ,
													.address = QUA_DATA_W_LSB,
													.length = 8u
												};

	bool imu_debug_request = false;
	uint8_t imu_current_page = IMUREQUEST_CURRENTPAGE_0;

	bool ProcessResponse(const ImuRequest& request, ImuRequest& response, uint32_t timeout);
	void WaitForResponse(char* buffer, uint32_t size);
	bool WaitForResponseWithTimeout(char* buffer, uint32_t size, uint32_t timeout);
	bool ChangeRegisterPage(uint8_t page);

public:
	IMU(USART_TypeDef *uart);
	UART& getUART();

	ImuRequest& GetImu_inRequestDebug() {return imu_inRequestDebug;}
	ImuRequest& GetImu_outRequestDebug() {return imu_outRequestDebug;}
	ImuRequest& GetImu_outRequestQuaternion() {return imu_outRequestQuaternion;}

	bool& GetImu_debugRequest() {return imu_debug_request;}
	void SetImu_debugRequest() {imu_debug_request = true;}
	void ClearImu_debugRequest() {imu_debug_request = false;}

	bool Start();
	void Process();
};

}
