#include "perif/imu.h"

#define IMU_1_MS_WAIT			1
#define IMU_2_MS_WAIT			2
#define IMU_CALIBRATIONWAIT		500u
#define IMU_CALIBRATIONRESET	60000u
#define IMU_REQUESTTIMEOUT		10u

namespace quad
{
IMU::IMU(USART_TypeDef *uart):
		m_BNOcomm(uart)
{

}

UART& IMU::getUART()
{
	return m_BNOcomm;
}

bool IMU::Start()
{
	/* Create a ImuRequest variable for requests */
	ImuRequest imu_RequestStartRequest = IMU_REQUEST_DEFAULT;
	ImuRequest imu_RequestStartResponse = IMU_REQUEST_DEFAULT;

	/* Clear uart in fifo */
	m_BNOcomm.ClearInFifo();

	/* Reset the BNO055 chip */
	LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_2);
	vTaskDelay( 1 / portTICK_PERIOD_MS );
	LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_2);
	vTaskDelay( 500 / portTICK_PERIOD_MS );

	/* PAGE 0 is selected after startup */
	imu_current_page = IMUREQUEST_CURRENTPAGE_0;

#if DEBUG_CODE_ACTIVE
	/* Set register page to PAGE 0 */
	if(true == ChangeRegisterPage(IMUREQUEST_CURRENTPAGE_0))
	{
		imu_current_page = IMUREQUEST_CURRENTPAGE_0;
	}
	else
	{
		/* todo error report */
	}
#endif

	/* todo bist + register */

	/* Get operation mode */
	imu_RequestStartRequest.direction = IMUREQUEST_DIRECTION_READ;
	imu_RequestStartRequest.address = OPR_MODE;
	imu_RequestStartRequest.length = 1u;
	bool opModeRequest = false;
	while(false == opModeRequest)
	{
		m_BNOcomm.SendData((char*)&imu_RequestStartRequest, FREERTOS_IMUREQUEST_FIX_SIZE);
		opModeRequest = ProcessResponse(imu_RequestStartRequest, imu_RequestStartResponse, IMU_REQUESTTIMEOUT);
	}

	if(IMUREQUEST_OPERATIONMODE_CONFIGMODE != (imu_RequestStartResponse.data[0] & OPR_MODE_MASK))
	{
		/* Change to operation mode config if oepration mode is set to something else */
		imu_RequestStartRequest.direction = IMUREQUEST_DIRECTION_WRITE;
		imu_RequestStartRequest.address = OPR_MODE;
		imu_RequestStartRequest.length = 1u;
		imu_RequestStartRequest.data[0] = IMUREQUEST_OPERATIONMODE_CONFIGMODE;
		m_BNOcomm.SendData((char*)&imu_RequestStartRequest, (FREERTOS_IMUREQUEST_FIX_SIZE + imu_RequestStartRequest.length));
		ProcessResponse(imu_RequestStartRequest, imu_RequestStartResponse, 0u);
	}

	/* Set power mode */
	imu_RequestStartRequest.direction = IMUREQUEST_DIRECTION_WRITE;
	imu_RequestStartRequest.address = PWR_MODE;
	imu_RequestStartRequest.length = 1u;
	imu_RequestStartRequest.data[0] = IMUREQUEST_POWERMODE_NORMAL;
	m_BNOcomm.SendData((char*)&imu_RequestStartRequest, (FREERTOS_IMUREQUEST_FIX_SIZE + imu_RequestStartRequest.length));
	ProcessResponse(imu_RequestStartRequest, imu_RequestStartResponse, 0u);

	/* Set operation mode to NDOF */
	imu_RequestStartRequest.direction = IMUREQUEST_DIRECTION_WRITE;
	imu_RequestStartRequest.address = OPR_MODE;
	imu_RequestStartRequest.length = 1u;
	imu_RequestStartRequest.data[0] = IMUREQUEST_OPERATIONMODE_NDOF;
	m_BNOcomm.SendData((char*)&imu_RequestStartRequest, (FREERTOS_IMUREQUEST_FIX_SIZE + imu_RequestStartRequest.length));
	ProcessResponse(imu_RequestStartRequest, imu_RequestStartResponse, 0u);

	/* Wait for calibration */
	imu_RequestStartRequest.direction = IMUREQUEST_DIRECTION_READ;
	imu_RequestStartRequest.address = CALIB_STAT;
	imu_RequestStartRequest.length = 1u;
	bool calibrationDone = false;
	for(uint32_t calibrationTime = 0; true; calibrationTime += IMU_CALIBRATIONWAIT)
	{
		m_BNOcomm.SendData((char*)&imu_RequestStartRequest, FREERTOS_IMUREQUEST_FIX_SIZE);
		calibrationDone = ProcessResponse(imu_RequestStartRequest, imu_RequestStartResponse, IMU_REQUESTTIMEOUT);

		if((calibrationDone) && (IMUREQUEST_HEADER_RESPONSE == imu_RequestStartResponse.header) && (CALIBRATION_DONE_VALUE == (imu_RequestStartResponse.data[0] & CALIBRATION_DONE_VALUE)))
		{
			/* Calibration done*/
			break;
		}

		calibrationDone = false;
		m_BNOcomm.ClearInFifo();
		vTaskDelay( IMU_CALIBRATIONWAIT / portTICK_PERIOD_MS );

		if(calibrationTime > IMU_CALIBRATIONRESET)
		{
			return false;
		}
	}

	return true;
}

void IMU::Process()
{
	if(IMUREQUEST_CURRENTPAGE_0 == imu_current_page)
	{
		bool qDataRequest = false;
		while(false == qDataRequest)
		{
			/* Send quaternion data request */
			m_BNOcomm.SendData((char*)&imu_inRequestQuaternion, FREERTOS_IMUREQUEST_FIX_SIZE);

			/* The response will be placed into the second argument of this function call */
			qDataRequest = ProcessResponse(imu_inRequestQuaternion, imu_outRequestQuaternion, IMU_REQUESTTIMEOUT);
			if(false == qDataRequest)
			{
				m_BNOcomm.ClearInFifo();
			}
		}
	}

	/* Check if there is a debug request */
	if(imu_debug_request)
	{
		if(IMUREQUEST_HEADER_START_PAGE1 == imu_inRequestDebug.header)
		{
			/* PAGE 1 is requested */
			/* Register page should be changed to PAGE 1 and then back to PAGE 0 */

			/* Correct the request header */
			imu_inRequestDebug.header = IMUREQUEST_HEADER_START;

			/* Send page change request */
			if(false == ChangeRegisterPage(IMUREQUEST_CURRENTPAGE_1))
			{
				/* todo error report */
				return;
			}
		}

		bool qDataRequest = false;
		while(false == qDataRequest)
		{
			/* Send quaternion data request */
			if(IMUREQUEST_DIRECTION_WRITE == imu_inRequestDebug.direction)
			{
				m_BNOcomm.SendData((char*)&imu_inRequestDebug, (FREERTOS_IMUREQUEST_FIX_SIZE + imu_inRequestDebug.length));
			}
			else
			{
				m_BNOcomm.SendData((char*)&imu_inRequestDebug, FREERTOS_IMUREQUEST_FIX_SIZE);
			}

			/* The response will be placed into the second argument of this function call */
			qDataRequest = ProcessResponse(imu_inRequestDebug, imu_outRequestDebug, IMU_REQUESTTIMEOUT);
			if(false == qDataRequest)
			{
				m_BNOcomm.ClearInFifo();
			}
		}

		if(IMUREQUEST_CURRENTPAGE_1 == imu_current_page)
		{
			/* Send page change request */
			if(true == ChangeRegisterPage(IMUREQUEST_CURRENTPAGE_0))
			{
				/* todo error report */
				return;
			}
		}
	}
}

bool IMU::ProcessResponse(const ImuRequest& request, ImuRequest& response, uint32_t timeout)
{
	bool returnValue = false;

	if(0u == timeout)
	{
		/* Wait for the header of the response */
		WaitForResponse((char*)&response, 1u);
		returnValue = true;
	}
	else
	{
		/* Wait for the header of the response with timeout */
		if(false == WaitForResponseWithTimeout((char*)&response, 1u, timeout))
		{
			return returnValue;
		}
		returnValue = true;
	}

	/* Set the direction field to result to indicate that this request is processed */
	response.direction = IMUREQUEST_DIRECTION_RESULT;
	/* Set the register address field to the value of the original request */
	response.address = request.address;

	/* Check response */
	if(IMUREQUEST_HEADER_RESPONSE == response.header)
	{
		if(0u == timeout)
		{
			/* Wait for the length byte to arrive */
			WaitForResponse((char*)&(response.length), 1);

			/* Wait for every data byte to arrive */
			WaitForResponse((char*)&(response.data[0]), response.length);
		}
		else
		{
			/* Wait for the length byte to arrive */
			if(false == WaitForResponseWithTimeout((char*)&(response.length), 1, timeout))
			{
				return false;
			}

			/* Wait for every data byte to arrive */
			if(false == WaitForResponseWithTimeout((char*)&(response.data[0]), response.length, timeout))
			{
				return false;
			}
		}
	}
	else if(IMUREQUEST_HEADER_RESULT == response.header)
	{
		response.length = 1u;

		if(0u == timeout)
		{
			WaitForResponse((char*)&(response.data[0]), 1u);
		}
		else
		{
			if(false == WaitForResponseWithTimeout((char*)&(response.data[0]), 1u, timeout))
			{
				return false;
			}
		}
	}
	else
	{
		/* Invalid response */
	}

	return returnValue;
}

void IMU::WaitForResponse(char* buffer, uint32_t size)
{
	while(false == m_BNOcomm.ReadBuffer(buffer, size))
	{
		vTaskDelay( IMU_1_MS_WAIT / portTICK_PERIOD_MS );
	}
}

bool IMU::WaitForResponseWithTimeout(char* buffer, uint32_t size, uint32_t timeout)
{
	bool returnValue = false;

	for(uint32_t timeoutCounter = 0u; timeoutCounter < timeout; timeoutCounter++)
	{
		if(m_BNOcomm.ReadBuffer(buffer, size))
		{
			returnValue = true;
			break;
		}
		vTaskDelay( IMU_1_MS_WAIT / portTICK_PERIOD_MS );
	}

	return returnValue;
}

bool IMU::ChangeRegisterPage(uint8_t page)
{
	bool returnValue = false;

	ImuRequest imu_ChnagePageRequest = {
											.header = IMUREQUEST_HEADER_START,
											.direction = IMUREQUEST_DIRECTION_WRITE,
											.address = PAGE_ID,
											.length = 1u,
											.data = {page}
										};

	uint8_t changePageResult_header;
	uint8_t changePageResult_status;

	m_BNOcomm.SendData((char*)&imu_ChnagePageRequest, (FREERTOS_IMUREQUEST_FIX_SIZE + imu_ChnagePageRequest.length));

	/* Wait for the header of the response */
	WaitForResponse((char*)&changePageResult_header, 1u);
	/* Wait for the status of the response */
	WaitForResponse((char*)&changePageResult_status, 1u);

	if(WRITE_SUCCESS == changePageResult_status)
	{
		/* Store the currently selected register page */
		imu_current_page = page;
		returnValue = true;
	}

	return returnValue;
}

}
