/*
 * common.h
 *
 *  Created on: Nov 16, 2019
 *      Author: LORANT
*/
#pragma once

#include "program/debug.h"
#include "cmain.h"

/* #### MACROS #### */
/* FreeRTOS related defines */
/* FreRTOS Event Group */
#define SCHEDULEREVENT_WIFI_BIT				( 1 << 0 )
#define SCHEDULEREVENT_IMU_BIT				( 1 << 1 )
#define SCHEDULEREVENT_LIDAR_BIT			( 1 << 2 )
#define SCHEDULEREVENT_WAIT_ROS_INIT_BIT	( 1 << 3 )
#define SCHEDULEREVENT_IMU_CALIBRATION_BIT	( 1 << 4 )
#define SCHEDULEREVENT_IMU_CALIBRFAIL_BIT	( 1 << 5 )

/* FreeRTOS Message Buffer */
#define FREERTOS_MESSAGEBUFFER_LENGTH_SMALL		128u
#define FREERTOS_MESSAGEBUFFER_LENGTH_BIG		1024u

/* FreeRTOS IMU Request */
#define FREERTOS_IMUREQUEST_DATA_SIZE		12u
#define FREERTOS_IMUREQUEST_FIX_SIZE		4u
#define FREERTOS_IMUREQUEST_BUFFER_SIZE		(FREERTOS_IMUREQUEST_FIX_SIZE + FREERTOS_IMUREQUEST_DATA_SIZE)

#define IMUREQUEST_HEADER_START				(0xAA)
#define IMUREQUEST_HEADER_RESPONSE			(0xBB)
#define IMUREQUEST_HEADER_RESULT			(0xEE)
#define IMUREQUEST_HEADER_START_PAGE1		(0xFF)	/* Not a valid value for the BNO055 chip, used to indicate that the requested register is in PAGE 1 insted of PAGE 0*/

#define IMUREQUEST_DIRECTION_WRITE			(0x00)
#define IMUREQUEST_DIRECTION_READ			(0x01)
#define IMUREQUEST_DIRECTION_RESULT			(0xFF)	/* Not a valid value for the BNO055 chip, used in result, since this field is irrelevant then */

#define IMUREQUEST_CURRENTPAGE_0			(0x00)
#define IMUREQUEST_CURRENTPAGE_1			(0x01)

#define IMUREQUEST_POWERMODE_NORMAL			(0x00)

#define IMUREQUEST_OPERATIONMODE_CONFIGMODE	(0x00)
#define IMUREQUEST_OPERATIONMODE_NDOF		(0x0C)

/* Register */
#define PAGE_ID								(0x07)
/* Register Map 0 */
#define CHIP_ID								(0x00)
#define QUA_DATA_W_LSB						(0x20)
#define QUA_DATA_W_MSB						(0x21)
#define QUA_DATA_X_LSB						(0x22)
#define QUA_DATA_X_MSB						(0x23)
#define QUA_DATA_Y_LSB						(0x24)
#define QUA_DATA_Y_MSB						(0x25)
#define QUA_DATA_Z_LSB						(0x26)
#define QUA_DATA_Z_MSB						(0x27)
#define CALIB_STAT							(0x35)
#define CALIBRATION_DONE_VALUE					(0xC0)
#define OPR_MODE							(0x3D)
#define OPR_MODE_MASK							(0x0F)
#define PWR_MODE							(0x3E)
#define PWR_MODE_MASK							(0x03)

/* BNO055 response */
#define WRITE_SUCCESS						(0x01)
#define READ_FAIL							(0x02)
#define WRITE_FAIL							(0x03)
#define REGMAP_INVALID_ADDRESS				(0x04)
#define REGMAP_WRITE_DISABLED				(0x05)
#define WRONG_START_BYTE					(0x06)
#define BUS_OVER_RUN_ERROR					(0x07)
#define MAX_LENGTH_ERROR					(0x08)
#define MIN_LENGTH_ERROR					(0x09)
#define RECEIVE_CHARACTER_TIMEOUT			(0x0A)

/* YDLIDAR */
#define LIDAR_PACKET_HEADER					(0x55AA)
#define LIDAR_MAX_PACKET_NUMBER				1088

/* Communication related defines */
#define UART_INDATA_LENGTH			256u
#define UART_OUTDATA_LENGTH			128u
#define UART_OUTDATA_LENGTH_DIGITS	3u
#define ESP_BUFFER_MAX_LENTH		2048u
#define ESP_BUFFER_MAX_LENTH_DIGITS	5u
#define UART_OUTDATA_MESSAGE_LENGTH	(strlen("AT+CIPSEND=") + (ESP_BUFFER_MAX_LENTH_DIGITS) + strlen("\r\n") + 1u)

#define WIFI_INDATA_LENGTH			384u
#define WIFI_OUTDATA_LENGTH			4352u

#define LIDAR_INDATA_LENGTH			256u

#define TOPIC_INDATA_LENGTH			384u

/* Walking related defines */
#define WALK_COMMAND_SET_ENABLE		0u
#define WALK_COMMAND_SET_WALKING	1u
#define WALK_COMMAND_FORWARD		2u
#define WALK_COMMAND_TURN			3u
/* #### #### */

/* #### TYPE DEFINITIONS #### */
/* FreeRTOS related defines */
typedef struct __attribute__((__packed__))ImuRequest{
	uint8_t header;
	uint8_t direction;
	uint8_t address;
	uint8_t length;
	uint8_t	data[FREERTOS_IMUREQUEST_DATA_SIZE];
} Imu_request;

#define IMU_REQUEST_DEFAULT {.header = IMUREQUEST_HEADER_START, .direction = IMUREQUEST_DIRECTION_READ, .address = CHIP_ID, .length = 0u}

typedef struct __attribute__((__packed__))WalkCommand{
	float x;
	float z;
	float phi;
} WalkCommand;
/* #### #### */
