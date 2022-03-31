#pragma once

#include "cmain.h"
#include "perif/uart.h"
#include <functional>
#include "usart.h"
#include "util/fifo.h"

#include "sensor_msgs/LaserScan.h"
#include "CYdLidar.h"

#include "FreeRTOS.h"
#include "task.h"

namespace quad
{

class LIDAR
{
private:
	UART m_LIDARcomm;

	util::fifo<uint16_t, LIDAR_INDATA_LENGTH> lidar_inData;

	/* YDLIDAR packet related variables */
	float IntervalSampleAngle_LastPackage = 0.0f;

	/* CYdLidar related variables */
	CYdLidar laser;
	size_t rxProcessCounter = 0u;
	LaserScan scan;

	/* ROS message related variables */
	sensor_msgs::LaserScan scan_msg;
	float scan_msg_ranges[LIDAR_MAX_PACKET_NUMBER];
	char frame_id[20];

	/* This function should return true, when a start packet is found */
	bool Rx_ProcessFindStart();

	/* This function should return true, when a full measurement is found */
	bool Rx_ProcessMeasurement(node_info *nodebuffer, size_t &count);
	void Pop_InData(uint16_t& inData);

public:
	LIDAR(USART_TypeDef *uart);
	UART& getUART();

	sensor_msgs::LaserScan& GetLidar_LaserScan() {return scan_msg;}

	void Start();
	bool Process();
};

}
