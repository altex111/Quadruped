/*
 * ros_wrapper.h
 *
 *  Created on: 2020. okt. 09.
 *      Author: Mass�r L�r�nt
 */

#pragma once

#include "ros/node_handle.h"
#include "ros/STM32Hardware.h"
#include "perif/esp.h"
#include <string.h>
#include "util/linalg.h"

/* todo delete*/
#include "FreeRTOS.h"
#include "event_groups.h"
/* todo delete*/

/* Ros types */
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include "sensor_msgs/LaserScan.h"

#define DEBUG_REQUEST_STATUS_IDLE			0u
#define DEBUG_REQUEST_STATUS_REQUESTED		1u
#define DEBUG_REQUEST_STATUS_IN_PROGRESS	2u
#define DEBUG_REQUEST_STATUS_ARRIVED		3u

namespace ros
{
typedef NodeHandle_<quad::STM32Hardware> NodeHandle; // default 25, 25, 512, 512

class ROSNode
{
private:
	ros::NodeHandle nh;
	quad::ESP m_Esp;

	EventBits_t schedulerEventBits;

	/* Debug topic for general purposes */
	ros::Publisher debug;
	std_msgs::String str_msg;

	/* Imus related topics */
	ros::Publisher imu_outQuaternion;
	ros::Publisher imu_outDebug;
	geometry_msgs::PoseStamped imu_quaternion_msg;
	char frame_id[4];
	std_msgs::String imu_debug_msg;

	ros::Subscriber<std_msgs::String, ROSNode> imu_inDebug;
	void imu_inDebugCallback(const std_msgs::String& imu_debug_msg);

	ImuRequest imu_inRequestDebug = IMU_REQUEST_DEFAULT;
	ImuRequest imu_outRequestDebug = IMU_REQUEST_DEFAULT;
	ImuRequest imu_inRequestQuaternion = IMU_REQUEST_DEFAULT;

	uint8_t imu_debug_status = DEBUG_REQUEST_STATUS_IDLE;

	uint32_t imu_reset_counter;

	/* Lidar related topics */
	ros::Publisher lidar_outScan;
	sensor_msgs::LaserScan lidar_scan;

	/* Walk related topics */
	ros::Subscriber<std_msgs::String, ROSNode> walk_inCommand;
	void walk_inCommandCallback(const std_msgs::String& walk_command_msg);

	const char walk_command_forward[8];
	const char walk_command_left[5];
	const char walk_command_right[6];
	const char walk_command_enable[7];
	const char walk_command_walking[8];
	WalkCommand walk_Command;

	bool walk_command_status;

	void publishDebugMessage(char* message)
	{
		std_msgs::String str_msg;
		str_msg.data = message;
		debug.publish(&str_msg);
	}

public:
	ROSNode();
	inline quad::ESP& getESP() {return m_Esp;}

	EventBits_t& Get_schedulerEventBits() {return schedulerEventBits;}

	ImuRequest& GetImu_inRequestDebug() {return imu_inRequestDebug;}
	ImuRequest& GetImu_outRequestDebug() {return imu_outRequestDebug;}
	ImuRequest& GetImu_inRequestQuaternion() {return imu_inRequestQuaternion;}

	sensor_msgs::LaserScan& GetLidar_LidarScan() {return lidar_scan;}

	WalkCommand& GetWalk_Command() {return walk_Command;}

	uint8_t& GetImu_debugRequest() {return imu_debug_status;}

	bool& GetWalk_commandRequest() {return walk_command_status;}

	void ROSSetup();
	void ROSLoop();
};

}
