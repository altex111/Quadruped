/*
 * ros_wrapper.cpp
 *
 *  Created on: 2020. okt. 09.
 *      Author: Massár Lóránt
 */

#include "ros/ros_wrapper.h"

namespace ros
{

//void imu_inDebugCallback(const std_msgs::String& imu_debug_msg); todo

ROSNode::ROSNode()
: m_Esp(USART3), schedulerEventBits(0u),
  debug("debug", &str_msg),
  imu_outQuaternion("imu_outQuaternion", &imu_quaternion_msg),
  imu_outDebug("imu_outDebug", &imu_debug_msg),
  imu_inDebug("imu_inDebug", &ROSNode::imu_inDebugCallback, this),
  imu_reset_counter(0u),
  lidar_outScan("lidar_outScan", &lidar_scan),
  walk_inCommand("walk_inCommand", &ROSNode::walk_inCommandCallback, this),
  frame_id("map"),
  walk_command_forward("forward"),
  walk_command_left("left"),
  walk_command_right("right"),
  walk_command_enable("enable"),
  walk_command_walking("walking"),
  walk_command_status(false)
  /*imu_debug_msg("")*/
{
	nh.getHardware()->setESP(&m_Esp);

	str_msg.data = "Debug topic working!";

	imu_quaternion_msg.pose.position.x = 0.0f;
	imu_quaternion_msg.pose.position.y = 0.0f;
	imu_quaternion_msg.pose.position.z = 0.0f;

	imu_quaternion_msg.header.frame_id = frame_id;
}

void ROSNode::ROSSetup()
{
	m_Esp.Start();
	nh.initNode();

	/* Advertise publishers */
	nh.advertise(debug);
	nh.advertise(imu_outQuaternion);
	nh.advertise(imu_outDebug);
	nh.advertise(lidar_outScan);

	/* Subscribe */
	nh.subscribe(imu_inDebug);
	nh.subscribe(walk_inCommand);

	m_Esp.Process();
}

void ROSNode::ROSLoop()
{
	m_Esp.Process();

	if(SCHEDULEREVENT_IMU_BIT == (schedulerEventBits & SCHEDULEREVENT_IMU_BIT))
	{
		if(DEBUG_REQUEST_STATUS_ARRIVED == imu_debug_status)
		{
			char tmp_string[32];

			for(uint32_t dataCounter = 0, stringCounter = 0; dataCounter < FREERTOS_IMUREQUEST_FIX_SIZE + imu_inRequestDebug.length; dataCounter++, stringCounter+=2)
			{
				tmp_string[stringCounter] = ((((uint8_t*)&imu_inRequestDebug)[dataCounter] >> 4) & 0x0F) +'0';
				tmp_string[stringCounter+1] = ((((uint8_t*)&imu_inRequestDebug)[dataCounter]) & 0x0F) +'0';
			}

			imu_debug_msg.data = tmp_string;

			/* Publish debug request response */
			imu_outDebug.publish(&imu_debug_msg);
		}

		if(IMUREQUEST_HEADER_RESPONSE == imu_inRequestQuaternion.header)
		{
			/* Create quaternion request response */
			imu_quaternion_msg.pose.orientation.w = (float)((int16_t)(imu_inRequestQuaternion.data[0u] + (imu_inRequestQuaternion.data[1u]<<8)));
			imu_quaternion_msg.pose.orientation.x = (float)((int16_t)(imu_inRequestQuaternion.data[2u] + (imu_inRequestQuaternion.data[3u]<<8)));
			imu_quaternion_msg.pose.orientation.y = (float)((int16_t)(imu_inRequestQuaternion.data[4u] + (imu_inRequestQuaternion.data[5u]<<8)));
			imu_quaternion_msg.pose.orientation.z = (float)((int16_t)(imu_inRequestQuaternion.data[6u] + (imu_inRequestQuaternion.data[7u]<<8)));

			imu_outQuaternion.publish(&imu_quaternion_msg);
		}
	}

	if(SCHEDULEREVENT_LIDAR_BIT == (schedulerEventBits & SCHEDULEREVENT_LIDAR_BIT))
	{
		lidar_scan.header.stamp = nh.now();
		lidar_scan.header.stamp += Duration(1, 0);
		lidar_outScan.publish(&lidar_scan);
	}

	if(SCHEDULEREVENT_IMU_CALIBRATION_BIT == (schedulerEventBits & SCHEDULEREVENT_IMU_CALIBRATION_BIT))
	{
		publishDebugMessage("IMU calibration succeeded");
	}

	if(SCHEDULEREVENT_IMU_CALIBRFAIL_BIT == (schedulerEventBits & SCHEDULEREVENT_IMU_CALIBRFAIL_BIT))
	{
		char message[50];
		imu_reset_counter++;

		sprintf(message, "IMU calibration fail counter: %d", imu_reset_counter);

		publishDebugMessage("IMU calibration failed! -- Reseting IMU chip");
		publishDebugMessage(message);
	}

	nh.spinOnce();

	m_Esp.Process();
}

void ROSNode::imu_inDebugCallback(const std_msgs::String& imu_debug_msg)
{
	/* Copy the received request */
	imu_outRequestDebug.header = ((imu_debug_msg.data[0]-'0') << 4) + (imu_debug_msg.data[1]-'0');
	imu_outRequestDebug.direction = ((imu_debug_msg.data[2]-'0') << 4) + (imu_debug_msg.data[3]-'0');
	imu_outRequestDebug.address = ((imu_debug_msg.data[4]-'0') << 4) + (imu_debug_msg.data[5]-'0');
	imu_outRequestDebug.length = ((imu_debug_msg.data[6]-'0') << 4) + (imu_debug_msg.data[7]-'0');
	if(IMUREQUEST_DIRECTION_WRITE == imu_outRequestDebug.header)
	{
		for(uint32_t dataCounter = 0; dataCounter < imu_outRequestDebug.length; dataCounter+=2)
		{
			imu_outRequestDebug.data[dataCounter] = ((imu_debug_msg.data[dataCounter+8]-'0') << 4) + (imu_debug_msg.data[dataCounter+9]-'0');
		}
	}

	/* Set the imu_debug_status to DEBUG_REQUEST_STATUS_REQUESTED */
	imu_debug_status = DEBUG_REQUEST_STATUS_REQUESTED;
}

void ROSNode::walk_inCommandCallback(const std_msgs::String& walk_command_msg)
{
	const char* commandParameterPosition = NULL;
	float commandParameter = 0.0f;

	if(0 == strncmp(walk_command_msg.data, walk_command_forward, strlen(walk_command_forward)))
	{
		walk_Command.direction = WALK_COMMAND_FORWARD;

		commandParameterPosition = &walk_command_msg.data[8];
		commandParameter = atof(commandParameterPosition);
		walk_Command.parameter = commandParameter;
	}
	else if(0 == strncmp(walk_command_msg.data, walk_command_left, strlen(walk_command_left)))
	{
		walk_Command.direction = WALK_COMMAND_TURN;

		commandParameterPosition = &walk_command_msg.data[5];
		commandParameter = atof(commandParameterPosition);
		commandParameter = commandParameter/180*mth::pi;
		walk_Command.parameter = commandParameter;
	}
	else if(0 == strncmp(walk_command_msg.data, walk_command_right, strlen(walk_command_right)))
	{
		walk_Command.direction = WALK_COMMAND_TURN;

		commandParameterPosition = &walk_command_msg.data[6];
		commandParameter = atof(commandParameterPosition);
		commandParameter = commandParameter/180*mth::pi;
		walk_Command.parameter = -commandParameter;
	}
	else if(0 == strncmp(walk_command_msg.data, walk_command_enable, strlen(walk_command_enable)))
	{
		walk_Command.direction = WALK_COMMAND_SET_ENABLE;

		commandParameterPosition = &walk_command_msg.data[7];
		commandParameter = atof(commandParameterPosition);
		walk_Command.parameter = commandParameter;

		publishDebugMessage("Enable message received:");
		debug.publish(&walk_command_msg);
	}
	else if(0 == strncmp(walk_command_msg.data, walk_command_walking, strlen(walk_command_walking)))
	{
		walk_Command.direction = WALK_COMMAND_SET_WALKING;

		commandParameterPosition = &walk_command_msg.data[8];
		commandParameter = atof(commandParameterPosition);
		walk_Command.parameter = commandParameter;

		publishDebugMessage("Walking message received:");
		debug.publish(&walk_command_msg);
	}
	else
	{
		/* Invalid command */
		/* todo error report */
		return;
	}

	/* Set the walk_command_status */
	walk_command_status = true;
}

}
