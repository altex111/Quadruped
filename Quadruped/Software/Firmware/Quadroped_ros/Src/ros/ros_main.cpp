/*
 * ros_main.cpp

 *
 *  Created on: 2019/03/27
 *      Author: Massar Lorant
 */
#include <ros/ros_main.h>
//#include "ros.h"
#include <std_msgs/String.h>

#include <std_msgs/UInt8.h>
#include <stdio.h>

#if 0
ros::NodeHandle nh;

std_msgs::String str_msg;
sensor_msgs::JointState jointState;
sensor_msgs::JointState positionState;
volatile float jointState_array[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
volatile float positionState_array[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

geometry_msgs::PoseArray poseArrayRF;
geometry_msgs::PoseArray poseArrayLF;
geometry_msgs::PoseArray poseArrayRB;
geometry_msgs::PoseArray poseArrayLB;
geometry_msgs::Pose poseArrayRF_position[3];
geometry_msgs::Pose poseArrayLF_position[3];
geometry_msgs::Pose poseArrayRB_position[3];
geometry_msgs::Pose poseArrayLB_position[3];

ros::Publisher debug("debug", &str_msg);
ros::Subscriber<std_msgs::UInt8> listener("listener", &listenerCallback);

bool setRF_joints_request = false;
bool setLF_joints_request = false;
bool setRB_joints_request = false;
bool setLB_joints_request = false;
float set_joints_x = 0.0f;
float set_joints_y = 0.0f;
float set_joints_z = 0.0f;
ros::Publisher joint("joint", &jointState);
ros::Publisher position("position", &positionState);
ros::Subscriber<geometry_msgs::Vector3> setRFJoints("setRFJoints", &setRFJointsCallback);
ros::Subscriber<geometry_msgs::Vector3> setLFJoints("setLFJoints", &setLFJointsCallback);
ros::Subscriber<geometry_msgs::Vector3> setRBJoints("setRBJoints", &setRBJointsCallback);
ros::Subscriber<geometry_msgs::Vector3> setLBJoints("setLBJoints", &setLBJointsCallback);

ros::Publisher poseRF("poseRF", &poseArrayRF);
ros::Publisher poseLF("poseLF", &poseArrayLF);
ros::Publisher poseRB("poseRB", &poseArrayRB);
ros::Publisher poseLB("poseLB", &poseArrayLB);

char hello[] = "Hello world!";
char still_alive[] = "Still alive: ";

uint16_t i = 0;
uint32_t alive_counter = 0;
bool ros_quadEnabled = false;
bool ros_walking = false;

void ros_setup(void)
{
	nh.initNode();
	nh.advertise(debug);
	nh.advertise(joint);
	nh.advertise(position);

	nh.advertise(poseRF);
	nh.advertise(poseLF);
	nh.advertise(poseRB);
	nh.advertise(poseLB);

	nh.subscribe(listener);
	nh.subscribe(setRFJoints);
	nh.subscribe(setLFJoints);
	nh.subscribe(setRBJoints);
	nh.subscribe(setLBJoints);

	jointState.position_length = 12u;
	jointState.position = (float *)jointState_array;

	positionState.position_length = 12u;
	positionState.position = (float *)positionState_array;

	poseArrayRF.poses_length = 3u;
	poseArrayRF.poses = poseArrayRF_position;
	poseArrayLF.poses_length = 3u;
	poseArrayLF.poses = poseArrayLF_position;
	poseArrayRB.poses_length = 3u;
	poseArrayRB.poses = poseArrayRB_position;
	poseArrayLB.poses_length = 3u;
	poseArrayLB.poses = poseArrayLB_position;

	poseArrayRF.header.frame_id = "map";
	poseArrayLF.header.frame_id = "map";
	poseArrayRB.header.frame_id = "map";
	poseArrayLB.header.frame_id = "map";
}

void ros_loop(void)
{
	//if(243 == i)
	if(195 == i)
	{
		char message[20];
		sprintf(message, "Still alive: %d", alive_counter);
		str_msg.data = message;
		alive_counter++;

		debug.publish(&str_msg);
		joint.publish(&jointState);
		position.publish(&positionState);

		//poseRF.publish(&poseArrayRF);
		//poseLF.publish(&poseArrayLF);
		//poseRB.publish(&poseArrayRB);
		//poseLB.publish(&poseArrayLB);

		i = 0;
	}
	else
	{
		i++;
	}

	/*todo test*/
	if(0 == i%117)
	{
		poseRF.publish(&poseArrayRF);
		poseLF.publish(&poseArrayLF);
		poseRB.publish(&poseArrayRB);
		poseLB.publish(&poseArrayLB);
	}
	/*todo test*/

	nh.spinOnce();
}

void publishDebugMessage(char* message)
{
	std_msgs::String str_msg;
	str_msg.data = message;
	debug.publish(&str_msg);
}

void listenerCallback(const std_msgs::UInt8& msg)
{
	char message[100];

	if(0 == msg.data)
	{
		sprintf(message, "Your number was: %d, ros_quadEnabled is set to true", msg.data);
		ros_quadEnabled = true;
	}
	if(1 == msg.data)
	{
		sprintf(message, "Your number was: %d, ros_quadEnabled is set to false", msg.data);
		ros_quadEnabled = false;
	}
	if(2 == msg.data)
	{
		sprintf(message, "Your number was: %d, ros_walking is set to true", msg.data);
		ros_walking = true;
	}
	if(3 == msg.data)
	{
		sprintf(message, "Your number was: %d, ros_walking is set to false", msg.data);
		ros_walking = false;
	}
	else if(100 > msg.data)
	{
		sprintf(message, "Your number was: %d", msg.data);
	}
	else
	{
		sprintf(message, "The number was too big");
	}

	publishDebugMessage(message);
}

void setJointState(float* joint)
{
	for(uint8_t position_counter = 0; position_counter < jointState.position_length; position_counter++)
	{
		jointState.position[position_counter] = joint[position_counter];
	}
}

void setPositionState(float* position)
{
	for(uint8_t position_counter = 0; position_counter < positionState.position_length; position_counter++)
	{
		positionState.position[position_counter] = position[position_counter];
	}
}

void setPose(geometry_msgs::PoseArray* pose_array, float* point)
{
	pose_array->poses[0].position.x = point[2];		//z
	pose_array->poses[0].position.y = -point[0];		//x
	pose_array->poses[0].position.z = point[1];		//y
	pose_array->poses[1].position.x = point[2];
	pose_array->poses[1].position.y = -point[0];
	pose_array->poses[1].position.z = point[1];
	pose_array->poses[2].position.x = point[2];
	pose_array->poses[2].position.y = -point[0];
	pose_array->poses[2].position.z = point[1];

	pose_array->poses[0].orientation.x = 0;
	pose_array->poses[0].orientation.y = 0;
	pose_array->poses[0].orientation.z = 0;
	pose_array->poses[0].orientation.w = 1;
	pose_array->poses[1].orientation.x = 0;
	pose_array->poses[1].orientation.y = 0;
	pose_array->poses[1].orientation.z = 0;
	pose_array->poses[1].orientation.w = 1;
	pose_array->poses[2].orientation.x = 0;
	pose_array->poses[2].orientation.y = 0;
	pose_array->poses[2].orientation.z = 0;
	pose_array->poses[2].orientation.w = 1;
}

void setRFJointsCallback(const geometry_msgs::Vector3& joint)
{
	char message[100];

	setRF_joints_request = true;
	sprintf(message, "Right, front joints set function was called\nx: %f\ny: %f\nz: %f", joint.x, joint.y, joint.z);
	publishDebugMessage(message);

	set_joints_x = joint.x;
	set_joints_y = joint.y;
	set_joints_z = joint.z;
}

void setLFJointsCallback(const geometry_msgs::Vector3& joint)
{
	char message[100];

	setLF_joints_request = true;
	sprintf(message, "Left, front joints set function was called\nx: %f\ny: %f\nz: %f", joint.x, joint.y, joint.z);
	publishDebugMessage(message);

	set_joints_x = joint.x;
	set_joints_y = joint.y;
	set_joints_z = joint.z;
}

void setRBJointsCallback(const geometry_msgs::Vector3& joint)
{
	char message[100];

	setRB_joints_request = true;
	sprintf(message, "Right, back joints set function was called\nx: %f\ny: %f\nz: %f", joint.x, joint.y, joint.z);
	publishDebugMessage(message);

	set_joints_x = joint.x;
	set_joints_y = joint.y;
	set_joints_z = joint.z;
}

void setLBJointsCallback(const geometry_msgs::Vector3& joint)
{
	char message[100];

	setLB_joints_request = true;
	sprintf(message, "Left, back joints set function was called\nx: %f\ny: %f\nz: %f", joint.x, joint.y, joint.z);
	publishDebugMessage(message);

	set_joints_x = joint.x;
	set_joints_y = joint.y;
	set_joints_z = joint.z;
}

#endif
