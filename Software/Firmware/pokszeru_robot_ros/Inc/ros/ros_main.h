/*
* ros_main.h
*
*  Created on: 2019/03/27
*      Author: Massar Lorant
*/
//#include "ros.h"				//NOTE: this is only needed for ros NodeHandle type, once a ROS class is created, should be deleted
#include <std_msgs/UInt8.h>		//NOTE: this is only needed for ros NodeHandle type, once a ROS class is created, should be deleted
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>

#ifndef ROS_MAIN_H_
#define ROS_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#if 0

	//extern ros::NodeHandle nh;	//NOTE: this is only needed for ros NodeHandle type, once a ROS class is created, should be deleted

	extern bool ros_quadEnabled;
	extern bool ros_walking;
	extern sensor_msgs::JointState jointState;

	void setJointState(float* joint);
	void setPositionState(float* position);

	void listenerCallback(const std_msgs::UInt8& msg);

	extern geometry_msgs::PoseArray poseArrayRF;
	extern geometry_msgs::PoseArray poseArrayLF;
	extern geometry_msgs::PoseArray poseArrayRB;
	extern geometry_msgs::PoseArray poseArrayLB;
	extern geometry_msgs::Pose poseArrayRF_position[3];
	extern geometry_msgs::Pose poseArrayLF_position[3];
	extern geometry_msgs::Pose poseArrayRB_position[3];
	extern geometry_msgs::Pose poseArrayLB_position[3];
	void setPose(geometry_msgs::PoseArray* pose_array, float* point);

	extern bool setRF_joints_request;
	extern bool setLF_joints_request;
	extern bool setRB_joints_request;
	extern bool setLB_joints_request;
	extern float set_joints_x;
	extern float set_joints_y;
	extern float set_joints_z;
	void setRFJointsCallback(const geometry_msgs::Vector3& joint);
	void setLFJointsCallback(const geometry_msgs::Vector3& joint);
	void setRBJointsCallback(const geometry_msgs::Vector3& joint);
	void setLBJointsCallback(const geometry_msgs::Vector3& joint);

	void publishDebugMessage(char* message);

	void ros_setup(void);
	void ros_loop(void);

#endif

#ifdef __cplusplus
}
#endif


#endif /* ROS_MAIN_H_ */
