#pragma once

#include "perif/servo.h"
#include "util/linalg.h"

namespace quad
{
enum LegID
{
	LID_RF = 0,
	LID_LF = 1,
	LID_RB = 2,
	LID_LB = 3
};

struct LegInitStruct
{
	ServoInitStruct baseServo;
	ServoInitStruct shoulderServo;
	ServoInitStruct kneeServo;
	float dir1, dir2, dir3;
	float a1, a2, a3;
	float ox, oy, oz;
	float o1, o2, o3;
	float o3x;
};

LegInitStruct GetLegRBInitStruct();
LegInitStruct GetLegLBInitStruct();
LegInitStruct GetLegRFInitStruct();
LegInitStruct GetLegLFInitStruct();

class Leg
{
	Servo m_base;
	Servo m_shoulder;
	Servo m_knee;

	float m_dir1, m_dir2, m_dir3;
	float m_a1, m_a2, m_a3;
	float m_ox, m_oy, m_oz;
	float m_o1, m_o2, m_o3;
	float m_o3x;

	mth::float3 m_joints[2];
	bool m_valid[2];
	mth::float3 m_position;
	uint32_t m_chosenJoint;

private:
	void SetJointRotation();
	void ForwardGeometry();

	void CalculateKneeAngle(uint32_t index, float l);
	bool InverseBaseAngle();
	bool InverseShoulderAngle();
	void InverseKneeAngle();
	void InverseGeometry();

public:
	Leg(LegInitStruct lis);
	void setJointStates(mth::float3 joints);
	void setJointStates(uint32_t index);
	mth::float3 getJointStates();
	mth::float3 getJointStates(uint32_t index);
	bool isValid();
	bool isValid(uint32_t index);
	void MoveJoints(mth::float3 delta);
	void setPosition(mth::float3 position);
	mth::float3 getPosition();
	void MovePosition(mth::float3 delta);
	void Enable();
	void Disable();
	Servo& getBaseServo();
	Servo& getShoulderServo();
	Servo& getKneeServo();
};

}
