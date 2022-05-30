#pragma once

#include "entity.h"
#include <array>

namespace quad
{
	enum LegID
	{
		LID_RF = 0,
		LID_LF = 1,
		LID_RB = 2,
		LID_LB = 3
	};

	class Leg
	{
		gfx::Entity m_shouder;
		gfx::Entity m_thigh;
		gfx::Entity m_toe;

		float m_a1, m_a2, m_a3;
		float m_ox, m_oy, m_oz;
		float m_o1, m_o2, m_o3;
		float m_o3x;

		mth::float3 m_joints[2];
		bool m_valid[2];
		mth::float3 m_position;
		int m_chosenJoint;

	private:
		void SetJointRotation();
		void ForwardGeometry();

		void CalculateKneeAngle(int index, float l);
		bool InverseBaseAngle();
		bool InverseShoulderAngle();
		void InverseKneeAngle();
		void InverseGeometry();

	public:
		bool InitRF(ID3D11Device *device);
		bool InitLF(ID3D11Device *device);
		bool InitRB(ID3D11Device *device);
		bool InitLB(ID3D11Device *device);

		void Install(gfx::Entity& body);

		void setJointStates(mth::float3 joints);
		void setJointStates(int index);
		mth::float3 getJointStates();
		mth::float3 getJointStates(int index);
		bool isValid();
		bool isValid(int index);
		void MoveJoints(mth::float3 delta);
		void setPosition(mth::float3 position);
		mth::float3 getPosition();
		void MovePosition(mth::float3 delta);
	};

	class Quadruped
	{
		gfx::Entity m_body;
		std::array<Leg, 4> m_legs;

	public:
		Quadruped() = default;
		bool Init(ID3D11Device* device);

		gfx::Entity& getEntity();
		Leg& getLegRF();
		Leg& getLegLF();
		Leg& getLegRB();
		Leg& getLegLB();
		Leg& getLeg(UINT index);
		std::array<Leg, 4>& getLegs();
	};
}