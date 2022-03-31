#include "program/walkprogram.h"
#include <memory.h>

const uint8_t CMD_get = 0x01;
const uint8_t CMD_set = 0x02;
const uint8_t CMD_joints = 0x10;
const uint8_t CMD_positions = 0x20;
const uint8_t CMD_offsets = 0x40;

namespace quad
{
void WalkProgram::PushUps()
{
	m_time = fmodf(m_time, mth::pi*2.0f);
	float lift = (1.0f + sinf(m_time*5.0f))*0.15f - 0.55f;

	m_quad.getLegRF().setPosition(mth::float3(0.9f, lift, 0.9f));
	m_quad.getLegLF().setPosition(mth::float3(-0.9f, lift, 0.9f));
	m_quad.getLegRB().setPosition(mth::float3(0.9f, lift, -0.9f));
	m_quad.getLegLB().setPosition(mth::float3(-0.9f, lift, -0.9f));
}
void WalkProgram::Running()
{
	m_time = fmodf(m_time, 2.0f);
	float y1 = 2.0f*fmodf(m_time, 1.0f) - 1.0f;
	y1 = -0.4f + (1.0f - y1 * y1)*0.25f;
	float y2;
	if (m_time > 1.0f)
	{
		y2 = -0.4f;
	}
	else
	{
		y2 = y1;
		y1 = -0.4f;
	}
	float x = fabsf(1.0f - m_time);

	m_quad.getLegRF().setPosition(mth::float3(0.9f, y1, 0.4f + x));
	m_quad.getLegLF().setPosition(mth::float3(-0.9f, y2, 0.4f + x));
	m_quad.getLegRB().setPosition(mth::float3(0.9f, y2, -0.4f - x));
	m_quad.getLegLB().setPosition(mth::float3(-0.9f, y1, -0.4f - x));
}
void WalkProgram::ToggleQuadEnabled()
{
	if (m_quadEnabled)
	{
		m_quad.DisableLegs();
		m_quadEnabled = false;
	}
	else
	{
		m_quad.EnableLegs();
		m_quadEnabled = true;
	}
}
void WalkProgram::HandleUARTMessage()
{
	if (m_msgSize == 1)
	{
		if (m_PCcomm.ReadBuffer(&m_cmd, m_msgSize))
		{
			if (m_cmd & CMD_set)
			{
				m_msgSize = 48;
			}
			else
			{
				mth::float3 states[4];
				uint8_t cmd = m_cmd & 0xf0;
				switch (cmd)
				{
				case CMD_joints:
					states[0] = m_quad.getLegRF().getJointStates();
					states[1] = m_quad.getLegLF().getJointStates();
					states[2] = m_quad.getLegRB().getJointStates();
					states[3] = m_quad.getLegLB().getJointStates();
					break;
				case CMD_positions:
					states[0] = m_quad.getLegRF().getPosition();
					states[1] = m_quad.getLegLF().getPosition();
					states[2] = m_quad.getLegRB().getPosition();
					states[3] = m_quad.getLegLB().getPosition();
					break;
				case CMD_offsets:
					states[0].x = m_quad.getLegRF().getBaseServo().getAssemblyOffset();
					states[0].y = m_quad.getLegRF().getShoulderServo().getAssemblyOffset();
					states[0].z = m_quad.getLegRF().getKneeServo().getAssemblyOffset();
					states[1].x = m_quad.getLegLF().getBaseServo().getAssemblyOffset();
					states[1].y = m_quad.getLegLF().getShoulderServo().getAssemblyOffset();
					states[1].z = m_quad.getLegLF().getKneeServo().getAssemblyOffset();
					states[2].x = m_quad.getLegRB().getBaseServo().getAssemblyOffset();
					states[2].y = m_quad.getLegRB().getShoulderServo().getAssemblyOffset();
					states[2].z = m_quad.getLegRB().getKneeServo().getAssemblyOffset();
					states[3].x = m_quad.getLegLB().getBaseServo().getAssemblyOffset();
					states[3].y = m_quad.getLegLB().getShoulderServo().getAssemblyOffset();
					states[3].z = m_quad.getLegLB().getKneeServo().getAssemblyOffset();
					break;
				}
				m_PCcomm.SendData(m_cmd);
				m_PCcomm.SendData((uint8_t*)states, sizeof(states));
			}
		}
	}
	else
	{
		mth::float3 states[4];
		if (m_PCcomm.ReadBuffer((uint8_t*)states, 48))
		{
			uint8_t cmd = m_cmd & 0xf0;
			switch (cmd)
			{
			case CMD_joints:
				m_quad.getLegRF().setJointStates(states[0]);
				m_quad.getLegLF().setJointStates(states[1]);
				m_quad.getLegRB().setJointStates(states[2]);
				m_quad.getLegLB().setJointStates(states[3]);
				break;
			case CMD_positions:
				m_quad.getLegRF().setPosition(states[0]);
				m_quad.getLegLF().setPosition(states[1]);
				m_quad.getLegRB().setPosition(states[2]);
				m_quad.getLegLB().setPosition(states[3]);
				break;
			case CMD_offsets:
				m_quad.getLegRF().getBaseServo().setAssemblyOffset(states[0].x);
				m_quad.getLegRF().getShoulderServo().setAssemblyOffset(states[1].y);
				m_quad.getLegRF().getKneeServo().setAssemblyOffset(states[2].z);
				m_quad.getLegLF().getBaseServo().setAssemblyOffset(states[3].x);
				m_quad.getLegLF().getShoulderServo().setAssemblyOffset(states[4].y);
				m_quad.getLegLF().getKneeServo().setAssemblyOffset(states[5].z);
				m_quad.getLegRB().getBaseServo().setAssemblyOffset(states[6].x);
				m_quad.getLegRB().getShoulderServo().setAssemblyOffset(states[7].y);
				m_quad.getLegRB().getKneeServo().setAssemblyOffset(states[8].z);
				m_quad.getLegLB().getBaseServo().setAssemblyOffset(states[9].x);
				m_quad.getLegLB().getShoulderServo().setAssemblyOffset(states[10].y);
				m_quad.getLegLB().getKneeServo().setAssemblyOffset(states[11].z);
				break;
			}
			m_msgSize = 1;
		}
	}
}

WalkProgram::WalkProgram()
:	Program(),
	m_time(0.0f),
	m_msgSize(1),
	m_cmd(0),
	m_walking(false)
{}
void WalkProgram::Start()
{
	m_quad.getLegRF().setPosition(mth::float3(0.9f, -0.35f, 0.9f));
	m_quad.getLegLF().setPosition(mth::float3(-0.9f, -0.35f, 0.9f));
	m_quad.getLegRB().setPosition(mth::float3(0.9f, -0.35f, -0.9f));
	m_quad.getLegLB().setPosition(mth::float3(-0.9f, -0.35f, -0.9f));

	quad::IO::btn2.setCallback([this](){ToggleQuadEnabled();});
	quad::IO::btn1.setCallback([this](){m_walking = !m_walking;});
}
void WalkProgram::Loop()
{
	m_time += 0.05f;

	HandleUARTMessage();

	if (m_quadEnabled)
	{
 		if (m_walking)
		{
			m_walkScipt.Update(0.02f);
		}
		else
		{
			m_quad.getLegLB().setJointStates(mth::float3(-mth::pi*0.25f, 0.5f, 1.3f));
			m_quad.getLegRB().setJointStates(mth::float3(mth::pi*0.25f, 0.5f, 1.3f));
			m_quad.getLegLF().setJointStates(mth::float3(mth::pi*0.25f, 0.5f, 1.3f));
			m_quad.getLegRF().setJointStates(mth::float3(-mth::pi*0.25f, 0.5f, 1.3f));
			//PushUps();
		}
	}
}

}
