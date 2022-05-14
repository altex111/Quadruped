#include <memory.h>
#include <program/walk.h>

namespace quad
{
void Walk::PushUps()
{
	m_time = fmodf(m_time, mth::pi*2.0f);
	float lift = (1.0f + sinf(m_time*5.0f))*0.15f - 0.55f;

	m_quad.getLegRF().setPosition(mth::float3(0.9f, lift, 0.9f));
	m_quad.getLegLF().setPosition(mth::float3(-0.9f, lift, 0.9f));
	m_quad.getLegRB().setPosition(mth::float3(0.9f, lift, -0.9f));
	m_quad.getLegLB().setPosition(mth::float3(-0.9f, lift, -0.9f));
}
void Walk::Running()
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
void Walk::ToggleQuadEnabled()
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

Walk::Walk()
:	m_quad(),
	m_walkScipt(m_quad),
	m_quadEnabled(true),
	m_time(0.0f),
	m_walking(true)
{

}
void Walk::Start()
{
	m_quad.getLegRF().setPosition(mth::float3(0.9f, -0.35f, 0.9f));
	m_quad.getLegLF().setPosition(mth::float3(-0.9f, -0.35f, 0.9f));
	m_quad.getLegRB().setPosition(mth::float3(0.9f, -0.35f, -0.9f));
	m_quad.getLegLB().setPosition(mth::float3(-0.9f, -0.35f, -0.9f));
}
void Walk::Loop()
{
	m_time += 0.05f;

	if (m_quadEnabled)
	{
 		if (m_walking)
		{
			m_walkScipt.Update(0.02f);
		}
		else
		{
			mth::float2circle m_legCenterPos(1.3f, mth::pi*0.25f);
			float hight = 0.3;
			//m_quad.getLegLB().setJointStates(mth::float3(-mth::pi*0.25f, 0.5f, 1.3f));
			//m_quad.getLegRB().setJointStates(mth::float3(mth::pi*0.25f, 0.5f, 1.3f));
			//m_quad.getLegLF().setJointStates(mth::float3(mth::pi*0.25f, 0.5f, 1.3f));
			//m_quad.getLegRF().setJointStates(mth::float3(-mth::pi*0.25f, 0.5f, 1.3f));
//			m_quad.getLegLF().setPosition(mth::float3(-(m_walkScipt.m_legBasePos.y + m_legCenterPos.getY()), -hight, m_walkScipt.m_legBasePos.x + m_legCenterPos.getX()));
//			m_quad.getLegRB().getShoulderServo().setState(mth::pi * a);
//			PushUps();
		}
	}

#if 0
	/*todo this code should be places into the ros wrapper*/

	m_Esp.Process();
	ros_loop();

	if(m_quadEnabled != ros_quadEnabled)
	{
		ToggleQuadEnabled();
	}
	m_walking = ros_walking;



	ros_joints[0] = m_quad.getLegRF().getJointStates();
	ros_joints[1] = m_quad.getLegLF().getJointStates();
	ros_joints[2] = m_quad.getLegRB().getJointStates();
	ros_joints[3] = m_quad.getLegLB().getJointStates();
	float jointState[12] = {ros_joints[0].x, ros_joints[0].y, ros_joints[0].z, ros_joints[1].x, ros_joints[1].y, ros_joints[1].z, ros_joints[2].x, ros_joints[2].y, ros_joints[2].z, ros_joints[3].x, ros_joints[3].y, ros_joints[3].z};
	setJointState(jointState);

	ros_positions[0] = m_quad.getLegRF().getPosition();
	ros_positions[1] = m_quad.getLegLF().getPosition();
	ros_positions[2] = m_quad.getLegRB().getPosition();
	ros_positions[3] = m_quad.getLegLB().getPosition();
	float positionState[12] = {ros_positions[0].x, ros_positions[0].y, ros_positions[0].z, ros_positions[1].x, ros_positions[1].y, ros_positions[1].z, ros_positions[2].x, ros_positions[2].y, ros_positions[2].z, ros_positions[3].x, ros_positions[3].y, ros_positions[3].z};
	setPositionState(positionState);
	setPose(&poseArrayRF, &positionState[0]);
	setPose(&poseArrayLF, &positionState[3]);
	setPose(&poseArrayRB, &positionState[6]);
	setPose(&poseArrayLB, &positionState[9]);

	ros_set_joint.x = set_joints_x;
	ros_set_joint.y = set_joints_y;
	ros_set_joint.z = set_joints_z;

	if (m_quadEnabled)
	{
		if(setRF_joints_request)
		{
			setRF_joints_request = false;
			//m_quad.getLegRF().setJointStates(ros_set_joint);
			m_quad.getLegRF().MoveJoints(ros_set_joint);
		}

		if(setLF_joints_request)
		{
			setLF_joints_request = false;
			//m_quad.getLegLF().setJointStates(ros_set_joint);
			m_quad.getLegLF().MoveJoints(ros_set_joint);
		}

		if(setRB_joints_request)
		{
			setRB_joints_request = false;
			//m_quad.getLegRB().setJointStates(ros_set_joint);
			m_quad.getLegRB().MoveJoints(ros_set_joint);
		}

		if(setLB_joints_request)
		{
			setLB_joints_request = false;
			//m_quad.getLegLB().setJointStates(ros_set_joint);
			m_quad.getLegLB().MoveJoints(ros_set_joint);
		}
	}
#endif
}

void Walk::StoreCommand(WalkCommand command)
{
	if(WALK_COMMAND_FORWARD == command.direction)
	{
		m_walkScipt.AddPathElementWalkStraight(command.parameter);
	}
	else if(WALK_COMMAND_TURN == command.direction)
	{
		m_walkScipt.AddPathElementTurn(command.parameter);
	}
	else if(WALK_COMMAND_SET_ENABLE == command.direction)
	{
		bool enable = false;
		if(command.parameter > 0.0f)
		{
			enable = true;
		}
		if(m_quadEnabled != enable)
		{
			ToggleQuadEnabled();
		}
	}
	else if(WALK_COMMAND_SET_WALKING == command.direction)
	{
		bool walking = false;
		if(command.parameter > 0.0f)
		{
			walking = true;
		}
		m_walking = walking;
	}
	else
	{
		/* Invalid command */
	}
}

}
