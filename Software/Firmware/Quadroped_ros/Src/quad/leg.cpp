#include <quad/leg.h>
#include <cmath>

namespace quad
{

LegInitStruct GetLegRBInitStruct()
{
	LegInitStruct lis;
	lis.baseServo.timer = TIM1;
	lis.baseServo.channel = TimerChannel::CHANNEL1;
	lis.baseServo.assemblyOffset = -0.15f;
	lis.shoulderServo.timer = TIM3;
	lis.shoulderServo.channel = TimerChannel::CHANNEL4;
	lis.shoulderServo.assemblyOffset = 0.0f;
	lis.kneeServo.timer = TIM3;
	lis.kneeServo.channel = TimerChannel::CHANNEL3;
	lis.kneeServo.assemblyOffset = -0.15f;
	lis.ox = 0.35f;
	lis.oy = 0.21f;
	lis.oz = -0.4f;
	lis.o1 = 0.18f;
	lis.o2 = 0.4f;
	lis.o3 = 0.8f;
	lis.o3x = -0.04f;
	lis.a1 = mth::pi*0.75f;
	lis.a2 = 0.0f;
	lis.a3 = mth::pi*0.25f;
	lis.dir1 = 1.0f;
	lis.dir2 = -1.0f;
	lis.dir3 = 1.0f;
	return lis;
}
LegInitStruct GetLegLBInitStruct()
{
	LegInitStruct lis;
	lis.baseServo.timer = TIM2;
	lis.baseServo.channel = TimerChannel::CHANNEL4;
	lis.baseServo.assemblyOffset = 0.0f;
	lis.shoulderServo.timer = TIM3;
	lis.shoulderServo.channel = TimerChannel::CHANNEL2;
	lis.shoulderServo.assemblyOffset = 0.0f;
	lis.kneeServo.timer = TIM3;
	lis.kneeServo.channel = TimerChannel::CHANNEL1;
	lis.kneeServo.assemblyOffset = -0.2f;
	lis.ox = -0.35f;
	lis.oy = 0.21f;
	lis.oz = -0.4f;
	lis.o1 = 0.18f;
	lis.o2 = 0.4f;
	lis.o3 = 0.8f;
	lis.o3x = 0.04f;
	lis.a1 = -mth::pi*0.75f;
	lis.a2 = 0.0f;
	lis.a3 = mth::pi*0.25f;
	lis.dir1 = 1.0f;
	lis.dir2 = 1.0f;
	lis.dir3 = -1.0f;
	return lis;
}
LegInitStruct GetLegRFInitStruct()
{
	LegInitStruct lis;
	lis.baseServo.timer = TIM1;
	lis.baseServo.channel = TimerChannel::CHANNEL4;
	lis.baseServo.assemblyOffset = -0.1f;
	lis.shoulderServo.timer = TIM1;
	lis.shoulderServo.channel = TimerChannel::CHANNEL3;
	lis.shoulderServo.assemblyOffset = -0.2f;
	lis.kneeServo.timer = TIM1;
	lis.kneeServo.channel = TimerChannel::CHANNEL2;
	lis.kneeServo.assemblyOffset = 0.0f;
	lis.ox = 0.35f;
	lis.oy = 0.21f;
	lis.oz = 0.4f;
	lis.o1 = 0.18f;
	lis.o2 = 0.4f;
	lis.o3 = 0.8f;
	lis.o3x = 0.04f;
	lis.a1 = mth::pi*0.25f;
	lis.a2 = 0.0f;
	lis.a3 = mth::pi*0.25f;
	lis.dir1 = 1.0f;
	lis.dir2 = 1.0f;
	lis.dir3 = -1.0f;
	return lis;
}
LegInitStruct GetLegLFInitStruct()
{
	LegInitStruct lis;
	lis.baseServo.timer = TIM2;
	lis.baseServo.channel = TimerChannel::CHANNEL3;
	lis.baseServo.assemblyOffset = 0.0f;
	lis.shoulderServo.timer = TIM2;
	lis.shoulderServo.channel = TimerChannel::CHANNEL2;
	lis.shoulderServo.assemblyOffset = -0.15f;
	lis.kneeServo.timer = TIM2;
	lis.kneeServo.channel = TimerChannel::CHANNEL1;
	lis.kneeServo.assemblyOffset = 0.0f;
	lis.ox = -0.35f;
	lis.oy = 0.21f;
	lis.oz = 0.4f;
	lis.o1 = 0.18f;
	lis.o2 = 0.4f;
	lis.o3 = 0.8f;
	lis.o3x = -0.04f;
	lis.a1 = -mth::pi*0.25f;
	lis.a2 = 0.0f;
	lis.a3 = mth::pi*0.25f;
	lis.dir1 = 1.0f;
	lis.dir2 = -1.0f;
	lis.dir3 = 1.0f;
	return lis;
}

Leg::Leg(LegInitStruct lis)
:m_base(lis.baseServo),
 m_shoulder(lis.shoulderServo),
 m_knee(lis.kneeServo)
{
	m_ox = lis.ox;
	m_oy = lis.oy;
	m_oz = lis.oz;
	m_o1 = lis.o1;
	m_o2 = lis.o2;
	m_o3 = lis.o3;
	m_o3x = lis.o3x;
	m_a1 = lis.a1;
	m_a2 = lis.a2;
	m_a3 = lis.a3;
	m_dir1 = lis.dir1;
	m_dir2 = lis.dir2;
	m_dir3 = lis.dir3;

	setJointStates(mth::float3());
}

void Leg::SetJointRotation()
{
	m_base.setState(m_dir1*m_joints[m_chosenJoint].x);
	m_shoulder.setState(m_dir2*m_joints[m_chosenJoint].y);
	m_knee.setState(m_dir3*m_joints[m_chosenJoint].z);
}

void Leg::ForwardGeometry()
{
	float cos1 = cosf(m_a1 + m_joints[0].x);
	float cos2 = cosf(m_a2 + m_joints[0].y);
	float cos23 = cosf(m_a2 + m_joints[0].y + m_a3 + m_joints[0].z);
	float sin1 = sinf(m_a1 + m_joints[0].x);
	float sin2 = sinf(m_a2 + m_joints[0].y);
	float sin23 = sinf(m_a2 + m_joints[0].y + m_a3 + m_joints[0].z);

	m_position.x = cos1 * m_o3x + sin1 * cos23*m_o3 + sin1 * cos2*m_o2 + sin1 * m_o1 + m_ox;
	m_position.y = -sin23 * m_o3 - sin2 * m_o2 + m_oy;
	m_position.z = -sin1 * m_o3x + cos1 * cos23*m_o3 + cos1 * cos2*m_o2 + cos1 * m_o1 + m_oz;

	m_chosenJoint = 0;
	m_valid[0] = true;
	m_valid[1] = false;
}

bool Leg::InverseBaseAngle()
{
	float a, b, d, discriminant, drt, sol;
	a = m_oz - m_position.z;
	b = m_position.x - m_ox;
	d = m_o3x;
	discriminant = a * a + b * b - d * d;
	if (discriminant < 0.0f)
		return false;
	drt = sqrtf(discriminant);
	sol = atan2f((a*d + b * drt), (b * d - a * drt));
	if (m_a1 > mth::pi*0.5f && sol < 0.0f)
		sol += mth::pi*2.0f;
	else if (m_a1 < -mth::pi*0.5f && sol > 0.0f)
		sol -= mth::pi*2.0f;
	if (sol < m_a1 - mth::pi*0.5f || sol > m_a1 + mth::pi*0.5f)
	{
		sol = atan2f((a*d - b * drt), (b * d + a * drt));
		if (m_a1 > mth::pi*0.5f && sol < 0.0f)
			sol += mth::pi*2.0f;
		else if (m_a1 < -mth::pi*0.5f && sol > 0.0f)
			sol -= mth::pi*2.0f;
		if (sol < m_a1 - mth::pi*0.5f || sol > m_a1 + mth::pi*0.5f)
			return false;
	}
	m_joints[0].x = sol;
	m_joints[1].x = sol;
	return true;
}

bool Leg::InverseShoulderAngle()
{
	float a, b, d, discriminant, drt;
	a = m_oy - m_position.y;
	b = (m_position.x - m_ox - cosf(m_joints[0].x)*m_o3x) / sinf(m_joints[0].x) - m_o1;
	d = (b*b + a * a + m_o2 * m_o2 - m_o3 * m_o3) / (2.0f*m_o2);
	discriminant = a * a + b * b - d * d;
	if (discriminant < 0.0f)
		return false;
	drt = sqrtf(discriminant);
	m_joints[0].y = atan2f((a*d + b * drt), (b * d - a * drt));
	m_joints[1].y = atan2f((a*d - b * drt), (b * d + a * drt));
	m_valid[0] = (m_joints[0].y >= m_a2 - mth::pi*0.5f &&  m_joints[0].y <= m_a2 + mth::pi*0.5f);
	m_valid[1] = (m_joints[1].y >= m_a2 - mth::pi*0.5f &&  m_joints[1].y <= m_a2 + mth::pi*0.5f);

	if (!m_valid[0] && !m_valid[1])
		return false;
	return true;
}

void Leg::CalculateKneeAngle(uint32_t index, float l)
{
	if (m_valid[index])
	{
		m_joints[index].z = atan2f(m_oy - m_position.y - sinf(m_joints[index].y)*m_o2, l - cosf(m_joints[index].y)*m_o2) - m_joints[index].y;
		m_joints[index].x -= m_a1;
		m_joints[index].y -= m_a2;
		m_joints[index].z -= m_a3;
		m_valid[index] = (m_joints[index].z >= m_a3 - mth::pi*0.5f) && (m_joints[index].z <= m_a3 + mth::pi*0.5f);
		if (m_valid[index])
			m_chosenJoint = index;
	}
}
void Leg::InverseKneeAngle()
{
	float l = (m_position.x - m_ox - cosf(m_joints[0].x)*m_o3x) / sinf(m_joints[0].x) - m_o1;
	m_chosenJoint = -1;
	CalculateKneeAngle(0, l);
	CalculateKneeAngle(1, l);
}
void Leg::InverseGeometry()
{
	m_valid[0] = false;
	m_valid[1] = false;

	if (InverseBaseAngle())
		if (InverseShoulderAngle())
			InverseKneeAngle();
}

void Leg::setJointStates(mth::float3 joints)
{
	m_joints[0] = joints;
	ForwardGeometry();
	SetJointRotation();
}
void Leg::setJointStates(uint32_t index)
{
	m_chosenJoint = index;
	SetJointRotation();
}
mth::float3 Leg::getJointStates()
{
	return m_joints;
}
mth::float3 Leg::getJointStates(uint32_t index)
{
	return m_joints[index];
}
bool Leg::isValid()
{
	return m_chosenJoint > 0;
}
bool Leg::isValid(uint32_t index)
{
	return m_valid[index];
}
void Leg::MoveJoints(mth::float3 delta)
{
	m_joints[0] = m_joints[m_chosenJoint] + delta;
	ForwardGeometry();
	SetJointRotation();
}
void Leg::setPosition(mth::float3 position)
{
	m_position = position;
	InverseGeometry();
	SetJointRotation();
}
mth::float3 Leg::getPosition()
{
	return m_position;
}
void Leg::MovePosition(mth::float3 delta)
{
	setPosition(m_position + delta);
}
void Leg::Enable()
{
	m_base.Enable();
	m_shoulder.Enable();
	m_knee.Enable();
}
void Leg::Disable()
{
	m_base.Disable();
	m_shoulder.Disable();
	m_knee.Disable();
}
Servo& Leg::getBaseServo()
{
	return m_base;
}
Servo& Leg::getShoulderServo()
{
	return m_shoulder;
}
Servo& Leg::getKneeServo()
{
	return m_knee;
}

}
