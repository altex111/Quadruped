#include <quad/quadruped.h>

namespace quad
{
Quadruped::Quadruped()
:m_legs({Leg(GetLegRFInitStruct()),Leg(GetLegLFInitStruct()),Leg(GetLegRBInitStruct()),Leg(GetLegLBInitStruct())})
{
	//DisableLegs();
}
Leg& Quadruped::getLegRF()
{
	return m_legs[LID_RF];
}
Leg& Quadruped::getLegLF()
{
	return m_legs[LID_LF];
}
Leg& Quadruped::getLegRB()
{
	return m_legs[LID_RB];
}
Leg& Quadruped::getLegLB()
{
	return m_legs[LID_LB];
}
Leg& Quadruped::getLeg(uint32_t index)
{
	return m_legs[index];
}
std::array<Leg, 4>& Quadruped::getLegs()
{
	return m_legs;
}
void Quadruped::EnableLegs()
{
	for (auto& l : m_legs)
		l.Enable();
}
void Quadruped::DisableLegs()
{
	for (auto& l : m_legs)
		l.Disable();
}

} /* namespace quad */
