#include "perif/servo.h"
#include "util/linalg.h"
#include <array>

namespace quad
{
uint32_t DegToCompareValue(float deg, TIM_TypeDef* tim)
{
	uint32_t arr = LL_TIM_GetAutoReload(tim);
	//return ((3.0f + 4.0f*deg / mth::pi) * arr) / 20;
	return (1.5f + (deg / (mth::pi / 2.0f))) / 20.0f * arr;
}


typedef void (*ServoCHFunction)(TIM_TypeDef *timer, float state);
ServoCHFunction setServoState[] = {
	[](TIM_TypeDef *timer, float state)
		{ LL_TIM_OC_SetCompareCH1(timer, DegToCompareValue(state, timer)); },
	[](TIM_TypeDef *timer, float state)
		{ LL_TIM_OC_SetCompareCH2(timer, DegToCompareValue(state, timer)); },
	[](TIM_TypeDef *timer, float state)
		{ LL_TIM_OC_SetCompareCH3(timer, DegToCompareValue(state, timer)); },
	[](TIM_TypeDef *timer, float state)
		{ LL_TIM_OC_SetCompareCH4(timer, DegToCompareValue(state, timer)); }
};
uint32_t channelConvertedToLL[] = {
	LL_TIM_CHANNEL_CH1,
	LL_TIM_CHANNEL_CH2,
	LL_TIM_CHANNEL_CH3,
	LL_TIM_CHANNEL_CH4
};

Servo::Servo(ServoInitStruct sis)
:m_timer(sis.timer), m_channel(sis.channel), m_assemblyOffset(sis.assemblyOffset)
{
	setState(0.0f);
}

void Servo::setAssemblyOffset(float o)
{
	m_assemblyOffset = o;
}
float Servo::getAssemblyOffset()
{
	return m_assemblyOffset;
}
void Servo::setState(float r)
{
	m_state = r + m_assemblyOffset;
	if (m_state < -mth::pi)
		m_state = -mth::pi;
	else if (m_state > mth::pi)
		m_state = mth::pi;
	setServoState[m_channel](m_timer, m_state);
}
float Servo::getState()
{
	return m_state;
}
void Servo::Move(float delta)
{
	setState(m_state + delta);
}
void Servo::Enable()
{
	  LL_TIM_CC_EnableChannel(m_timer, channelConvertedToLL[m_channel]);
}
void Servo::Disable()
{
	  LL_TIM_CC_DisableChannel(m_timer, channelConvertedToLL[m_channel]);
}

}
