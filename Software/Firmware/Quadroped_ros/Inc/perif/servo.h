#pragma once

#include "cmain.h"
#include <functional>

namespace quad
{
enum TimerChannel:uint32_t
{
	CHANNEL1 = 0,
	CHANNEL2 = 1,
	CHANNEL3 = 2,
	CHANNEL4 = 3
};

struct ServoInitStruct
{
	TIM_TypeDef *timer;
	TimerChannel channel;
	float assemblyOffset;
};

class Servo
{
	TIM_TypeDef *m_timer;
	TimerChannel m_channel;
	float m_assemblyOffset;
	float m_state;

public:
	Servo(ServoInitStruct sis);

	void setAssemblyOffset(float o);
	float getAssemblyOffset();
	void setState(float r);
	float getState();
	void Move(float delta);
	void Enable();
	void Disable();
};
}
