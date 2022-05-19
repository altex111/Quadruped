#pragma once

#include "cmain.h"
#include "util/linalg.h"
#include <functional>

namespace quadroped
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
	ServoInitStruct(): positiveLimit(mth::pi * 0.345/*0.345*/), negativeLimit(mth::pi * 0.345) {}
	TIM_TypeDef *timer;
	TimerChannel channel;
	float assemblyOffset;
	float positiveLimit;
	float negativeLimit;
};

class Servo
{
	TIM_TypeDef *m_timer;
	TimerChannel m_channel;
	float m_assemblyOffset;
	float m_positiveLimit;
	float m_negativeLimit;
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
