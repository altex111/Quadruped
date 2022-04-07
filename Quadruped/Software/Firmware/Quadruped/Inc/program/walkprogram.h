#pragma once

#include "program.h"

namespace quad
{

class WalkProgram : public Program
{
	float m_time;
	uint32_t m_msgSize;
	uint8_t m_cmd;
	bool m_walking;

private:
	void PushUps();
	void Running();
	void ToggleQuadEnabled();
	void HandleUARTMessage();

public:
	WalkProgram();
	virtual ~WalkProgram() override = default;
	virtual void Start() override;
	virtual void Loop() override;
};

}
