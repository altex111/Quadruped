#pragma once

#include "quad/walkscript.h"
#include "perif/io.h"
#include "cmain.h"
#include <cmath>

namespace quadroped
{

class Walk
{
	quadroped::Quadruped m_quad;
	quadroped::WalkScript m_walkScipt;
	bool m_quadEnabled;

	float m_time;
	bool m_walking;

	mth::float3 ros_joints[4];
	mth::float3 ros_positions[4];
	mth::float3 ros_set_joint;

private:
	void PushUps();
	void Running();
	void ToggleQuadEnabled();

public:
	Walk();
	~Walk() = default;
	void Start();
	void Loop();
	void StoreCommand(WalkCommand command);
};

}
