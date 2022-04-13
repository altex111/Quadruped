#include "quad_movement.h"

namespace quad
{
#pragma region WalkStraight

	void WalkStraight::Part0(float delta)
	{
		float t, z;
		t = fmodf(m_time, 1.0f);
		z = -m_legZRetracted - (1.0f - t)*m_legStretchHalf * 2.0f;

		m_quad->getEntity().MoveForward(TimeInPrevPart(delta)*m_legStretchHalf);
		m_quad->getLegRF().setPosition({ m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf });
		m_quad->getLegRB().setPosition({ m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) });
		m_quad->getLegLF().setPosition({ -m_legXPos, -m_bellyy, m_legZRetracted });
		m_quad->getLegLB().setPosition({ -m_legXPos, Y(t), z });
	}
	void WalkStraight::Part1(float delta)
	{
		float t, z;
		t = fmodf(m_time, 1.0f);
		z = t * m_legStretchHalf*2.0f + m_legZRetracted;

		m_quad->getLegRF().setPosition({ m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf });
		m_quad->getLegRB().setPosition({ m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) });
		m_quad->getLegLF().setPosition({ -m_legXPos, Y(t), z });
		m_quad->getLegLB().setPosition({ -m_legXPos, -m_bellyy, -m_legZRetracted });
	}
	void WalkStraight::Part2(float delta)
	{
		float t = fmodf(m_time, 1.0f)*m_legStretchHalf;

		m_quad->getEntity().MoveForward(TimeInThisPart(delta)*m_legStretchHalf);
		m_quad->getLegRF().setPosition({ m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf - t });
		m_quad->getLegRB().setPosition({ m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) - t });
		m_quad->getLegLB().setPosition({ -m_legXPos, -m_bellyy, -m_legZRetracted - t });
		m_quad->getLegLF().setPosition({ -m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf * 2.0f - t });
	}
	void WalkStraight::Part3(float delta)
	{
		float t, z;
		t = fmodf(m_time, 1.0f);
		z = -m_legZRetracted - (1.0f - t)*m_legStretchHalf * 2.0f;

		m_quad->getEntity().MoveForward(TimeInPrevPart(delta)*m_legStretchHalf);
		m_quad->getLegRF().setPosition({ m_legXPos, -m_bellyy, m_legZRetracted });
		m_quad->getLegRB().setPosition({ m_legXPos, Y(t), z });
		m_quad->getLegLF().setPosition({ -m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf });
		m_quad->getLegLB().setPosition({ -m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) });
	}
	void WalkStraight::Part4(float delta)
	{
		float t, z;
		t = fmodf(m_time, 1.0f);
		z = t * m_legStretchHalf*2.0f + m_legZRetracted;

		m_quad->getLegRF().setPosition({ m_legXPos, Y(t), z });
		m_quad->getLegRB().setPosition({ m_legXPos, -m_bellyy, -m_legZRetracted });
		m_quad->getLegLF().setPosition({ -m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf });
		m_quad->getLegLB().setPosition({ -m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) });
	}
	void WalkStraight::Part5(float delta)
	{
		float t = fmodf(m_time, 1.0f)*m_legStretchHalf;

		m_quad->getEntity().MoveForward(TimeInThisPart(delta)*m_legStretchHalf);
		m_quad->getLegRF().setPosition({ m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf * 2.0f - t });
		m_quad->getLegRB().setPosition({ m_legXPos, -m_bellyy, -m_legZRetracted - t });
		m_quad->getLegLF().setPosition({ -m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf - t });
		m_quad->getLegLB().setPosition({ -m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) - t });
	}
	float WalkStraight::TimeInThisPart(float delta)
	{
		float tmp = fmodf(m_time, 1.0f);
		if (tmp - delta < 0.0f)
			return tmp;
		return delta;
	}
	float WalkStraight::TimeInPrevPart(float delta)
	{
		float tmp = fmodf(m_time, 1.0f);
		if (tmp - delta < 0.0f)
			return delta - tmp;
		return 0.0f;
	}
	float WalkStraight::Y(float t)
	{
		t = 2.0f*t - 1.0f;
		return -m_bellyy + (1.0f - t * t)*m_legLift;
	}
	void WalkStraight::Init(Quadruped* quadruped)
	{
		m_length = 6.0f;
		m_speed = 3.0f;
		m_time = 0.0f;
		m_bellyy = 0.3f;
		m_legLift = 0.2f;
		m_legXPos = 0.9f;
		m_legZRetracted = 0.4f;
		m_legStretchHalf = 0.5f;
		m_quad = quadruped;
		m_quad->getEntity().position = { 0.0f, m_bellyy, 0.0f };
		m_quad->getLegRF().setPosition({ m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf });
		m_quad->getLegRB().setPosition({ m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) });
		m_quad->getLegLF().setPosition({ -m_legXPos, -m_bellyy, m_legZRetracted });
		m_quad->getLegLB().setPosition({ -m_legXPos, -m_bellyy, -m_legZRetracted - m_legStretchHalf * 2.0f });
	}

	void WalkStraight::Update(float deltaTime)
	{
		float delta = deltaTime * m_speed;
		m_time = fmodf(m_time + delta, m_length);
		switch ((int)m_time)
		{
		case 0:
			Part0(delta);
			break;
		case 1:
			Part1(delta);
			break;
		case 2:
			Part2(delta);
			break;
		case 3:
			Part3(delta);
			break;
		case 4:
			Part4(delta);
			break;
		case 5:
			Part5(delta);
			break;
		}
	}

#pragma endregion

#pragma region WalkTurn

	float WalkLegPosScript::LegY(float t)
	{
		t = 2.0f*t - 1.0f;
		return -m_bellyy + (1.0f - t * t)*m_legLift;
	}
	void WalkLegPosScript::Init(Quadruped* quadruped)
	{
		m_length = 6.0f;
		m_speed = 3.0f;
		m_time = 0.0f;
		m_bellyy = 0.3f;
		m_legLift = 0.2f;
		m_legXPos = 0.7f;
		m_legZRetracted = 0.4f;
		m_legStretchHalf = 0.2f;
		m_radInv = 0.01f;
		m_quad = quadruped;
		m_quad->getEntity().position = { 0.0f, m_bellyy, 0.0f };
		m_quad->getLegRF().setPosition({ m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf });
		m_quad->getLegRB().setPosition({ m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) });
		m_quad->getLegLF().setPosition({ -m_legXPos, -m_bellyy, m_legZRetracted });
		m_quad->getLegLB().setPosition({ -m_legXPos, -m_bellyy, -m_legZRetracted - m_legStretchHalf * 2.0f });
	}
	void WalkLegPosScript::Update(float deltaTime)
	{
		float delta = deltaTime * m_speed;
		m_time = fmodf(m_time + delta, m_length);

	}

#pragma endregion

}
