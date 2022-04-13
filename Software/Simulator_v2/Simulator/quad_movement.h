#pragma once

#include "quad_quad.h"
#include <queue>
#include <array>

namespace quad
{
	class WalkStraight
	{
		float m_length;
		float m_time;
		float m_speed;
		float m_bellyy;
		float m_legLift;
		float m_legXPos;
		float m_legZRetracted;
		float m_legStretchHalf;
		Quadruped* m_quad;

	private:
		void Part0(float delta);
		void Part1(float delta);
		void Part2(float delta);
		void Part3(float delta);
		void Part4(float delta);
		void Part5(float delta);

		float TimeInThisPart(float delta);
		float TimeInPrevPart(float delta);
		float Y(float t);

	public:
		void Init(Quadruped* quadruped);
		void Update(float deltaTime);
	};

	class WalkLegPosScript
	{
		float m_length;
		float m_time;
		float m_speed;
		float m_bellyy;
		float m_legLift;
		float m_legXPos;
		float m_legZRetracted;
		float m_legStretchHalf;
		float m_radInv;
		Quadruped* m_quad;

		std::array<std::queue<mth::float3>, 4> m_legPositions;

	private:
		float LegY(float t);

	public:
		void Init(Quadruped* quadruped);
		void Update(float deltaTime);

		void Spin(float angle);
	};
}