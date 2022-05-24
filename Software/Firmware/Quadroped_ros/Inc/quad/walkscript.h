#pragma once

#include "quad/quadruped.h"
#include "util/fifo.h"

#include <list>

namespace quadroped
{
struct QuadAction
{
	int32_t legID;
	mth::float2 goalPos;
	float rot;
};

class WalkScript
{
	util::fifo<QuadAction, 128> m_script;
	QuadAction m_action;
	QuadAction m_prevAction;
	Quadruped& m_quad;
	float m_time;
	float m_speed;
	float m_maxTurnAtOnce;
	float m_bellyy;
	float m_legLift;

	float m_legXPos;
	float m_legZRetracted;
	float m_legStretchHalf;

	float m_criticalAngle;
	float m_legMaxStretchHalf;
	float m_legRReachOffset;
	float m_legRReachMax;
	float m_EPS;

	bool m_rightBalanced;
	bool m_running;

public:
	mth::float2 m_legBasePos;
	mth::float2circle m_legCenterPos;

private:
	void AddPathElementLegMovement(LegID legid, mth::float2 pos);
	void AddPathElementBodyMovement(mth::float2 pos, float turn);
	void AddLegTurnRightBalanced(mth::float2x2 rotmat);
	void AddLegTurnLeftBalanced(mth::float2x2 rotmat);
	void AddLegBodyElementsTurn(float angle);
	void AddLegWalkStraightRightBalanced(float ratio);
	void AddLegWalkStraightLeftBalanced(float ratio);
	void AddLegBodyElementsWalkStraight(float distance);
	void AddLegBodyElementsMove(uint8_t *legCount, float legStretchHalf, float turnAtOnce, mth::float2 motionDirection, quadroped::LegID* stepOrder);
	void calculateOptimalsteps(float *legStretchHalf, float *turnAtOnce, float distance, float relativeHeadding);
	float calculateMaxLegStretchHalf(mth::float2 motionDirection);
	float legStretchHalf(mth::float2 motionDirection, quadroped::LegID legId);
	std::list<mth::float2> findTrajectoryIntersections(mth::float2 motionDirection, mth::float2 section);
	mth::float3 getLegRFStartPos();
	mth::float3 getLegLFStartPos();
	mth::float3 getLegRBStartPos();
	mth::float3 getLegLBStartPos();
	float getBellyY();
	float LegY(float t);
	bool NextAction(QuadAction& action);
	void MoveBody(float deltaTime);
	void MoveLeg();
	void ReceiveNextAction();
	float FinishPreviousAction(float timeLeft);
	void ExecuteAction(float deltaTime);

public:
	WalkScript(Quadruped& q);
	void AddPathElementTurn(float angle);
	void AddPathElementWalkStraight(float distance);
	void AddPathElementCircle(float circleR, float rotation);
	void AddPathElementMove(mth::float2 relativePos, float relativeHeadding = 0);
	void Clear();
	void Update(float deltaTime);
};

}
