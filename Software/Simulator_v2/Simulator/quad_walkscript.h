#pragma once

#include "quad_quad.h"
#include <list>

namespace quad
{
	struct QuadAction
	{
		int legID;
		mth::float2 goalPos;
		float rot;
	};
	
	class WalkScript
	{
		float m_maxTurnAtOnce;
		float m_criticalAngle;

		float m_bellyy;
		float m_legLift;
		float m_legXPos;
		float m_legZRetracted;
		float m_legStretchHalf;

		float m_legXBasePos;
		float m_legZBasePos;
		float m_legMaxStretchHalf;
		mth::float2 m_legBasePos;
		mth::float2circle m_legCenterPos;
		float m_legRReachOffset;
		float m_legRReachMax;
		float m_EPS;
		
	private:
		bool m_rightBalanced;
		std::list<QuadAction> m_script;

	private:
		void AddPathElementLegMovement(LegID legid, mth::float2 pos);
		void AddPathElementBodyMovement(mth::float2 pos, float turn);
		void AddLegTurnRightBalanced(mth::float2x2 rotmat);
		void AddLegTurnLeftBalanced(mth::float2x2 rotmat);
		void AddLegBodyElementsTurn(float angle);
		void AddLegWalkStraightRightBalanced(float ratio);
		void AddLegWalkStraightLeftBalanced(float ratio);
		void AddLegBodyElementsWalkStraight(float distance);

		void AddLegBodyElementsMove(float *distance,uint8_t *legCount, float legStretchHalf, float turnAtOnce, mth::float2 motionDirection, quad::LegID* stepOrder, uint8_t motionFlag);

		void calculateOptimalsteps(float *legStretchHalf, float *turnAtOnce, float distance, float relativeHeadding);
		float calculateMaxLegStretchHalf(mth::float2 motionDirection);
		float legStretchHalf(mth::float2 motionDirection, quad::LegID legId);
		std::list<mth::float2> findTrajectoryIntersections(mth::float2 motionDirection, mth::float2 section);
		mth::float2x2 CircleLineIntersection(float rCircle, mth::float2 normalVector, mth::float2 pointLine, float eps);
		mth::float2x2 LineXYAxisIntersection(float mLine, mth::float2 pointLine);
		mth::float2 RelativePointOnACircle(mth::float2 pointOnCircle, float angle);
		mth::float2 SubtractPoints(mth::float2 pont1, mth::float2 point2);
	public:
		WalkScript();
		void AddPathElementTurn(float angle);
		void AddPathElementWalkStraight(float distance);
		void AddPathElementCircle(float circleR, float rotation);
		void AddPathElementMove(mth::float2 relativePos, float relativeHeadding = 0);

		void Clear();
		bool NextAction(QuadAction& action);
		mth::float3 getLegRFStartPos();
		mth::float3 getLegLFStartPos();
		mth::float3 getLegRBStartPos();
		mth::float3 getLegLBStartPos();
		float getBellyY();
		float LegY(float t);
	};

	class WalkManager
	{
		float m_time;
		float m_speed;
		WalkScript m_script;
		Quadruped *m_quad;
		QuadAction m_action;
		QuadAction m_prevAction;
		bool m_running;

	private:
		void MoveBody(float deltaTime);
		void MoveLeg();
		void ReceiveNextAction();
		float FinishPreviousAction(float timeLeft);
		void ExecuteAction(float deltaTime);

	public:
		void Init(Quadruped *quadruped);
		void Update(float deltaTime);
		WalkScript& getWalkScript();
	};

	mth::float2 Section(quad::LegID legID);
}