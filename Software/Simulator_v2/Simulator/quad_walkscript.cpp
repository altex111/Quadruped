#include "quad_walkscript.h"

namespace quad
{
#pragma region WalkScript

	void WalkScript::AddPathElementLegMovement(LegID  legid, mth::float2 pos)
	{
		QuadAction qa;
		qa.legID = legid;
		qa.goalPos = pos;
		m_script.push_back(qa);
	}
	void WalkScript::AddPathElementBodyMovement(mth::float2 pos, float turn)
	{
		QuadAction qa;
		qa.legID = -1;
		qa.goalPos = pos;
		qa.rot = turn;
		m_script.push_back(qa);
	}

	void WalkScript::AddLegTurnRightBalanced(mth::float2x2 rotmat)
	{
		AddPathElementLegMovement(LID_LB, rotmat * mth::float2(-m_legXPos, -(m_legZRetracted + m_legStretchHalf)));
		AddPathElementLegMovement(LID_LF, rotmat * mth::float2(-m_legXPos, m_legZRetracted + m_legStretchHalf));
		AddPathElementLegMovement(LID_RF, rotmat * mth::float2(m_legXPos, m_legZRetracted + m_legStretchHalf));
		AddPathElementLegMovement(LID_RB, rotmat * mth::float2(m_legXPos, -(m_legZRetracted + m_legStretchHalf)));
	}
	void WalkScript::AddLegTurnLeftBalanced(mth::float2x2 rotmat)
	{
		AddPathElementLegMovement(LID_RB, rotmat * mth::float2(m_legXPos, -(m_legZRetracted + m_legStretchHalf)));
		AddPathElementLegMovement(LID_RF, rotmat * mth::float2(m_legXPos, m_legZRetracted + m_legStretchHalf));
		AddPathElementLegMovement(LID_LF, rotmat * mth::float2(-m_legXPos, m_legZRetracted + m_legStretchHalf));
		AddPathElementLegMovement(LID_LB, rotmat * mth::float2(-m_legXPos, -(m_legZRetracted + m_legStretchHalf)));
	}
	void WalkScript::AddLegBodyElementsTurn(float angle)
	{
		float ratio = min(fabsf(angle) / m_maxTurnAtOnce, 1.0f);
		if (angle < 0.0f)
			ratio = -ratio;
		mth::float2x2 rotmat = mth::float2x2::Rotation(-ratio * m_maxTurnAtOnce);
		if (m_rightBalanced)
			AddLegTurnRightBalanced(rotmat);
		else
			AddLegTurnLeftBalanced(rotmat);
		m_rightBalanced = angle > 0.0f;
		AddPathElementBodyMovement(mth::float2(), m_maxTurnAtOnce * ratio);
	}

	void WalkScript::AddLegWalkStraightRightBalanced(float ratio)
	{
		AddPathElementLegMovement(LID_LB, mth::float2(-m_legXPos, -m_legZRetracted - (1.0f - ratio)*m_legStretchHalf));
		AddPathElementLegMovement(LID_LF, mth::float2(-m_legXPos, m_legZRetracted + 2.0f*m_legStretchHalf - (1.0f - ratio)*m_legStretchHalf));
		m_rightBalanced = false;
	}
	void WalkScript::AddLegWalkStraightLeftBalanced(float ratio)
	{
		AddPathElementLegMovement(LID_RB, mth::float2(m_legXPos, -m_legZRetracted - (1.0f - ratio)*m_legStretchHalf));
		AddPathElementLegMovement(LID_RF, mth::float2(m_legXPos, m_legZRetracted + 2.0f*m_legStretchHalf - (1.0f - ratio)*m_legStretchHalf));
		m_rightBalanced = true;
	}
	void WalkScript::AddLegBodyElementsWalkStraight(float distance)
	{
		float ratio = min(distance / m_legStretchHalf, 1.0f);
		if (m_rightBalanced)
			AddLegWalkStraightRightBalanced(ratio);
		else
			AddLegWalkStraightLeftBalanced(ratio);
		AddPathElementBodyMovement(mth::float2(0.0f, m_legStretchHalf*ratio), 0.0f);
	}

	void WalkScript::calculateOptimalsteps(float* legStretchHalf, float* turnAtOnce, mth::float2 relativePos, float relativeHeadding)
	{

	}
	/*
		Claculate maximum step length at the actual moveing direction
		Input: Normalized moveing direction vector
		Output: maximum step length based on the the leg moveing trajectory
	*/
	float WalkScript::calculateMaxLegStretch(mth::float2 motionDirection)
	{
		float legMaxStepHalfs[4];
		legMaxStepHalfs[quad::LID_RF] = findTrajectoryIntersection(motionDirection, mth::float2(1, 1));
		legMaxStepHalfs[quad::LID_LF] = findTrajectoryIntersection(motionDirection, mth::float2(-1, 1));
		legMaxStepHalfs[quad::LID_RB] = findTrajectoryIntersection(motionDirection, mth::float2(1, -1));
		legMaxStepHalfs[quad::LID_LB] = findTrajectoryIntersection(motionDirection, mth::float2(-1, -1));

		float minLegStepHalf = legMaxStepHalfs[quad::LID_RF];
		for (uint8_t i = 1; i < 4; i++)
		{
			if (legMaxStepHalfs[i] < minLegStepHalf)
				minLegStepHalf = legMaxStepHalfs[i];
		}
		return minLegStepHalf;
	}

	float WalkScript::findTrajectoryIntersection(mth::float2 motionDirection, mth::float2 section)
	{
		motionDirection.Normalize();
		mth::float2 sectionAbs = section; sectionAbs.Abs();
		section /= sectionAbs;
		mth::float2x2 section2x2;
		section2x2 = mth::float2x2::Scaling(section);
		mth::float2 centerPosXY = m_legCenterPos.getXY()*section;
		float rOuterCircle = (m_legRReachOffset + m_legRReachMax);
		float rInnerCircle = m_legRReachOffset;

		mth::float2x2 innerCircleIntersection = circleLineIntersection(rInnerCircle, 
			motionDirection.Slope(), centerPosXY);
		mth::float2x2 outerCircleIntersection = circleLineIntersection(rOuterCircle,
			motionDirection.Slope(), centerPosXY);
		mth::float2x2 xyAxisIntersection = lineXYAxisIntersection(motionDirection.Slope(), centerPosXY);
		innerCircleIntersection *= section2x2;
		outerCircleIntersection *= section2x2;
		xyAxisIntersection *= section2x2;

		std::list<mth::float2> foundPoints;

		if (innerCircleIntersection(0, 0) > 0 && innerCircleIntersection(0, 1) > 0)
			foundPoints.push_back((innerCircleIntersection*section2x2).Transposed() * mth::float2(1, 0));

		if (innerCircleIntersection(1, 0) > 0 && innerCircleIntersection(1, 1) > 0)
			foundPoints.push_back((innerCircleIntersection*section2x2).Transposed() * mth::float2(0, 1));

		if (outerCircleIntersection(0, 0) > 0 && outerCircleIntersection(0, 1) > 0)
			foundPoints.push_back((outerCircleIntersection*section2x2).Transposed() * mth::float2(1, 0));

		if (outerCircleIntersection(1, 0) > 0 && outerCircleIntersection(1, 1) > 0)
			foundPoints.push_back((outerCircleIntersection*section2x2).Transposed() * mth::float2(0, 1));

		if (xyAxisIntersection(0, 0) >= rInnerCircle && xyAxisIntersection(0, 0) <= rOuterCircle)
		{
			xyAxisIntersection(1, 0) = 0; xyAxisIntersection(1, 1) = 0;
			foundPoints.push_back((xyAxisIntersection * section2x2).Transposed() * mth::float2(1, 0));
		}

		if (xyAxisIntersection(1, 1) >= rInnerCircle && xyAxisIntersection(1, 1) <= rOuterCircle)
		{
			xyAxisIntersection(0, 0) = 0; xyAxisIntersection(0, 1) = 0;
			foundPoints.push_back((xyAxisIntersection * section2x2).Transposed() * mth::float2(0, 1));
		}

		uint8_t numberOfFoundPoints = foundPoints.size();
		for (uint8_t i = 0; i < numberOfFoundPoints; i++)
		{
			mth::float2 foundPoint = foundPoints.front();
			mth::float2 potentialStepTrajectory = twoPointsDistance(centerPosXY, foundPoint);
			if (motionDirection.isNear(potentialStepTrajectory.Normalized(), m_EPS))
			{
				return potentialStepTrajectory.Length();
			}
			foundPoints.pop_front();
		}
		return NULL;
	}
		
	/*
	Calculat the interactions of a line and a circle
	Input:	(float) rCircle:			The circles radius
			(float) mLine:				The lines slope
			(mth::float2) pointLine:	The point within the line
	Output: (mth::float2x2):			Up to two crossing points
	*/
	mth::float2x2 WalkScript::circleLineIntersection(float rCircle, float mLine, mth::float2 pointLine)
	{
		float r = rCircle;
		float A = -mLine;
		float B = 1;
		float C = mLine * pointLine.x - pointLine.y;
		float x0 = -A * C / (A * A + B * B);
		float y0 = -B * C / (A * A + B * B);
		mth::float2x2 foundPoinst;
		if (C * C > r * r * (A * A + B * B) + m_EPS)
		{
			//No points
			foundPoinst = NULL;
		}
		else if (abs(C * C - r * r * (A * A + B * B)) < m_EPS)
		{
			//One point found
			foundPoinst(0, 0) = x0;
			foundPoinst(0, 1) = y0;
		}
		else 
		{
			//Two points found
			float d = r * r - C * C / (A * A + B * B);
			float mult = sqrtf(d / (A * A + B * B));
			foundPoinst(0, 0) = x0 + B * mult; foundPoinst(0, 1) = y0 - A * mult;
			foundPoinst(1, 0) = x0 - B * mult; foundPoinst(1, 1) = y0 + A * mult;
		}


		return foundPoinst;
	}
	/*
		Calculat a lines points thats on the x and y axis
		Input:	(float) mLine:				The lines slope
				(mth::float2) pointLine:	The point within the line
		Output: (mth::float2x2)	| x1 y1 | (x,y) coordinats of the line at y = 0
								| x2 y2 | (x,y) coordinats of the line at x = 0
	*/					
	mth::float2x2 WalkScript::lineXYAxisIntersection(float mLine, mth::float2 pointLine)
	{
		float m = mLine;
		float x0 = pointLine.x;
		float y0 = pointLine.y;
		mth::float2x2 foundPoint;
		//y == 0
		//x = 1/m * (y - y0) + x0
		//x == 0
		//y = m(x - x0) + y0
		//|x1 = 1/m * (y - y0) + x0	,		y1 = 0		 |
		//|			x2 = 0			, y2 = m(x - x0) + y0|
		foundPoint(0, 0) = -y0 / m + x0;	foundPoint(1, 0) = 0;
		foundPoint(1, 0) = 0;				foundPoint(1, 1) = -m * x0 + y0;

		return foundPoint;
	}

	mth::float2 WalkScript::twoPointsDistance(mth::float2 point1, mth::float2 point2)
	{
		return mth::float2(point2.x-point1.x,point2.y-point1.y);
	}

	WalkScript::WalkScript() :
		m_maxTurnAtOnce(mth::pi * 0.25f),
		m_bellyy(0.3f),
		m_legLift(0.2f),
		m_legXPos(0.9f),
		m_legZRetracted(0.4f),
		m_legStretchHalf(0.5f),
		m_legXBasePos(0.52462f),
		m_legZBasePos(0.74962f),
		m_legCenterPos(1.4f, mth::pi * 0.25f),
		m_legRReachOffset(0.9f),
		m_legRReachMax(1.0f),
		m_EPS(0.001),
		m_rightBalanced(true) {}

	void WalkScript::AddPathElementTurn(float angle)
	{
		float angleabs = fabsf(angle);
		while (angleabs > 0.0f)
		{
			AddLegBodyElementsTurn(angle);
			angleabs -= m_maxTurnAtOnce;
			if (angle > 0.0f)
				angle -= m_maxTurnAtOnce;
			else
				angle += m_maxTurnAtOnce;
		}
	}

	void WalkScript::AddPathElementWalkStraight(float distance)
	{
		while (distance > 0.0f)
		{
			AddLegBodyElementsWalkStraight(distance);
			distance -= m_legStretchHalf;
		}
	}

	void WalkScript::AddPathElementMove(mth::float2 relativePos, float relativeHeadding)
	{
		float legStretchHalf = 0;
		float turnAtOnce = 0;
		calculateMaxLegStretch(relativePos.Normalized());
		//calculateOptimalsteps(&legStretchHalf, &turnAtOnce, relativePos, relativeHeadding);
	}

	void WalkScript::Clear()
	{
		m_script.clear();
	}

	void WalkScript::AddPathElementCircle(float circleR, float rotation)
	{
		float rotabs = fabsf(rotation);
		float rotpermove = m_legStretchHalf / circleR;
		mth::float2x2 rotmat = mth::float2x2::Rotation(rotpermove);
		QuadAction qa;
		while (rotabs > 0.0f)
		{
			if (m_rightBalanced)
			{				
				m_rightBalanced = false;
			}
			else
			{
				m_rightBalanced = true;
			}
			qa.goalPos = rotmat * mth::float2(0.0f, m_legStretchHalf);

			rotabs -= rotpermove;
			if (rotation > 0.0f)
				rotation -= rotpermove;
			else
				rotation += rotpermove;
		}
	}
	bool WalkScript::NextAction(QuadAction& action)
	{
		if (m_script.empty())
			return false;
		action = *m_script.begin();
		m_script.pop_front();
		return true;
	}
	mth::float3 WalkScript::getLegRFStartPos()
	{
		//return { m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf };
		return { m_legXBasePos + cos(mth::pi * 0.25f) * 1.4f, -m_bellyy, m_legZBasePos + sin(mth::pi * 0.25f) * 1.4f };
	}
	mth::float3 WalkScript::getLegLFStartPos()
	{
		//return { -m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf };
		return { -(m_legXBasePos + 0.8f), -m_bellyy, m_legZBasePos + 0.8f };
	}
	mth::float3 WalkScript::getLegRBStartPos()
	{
		//return { m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) };
		return { m_legXBasePos + 0.8f, -m_bellyy, -(m_legZBasePos + 0.8f) };
	}
	mth::float3 WalkScript::getLegLBStartPos()
	{
		//return mth::float3({ -m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) });
		return { -(m_legXBasePos + 0.8f), -m_bellyy, -(m_legZBasePos + 0.8f) };
	}
	float WalkScript::getBellyY()
	{
		return m_bellyy;
	}
	float WalkScript::LegY(float t)
	{
		t = 2.0f*t - 1.0f;
		return -m_bellyy + (1.0f - t * t)*m_legLift;
	}

#pragma endregion

#pragma region WalkManager

	void WalkManager::Init(Quadruped *quadruped)
	{
		m_script.Clear();
		m_quad = quadruped;
		m_time = 0.0f;
		m_speed = 1.0f;
		m_running = false;
		m_quad->getEntity().position = { 0.0f, m_script.getBellyY(), 0.0f };
		m_quad->getEntity().rotation = { 0.0f, 0.0f, 0.0f };
		m_quad->getLegRF().setPosition(m_script.getLegRFStartPos());
		m_quad->getLegRB().setPosition(m_script.getLegRBStartPos());
		m_quad->getLegLF().setPosition(m_script.getLegLFStartPos());
		m_quad->getLegLB().setPosition(m_script.getLegLBStartPos());
	}

	void WalkManager::MoveBody(float deltaTime)
	{
		mth::float3 delta = { -deltaTime * m_action.goalPos.x, 0.0f, deltaTime * m_action.goalPos.y };
		m_quad->getEntity().MoveInLookDirection(delta);
		m_quad->getEntity().rotation.y += deltaTime * m_action.rot;
		delta = mth::float3x3::RotationY(m_action.rot)*delta;
		for (Leg& l : m_quad->getLegs())
			l.setPosition(mth::float3x3::RotationY(-deltaTime * m_action.rot)*(-delta + l.getPosition()));
	}
	void WalkManager::MoveLeg()
	{
		mth::float3 pos = { m_prevAction.goalPos.x + (m_action.goalPos.x - m_prevAction.goalPos.x)*m_time,
			0.0f, m_prevAction.goalPos.y + (m_action.goalPos.y - m_prevAction.goalPos.y)*m_time };
		pos.Normalize();
		pos *= (1.0f - m_time)*m_prevAction.goalPos.Length() + m_time * m_action.goalPos.Length();
		pos.y = m_script.LegY(m_time);
		m_quad->getLeg(m_action.legID).setPosition(pos);
	}
	void WalkManager::ReceiveNextAction()
	{
		if (m_running = m_script.NextAction(m_action))
		{
			if (m_action.legID >= 0)
			{
				mth::float3 prevPos = m_quad->getLeg(m_action.legID).getPosition();
				m_prevAction.goalPos.x = prevPos.x;
				m_prevAction.goalPos.y = prevPos.z;
			}
		}
	}
	float WalkManager::FinishPreviousAction(float timeLeft)
	{
		if (m_action.legID < 0)
			MoveBody(timeLeft);
		else
			m_quad->getLeg(m_action.legID).setPosition({ m_action.goalPos.x, -m_script.getBellyY(), m_action.goalPos.y });
		ReceiveNextAction();
		m_time = fmodf(m_time, 1.0f);
		return m_time;
	}
	void WalkManager::ExecuteAction(float deltaTime)
	{
		m_time += deltaTime;
		if (m_time >= 1.0f)
			deltaTime = FinishPreviousAction(1.0f - (m_time - deltaTime));
		if (m_action.legID < 0)
			MoveBody(deltaTime);
		else
			MoveLeg();
	}
	void WalkManager::Update(float deltaTime)
	{
		if (!m_running)
		{
			//Leg reachble area test
			// test needs: Set public the function, and the base positions
			//m_script.AddPathElementLegMovement(LID_RF, mth::float2(m_script.m_legXBasePos + cos(mth::pi * 0.25f) * 1.9f, m_script.m_legZBasePos + sin(mth::pi * 0.25f) * 1.9f));
			//m_script.AddPathElementLegMovement(LID_RF, mth::float2(m_script.m_legXBasePos + cos(mth::pi * 0.0f) * 1.9f, m_script.m_legZBasePos + sin(mth::pi * 0.0f) * 1.9f));
			//m_script.AddPathElementLegMovement(LID_RF, mth::float2(m_script.m_legXBasePos + cos(mth::pi * 0.0f) * 0.9f, m_script.m_legZBasePos + sin(mth::pi * 0.0f) * 0.9f));
			//m_script.AddPathElementLegMovement(LID_RF, mth::float2(m_script.m_legXBasePos + cos(mth::pi * 0.25f) * 0.9f, m_script.m_legZBasePos + sin(mth::pi * 0.25f) * 0.9f));
			//m_script.AddPathElementLegMovement(LID_RF, mth::float2(m_script.m_legXBasePos + cos(mth::pi * 0.5f) * 0.9f, m_script.m_legZBasePos + sin(mth::pi * 0.5f) * 0.9f));
			//m_script.AddPathElementLegMovement(LID_RF, mth::float2(m_script.m_legXBasePos + cos(mth::pi * 0.5f) * 1.9f, m_script.m_legZBasePos + sin(mth::pi * 0.5f) * 1.9f));
			//End test
			
			//m_script.AddPathElementWalkStraight(2.0f);
			//m_script.AddPathElementTurn(-mth::pi*0.5f);
			m_script.AddPathElementMove(mth::float2(10.0f, 4.0f), mth::pi * 0.25);
			

			ReceiveNextAction();
			m_time = 0.0f;
		}
		if (m_running)
			ExecuteAction(deltaTime * m_speed);
	}
	WalkScript& WalkManager::getWalkScript()
	{
		return m_script;
	}

#pragma endregion
}

