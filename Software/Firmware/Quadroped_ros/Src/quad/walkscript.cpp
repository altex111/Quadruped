#include "quad/walkscript.h"
#include <cmath>

#define MIN(a, b) a<b?a:b
#define MAX(a, b) a>b?a:b

namespace quadroped
{
	void WalkScript::AddPathElementLegMovement(LegID legid, mth::float2 pos)
	{
		QuadAction qa;
		qa.legID = legid;
		qa.goalPos = pos;
		m_script.push(qa);
	}
	void WalkScript::AddPathElementBodyMovement(mth::float2 pos, float turn)
	{
		QuadAction qa;
		qa.legID = -1;
		qa.goalPos = pos;
		qa.rot = turn;
		m_script.push(qa);
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
		float ratio = MIN(fabsf(angle) / m_maxTurnAtOnce, 1.0f);
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
		float ratio = MIN(distance / m_legStretchHalf, 1.0f);
		if (m_rightBalanced)
			AddLegWalkStraightRightBalanced(ratio);
		else
			AddLegWalkStraightLeftBalanced(ratio);
		AddPathElementBodyMovement(mth::float2(0.0f, m_legStretchHalf*ratio), 0.0f);
	}
	void WalkScript::AddLegBodyElementsMove(float *distance,uint8_t *legCount, float legStretchHalf, float turnAtOnce, mth::float2 motionDirection, quadroped::LegID* stepOrder)
	{
		float motionAngle = motionDirection.getRA().a;
		motionDirection.Normalize();

		if ((* legCount) == 0)
		{
			if (motionDirection.x > 0 && motionDirection.y >= 0)
			{
				stepOrder[0] = LID_RF;
				stepOrder[1] = LID_LB;
				if (motionAngle < m_criticalAngle)
				{
					stepOrder[2] = LID_LF;
					stepOrder[3] = LID_RB;
				}
				else
				{
					stepOrder[2] = LID_RB;
					stepOrder[3] = LID_LF;
				}
			}
			else if (motionDirection.x >= 0 && motionDirection.y < 0)
			{
				stepOrder[0] = LID_LF;
				stepOrder[1] = LID_RB;
				if (fabsf(motionAngle) < m_criticalAngle)
				{
					stepOrder[2] = LID_RF;
					stepOrder[3] = LID_LB;
				}
				else
				{
					stepOrder[2] = LID_LB;
					stepOrder[3] = LID_RF;
				}
			}
			else if (motionDirection.x <= 0 && motionDirection.y > 0)
			{
				stepOrder[0] = LID_RB;
				stepOrder[1] = LID_LF;
				if (motionAngle > mth::pi - m_criticalAngle)
				{
					stepOrder[2] = LID_LB;
					stepOrder[3] = LID_RF;
				}
				else
				{
					stepOrder[2] = LID_RF;
					stepOrder[3] = LID_LB;
				}
			}
			else
			{
				stepOrder[0] = LID_LB;
				stepOrder[1] = LID_RF;
				if (fabsf(motionAngle) > mth::pi - m_criticalAngle)
				{
					stepOrder[2] = LID_RB;
					stepOrder[3] = LID_LF;
				}
				else
				{
					stepOrder[2] = LID_LF;
					stepOrder[3] = LID_RB;
				}
			}
		}
		*distance -= legStretchHalf;
		mth::float2 point = Section(stepOrder[*legCount]) * (m_legBasePos + m_legCenterPos.getXY()) + motionDirection * legStretchHalf;
		//point = RelativePointOnACircle(point, turnAtOnce);
		AddPathElementLegMovement(stepOrder[*legCount], mth::float2(point.y,point.x));

		(* legCount)++;

		point = Section(stepOrder[*legCount]) * (m_legBasePos + m_legCenterPos.getXY()) + motionDirection * legStretchHalf;
		//point = RelativePointOnACircle(point, turnAtOnce);
		AddPathElementLegMovement(stepOrder[*legCount], mth::float2(point.y, point.x));

		(* legCount)++;
		if (*legCount >= 4)
			*legCount = 0;

		point = motionDirection * legStretchHalf;
		AddPathElementBodyMovement(mth::float2(point.y, point.x), turnAtOnce);
	}

	/*
	* Calculates the legStretchHalf and the turnAtOnce values of the robot motion
	* Input:	(float*) legStretchHalf:	The leg stretch half calculated based on the (distance/m_legMaxStretch) not always a whole number
	*			(flaot*) turnAtOnce:		Turn at once angle based on the relatve headding and the steps the robot has to make
	*			(float) distance:			The distance the robot has to travel
	*			(float)	relativeHeadding:	The headding the robot have to be after the motion
	* Output:	(float*) legStretchHalf:	(Pointer output)
	*			(float*) turnAtOnce:		(Pointer output)
	*/
	void WalkScript::calculateOptimalsteps(float *legStretchHalf, float *turnAtOnce, float distance, float relativeHeadding)
	{
		float relativeHeaddingAbs = fabsf(relativeHeadding); // The absolute value of the relativeHeadding

		float preferedSteps = ceilf(distance / m_legMaxStretchHalf); // Calculate minimum steps
		float preferedTurnAtOnce = relativeHeaddingAbs / preferedSteps; // Calculate prefered one turn angle

		if (preferedTurnAtOnce > m_maxTurnAtOnce || m_legMaxStretchHalf == 0)
		{
			//calculate parameters when preferdTurnAtOnce biger then the robot can handle
			float steps = ceilf(relativeHeaddingAbs / m_maxTurnAtOnce); // calculate new step number
			*legStretchHalf = distance / steps;	// calculate new step distance
			*turnAtOnce = (relativeHeadding / relativeHeaddingAbs) * (relativeHeaddingAbs / steps);// calculate new turn angle per step
		}
		else
		{
			*legStretchHalf = distance / preferedSteps;
			if (relativeHeaddingAbs > 0)
				*turnAtOnce = (relativeHeadding / relativeHeaddingAbs) * preferedTurnAtOnce;
			else
				*turnAtOnce = 0;
		}
	}

	/*
	* Claculate maximum step length at the actual moveing direction
	* Input:	(mth::float2) motionDirection:	Normalized moveing direction vector
	* Output:	(float):						Maximum step length based on the the leg moveing trajectory
	*/
	float WalkScript::calculateMaxLegStretchHalf(mth::float2 motionDirection)
	{
		float legMaxStepHalfs[4] = {0,0,0,0};
		legMaxStepHalfs[quadroped::LID_RF] = legStretchHalf(motionDirection, quadroped::LID_RF);
		legMaxStepHalfs[quadroped::LID_LF] = legStretchHalf(motionDirection, quadroped::LID_LF);
		legMaxStepHalfs[quadroped::LID_RB] = legStretchHalf(motionDirection, quadroped::LID_RB);
		legMaxStepHalfs[quadroped::LID_LB] = legStretchHalf(motionDirection, quadroped::LID_LB);

		float minLegStepHalf = legMaxStepHalfs[quadroped::LID_RF];
		for (uint8_t i = 1; i < 4; i++)
		{
			if (legMaxStepHalfs[i] < minLegStepHalf)
				minLegStepHalf = legMaxStepHalfs[i];
		}
		return minLegStepHalf;
	}

	/*
	* Calculates the legStrechHalf for a specific leg
	* Input:	(mth::float2) motionDirection:	The direction of the robots motion (Normal vector)
	*			(quadroped::LegID) legId:			A specific legs ID
	* Output:	(float): The length of a specific legs strechHalf
	*/
	float WalkScript::legStretchHalf(mth::float2 motionDirection, quadroped::LegID legId)
	{
		std::list<mth::float2> foundPoints = findTrajectoryIntersections(motionDirection, Section(legId));

		uint8_t numberOfFoundPoints = foundPoints.size();
		if (numberOfFoundPoints > 2)
			return NULL;
		for (uint8_t i = 0; i < numberOfFoundPoints; i++)
		{
			mth::float2 foundPoint = foundPoints.front();
			mth::float2 potentialStepTrajectory = SubtractPoints(m_legCenterPos.getXY() * Section(legId), foundPoint);
			if (motionDirection.isNear(potentialStepTrajectory.Normalized(), m_EPS))
			{
				return potentialStepTrajectory.Length();
			}
			foundPoints.pop_front();
		}

		return NULL;
	}

	/*
	* Finds the two points where the motion trajectory cross the reachble area for one leg
	* Input:	(mth::float2) motionDirection:	The direction of the robots motion (Normal vector)
	*			(mth::float2) section:			A vector that points where the leg is.
	*											The vectors prefix important only
	* Output:	(std::list<mth::float2>):		A lost of the crossing points
	*/
	std::list<mth::float2> WalkScript::findTrajectoryIntersections(mth::float2 motionDirection, mth::float2 section)
	{
		motionDirection.Normalize();
		mth::float2 sectionAbs = section; sectionAbs.Abs();
		section /= sectionAbs;
		mth::float2x2 section2x2;
		section2x2 = mth::float2x2::Scaling(section);
		mth::float2 centerPosXY = m_legCenterPos.getXY()*section;
		float rOuterCircle = (m_legRReachOffset + m_legRReachMax);
		float rInnerCircle = m_legRReachOffset;

		mth::float2x2 innerCircleIntersection = CircleLineIntersection(rInnerCircle,
			motionDirection, centerPosXY, m_EPS);
		mth::float2x2 outerCircleIntersection = CircleLineIntersection(rOuterCircle,
			motionDirection, centerPosXY, m_EPS);
		mth::float2x2 xyAxisIntersection = LineXYAxisIntersection(motionDirection.Slope(), centerPosXY);
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

		return foundPoints;
	}
	WalkScript::WalkScript(Quadruped& q)
	:	m_script(),
		m_action(),
		m_prevAction(),
		m_quad(q),
		m_time(0.0f),
		m_speed(0.5f),
		m_maxTurnAtOnce(mth::pi*0.25f),
		m_bellyy(0.3f),
		m_legLift(0.2f),
		m_legXPos(0.8f),
		m_legZRetracted(0.3f),
		m_legStretchHalf(0.5f),
		m_rightBalanced(true),
		m_running(false),

		m_legMaxStretchHalf(0.5),//This is going to be overwriten
		m_legBasePos(0.74962f, 0.52462f),
		m_legCenterPos(1.3f, mth::pi * 0.25f),
		m_legRReachOffset(0.7f),
		m_legRReachMax(1.2f),
		m_EPS(0.01)
	{
		m_criticalAngle = atan2f(m_legBasePos.y + m_legCenterPos.getY(),
								 m_legBasePos.x + m_legCenterPos.getX());

		m_quad.EnableLegs();
		m_quad.getLegRF().setPosition(getLegRFStartPos());
		m_quad.getLegLF().setPosition(getLegLFStartPos());
		m_quad.getLegRB().setPosition(getLegRBStartPos());
		m_quad.getLegLB().setPosition(getLegLBStartPos());

	//	m_quad.getLegLB().setJointStates(mth::float3(0.0f,mth::pi * -1.0f,2.5f));

	//	m_quad.getLegLB().getBaseServo().setState(mth::pi * 0.0f);
	//	m_quad.getLegLB().getShoulderServo().setState(mth::pi * 0.34f);
	//	m_quad.getLegLB().getKneeServo().setState(mth::pi * 0.34f);
	//	m_quad.getLegRB().getBaseServo().setState(mth::pi * 0.0f);
	//	m_quad.getLegRB().getShoulderServo().setState(mth::pi * -0.34f);
	//	m_quad.getLegRB().getKneeServo().setState(mth::pi * -0.24f); //elszartam a szervot :(
	//	m_quad.getLegLF().getBaseServo().setState(mth::pi * 0.0f);
	//	m_quad.getLegLF().getShoulderServo().setState(mth::pi * -0.25f);
	//	m_quad.getLegLF().getKneeServo().setState(mth::pi * -0.34f);
	//	m_quad.getLegRF().getBaseServo().setState(mth::pi * 0.0f);
	//	m_quad.getLegRF().getShoulderServo().setState(mth::pi * 0.34f);
	//	m_quad.getLegRF().getKneeServo().setState(mth::pi * 0.34f);
	}

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

	/*
	*
	* Input:	(mth::float2) relativePos:		The new point based on robot's coordinat, it is going to move
	*			(float)	relativeHeadding = 0:	The new headding based on robot's coordinat, it is going to have
	* Output:	None
	*/
	void WalkScript::AddPathElementMove(mth::float2 relativePos, float relativeHeadding)
	{
		//Comment out, when turning is working
		relativeHeadding = 0;
		uint8_t legCount = 0;
		quadroped::LegID stepOrder[4] = { LID_RF,LID_LF,LID_RB,LID_LB };
		float legStretchHalf = 0;
		float turnAtOnce = 0;
		float distance = relativePos.Length();
		float relativeHeaddingAbs = fabsf(relativeHeadding);
		m_legMaxStretchHalf = calculateMaxLegStretchHalf(relativePos.Normalized());
		calculateOptimalsteps(&legStretchHalf, &turnAtOnce, distance, relativeHeadding);
		while (distance > m_EPS)
		{
			AddLegBodyElementsMove(&distance,&legCount, legStretchHalf, turnAtOnce, relativePos, &stepOrder[0]);
//			mth::float2circle posRA = relativePos.getRA();
//			posRA.a -= turnAtOnce;
//			relativePos = mth::float2(posRA.getY(), posRA.getX());
		}
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


	void WalkScript::Clear()
	{
		m_script.clear();
	}
	bool WalkScript::NextAction(QuadAction& action)
	{
		return m_script.pop(action);
	}
	mth::float3 WalkScript::getLegRFStartPos()
	{
		return { m_legBasePos.y + m_legCenterPos.getY(), -m_bellyy, m_legBasePos.x + m_legCenterPos.getX() };
	}
	mth::float3 WalkScript::getLegLFStartPos()
	{
		return { -(m_legBasePos.y + m_legCenterPos.getY()), -m_bellyy, m_legBasePos.x + m_legCenterPos.getX() };
	}
	mth::float3 WalkScript::getLegRBStartPos()
	{
		return { m_legBasePos.y + m_legCenterPos.getY(), -m_bellyy, -(m_legBasePos.x + m_legCenterPos.getX()) };
	}
	mth::float3 WalkScript::getLegLBStartPos()
	{
		return mth::float3({ -(m_legBasePos.y + m_legCenterPos.getY()), -m_bellyy, -(m_legBasePos.x + m_legCenterPos.getX()) });
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

	void WalkScript::MoveBody(float deltaTime)
	{
		mth::float3 delta = { deltaTime * m_action.goalPos.x, 0.0f, deltaTime * m_action.goalPos.y };
		delta = mth::float3x3::RotationY(m_action.rot)*delta;
		for (Leg& l : m_quad.getLegs())
			l.setPosition(mth::float3x3::RotationY(-deltaTime * m_action.rot)*(-delta + l.getPosition()));
	}
	void WalkScript::MoveLeg()
	{
		mth::float3 pos = { m_prevAction.goalPos.x + (m_action.goalPos.x - m_prevAction.goalPos.x)*m_time,
			0.0f, m_prevAction.goalPos.y + (m_action.goalPos.y - m_prevAction.goalPos.y)*m_time };
		pos.Normalize();
		pos *= (1.0f - m_time)*m_prevAction.goalPos.Length() + m_time * m_action.goalPos.Length();
		pos.y = LegY(m_time);
		m_quad.getLeg(m_action.legID).setPosition(pos);
	}
	void WalkScript::ReceiveNextAction()
	{
		m_running = m_script.pop(m_action);
		if (m_running)
		{
			if (m_action.legID >= 0)
			{
				mth::float3 prevPos = m_quad.getLeg(m_action.legID).getPosition();
				m_prevAction.goalPos.x = prevPos.x;
				m_prevAction.goalPos.y = prevPos.z;
			}
		}
		else
		{
			return;
		}
	}
	float WalkScript::FinishPreviousAction(float timeLeft)
	{
		if (m_action.legID < 0)
			MoveBody(timeLeft);
		else
			m_quad.getLeg(m_action.legID).setPosition({ m_action.goalPos.x, -getBellyY(), m_action.goalPos.y });
		ReceiveNextAction();
		m_time = fmodf(m_time, 1.0f);
		return m_time;
	}
	void WalkScript::ExecuteAction(float deltaTime)
	{
		m_time += deltaTime;
		if (m_time >= 1.0f)
			deltaTime = FinishPreviousAction(1.0f - (m_time - deltaTime));
		if (m_action.legID < 0)
			MoveBody(deltaTime);
		else
			MoveLeg();
	}
	void WalkScript::Update(float deltaTime)
	{
		if (!m_running)
		{
			AddPathElementMove(mth::float2(2.0f,1.0f), 0.0f);
			AddPathElementMove(mth::float2(0.0f,-2.0f), 0.0f);
			AddPathElementMove(mth::float2(-2.0f,1.0f), 0.0f);
			//todo there are previus solutions AddPathElementWalkStraight(2.0f);
			//todo there are previus solutions AddPathElementTurn(-mth::pi*0.5f);

			ReceiveNextAction();
			m_time = 0.0f;
		}
		if (m_running)
			ExecuteAction(deltaTime * m_speed);
	}
}
