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

	void WalkScript::AddLegBodyElementsMove(float *distance, uint8_t *legCount,
		float legStretchHalf, float turnAtOnce, mth::float2 motionDirection, quad::LegID *stepOrder)
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
		point = RelativePointOnACircle(point, turnAtOnce);
		AddPathElementLegMovement(stepOrder[*legCount], mth::float2(point.y,point.x));

		(* legCount)++;

		point = Section(stepOrder[*legCount]) * (m_legBasePos + m_legCenterPos.getXY()) + motionDirection * legStretchHalf;
		point = RelativePointOnACircle(point, turnAtOnce);
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
		legMaxStepHalfs[quad::LID_RF] = legStretchHalf(motionDirection, quad::LID_RF);
		legMaxStepHalfs[quad::LID_LF] = legStretchHalf(motionDirection, quad::LID_LF);
		legMaxStepHalfs[quad::LID_RB] = legStretchHalf(motionDirection, quad::LID_RB);
		legMaxStepHalfs[quad::LID_LB] = legStretchHalf(motionDirection, quad::LID_LB);

		float minLegStepHalf = legMaxStepHalfs[quad::LID_RF];
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
	*			(quad::LegID) legId:			A specific legs ID
	* Output:	(float): The length of a specific legs strechHalf
	*/
	float WalkScript::legStretchHalf(mth::float2 motionDirection, quad::LegID legId)
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
		
	/*
	* Calculat the interactions of a line and a circle
	* Input:	(float) rCircle:			The circles radius,
	*			(float) mLine:				The lines slope,
	*			(mth::float2) pointLine:	The point within the line
	* Output: (mth::float2x2):			Up to two crossing points
	*/
	mth::float2x2 WalkScript::CircleLineIntersection(float rCircle, mth::float2 normalVector, mth::float2 pointLine, float eps)
	{
		float r = rCircle;
		float A = -normalVector.y;
		float B = normalVector.x;
		float C = normalVector.y * pointLine.x - normalVector.x * pointLine.y;
		float x0 = -A * C / (A * A + B * B);
		float y0 = -B * C / (A * A + B * B);
		mth::float2x2 foundPoinst;
		if (C * C > r * r * (A * A + B * B) + eps)
		{
			//No points
			foundPoinst = NULL;
		}
		else if (abs(C * C - r * r * (A * A + B * B)) < eps)
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
		Input:	(float) mLine:				The lines slope,
				(mth::float2) pointLine:	The point within the line
		Output: (mth::float2x2)	| x1 y1 | (x,y) coordinats of the line at y = 0
								| x2 y2 | (x,y) coordinats of the line at x = 0
	*/					
	mth::float2x2 WalkScript::LineXYAxisIntersection(float mLine, mth::float2 pointLine)
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

	mth::float2 WalkScript::RelativePointOnACircle(mth::float2 pointOnCircle, float angle)
	{
		mth::float2circle posRA = pointOnCircle.getRA();
		posRA.a += angle;
		return mth::float2(posRA.getY(), posRA.getX());
	}

	/*
		Calculates the subtraction of two points
		Input:	(mth::float2) point1:	The subtracted vector starting point,
				(mth::float2) point2:	The subtracted vector ending point
		Output	(mth::float2):			The subtracted vector
	*/
	mth::float2 WalkScript::SubtractPoints(mth::float2 point1, mth::float2 point2)
	{
		return mth::float2(point2.x-point1.x,point2.y-point1.y);
	}

	WalkScript::WalkScript() :
		m_maxTurnAtOnce(mth::pi * 0.25f*0.5f),

		m_bellyy(0.6f),
		m_legLift(0.2f),
		m_legXPos(0.9f),
		m_legZRetracted(0.4f),
		m_legStretchHalf(0.5f),

		m_legXBasePos(0.52462f),
		m_legZBasePos(0.74962f),
		m_legMaxStretchHalf(0.5),//This is going to be overwriten
		m_legBasePos(0.74962f, 0.52462f),
		m_legCenterPos(1.1f, mth::pi * 0.25f),
		m_legRReachOffset(0.5f),
		m_legRReachMax(1.0f),
		m_EPS(0.001),

		m_rightBalanced(true) 
	{
		m_criticalAngle = atan2f(m_legBasePos.y + m_legCenterPos.getY(),
								 m_legBasePos.x + m_legCenterPos.getX());
		m_legCenterPos = mth::float2circle((2 * m_legRReachOffset + m_legRReachMax) / 2, mth::pi * 0.25);

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
		uint8_t legCount = 0;
		quad::LegID stepOrder[4] = { LID_RF,LID_LF,LID_RB,LID_LB };
		float legStretchHalf = 0;
		float turnAtOnce = 0;
		float distance = relativePos.Length();
		float relativeHeaddingAbs = fabsf(relativeHeadding);
		m_legMaxStretchHalf = calculateMaxLegStretchHalf(relativePos.Normalized());
		//TODO: Uncomment relativeHeadding if turn and move together
		calculateOptimalsteps(&legStretchHalf, &turnAtOnce, distance, relativeHeadding);
		

		//Works on that the robot can move and turn together
		//	O-TODO: Redo all turning commands in move section
		//	O-TODO: If distance 0 but has relative headding the robot dosent do anything.
		//	O-TODO: Better leg order for turning as well
		//	O-TODO: Comment and uncomment all things for the other movement method

		//Works on that the robot can turn after its move
		//	x-TODO: Comment all turning command in move section
		//	O-TODO: Redesign turning command
		//	x-TODO: Implement turning after move
		//	x-TODO: Comment and uncomment all things for the other movement method
		while (distance > m_EPS)
		{
			AddLegBodyElementsMove(&distance,&legCount, legStretchHalf, turnAtOnce, relativePos, &stepOrder[0]);
			mth::float2circle posRA = relativePos.getRA();
			posRA.a -= turnAtOnce;
			relativePos = mth::float2(posRA.getY(), posRA.getX());
		}
		//TODO: Comment this out if tudn and move is together
		/*while (relativeHeaddingAbs > 0.0f)
		{
			AddLegBodyElementsTurn(relativeHeadding);
			relativeHeaddingAbs -= turnAtOnce;
			if (relativeHeadding > 0.0f)
				relativeHeadding -= turnAtOnce;
			else
				relativeHeadding += turnAtOnce;
		}*/
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
		return { m_legXBasePos + m_legCenterPos.getY(), -m_bellyy, m_legZBasePos + m_legCenterPos.getX()};
	}
	mth::float3 WalkScript::getLegLFStartPos()
	{
		//return { -m_legXPos, -m_bellyy, m_legZRetracted + m_legStretchHalf };
		return { -(m_legXBasePos + m_legCenterPos.getY()), -m_bellyy, m_legZBasePos + m_legCenterPos.getX()};
	}
	mth::float3 WalkScript::getLegRBStartPos()
	{
		//return { m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) };
		return { m_legXBasePos + m_legCenterPos.getY(), -m_bellyy, -(m_legZBasePos + m_legCenterPos.getX()) };
	}
	mth::float3 WalkScript::getLegLBStartPos()
	{
		//return mth::float3({ -m_legXPos, -m_bellyy, -(m_legZRetracted + m_legStretchHalf) });
		return { -(m_legXBasePos + m_legCenterPos.getY()), -m_bellyy, -(m_legZBasePos + m_legCenterPos.getX()) };
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
		m_speed = 2.0f;
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
		//Removed - befor deltaTime
		mth::float3 delta = { /*-*/deltaTime * m_action.goalPos.x, 0.0f, deltaTime * m_action.goalPos.y};
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
			//m_script.AddPathElementMove(mth::float2(1.0f, 0.5f), mth::pi * -0.3f);
			//m_script.AddPathElementMove(mth::float2(0.0f, 0.0f), mth::pi * 2.0f);
			//m_script.AddPathElementMove(mth::float2(-2.0f, 1.0f), mth::pi * -0.5f);
			

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
	mth::float2 Section(quad::LegID legID)
	{
		if (legID == quad::LID_RF)
			return mth::float2(1, 1);
		else if (legID == quad::LID_LF)
			return mth::float2(1, -1);
		else if (legID == quad::LID_RB)
			return mth::float2(-1, 1);
		else //(legID == quad::LID_LB)
			return mth::float2(-1, -1);
	}
}

