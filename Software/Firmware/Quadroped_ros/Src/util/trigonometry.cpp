/*
 * trigonometry.c
 *
 *  Created on: 2022. máj. 14.
 *      Author: sanyi
 */
#include "util/trigonometry.h"

	/*
	* Calculat the interactions of a line and a circle
	* Input:	(float) rCircle:			The circles radius,
	*			(float) mLine:				The lines slope,
	*			(mth::float2) pointLine:	The point within the line
	* Output: (mth::float2x2):			Up to two crossing points
	*/
namespace mth
{
	mth::float2x2 CircleLineIntersection(float rCircle, mth::float2 normalVector, mth::float2 pointLine, float eps)
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
			//Do nothing
		}
		else if (fabsf(C * C - r * r * (A * A + B * B)) < eps)
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
	mth::float2x2 LineXYAxisIntersection(float mLine, mth::float2 pointLine)
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

	mth::float2 RelativePointOnACircle(mth::float2 pointOnCircle, float angle)
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
	mth::float2 SubtractPoints(mth::float2 point1, mth::float2 point2)
	{
		return mth::float2(point2.x-point1.x,point2.y-point1.y);
	}
}
