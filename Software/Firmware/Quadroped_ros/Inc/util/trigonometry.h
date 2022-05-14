/*
 * trigonometry.h
 *
 *  Created on: 2022. máj. 14.
 *      Author: sanyi
 */

#ifndef UTIL_TRIGONOMETRY_H_
#define UTIL_TRIGONOMETRY_H_
#include "util/linalg.h"
#include <cmath>

namespace mth
{
	mth::float2x2 CircleLineIntersection(float rCircle, mth::float2 normalVector, mth::float2 pointLine, float eps);
	mth::float2x2 LineXYAxisIntersection(float mLine, mth::float2 pointLine);
	mth::float2 RelativePointOnACircle(mth::float2 pointOnCircle, float angle);
	mth::float2 SubtractPoints(mth::float2 pont1, mth::float2 point2);
}

#endif /* UTIL_TRIGONOMETRY_H_ */
