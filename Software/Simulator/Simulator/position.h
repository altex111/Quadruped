#pragma once

#include "linalg.h"

namespace mth
{
	class Position
	{
	private:
		float3 position;
		float3 rotation;
		float3 scale;

	protected:
		float3 posOffset;
		float3 rotOffset;


	public:
		Position();
		void setPosition(float3 pos);
		void setRotation(float3 rot);
		void setScale(float3 scale);
		void setPositionX(float posX);
		void setPositionY(float posY);
		void setPositionZ(float posZ);
		void setRotationX(float rotX);
		void setRotationY(float rotY);
		void setRotationZ(float rotZ);
		float3 getPosition();
		float3 getRotation();
		float3 getscale();

		void MoveForward(float d);
		void MoveBackward(float d);
		void MoveRight(float d);
		void MoveLeft(float d);
		void MoveUp(float d);
		void MoveDown(float d);
		void Move(float3 delta);
		void MoveInLookDirection(float d);
		void MoveInLookDirection(float3 delta);

		void LookDown(float r);
		void LookUp(float r);
		void TurnRight(float r);
		void TurnLeft(float r);
		void RollRight(float r);
		void RollLeft(float r);

		void ScaleX(float s);
		void ScaleY(float s);
		void ScaleZ(float s);

		float4x4 GetPositionMatrix();
		float4x4 GetRotationMatrix();
		float4x4 GetScaleMatrix();
		float4x4 GetWorldMatrix();
		void Reset();
		float DistanceSquare(Position& other);
		float Distance(Position& other);
	};
}