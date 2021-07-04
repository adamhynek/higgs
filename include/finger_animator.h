#pragma once

#include "skse64/NiTypes.h"

struct FingerAnimator
{
	enum class AnimState : UInt8
	{
		Start,
		End
	};

	FingerAnimator(bool isLeft, BSFixedString fingerNodeNames[5][3]) :
		isLeft(isLeft)
	{
		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 3; j++) {
				this->fingerNodeNames[i][j] = fingerNodeNames[i][j];
			}
		}
	}

	NiPoint3 LerpFingerPosition(int finger, int knuckle, float lerpVal);
	NiMatrix33 LerpFingerRotation(int finger, int knuckle, float lerpVal);
	void Update();
	void SetFingerValues(float values[5], float linearSpeed, float angularSpeed);
	void SetFingerValues(float value, float linearSpeed, float angularSpeed);
	void RestoreFingers();

	BSFixedString fingerNodeNames[5][3];
	NiTransform localTransforms[5][3];
	double restoreFingersTime;
	float animValues[5];
	float animSpeedLinear;
	float animSpeedAngular;
	AnimState animState = AnimState::Start;
	bool animate = false;
	bool saveVrikTransforms = false;
	bool isLeft;
};
