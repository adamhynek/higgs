#include <sstream>

#include "skse64/NiNodes.h"

#include "finger_animator.h"
#include "finger_curves.h"
#include "math_utils.h"
#include "config.h"
#include "utils.h"
#include "RE/offsets.h"


NiPoint3 FingerAnimator::LerpFingerPosition(int finger, int knuckle, float lerpVal)
{
	NiPoint3 pos = lerp(g_closedFingerPositions[finger][knuckle], g_openFingerPositions[finger][knuckle], lerpVal);
	if (isLeft) {
		pos.x *= -1;
	}
	return pos;
}

NiMatrix33 FingerAnimator::LerpFingerRotation(int finger, int knuckle, float lerpVal)
{
	NiMatrix33 rot = QuaternionToMatrix(slerp(g_closedFingerRotations[finger][knuckle], g_openFingerRotations[finger][knuckle], lerpVal));
	if (isLeft) {
		NiPoint3 euler = MatrixToEuler(rot);
		euler.y *= -1;
		euler.z *= -1;
		rot = EulerToMatrix(euler);
	}
	return rot;
}

bool FillFingerNodes(NiPointer<NiAVObject> fingerNodes[5][3], BSFixedString fingerNodeNames[5][3])
{
	PlayerCharacter *player = *g_thePlayer;
	NiPointer<NiNode> tpNode = player->GetNiRootNode(0);
	if (tpNode) {
		for (int i = 0; i < 5; i++) {
			fingerNodes[i][0] = tpNode->GetObjectByName(&fingerNodeNames[i][0].data);
			if (!fingerNodes[i][0]) return false;
			fingerNodes[i][1] = fingerNodes[i][0]->GetObjectByName(&fingerNodeNames[i][1].data);
			if (!fingerNodes[i][1]) return false;
			fingerNodes[i][2] = fingerNodes[i][1]->GetObjectByName(&fingerNodeNames[i][2].data);
			if (!fingerNodes[i][2]) return false;
		}
		return true;
	}
	return false;
}

void FingerAnimator::Update()
{
	if (!animate) return;

	NiPointer<NiAVObject> fingerNodes[5][3];
	if (!FillFingerNodes(fingerNodes, fingerNodeNames)) return;

	float posSpeed = animSpeedLinear;
	float rotSpeed = animSpeedAngular;
	if (animState == AnimState::End) {
		double elapsedTimeFraction = (g_currentFrameTime - restoreFingersTime) / Config::options.fingerAnimateEndTime;
		posSpeed += elapsedTimeFraction * animSpeedLinear;
		rotSpeed += elapsedTimeFraction * animSpeedAngular;
	}

	bool stopAnimating = true;
	//std::vector<NiAVObject *> stillAnimating;
	auto AnimateFinger = [this, &fingerNodes, &stopAnimating, posSpeed, rotSpeed](int finger, float lerpVal)
	{
		// What we should do:
		// - When starting to animate, start from where the fingers started, and move towards the desired state at a constant speed
		// - When ending animating, start from where _we_ had them last (NOT oldWorldTransform), and move towards oldWorldTransform at a constant speed
		//   - This raises a question: When do we terminate?

		// Current 3rd person finger transform: The transform that vrik sets
		// Old 3rd person finger transform: Ours if we set it last frame, vriks if we didn't

		for (int i = 0; i < 3; i++) {
			NiTransform desiredTransform = fingerNodes[finger][i]->m_localTransform;
			if (animState == AnimState::Start) {
				desiredTransform.pos = LerpFingerPosition(finger, i, lerpVal);
				desiredTransform.rot = LerpFingerRotation(finger, i, lerpVal);
			}

			NiAVObject *fingerNode = fingerNodes[finger][i];
			NiTransform &vrikLocalTransform = fingerNodes[finger][i]->m_localTransform;
			if (saveVrikTransforms) {
				localTransforms[finger][i] = vrikLocalTransform;
			}

			NiTransform &currentTransform = localTransforms[finger][i]; // Is the vrik transform if we just started animating, or our last transform if we already were animating

			std::optional<NiTransform> advancedTransform = AdvanceTransform(currentTransform, desiredTransform, posSpeed, rotSpeed);
			NiTransform finalTransform;
			if (advancedTransform) {
				// This part of the finger has not reached its destination yet - keep animating
				finalTransform = *advancedTransform;
				stopAnimating = false;
				//stillAnimating.push_back(fingerNode);
			}
			else {
				finalTransform = desiredTransform;
			}
			localTransforms[finger][i] = finalTransform;
			fingerNodes[finger][i]->m_localTransform = finalTransform;
		}

		NiAVObject::ControllerUpdateContext ctx{ 0, 0 };
		NiAVObject_UpdateObjectUpwards(fingerNodes[finger][0], &ctx);
	};

	for (int i = 0; i < 5; i++) {
		AnimateFinger(i, animValues[i]);
	}

	/*std::stringstream str;
	for (NiAVObject *node : stillAnimating) {
		str << node->m_name << ", ";
	}
	_MESSAGE(str.str().c_str());*/

	if (animState == AnimState::End && (stopAnimating || g_currentFrameTime - restoreFingersTime > Config::options.fingerAnimateEndTime)) {
		animate = false;
	}

	saveVrikTransforms = false;
}

void FingerAnimator::SetFingerValues(float values[5], float linearSpeed, float angularSpeed)
{
	for (int i = 0; i < 5; i++) {
		animValues[i] = values[i];
	}

	if (!animate) {
		saveVrikTransforms = true;
	}

	animSpeedLinear = linearSpeed;
	animSpeedAngular = angularSpeed;
	animState = AnimState::Start;
	animate = true;
}

void FingerAnimator::SetFingerValues(float value, float linearSpeed, float angularSpeed)
{
	float values[5] = { value, value, value, value, value };
	SetFingerValues(values, linearSpeed, angularSpeed);
}

void FingerAnimator::RestoreFingers()
{
	if (!animate) return;
	animSpeedLinear = Config::options.fingerAnimateEndLinearSpeed;
	animSpeedAngular = Config::options.fingerAnimateEndAngularSpeed;
	restoreFingersTime = g_currentFrameTime;
	animState = AnimState::End;
}
