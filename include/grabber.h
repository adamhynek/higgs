#pragma once

#include "skse64/InternalVR.h"
#include "RE/havok.h"
#include "physics.h"


static BSFixedString hmdNodeStr("HmdNode");

// Gets callbacks from havok linear cast
static CdPointCollector cdPointCollector;
static hkpLinearCastInput linearCastInput;
static RayHitCollector rayHitCollector;
// 'ItemPicker' collision layer; player collision group
// Why ItemPicker? It ignores stuff like weapons the player is holding
//static hkpWorldRayCastInput rayCastInput(0x02420028);
static hkpWorldRayCastInput rayCastInput(0x00090028);


struct Grabber
{
	struct SelectedObject
	{
		UInt32 handle = 0;
		hkpCollidable *collidable = nullptr;
		TESEffectShader *appliedShader = nullptr;
		bool isActor = false;
		bool isImpactedProjectile = false;
	};

	enum State : int
	{
		IDLE, // not pointing at anything meaningful
		SELECTED, // pointing at something meaningful
		SELECTION_LOCKED, // player has locked in their selection
		GRABBED, // player has grabbed the selected object
		PULLED // player is pulling the object towards them
	};

	Grabber(BSFixedString handNodeName, BSFixedString upperArmNodeName, BSFixedString wandNodeName, NiPoint3 rolloverOffset)
		: handNodeName(handNodeName), upperArmNodeName(upperArmNodeName), wandNodeName(wandNodeName), rolloverOffset(rolloverOffset)
	{
	};

	void PoseUpdate(const Grabber &other, bool allowGrab, NiNode *playerWorldNode);
	void ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState);
	bool ShouldDisplayRollover();
	bool IsObjectPullable();
	bool HasExclusiveObject() const;
	void SetupRollover(NiAVObject *rolloverNode, bool isLeftHanded);
	void Select(TESObjectREFR *obj, const SelectedObject &other);
	void Deselect(TESObjectREFR *obj, const SelectedObject &other);


	static const int numPrevPos = 5;

	State state = IDLE;

	BSFixedString handNodeName;
	BSFixedString upperArmNodeName;
	BSFixedString wandNodeName;

	NiPoint3 rolloverOffset;
	NiMatrix33 rolloverRotation;
	float rolloverScale;

	TESEffectShader *itemSelectedShader = nullptr;
	TESEffectShader *itemSelectedShaderOffLimits = nullptr;

	NiPoint3 handPositions[numPrevPos]; // previous n hand positions

	SelectedObject selectedObject;
	TESObjectREFR *prevGrabbedObj = nullptr;

	NiPoint3 initialGrabbedObjRelativePosition;
	NiPoint3 initialGrabbedObjWorldPosition;
	NiPoint3 initialHandDirection;
	float initialHandShoulderDistance = 0;
	NiPoint3 prevHandPosRoomspace;
	NiPoint3 prevHandDirection;

	float prevHandSpeedInSpellDirection = 0;

	float pullSpeed = 0;

	bool unsheatheDesired = false;

	double lastSelectedTime = 0; // Timestamp of the last time we were pointing at something valid
	double triggerPressedTime = 0; // Timestamp when the trigger was pressed
	double selectionLockedTime = 0; // Timestamp when the currently grabbed object (if there is one) was locked for selection
	double grabbedTime = 0; // Timestamp when the currently grabbed object (if there is one) was grabbed

	bool triggerDown = false; // Whether the trigger was down last frame
	bool triggerPressed = false; // True on rising edge of trigger press
	bool triggerReleased = false; // True on falling edge of trigger press
	bool didTriggerPressGrabObject = false;
};
