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
	};

	struct GrabbedObject
	{
		UInt32 handle = 0;
		hkpCollidable *collidable = nullptr;
		bool isActor = false;
		bool isImpactedProjectile = false;
	};

	Grabber(BSFixedString handNodeName, BSFixedString upperArmNodeName, BSFixedString wandNodeName, NiPoint3 rolloverOffset)
		: handNodeName(handNodeName), upperArmNodeName(upperArmNodeName), wandNodeName(wandNodeName), rolloverOffset(rolloverOffset)
	{
		for (int i = 0; i < numPrevPos; i++) {
			handPositions[i] = { 0, 0, 0 };
		}
	};

	void PoseUpdate(const Grabber &other, bool allowGrab);
	void ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState);
	bool ShouldDisplayRollover(const TESObjectREFR *grabbedObj);
	void SetupRollover(NiAVObject *rolloverNode, const TESObjectREFR *grabbedObj, bool isLeftHanded);
	void Select(TESObjectREFR *obj, const SelectedObject &other);
	void Deselect(TESObjectREFR *obj, const SelectedObject &other);
	void Grab(TESObjectREFR *obj);
	void UnGrab();


	static const int numPrevPos = 5;

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
	GrabbedObject grabbedObject;
	TESObjectREFR *prevGrabbedObj = nullptr;

	NiPoint3 initialGrabbedObjRelativePosition = { 0, 0, 0 };
	float initialHandShoulderDistance = 0;
	NiPoint3 prevHandPosRoomspace = { 0, 0, 0 };

	float prevHandSpeedInSpellDirection = 0;

	bool pullDesired = false;
	float pullSpeed = 0;

	bool pushDesired = false;

	bool unsheatheDesired = false;

	long long lastSelectedTime = 0; // Timestamp of the last time we were pointing at something valid
	long long triggerPressedTime = 0; // Timestamp when the trigger was pressed

	bool triggerDown = false; // Whether the trigger was down last frame
	bool triggerPressed = false; // True on rising edge of trigger press
	bool triggerReleased = false; // True on falling edge of trigger press
	bool didTriggerPressGrabObject = false;
};
