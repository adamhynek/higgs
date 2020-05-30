#pragma once

#include "skse64/InternalVR.h"
#include "RE/havok.h"
#include "physics.h"
#include "utils.h"

#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>


struct Grabber
{
	struct SelectedObject
	{
		UInt32 handle = 0;
		hkpCollidable *collidable = nullptr;
		hkpRigidBody *rigidBody = nullptr;
		bool isActor = false;
		bool isImpactedProjectile = false;
		bool hasSavedAngularDamping = false;
		hkHalf savedAngularDamping;
		bool hasSavedCollisionFilterInfo = false;
		UInt32 savedCollisionFilterInfo = 0;
		bool hasSavedMotionType = false;
		hkpMotion::MotionType savedMotionType = hkpMotion::MotionType::MOTION_INVALID;
		hkInt8 savedQuality = HK_COLLIDABLE_QUALITY_INVALID;
		TESForm *hitForm = nullptr;
		NiPointer<NiAVObject> shaderNode = nullptr;
		NiPointer<NiAVObject> hitNode = nullptr;
	};

	enum State : int
	{
		IDLE, // not pointing at anything meaningful
		SELECTED_FAR, // pointing at something meaningful that isn't close
		SELECTED_CLOSE, // selected something that's next to the hand
		SELECTION_LOCKED, // player has locked in their selection
		GRABBED, // player has grabbed the selected object
		PULLED, // player is pulling the object towards them
		HELD // player is holding the object in their hand
	};

	Grabber(BSFixedString handNodeName, BSFixedString upperArmNodeName, BSFixedString wandNodeName, BSFixedString palmNodeName, NiPoint3 rolloverOffset)
		: handNodeName(handNodeName), upperArmNodeName(upperArmNodeName), wandNodeName(wandNodeName), palmNodeName(palmNodeName), rolloverOffset(rolloverOffset)
	{
		// We don't want to even attempt to call the constructors of these, but we do want space for them
		handCollShape = (hkpBoxShape *)malloc(sizeof(hkpBoxShape));
		handCollCInfo = (hkpRigidBodyCinfo *)malloc(sizeof(hkpRigidBodyCinfo));
		handCollBody = (hkpRigidBody *)malloc(sizeof(hkpRigidBody));
	};

	void PoseUpdate(const Grabber &other, bool allowGrab, NiNode *playerWorldNode);
	void ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState);
	bool ShouldDisplayRollover();
	bool IsObjectPullable();
	bool HasExclusiveObject() const;
	void SetupRollover(NiAVObject *rolloverNode, bool isLeftHanded);
	void Select(TESObjectREFR *obj);
	void Deselect();

	static const int equippedWeaponSlotBase = 32; // First biped slot to have equipped weapons

	static const int numPrevVel = 5;

	State state = IDLE;
	State prevState = IDLE;

	hkpWorld *savedWorld = nullptr;

	hkpBoxShape *handCollShape;
	hkpRigidBodyCinfo *handCollCInfo;
	hkpRigidBody *handCollBody;

	BSFixedString handNodeName;
	BSFixedString upperArmNodeName;
	BSFixedString wandNodeName;
	BSFixedString palmNodeName;

	NiPoint3 rolloverOffset;
	NiMatrix33 rolloverRotation;
	float rolloverScale;

	TESEffectShader *itemSelectedShader = nullptr;
	TESEffectShader *itemSelectedShaderOffLimits = nullptr;

	NiPoint3 handVelocities[numPrevVel]; // previous n hand velocities
	NiPoint3 handDirectionVelocities[numPrevVel]; // previous n hand direction velocities

	SelectedObject selectedObject;

	NiPoint3 initialObjPosRaySpace;
	NiPoint3 initialGrabbedObjRelativePosition;
	NiPoint3 initialGrabbedObjWorldPosition;
	float initialHandShoulderDistance = 0;

	NiTransform initialObjTransformHandSpace;

	NiPoint3 prevHandPosRoomspace;
	NiPoint3 prevHandDirectionRoomspace;

	float prevHandSpeedInSpellDirection = 0;
	float prevHandSpeedInObjDirection = 0;

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
