#pragma once

#include "skse64/InternalVR.h"
#include "RE/havok.h"
#include "physics.h"
#include "utils.h"
#include "havok_ref_ptr.h"

#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>


struct Grabber
{
	struct SelectedObject
	{
		UInt32 handle = 0;
		havokRefPtr<hkpRigidBody> rigidBody;
		hkpCollidable *collidable = nullptr; // ptr to collidable owned by rigidBody
		bool isActor = false;
		bool isImpactedProjectile = false;
		hkpMotion::MotionType savedMotionType = hkpMotion::MotionType::MOTION_INVALID;
		hkInt8 savedQuality = HK_COLLIDABLE_QUALITY_INVALID;
		TESForm *hitForm;
		NiPointer<NiAVObject> shaderNode;
		NiPointer<NiAVObject> hitNode;
	};

	struct PulledObject
	{
		UInt32 handle = 0;
		havokRefPtr<hkpRigidBody> rigidBody;
		hkHalf savedAngularDamping;
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
		handCollBody->m_world = nullptr; // a bit weird, but we want to make sure it's 0 for when we check it
	};

	void PoseUpdate(const Grabber &other, bool allowGrab, NiNode *playerWorldNode);
	void ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState);
	bool ShouldDisplayRollover();
	bool IsObjectPullable();
	bool HasExclusiveObject() const;
	void SetupRollover(NiAVObject *rolloverNode, bool isLeftHanded);
	void Select(TESObjectREFR *obj);
	void Deselect();
	void EndPull();

	static const int equippedWeaponSlotBase = 32; // First biped slot to have equipped weapons

	static const int numPrevVel = 5;
	static const int numPullFrames = 5; // TODO: Should be time, not frames

	State state = IDLE;
	State prevState = IDLE;

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

	UInt16 playerCollisionGroup;

	TESEffectShader *itemSelectedShader = nullptr;
	TESEffectShader *itemSelectedShaderOffLimits = nullptr;

	NiPoint3 handVelocities[numPrevVel]; // previous n hand velocities
	NiPoint3 handDirectionVelocities[numPrevVel]; // previous n hand direction velocities

	SelectedObject selectedObject;
	PulledObject pulledObject;

	NiPoint3 initialObjPosRaySpace;
	NiPoint3 initialGrabbedObjRelativePosition;
	NiPoint3 initialGrabbedObjWorldPosition;
	float initialHandShoulderDistance = 0;

	NiTransform initialObjTransformHandSpace;

	NiPoint3 prevHandPosRoomspace;
	NiPoint3 prevHandDirectionRoomspace;

	float prevHandSpeedInSpellDirection = 0;
	float prevHandSpeedInObjDirection = 0;

	int pullFrameCounter = 0;

	bool idleDesired = false;
	bool unsheatheDesired = false;

	double lastSelectedTime = 0; // Timestamp of the last time we were pointing at something valid
	double triggerPressedTime = 0; // Timestamp when the trigger was pressed
	double selectionLockedTime = 0; // Timestamp when the currently grabbed object (if there is one) was locked for selection
	double grabbedTime = 0; // Timestamp when the currently grabbed object (if there is one) was grabbed
	double pulledExpireTime = 0; // Amount of time after pulling to wait before restoring original collision information
	double pulledTime = 0; // Timestamp when the last pulled object was pulled

	bool triggerDown = false; // Whether the trigger was down last frame
	bool triggerPressed = false; // True on rising edge of trigger press
	bool triggerReleased = false; // True on falling edge of trigger press
	bool didTriggerPressGrabObject = false;
};
