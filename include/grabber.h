#pragma once

#include <mutex>
#include <array>

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
		NiPointer<bhkRigidBody> rigidBody;
		hkpCollidable *collidable = nullptr; // ptr to collidable owned by rigidBody
		TESForm *hitForm;
		NiPointer<NiAVObject> shaderNode;
		NiPointer<NiAVObject> hitNode;
		UInt32 handle = 0;
		hkpMotion::MotionType savedMotionType = hkpMotion::MotionType::MOTION_INVALID;
		hkInt8 savedQuality = HK_COLLIDABLE_QUALITY_INVALID;
		bool isActor = false;
		bool isImpactedProjectile = false;
	};

	struct PulledObject
	{
		NiPointer<bhkRigidBody> rigidBody;
		UInt32 handle = 0;
		hkHalf savedAngularDamping;
	};

	enum class State : UInt8
	{
		Idle, // not pointing at anything meaningful
		SelectedFar, // pointing at something meaningful that isn't close
		SelectedClose, // selected something that's next to the hand
		SelectionLocked, // player has locked in their selection
		PrepullItem , // player wants to pull a piece of armor off
		Pulled, // player is pulling the object towards them
		HeldInit, // held object is moving towards hand
		Held, // player is holding the object in their hand
		HeldBody // player is holding a body
	};

	enum class InputState : UInt8
	{
		Idle,
		Leeway,
		Block,
		Force
	};

	Grabber(bool isLeft, BSFixedString name, BSFixedString handNodeName, BSFixedString upperArmNodeName, BSFixedString wandNodeName, BSFixedString fingerNodeNames[5][3], NiPoint3 palmPosHandspace, NiPoint3 rolloverOffset, bool delayGripInput) :
		isLeft(isLeft),
		name(name),
		handNodeName(handNodeName),
		upperArmNodeName(upperArmNodeName),
		wandNodeName(wandNodeName),
		palmPosHandspace(palmPosHandspace),
		rolloverOffset(rolloverOffset),
		delayGripInput(delayGripInput)
	{
		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 3; j++) {
				this->fingerNodeNames[i][j] = fingerNodeNames[i][j];
			}
		}

		// We don't want to even attempt to call the constructors of these, but we do want space for them
		handCollShape = (hkpBoxShape *)malloc(sizeof(hkpBoxShape));
		handCollCInfo = (hkpRigidBodyCinfo *)malloc(sizeof(hkpRigidBodyCinfo));
		handCollBody = (hkpRigidBody *)malloc(sizeof(hkpRigidBody));
		handCollBody->m_world = nullptr; // a bit weird, but we want to make sure it's 0 for when we check it
	};

	~Grabber() = delete; // Hacky way to prevent trying to free NiPointers when the game quits and memory is fucked

	void PoseUpdate(const Grabber &other, bool allowGrab, NiNode *playerWorldNode);
	void ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState);

	void PlaySelectionEffect(UInt32 objHandle, NiAVObject *node);
	void StopSelectionEffect(UInt32 objHandle, NiAVObject *node);
	bool FindCloseObject(bhkWorld *world, bool allowGrab, const Grabber &other, NiPoint3 &hkPalmNodePos, NiPoint3 &castDirection, bhkSimpleShapePhantom *sphere,
		NiPointer<TESObjectREFR> *closestObj, NiPointer<bhkRigidBody> *closestRigidBody, hkContactPoint *closestPoint);
	void TransitionHeld(bhkWorld *world, NiPoint3 &hkPalmNodePos, NiPoint3 &castDirection, hkContactPoint &closestPoint, float havokWorldScale, NiAVObject *handNode, TESObjectREFR *selectedObj);
	bool ShouldDisplayRollover();
	bool IsSafeToClearSavedCollision();
	bool IsObjectPullable();
	bool HasExclusiveObject() const;
	void SetSelectedHandles(bool isLeftHanded);
	void SetupRollover(NiAVObject *rolloverNode, bool isLeftHanded);
	void Select(TESObjectREFR *obj);
	void Deselect();
	void EndPull();

	static const int equippedWeaponSlotBase = 32; // First biped slot to have equipped weapons

	hkpBoxShape *handCollShape;
	hkpRigidBodyCinfo *handCollCInfo;
	hkpRigidBody *handCollBody;

	const bool isLeft = false;
	const bool delayGripInput = false;
	const NiPoint3 palmPosHandspace;
	BSFixedString fingerNodeNames[5][3]; // 5 fingers, 3 joints
	BSFixedString name; // Used for logging
	BSFixedString handNodeName;
	BSFixedString upperArmNodeName;
	BSFixedString wandNodeName;

	std::mutex deselectLock;

	NiPoint3 rolloverOffset;
	NiMatrix33 rolloverRotation;
	float rolloverScale;

	UInt16 playerCollisionGroup;

	TESEffectShader *itemSelectedShader = nullptr;
	TESEffectShader *itemSelectedShaderOffLimits = nullptr;
	BGSReferenceEffect *itemSelectedEffect = nullptr;
	BGSReferenceEffect *itemSelectedEffectOffLimits = nullptr;

	std::array<NiPoint3, 5> handVelocitiesWorldspace; // previous n hand velocities in skyrim worldspace
	std::array<NiPoint3, 5> handVelocitiesRoomspace; // previous n wand velocities in local roomspace
	std::array<NiPoint3, 5> handDirectionVelocities; // previous n hand direction velocities

	SelectedObject selectedObject;
	PulledObject pulledObject;

	NiPoint3 initialObjPosRaySpace;
	NiPoint3 initialGrabbedObjRelativePosition;
	NiPoint3 initialGrabbedObjWorldPosition;
	float initialHandShoulderDistance = 0;

	NiTransform initialObjTransformHandSpace;

	NiPoint3 prevHandPosWorldspace;
	NiPoint3 prevHandPosRoomspace;
	NiPoint3 prevHandDirectionRoomspace;

	bool idleDesired = false;

	double lastSelectedTime = 0; // Timestamp of the last time we were pointing at something valid
	double grabRequestedTime = 0; // Timestamp when the trigger was pressed
	double selectionLockedTime = 0; // Timestamp when the currently grabbed object (if there is one) was locked for selection
	double grabbedTime = 0; // Timestamp when the currently grabbed object (if there is one) was grabbed
	double pulledExpireTime = 0; // Amount of time after pulling to wait before restoring original collision information
	double pulledTime = 0; // Timestamp when the last pulled object was pulled

	State state = State::Idle;
	State prevState = State::Idle;

	InputState inputState = InputState::Idle;
	bool inputTrigger = false;
	bool inputGrip = false;

	bool forceInput = false;
	bool triggerDown = false; // Whether the trigger was down last frame
	bool gripDown = false;
	bool grabRequested = false; // True on rising edge of trigger press
	bool releaseRequested = false; // True on falling edge of trigger press
	bool wasObjectGrabbed = false;
};

extern Grabber *g_rightGrabber;
extern Grabber *g_leftGrabber;
