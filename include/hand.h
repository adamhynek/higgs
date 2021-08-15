#pragma once

#include <mutex>
#include <array>
#include <deque>

#include "skse64/InternalVR.h"

#include "RE/havok.h"
#include "physics.h"
#include "utils.h"
#include "haptics.h"
#include "finger_animator.h"

#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>


struct Hand
{
	struct SelectedObject
	{
		NiPointer<bhkRigidBody> rigidBody;
		hkpCollidable *collidable = nullptr; // ptr to collidable owned by rigidBody
		TESForm *hitForm;
		BaseExtraList *hitExtraList;
		NiPointer<NiAVObject> shaderNode;
		NiPointer<NiAVObject> hitNode;
		NiPoint3 point; // in meters (havok), the last point that was selected by the collision checks
		UInt32 handle = 0;
		hkpMotion::MotionType savedMotionType = hkpMotion::MotionType::MOTION_INVALID;
		hkInt8 savedQuality = HK_COLLIDABLE_QUALITY_INVALID;
		bool isActor = false;
		bool isBook = false;
		bool isDisconnected = false;
		bool isImpactedProjectile = false;
	};

	struct PulledObject
	{
		NiPointer<bhkRigidBody> rigidBody;
		UInt32 handle = 0;
		hkHalf savedAngularDamping;
	};

	struct TwoHandedState
	{
		NiTransform weaponOffsetNodeLocalTransform;
		NiTransform collisionOffsetNodeLocalTransform;
		NiTransform handToWeapon;
		TESObjectWEAP *weapon;
	};

	enum class State : UInt8
	{
		Idle, // not pointing at anything meaningful
		SelectedFar, // pointing at something meaningful that isn't close
		SelectedClose, // pointing at something that's next to the hand
		SelectionLocked, // player has locked in their selection, i.e. is holding the button
		PreGrabItem, // player wants to grab an object that isn't loaded yet
		PrePullItem, // player wants to pull a piece of armor off, wait for it to spawn
		Pulled, // first few frames when a player is pulling the object towards them
		HeldInit, // held object is moving towards hand
		Held, // player is holding the object in their hand
		HeldBody, // player is holding a body / other constrained object
		GrabFromOtherHand, // wait after requesting the other hand to drop the object so that we can grab it
		GrabExternal, // want to grab an object that we didn't have selected already
		LootOtherHand, // want to loot from what the other hand is holding
		SelectedTwoHand,
		HeldTwoHanded,
	};

	enum class DampingState : UInt8
	{
		Undamped,
		PreDamp,
		Damped,
		TryLeaveDamped
	};

	enum class InputState : UInt8
	{
		Idle,
		Leeway,
		Block,
		Force
	};

	Hand(bool isLeft, BSFixedString name, BSFixedString handNodeName, BSFixedString wandNodeName, BSFixedString fingerNodeNames[5][3], NiPoint3 palmPosHandspace, NiPoint3 rolloverOffset, bool delayGripInput) :
		isLeft(isLeft),
		collisionMapState(isLeft ? CollisionInfo::State::HeldLeft : CollisionInfo::State::HeldRight),
		name(name),
		handNodeName(handNodeName),
		wandNodeName(wandNodeName),
		palmPosHandspace(palmPosHandspace),
		rolloverOffset(rolloverOffset),
		delayGripInput(delayGripInput),
		controllerVelocities(5, NiPoint3()),
		controllerAngularVelocities(5, NiPoint3()),
		playerVelocitiesWorldspace(5, NiPoint3()),
		haptics(isLeft ? BSVRInterface::BSControllerHand::kControllerHand_Left : BSVRInterface::BSControllerHand::kControllerHand_Right),
		fingerAnimator(isLeft, fingerNodeNames)
	{
		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 3; j++) {
				this->fingerNodeNames[i][j] = fingerNodeNames[i][j];
			}
		}
	};

	~Hand() = delete; // Hacky way to prevent trying to free NiPointers when the game quits and memory is fucked

	void Update(Hand &other, bool allowGrab, NiNode *playerWorldNode, bhkWorld *world);
	void ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState, bool allowGrab);

	void PlaySelectionEffect(UInt32 objHandle, NiAVObject *node);
	void StopSelectionEffect(UInt32 objHandle, NiAVObject *node);
	void ResetNearbyDamping();
	void StartNearbyDamping(bhkWorld &world);
	bool FindCloseObject(bhkWorld *world, bool allowGrab, const Hand &other, const NiPoint3 &hkPalmNodePos, const NiPoint3 &castDirection, const bhkSimpleShapePhantom *sphere, bool isTwoHandedOffhand,
		NiPointer<TESObjectREFR> &closestObj, NiPointer<bhkRigidBody> &closestRigidBody, hkVector4 &closestPoint);
	bool FindFarObject(bhkWorld *world, const Hand &other, const NiPoint3 &hkPalmNodePos, const NiPoint3 &castDirection, const NiPoint3 &hkHmdPos, const NiPoint3 &hmdForward, const bhkSimpleShapePhantom *sphere,
		NiPointer<TESObjectREFR> &closestObj, NiPointer<bhkRigidBody> &closestRigidBody, hkVector4 &closestPoint);
	void CreateHandCollision(bhkWorld *world);
	void RemoveHandCollision(bhkWorld *world);
	void UpdateHandCollision(NiAVObject *handNode, bhkWorld *world);
	hkTransform ComputeHandCollisionTransform(NiAVObject *handNode);
	hkTransform ComputeWeaponCollisionTransform(bhkRigidBody *existingWeaponCollision);
	void CreateWeaponCollision(bhkWorld *world);
	void RemoveWeaponCollision(bhkWorld *world);
	void UpdateWeaponCollision(NiPoint3 &playerVelocity);
	bool GetAttachTransform(const TESForm *baseForm, NiTransform &transform);
	bool ComputeInitialObjectTransform(const TESForm *baseForm, NiTransform &transform);
	bool ShouldUsePhysicsBasedGrab(NiNode *root, NiAVObject *node);
	void TransitionHeld(Hand &other, bhkWorld &world, const NiPoint3 &hkPalmNodePos, const NiPoint3 &castDirection, const NiPoint3 &closestPoint, float havokWorldScale, const NiAVObject *handNode, float handSize, TESObjectREFR *selectedObj,
		NiTransform *initialTransform = nullptr, bool playSound = true);
	void TransitionHeldTwoHanded(Hand &other, bhkWorld &world, const NiPoint3 &hkPalmPos, const NiPoint3 &palmDirection, const NiPoint3 &closestPoint,
		float havokWorldScale, const NiAVObject *handNode, float handSize, NiAVObject *weaponNode, TESObjectWEAP *otherHandWeapon, bool playSound = true);
	void TransitionPreGrab(TESObjectREFR *selectedObj, bool isExternal = false);
	bool TransitionGrabExternal(TESObjectREFR *refr);
	void GrabExternalObject(Hand &other, bhkWorld &world, TESObjectREFR *selectedObj, NiNode *objRoot, NiAVObject *collidableNode, NiAVObject *handNode, float handSize, bhkSimpleShapePhantom *sphere, const NiPoint3 &hkPalmNodePos, const NiPoint3 &palmVector, float havokWorldScale);
	void SetPulledDuration(const NiPoint3 &hkPalmNodePos, const NiPoint3 &objPoint);
	NiPointer<NiAVObject> GetFirstPersonHandNode();
	NiPointer<NiAVObject> GetThirdPersonHandNode();
	NiPointer<NiAVObject> GetWeaponOffsetNode(TESObjectWEAP *weapon);
	NiPointer<NiAVObject> GetWeaponCollisionOffsetNode(TESObjectWEAP *weapon);
	NiPointer<NiAVObject> GetWeaponNode(bool thirdPerson);
	float GetHandSize();
	void UpdateHandTransform(NiTransform &worldTransform);
	inline NiPoint3 GetPalmPositionWS(NiTransform &handTransform) { return handTransform * palmPosHandspace; }
	NiPoint3 GetPalmVectorWS(NiMatrix33 &handRotation);
	NiPoint3 GetPointingVectorWS(NiMatrix33 &handRotation);
	NiPoint3 GetHandVelocity();
	bool IsObjectDepositable(TESObjectREFR *refr, NiAVObject *hmdNode, const NiPoint3 &handPos) const;
	bool IsObjectConsumable(TESObjectREFR *refr, NiAVObject *hmdNode, const NiPoint3 &handPos) const;
	bool IsTwoHanding() const;
	UInt32 SpawnEquippedSelectedObject(TESObjectREFR *selectedObj, float zOffsetWhenNotDisconnected);
	bool ShouldDisplayRollover(Hand &other);
	bool IsSafeToClearSavedCollision() const;
	bool IsObjectPullable();
	bool HasExclusiveObject() const;
	bool CanGrabObject() const;
	bool HasHeldObject() const;
	bool CanOtherGrab() const;
	bool HasHeldKeyframed() const;
	bool GetActivateText(std::string &str);
	bool GetActivateButton(std::string &str);
	void SetupRollover(NiAVObject *rolloverNode);
	void SetupSelectionBeam(NiNode *spellOrigin);
	void Select(TESObjectREFR *obj);
	void Deselect();
	void RestoreHandTransform();
	void EndPull();
	void PlayPhysicsSound(const NiPoint3 &location, bool loud = false);
	void TriggerCollisionHaptics(float mass, float separatingVelocity);

	static const int equippedWeaponSlotBase = 32; // First biped slot to have equipped weapons

	HapticsManager haptics;
	FingerAnimator fingerAnimator;

	NiPointer<bhkRigidBody> handBody = nullptr;

	NiPointer<bhkRigidBody> weaponBody = nullptr; // Owned by us - this is our weapon collision
	bhkRigidBody *clonedFromBody = nullptr; // the collision we cloned to create ours
	float maxWeaponSpeed = 100.0f; // m/s
	float maxWeaponAngularSpeed = 100.0f; // rad/s

	const bool isLeft = false;
	const bool delayGripInput = false;
	const CollisionInfo::State collisionMapState = CollisionInfo::State::HeldRight;
	const NiPoint3 palmPosHandspace;
	BSFixedString fingerNodeNames[5][3]; // 5 fingers, 3 joints
	BSFixedString name; // Used for logging
	BSFixedString handNodeName;
	BSFixedString wandNodeName;

	std::mutex deselectLock;

	NiPoint3 rolloverOffset;
	NiMatrix33 rolloverRotation;
	float rolloverScale;

	UInt16 playerCollisionGroup;

	TESEffectShader *itemSelectedShader = nullptr;
	TESEffectShader *itemSelectedShaderOffLimits = nullptr;

	std::deque<NiPoint3> playerVelocitiesWorldspace; // previous n player velocities in skyrim worldspace
	std::deque<NiPoint3> controllerVelocities;
	std::deque<NiPoint3> controllerAngularVelocities;

	std::vector<NiPointer<bhkRigidBody>> nearbyBodies; // This only exists to hold the NiPointers
	std::unordered_map<bhkRigidBody *, std::pair<hkHalf, hkHalf>> nearbyBodyMap;

	SelectedObject selectedObject;
	PulledObject pulledObject;
	TwoHandedState twoHandedState;

	NiPoint3 pulledPointOffset; // Offset from the center of mass of the point we're pulling on the pulled object
	NiPoint3 pullTarget;
	NiPoint3 initialGrabbedObjRelativePosition;

	NiTransform desiredNodeTransformHandSpace;
	NiTransform desiredHavokTransformHandSpace;

	NiTransform handTransform;
	NiTransform adjustedHandTransform;

	NiPoint3 prevPlayerPosWorldspace;

	NiPoint3 prevHeldObjPosPlayerspace;
	NiPoint3 prevHeldObjVelocityPlayerspace;

	bool idleDesired = false;

	bool isExternalGrab = false;
	bool disableDropEvents = false;

	bool externalGrabRequested = false;
	NiPointer<TESObjectREFR> externalGrabRequestedObject = nullptr;

	double lastSelectedTime = 0; // Timestamp of the last time we were pointing at something valid
	double grabRequestedTime = 0; // Timestamp when the trigger was pressed
	double rolloverDisplayTime = 0; // Timestamp when we performed the last action that warrants showing the rollover text
	double grabbedTime = 0; // Timestamp when the currently grabbed object (if there is one) was grabbed
	double droppedTime = 0;
	double pullDuration = 0;
	double pulledExpireTime = 0; // Amount of time after pulling to wait before restoring original collision information
	double pulledTime = 0; // Timestamp when the last pulled object was pulled
	double heldTime = 0;
	double forceInputTime = 0;
	double preDampTime = 0;
	double tryLeaveDampedTime = 0;
	double weaponHitTime = 0;
	double rolloverAlphaSetTime = 0;

	float grabbedFingerValues[5];
	bool useAlternateThumbCurve = false;

	float prevFrameTwistAngle360 = 0;
	bool crossesPi = false;
	bool crossesNegativePi = false;

	State state = State::Idle;
	State prevState = State::Idle;

	DampingState dampingState = DampingState::Undamped;

	InputState inputState = InputState::Idle;
	bool inputTrigger = false;
	bool inputGrip = false;

	bool triggerDown = false; // Whether the trigger was down last frame
	bool isTriggerDisabled = false;
	bool gripDown = false;
	bool grabRequested = false; // True on rising edge of trigger press
	bool releaseRequested = false; // True on falling edge of trigger press
	bool wasObjectGrabbed = false;
};

extern Hand *g_rightHand;
extern Hand *g_leftHand;
