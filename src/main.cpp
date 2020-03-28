#include <functional>
#include <string>
#include <regex>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <limits>
#include <atomic>
#include "common/IDebugLog.h"  // IDebugLog
#include "skse64_common/skse_version.h"  // RUNTIME_VERSION
#include "skse64/PluginAPI.h"  // SKSEInterface, PluginInfo
#include "skse64/GameRTTI.h"
#include "skse64/GameSettings.h"
#include "skse64/NiNodes.h"
#include "skse64/NiObjects.h"
#include "skse64/NiExtraData.h"
#include "skse64/GameData.h"
#include "skse64/GameForms.h"
#include "skse64/PapyrusActor.h"
#include "skse64/GameVR.h"
#include "skse64_common/SafeWrite.h"
#include "skse64_common/BranchTrampoline.h"
#include "xbyak/xbyak.h"

#include <ShlObj.h>  // CSIDL_MYDOCUMENTS

#include "main.h"
#include "version.h"
#include "physics.h"
#include "utils.h"
#include "config.h"
#include "MenuChecker.h"


// Multiply skyrim coords by this to get havok coords
// It's the number of meters per skyrim unit
RelocPtr<float> HAVOK_WORLD_SCALE_ADDR(0x15B78F4);

// Alternatively, 0x30008E0 + 0x78
// Even better, (*0x2FC60C0) + 0x78
// Address of pointer to bhkSimpleShapePhantom that tracks the right hand - more or less
RelocPtr<bhkSimpleShapePhantom *> SPHERE_SHAPE_ADDR(0x3000958);

RelocPtr<UInt32 *> SELECTED_HANDLES(0x2FC60C0);


// SKSE globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

VMClassRegistry *vmRegistry = nullptr;

SKSEVRInterface *g_vrInterface = nullptr;

// Gets callbacks from havok linear cast
CdPointCollector cdPointCollector;
hkpLinearCastInput linearCastInput;
RayHitCollector rayHitCollector;
hkpWorldRayCastInput rayCastInput(0x02420028); // 'ItemPicker' collision layer; player collision group

// Config params
float g_castDistance = 5.0f;
float g_castRadius = 0.4f;
float g_handActivateDistance = 30.0f;
float g_requiredCastDotProduct = cosf(50.0f * 0.0174533);
float g_hoverVelocityMultiplier = 0.17f;
float g_pullVelocityMultiplier = 0.9f;
float g_pushVelocityMultiplier = 0.9f;
float g_bodyVelocityMultiplier = 0.6f;
float g_massExponent = 0.5;
long long g_selectedLeewayTime = 250; // in ms, time to keep something selected after not pointing at it anymore
bool g_equipWeapons = false;

NiPoint3 g_handAdjust = { -0.018, -0.965, 0.261 };

NiPoint3 g_rolloverOffset = { 7, -5, -2 };
NiMatrix33 g_rolloverRotation; // Set on plugin load
float g_rolloverScale = 10.0f;

bool g_isLeftHanded = false;

BSFixedString rolloverNodeStr("WSActivateRollover");
BSFixedString hmdNodeStr("HmdNode");

long long g_triggerPressedLeewayTime = 300; // in ms, time after pressing the trigger after which the trigger is considered not pressed anymore

bool g_isLoaded = false;

TESEffectShader *g_itemSelectedShader = nullptr;
TESEffectShader *g_itemSelectedShaderOffLimits = nullptr;
TESEffectShader *g_itemSelectedShader2 = nullptr;
TESEffectShader *g_itemSelectedShaderOffLimits2 = nullptr;

const int numPrevPos = 5; // length of previous kept hand positions

bool g_hasSavedRumbleIntensity = false;
float g_normalRumbleIntensity;
bool g_hasSavedRollover = false;
NiTransform g_normalRolloverTransform;

struct Grabber
{
	BSFixedString handNodeName;
	BSFixedString upperArmNodeName;
	BSFixedString wandNodeName;

	NiPoint3 rolloverOffset;
	NiMatrix33 rolloverRotation;
	float rolloverScale;

	TESEffectShader *itemSelectedShader = nullptr;
	TESEffectShader *itemSelectedShaderOffLimits = nullptr;
	bool isShaderPlaying = false;

	NiPoint3 handPositions[numPrevPos]; // previous n hand positions

	UInt32 grabbedObjHandle = 0;
	UInt32 selectedObjHandle = 0;
	TESObjectREFR *prevGrabbedObj = nullptr;
	hkpCollidable *selectedColl = nullptr;
	hkpCollidable *grabbedColl = nullptr;
	bool isGrabbedObjActor = false;
	bool isGrabbedObjImpactedProjectile = false;

	NiPoint3 initialGrabbedObjRelativePosition = { 0, 0, 0 };
	float initialHandHorizontalDistance = 0;
	NiPoint3 prevHandPosRoomspace = { 0, 0, 0 };

	bool pullDesired = false;
	bool pushDesired = false;

	bool unsheatheDesired = false;

	long long lastSelectedTime = 0; // Timestamp of the last time we were pointing at something valid
	long long triggerPressedTime = 0; // Timestamp when the trigger was pressed

	bool triggerDown = false; // Whether the trigger was down last frame
	bool triggerPressed = false; // True on rising edge of trigger press
	bool triggerReleased = false; // True on falling edge of trigger press
	bool didTriggerPressGrabObject = false;

	TESEffectShader *currentSelectedShader = nullptr; // Shader currently applied to grabbed object	

	Grabber(BSFixedString handNodeName, BSFixedString upperArmNodeName, BSFixedString wandNodeName)
		: handNodeName(handNodeName), upperArmNodeName(upperArmNodeName), wandNodeName(wandNodeName) {};

	void PoseUpdate(const Grabber &other);
	void ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState);
	void SetupRollover(NiAVObject *rolloverNode, const TESObjectREFR *grabbedObj);
};

Grabber g_rightGrabber("NPC R Hand [RHnd]", "NPC R UpperArm [RUar]", "RightWandNode");
Grabber g_leftGrabber("NPC L Hand [LHnd]", "NPC L UpperArm [LUar]", "LeftWandNode");


// Called on each update (about 90-100 calls per second)
void Grabber::PoseUpdate(const Grabber &other)
{
	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->loadedState || !player->loadedState->node)
		return;

	TESObjectCELL* cell = player->parentCell;
	if (!cell)
		return;

	long long currentTime = GetTime();


	NiAVObject *handNode = player->GetNiRootNode(1)->GetObjectByName(&handNodeName.data);
	if (!handNode)
		return;

	NiPoint3 handPos = handNode->m_worldTransform.pos;

	NiPoint3 castDirection = handNode->m_worldTransform.rot * g_handAdjust;

	NiAVObject *hmdNode = player->loadedState->node->m_parent->GetObjectByName(&hmdNodeStr.data);

	NiPoint3 hmdPos = hmdNode->m_worldTransform.pos;

	NiPoint3 hmdForward = { hmdNode->m_worldTransform.rot.data[0][1], hmdNode->m_worldTransform.rot.data[1][1], hmdNode->m_worldTransform.rot.data[2][1] };

	NiAVObject *wandNode = player->loadedState->node->m_parent->GetObjectByName(&wandNodeName.data);
	NiPoint3 handPosRoomspace = wandNode->m_localTransform.pos;

	// Update positions array to this frame
	for (int i = numPrevPos - 1; i >= 1; i--) {
		handPositions[i] = handPositions[i - 1];
	}
	handPositions[0] = handPosRoomspace;

	NiAVObject *upperArmNode = player->GetNiRootNode(0)->GetObjectByName(&upperArmNodeName.data);
	NiPoint3 upperArmPos = upperArmNode->m_worldTransform.pos;

	NiPointer<TESObjectREFR> grabbedObj;

	if (!LookupREFRByHandle(grabbedObjHandle, grabbedObj)) {
		// Convert hand position from skyrim coords to havok coords
		float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;
		NiPoint3 hkHmdPos = hmdPos * havokWorldScale;
		NiPoint3 hkHandPos = handPos * havokWorldScale;
		NiPoint3 hkTargetPos = hkHandPos + castDirection * g_castDistance;

		NiPoint3 hitPosition = { hkTargetPos.x, hkTargetPos.y, hkTargetPos.z };

		bhkWorld *world = GetWorld(cell);
		if (!world) {
			_MESSAGE("Could not get havok world from player cell");
			return;
		}

		// First, raycast in the pointing direction
		rayHitCollector.reset();
		rayCastInput.m_from = { hkHandPos.x, hkHandPos.y, hkHandPos.z, 0 };
		rayCastInput.m_to = { hkTargetPos.x, hkTargetPos.y, hkTargetPos.z, 0 };
		hkpWorld_CastRay(world->world, &rayCastInput, &rayHitCollector);
		if (rayHitCollector.m_doesHitExist) {
			// If raycast hit, we want to linearcast only up to the ray hit location
			hitPosition = hkHandPos + (hkTargetPos - hkHandPos) * rayHitCollector.m_closestHitInfo.m_hitFraction;
			//hitPosition += castDirection * 0.4f; // add a bit extra past the hit location
		}

		// Now, linearcast up to the point the raycast hit, or up to the limit if it's empty space
		bhkSimpleShapePhantom *sphere = *SPHERE_SHAPE_ADDR;
		cdPointCollector.reset();
		auto sphereShape = (hkpConvexShape *)sphere->phantom->m_collidable.m_shape;
		UInt32 filterInfoBefore = sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo;
		sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = 0; // We want to hit _anything_, including in-flight projectiles
		float radiusBefore = sphereShape->m_radius; // save radius so we can restore it
		sphereShape->m_radius = g_castRadius;
		hkVector4 translationBefore = sphere->phantom->m_motionState.m_transform.m_translation;
		sphere->phantom->m_motionState.m_transform.m_translation = { hkHandPos.x, hkHandPos.y, hkHandPos.z, VectorLength(hkHandPos) };

		linearCastInput.m_to = { hitPosition.x, hitPosition.y, hitPosition.z, 0 };
		hkpWorld_LinearCast(world->world, &sphere->phantom->m_collidable, &linearCastInput, &cdPointCollector, nullptr);

		sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = filterInfoBefore;
		sphereShape->m_radius = radiusBefore;
		sphere->phantom->m_motionState.m_transform.m_translation = translationBefore;

		// Process result of cast
		NiPointer<TESObjectREFR> closestObj;
		hkpCollidable *closestColl = nullptr;
		float closestDistance = (std::numeric_limits<float>::max)();
		NiPoint3 closestHit;

		for (auto pair : cdPointCollector.m_hits) {
			auto collidable = static_cast<hkpCollidable *>(pair.second);
			if (collidable == other.selectedColl) {
				continue;
			}
			NiPointer<TESObjectREFR> ref = FindCollidableRef(collidable);
			if (ref) {
				if (IsAllowedCollidable(collidable) || ref->baseForm && ref->baseForm->formType == kFormType_Projectile) {
					if (ref->baseForm->formType == kFormType_Projectile) {
						auto impactData = *(void **)((UInt64)ref.m_pObject + 0x98);
						if (!impactData) {
							// Only grab projectiles that are not mid flight
							continue;
						}
					}
					// Get distance from the hit on the collidable to the ray
					NiPoint3 hit = { pair.first.x, pair.first.y, pair.first.z };
					NiPoint3 handToHit = hit - hkHandPos;
					NiPoint3 handToHitAlongRay = castDirection * DotProduct(handToHit, castDirection); // project above vector onto ray
					float dist = VectorLength(handToHit - handToHitAlongRay); // distance from hit location to closest point on the ray
					if (dist < closestDistance) {
						closestObj = ref;
						closestColl = collidable;
						closestDistance = dist;
						closestHit = hit;
					}
				}
			}
		}

		// Select the new thing
		bool isSelected = false;
		if (closestObj && DotProduct(VectorNormalized(closestHit - hkHmdPos), hmdForward) >= g_requiredCastDotProduct) {
			NiPointer<TESObjectREFR> selectedObj;
			if (!LookupREFRByHandle(selectedObjHandle, selectedObj) ||  closestObj != selectedObj) {
				if (selectedObj) {
					// Deselect the old thing if something else was selected
					if (isShaderPlaying) {
						EffectShader_Stop(vmRegistry, 0, currentSelectedShader, selectedObj);
						isShaderPlaying = false;
					}
				}
				selectedObjHandle = GetOrCreateRefrHandle(closestObj.m_pObject);

				if (CALL_MEMBER_FN(closestObj, IsOffLimits)()) {
					currentSelectedShader = itemSelectedShaderOffLimits;
				}
				else {
					currentSelectedShader = itemSelectedShader;
				}
			}
			selectedColl = closestColl; // Set selected collidable no matter what, as we can have objects with more than one collidable

			isSelected = true;
			lastSelectedTime = currentTime;
		}

		// If time has run out and nothing is selected, deselect whatever is selected
		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObjHandle, selectedObj) && !isSelected && currentTime - lastSelectedTime > g_selectedLeewayTime) {
			if (isShaderPlaying) {
				EffectShader_Stop(vmRegistry, 0, currentSelectedShader, selectedObj);
				isShaderPlaying = false;
			}
			selectedObjHandle = *g_invalidRefHandle;
			selectedColl = nullptr;
		}

		if (triggerPressed && currentTime - triggerPressedTime <= g_triggerPressedLeewayTime) {
			if (LookupREFRByHandle(selectedObjHandle, selectedObj)) {
				// Pick up the item

				grabbedColl = selectedColl;
				grabbedObjHandle = selectedObjHandle;

				// Set to false only here, so that you can hold the trigger until the cast hits something valid
				triggerPressed = false;
				didTriggerPressGrabObject = true; // This variable is not set to false when we push/pull the object

				isGrabbedObjImpactedProjectile = false;
				auto baseForm = selectedObj->baseForm;
				if (baseForm && baseForm->formType == kFormType_Projectile) {
					auto impactData = *(void **)((UInt64)selectedObj.m_pObject + 0x98);
					if (impactData) {
						// If the projectile has impact data, then it has well, impacted something
						isGrabbedObjImpactedProjectile = true;
					}
				}

				isGrabbedObjActor = false;
				auto actor = DYNAMIC_CAST(selectedObj, TESObjectREFR, Actor);
				if (actor) {
					isGrabbedObjActor = true;
				}
			}
		}
	}

	// Play effect shader only if the other hand doesn't have its shader already playing on the object
	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObjHandle, selectedObj) && !isShaderPlaying) {
		NiPointer<TESObjectREFR> otherSelectedObj;
		UInt32 otherSelectedObjHandle = other.selectedObjHandle;
		if (!LookupREFRByHandle(otherSelectedObjHandle, otherSelectedObj) || otherSelectedObj != selectedObj) {
			EffectShader_Play(vmRegistry, 0, currentSelectedShader, selectedObj, -1.0f);
			isShaderPlaying = true;
		}
	}

	if (triggerReleased) {
		triggerReleased = false;
		didTriggerPressGrabObject = false;

		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObjHandle, selectedObj)) {
			if (isShaderPlaying) {
				EffectShader_Stop(vmRegistry, 0, currentSelectedShader, selectedObj);
				isShaderPlaying = false;
			}
			selectedObjHandle = *g_invalidRefHandle;
			selectedColl = nullptr;
		}
		if (LookupREFRByHandle(grabbedObjHandle, grabbedObj)) {
			// Drop the item
			grabbedObjHandle = *g_invalidRefHandle;
			pullDesired = false;
		}
	}

	if (LookupREFRByHandle(grabbedObjHandle, grabbedObj) && grabbedObj->loadedState && grabbedObj->loadedState->node) {
		float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;

		hkpMotion *motion;
		if (grabbedObj->loadedState->node->unk040) {
			auto collObj = (bhkCollisionObject *)grabbedObj->loadedState->node->unk040;
			motion = &collObj->body->hkBody->motion;
		}
		else {
			motion = reinterpret_cast<hkpMotion *>((UInt64)grabbedColl->m_motion - offsetof(hkpMotion, m_motionState));
		}
		hkVector4 translation = motion->m_motionState.m_transform.m_translation;

		NiPoint3 hkObjPos = { translation.x, translation.y, translation.z };
		NiPoint3 hkHandPos = handPos * havokWorldScale;

		NiPoint3 relObjPos = hkObjPos - hkHandPos;

		if (!prevGrabbedObj) {
			pushDesired = false;
			pullDesired = false;

			initialGrabbedObjRelativePosition = relObjPos;
			initialHandHorizontalDistance = VectorLength(handPos - upperArmPos);

			if (isGrabbedObjImpactedProjectile) { // It's an embedded projectile, i.e. stuck in a wall etc.
				auto collObj = (bhkCollisionObject *)grabbedObj->loadedState->node->unk040;
				if (collObj) {
					// Do not use pullColl here, it's not the right collidable to set collision for
					auto collidable = &collObj->body->hkBody->m_collidable;
					// Projectiles have 'Fixed' motion type by default, making them unmovable
					SetMotionTypeFunctor(vmRegistry, 0, grabbedObj, 3, true);
					// Projectiles also do not interact with collision usually. We need to change the filter to make them interact.
					collidable->m_broadPhaseHandle.m_collisionFilterInfo = 0x02420006; // player collision group, 'projectile' collision layer
					collidable->m_broadPhaseHandle.m_objectQualityType = 4; // Set to 'moving' quality instead of 'fixed'
				}
			}
		}

		// Determine the position where we want the object to be
		// Essentially it's a cylinder with a radius of the original distance when we grabbed it, and a height determined by some limit
		float w = VectorLength(NiPoint3(initialGrabbedObjRelativePosition.x, initialGrabbedObjRelativePosition.y, 0)); // horizontal distance from hand to object
		float h;
		NiPoint3 horiz;
		if (fabs(castDirection.z) <= 0.85f) {
			float theta = asinf(castDirection.z); // vertical angle of hand relative to horizontal
			h = w * tanf(theta); // desired height relative to horizontal from hand
			horiz = VectorNormalized(NiPoint3(castDirection.x, castDirection.y, 0)) * w; // desired horizontal position relative to hand
		}
		else {
			// Handle degenerate case and clamp
			h = w * tanf(asinf(copysignf(0.85f, castDirection.z))); // desired height relative to horizontal from hand
			float theta = asinf(castDirection.z); // vertical angle of hand relative to horizontal
			horiz = VectorNormalized(NiPoint3(castDirection.x, castDirection.y, 0)) * (h / tanf(theta)); // desired horizontal position relative to hand
		}

		// Basic hand motions

		NiPoint3 deltaHandPos = handPosRoomspace - prevHandPosRoomspace; // in room space

		// Get whatever transform takes the wand position from room space to skyrim worldspace
		NiMatrix33 localToWorldTransform = wandNode->m_worldTransform.rot * wandNode->m_localTransform.rot.Transpose();
		NiPoint3 deltaWorld = localToWorldTransform * deltaHandPos;

		if (VectorLength(deltaWorld) < 5.0f) { // Don't do anything if some weird jump happens
			float handSpeedInSpellDirection = DotProduct(deltaWorld, castDirection);
			if (handSpeedInSpellDirection < -1.0f) {
				pullDesired = true;
				_MESSAGE("Pull");
			}
			else if (handSpeedInSpellDirection > 1.0f) {
				pushDesired = true;
				_MESSAGE("Push");
			}
			else {
				pushDesired = false; // push should only be set in the frame that it happens (it's a 1-frame action)
			}
		}

		if (pullDesired && !isGrabbedObjActor && grabbedObj->baseForm->formType != kFormType_MovableStatic) {
			// If it's an in-flight projectile or dead body, no pull effect

			float inverseMass = motion->m_inertiaAndMassInv.w;

			float newMagnitude = (pow(inverseMass, g_massExponent) * g_pullVelocityMultiplier) / havokWorldScale;
			newMagnitude = min(newMagnitude, 12.0f); // Cap at some reasonable value
			NiPoint3 newVelocity = VectorNormalized(-relObjPos) * newMagnitude;

			ApplyHavokImpulse(vmRegistry, 0, grabbedObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
			motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };

			// If close enough to hand, pick it up
			if (VectorLength(relObjPos) < g_handActivateDistance * havokWorldScale) {
				_MESSAGE("Equipping");

				if (isShaderPlaying) {
					EffectShader_Stop(vmRegistry, 0, currentSelectedShader, selectedObj);
					isShaderPlaying = false;
				}

				// Pickup the item
				Activate(vmRegistry, 0, grabbedObj, player, false);
				// If the item is a weapon, equip it too
				auto baseForm = grabbedObj->baseForm;
				if (g_equipWeapons && baseForm && baseForm->formType == kFormType_Weapon) {
					auto *weapon = DYNAMIC_CAST(baseForm, TESForm, TESObjectWEAP);
					if (weapon) {
						papyrusActor::EquipItemEx(player, weapon, 1, false, false);
					}
				}

				grabbedObjHandle = *g_invalidRefHandle;
				selectedObjHandle = *g_invalidRefHandle;
				selectedColl = nullptr;
				pullDesired = false;
				pushDesired = false;
			}
		}
		else if (pushDesired) {
			NiPoint3 dir = VectorNormalized(relObjPos);

			float inverseMass = motion->m_inertiaAndMassInv.w;

			if (isGrabbedObjActor && grabbedObj->loadedState && grabbedObj->loadedState->node) {
				// For dead bodies, instead of the mass of the individual collidable we hit, use the mass of of the sort of root node
				Actor *actor = DYNAMIC_CAST(grabbedObj, TESObjectREFR, Actor);
				if (actor) {
					float actorInvereMass = GetActorInverseMass(actor);
					if (actorInvereMass >= 0) {
						inverseMass = actorInvereMass;
					}
					else {
						_WARNING("Could not get mass for actor");
					}
				}
			}

			float newMagnitude = (pow(inverseMass, g_massExponent) * g_pushVelocityMultiplier) / havokWorldScale;
			NiPoint3 newVelocity = VectorNormalized(relObjPos) * newMagnitude;

			ApplyHavokImpulse(vmRegistry, 0, grabbedObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
			motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };

			if (isShaderPlaying) {
				EffectShader_Stop(vmRegistry, 0, currentSelectedShader, grabbedObj);
				isShaderPlaying = false;
			}

			grabbedObjHandle = *g_invalidRefHandle;
			selectedObjHandle = *g_invalidRefHandle;
			selectedColl = nullptr;
		}
		else {
			// No push or pull - just hold it where we want it

			NiPoint3 desiredPos = NiPoint3(horiz.x, horiz.y, h);

			// Use distance from upper arm (shoulder-ish) to hand to control how far away from us we want the object to be
			float handHorizontalDistance = VectorLength(handPos - upperArmPos);
			float handDistanceRatio = (handHorizontalDistance - 10.0f) / (initialHandHorizontalDistance - 10.0f);
			desiredPos *= handDistanceRatio;

			NiPoint3 deltaPos = desiredPos - relObjPos;

			float inverseMass = motion->m_inertiaAndMassInv.w;

			if (isGrabbedObjActor && grabbedObj->loadedState && grabbedObj->loadedState->node) {
				// For dead bodies, instead of the mass of the individual collidable we hit, use the mass of the sort of root node
				Actor *actor = DYNAMIC_CAST(grabbedObj, TESObjectREFR, Actor);
				if (actor) {
					float actorInvereMass = GetActorInverseMass(actor);
					if (actorInvereMass >= 0) {
						inverseMass = actorInvereMass;
					}
					else {
						_WARNING("Could not get mass for actor");
					}
				}
			}

			float newMagnitude = VectorLength(deltaPos) * pow(inverseMass, g_massExponent) * g_hoverVelocityMultiplier;

			if (isGrabbedObjActor) {
				newMagnitude *= g_bodyVelocityMultiplier;
			}

			newMagnitude /= havokWorldScale;

			NiPoint3 newVelocity = VectorNormalized(deltaPos) * newMagnitude;

			ApplyHavokImpulse(vmRegistry, 0, grabbedObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
			motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };
		}
	}


	prevGrabbedObj = grabbedObj;
	prevHandPosRoomspace = handPosRoomspace;
}

void Grabber::SetupRollover(NiAVObject *rolloverNode, const TESObjectREFR *grabbedObj)
{
	// Give the hud with info about the object you're floating
	if (grabbedObj->baseForm && grabbedObj->baseForm->formType != kFormType_MovableStatic && !isGrabbedObjActor) { // We can't pick up movablestatics anyways
		// First, change rotation/position/scale of the hud prompt

		if (!g_hasSavedRollover) {
			g_hasSavedRollover = true;
			g_normalRolloverTransform = rolloverNode->m_localTransform;
		}

		rolloverNode->m_localTransform.pos = rolloverOffset;
		rolloverNode->m_localTransform.rot = rolloverRotation;
		rolloverNode->m_localTransform.scale = rolloverScale;

		// Now set all the places I could find that get set to the handle of the pointed at object usually
		if (SELECTED_HANDLES) {
			UInt32 *selectedHandles = *SELECTED_HANDLES;
			selectedHandles[2] = grabbedObjHandle;
			selectedHandles[5] = grabbedObjHandle;
			selectedHandles[8] = grabbedObjHandle;
		}
	}
}

bool WaitPosesCB(vr_src::TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, vr_src::TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount)
{
	if (!g_isLoaded) return true;

	if (MenuChecker::isGameStopped()) return true;

	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->loadedState || !player->loadedState->node)
		return true;

	g_rightGrabber.PoseUpdate(g_leftGrabber);
	g_leftGrabber.PoseUpdate(g_rightGrabber);

	NiPointer<TESObjectREFR> rightGrabbedObj, leftGrabbedObj;
	bool doesRightHaveGrab = LookupREFRByHandle(g_rightGrabber.grabbedObjHandle, rightGrabbedObj);
	bool doesLeftHaveGrab = LookupREFRByHandle(g_leftGrabber.grabbedObjHandle, leftGrabbedObj);

	static BSFixedString rightWandStr("RightWandNode");
	NiNode *rightWandNode = player->loadedState->node->m_parent->GetObjectByName(&rightWandStr.data)->GetAsNiNode();

	static BSFixedString leftWandStr("LeftWandNode");
	NiNode *leftWandNode = player->loadedState->node->m_parent->GetObjectByName(&leftWandStr.data)->GetAsNiNode();

	if (doesRightHaveGrab || doesLeftHaveGrab) {
		// Something is grabbed

		Setting	* activateRumbleIntensitySetting = GetINISetting("fActivateRumbleIntensity:VRInput");
		if (!g_hasSavedRumbleIntensity) {
			g_hasSavedRumbleIntensity = true;
			g_normalRumbleIntensity = activateRumbleIntensitySetting->data.f32;
		}
		activateRumbleIntensitySetting->SetDouble(0);

		if (doesRightHaveGrab && doesLeftHaveGrab) {
			// Toggle rollover menu between hands
			NiAVObject *rolloverNode = rightWandNode->GetObjectByName(&rolloverNodeStr.data);
			if (rolloverNode) {
				leftWandNode->AttachChild(rolloverNode, false);
				rightWandNode->RemoveChild(rolloverNode);
				g_leftGrabber.SetupRollover(rolloverNode, leftGrabbedObj);
			}
			else {
				rolloverNode = leftWandNode->GetObjectByName(&rolloverNodeStr.data);
				rightWandNode->AttachChild(rolloverNode, false);
				leftWandNode->RemoveChild(rolloverNode);
				g_rightGrabber.SetupRollover(rolloverNode, rightGrabbedObj);
			}
		}
		else if (doesRightHaveGrab) {
			NiAVObject *rolloverNode = rightWandNode->GetObjectByName(&rolloverNodeStr.data);
			if (!rolloverNode) {
				// Switch menu to right hand if it's on the left
				rolloverNode = leftWandNode->GetObjectByName(&rolloverNodeStr.data);
				rightWandNode->AttachChild(rolloverNode, false);
				leftWandNode->RemoveChild(rolloverNode);
			}
			g_rightGrabber.SetupRollover(rolloverNode, rightGrabbedObj);
		}
		else if (doesLeftHaveGrab) {
			NiAVObject *rolloverNode = leftWandNode->GetObjectByName(&rolloverNodeStr.data);
			if (!rolloverNode) {
				// Switch menu to left hand if it's on the right
				rolloverNode = rightWandNode->GetObjectByName(&rolloverNodeStr.data);
				leftWandNode->AttachChild(rolloverNode, false);
				rightWandNode->RemoveChild(rolloverNode);
			}
			g_leftGrabber.SetupRollover(rolloverNode, leftGrabbedObj);
		}
	}
	else {
		// Nothing is grabbed

		NiNode *mainWandNode = g_isLeftHanded ? leftWandNode : rightWandNode;
		NiNode *offhandWandNode = g_isLeftHanded ? rightWandNode : leftWandNode;
		NiAVObject *rolloverNode = mainWandNode->GetObjectByName(&rolloverNodeStr.data);
		if (!rolloverNode) {
			rolloverNode = offhandWandNode->GetObjectByName(&rolloverNodeStr.data);
			mainWandNode->AttachChild(rolloverNode, false);
			offhandWandNode->RemoveChild(rolloverNode);
		}
		if (g_hasSavedRollover) {
			rolloverNode->m_localTransform = g_normalRolloverTransform;
		}

		if (g_hasSavedRumbleIntensity) {
			Setting	* activateRumbleIntensitySetting = GetINISetting("fActivateRumbleIntensity:VRInput");
			activateRumbleIntensitySetting->data.f32 = g_normalRumbleIntensity;
		}
	}

	return true;
}


void Grabber::ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState)
{
	bool triggerDownBefore = triggerDown;

	// Check if the trigger is pressed
	if (pControllerState->ulButtonPressed & vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_SteamVR_Trigger)) {
		triggerDown = true;

		if (!triggerDownBefore) {
			triggerPressedTime = GetTime();
			triggerPressed = true;
			triggerReleased = false;
		}

		if (didTriggerPressGrabObject) {
			// If something is grabbed, disable the trigger
			pControllerState->ulButtonPressed &= ~vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_SteamVR_Trigger);
		}
		else {
			if (!triggerDownBefore) {
				// Trigger pressed but object is not grabbed (yet?). Do not unsheathe weapons until leeway time has passed.
				PlayerCharacter *pc = *g_thePlayer;
				if (pc && !pc->actorState.IsWeaponDrawn() && !MenuChecker::isGameStopped()) {
					unsheatheDesired = true;
				}
			}
		}
	}
	else {
		triggerDown = false;

		if (triggerDownBefore) {
			triggerReleased = true;
			triggerPressed = false;
		}
	}

	if (unsheatheDesired) {
		long long currentTime = GetTime();
		if (currentTime - triggerPressedTime <= g_triggerPressedLeewayTime) {
			// Suppress trigger press
			pControllerState->ulButtonPressed &= ~vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_SteamVR_Trigger);
		}
		else {
			unsheatheDesired = false;
			// Do the unsheathing, only if we didn't grab something
			if (!didTriggerPressGrabObject) {
				PlayerCharacter *pc = *g_thePlayer;
				if (pc && !pc->actorState.IsWeaponDrawn() && !MenuChecker::isGameStopped()) {
					pc->DrawSheatheWeapon(true);
				}
			}
		}
	}
}


void ControllerStateCB(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState, uint32_t unControllerStateSize, bool& state)
{
	vr_src::ETrackedControllerRole rightControllerRole = vr_src::ETrackedControllerRole::TrackedControllerRole_RightHand;
	vr_src::TrackedDeviceIndex_t rightController = (*g_openVR)->vrSystem->GetTrackedDeviceIndexForControllerRole(rightControllerRole);

	vr_src::ETrackedControllerRole leftControllerRole = vr_src::ETrackedControllerRole::TrackedControllerRole_LeftHand;
	vr_src::TrackedDeviceIndex_t leftController = (*g_openVR)->vrSystem->GetTrackedDeviceIndexForControllerRole(leftControllerRole);

	if (unControllerDeviceIndex == rightController) {
		g_rightGrabber.ControllerStateUpdate(unControllerDeviceIndex, pControllerState);
	}
	else if (unControllerDeviceIndex == leftController) {
		g_leftGrabber.ControllerStateUpdate(unControllerDeviceIndex, pControllerState);
	}
}

extern "C" {
	void OnDataLoaded()
	{
		g_rightGrabber.grabbedObjHandle = *g_invalidRefHandle;
		g_leftGrabber.grabbedObjHandle = *g_invalidRefHandle;

		const ModInfo *modInfo = DataHandler::GetSingleton()->LookupModByName("ForcePullVR.esp");
		if (!modInfo) {
			_MESSAGE("[CRITICAL] Could not get modinfo. Most likely the .esp is not loaded.");
			return;
		}

		TESForm *shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x3E90));
		if (!shaderForm) {
			_MESSAGE("Failed to get slected item shader form");
			return;
		}
		g_itemSelectedShader = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
		if (!g_itemSelectedShader) {
			_MESSAGE("Failed to cast selected item shader form");
			return;
		}

		shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x5EDA));
		if (!shaderForm) {
			_MESSAGE("Failed to get slected item shader form 2");
			return;
		}
		g_itemSelectedShader2 = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
		if (!g_itemSelectedShader2) {
			_MESSAGE("Failed to cast selected item shader form 2");
			return;
		}
		
		shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x4EB5));
		if (!shaderForm) {
			_MESSAGE("Failed to get slected item off limits shader form");
			return;
		}
		g_itemSelectedShaderOffLimits = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
		if (!g_itemSelectedShaderOffLimits) {
			_MESSAGE("Failed to cast selected item off limits shader form");
			return;
		}
		
		shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x5EDC));
		if (!shaderForm) {
			_MESSAGE("Failed to get slected item off limits shader form 2");
			return;
		}
		g_itemSelectedShaderOffLimits2 = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
		if (!g_itemSelectedShaderOffLimits2) {
			_MESSAGE("Failed to cast selected item off limits shader form 2");
			return;
		}
		
		MenuManager * menuManager = MenuManager::GetSingleton();
		if (menuManager) {
			menuManager->MenuOpenCloseEventDispatcher()->AddEventSink(&MenuChecker::menuEvent);
		}

		g_rightGrabber.itemSelectedShader = g_itemSelectedShader;
		g_rightGrabber.itemSelectedShaderOffLimits = g_itemSelectedShaderOffLimits;

		g_leftGrabber.itemSelectedShader = g_itemSelectedShader2;
		g_leftGrabber.itemSelectedShaderOffLimits = g_itemSelectedShaderOffLimits2;

		_MESSAGE("Successfully loaded all forms");
	}

	void OnInputLoaded()
	{

	}

	// Listener for SKSE Messages
	void OnSKSEMessage(SKSEMessagingInterface::Message* msg)
	{
		if (msg) {
			if (msg->type == SKSEMessagingInterface::kMessage_InputLoaded) {
				OnInputLoaded();
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_DataLoaded) {
				OnDataLoaded();
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_PreLoadGame) {
				_MESSAGE("SKSE PreLoadGame message received");
				g_isLoaded = false;
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_PostLoadGame || msg->type == SKSEMessagingInterface::kMessage_NewGame) {
				_MESSAGE("SKSE PostLoadGame or NewGame message received, type: %d", msg->type);
				g_isLoaded = true;
				Setting	* isLeftHandedSetting = GetINISetting("bLeftHandedMode:VRInput");
				g_isLeftHanded = (bool)isLeftHandedSetting->data.u8;
				vmRegistry = (*g_skyrimVM)->GetClassRegistry();
			}
		}
	}

	bool SKSEPlugin_Query(const SKSEInterface* skse, PluginInfo* info)
	{
		gLog.OpenRelative(CSIDL_MYDOCUMENTS, "\\My Games\\Skyrim VR\\SKSE\\ForcePullVR.log");
		gLog.SetPrintLevel(IDebugLog::kLevel_DebugMessage);
		gLog.SetLogLevel(IDebugLog::kLevel_DebugMessage);

		_MESSAGE("ForcePullVR v%s", FPVR_VERSION_VERSTRING);

		info->infoVersion = PluginInfo::kInfoVersion;
		info->name = "ForcePullVR";
		info->version = FPVR_VERSION_MAJOR;

		g_pluginHandle = skse->GetPluginHandle();

		if (skse->isEditor) {
			_FATALERROR("[FATAL ERROR] Loaded in editor, marking as incompatible!\n");
			return false;
		}
		else if (skse->runtimeVersion != RUNTIME_VR_VERSION_1_4_15) {
			_FATALERROR("[FATAL ERROR] Unsupported runtime version %08X!\n", skse->runtimeVersion);
			return false;
		}

		return true;
	}

	bool ReadConfigOptions()
	{
		float handAdjustX, handAdjustY, handAdjustZ;
		if (!Config::GetConfigOptionFloat("Settings", "HandAdjustX", &handAdjustX)) return false;
		if (!Config::GetConfigOptionFloat("Settings", "HandAdjustY", &handAdjustY)) return false;
		if (!Config::GetConfigOptionFloat("Settings", "HandAdjustZ", &handAdjustZ)) return false;

		NiPoint3 handAdjust = { handAdjustX, handAdjustY, handAdjustZ };
		if (VectorLength(handAdjust) > 0.0001f) {
			g_handAdjust = VectorNormalized(handAdjust);
		}
		else {
			_WARNING("Supplied hand adjust vector is too small - using default.");
		}

		if (!Config::GetConfigOptionFloat("Settings", "CastRadius", &g_castRadius)) return false;
		if (!Config::GetConfigOptionFloat("Settings", "CastDistance", &g_castDistance)) return false;
		if (!Config::GetConfigOptionFloat("Settings", "HandActivateDistance", &g_handActivateDistance)) return false;

		float castDirectionRequiredHalfAngle;
		if (!Config::GetConfigOptionFloat("Settings", "CastDirectionRequiredHalfAngle", &castDirectionRequiredHalfAngle)) return false;
		g_requiredCastDotProduct = cosf(castDirectionRequiredHalfAngle * 0.0174533); // degrees to radians

		int selectedFadeTime;
		if (!Config::GetConfigOptionInt("Settings", "SelectedFadeTime", &selectedFadeTime)) return false;
		g_selectedLeewayTime = selectedFadeTime;

		int triggerPreemptTime;
		if (!Config::GetConfigOptionInt("Settings", "TriggerPreemptTime", &triggerPreemptTime)) return false;
		g_triggerPressedLeewayTime = triggerPreemptTime;

		if (!Config::GetConfigOptionBool("Settings", "EquipWeapons", &g_equipWeapons)) return false;

		if (!Config::GetConfigOptionFloat("Settings", "HoverVelocityMultiplier", &g_hoverVelocityMultiplier)) return false;
		if (!Config::GetConfigOptionFloat("Settings", "PullVelocityMultiplier", &g_pullVelocityMultiplier)) return false;
		if (!Config::GetConfigOptionFloat("Settings", "PushVelocityMultiplier", &g_pushVelocityMultiplier)) return false;
		if (!Config::GetConfigOptionFloat("Settings", "BodyVelocityMultiplier", &g_bodyVelocityMultiplier)) return false;
		if (!Config::GetConfigOptionFloat("Settings", "MassExponent", &g_massExponent)) return false;

		return true;
	}

	bool SKSEPlugin_Load(const SKSEInterface * skse)
	{	// Called by SKSE to load this plugin
		_MESSAGE("ForcePullVR loaded");

		// Registers for SKSE Messages (PapyrusVR probably still need to load, wait for SKSE message PostLoad)
		_MESSAGE("Registering for SKSE messages");
		g_messaging = (SKSEMessagingInterface*)skse->QueryInterface(kInterface_Messaging);
		g_messaging->RegisterListener(g_pluginHandle, "SKSE", OnSKSEMessage);

		if (ReadConfigOptions()) {
			_MESSAGE("Successfully read config parameters");
		}
		else {
			_WARNING("[WARNING] Failed to read config options. Using defaults instead.");
		}

		g_vrInterface = (SKSEVRInterface *)skse->QueryInterface(kInterface_VR);
		if (!g_vrInterface) {
			_ERROR("[CRITICAL] Couldn't get SKSE VR interface. You probably have an outdated SKSE version.");
			return false;
		}
		g_vrInterface->RegisterForControllerState(g_pluginHandle, 0, ControllerStateCB);
		g_vrInterface->RegisterForPoses(g_pluginHandle, 0, WaitPosesCB);

		for (int i = 0; i < numPrevPos; i++) {
			g_rightGrabber.handPositions[i] = {0, 0, 0};
			g_leftGrabber.handPositions[i] = { 0, 0, 0 };
		}

		// Right vector points to the right of the text
		g_rolloverRotation.data[0][0] = 0;
		g_rolloverRotation.data[1][0] = -cosf(30 * 0.0174533);
		g_rolloverRotation.data[2][0] = -sinf(30 * 0.0174533);
		// Forward vector points into the text
		g_rolloverRotation.data[0][1] = -1;
		g_rolloverRotation.data[1][1] = 0;
		g_rolloverRotation.data[2][1] = 0;
		// Up vector points up from the text
		g_rolloverRotation.data[0][2] = 0;
		g_rolloverRotation.data[1][2] = sinf(30 * 0.0174533);
		g_rolloverRotation.data[2][2] = -cosf(30 * 0.0174533);

		g_rightGrabber.rolloverOffset = g_rolloverOffset;
		g_rightGrabber.rolloverRotation = g_rolloverRotation;
		g_rightGrabber.rolloverScale = g_rolloverScale;

		// Flip right vector offset - TODO: proper offset
		g_leftGrabber.rolloverOffset = { -g_rolloverOffset.x, g_rolloverOffset.y, g_rolloverOffset.z };
		g_leftGrabber.rolloverRotation = g_rolloverRotation;
		// Flip right/forward vectors
		g_leftGrabber.rolloverRotation.data[0][0] = -g_leftGrabber.rolloverRotation.data[0][0];
		g_leftGrabber.rolloverRotation.data[1][0] = -g_leftGrabber.rolloverRotation.data[1][0];
		g_leftGrabber.rolloverRotation.data[2][0] = -g_leftGrabber.rolloverRotation.data[2][0];
		g_leftGrabber.rolloverRotation.data[0][1] = -g_leftGrabber.rolloverRotation.data[0][1];
		g_leftGrabber.rolloverRotation.data[1][1] = -g_leftGrabber.rolloverRotation.data[1][1];
		g_leftGrabber.rolloverRotation.data[2][1] = -g_leftGrabber.rolloverRotation.data[2][1];
		g_leftGrabber.rolloverScale = g_rolloverScale;

		return true;
	}
};
