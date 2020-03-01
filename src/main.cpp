#include <functional>
#include <string>
#include <regex>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <limits>
#include "common/IDebugLog.h"  // IDebugLog
#include "skse64_common/skse_version.h"  // RUNTIME_VERSION
#include "skse64/PluginAPI.h"  // SKSEInterface, PluginInfo
#include "skse64/GameRTTI.h"
#include "skse64/GameSettings.h"
#include "skse64/NiNodes.h"
#include "skse64/NiExtraData.h"
#include "skse64/GameData.h"
#include "skse64/GameForms.h"
#include "skse64/PapyrusActor.h"

#include <ShlObj.h>  // CSIDL_MYDOCUMENTS

#include "main.h"
#include "version.h"
#include "physics.h"
#include "utils.h"

// Headers under api/ folder
#include "api/PapyrusVRAPI.h"
#include "api/VRManagerAPI.h"
#include "api/utils/OpenVRUtils.h"


// Multiply skyrim coords by this to get havok coords
// It's the number of meters per skyrim unit
RelocAddr<float *> HAVOK_WORLD_SCALE_ADDR(0x15B78F4);

// Address of pointer that points to the bhkWorld pointer
RelocAddr<bhkWorld ***> BHKWORLD(0x1f850d0);

// Alternatively, 0x30008E0 + 0x78
// Even better, (*0x2FC60C0) + 0x78
// Address of pointer to bhkSimpleShapePhantom that tracks the right hand - maybe not actually the right hand?
RelocAddr<bhkSimpleShapePhantom **> SPHERE_SHAPE_ADDR(0x3000958);


// SKSE / SkyrimVRTools globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

PapyrusVRAPI *g_papyrusvr = nullptr;
PapyrusVR::VRManagerAPI *g_papyrusvrManager = nullptr;
OpenVRHookManagerAPI *g_openvrHook = nullptr;

VMClassRegistry *vmRegistry = nullptr;

// Gets callbacks from havok linear cast
CdPointCollector cdPointCollector;
hkpLinearCastInput linearCastInput;
RayHitCollector rayHitCollector;
hkpWorldRayCastInput rayCastInput(0x02420028); // 'ItemPicker' collision layer; player collision group

TESObjectREFR *pullObj = nullptr;
TESObjectREFR *prevPullObj = nullptr;
TESObjectREFR *selectedObj = nullptr;
hkpCollidable *selectedColl = nullptr;
hkpCollidable *pullColl = nullptr;
bool isPullObjInFlightProjectile = false;
bool isPullObjImpactedProjectile = false;
float inFlightProjectileOriginalSpeed = 0;

NiPoint3 initialPullObjRelativePosition(0, 0, 0);
NiPoint3 prevHandPosLocal(0, 0, 0); // Relative to hmd

bool pullDesired = false;
bool pushDesired = false;

long long lastDebugCastTime = 0;
long long lastSelectedTime = 0;

long long g_selectedLeewayTime = 250; // in ms, time to keep something selected after not pointing at it anymore

bool isLoaded = false;
bool isLastUpdateValid = false;
bool g_triggerPressed = false;
bool g_triggerReleased = false;

//UInt32 spellFormId = 0x1C789; // fireball
//UInt32 spellFormId = 0x2B96C; // ice spike
//UInt32 spellFormId = 0x2dd29; // lightning bolt
//UInt32 spellFormId = 0x00012fcd; // flames
//UInt32 spellFormId = 0x0001A4CC; // telekinesis
SpellItem *g_debugSpell = nullptr;
TESObjectREFR *debugSourceActivator = nullptr;
TESObjectREFR *debugTargetActivator = nullptr;

bool g_debug = false; // Set to true to fire a spell (visual only) in the direction that the cast happens

TESEffectShader *g_itemSelectedShader = nullptr;


// Called on each update (about 90-100 calls per second)
void OnPoseUpdateUntimed(float deltaTime)
{
	if (!isLoaded) return;

	if (IsInMenuMode(vmRegistry, 0)) return;

	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->loadedState || !player->loadedState->node)
		return;

	TESObjectCELL* cell = player->parentCell;
	if (!cell)
		return;

	bool wasLastUpdateValid = isLastUpdateValid;
	isLastUpdateValid = false;


	long long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();


	static BSFixedString rHandStr("NPC R Hand [RHnd]");
	NiAVObject *rightHand = player->GetNiRootNode(1)->GetObjectByName(&rHandStr.data);
	if (!rightHand) {
		_MESSAGE("No right hand");
		return;
	}
	NiPoint3 handPos = rightHand->m_worldTransform.pos;

	NiPoint3 handAdjust = { -0.2, -1, 0.4 }; // A bit down (negative up), a lot left (negative right) and a bit forward
	handAdjust = VectorNormalized(handAdjust);
	NiPoint3 castDirection = rightHand->m_worldTransform.rot * handAdjust;

	static BSFixedString hmdNodeStr("HmdNode");
	NiAVObject *hmdNode = GetHighestParent(player->loadedState->node)->GetObjectByName(&hmdNodeStr.data);

	NiTransform inversePlayerTransform;
	hmdNode->m_worldTransform.Invert(inversePlayerTransform);

	NiPoint3 handPosLocal = inversePlayerTransform * handPos;

	static BSFixedString rClavicleStr("NPC R Clavicle [RClv]");
	NiAVObject *rightClavicle = player->GetNiRootNode(0)->GetObjectByName(&rClavicleStr.data);
	// TODO: Use delta between consecutive clavicle->hand vectors for hand motions?

	
	// Fire a spell at the direction that the cast happens, so that you can visually see and adjust
	if (g_debug) {
		UInt32 nullHandle = *g_invalidRefHandle;
		NiPoint3 rot(0, 0, 0);
		MoveRefrToPosition(debugSourceActivator, &nullHandle, player->parentCell, CALL_MEMBER_FN(player, GetWorldspace)(), &handPos, &rot);

		NiPoint3 targetPos = handPos + castDirection;
		MoveRefrToPosition(debugTargetActivator, &nullHandle, player->parentCell, CALL_MEMBER_FN(player, GetWorldspace)(), &targetPos, &rot);
		if (currentTime - lastDebugCastTime > 500) { // in ms
			lastDebugCastTime = currentTime;
			Cast(vmRegistry, 0, g_debugSpell, debugSourceActivator, debugTargetActivator);
		}
	}

	if (!pullObj) {
		// Convert hand position from skyrim coords to havok coords
		float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;
		NiPoint3 hkHandPos = handPos * havokWorldScale;
		NiPoint3 hkTargetPos = hkHandPos + castDirection * 5;

		NiPoint3 hitPosition = { hkTargetPos.x, hkTargetPos.y, hkTargetPos.z };

		bhkWorld *world = **BHKWORLD;

		// First, raycast in the pointing direction
		// TODO: Make raycast ignore plants and shit?
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
		sphereShape->m_radius = 0.4f;
		linearCastInput.m_to = { hitPosition.x, hitPosition.y, hitPosition.z, 0 };
		hkpWorld_LinearCast(world->world, &sphere->phantom->m_collidable, &linearCastInput, &cdPointCollector, nullptr);
		sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = filterInfoBefore;
		sphereShape->m_radius = radiusBefore;

		// Process result of cast
		bool isSelected = false;
		TESObjectREFR *closestObj = nullptr;
		hkpCollidable *closestColl = nullptr;
		float closestDistance = (std::numeric_limits<float>::max)();

		for (auto pair : cdPointCollector.m_hits) {
			auto collidable = static_cast<hkpCollidable *>(pair.second);
			auto ref = FindCollidableRef(collidable);
			if (ref) {
				TESForm *baseForm = ref->baseForm;
				if (baseForm && IsSelectable(baseForm)) {					
					// Get distance from the hit on the collidable to the ray
					NiPoint3 handToHit = NiPoint3(pair.first.x, pair.first.y, pair.first.z) - hkHandPos;
					NiPoint3 handToHitAlongRay = castDirection * DotProduct(handToHit, castDirection); // project above vector onto ray
					float dist = VectorLength(handToHit - handToHitAlongRay); // distance from hit location to closest point on the ray
					if (dist < closestDistance) {
						closestObj = ref;
						closestColl = collidable;
						closestDistance = dist;
					}
				}
			}
		}

		if (closestObj) {
			if (closestObj != selectedObj) {
				if (selectedObj) {
					EffectShader_Stop(vmRegistry, 0, g_itemSelectedShader, selectedObj);
				}
				selectedObj = closestObj;
				selectedColl = closestColl;
				EffectShader_Play(vmRegistry, 0, g_itemSelectedShader, selectedObj, -1.0f);
			}
			isSelected = true;
			lastSelectedTime = currentTime;
		}

		if (!isSelected && selectedObj && currentTime - lastSelectedTime > g_selectedLeewayTime) {
			EffectShader_Stop(vmRegistry, 0, g_itemSelectedShader, selectedObj);
			selectedObj = nullptr;
		}

		if (g_triggerPressed) {
			// Pick up the item
			pullObj = selectedObj;
			pullColl = selectedColl;

			if (pullObj) {
				// Set to false only here, so that you can hold the trigger until the cast hits something valid
				g_triggerPressed = false;

				isPullObjInFlightProjectile = false;
				isPullObjImpactedProjectile = false;
				auto baseForm = pullObj->baseForm;
				if (baseForm && baseForm->formType == kFormType_Projectile) {
					auto velocity = (NiPoint3 *)((UInt64)pullObj + 0xfc);
					auto impactData = *(void **)((UInt64)pullObj + 0x98);
					if (impactData) {
						// If the projectile has impact data, then it has well, impacted something
						isPullObjImpactedProjectile = true;
					}
					else {
						isPullObjInFlightProjectile = true;
						inFlightProjectileOriginalSpeed = VectorLength(*velocity);
					}
				}
			}
		}
	}

	if (g_triggerReleased) {
		g_triggerReleased = false;

		if (selectedObj) {
			EffectShader_Stop(vmRegistry, 0, g_itemSelectedShader, selectedObj);
			selectedObj = nullptr;
		}
		if (pullObj) {
			if (isPullObjInFlightProjectile) {
				// In-flight projectile
				if (pullObj->loadedState) {
					auto velocity = (NiPoint3 *)((UInt64)pullObj + 0xfc);

					// Derotate by whatever we need to to make it actually face us...
					auto rot = pullObj->loadedState->node->m_localTransform.rot;
					rot = MatrixFromAxisAngle({ rot.data[0][0], rot.data[1][0], rot.data[2][0] }, -90 * 0.0174533) * rot;
					NiPoint3 forward = { rot.data[0][2], rot.data[1][2], rot.data[2][2] };

					*velocity = forward * inFlightProjectileOriginalSpeed;
				}
			}
			// Drop the item
			pullObj = nullptr;
			pullDesired = false;
		}
	}

	if (pullObj && !(pullObj->flags & TESForm::kFlagIsDeleted)) {
		bool cancel = false;
		// loadedState can be an invalid pointer for projectiles soon after fired
		// Need to check waitingToInitialize3D(0x1DC) on missileprojectiles before accessing them
		if (isPullObjInFlightProjectile) {
			bool waitingToInitialize3D = *(bool *)((UInt64)pullObj + 0x1dc);
			if (waitingToInitialize3D) {
				cancel = true;
			}
		}

		if (!cancel && pullObj->loadedState && pullObj->loadedState->node) {
			float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;

			hkpMotion *motion = nullptr;
			if (pullObj->loadedState->node->unk040) {
				auto collObj = (bhkCollisionObject *)pullObj->loadedState->node->unk040;
				motion = &collObj->body->hkBody->motion;
			}
			else {
				motion = reinterpret_cast<hkpMotion *>((UInt64)pullColl->m_motion - offsetof(hkpMotion, m_motionState));
			}
			auto translation = motion->m_motionState.m_transform.m_translation;

			NiPoint3 hkObjPos = { translation.x, translation.y, translation.z };
			NiPoint3 hkHandPos = handPos * havokWorldScale;

			auto relObjPos = hkObjPos - hkHandPos;

			if (!prevPullObj) {
				pushDesired = false;
				pullDesired = false;

				initialPullObjRelativePosition = relObjPos;

				if (isPullObjImpactedProjectile) { // It's an embedded projectile, i.e. stuck in a wall etc.
					auto collObj = (bhkCollisionObject *)pullObj->loadedState->node->unk040;
					if (collObj) {
						// Do not use pullColl here, it's not the right collidable to set collision for
						auto collidable = &collObj->body->hkBody->m_collidable;
						// Projectiles have 'Fixed' motion type by default, making them unmovable
						SetMotionTypeFunctor(vmRegistry, 0, pullObj, 3, true);
						// Projectiles also do not interact with collision usually. We need to change the filter to make them interact.
						collidable->m_broadPhaseHandle.m_collisionFilterInfo = 0x02420006; // player collision group, 'projectile' collision layer
						collidable->m_broadPhaseHandle.m_objectQualityType = 4; // Set to 'moving' quality instead of 'fixed'
					}
				}
				else if (isPullObjInFlightProjectile) {
					// If the player grabs a projectile in flight, make them the shooter and actor cause
					UInt32 playerHandle = GetOrCreateRefrHandle(player);
					auto shooter = (UInt32 *)((UInt64)pullObj + 0x120);
					*shooter = playerHandle;

					auto actorCause = *(UInt32 **)((UInt64)pullObj + 0x118);
					UInt32 handle = *actorCause;
					*actorCause = playerHandle;

					// Need to set collision to same as if the player cast it, so that it collides with the _original_ caster (probably an enemy)
					auto phantom = *(bhkSimpleShapePhantom**)((UInt64)pullObj + 0xE0);
					phantom->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= 0x0000FFFF; // clear out the collision group
					phantom->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= 0x02420000; // set the group to player group
				}
			}

			// Determine the position where we want the object to be
			float theta = asinf(castDirection.z); // vertical angle of hand relative to horizontal
			float w = VectorLength(NiPoint3(initialPullObjRelativePosition.x, initialPullObjRelativePosition.y, 0)); // horizontal distance from hand to object
			float h = w * tanf(theta); // desired height relative to horizontal from hand
			//float delta_h = h - relObjPos.z;

			NiPoint3 horiz = VectorNormalized(NiPoint3(castDirection.x, castDirection.y, 0)) * w; // desired horizontal position relative to hand

			NiPoint3 deltaPos = NiPoint3(horiz.x, horiz.y, h) - relObjPos;

			// Basic hand motions
			NiTransform inversePlayerTransform;
			hmdNode->m_worldTransform.Invert(inversePlayerTransform);

			NiPoint3 handPosLocal = inversePlayerTransform * handPos;

			NiPoint3 deltaHandPos = handPosLocal - prevHandPosLocal; // in hmd space

			NiPoint3 spellDirectionLocal = hmdNode->m_worldTransform.rot.Transpose() * castDirection; // transpose == inverse, since rotation matrix is orthogonal

			if (VectorLength(deltaHandPos) < 5.0f) { // Don't do anything if some weird jump happens
				float handSpeedInSpellDirection = DotProduct(deltaHandPos, spellDirectionLocal);
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

			if (pullDesired && !isPullObjInFlightProjectile) {
				// If it's an in-flight projectile, no pull effect
				float newMagnitude = (VectorLength(relObjPos) * 0.1f) / havokWorldScale;
				newMagnitude = min(newMagnitude, 12.0f); // Cap at some reasonable value
				NiPoint3 newVelocity = VectorNormalized(-relObjPos) * newMagnitude;

				ApplyHavokImpulse(vmRegistry, 0, pullObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
				motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };

				// If close enough to hand, pick it up
				if (VectorLength(relObjPos) < 30.0f * havokWorldScale) {
					_MESSAGE("Equipping");
					// Pickup the item
					Activate(vmRegistry, 0, pullObj, player, false);
					// If the item is a weapon, equip it too
					auto baseForm = pullObj->baseForm;
					if (baseForm && baseForm->formType == kFormType_Weapon) {
						auto *weapon = DYNAMIC_CAST(baseForm, TESForm, TESObjectWEAP);
						if (weapon) {
							papyrusActor::EquipItemEx(player, weapon, 1, false, false);
						}
					}
					pullObj = nullptr;
					selectedObj = nullptr;
					pullDesired = false;
					pushDesired = false;
				}
			}
			else if (pushDesired) {
				NiPoint3 dir = VectorNormalized(relObjPos);

				if (isPullObjInFlightProjectile) {
					auto velocity = (NiPoint3 *)((UInt64)pullObj + 0xfc);
					*velocity = dir * inFlightProjectileOriginalSpeed;

					NiPoint3 forward = dir;

					NiPoint3 worldUpAlongForward = forward * DotProduct({ 0, 0, 1 }, forward); // Project world up vector onto our forward vector
					NiPoint3 up = VectorNormalized(NiPoint3(0, 0, 1) - worldUpAlongForward);
					NiPoint3 right = CrossProduct(forward, up);

					pullObj->loadedState->node->m_localTransform.rot.data[0][0] = up.x;
					pullObj->loadedState->node->m_localTransform.rot.data[1][0] = up.y;
					pullObj->loadedState->node->m_localTransform.rot.data[2][0] = up.z;

					pullObj->loadedState->node->m_localTransform.rot.data[0][1] = right.x;
					pullObj->loadedState->node->m_localTransform.rot.data[1][1] = right.y;
					pullObj->loadedState->node->m_localTransform.rot.data[2][1] = right.z;

					pullObj->loadedState->node->m_localTransform.rot.data[0][2] = forward.x;
					pullObj->loadedState->node->m_localTransform.rot.data[1][2] = forward.y;
					pullObj->loadedState->node->m_localTransform.rot.data[2][2] = forward.z;

					// Rotate by whatever we need to to make it actually face us...
					pullObj->loadedState->node->m_localTransform.rot = MatrixFromAxisAngle(up, 90 * 0.0174533) * pullObj->loadedState->node->m_localTransform.rot;
				}
				else {
					float magnitude = 80.0f;
					ApplyHavokImpulse(vmRegistry, 0, pullObj, dir.x, dir.y, dir.z, magnitude);
					EffectShader_Stop(vmRegistry, 0, g_itemSelectedShader, pullObj);
				}

				pullObj = nullptr;
				selectedObj = nullptr;
			}
			else {
				// No push or pull - just hold it where we want it
				if (isPullObjInFlightProjectile) {
					// In-flight projectile
					float newMagnitude = (VectorLength(deltaPos) * 5.0f) / havokWorldScale;
					newMagnitude = min(newMagnitude, 10.0f / havokWorldScale); // Cap at some reasonable value
					NiPoint3 newVelocity = VectorNormalized(deltaPos) * newMagnitude;

					auto velocity = (NiPoint3 *)((UInt64)pullObj + 0xfc);
					*velocity = newVelocity;
				}
				else {
					// Everything else
					float newMagnitude = (VectorLength(deltaPos) * 0.05f) / havokWorldScale;
					newMagnitude = min(newMagnitude, 10.0f); // Cap at some reasonable value
					NiPoint3 newVelocity = VectorNormalized(deltaPos) * newMagnitude;

					ApplyHavokImpulse(vmRegistry, 0, pullObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
					motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };
				}
			}
		}
	}


	prevPullObj = pullObj;
	prevHandPosLocal = handPosLocal;

	isLastUpdateValid = true;
}

void OnPoseUpdate(float Deltatime)
{
	//long long currentTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	OnPoseUpdateUntimed(Deltatime);
	//long long timeElapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - currentTime;
	//_MESSAGE("%d", timeElapsed);
}

void OnButtonEvent(PapyrusVR::VREventType eventType, PapyrusVR::EVRButtonId buttonId, PapyrusVR::VRDevice device)
{
	// This function runs before OnPoseUpdate, so there are no race conditions	

	if (buttonId == PapyrusVR::EVRButtonId::k_EButton_SteamVR_Trigger && device == PapyrusVR::VRDevice::VRDevice_RightController) {		
		if (eventType == PapyrusVR::VREventType::VREventType_Pressed) {
			g_triggerPressed = true;
			g_triggerReleased = false;
		}
		else if (eventType == PapyrusVR::VREventType::VREventType_Released) {
			g_triggerReleased = true;
			g_triggerPressed = false;
		}
	}
}

extern "C" {	
	void OnDataLoaded()
	{
		const ModInfo *modInfo = DataHandler::GetSingleton()->LookupModByName("ForcePullVR.esp");
		if (!modInfo) {
			_MESSAGE("[CRITICAL] Could not get modinfo. Most likely the .esp is not loaded.");
			return;
		}

		TESForm *shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x3E90));
		if (shaderForm) {
			g_itemSelectedShader = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
			if (!g_itemSelectedShader) {
				_MESSAGE("Failed to cast selected item shader form");
				return;
			}
		}
		else {
			_MESSAGE("Failed to get slected item shader form");
			return;
		}

		UInt32 spellFormId = GetFullFormID(modInfo, 0xd65);
		TESForm *spellForm = LookupFormByID(spellFormId);
		if (spellForm) {
			g_debugSpell = DYNAMIC_CAST(spellForm, TESForm, SpellItem);
			if (!g_debugSpell) {
				_MESSAGE("Failed to cast spell form to spellitem");
				return;
			}
		}
		else {
			_MESSAGE("Failed to get spell form");
			return;
		}

		UInt32 sourceActivatorFormId = GetFullFormID(modInfo, 0x2906);
		UInt32 targetActivatorFormId = GetFullFormID(modInfo, 0x2907);
		TESForm *sourceActivatorForm = LookupFormByID(sourceActivatorFormId);
		TESForm *targetActivatorForm = LookupFormByID(targetActivatorFormId);
		if (sourceActivatorForm && targetActivatorForm) {
			debugSourceActivator = DYNAMIC_CAST(sourceActivatorForm, TESForm, TESObjectREFR);
			debugTargetActivator = DYNAMIC_CAST(targetActivatorForm, TESForm, TESObjectREFR);
			if (!debugSourceActivator || !debugTargetActivator) {
				_MESSAGE("Failed to cast source or target activators");
				return;
			}
		}
		else {
			_MESSAGE("Failed to get source or target activator forms");
			return;
		}
		_MESSAGE("Successfully loaded all forms");
	}

	void OnInputLoaded()
	{

	}

	// Listener for PapyrusVR Messages
	void OnPapyrusVRMessage(SKSEMessagingInterface::Message* msg)
	{
		if (msg) {
			if (msg->type == kPapyrusVR_Message_Init && msg->data) {
				_MESSAGE("PapyrusVR Init Message recived with valid data, registering for pose update callback");
				g_papyrusvr = (PapyrusVRAPI*)msg->data;
				g_papyrusvrManager = g_papyrusvr->GetVRManager();
				g_openvrHook = g_papyrusvr->GetOpenVRHook();

				if (!g_papyrusvrManager) {
					_MESSAGE("Could not get PapyrusVRManager");
					return;
				}
				if (!g_openvrHook) {
					_MESSAGE("Could not get OpenVRHook");
					return;
				}
				// Registers for PoseUpdates
				g_papyrusvrManager->RegisterVRUpdateListener(OnPoseUpdate);
				g_papyrusvrManager->RegisterVRButtonListener(OnButtonEvent);
			}
		}
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
			else if (msg->type == SKSEMessagingInterface::kMessage_PostLoad) {
				_MESSAGE("SKSE PostLoad recived, registering for SkyrimVRTools messages");
				g_messaging->RegisterListener(g_pluginHandle, "SkyrimVRTools", OnPapyrusVRMessage);
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_PreLoadGame) {
				_MESSAGE("SKSE PreLoadGame message received");
				isLoaded = false;
			}
			else if (msg->type == SKSEMessagingInterface::kMessage_PostLoadGame || msg->type == SKSEMessagingInterface::kMessage_NewGame) {
				_MESSAGE("SKSE PostLoadGame or NewGame message received, type: %d", msg->type);
				isLoaded = true;
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

		// wait for PapyrusVR init (during PostPostLoad SKSE Message)

		return true;
	}
};