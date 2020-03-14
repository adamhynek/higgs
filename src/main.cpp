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
#include "skse64_common/SafeWrite.h"
#include "skse64_common/BranchTrampoline.h"
#include "xbyak/xbyak.h"

#include <ShlObj.h>  // CSIDL_MYDOCUMENTS

#include "main.h"
#include "version.h"
#include "physics.h"
#include "utils.h"
#include "config.h"

// Headers under api/ folder
#include "api/PapyrusVRAPI.h"
#include "api/VRManagerAPI.h"
#include "api/utils/OpenVRUtils.h"


// Multiply skyrim coords by this to get havok coords
// It's the number of meters per skyrim unit
RelocAddr<float *> HAVOK_WORLD_SCALE_ADDR(0x15B78F4);

// Alternatively, 0x30008E0 + 0x78
// Even better, (*0x2FC60C0) + 0x78
// Address of pointer to bhkSimpleShapePhantom that tracks the right hand - more or less
RelocAddr<bhkSimpleShapePhantom **> SPHERE_SHAPE_ADDR(0x3000958);


// SKSE / SkyrimVRTools globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

PapyrusVRAPI *g_papyrusvr = nullptr;
PapyrusVR::VRManagerAPI *g_papyrusvrManager = nullptr;
OpenVRHookManagerAPI *g_openvrHook = nullptr;

VMClassRegistry *vmRegistry = nullptr;

SKSETrampolineInterface *g_trampoline = nullptr;

// Gets callbacks from havok linear cast
CdPointCollector cdPointCollector;
hkpLinearCastInput linearCastInput;
RayHitCollector rayHitCollector;
hkpWorldRayCastInput rayCastInput(0x02420028); // 'ItemPicker' collision layer; player collision group

// Config params
float g_castDistance = 5.0f;
float g_castRadius = 0.3f;
float g_handActivateDistance = 30.0f;
float g_requiredCastDotProduct = cosf(50.0f * 0.0174533);
long long g_selectedLeewayTime = 250; // in ms, time to keep something selected after not pointing at it anymore
bool g_equipWeapons = false;

NiPoint3 g_handAdjust = { -0.2, -1, 0.4 };

//TESObjectREFR *pullObj = nullptr;
UInt32 pullObjHandle = 0;
//TESObjectREFR *selectedObj = nullptr;
UInt32 selectedObjHandle = 0;
TESObjectREFR *prevPullObj = nullptr;
hkpCollidable *selectedColl = nullptr;
hkpCollidable *pullColl = nullptr;
bool isPullObjActor = false;
bool isPullObjInFlightProjectile = false;
bool isPullObjImpactedProjectile = false;
float inFlightProjectileOriginalSpeed = 0;
NiMatrix33 inFlightOriginalRotation;

NiPoint3 initialPullObjRelativePosition(0, 0, 0);
NiPoint3 prevHandPosLocal(0, 0, 0); // Relative to hmd

bool pullDesired = false;
bool pushDesired = false;

long long lastSelectedTime = 0;
long long triggerPressedTime = 0; // The timestamp when the trigger was pressed

// Need to disable this feature due to input blocking
long long g_triggerPressedLeewayTime = 300; // in ms, time after pressing the trigger after which the trigger is considered not pressed anymore

bool isLoaded = false;
bool isLastUpdateValid = false;
bool g_triggerPressed = false;
bool g_triggerReleased = false;
bool g_didTriggerPressGrabObject = false;

TESEffectShader *g_itemSelectedShader = nullptr;
TESEffectShader *g_itemSelectedShaderOffLimits = nullptr;
TESEffectShader *g_currentSelectedShader = nullptr;

const int numPrevPos = 5; // length of previous kept hand positions
NiPoint3 rightHandPositions[numPrevPos]; // previous n hand positions


//auto hookLoc = RelocAddr<uintptr_t>(0x6496D3); // - CellAnimations <- this one is actually correct, but doesnt work for arrows
auto hookLoc = RelocAddr<uintptr_t>(0x77033C);
//auto hookedFunc = RelocAddr<uintptr_t>(0x648960); // - CellAnimations
auto hookedFunc = RelocAddr<uintptr_t>(0x77E1F0);

uintptr_t hookedFuncAddr = 0;
typedef void(*_UpdateImpl)(Projectile *_this, float a_delta);


void HookFunc(Projectile *_this)
{
	NiPointer<TESObjectREFR> pullObj;
	if (LookupREFRByHandle(pullObjHandle, pullObj)) {
		if (pullObj == _this && isPullObjInFlightProjectile && _this->loadedState) {
			_this->loadedState->node->m_localTransform.rot = inFlightOriginalRotation;
		}
	}
}


void Hook_Commit(void)
{
	struct Code : Xbyak::CodeGenerator {
		Code(void * buf) : Xbyak::CodeGenerator(256, buf)
		{
			Xbyak::Label jumpBack, tmpStore1;

			// Save args
			mov(rax, tmpStore1);
			mov(ptr[rax], rcx);

			// Call original function
			mov(rax, hookedFuncAddr);
			call(rax);

			// Restore args
			mov(rax, tmpStore1);
			mov(rcx, ptr[rax]);

			// Just push all regs that could possibly be modified in a function
			push(rax);
			push(rcx);
			push(rdx);
			push(r8);
			push(r9);
			push(r10);
			push(r11);
			sub(rsp, 0x68); // Need to keep the stack SIXTEEN BYTE ALIGNED
			movsd(ptr[rsp], xmm0);
			movsd(ptr[rsp + 0x10], xmm1);
			movsd(ptr[rsp + 0x20], xmm2);
			movsd(ptr[rsp + 0x30], xmm3);
			movsd(ptr[rsp + 0x40], xmm4);
			movsd(ptr[rsp + 0x50], xmm5);

			// Call our damn function
			mov(rax, (uintptr_t)HookFunc);
			call(rax);

			movsd(xmm0, ptr[rsp]);
			movsd(xmm1, ptr[rsp + 0x10]);
			movsd(xmm2, ptr[rsp + 0x20]);
			movsd(xmm3, ptr[rsp + 0x30]);
			movsd(xmm4, ptr[rsp + 0x40]);
			movsd(xmm5, ptr[rsp + 0x50]);
			add(rsp, 0x68);
			pop(r11);
			pop(r10);
			pop(r9);
			pop(r8);
			pop(rdx);
			pop(rcx);
			pop(rax);

			// Jump back to whence we came (+ the size of the initial branch instruction)
			jmp(ptr[rip + jumpBack]);

			L(jumpBack);
			dq(hookLoc.GetUIntPtr() + 5);
			L(tmpStore1);
			dq(0);
		}
	};

	void * codeBuf = g_localTrampoline.StartAlloc();
	Code code(codeBuf);
	g_localTrampoline.EndAlloc(code.getCurr());

	g_branchTrampoline.Write5Branch(hookLoc.GetUIntPtr(), uintptr_t(code.getCode()));
}


bool TryHook()
{
	// This should be sized to the actual amount used by your trampoline
	static const size_t TRAMPOLINE_SIZE = 256;

	if (g_trampoline) {
		void* branch = g_trampoline->AllocateFromBranchPool(g_pluginHandle, TRAMPOLINE_SIZE);
		if (!branch) {
			_ERROR("couldn't acquire branch trampoline from SKSE. this is fatal. skipping remainder of init process.");
			return false;
		}

		g_branchTrampoline.SetBase(TRAMPOLINE_SIZE, branch);

		void* local = g_trampoline->AllocateFromLocalPool(g_pluginHandle, TRAMPOLINE_SIZE);
		if (!local) {
			_ERROR("couldn't acquire codegen buffer from SKSE. this is fatal. skipping remainder of init process.");
			return false;
		}

		g_localTrampoline.SetBase(TRAMPOLINE_SIZE, local);
	}
	else {
		if (!g_branchTrampoline.Create(TRAMPOLINE_SIZE)) {
			_ERROR("couldn't create branch trampoline. this is fatal. skipping remainder of init process.");
			return false;
		}
		if (!g_localTrampoline.Create(TRAMPOLINE_SIZE, nullptr))
		{
			_ERROR("couldn't create codegen buffer. this is fatal. skipping remainder of init process.");
			return false;
		}
	}

	Hook_Commit();
	return true;
}


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


	long long currentTime = GetTime();


	static BSFixedString rHandStr("NPC R Hand [RHnd]");
	NiAVObject *rightHand = player->GetNiRootNode(1)->GetObjectByName(&rHandStr.data);
	if (!rightHand) {
		_MESSAGE("No right hand");
		return;
	}
	NiPoint3 handPos = rightHand->m_worldTransform.pos;

	// Update positions array to this frame
	for (int i = numPrevPos - 1; i >= 1; i--) {
		rightHandPositions[i] = rightHandPositions[i - 1];
	}
	rightHandPositions[0] = handPos;

	NiPoint3 castDirection = rightHand->m_worldTransform.rot * g_handAdjust;

	static BSFixedString hmdNodeStr("HmdNode");
	NiAVObject *hmdNode = GetHighestParent(player->loadedState->node)->GetObjectByName(&hmdNodeStr.data);

	NiTransform inversePlayerTransform;
	hmdNode->m_worldTransform.Invert(inversePlayerTransform);

	NiPoint3 hmdForward = { hmdNode->m_worldTransform.rot.data[0][1], hmdNode->m_worldTransform.rot.data[1][1], hmdNode->m_worldTransform.rot.data[2][1] };

	NiPoint3 handPosLocal = inversePlayerTransform * handPos;

	static BSFixedString rClavicleStr("NPC R Clavicle [RClv]");
	NiAVObject *rightClavicle = player->GetNiRootNode(0)->GetObjectByName(&rClavicleStr.data);
	// TODO: Use delta between consecutive clavicle->hand vectors for hand motions?

	NiPointer<TESObjectREFR> pullObj;

	if (!LookupREFRByHandle(pullObjHandle, pullObj)) {
		// Convert hand position from skyrim coords to havok coords
		float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;
		NiPoint3 hkHandPos = handPos * havokWorldScale;
		NiPoint3 hkTargetPos = hkHandPos + castDirection * g_castDistance;

		NiPoint3 hitPosition = { hkTargetPos.x, hkTargetPos.y, hkTargetPos.z };

		bhkWorld *world = GetWorld(cell);
		if (!world) {
			_MESSAGE("Could not get havok world from player cell");
			return;
		}

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
		sphereShape->m_radius = g_castRadius;
		linearCastInput.m_to = { hitPosition.x, hitPosition.y, hitPosition.z, 0 };
		hkpWorld_LinearCast(world->world, &sphere->phantom->m_collidable, &linearCastInput, &cdPointCollector, nullptr);
		sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = filterInfoBefore;
		sphereShape->m_radius = radiusBefore;

		// Process result of cast
		bool isSelected = false;
		//TESObjectREFR *closestObj = nullptr;
		NiPointer<TESObjectREFR> closestObj;
		hkpCollidable *closestColl = nullptr;
		float closestDistance = (std::numeric_limits<float>::max)();

		for (auto pair : cdPointCollector.m_hits) {
			auto collidable = static_cast<hkpCollidable *>(pair.second);
			NiPointer<TESObjectREFR> ref = FindCollidableRef(collidable);
			if (ref) {
				TESForm *baseForm = ref->baseForm;
				if (baseForm) {
					auto actor = DYNAMIC_CAST(ref, TESObjectREFR, Actor);
					if (actor || IsSelectable(baseForm)) {
						if (actor) {
							if (!ref->IsDead(1)) {
								// For actors, only select them if they're dead
								continue;
							}
						}
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
		}

		// Select the new thing
		if (closestObj && DotProduct(castDirection, hmdForward) >= g_requiredCastDotProduct) {
			NiPointer<TESObjectREFR> selectedObj;
			if (!LookupREFRByHandle(selectedObjHandle, selectedObj) ||  closestObj != selectedObj) {
				if (selectedObj) {
					// Deselect the old thing if something else was selected
					EffectShader_Stop(vmRegistry, 0, g_currentSelectedShader, selectedObj);
				}
				selectedObjHandle = GetOrCreateRefrHandle(closestObj.m_pObject);
				selectedColl = closestColl;

				if (CALL_MEMBER_FN(closestObj, IsOffLimits)()) {
					g_currentSelectedShader = g_itemSelectedShaderOffLimits;
				}
				else {
					g_currentSelectedShader = g_itemSelectedShader;
				}
				EffectShader_Play(vmRegistry, 0, g_currentSelectedShader, closestObj, -1.0f);
			}
			isSelected = true;
			lastSelectedTime = currentTime;
		}

		// If time has run out and nothing is selected, deselect whatever is selected
		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObjHandle, selectedObj) && !isSelected && currentTime - lastSelectedTime > g_selectedLeewayTime) {
			EffectShader_Stop(vmRegistry, 0, g_currentSelectedShader, selectedObj);
			selectedObjHandle = *g_invalidRefHandle;
		}

		if (g_triggerPressed && currentTime - triggerPressedTime <= g_triggerPressedLeewayTime) {
			if (LookupREFRByHandle(selectedObjHandle, selectedObj)) {
				// Pick up the item

				pullColl = selectedColl;
				pullObjHandle = selectedObjHandle;

				// Set to false only here, so that you can hold the trigger until the cast hits something valid
				g_triggerPressed = false;
				g_didTriggerPressGrabObject = true; // This variable is not set to false when we push/pull the object

				isPullObjInFlightProjectile = false;
				isPullObjImpactedProjectile = false;
				auto baseForm = selectedObj->baseForm;
				if (baseForm && baseForm->formType == kFormType_Projectile) {
					auto velocity = (NiPoint3 *)((UInt64)selectedObj.m_pObject + 0xfc);
					auto impactData = *(void **)((UInt64)selectedObj.m_pObject + 0x98);
					if (impactData) {
						// If the projectile has impact data, then it has well, impacted something
						isPullObjImpactedProjectile = true;
					}
					else {
						isPullObjInFlightProjectile = true;
						inFlightProjectileOriginalSpeed = VectorLength(*velocity);
					}
				}

				isPullObjActor = false;
				auto actor = DYNAMIC_CAST(selectedObj, TESObjectREFR, Actor);
				if (actor) {
					isPullObjActor = true;
				}
			}
		}
	}

	if (g_triggerReleased) {
		g_triggerReleased = false;
		g_didTriggerPressGrabObject = false;

		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObjHandle, selectedObj)) {
			EffectShader_Stop(vmRegistry, 0, g_currentSelectedShader, selectedObj);
			selectedObjHandle = *g_invalidRefHandle;
		}
		if (LookupREFRByHandle(pullObjHandle, pullObj)) {
			if (isPullObjInFlightProjectile) {
				// In-flight projectile
				if (pullObj->loadedState) {
					auto velocity = (NiPoint3 *)((UInt64)pullObj.m_pObject + 0xfc);

					auto rot = pullObj->loadedState->node->m_localTransform.rot;
					NiPoint3 forward = { rot.data[0][1], rot.data[1][1], rot.data[2][1] };

					*velocity = forward * inFlightProjectileOriginalSpeed;
				}
			}
			// Drop the item
			pullObjHandle = *g_invalidRefHandle;
			pullDesired = false;
		}
	}

	if (LookupREFRByHandle(pullObjHandle, pullObj) && !(pullObj->flags & TESForm::kFlagIsDeleted)) {
		bool cancel = false;
		// Only try to access loadedState after 3d is loaded for the projectile
		if (isPullObjInFlightProjectile) {
			bool waitingToInitialize3D = *(bool *)((UInt64)pullObj.m_pObject + 0x1dc);
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
					auto shooter = (UInt32 *)((UInt64)pullObj.m_pObject + 0x120);
					*shooter = playerHandle;

					auto actorCause = *(UInt32 **)((UInt64)pullObj.m_pObject + 0x118);
					UInt32 handle = *actorCause;
					*actorCause = playerHandle;

					// Need to set collision to same as if the player cast it, so that it collides with the _original_ caster (probably an enemy)
					auto phantom = *(bhkSimpleShapePhantom**)((UInt64)pullObj.m_pObject + 0xE0);
					phantom->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= 0x0000FFFF; // clear out the collision group
					phantom->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= 0x02420000; // set the group to player group

					// Save the rotation when we catch it
					inFlightOriginalRotation = pullObj->loadedState->node->m_localTransform.rot;
				}
			}

			// Determine the position where we want the object to be
			// Essentially it's a cylinder with a radius of the original distance when we grabbed it, and a height determined by some limit
			float w = VectorLength(NiPoint3(initialPullObjRelativePosition.x, initialPullObjRelativePosition.y, 0)); // horizontal distance from hand to object
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

			if (pullDesired && !isPullObjInFlightProjectile && !isPullObjActor && pullObj->baseForm->formType != kFormType_MovableStatic) {
				// If it's an in-flight projectile or dead body, no pull effect
				float newMagnitude = (VectorLength(relObjPos) * 0.1f) / havokWorldScale;
				newMagnitude = min(newMagnitude, 12.0f); // Cap at some reasonable value
				NiPoint3 newVelocity = VectorNormalized(-relObjPos) * newMagnitude;

				ApplyHavokImpulse(vmRegistry, 0, pullObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
				motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };

				// If close enough to hand, pick it up
				if (VectorLength(relObjPos) < g_handActivateDistance * havokWorldScale) {
					_MESSAGE("Equipping");
					// Pickup the item
					Activate(vmRegistry, 0, pullObj, player, false);
					// If the item is a weapon, equip it too
					auto baseForm = pullObj->baseForm;
					if (g_equipWeapons && baseForm && baseForm->formType == kFormType_Weapon) {
						auto *weapon = DYNAMIC_CAST(baseForm, TESForm, TESObjectWEAP);
						if (weapon) {
							papyrusActor::EquipItemEx(player, weapon, 1, false, false);
						}
					}
					pullObjHandle = *g_invalidRefHandle;
					selectedObjHandle = *g_invalidRefHandle;
					pullDesired = false;
					pushDesired = false;
				}
			}
			else if (pushDesired) {
				NiPoint3 dir = VectorNormalized(relObjPos);

				if (isPullObjInFlightProjectile) {
					auto velocity = (NiPoint3 *)((UInt64)pullObj.m_pObject + 0xfc);
					*velocity = dir * inFlightProjectileOriginalSpeed;

					NiPoint3 forward = dir;

					NiPoint3 worldUpAlongForward = forward * DotProduct({ 0, 0, 1 }, forward); // Project world up vector onto our forward vector
					NiPoint3 up = VectorNormalized(NiPoint3(0, 0, 1) - worldUpAlongForward);
					NiPoint3 right = CrossProduct(forward, up);

					pullObj->loadedState->node->m_localTransform.rot.data[0][0] = right.x;
					pullObj->loadedState->node->m_localTransform.rot.data[1][0] = right.y;
					pullObj->loadedState->node->m_localTransform.rot.data[2][0] = right.z;

					pullObj->loadedState->node->m_localTransform.rot.data[0][1] = forward.x;
					pullObj->loadedState->node->m_localTransform.rot.data[1][1] = forward.y;
					pullObj->loadedState->node->m_localTransform.rot.data[2][1] = forward.z;

					pullObj->loadedState->node->m_localTransform.rot.data[0][2] = up.x;
					pullObj->loadedState->node->m_localTransform.rot.data[1][2] = up.y;
					pullObj->loadedState->node->m_localTransform.rot.data[2][2] = up.z;

					// Set Z rotation only - it's the only one the game cares about for projectiles anyways
					NiPoint3 zeroRotationVec(0, 1, 0);
					NiPoint3 worldRightVec(1, 0, 0);
					NiPoint3 forwardFlattened = VectorNormalized({ forward.x, forward.y, 0 });
					float angleFromZero = acosf(DotProduct(forwardFlattened, zeroRotationVec));

					// Angle above is always positive - need to negate it depending on which way we 'wind' around Z
					if (DotProduct(forwardFlattened, worldRightVec) < 0) {
						angleFromZero *= -1;
					}

					pullObj->rot.z = angleFromZero;
				}
				else {
					float magnitude = 80.0f;
					ApplyHavokImpulse(vmRegistry, 0, pullObj, dir.x, dir.y, dir.z, magnitude);
				}

				EffectShader_Stop(vmRegistry, 0, g_currentSelectedShader, pullObj);

				pullObjHandle = *g_invalidRefHandle;
				selectedObjHandle = *g_invalidRefHandle;
			}
			else {
				// No push or pull - just hold it where we want it
				if (isPullObjInFlightProjectile) {
					// In-flight projectile
					float newMagnitude = (VectorLength(deltaPos) * 5.0f) / havokWorldScale;
					newMagnitude = min(newMagnitude, 10.0f / havokWorldScale); // Cap at some reasonable value
					NiPoint3 newVelocity = VectorNormalized(deltaPos) * newMagnitude;

					auto velocity = (NiPoint3 *)((UInt64)pullObj.m_pObject + 0xfc);
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
			triggerPressedTime = GetTime();
			g_triggerPressed = true;
			g_triggerReleased = false;
		}
		else if (eventType == PapyrusVR::VREventType::VREventType_Released) {
			g_triggerReleased = true;
			g_triggerPressed = false;
		}
	}
}

bool triggerPressed = false;
bool unsheatheDesired = false;
bool OnControllerStateChanged(vr::TrackedDeviceIndex_t unControllerDeviceIndex, const vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, vr::VRControllerState_t* pOutputControllerState)
{
	// TODO: Deal with left handed mode
	vr::ETrackedControllerRole rightControllerRole = vr::ETrackedControllerRole::TrackedControllerRole_RightHand;
	vr::TrackedDeviceIndex_t rightController = g_openvrHook->GetVRSystem()->GetTrackedDeviceIndexForControllerRole(rightControllerRole);
	if (unControllerDeviceIndex == rightController) {
		bool triggerPressedBefore = triggerPressed;
		// Check if the trigger is pressed
		if (pOutputControllerState->ulButtonPressed & vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Trigger)) {
			triggerPressed = true;
			if (g_didTriggerPressGrabObject) {
				// If something is grabbed, disable the trigger
				pOutputControllerState->ulButtonPressed &= ~vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Trigger);
				if (!triggerPressedBefore) {
					// but still propagate the trigger to _us_
					OnButtonEvent(PapyrusVR::VREventType::VREventType_Pressed, PapyrusVR::EVRButtonId::k_EButton_SteamVR_Trigger, PapyrusVR::VRDevice::VRDevice_RightController);
				}
			}
			else {
				if (!triggerPressedBefore) {
					// Trigger pressed but not object is grabbed (yet?). Do not unsheathe weapons until leeway time has passed.
					PlayerCharacter *pc = *g_thePlayer;
					if (pc && !pc->actorState.IsWeaponDrawn() && !IsInMenuMode(vmRegistry, 0)) {
						unsheatheDesired = true;
					}
				}
			}
		}
		else {
			triggerPressed = false;
		}

		if (unsheatheDesired) {
			long long currentTime = GetTime();
			if (currentTime - triggerPressedTime <= g_triggerPressedLeewayTime) {
				// Suppress trigger press
				pOutputControllerState->ulButtonPressed &= ~vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Trigger);
			}
			else {
				unsheatheDesired = false;
				// Do the unsheathing, only if we didn't grab something
				if (!g_didTriggerPressGrabObject) {
					PlayerCharacter *pc = *g_thePlayer;
					if (pc && !pc->actorState.IsWeaponDrawn() && !IsInMenuMode(vmRegistry, 0)) {
						// Vtbl offset for DrawSheatheWeapon is wrong in skse vr
						typedef void(*_DrawSheatheWeapon)(Actor *actor, bool draw);
						UInt64 *vtbl = *((UInt64 **)pc);
						((_DrawSheatheWeapon)(vtbl[0xA8]))(pc, true);
					}
				}
			}
		}
	}
	return true;
}

extern "C" {	
	void OnDataLoaded()
	{
		pullObjHandle = *g_invalidRefHandle;

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

		shaderForm = LookupFormByID(GetFullFormID(modInfo, 0x4EB5));
		if (shaderForm) {
			g_itemSelectedShaderOffLimits = DYNAMIC_CAST(shaderForm, TESForm, TESEffectShader);
			if (!g_itemSelectedShaderOffLimits) {
				_MESSAGE("Failed to cast selected item off limits shader form");
				return;
			}
		}
		else {
			_MESSAGE("Failed to get slected item off limits shader form");
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
				g_openvrHook->RegisterControllerStateCB(OnControllerStateChanged);
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

		hookedFuncAddr = hookedFunc.GetUIntPtr(); // before trampolines

		g_trampoline = (SKSETrampolineInterface *)skse->QueryInterface(kInterface_Trampoline);
		if (!g_trampoline) {
			_ERROR("couldn't get trampoline interface");
		}
		if (!TryHook()) {
			_ERROR("Failed to perform hook");
		}

		for (int i = 0; i < numPrevPos; i++) {
			rightHandPositions[i] = {0, 0, 0};
		}

		// wait for PapyrusVR init (during PostPostLoad SKSE Message)

		return true;
	}
};