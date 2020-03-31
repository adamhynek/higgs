#include "skse64/GameRTTI.h"
#include "skse64/PapyrusActor.h"

#include "grabber.h"
#include "offsets.h"
#include "utils.h"
#include "config.h"
#include "menu_checker.h"


void Grabber::Select(TESObjectREFR *obj, const SelectedObject &other)
{
	selectedObject.handle = GetOrCreateRefrHandle(obj);

	if (CALL_MEMBER_FN(obj, IsOffLimits)()) {
		selectedObject.appliedShader = itemSelectedShaderOffLimits;
	}
	else {
		selectedObject.appliedShader = itemSelectedShader;
	}

	// Play effect shader only if the other hand doesn't have the shader already playing on the object
	NiPointer<TESObjectREFR> otherSelectedObj;
	UInt32 otherSelectedObjHandle = other.handle;
	if (!LookupREFRByHandle(otherSelectedObjHandle, otherSelectedObj) || otherSelectedObj != obj) {
		EffectShader_Play(VM_REGISTRY, 0, selectedObject.appliedShader, obj, -1.0f);
	}
}


void Grabber::Deselect(TESObjectREFR *obj, const SelectedObject &other)
{
	UInt32 otherHandle = other.handle;
	NiPointer<TESObjectREFR> otherSelectedObj;
	if (!LookupREFRByHandle(otherHandle, otherSelectedObj) || other.handle != selectedObject.handle) {
		EffectShader_Stop(VM_REGISTRY, 0, selectedObject.appliedShader, obj);
		selectedObject.appliedShader = nullptr;
	}

	selectedObject.handle = *g_invalidRefHandle;
	selectedObject.collidable = nullptr;
}


void Grabber::Grab(TESObjectREFR *obj)
{
	grabbedObject.collidable = selectedObject.collidable;
	grabbedObject.handle = selectedObject.handle;

	// Set to false only here, so that you can hold the trigger until the cast hits something valid
	triggerPressed = false;
	didTriggerPressGrabObject = true; // This variable is not set to false when we push/pull the object

	grabbedObject.isImpactedProjectile = false;
	auto baseForm = obj->baseForm;
	if (baseForm && baseForm->formType == kFormType_Projectile) {
		auto impactData = *(void **)((UInt64)obj + 0x98);
		if (impactData) {
			// If the projectile has impact data, then it has well, impacted something
			grabbedObject.isImpactedProjectile = true;
		}
	}

	grabbedObject.isActor = false;
	auto actor = DYNAMIC_CAST(obj, TESObjectREFR, Actor);
	if (actor) {
		grabbedObject.isActor = true;
	}
}


void Grabber::UnGrab()
{
	grabbedObject.handle = *g_invalidRefHandle;
	grabbedObject.collidable = nullptr;
	grabbedObject.isActor = false;
	grabbedObject.isImpactedProjectile = false;
}


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

	NiPoint3 castDirection = handNode->m_worldTransform.rot * Config::options.handAdjust;

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

	if (!LookupREFRByHandle(grabbedObject.handle, grabbedObj)) {
		// Convert hand position from skyrim coords to havok coords
		float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;
		NiPoint3 hkHmdPos = hmdPos * havokWorldScale;
		NiPoint3 hkHandPos = handPos * havokWorldScale;
		NiPoint3 hkTargetPos = hkHandPos + castDirection * Config::options.castDistance;

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
		sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = 0x2C; // 'CustomPick2' layer, to pick up projectiles, because ONLY THIS GODDAMN LAYER can collide with projectiles
		float radiusBefore = sphereShape->m_radius; // save radius so we can restore it
		sphereShape->m_radius = Config::options.castRadius;
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
			if (collidable == other.grabbedObject.collidable) {
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
		bool isSelectedThisFrame = false;
		if (closestObj && DotProduct(VectorNormalized(closestHit - hkHmdPos), hmdForward) >= Config::options.requiredCastDotProduct) {
			NiPointer<TESObjectREFR> selectedObj;
			if (!LookupREFRByHandle(selectedObject.handle, selectedObj) || closestObj != selectedObj) {
				if (selectedObj) {
					// Deselect the old thing if something else was selected
					Deselect(selectedObj, other.selectedObject);
				}
				Select(closestObj, other.selectedObject);
			}
			selectedObject.collidable = closestColl; // Set selected collidable no matter what, as we can have objects with more than one collidable

			isSelectedThisFrame = true;
			lastSelectedTime = currentTime;
		}

		// If time has run out and nothing is selected, deselect whatever is selected
		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObject.handle, selectedObj) && !isSelectedThisFrame && currentTime - lastSelectedTime > Config::options.selectedLeewayTime) {
			Deselect(selectedObj, other.selectedObject);
		}

		if (triggerPressed && currentTime - triggerPressedTime <= Config::options.triggerPressedLeewayTime) {
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				// Pick up the item
				Grab(selectedObj);
			}
		}
	}

	if (triggerReleased) {
		triggerReleased = false;
		didTriggerPressGrabObject = false;

		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
			Deselect(selectedObj, other.selectedObject);
		}
		if (LookupREFRByHandle(grabbedObject.handle, grabbedObj)) {
			// Drop the item
			UnGrab();
			pullDesired = false;
		}
	}

	if (LookupREFRByHandle(grabbedObject.handle, grabbedObj) && grabbedObj->loadedState && grabbedObj->loadedState->node) {
		float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;

		// This is a bit hacky...
		hkpMotion *motion = reinterpret_cast<hkpMotion *>((UInt64)grabbedObject.collidable->m_motion - offsetof(hkpMotion, m_motionState));

		hkVector4 translation = motion->m_motionState.m_transform.m_translation;

		NiPoint3 hkObjPos = { translation.x, translation.y, translation.z };
		NiPoint3 hkHandPos = handPos * havokWorldScale;

		NiPoint3 relObjPos = hkObjPos - hkHandPos;

		if (!prevGrabbedObj) {
			pushDesired = false;
			pullDesired = false;

			initialGrabbedObjRelativePosition = relObjPos;
			initialHandShoulderDistance = VectorLength(handPos - upperArmPos);

			if (grabbedObject.isImpactedProjectile) { // It's an embedded projectile, i.e. stuck in a wall etc.
				auto collObj = (bhkCollisionObject *)grabbedObj->loadedState->node->unk040;
				if (collObj) {
					// Do not use grabbedObject.collidable here, as that's the _phantom_ on the projectile. We want the one attached to the 3D
					auto collidable = &collObj->body->hkBody->m_collidable;
					// Projectiles have 'Fixed' motion type by default, making them unmovable
					SetMotionTypeFunctor(VM_REGISTRY, 0, grabbedObj, 3, true);
					// Projectiles also do not interact with collision usually. We need to change the filter to make them interact.
					collidable->m_broadPhaseHandle.m_collisionFilterInfo = 0x02420006; // player collision group, 'projectile' collision layer
					collidable->m_broadPhaseHandle.m_objectQualityType = 4; // Set to 'moving' quality instead of 'fixed'
				}
			}
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

		if (pullDesired && !grabbedObject.isActor && grabbedObj->baseForm->formType != kFormType_MovableStatic) {
			// If it's an in-flight projectile or dead body, no pull effect

			float inverseMass = motion->m_inertiaAndMassInv.w;

			float newMagnitude = (pow(inverseMass, Config::options.massExponent) * Config::options.pullVelocityMultiplier) / havokWorldScale;
			newMagnitude = min(newMagnitude, 12.0f); // Cap at some reasonable value
			NiPoint3 newVelocity = VectorNormalized(-relObjPos) * newMagnitude;

			ApplyHavokImpulse(VM_REGISTRY, 0, grabbedObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
			motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };

			// If close enough to hand, pick it up
			if (VectorLength(relObjPos) < Config::options.handActivateDistance * havokWorldScale) {
				_MESSAGE("Equipping");

				Deselect(grabbedObj, other.selectedObject);
				UnGrab();

				// Pickup the item
				Activate(VM_REGISTRY, 0, grabbedObj, player, false);
				// If the item is a weapon, equip it too
				auto baseForm = grabbedObj->baseForm;
				if (Config::options.equipWeapons && baseForm && baseForm->formType == kFormType_Weapon) {
					auto *weapon = DYNAMIC_CAST(baseForm, TESForm, TESObjectWEAP);
					if (weapon) {
						papyrusActor::EquipItemEx(player, weapon, 1, false, false);
					}
				}

				pullDesired = false;
				pushDesired = false;
			}
		}
		else if (pushDesired) {
			NiPoint3 dir = VectorNormalized(relObjPos);

			float inverseMass = motion->m_inertiaAndMassInv.w;

			if (grabbedObject.isActor && grabbedObj->loadedState && grabbedObj->loadedState->node) {
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

			float newMagnitude = (pow(inverseMass, Config::options.massExponent) * Config::options.pushVelocityMultiplier) / havokWorldScale;
			NiPoint3 newVelocity = VectorNormalized(relObjPos) * newMagnitude;

			ApplyHavokImpulse(VM_REGISTRY, 0, grabbedObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
			motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };

			Deselect(grabbedObj, other.selectedObject);
			UnGrab();
		}
		else {
			// No push or pull - just hold it where we want it

			// Determine the position where we want the object to be
			// Essentially it's a cylinder with a radius of the original distance when we grabbed it, and a height determined by some limit
			float maxHeight = grabbedObject.isActor ? Config::options.maxBodyHeight : Config::options.maxItemHeight;
			NiPoint3 horiz;
			float h;
			if (castDirection.z <= 0.9999f) {
				float w = VectorLength(NiPoint3(initialGrabbedObjRelativePosition.x, initialGrabbedObjRelativePosition.y, 0)); // horizontal distance from hand to object
				float theta = asinf(castDirection.z); // vertical angle of hand relative to horizontal
				float tanTheta = tanf(theta);
				h = min(w * tanTheta, maxHeight); // desired height relative to horizontal from hand
				horiz = VectorNormalized(NiPoint3(castDirection.x, castDirection.y, 0)) * (h / tanTheta); // desired horizontal position relative to hand
			}
			else {
				// Degenerate case
				h = maxHeight;
				horiz = { 0, 0, 0 };
			}
			
			NiPoint3 desiredPos = NiPoint3(horiz.x, horiz.y, h);

			// Use distance from upper arm (shoulder-ish) to hand to control how far away from us we want the object to be
			float handShoulderDistance = VectorLength(handPos - upperArmPos);
			float handDistanceRatio = (handShoulderDistance - 10.0f) / (initialHandShoulderDistance - 10.0f);
			desiredPos *= handDistanceRatio;

			NiPoint3 deltaPos = desiredPos - relObjPos;

			float inverseMass = motion->m_inertiaAndMassInv.w;

			if (grabbedObject.isActor && grabbedObj->loadedState && grabbedObj->loadedState->node) {
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

			float newMagnitude = VectorLength(deltaPos) * pow(inverseMass, Config::options.massExponent) * Config::options.hoverVelocityMultiplier;

			if (grabbedObject.isActor) {
				newMagnitude *= Config::options.bodyVelocityMultiplier;
			}

			newMagnitude /= havokWorldScale;

			NiPoint3 newVelocity = VectorNormalized(deltaPos) * newMagnitude;

			ApplyHavokImpulse(VM_REGISTRY, 0, grabbedObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
			motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };
		}
	}


	prevGrabbedObj = grabbedObj;
	prevHandPosRoomspace = handPosRoomspace;
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
		if (currentTime - triggerPressedTime <= Config::options.triggerPressedLeewayTime) {
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


bool Grabber::ShouldDisplayRollover(const TESObjectREFR *grabbedObj)
{
	return (grabbedObj->baseForm && grabbedObj->baseForm->formType != kFormType_MovableStatic && !grabbedObject.isActor);
}


void Grabber::SetupRollover(NiAVObject *rolloverNode, const TESObjectREFR *grabbedObj)
{
	// Give the hud with info about the object you're floating

	// First, change rotation/position/scale of the hud prompt
	rolloverNode->m_localTransform.pos = rolloverOffset;
	rolloverNode->m_localTransform.rot = rolloverRotation;
	rolloverNode->m_localTransform.scale = Config::options.rolloverScale;

	// Now set all the places I could find that get set to the handle of the pointed at object usually
	if (SELECTED_HANDLES) {
		UInt32 *selectedHandles = *SELECTED_HANDLES;
		selectedHandles[2] = grabbedObject.handle;
		selectedHandles[5] = grabbedObject.handle;
		selectedHandles[8] = grabbedObject.handle;
	}
}
