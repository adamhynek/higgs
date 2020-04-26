#include <numeric>

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

	selectedObject.isImpactedProjectile = false;
	auto baseForm = obj->baseForm;
	if (baseForm && baseForm->formType == kFormType_Projectile) {
		auto impactData = *(void **)((UInt64)obj + 0x98);
		if (impactData) {
			// If the projectile has impact data, then it has well, impacted something
			selectedObject.isImpactedProjectile = true;
		}
	}

	selectedObject.isActor = false;
	auto actor = DYNAMIC_CAST(obj, TESObjectREFR, Actor);
	if (actor) {
		selectedObject.isActor = true;
	}

	state = SELECTED;
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

	state = IDLE;
}


void Grabber::PoseUpdate(const Grabber &other, bool allowGrab, NiNode *playerWorldNode)
{
	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->loadedState || !player->loadedState->node)
		return;

	TESObjectCELL* cell = player->parentCell;
	if (!cell)
		return;

	NiAVObject *handNode = player->GetNiRootNode(1)->GetObjectByName(&handNodeName.data);
	if (!handNode)
		return;

	NiPoint3 handPos = handNode->m_worldTransform.pos;

	NiPoint3 castDirection = handNode->m_worldTransform.rot * Config::options.handAdjust;

	NiAVObject *hmdNode = playerWorldNode->GetObjectByName(&hmdNodeStr.data);
	if (!hmdNode)
		return;

	NiPoint3 hmdPos = hmdNode->m_worldTransform.pos;

	NiPoint3 hmdForward = { hmdNode->m_worldTransform.rot.data[0][1], hmdNode->m_worldTransform.rot.data[1][1], hmdNode->m_worldTransform.rot.data[2][1] };

	NiAVObject *wandNode = playerWorldNode->GetObjectByName(&wandNodeName.data);
	if (!wandNode)
		return;

	NiPoint3 handPosRoomspace = wandNode->m_localTransform.pos;
	NiPoint3 handVelocityRoomspace = (handPosRoomspace - prevHandPosRoomspace) * g_deltaTime;

	// Update velocities array to this frame
	for (int i = numPrevVel - 1; i >= 1; i--) {
		handVelocities[i] = handVelocities[i - 1];
	}
	handVelocities[0] = handVelocityRoomspace;

	NiPoint3 avgVelocityRoomspace = std::accumulate(handVelocities, &handVelocities[numPrevVel - 1], NiPoint3()) / numPrevVel;

	// Get whatever transform takes the wand position from room space to skyrim worldspace
	NiMatrix33 localToWorldTransform = wandNode->m_worldTransform.rot * wandNode->m_localTransform.rot.Transpose();
	NiPoint3 velocityWorldspace = localToWorldTransform * avgVelocityRoomspace;
	float handSpeedInSpellDirection = DotProduct(velocityWorldspace, castDirection) * 90.0f; // divide by delta time of 11ms because that's where I derived the thresholds

	if (std::string(handNodeName.data) == "NPC R Hand [RHnd]") {
		PrintToFile(std::to_string(VectorLength(castDirection - prevHandDirection)), "hand_speed_r.txt");
	}
	if (std::string(handNodeName.data) == "NPC L Hand [LHnd]") {
		PrintToFile(std::to_string(VectorLength(castDirection - prevHandDirection)), "hand_speed_l.txt");
	}

	NiAVObject *upperArmNode = player->GetNiRootNode(0)->GetObjectByName(&upperArmNodeName.data);
	if (!upperArmNode)
		return;

	NiPoint3 upperArmPos = upperArmNode->m_worldTransform.pos;

	if (state == IDLE || state == SELECTED) {
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
		// 'CustomPick2' layer, to pick up projectiles, because ONLY THIS GODDAMN LAYER can collide with projectiles. This filterinfo will collide with everything.
		sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = 0x2C;
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
			if (!allowGrab || (other.HasExclusiveObject() && collidable == other.selectedObject.collidable)) {
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

		// Check if we should select something new. If yes, stay in SELECTED but select the new object
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
			lastSelectedTime = g_currentFrameTime;
		}

		if (state == SELECTED) {
			// If time has run out and nothing is selected, deselect whatever is selected
			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				if (LookupREFRByHandle(selectedObject.handle, selectedObj) && !isSelectedThisFrame && g_currentFrameTime - lastSelectedTime > Config::options.selectedLeewayTime) {
					Deselect(selectedObj, other.selectedObject);
				}

				// Check if we should grab the object. If yes, grab and go to GRABBED
				if (triggerPressed && g_currentFrameTime - triggerPressedTime <= Config::options.triggerPressedLeewayTime) {
					if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
						state = SELECTION_LOCKED;
						selectionLockedTime = g_currentFrameTime;
						initialHandDirection = castDirection;
						// Set to false only here, so that you can hold the trigger until the cast hits something valid
						triggerPressed = false;
						didTriggerPressGrabObject = true; // This variable is not set to false when we push/pull the object
					}
					else {
						// Selected object no longer exists
						state = IDLE;
					}
				}
			}
			else {
				state = IDLE;
			}
		}

		if (triggerReleased) {
			triggerReleased = false;
			didTriggerPressGrabObject = false;
		}
	}

	if (state == SELECTION_LOCKED) {
		if (triggerReleased) {
			triggerReleased = false;
			didTriggerPressGrabObject = false;

			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				Deselect(selectedObj, other.selectedObject);
			}
		}

		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
//			if (!IsObjectPullable()) {
//				// TODO: Check hand angle still
//				state = GRABBED;
//			}
//			else {
				hkpMotion *motion = reinterpret_cast<hkpMotion *>((UInt64)selectedObject.collidable->m_motion - offsetof(hkpMotion, m_motionState));
				NiPoint3 linearVelocity = { motion->m_angularVelocity.x, motion->m_linearVelocity.y, motion->m_linearVelocity.z };
				NiPoint3 angularVelocity = { motion->m_angularVelocity.x, motion->m_angularVelocity.y, motion->m_angularVelocity.z };
				// TODO: Config all these thresholds
				if (VectorLength(linearVelocity) > 0.1f || VectorLength(angularVelocity) > 0.1f) {
					state = GRABBED;
					grabbedTime = g_currentFrameTime;
				}
				else {

					// TODO: Deselect / go back to IDLE if object gets too far away. Do that for 'grabbed' state too? For movablestatic signs and such.

					bool grab = false, pull = false;
					if (abs(handSpeedInSpellDirection) < abs(prevHandSpeedInSpellDirection) && prevHandSpeedInSpellDirection < -Config::options.pushPullSpeedThreshold) {
						if (IsObjectPullable()) {
							pull = true;
						}
					}
					if (!pull) {
						// Hold object still while hand doesn't point too far away from original location. If far enough, transition to grabbed.
						if (DotProduct(castDirection, initialHandDirection) < Config::options.grabbedDotProductThreshold) {
							grab = true;
						}
					}

					if (pull) {
						pullSpeed = abs(prevHandSpeedInSpellDirection);
						grabbedTime = g_currentFrameTime;
						state = PULLED;
					}
					else if (grab) {
						state = GRABBED;
						grabbedTime = g_currentFrameTime;
					}
				}
//			}
		}
		else {
			state = IDLE;
		}
	}

	NiPointer<TESObjectREFR> selectedObj;
	if (state == GRABBED || state == PULLED) {
		// Check if we should drop the object. If yes, go to IDLE
		if (triggerReleased) {
			triggerReleased = false;
			didTriggerPressGrabObject = false;

			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				Deselect(selectedObj, other.selectedObject);
			}
		}

		if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
			float havokWorldScale = *HAVOK_WORLD_SCALE_ADDR;

			// This is a bit hacky...
			hkpMotion *motion = reinterpret_cast<hkpMotion *>((UInt64)selectedObject.collidable->m_motion - offsetof(hkpMotion, m_motionState));

			hkVector4 translation = motion->m_motionState.m_transform.m_translation;

			NiPoint3 hkObjPos = { translation.x, translation.y, translation.z };
			NiPoint3 hkHandPos = handPos * havokWorldScale;

			NiPoint3 relObjPos = hkObjPos - hkHandPos;

			if (!prevGrabbedObj) {
				initialGrabbedObjRelativePosition = relObjPos;
				initialGrabbedObjWorldPosition = hkObjPos;
				initialHandShoulderDistance = VectorLength(handPos - upperArmPos);

				if (selectedObject.isImpactedProjectile) { // It's an embedded projectile, i.e. stuck in a wall etc.
					auto collObj = (bhkCollisionObject *)selectedObj->loadedState->node->unk040;
					if (collObj) {
						// Do not use grabbedObject.collidable here, as sometimes we end up grabbing the phantom shape of the projectile instead of the 3D one
						auto collidable = &collObj->body->hkBody->m_collidable;
						// Projectiles have 'Fixed' motion type by default, making them unmovable
						SetMotionTypeFunctor(VM_REGISTRY, 0, selectedObj, 3, true);
						// Projectiles also do not interact with collision usually. We need to change the filter to make them interact.
						collidable->m_broadPhaseHandle.m_collisionFilterInfo = 0x02420006; // player collision group, 'projectile' collision layer
						collidable->m_broadPhaseHandle.m_objectQualityType = 4; // Set to 'moving' quality instead of 'fixed'
					}
				}
			}

			if (state == GRABBED) {
				// Check if we should push or pull. If pull, go to PULLED. If push, push it and go to IDLE

				bool push = false, pull = false;
				if (abs(handSpeedInSpellDirection) < abs(prevHandSpeedInSpellDirection)) {
					// We've hit a local max in hand speeds

					// See if we passed the speed threshold. If we did, we want to push or pull.
					if (prevHandSpeedInSpellDirection > Config::options.pushPullSpeedThreshold) {
						push = true;
					}
					else if (prevHandSpeedInSpellDirection < -Config::options.pushPullSpeedThreshold) {
						if (IsObjectPullable()) {
							pull = true;
						}
					}
				}

				if (push) {
					float inverseMass = min(motion->m_inertiaAndMassInv.w, Config::options.inverseMassLimit);

					if (selectedObject.isActor && selectedObj->loadedState && selectedObj->loadedState->node) {
						// For dead bodies, instead of the mass of the individual collidable we hit, use the summed mass over the entire body
						Actor *actor = DYNAMIC_CAST(selectedObj, TESObjectREFR, Actor);
						if (actor) {
							float mass = GetMass(0, false, actor);
							if (mass > 0) {
								inverseMass = 1.0f / mass;
							}
							else {
								auto actorBase = DYNAMIC_CAST(actor->baseForm, TESForm, TESActorBase);
								_WARNING("Could not get mass for actor: %s", actorBase->fullName.name.data);
							}
						}
					}

					float newMagnitude = (pow(inverseMass, Config::options.massExponent) * prevHandSpeedInSpellDirection * Config::options.pushVelocityMultiplier) / havokWorldScale;
					NiPoint3 newVelocity = VectorNormalized(velocityWorldspace) * newMagnitude;

					ApplyHavokImpulse(VM_REGISTRY, 0, selectedObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
					motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };

					Deselect(selectedObj, other.selectedObject);
				}
				else if (pull) {
					pullSpeed = abs(prevHandSpeedInSpellDirection);
					grabbedTime = g_currentFrameTime;
					initialGrabbedObjWorldPosition = hkObjPos;
					state = PULLED;
				}
				else {
					// Determine the position where we want the object to be
					// Essentially it's a cylinder with a radius of the original distance when we grabbed it, and a height determined by some limit
					float maxHeight = selectedObject.isActor ? Config::options.maxBodyHeight : Config::options.maxItemHeight;
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

					float inverseMass = min(motion->m_inertiaAndMassInv.w, Config::options.inverseMassLimit);

					if (selectedObject.isActor && selectedObj->loadedState && selectedObj->loadedState->node) {
						// For dead bodies, instead of the mass of the individual collidable we hit, use the summed mass over the entire body
						Actor *actor = DYNAMIC_CAST(selectedObj, TESObjectREFR, Actor);
						if (actor) {
							float mass = GetMass(0, false, actor);
							if (mass > 0) {
								inverseMass = 1.0f / mass;
							}
							else {
								auto actorBase = DYNAMIC_CAST(actor->baseForm, TESForm, TESActorBase);
								_WARNING("Could not get mass for actor: %s", actorBase->fullName.name.data);
							}
						}
					}

					float rampUp = min((g_currentFrameTime - grabbedTime) / Config::options.grabbedRampUpTime, 1.0f); // 0 to 1 over some number of ms
					float newMagnitude = VectorLength(deltaPos) * pow(inverseMass, Config::options.massExponent) * Config::options.hoverVelocityMultiplier * rampUp;
					newMagnitude /= havokWorldScale;

					NiPoint3 newVelocity = VectorNormalized(deltaPos) * newMagnitude;

					ApplyHavokImpulse(VM_REGISTRY, 0, selectedObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
					motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };
				}
			}
			if (state == PULLED) {
				// Pull object towards the hand
				float inverseMass = min(motion->m_inertiaAndMassInv.w, Config::options.inverseMassLimit);

				//float newMagnitude = (pow(inverseMass, Config::options.massExponent) * pullSpeed * Config::options.pullVelocityMultiplier) / havokWorldScale;
				//newMagnitude = min(newMagnitude, 12.0f); // Cap at some reasonable value
				//NiPoint3 newVelocity = VectorNormalized(-relObjPos) * newMagnitude;

				float lerp = min((g_currentFrameTime - grabbedTime) / 0.75f, 1.0f);

				NiPoint3 horizontalDelta = hkHandPos - initialGrabbedObjWorldPosition;
				horizontalDelta.z = 0;
				horizontalDelta *= lerp;

				NiPoint2 leewayPoint = { 0.5f, max(hkHandPos.z, initialGrabbedObjWorldPosition.z) + 0.5f };
				NiPoint3 coeffs = QuadraticFromPoints({ 0, initialGrabbedObjWorldPosition.z }, leewayPoint, { 1, hkHandPos.z });

				NiPoint3 desiredPos = initialGrabbedObjWorldPosition + horizontalDelta;
				desiredPos.z = coeffs.z + coeffs.y * lerp + coeffs.x * lerp * lerp;

				NiPoint3 deltaPos = desiredPos - hkObjPos;
				float newMagnitude = VectorLength(deltaPos) * pow(inverseMass, Config::options.massExponent) * Config::options.pullVelocityMultiplier;
				newMagnitude /= havokWorldScale;

				NiPoint3 newVelocity = VectorNormalized(deltaPos) * newMagnitude;

				ApplyHavokImpulse(VM_REGISTRY, 0, selectedObj, 0, 0, 1, 0); // 0 force, just to 'activate' it
				motion->m_linearVelocity = { newVelocity.x, newVelocity.y, newVelocity.z, motion->m_linearVelocity.w };

				// If close enough to hand, pick it up and go to IDLE
				if (VectorLength(relObjPos) < Config::options.handActivateDistance * havokWorldScale) {
					_MESSAGE("Equipping");

					Deselect(selectedObj, other.selectedObject);

					// Pickup the item
					Activate(VM_REGISTRY, 0, selectedObj, player, false);
					// If the item is a weapon, equip it too
					auto baseForm = selectedObj->baseForm;
					if (Config::options.equipWeapons && baseForm && baseForm->formType == kFormType_Weapon) {
						auto *weapon = DYNAMIC_CAST(baseForm, TESForm, TESObjectWEAP);
						if (weapon) {
							papyrusActor::EquipItemEx(player, weapon, 1, false, false);
						}
					}
				}
			}
		}
		else {
			// Grabbed object doesn't exist - go back to IDLE
			state = IDLE;
		}
	}

	prevGrabbedObj = selectedObj;
	prevHandPosRoomspace = handPosRoomspace;
	prevHandSpeedInSpellDirection = handSpeedInSpellDirection;
	prevHandDirection = castDirection;
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
				if (pc && !pc->actorState.IsWeaponDrawn()) {
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
				if (pc && !pc->actorState.IsWeaponDrawn()) {
					pc->DrawSheatheWeapon(true);
				}
			}
		}
	}
}


bool Grabber::IsObjectPullable()
{
	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
		return (selectedObj->baseForm && selectedObj->baseForm->formType != kFormType_MovableStatic && !selectedObject.isActor);
	}
	return false;
}


bool Grabber::HasExclusiveObject() const
{
	return state == GRABBED || state == PULLED || state == SELECTION_LOCKED;
}


bool Grabber::ShouldDisplayRollover()
{
	if (state != GRABBED && state != PULLED && state != SELECTION_LOCKED) return false;

	return IsObjectPullable();
}


void Grabber::SetupRollover(NiAVObject *rolloverNode, bool isLeftHanded)
{
	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
		// Give the hud with info about the object you're floating

		// First, change rotation/position/scale of the hud prompt
		rolloverNode->m_localTransform.pos = rolloverOffset;
		rolloverNode->m_localTransform.rot = rolloverRotation;
		rolloverNode->m_localTransform.scale = rolloverScale;

		// Now set all the places I could find that get set to the handle of the pointed at object usually
		UInt32 *selectedHandles = *SELECTED_HANDLES;
		if (selectedHandles) {
			if (isLeftHanded) {
				selectedHandles[1] = selectedObject.handle;
				selectedHandles[4] = selectedObject.handle;
				selectedHandles[7] = selectedObject.handle;
			}
			else {
				selectedHandles[2] = selectedObject.handle;
				selectedHandles[5] = selectedObject.handle;
				selectedHandles[8] = selectedObject.handle;
			}
		}
	}
}
