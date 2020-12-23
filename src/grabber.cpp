#include <numeric>

#define _USE_MATH_DEFINES
#include <math.h>

#include "skse64/GameRTTI.h"
#include "skse64/PapyrusActor.h"
#include "skse64/NiGeometry.h"
#include "skse64/GameExtraData.h"

#include "grabber.h"
#include "offsets.h"
#include "utils.h"
#include "config.h"
#include "menu_checker.h"
#include "effects.h"
#include "math_utils.h"
#include "vrikinterface001.h"
#include "finger_curves.h"

#include <Physics/Collide/Query/CastUtil/hkpLinearCastInput.h>
#include <Physics/Collide/Query/CastUtil/hkpWorldRayCastInput.h>
#include <Physics/Collide/Agent3/Machine/Nn/hkpLinkedCollidable.h>


BSFixedString hmdNodeStr("HmdNode");

// Gets callbacks from havok linear cast
CdPointCollector cdPointCollector;
hkpLinearCastInput linearCastInput;
RayHitCollector rayHitCollector;
AllRayHitCollector allRayHitCollector;
CdBodyPairCollector pairCollector;
// 'ItemPicker' collision layer; player collision group
// Why ItemPicker? It ignores stuff like weapons the player is holding
//static hkpWorldRayCastInput rayCastInput(0x02420028);
hkpWorldRayCastInput rayCastInput;


UInt32 priorities[] = {
	3, // head
	2, // hair
	9, // body
	5, // hands
	6, // forearms
	1, // amulet
	4, // ring
	7, // feet
	8, // calves
	10, // shield
	13, // tail - seems to be used by cloaks
	11, // longhair
	0, // circlet
	12, // ears
	84, // 14
	85, // 15
	14, // 16 - seems to be used by cloaks as well
	86, // 17
	87, // 18
	88, // 19
	89, // decapitatehead
	90, // decapitate
	91, // 22
	92, // 23
	93, // 24
	94, // 25
	95, // 26
	96, // 27
	97, // 28
	98, // 29
	99, // 30
	100  // fx01
};
bool CompareBipedIndices(int i1, int i2)
{
	// Return true if second index beats first index
	return priorities[i2] < priorities[i1];
}


// Copied from PapyrusActor.cpp
class MatchByForm : public FormMatcher
{
	TESForm * m_form;
public:
	MatchByForm(TESForm * form) : m_form(form) {}

	bool Matches(TESForm* pForm) const { return m_form == pForm; }
};


void HapticsManager::TriggerHapticPulse(float duration)
{
	if (g_openVR && *g_openVR) {
		BSOpenVR *openVR = *g_openVR;
		openVR->TriggerHapticPulse(hand, duration);
	}
}

void HapticsManager::QueueHapticEvent(float startStrength, float endStrength, float duration)
{
	HapticsManager::HapticEvent hapticEvent;
	hapticEvent.startStrength = startStrength;
	hapticEvent.endStrength = endStrength;
	hapticEvent.duration = duration;
	hapticEvent.startTime = g_currentFrameTime;

	events.push_back(hapticEvent);
}

void HapticsManager::QueueHapticPulse(float strength)
{
	HapticEvent evnt;
	evnt.startStrength = strength;
	evnt.endStrength = strength;
	evnt.duration = 0;
	evnt.startTime = g_currentFrameTime;

	events.push_back(evnt);
}

void HapticsManager::Update()
{
	size_t numEvents = events.size();
	if (numEvents > 0) {
		// Just play the last event that was added
		HapticEvent &lastEvent = events[numEvents - 1];

		float strength;
		if (lastEvent.duration == 0) {
			strength = lastEvent.startStrength;
		}
		else {
			// Simple lerp from start to end strength over duration
			double elapsedTime = g_currentFrameTime - lastEvent.startTime;
			strength = lerp(lastEvent.startStrength, lastEvent.endStrength, min(1.0f, elapsedTime / lastEvent.duration));
		}
		TriggerHapticPulse(strength);

		// Cleanup events that are past their duration
		auto end = std::remove_if(events.begin(), events.end(),
			[](HapticEvent &evnt) { return g_currentFrameTime - evnt.startTime > evnt.duration; }
		);
		events.erase(end, events.end());
	}
}


void Grabber::Select(TESObjectREFR *obj)
{
	selectedObject.handle = GetOrCreateRefrHandle(obj);

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
}


void Grabber::Deselect()
{
	std::scoped_lock lock(deselectLock);

	selectedObject.handle = *g_invalidRefHandle;
	selectedObject.collidable = nullptr;
	selectedObject.rigidBody = nullptr;
	selectedObject.shaderNode = nullptr;
	selectedObject.hitNode = nullptr;
	selectedObject.hitForm = nullptr;

	state = State::Idle;
}


void Grabber::PlaySelectionEffect(UInt32 objHandle, NiAVObject *node)
{
	if (Config::options.disableShaders) return;

	NiPointer<TESObjectREFR> obj;
	if (LookupREFRByHandle(objHandle, obj)) {
		TESEffectShader *shader;
		if (CALL_MEMBER_FN(obj, IsOffLimits)()) {
			shader = itemSelectedShaderOffLimits;
		}
		else {
			shader = itemSelectedShader;
		}
		PlayShader(objHandle, node, shader);
	}
}


void Grabber::StopSelectionEffect(UInt32 objHandle, NiAVObject *node)
{
	if (Config::options.disableShaders) return;

	NiPointer<TESObjectREFR> obj;
	if (LookupREFRByHandle(objHandle, obj)) {
		TESEffectShader *shader;
		if (CALL_MEMBER_FN(obj, IsOffLimits)()) {
			shader = itemSelectedShaderOffLimits;
		}
		else {
			shader = itemSelectedShader;
		}
		StopShader(objHandle, node, shader);
	}
}


void Grabber::ResetNearbyDamping()
{
	if (nearbyBodies.size() > 0) {
		for (NiPointer<bhkRigidBody> body : nearbyBodies) {
			if (nearbyBodyMap.count(body) != 0) {
				auto[linearDamping, angularDamping] = nearbyBodyMap[body];
				body->hkBody->m_motion.m_motionState.m_linearDamping = linearDamping;
				body->hkBody->m_motion.m_motionState.m_angularDamping = angularDamping;
			}
		}

		nearbyBodies.clear();
		nearbyBodyMap.clear();
	}
}


std::unordered_set<bhkRigidBody *> _nearbyBodies; // prevent duplicates due to multiple contact points
void Grabber::FindBodiesToFreeze(bhkWorld &world)
{
	nearbyBodies.clear();
	nearbyBodyMap.clear();
	_nearbyBodies.clear();

	world.worldLock.LockForRead();

	hkpWorld_GetClosestPoints(world.world, selectedObject.collidable, world.world->m_collisionInput, &cdPointCollector);

	// Process result of cast
	float closestDistance = (std::numeric_limits<float>::max)();
	for (auto &pair : cdPointCollector.m_hits) {
		auto collidable = static_cast<hkpCollidable *>(pair.first);
		hkpRigidBody *rigidBody = hkpGetRigidBody(collidable);
		if (!rigidBody || !rigidBody->m_userData) {
			continue; // No rigidbody -> no movement :/
		}
		bhkRigidBody *bRigidBody = (bhkRigidBody *)rigidBody->m_userData;
		if (bRigidBody && IsAllowedCollidable(collidable)) {
			hkContactPoint &contactPoint = pair.second;
			if (contactPoint.getDistance() < Config::options.nearbyGrabBodyRadius) {
				hkpMotion &motion = rigidBody->m_motion;

				_MESSAGE("linear: %.3f\tangular: %.3f", VectorLength(HkVectorToNiPoint(motion.m_linearVelocity)), VectorLength(HkVectorToNiPoint(motion.m_angularVelocity)));

				if (VectorLength(HkVectorToNiPoint(motion.m_linearVelocity)) < Config::options.nearbyGrabMaxLinearVelocity &&
					VectorLength(HkVectorToNiPoint(motion.m_angularVelocity)) < Config::options.nearbyGrabMaxAngularVelocity) {
					if (_nearbyBodies.count(bRigidBody) == 0) {
						_nearbyBodies.insert(bRigidBody);
						nearbyBodies.push_back(bRigidBody);
						nearbyBodyMap[bRigidBody] = { motion.m_motionState.m_linearDamping, motion.m_motionState.m_angularDamping };
						motion.m_motionState.m_linearDamping = hkHalf(Config::options.nearbyGrabLinearDamping);
						motion.m_motionState.m_angularDamping = hkHalf(Config::options.nearbyGrabAngularDamping);
					}
				}
			}
		}
	}

	world.worldLock.UnlockRead();
}


bool Grabber::FindCloseObject(bhkWorld *world, bool allowGrab, const Grabber &other, NiPoint3 &hkPalmNodePos, NiPoint3 &castDirection, bhkSimpleShapePhantom *sphere,
	NiPointer<TESObjectREFR> *closestObj, NiPointer<bhkRigidBody> *closestRigidBody, hkContactPoint *closestPoint)
{
	bool isSomethingSelected = false;

	auto sphereShape = (hkpConvexShape *)sphere->phantom->m_collidable.m_shape;
	// Save sphere properties so we can change them and restore them later
	UInt32 filterInfoBefore = sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo;
	float radiusBefore = sphereShape->getRadius();
	hkVector4 translationBefore = sphere->phantom->m_motionState.getTransform().getTranslation();

	sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = 0x2C;
	sphereShape->m_radius = Config::options.nearCastRadius;
	sphere->phantom->m_motionState.m_transform.m_translation = NiPointToHkVector(hkPalmNodePos);

	NiPoint3 targetPos = hkPalmNodePos + castDirection * Config::options.nearCastDistance;
	linearCastInput.m_to = NiPointToHkVector(targetPos);
	cdPointCollector.reset();

	world->worldLock.LockForRead();
	hkpWorld_LinearCast(world->world, &sphere->phantom->m_collidable, &linearCastInput, &cdPointCollector, nullptr);

	// Process result of cast
	float closestDistance = (std::numeric_limits<float>::max)();
	for (auto pair : cdPointCollector.m_hits) {
		auto collidable = static_cast<hkpCollidable *>(pair.first);
		if (!allowGrab || (collidable == other.selectedObject.collidable && other.HasExclusiveObject())) {
			continue;
		}
		hkpRigidBody *rigidBody = hkpGetRigidBody(collidable);
		if (!rigidBody || !rigidBody->m_userData) {
			continue; // No rigidbody -> no movement :/
		}
		bhkRigidBody *bRigidBody = (bhkRigidBody *)rigidBody->m_userData;
		NiPointer<TESObjectREFR> ref = FindCollidableRef(collidable);
		if (ref && ref != *g_thePlayer) {
			if (IsAllowedCollidable(collidable) || ref->baseForm && ref->baseForm->formType == kFormType_Projectile) {
				if (ref->baseForm->formType == kFormType_Projectile) {
					auto impactData = *(void **)((UInt64)ref.m_pObject + 0x98);
					if (!impactData) {
						// Only grab projectiles that are not mid flight
						continue;
					}
				}
				// Get distance from the hit on the collidable to the ray
				NiPoint3 hit = HkVectorToNiPoint(pair.second.getPosition());
				NiPoint3 handToHit = hit - hkPalmNodePos;
				NiPoint3 handToHitAlongRay = castDirection * DotProduct(handToHit, castDirection); // project above vector onto ray
				float dist = VectorLength(handToHit - handToHitAlongRay); // distance from hit location to closest point on the ray
				if (dist < closestDistance) {
					*closestObj = ref;
					*closestRigidBody = bRigidBody;
					*closestPoint = pair.second;
					closestDistance = dist;
					isSomethingSelected = true;
				}
			}
		}
	}

	world->worldLock.UnlockRead();

	sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = filterInfoBefore;
	sphereShape->m_radius = radiusBefore;
	sphere->phantom->m_motionState.m_transform.m_translation = translationBefore;

	return isSomethingSelected;
}


void Grabber::PlayPhysicsSound(const NiPoint3 &location, bool loud)
{
	static UInt32 stoneMaterialId = 0xdf02f237;

	BGSSoundDescriptorForm *sound = nullptr;
	// Try and get the sound that plays when the object hits stone first, as the grab sound
	if (selectedObject.collidable->m_shape && selectedObject.collidable->m_shape->m_userData) {
		auto shape = (bhkShape *)selectedObject.collidable->m_shape->m_userData;
		if (shape) {
			UInt32 materialId = shape->materialId;

			// Handle MOPP shape, as it doesn't have a material on the MOPP shape itself, only the child collection shape...
			auto moppShape = DYNAMIC_CAST(shape->shape, hkpShape, hkpMoppBvTreeShape);
			if (moppShape) {
				const hkpShape *childShape = moppShape->getChild();
				if (childShape) {
					auto bChildShape = (bhkShape *)childShape->m_userData;
					if (bChildShape) {
						materialId = bChildShape->materialId;
					}
				}
			}

			BGSMaterialType *material = GetMaterialType(materialId);
			BGSMaterialType *stoneMaterial = GetMaterialType(stoneMaterialId);
			if (material && material->impactDataSet && stoneMaterial) {
				auto impactDataSet = DYNAMIC_CAST(material->impactDataSet, TESForm, BGSImpactDataSet);
				if (impactDataSet) {
					BGSImpactData *impactData = BGSImpactDataSet_GetImpactData(impactDataSet, stoneMaterial);
					if (impactData) {
						// [0] is quieter sound, [1] is louder sound
						int desiredIndex = (int)loud;
						int alternateIndex = (int)!loud;
						sound = impactData->sounds[desiredIndex];
						if (!sound) {
							sound = impactData->sounds[alternateIndex];
						}
					}
				}
			}
		}
	}
	if (!sound) {
		// Failed to get the physics sound, just use the generic pickup sound instead
		static RelocPtr<BGSDefaultObjectManager> defaultObjectManager(0x01F81D90); // The SKSE one is broken, it's a RelocPtr to a RelocPtr<BGSDefaultObjectManager*>
		TESForm *defaultPickupSound = defaultObjectManager->objects[113]; // kPickupSoundGeneric
		if (defaultPickupSound) {
			sound = DYNAMIC_CAST(defaultPickupSound, TESForm, BGSSoundDescriptorForm);
		}
	}
	if (sound) {
		PlaySoundAtNode(sound, nullptr, location);
	}
}


void Grabber::TransitionHeld(Grabber &other, bhkWorld &world, const NiPoint3 &hkPalmNodePos, const NiPoint3 &castDirection, const NiPoint3 &closestPoint, float havokWorldScale, const NiAVObject *handNode, TESObjectREFR *selectedObj)
{
	NiAVObject *n = FindCollidableNode(selectedObject.collidable);
	if (n) {
		StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

		NiPoint3 palmPos = hkPalmNodePos / havokWorldScale;

		PlayPhysicsSound(palmPos, Config::options.useLoudSoundGrab);

		float mass = NiAVObject_GetMass(n, 0);
		float hapticStrength = min(1.0f, Config::options.grabBaseHapticStrength + Config::options.grabProportionalHapticStrength * max(0.0f, powf(mass, Config::options.grabHapticMassExponent)));
		haptics.QueueHapticEvent(hapticStrength, 0, Config::options.grabHapticFadeTime);

		grabbedTime = g_currentFrameTime;
		rolloverDisplayTime = g_currentFrameTime;

		NiPoint3 ptPos = closestPoint;
		//NiPoint3 normal = HkVectorToNiPoint(closestPoint.m_separatingNormal); // vec from sphere center to point

		// Cancel a collision reset from pulling if we're grabbing the object
		if (pulledObject.handle == selectedObject.handle) {
			EndPull();
		}
		else if (other.pulledObject.handle == selectedObject.handle) {
			other.EndPull();
		}

		bool usePhysicsBasedGrab = DoesNodeHaveConstraint(selectedObj->loadedState->node, n) || IsSkinnedToNode(selectedObj->loadedState->node, n);
		if (selectedObject.isActor || usePhysicsBasedGrab || (selectedObj->baseForm && selectedObj->baseForm->formType == kFormType_Ammo)) {
			// Ragdolls, arrows (their collision gets offset for some reason when keyframed) and objects with constraints (books, skulls with jaws, wagons with wheels, etc. - physics goes crazy when keyframed) use physics based motion

			if (!selectedObject.isActor) {
				CollisionInfo::SetCollisionInfoForAllCollisionInRefr(selectedObj, playerCollisionGroup, collisionMapState);
			}

			// Use havok object pos / rot since we set that while holding it, and it can be slightly off from the ninode pos
			NiPoint3 centerOfMass = HkVectorToNiPoint(selectedObject.rigidBody->hkBody->m_motion.m_motionState.m_transform.m_translation);
			initialGrabbedObjWorldPosition = centerOfMass;

			NiPoint3 ptToCenter = centerOfMass - ptPos; // in hk coords
			NiPoint3 desiredPos = (hkPalmNodePos + ptToCenter) / havokWorldScale; // in skyrim coords
			NiTransform desiredTransform = n->m_worldTransform;
			desiredTransform.pos = desiredPos;
			HkMatrixToNiMatrix(selectedObject.rigidBody->hkBody->m_motion.m_motionState.m_transform.m_rotation, desiredTransform.rot);
			NiTransform inverseHand;
			handNode->m_worldTransform.Invert(inverseHand);
			desiredObjTransformHandSpace = inverseHand * desiredTransform;

			state = State::HeldBody;
		}
		else {
			FindBodiesToFreeze(world);

			CollisionInfo::SetCollisionInfoForAllCollisionInRefr(selectedObj, playerCollisionGroup, collisionMapState);

			selectedObject.savedMotionType = selectedObject.rigidBody->hkBody->m_motion.m_type;
			selectedObject.savedQuality = selectedObject.collidable->m_broadPhaseHandle.m_objectQualityType;

			// Set motion type before trying to set obj transform
			bhkRigidBody_setMotionType(selectedObject.rigidBody, hkpMotion::MotionType::MOTION_KEYFRAMED, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY);


			initialGrabbedObjWorldPosition = n->m_worldTransform.pos;

			NiPoint3 triPos, triNormal;
			float closestDist = (std::numeric_limits<float>::max)();
			double t = GetTime();
			bool success = GetClosestPointOnGraphicsGeometryToLine(selectedObj->loadedState->node, palmPos, castDirection, &triPos, &triNormal, &closestDist);

			NiTransform desiredTransform = n->m_worldTransform;

			if (success) {
				// We've got a point on the graphics geometry
				ptPos = triPos * havokWorldScale;

				PlayerCharacter *player = *g_thePlayer;

				NiPoint3 fingerNormalsWorldspace[5];
				NiPoint3 fingerZeroAngleVecsWorldspace[5];
				for (int i = 0; i < 5; i++) {
					NiPoint3 normalHandspace = g_fingerNormals[i];
					NiPoint3 zeroAngleVecHandspace = g_fingerZeroAngleVecs[i];
					if (isLeft) {
						// x axis is flipped for left hand
						zeroAngleVecHandspace.x *= -1;

						// Flip the entire vector, then flip the x-axis. Equivalent: flip y/z
						normalHandspace.y *= -1;
						normalHandspace.z *= -1;
					}
					fingerNormalsWorldspace[i] = VectorNormalized(handNode->m_worldTransform.rot * normalHandspace);
					fingerZeroAngleVecsWorldspace[i] = VectorNormalized(handNode->m_worldTransform.rot * zeroAngleVecHandspace);
				}

				auto FingerCheck = [this, player, handNode, palmPos, fingerNormalsWorldspace, fingerZeroAngleVecsWorldspace]
				(TESObjectREFR *refr, int fingerIndex) -> float
				{
					NiAVObject *startFinger = player->GetNiRootNode(1)->GetObjectByName(&fingerNodeNames[fingerIndex][0].data);
					NiAVObject *midFinger = player->GetNiRootNode(1)->GetObjectByName(&fingerNodeNames[fingerIndex][1].data);
					NiAVObject *endFinger = player->GetNiRootNode(1)->GetObjectByName(&fingerNodeNames[fingerIndex][2].data);

					if (startFinger && midFinger && endFinger) {
						NiPoint3 zeroAngleVectorWorldspace = fingerZeroAngleVecsWorldspace[fingerIndex];
						NiPoint3 normalWorldspace = fingerNormalsWorldspace[fingerIndex];

						NiPoint3 startFingerPos = startFinger->m_worldTransform.pos;
						NiPoint3 midFingerPos = midFinger->m_worldTransform.pos;
						NiPoint3 endFingerPos = endFinger->m_worldTransform.pos;

						_MESSAGE("%d", fingerIndex);

						float curveValOrAngle; // If negative, it's an angle. Otherwise curveVal
						bool intersects = GetIntersections(refr->loadedState->node, fingerIndex, startFingerPos, midFingerPos, endFingerPos, normalWorldspace, zeroAngleVectorWorldspace,
							&curveValOrAngle);
						if (intersects) {
							return curveValOrAngle;
						}
						else {
							// No finger intersection, so just close it completely
							return 0.0f; // 0 == closed
						}
					}
					// Couldn't get the fingers... that's a problem
					_ERROR("Could not get finger %d", fingerIndex);
					return 0;
				};

				NiTransform originalTransform = n->m_worldTransform;

				// Update transform to snap to the hand
				desiredTransform = n->m_worldTransform;
				desiredTransform.pos += palmPos - triPos;
				UpdateKeyframedNodeTransform(n, desiredTransform);

				std::array<float, 5> fingerData;
				for (int i = 0; i < fingerData.size(); i++) {
					fingerData[i] = FingerCheck(selectedObj, i);
				}

				// Reset to original transform
				UpdateKeyframedNodeTransform(n, originalTransform);

				if (g_vrikInterface) {
					std::array<float, 5> fingerRanges;
					for (int i = 0; i < fingerRanges.size(); i++) {
						float curveVal = fingerData[i];

						if (curveVal < 0) {
							// It's a negative angle - just open the hand
							_MESSAGE("%d angle: %.2f", i, curveVal);
							fingerRanges[i] = 1.0f;
						}
						else {
							// Positive => it's a curve val
							_MESSAGE("%d curve val: %.2f", i, curveVal);
							fingerRanges[i] = curveVal;
						}

						fingerRanges[i] = max(0.2f, fingerRanges[i]); // some min value to not overcurl the finger
					}

					g_vrikInterface->setFingerRange(isLeft, fingerRanges[0], fingerRanges[0], fingerRanges[1], fingerRanges[1], fingerRanges[2], fingerRanges[2], fingerRanges[3], fingerRanges[3], fingerRanges[4], fingerRanges[4]);
					//g_vrikInterface->setFingerRange(isLeft, fingerRanges[0], 1, fingerRanges[1], 1, fingerRanges[2], 1, fingerRanges[3], 1, fingerRanges[4], 1);
				}

				_MESSAGE("Geometry processing time: %.3f ms", (GetTime() - t) * 1000);				
			}
			else {
				// We've got a point on the collision geometry
				NiPoint3 centerOfMass = n->m_worldTransform.pos * havokWorldScale;
				NiPoint3 ptToCenter = centerOfMass - ptPos; // in hk coords

				NiPoint3 desiredPos = (hkPalmNodePos + ptToCenter) / havokWorldScale; // in skyrim coords
				desiredTransform.pos = desiredPos;
			}

			// Use ninode pos / rot since we set that while holding it
			NiTransform inverseHand;
			handNode->m_worldTransform.Invert(inverseHand);
			desiredObjTransformHandSpace = inverseHand * desiredTransform;

			state = State::HeldInit;
		}
	}
}


void Grabber::PoseUpdate(Grabber &other, bool allowGrab, NiNode *playerWorldNode)
{
	//_MESSAGE("%s:, pose update", name);
	forceInput = false; // 'Consume' the forceinput event

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

	NiPoint3 palmVectorHandspace = Config::options.palmVector;
	if (isLeft) palmVectorHandspace.x *= -1;
	NiPoint3 palmVector = VectorNormalized(handNode->m_worldTransform.rot * palmVectorHandspace);

	NiPoint3 pointingVectorHandspace = Config::options.pointingVector;
	if (isLeft) pointingVectorHandspace.x *= -1;
	NiPoint3 pointingVector = VectorNormalized(handNode->m_worldTransform.rot * pointingVectorHandspace);

	NiAVObject *hmdNode = playerWorldNode->GetObjectByName(&hmdNodeStr.data);
	if (!hmdNode)
		return;

	NiPoint3 hmdPos = hmdNode->m_worldTransform.pos;

	// Skyrim coords: +x: right vector, +y: forward vector, +z: up vector
	NiPoint3 hmdForward = { hmdNode->m_worldTransform.rot.data[0][1], hmdNode->m_worldTransform.rot.data[1][1], hmdNode->m_worldTransform.rot.data[2][1] };

	NiAVObject *wandNode = playerWorldNode->GetObjectByName(&wandNodeName.data);
	if (!wandNode)
		return;

	static BSFixedString comName("NPC COM [COM ]");
	NiAVObject *comNode = player->GetNiRootNode(0)->GetObjectByName(&comName.data);
	if (!comNode) {
		_MESSAGE("No COM [COM ] node on player");
		return;
	}
	if (!comNode->unk040) {
		_MESSAGE("COM node has no collision object");
		return;
	}

	{
		auto comRigidBody = GetRigidBody(comNode);
		if (comRigidBody) {
			playerCollisionGroup = comRigidBody->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 16;
		}
	}

	haptics.Update();

	bhkWorld *world = GetWorld(cell);
	if (!world) {
		_MESSAGE("Could not get havok world from player cell");
		return;
	}

	float havokWorldScale = *g_havokWorldScale;


	//if (!isLeft) {
	//	UpdateGenerateFingerCurve(handNodeName, fingerNodeNames);
	//}


	if (world->world != handCollBody->m_world) {
		if (handCollBody->m_world) {
			// If exists, remove the hand entity from the previous world
			_MESSAGE("%s: Removing collision for hand", name);

			bhkWorld *oldWorld = (bhkWorld *)static_cast<ahkpWorld *>(handCollBody->m_world)->m_userData;
			oldWorld->worldLock.LockForWrite();
			hkBool ret;
			hkpWorld_RemoveEntity(handCollBody->m_world, &ret, handCollBody);
			oldWorld->worldLock.UnlockWrite();
		}

		_MESSAGE("%s: Adding collision for hand", name);

		world->worldLock.LockForWrite();

		// Create our own layer in the first ununsed vanilla layer (56)
		bhkCollisionFilter *worldFilter = (bhkCollisionFilter *)world->world->m_collisionFilter;
		UInt64 bitfield = worldFilter->layerBitfields[5]; // copy of L_WEAPON layer bitfield

		bitfield |= ((UInt64)1 << 56); // collide with ourselves
		bitfield &= ~((UInt64)1 << 0x1e); // remove collision with character controllers
		worldFilter->layerBitfields[56] = bitfield;
		worldFilter->layerNames[56] = BSFixedString("L_HANDCOLLISION");
		// Set whether other layers should collide with our new layer
		for (int i = 0; i < 56; i++) {
			if ((bitfield >> i) & 1) {
				worldFilter->layerBitfields[i] |= ((UInt64)1 << 56);
			}
		}

		UInt8 ragdollBits = (UInt8)(isLeft ? CollisionInfo::RagdollLayer::LeftHand : CollisionInfo::RagdollLayer::RightHand);

		UInt32 filterInfo = ((UInt32)playerCollisionGroup << 16) | 56; // player group, our custom layer
		filterInfo |= (1 << 15); // set bit 15 to collide with same group that also has bit 15
		filterInfo |= (ragdollBits << 8);

		// Add collision object for the hand
		hkpBoxShape_ctor(handCollShape, NiPointToHkVector(Config::options.handCollisionBoxHalfExtents), Config::options.handCollisionBoxRadius);
		hkpRigidBodyCinfo_ctor(handCollCInfo); // initialize with defaults
		handCollCInfo->m_shape = handCollShape;
		handCollCInfo->m_collisionFilterInfo = filterInfo;
		handCollCInfo->m_motionType = hkpMotion::MotionType::MOTION_KEYFRAMED;
		handCollCInfo->m_enableDeactivation = false;
		handCollCInfo->m_solverDeactivation = hkpRigidBodyCinfo::SolverDeactivation::SOLVER_DEACTIVATION_OFF;

		hkpRigidBody_ctor(handCollBody, handCollCInfo);

		hkpWorld_AddEntity(world->world, handCollBody, HK_ENTITY_ACTIVATION_DO_ACTIVATE);

		world->worldLock.UnlockWrite();
	}

	// Set collision group for the hand collision every frame. The player collision changes sometimes, e.g. when getting on/off a horse
	handCollBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= (0x0000ffff); // zero out collision group
	handCollBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt32)playerCollisionGroup << 16); // set collision group to player group

	// Put our hand collision where we want it
	NiPoint3 handCollisionBoxOffset = Config::options.handCollisionBoxOffset;
	if (isLeft) handCollisionBoxOffset.x *= -1;
	NiPoint3 desiredPos = (handNode->m_worldTransform * (handCollisionBoxOffset / havokWorldScale)) * havokWorldScale;
	hkRotation desiredRot;
	NiMatrixToHkMatrix(handNode->m_worldTransform.rot, desiredRot);
	hkQuaternion desiredQuat;
	desiredQuat.setFromRotationSimd(desiredRot);
	hkpKeyFrameUtility_applyHardKeyFrame(NiPointToHkVector(desiredPos), desiredQuat, 1.0f / *g_deltaTime, handCollBody);

	// Update velocities to this frame
	NiPoint3 playerVelocityWorldspace = (player->pos - prevPlayerPosWorldspace) / *g_deltaTime;

	playerVelocitiesWorldspace.pop_back();
	playerVelocitiesWorldspace.push_front(playerVelocityWorldspace);

	NiPoint3 avgPlayerVelocityWorldspace = std::accumulate(playerVelocitiesWorldspace.begin(), playerVelocitiesWorldspace.end(), NiPoint3()) / playerVelocitiesWorldspace.size();

	if (g_currentFrameTime - pulledTime > pulledExpireTime) {
		EndPull();
	}

	bhkSimpleShapePhantom *sphere = *g_pickSphere;
	if (!sphere)
		return;

	NiPoint3 hkPalmNodePos = handNode->m_worldTransform * palmPosHandspace * havokWorldScale;

	auto sphereShape = (hkpConvexShape *)sphere->phantom->m_collidable.m_shape;
	// Save sphere properties so we can change them and restore them later
	UInt32 filterInfoBefore = sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo;
	float radiusBefore = sphereShape->getRadius();
	hkVector4 translationBefore = sphere->phantom->m_motionState.getTransform().getTranslation();

	if (state == State::Idle || state == State::SelectedClose || state == State::SelectedFar) {

		// See if there's something near the hand to pick up
		NiPointer<TESObjectREFR> closestObj;
		NiPointer<bhkRigidBody> closestRigidBody;
		hkContactPoint closestPoint;

		bool isSelectedNear = FindCloseObject(world, allowGrab, other, hkPalmNodePos, palmVector, sphere,
			&closestObj, &closestRigidBody, &closestPoint);

		if (!isSelectedNear) {
			// Nothing close by the hand. Check for stuff pointing from from the palm

			// Convert hand position from skyrim coords to havok coords
			NiPoint3 hkHmdPos = hmdPos * havokWorldScale;

			NiPoint3 hkTargetPos = hkPalmNodePos + pointingVector * Config::options.farCastDistance;

			NiPoint3 hitPosition = { hkTargetPos.x, hkTargetPos.y, hkTargetPos.z };

			// First, raycast in the pointing direction
			rayHitCollector.reset();
			rayCastInput.m_filterInfo = ((UInt32)playerCollisionGroup << 16) | 0x28;
			rayCastInput.m_from = NiPointToHkVector(hkPalmNodePos);
			rayCastInput.m_to = NiPointToHkVector(hkTargetPos);
			world->worldLock.LockForRead();
			hkpWorld_CastRay(world->world, &rayCastInput, &rayHitCollector);
			world->worldLock.UnlockRead();
			if (rayHitCollector.m_doesHitExist) {
				// If raycast hit, we want to linearcast only up to the ray hit location
				NiPoint3 handToTarget = hkTargetPos - hkPalmNodePos;
				hitPosition = hkPalmNodePos + (handToTarget * rayHitCollector.m_closestHitInfo.m_hitFraction);// -(VectorNormalized(handToTarget) * Config::options.castRadius);
			}

			// Now, linearcast up to the point the raycast hit, or up to the limit if it's empty space
			// 'CustomPick2' layer, to pick up projectiles, because ONLY THIS GODDAMN LAYER can collide with projectiles. This filterinfo will collide with everything.
			sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = 0x2C;
			sphereShape->m_radius = Config::options.farCastRadius;
			sphere->phantom->m_motionState.m_transform.m_translation = NiPointToHkVector(hkPalmNodePos);

			linearCastInput.m_to = NiPointToHkVector(hitPosition);
			cdPointCollector.reset();

			world->worldLock.LockForRead();
			hkpWorld_LinearCast(world->world, &sphere->phantom->m_collidable, &linearCastInput, &cdPointCollector, nullptr);

			// Process result of cast
			float closestDistance = (std::numeric_limits<float>::max)();
			for (auto pair : cdPointCollector.m_hits) {
				auto collidable = static_cast<hkpCollidable *>(pair.first);
				if (!allowGrab || (other.HasExclusiveObject() && collidable == other.selectedObject.collidable)) {
					continue;
				}
				hkpRigidBody *rigidBody = hkpGetRigidBody(collidable);
				if (!rigidBody || !rigidBody->m_userData) {
					continue; // No rigidbody -> no movement :/
				}
				bhkRigidBody *bRigidBody = (bhkRigidBody *)rigidBody->m_userData;
				NiPointer<TESObjectREFR> ref = FindCollidableRef(collidable);
				if (ref && ref != player) {
					if (IsAllowedCollidable(collidable) || ref->baseForm && ref->baseForm->formType == kFormType_Projectile) {
						if (ref->baseForm->formType == kFormType_Projectile) {
							auto impactData = *(void **)((UInt64)ref.m_pObject + 0x98);
							if (!impactData) {
								// Only grab projectiles that are not mid flight
								continue;
							}
						}
						// Get distance from the hit on the collidable to the ray
						NiPoint3 hit = HkVectorToNiPoint(pair.second.getPosition());
						NiPoint3 handToHit = hit - hkPalmNodePos;
						NiPoint3 handToHitAlongRay = pointingVector * DotProduct(handToHit, pointingVector); // project above vector onto ray
						float dist = VectorLength(handToHit - handToHitAlongRay); // distance from hit location to closest point on the ray
						if (dist < closestDistance && DotProduct(VectorNormalized(hit - hkHmdPos), hmdForward) >= Config::options.requiredCastDotProduct) {
							closestObj = ref;
							closestRigidBody = bRigidBody;
							closestPoint = pair.second;
							closestDistance = dist;
						}
					}
				}
			}

			world->worldLock.UnlockRead();

			sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = filterInfoBefore;
			sphereShape->m_radius = radiusBefore;
			sphere->phantom->m_motionState.m_transform.m_translation = translationBefore;
		}

		// Check if we should select something new. If yes, stay in SELECTED but select the new object
		bool isSelectedThisFrame = false;
		UInt32 prevSelectedHandle = selectedObject.handle;
		if (closestObj) {
			State newState = state; // We only want to set state after the collidable is updated, for threading reasons

			// We save this while the casts hit so that during fade time when the casts don't hit, we still have a point to use.
			selectedObject.point = HkVectorToNiPoint(closestPoint.getPosition());

			NiPointer<TESObjectREFR> selectedObj;
			if (!LookupREFRByHandle(selectedObject.handle, selectedObj) || closestObj != selectedObj) {
				if (selectedObj) {
					// Deselect the old thing if something else was selected
					StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);
					Deselect();
				}
				// select the new refr
				Select(closestObj);
				newState = isSelectedNear ? State::SelectedClose : State::SelectedFar;
				if (newState == State::SelectedClose) {
					if (g_vrikInterface) {
						float val = 0.9f;
						g_vrikInterface->setFingerRange(isLeft, val, val, val, val, val, val, val, val, val, val);
					}
					rolloverDisplayTime = g_currentFrameTime;
				}
				else if (newState == State::SelectedFar) {
					if (g_vrikInterface) {
						g_vrikInterface->restoreFingers(isLeft);
					}
				}
			}
			else if ((state == State::SelectedFar && isSelectedNear) || (state == State::SelectedClose && !isSelectedNear)) {
				// Same object is selected, but it has become near/far enough to switch states
				newState = isSelectedNear ? State::SelectedClose : State::SelectedFar;
				if (newState == State::SelectedClose) {
					if (g_vrikInterface) {
						float val = 0.9f;
						g_vrikInterface->setFingerRange(isLeft, val, val, val, val, val, val, val, val, val, val);
					}
					rolloverDisplayTime = g_currentFrameTime;
				}
				else if (newState == State::SelectedFar) {
					if (g_vrikInterface) {
						g_vrikInterface->restoreFingers(isLeft);
					}
				}
			}

			// Figure out which node we should be playing a shader on, and switch to that one
			NiAVObject *nodeOnWhichToPlayShader = nullptr;
			Actor *actor = DYNAMIC_CAST(closestObj, TESObjectREFR, Actor);
			bool breakStickiness = false;

			if (actor) {
				NiAVObject *hitNode = FindCollidableNode(&closestRigidBody->hkBody->m_collidable);
				if (hitNode) {
					BipedModel *biped = actor->GetBipedSmall();
					if (biped) {
						Biped *bipedData = biped->bipedData;
						if (bipedData) {
							int hitIndex = -1;
							TESForm *hitForm = nullptr;
							for (int i = 0; i < equippedWeaponSlotBase; i++) {
								// For skinned armor, the nodes are not attached to the skeleton. Find nodes the armor is skinned to and see if one of them was hit
								NiAVObject *geomNode = bipedData->unk10[i].object;
								if (geomNode) {
									TESForm *armorForm = bipedData->unk10[i].armor;
									if (armorForm) {
										// Now check if we actually hit a node the armor is skinned to
										bool isArmorHit = IsSkinnedToNode(geomNode, hitNode);
										if (isArmorHit) {
											if (hitIndex == -1 || CompareBipedIndices(hitIndex, i)) {
												hitIndex = i;
												hitForm = armorForm;
											}
										}
									}
								}
							}
							for (int i = equippedWeaponSlotBase; i < 42; i++) {
								// For equipped weapons, the nodes are attached to the skeleton. Find the nearest parent that has collision and see if it was hit
								NiAVObject *geomNode = bipedData->unk10[i].object;
								bool hasCollision = false;
								if (geomNode) {
									if (DoesNodeHaveNode(geomNode, hitNode)) {
										// We collided with the weapon, so it must not be attached to the character
										hitIndex = i;
										hitForm = bipedData->unk10[i].armor;
										break;
									}
									else {
										// Weapon is attached to the character - get the nearest parent node that does have collision
										NiAVObject *nodeWithCollision = geomNode;
										while (nodeWithCollision) {
											if (nodeWithCollision->unk040) {
												hasCollision = true;
												break;
											}
											nodeWithCollision = nodeWithCollision->m_parent;
										}
										if (hasCollision && nodeWithCollision == hitNode) {
											hitIndex = i;
											hitForm = bipedData->unk10[i].armor;
											break;
										}
									}
								}
							}
							if (hitForm) {
								// Make sure the armor we hit is actually equipped. When nothing is equipped, there can still be 'naked' armor in the biped data that's not really equipped armor.
								ExtraContainerChanges* containerChanges = static_cast<ExtraContainerChanges*>(actor->extraData.GetByType(kExtraData_ContainerChanges));
								if (containerChanges) {
									MatchByForm matcher(hitForm);
									EquipData equipData = containerChanges->FindEquipped(matcher);
									TESForm *equippedForm = equipData.pForm;
									if (equippedForm) {
										Biped::Data *hitBipedData = &bipedData->unk10[hitIndex];
										auto hitArmor = DYNAMIC_CAST(hitBipedData->armor, TESForm, TESObjectARMO);
										// If it's armor, make sure it has a name. If it doesn't, it could be FEC, or who knows...
										if (!hitArmor || *hitArmor->fullName.name.data) {
											nodeOnWhichToPlayShader = hitBipedData->object;
											selectedObject.hitForm = hitForm;
											selectedObject.hitNode = hitNode;
										}
									}
									else {
										// The form we hit is not equipped - do the same as if no form was selected
										breakStickiness = true;
									}
								}
							}
						}
					}
				}
			}

			if (nodeOnWhichToPlayShader) {
				if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
					if (nodeOnWhichToPlayShader != selectedObject.shaderNode) {
						// New node, same (or new) object
						if (selectedObject.handle == prevSelectedHandle) {
							// Stop the shader on the current reference, we've changed nodes
							StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);
						}
						else {
							// Stop the shader on the previous reference
							NiPointer<TESObjectREFR> prevSelectedObj;
							if (LookupREFRByHandle(prevSelectedHandle, prevSelectedObj)) {
								StopSelectionEffect(prevSelectedHandle, selectedObject.shaderNode);
							}
						}
						PlaySelectionEffect(selectedObject.handle, nodeOnWhichToPlayShader);
						selectedObject.shaderNode = nodeOnWhichToPlayShader;
					}
				}
			}
			else if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				// No (actor) node selected but refr is still selected

				if (selectedObject.handle != prevSelectedHandle) {
					// New refr is selected. Stopping the shader for the previous ref is done earlier.

					selectedObject.shaderNode = nullptr;
					selectedObject.hitNode = nullptr;
					selectedObject.hitForm = nullptr;

					if (!selectedObject.isActor) {
						NiAVObject *hitNode = FindCollidableNode(&closestRigidBody->hkBody->m_collidable);
						if (hitNode) {
							if (!IsSkinnedToNode(selectedObj->loadedState->node, hitNode)) {
								selectedObject.shaderNode = hitNode;
							}
						}
						PlaySelectionEffect(selectedObject.handle, selectedObject.shaderNode);
					}
				}
				else if (selectedObject.shaderNode) {
					if (selectedObject.isActor) {
						// Node was selected before (selectedObject.shaderNode), but not now (nodeOnWhichToPlayShader). Stop the shader on that node.
						if (breakStickiness) {
							StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

							//PlayShader(selectedObject.handle, nullptr, shader);

							selectedObject.shaderNode = nullptr;
							selectedObject.hitNode = nullptr;
							selectedObject.hitForm = nullptr;
						}
					}
					else {
						NiAVObject *hitNode = FindCollidableNode(&closestRigidBody->hkBody->m_collidable);
						if (hitNode != selectedObject.shaderNode) {
							// Moved nodes on the same reference
							StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

							selectedObject.shaderNode = nullptr;
							selectedObject.hitNode = nullptr;
							selectedObject.hitForm = nullptr;

							if (!IsSkinnedToNode(selectedObj->loadedState->node, hitNode)) {
								selectedObject.shaderNode = hitNode;
							}

							PlaySelectionEffect(selectedObject.handle, selectedObject.shaderNode);
						}
					}
				}
				else {
					// Same refr, no node selected before (skinned?)
					NiAVObject *hitNode = FindCollidableNode(&closestRigidBody->hkBody->m_collidable);
					if (hitNode && !IsSkinnedToNode(selectedObj->loadedState->node, hitNode)) {
						// Only replay the shader if we now have a specific node to play on
						StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

						selectedObject.hitNode = nullptr;
						selectedObject.hitForm = nullptr;
						selectedObject.shaderNode = hitNode;

						PlaySelectionEffect(selectedObject.handle, selectedObject.shaderNode);
					}
				}
			}

			// Set selected collidable no matter what, as we can have objects with more than one collidable
			selectedObject.rigidBody = closestRigidBody;
			selectedObject.collidable = &closestRigidBody->hkBody->m_collidable;

			state = newState;

			isSelectedThisFrame = true;
			lastSelectedTime = g_currentFrameTime;
		}

		if (state == State::SelectedClose || state == State::SelectedFar) {
			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				if (!isSelectedThisFrame && g_currentFrameTime - lastSelectedTime > Config::options.selectedLeewayTime) {
					// If time has run out and nothing is selected, deselect whatever is selected
					if (state == State::SelectedClose) {
						if (g_vrikInterface) {
							g_vrikInterface->restoreFingers(isLeft);
						}
					}


					//if (!isLeft) {
					//	StartGenerateFingerCurve(isLeft);
					//}


					StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);
					Deselect();
				}
				else {
					// Check if we should lock in the selection
					if (grabRequested && g_currentFrameTime - grabRequestedTime <= Config::options.triggerPressedLeewayTime) {
						if (state == State::SelectedFar) {
							hkVector4 translation = selectedObject.rigidBody->hkBody->m_motion.m_motionState.m_transform.m_translation;
							NiPoint3 hkHandPos = handPos * havokWorldScale;
							NiPoint3 hkObjPos = HkVectorToNiPoint(translation);
							NiPoint3 relObjPos = hkObjPos - hkHandPos;

							NiPoint3 forward = pointingVector;
							NiPoint3 worldUp = { 0, 0, 1 };
							NiPoint3 right = CrossProduct(forward, worldUp);
							NiPoint3 up = CrossProduct(right, forward);
							initialObjPosRaySpace = { DotProduct(relObjPos, forward), DotProduct(relObjPos, right) , DotProduct(relObjPos, up) };
							rolloverDisplayTime = g_currentFrameTime;
							initialGrabbedObjRelativePosition = relObjPos;
							initialGrabbedObjWorldPosition = hkObjPos;
							pulledPointOffset = selectedObject.point - hkObjPos;

							state = State::SelectionLocked;
						}
						else if (state == State::SelectedClose) {
							if (g_vrikInterface) {
								g_vrikInterface->restoreFingers(isLeft);
							}
							TransitionHeld(other, *world, hkPalmNodePos, palmVector, selectedObject.point, havokWorldScale, handNode, selectedObj);
						}
						// Set to false only here, so that you can hold the trigger until the cast hits something valid
						grabRequested = false;
						wasObjectGrabbed = true; // This variable is not set to false when we push/pull the object
					}
				}
			}
			else {
				// Selected object no longer exists
				if (state == State::SelectedClose) {
					if (g_vrikInterface) {
						g_vrikInterface->restoreFingers(isLeft);
					}
				}
				state = State::Idle;
			}
		}

		if (releaseRequested) {
			releaseRequested = false;
			wasObjectGrabbed = false;
		}
	}

	NiPointer<TESObjectREFR> selectedObj;
	if (state == State::SelectionLocked || state == State::PrepullItem || state == State::HeldInit || state == State::Held || state == State::HeldBody) {
		// Check if we should drop the object
		if (releaseRequested) {
			releaseRequested = false;
			wasObjectGrabbed = false;
			idleDesired = true;
		}
		if (idleDesired || !allowGrab) {
			idleDesired = false;

			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				if (state == State::SelectionLocked) {
					StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);
				}
				if (state == State::HeldInit || state == State::Held || state == State::HeldBody) {

					float largestSpeed = -1;
					int largestIndex = -1;
					for (int i = 0; i < controllerVelocities.size(); i++) {
						NiPoint3 velocity = controllerVelocities[i];
						float speed = VectorLength(velocity);
						if (speed > largestSpeed) {
							largestSpeed = speed;
							largestIndex = i;
						}
					}

					NiPoint3 velocity;
					if (largestIndex == 0) {
						// Max is the first value
						velocity = controllerVelocities[0];
					}
					else if (largestIndex == controllerVelocities.size() - 1) {
						// Max is the last value
						velocity = controllerVelocities[largestIndex];
					}
					else {
						// Regular case - avg 3 values centered at the peak
						velocity = (controllerVelocities[largestIndex - 1] + controllerVelocities[largestIndex] + controllerVelocities[largestIndex + 1]) / 3;
					}

					velocity = (avgPlayerVelocityWorldspace * *g_havokWorldScale) + velocity; // add the player velocity

					bool velocityAboveThreshold = VectorLength(velocity) > Config::options.throwVelocityThreshold;
					bool collideWithHandWhenLettingGo = !velocityAboveThreshold;

					if (state == State::HeldBody) {
						if (!selectedObject.isActor) {
							ResetCollisionInfoForAllCollisionInRefr(selectedObj, collisionMapState, nullptr, collideWithHandWhenLettingGo);
						}
					}
					if (state == State::Held || state == State::HeldInit) {
						if (g_vrikInterface) {
							g_vrikInterface->restoreFingers(isLeft);
						}

						ResetNearbyDamping();

						ResetCollisionInfoForAllCollisionInRefr(selectedObj, collisionMapState, selectedObject.collidable, collideWithHandWhenLettingGo); // skip the node we grabbed, we handle that below
						ResetCollisionInfoKeyframed(selectedObject.rigidBody, selectedObject.savedMotionType, selectedObject.savedQuality, collisionMapState, collideWithHandWhenLettingGo);
					}

					bhkRigidBody_setActivated(selectedObject.rigidBody, true);
					selectedObject.rigidBody->hkBody->m_motion.m_linearVelocity = NiPointToHkVector(velocity);

					float mass = NiAVObject_GetMass(FindCollidableNode(selectedObject.collidable), 0);
					float hapticStrength = min(1.0f, Config::options.grabBaseHapticStrength + Config::options.grabProportionalHapticStrength * max(0.0f, powf(mass, Config::options.grabHapticMassExponent)));
					haptics.QueueHapticEvent(hapticStrength, 0, Config::options.grabHapticFadeTime);

					PlayPhysicsSound(hkPalmNodePos / havokWorldScale, Config::options.useLoudSoundDrop);
				}
			}

			Deselect();
		}
	}

	if (state == State::SelectionLocked) {
		if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
			hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;

			auto TransitionPulled = [this, &other, motion, selectedObj, handNode]()
			{
				StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

				pulledTime = g_currentFrameTime;

				if (selectedObject.hitForm) {
					// Trying to pull armor off a body

					state = State::Idle; // If things don't go right, fallback to idle

					Actor *actor = DYNAMIC_CAST(selectedObj, TESObjectREFR, Actor);
					if (actor) {
						// drop the armor
						ExtraContainerChanges* containerChanges = static_cast<ExtraContainerChanges*>(actor->extraData.GetByType(kExtraData_ContainerChanges));
						if (containerChanges) {
							MatchByForm matcher(selectedObject.hitForm);
							EquipData equipData = containerChanges->FindEquipped(matcher);
							TESForm *itemForm = equipData.pForm;
							BaseExtraList *armorExtraData = equipData.pExtraData;
							if (itemForm) {
								TESBoundObject *item = DYNAMIC_CAST(itemForm, TESForm, TESBoundObject);
								if (item) {
									// pump armor form / extra data into actor->RemoveItem (vfunc 0x56)
									// Actor::RemoveItem is at 0x607F60

									UInt64 *vtbl = *((UInt64 **)actor);
									UInt32 droppedObjHandle = *g_invalidRefHandle;
									if (DYNAMIC_CAST(item, TESBoundObject, TESObjectWEAP)) {
										// For dropped weapons, make the drop pos / rot equal to where it was before
										NiPoint3 dropLoc = selectedObject.hitNode->m_worldTransform.pos;
										NiPoint3 dropRot = MatrixToEuler(selectedObject.hitNode->m_worldTransform.rot);
										((_RemoveItem)(vtbl[0x56]))(actor, &droppedObjHandle, item, 1, 3, armorExtraData, nullptr, &dropLoc, &dropRot);
									}
									else {
										NiPoint3 dirObjToHand = VectorNormalized(handNode->m_worldTransform.pos - selectedObject.hitNode->m_worldTransform.pos);
										NiPoint3 dropLoc = selectedObject.hitNode->m_worldTransform.pos + NiPoint3(0, 0, 20); // move it up a bit to not collide with the ragdoll too much
										((_RemoveItem)(vtbl[0x56]))(actor, &droppedObjHandle, item, 1, 3, armorExtraData, nullptr, &dropLoc, nullptr);
									}

									NiPointer<TESObjectREFR> droppedObj;
									if (LookupREFRByHandle(droppedObjHandle, droppedObj)) {
										std::scoped_lock lock(deselectLock);

										selectedObject.handle = droppedObjHandle;
										selectedObject.collidable = nullptr;
										selectedObject.rigidBody = nullptr;

										state = State::PrepullItem;
									}
								}
							}
						}
					}
				}
				else {
					// Not trying to pull armor off a body

					if (pulledObject.handle != selectedObject.handle) {
						EndPull(); // Cancel an existing pulled collision reset if we're pulling something new

						if (other.pulledObject.handle == selectedObject.handle) {
							other.EndPull();
						}

						pulledObject.handle = selectedObject.handle;
						pulledObject.rigidBody = selectedObject.rigidBody;
						pulledObject.savedAngularDamping = motion->m_motionState.m_angularDamping;
						motion->m_motionState.m_angularDamping = hkHalf(Config::options.pulledAngularDamping);

						if (selectedObject.isImpactedProjectile) { // It's an embedded projectile, i.e. stuck in a wall etc.
							auto rigidBody = GetRigidBody(selectedObj->loadedState->node);
							if (rigidBody) {
								// Do not use grabbedObject.collidable here, as sometimes we end up grabbing the phantom shape of the projectile instead of the 3D one
								auto collidable = &rigidBody->hkBody->m_collidable;
								// Projectiles do not interact with collision usually. We need to change the filter to make them interact.
								collidable->m_broadPhaseHandle.m_collisionFilterInfo = (((UInt32)playerCollisionGroup) << 16) | 5; // player collision group, 'weapon' collision layer
								// Projectiles have 'Fixed' motion type by default, making them unmovable
								bhkRigidBody_setMotionType(rigidBody, hkpMotion::MotionType::MOTION_DYNAMIC, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);
							}
						}

						CollisionInfo::SetCollisionInfoForAllCollisionInRefr(selectedObj, playerCollisionGroup, CollisionInfo::State::Unheld);
					}

					float hapticStrength = min(1.0f, Config::options.selectionLockedBaseHapticStrength + Config::options.selectionLockedProportionalHapticStrength);
					haptics.QueueHapticEvent(hapticStrength, 0, Config::options.pullHapticFadeTime);

					NiPoint3 hkObjPos = HkVectorToNiPoint(motion->m_motionState.m_transform.m_translation);
					NiPoint3 objPoint = (hkObjPos + pulledPointOffset) * *g_inverseHavokWorldScale;
					PlayPhysicsSound(objPoint, Config::options.useLoudSoundPull);

					state = State::Pulled;
				}
			};

			bool isSelectedNear = false;

			if (g_currentFrameTime - grabRequestedTime <= Config::options.triggerPressedLeewayTime) {
				NiPointer<TESObjectREFR> closestObj;
				NiPointer<bhkRigidBody> closestRigidBody;
				hkContactPoint closestPoint;

				isSelectedNear = FindCloseObject(world, allowGrab, other, hkPalmNodePos, palmVector, sphere,
					&closestObj, &closestRigidBody, &closestPoint);

				// Allow us to go to held if we had the thing selected from a distance and it came closer within the leeway time
				if (isSelectedNear && closestRigidBody == selectedObject.rigidBody) {
					TransitionHeld(other, *world, hkPalmNodePos, palmVector, HkVectorToNiPoint(closestPoint.getPosition()), havokWorldScale, handNode, selectedObj);
				}
			}

			if (!isSelectedNear) {
				hkVector4 translation = motion->m_motionState.m_transform.m_translation;
				NiPoint3 hkObjPos = HkVectorToNiPoint(translation);
				NiPoint3 hkHandPos = handPos * havokWorldScale;
				NiPoint3 relObjPos = hkObjPos - hkHandPos;

				//float handSpeedInObjDirection = DotProduct(localVelocityWorldspace, VectorNormalized(relObjPos));
				NiPoint3 controllerVelocity = controllerVelocities[0];
				float controllerSpeedDirectionalized = DotProduct(controllerVelocity, VectorNormalized(-relObjPos));
				//if (std::string(handNodeName.data) == "NPC R Hand [RHnd]") {
				//	PrintToFile(std::to_string(VectorLength(handDirectionVelocityRoomspace)), "hand_delta_dir_r.txt");
				//	PrintToFile(std::to_string(handSpeedInObjDirection), "hand_speed_r.txt");
				//}

				bool pull = false;
				if (controllerSpeedDirectionalized > Config::options.pullSpeedThreshold) {
					if (IsObjectPullable()) {
						pull = true;
					}
				}

				if (pull) {
					TransitionPulled();
				}
				else {
					// Hold object selected while hand doesn't point too far away from original location. If far enough, deselect.
					NiPoint3 forward = pointingVector;
					NiPoint3 worldUp = { 0, 0, 1 };
					NiPoint3 right = CrossProduct(forward, worldUp);
					NiPoint3 up = CrossProduct(right, forward);
					NiPoint3 initialObjPos = forward * initialObjPosRaySpace.x + right * initialObjPosRaySpace.y + up * initialObjPosRaySpace.z;

					// Determine the position where we want the object to be
					// Essentially it's a cylinder with a radius of the original distance when we grabbed it, and a height determined by some limit
					float maxHeight = 4.0f;
					NiPoint3 horiz;
					float h;
					if (pointingVector.z <= 0.9999f) {
						float w = VectorLength(NiPoint3(initialGrabbedObjRelativePosition.x, initialGrabbedObjRelativePosition.y, 0)); // horizontal distance from hand to object
						float theta = asinf(pointingVector.z); // vertical angle of hand relative to horizontal
						float tanTheta = tanf(theta);
						h = min(w * tanTheta, maxHeight); // desired height relative to horizontal from hand
						horiz = VectorNormalized(NiPoint3(pointingVector.x, pointingVector.y, 0)) * (h / tanTheta); // desired horizontal position relative to hand
					}
					else {
						// Degenerate case
						h = maxHeight;
						horiz = { 0, 0, 0 };
					}

					NiPoint3 desiredPos = NiPoint3(horiz.x, horiz.y, h);

					NiPoint3 deltaPos = desiredPos - relObjPos;

					if (DotProduct(VectorNormalized(initialObjPos), VectorNormalized(relObjPos)) < Config::options.grabbedDotProductThreshold || abs(DotProduct(deltaPos, pointingVector)) > 1.5f) {
						//grab = true;
						idleDesired = true;
					}

					float hapticStrength = min(1.0f, Config::options.selectionLockedBaseHapticStrength + Config::options.selectionLockedProportionalHapticStrength * max(0, (controllerSpeedDirectionalized / Config::options.pullSpeedThreshold)));
					haptics.QueueHapticPulse(hapticStrength);
				}
			}
		}
		else {
			state = State::Idle;
		}
	}

	if (state == State::PrepullItem) {
		if (g_currentFrameTime - pulledTime > Config::options.pulledLootSpawnInTime) { // wait for the item to spawn in
			state = State::Idle;
		}
		else {
			if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
				// Transition to pulled with the newly spawned item

				auto rigidBody = GetRigidBody(selectedObj->loadedState->node);
				if (rigidBody) {
					selectedObject.rigidBody = rigidBody;
					selectedObject.collidable = &selectedObject.rigidBody->hkBody->m_collidable;

					// Set owner to the player so it doesn't count as stealing
					TESObjectREFR_SetActorOwner(nullptr, 0, selectedObj, player->baseForm);

					// Cancel an existing pulled collision reset
					EndPull();

					pulledObject.handle = selectedObject.handle;
					pulledObject.rigidBody = selectedObject.rigidBody;

					hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;
					pulledObject.savedAngularDamping = motion->m_motionState.m_angularDamping;
					motion->m_motionState.m_angularDamping = hkHalf(Config::options.pulledAngularDamping);
					CollisionInfo::SetCollisionInfoForAllCollisionInRefr(selectedObj, playerCollisionGroup, CollisionInfo::State::Unheld);

					pulledTime = g_currentFrameTime;

					float hapticStrength = min(1.0f, Config::options.selectionLockedBaseHapticStrength + Config::options.selectionLockedProportionalHapticStrength);
					haptics.QueueHapticEvent(hapticStrength, 0, Config::options.pullHapticFadeTime);

					NiPoint3 hkObjPos = HkVectorToNiPoint(motion->m_motionState.m_transform.m_translation);
					NiPoint3 objPoint = (hkObjPos + pulledPointOffset) * *g_inverseHavokWorldScale;
					PlayPhysicsSound(objPoint, Config::options.useLoudSoundPull);

					pulledPointOffset = { 0, 0, 0 }; // point offset doesn't make sense when we are pulling something other than what we had selected

					state = State::Pulled;
				}
			}
		}
	}

	if (state == State::Pulled) {
		if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {

			hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;
			NiPoint3 hkObjPos = HkVectorToNiPoint(motion->m_motionState.m_transform.m_translation);

			NiPoint3 objPoint = hkObjPos + pulledPointOffset;

			float distanceFactor = VectorLength(hkPalmNodePos - objPoint) / Config::options.pullDurationExtensionDistance;
			float duration = Config::options.minPullDuration + Config::options.pullDurationExtensionDuration * distanceFactor;

			pulledExpireTime = duration + 1.0f;

			// Apply a predicted velocity to reach the destination, for a few frames after pulling starts.
			// Why for a few frames? Because then if it's next to something, it has a few frames to push it out of the way instead of just flopping right away

			if (g_currentFrameTime - pulledTime <= Config::options.pulledInitTime) {
				NiPoint3 horizontalDelta = hkPalmNodePos - objPoint;
				horizontalDelta.z = 0;
				NiPoint3 velocity = horizontalDelta / duration;
				float verticalDelta = hkPalmNodePos.z - objPoint.z + Config::options.pullDestinationZOffset; // Add an extra few cm up so that the object doesn't undershoot
				velocity.z = 0.5f * 9.81f * duration + verticalDelta / duration;

				NiAVObject *n = FindCollidableNode(selectedObject.collidable);
				if (n && DoesNodeHaveConstraint(selectedObj->loadedState->node, n)) {
					SetVelocityForAllCollisionInRefr(selectedObj, NiPointToHkVector(velocity));
				}
				else {
					hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;
					bhkRigidBody_setActivated(selectedObject.rigidBody, true);
					motion->m_linearVelocity = NiPointToHkVector(velocity);
				}
			}
			else {
				// Time's up, deselect the object and go back to idle
				Deselect();
			}
		}
		else {
			state = State::Idle;
		}
	}

	if (state == State::HeldInit || state == State::Held) {
		if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
			NiAVObject *n = FindCollidableNode(selectedObject.collidable);
			if (n) {
				NiTransform newTransform = handNode->m_worldTransform * desiredObjTransformHandSpace;

				if (state == State::HeldInit) {
					if (g_currentFrameTime - grabbedTime > 1.0f) {
						// It shouldn't take more than a second to move the object to the hand. If it does for some reason, just snap it there.
						heldTime = g_currentFrameTime;
						state = State::Held;
					}
					else {
						// Interpolate pos/rot towards the hand so it doesn't 'snap' too much

						hkQuaternion hkQuat;
						hkRotation hkRot;

						NiMatrixToHkMatrix(n->m_worldTransform.rot, hkRot);
						hkQuat.setFromRotationSimd(hkRot);
						NiQuaternion currentQuat = HkQuatToNiQuat(hkQuat);
						currentQuat = QuaternionNormalized(currentQuat);

						NiMatrixToHkMatrix(newTransform.rot, hkRot);
						hkQuat.setFromRotationSimd(hkRot);
						NiQuaternion desiredQuat = HkQuatToNiQuat(hkQuat);
						desiredQuat = QuaternionNormalized(desiredQuat);

						float deltaAngle = Config::options.grabStartAngularSpeed * 0.0174533f * *g_deltaTime;
						float quatAngle = QuaternionAngle(currentQuat, desiredQuat);

						NiPoint3 deltaDir = VectorNormalized(newTransform.pos - n->m_worldTransform.pos);
						NiPoint3 deltaPos = deltaDir * Config::options.grabStartSpeed * *g_deltaTime;

						bool doRotation = deltaAngle < quatAngle;
						bool doTranslation = VectorLengthSquared(deltaPos) < VectorLengthSquared(newTransform.pos - n->m_worldTransform.pos);

						if (doRotation || doTranslation) {
							// Rotation or position is not yet close enough

							if (doRotation) {
								// update rotation
								double slerpAmount = deltaAngle / quatAngle;
								NiQuaternion newQuat = slerp(currentQuat, desiredQuat, slerpAmount);
								newQuat = QuaternionNormalized(newQuat);
								newTransform.rot = QuaternionToMatrix(newQuat);
							}

							if (doTranslation) {
								// If not close enough, move the object closer to the hand at some velocity
								newTransform.pos = n->m_worldTransform.pos + deltaPos;
							}

							UpdateKeyframedNodeTransform(n, newTransform);
						}
						else {
							// Both position and rotation are close enough to their final values - we're done
							heldTime = g_currentFrameTime;
							state = State::Held;
						}
					}
				}

				if (state == State::Held) {
					UpdateKeyframedNodeTransform(n, newTransform);

					if (g_currentFrameTime - heldTime > Config::options.grabFreezeNearbyVelocityTime) {
						ResetNearbyDamping();
					}
				}
			}
		}
		else {
			if (g_vrikInterface) {
				g_vrikInterface->restoreFingers(isLeft);
			}

			ResetNearbyDamping();

			state = State::Idle;
		}
	}

	if (state == State::HeldBody) {
		if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
			NiAVObject *n = FindCollidableNode(selectedObject.collidable);
			if (n) {
				bhkRigidBody_setActivated(selectedObject.rigidBody, true);
				NiTransform newTransform = handNode->m_worldTransform * desiredObjTransformHandSpace;
				NiPoint3 desiredPos = newTransform.pos * havokWorldScale;
				hkRotation desiredRot;
				NiMatrixToHkMatrix(newTransform.rot, desiredRot);
				hkQuaternion desiredQuat;
				desiredQuat.setFromRotationSimd(desiredRot);
				hkpKeyFrameUtility_applyHardKeyFrame(NiPointToHkVector(desiredPos), desiredQuat, 1.0f / *g_deltaTime, selectedObject.rigidBody->hkBody);
			}
		}
		else {
			state = State::Idle;
		}
	}

	//if (state != prevState) {
	//	_MESSAGE("%s: %d -> %d", name, prevState, state);
	//}

	prevState = state;
	prevPlayerPosWorldspace = player->pos;
}


void Grabber::ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState)
{
	bool triggerDownBefore = triggerDown;
	bool gripDownBefore = gripDown;

	//_MESSAGE("%s:, controller update", name);

	uint64_t triggerMask = vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_SteamVR_Trigger);
	uint64_t gripMask = vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_Grip);

	// Check if the trigger is pressed
	triggerDown = Config::options.enableTrigger && (pControllerState->ulButtonPressed & triggerMask);
	gripDown = Config::options.enableGrip && (pControllerState->ulButtonPressed & gripMask);

	bool triggerRisingEdge = triggerDown && !triggerDownBefore;
	bool triggerFallingEdge = !triggerDown && triggerDownBefore;

	bool gripRisingEdge = gripDown && !gripDownBefore;
	bool gripFallingEdge = !gripDown && gripDownBefore;

	if (inputState == InputState::Idle) {
		if (triggerRisingEdge || gripRisingEdge) {
			grabRequestedTime = GetTime();
			grabRequested = true;

			inputTrigger = false;
			inputGrip = false;

			if (triggerRisingEdge) {
				PlayerCharacter *pc = *g_thePlayer;
				if (pc && !pc->actorState.IsWeaponDrawn()) {
					inputTrigger = true;
				}
			}

			if (gripRisingEdge && delayGripInput) {
				inputGrip = true;
			}

			inputState = InputState::Leeway;
		}
	}

	if (inputState == InputState::Leeway) {
		if ((triggerFallingEdge || gripFallingEdge) && !triggerDown && !gripDown) {
			releaseRequested = true;
			if (wasObjectGrabbed) {
				inputState = InputState::Idle;
			}
			else {
				forceInput = true;
				inputState = InputState::Force;
			}
		}
		else {
			if (wasObjectGrabbed) {
				inputState = InputState::Block;
			}
			else {
				double currentTime = GetTime();
				if (currentTime - grabRequestedTime <= Config::options.triggerPressedLeewayTime) {
					if (triggerRisingEdge) {
						PlayerCharacter *pc = *g_thePlayer;
						if (pc && !pc->actorState.IsWeaponDrawn()) {
							inputTrigger = true;
						}
					}
					if (gripRisingEdge && delayGripInput) {
						inputGrip = true;
					}

					// Suppress trigger press
					if (inputTrigger) {
						pControllerState->ulButtonPressed &= ~triggerMask;
					}
					if (inputGrip) {
						pControllerState->ulButtonPressed &= ~gripMask;
					}
				}
				else {
					// Leeway time has run out
					forceInput = true;
					inputState = InputState::Force;
				}
			}
		}
	}

	if (inputState == InputState::Block) {
		if ((triggerFallingEdge || gripFallingEdge) && !triggerDown && !gripDown) {
			releaseRequested = true;
			inputState = InputState::Idle;
		}
		else {
			pControllerState->ulButtonPressed &= ~triggerMask;
			pControllerState->ulButtonPressed &= ~gripMask;
		}
	}

	if (inputState == InputState::Force) {
		if (forceInput) {
			if (inputTrigger) {
				pControllerState->ulButtonPressed |= triggerMask;
			}
			if (inputGrip) {
				pControllerState->ulButtonPressed |= gripMask;
			}
		}
		else {
			inputState = InputState::Idle;
		}
	}
}


void Grabber::EndPull()
{
	NiPointer<TESObjectREFR> pulledObj;
	if (LookupREFRByHandle(pulledObject.handle, pulledObj)) {
		CollisionInfo::ResetCollisionInfoForAllCollisionInRefr(pulledObj, CollisionInfo::State::Unheld);
		pulledObject.rigidBody->hkBody->m_motion.m_motionState.m_angularDamping = pulledObject.savedAngularDamping;
	}
	pulledObject.handle = *g_invalidRefHandle;
	pulledObject.rigidBody = nullptr;
}


bool Grabber::IsObjectPullable()
{
	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
		return (!selectedObject.isActor || selectedObject.hitForm);
	}
	return false;
}


bool Grabber::HasExclusiveObject() const
{
	return state == State::Pulled || state == State::SelectionLocked || state == State::Held || state == State::HeldInit || state == State::HeldBody;
}


bool Grabber::ShouldDisplayRollover()
{
	if (state != State::SelectedClose && state != State::SelectionLocked && state != State::Held && state != State::HeldInit && state != State::HeldBody)
		return false;

	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
		return true;
	}
	return false;
}


bool Grabber::IsSafeToClearSavedCollision()
{
	return (state != State::Held && state != State::HeldInit && state != State::HeldBody && pulledObject.handle == *g_invalidRefHandle);
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

		SetSelectedHandles(isLeftHanded);
	}
}

void Grabber::SetSelectedHandles(bool isLeftHanded)
{
	// Now set all the places I could find that get set to the handle of the pointed at object usually
	CrosshairPickData *pickData = *g_pickData;
	if (pickData) {
		if (isLeftHanded) {
			pickData->leftHandle1 = selectedObject.handle;
			pickData->leftHandle2 = selectedObject.handle;
			pickData->leftHandle3 = selectedObject.handle;
		}
		else {
			pickData->rightHandle1 = selectedObject.handle;
			pickData->rightHandle2 = selectedObject.handle;
			pickData->rightHandle3 = selectedObject.handle;
		}
	}
}
