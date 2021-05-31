#include <numeric>
#include <regex>
#include <sstream>
#include <chrono>

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
#include "hooks.h"
#include "pluginapi.h"
#include "main.h"

#include <Physics/Collide/Query/CastUtil/hkpLinearCastInput.h>
#include <Physics/Collide/Query/CastUtil/hkpWorldRayCastInput.h>
#include <Physics/Collide/Agent3/Machine/Nn/hkpLinkedCollidable.h>
#include <Physics/Collide/Dispatch/hkpCollisionDispatcher.h>


int isHeadBobbingSavedCount = 0;
double savedHeadBobbingHeight = 0.0;

UInt32 skinMaterialId = 0x233db702;
UInt32 stoneMaterialId = 0xdf02f237;

// Gets callbacks from havok linear cast
CdPointCollector cdPointCollector;
hkpLinearCastInput linearCastInput;
RayHitCollector rayHitCollector;
AllRayHitCollector allRayHitCollector;
CdBodyPairCollector pairCollector;
SpecificPairCollector specificPairCollector;
// 'ItemPicker' collision layer; player collision group
// Why ItemPicker? It ignores stuff like weapons the player is holding
//static hkpWorldRayCastInput rayCastInput(0x02420028);
hkpWorldRayCastInput rayCastInput;


UInt32 priorities[] = {
	4, // head
	3, // hair
	10, // body
	6, // hands
	7, // forearms
	2, // amulet
	5, // ring
	8, // feet
	9, // calves
	0, // shield
	13, // tail - seems to be used by cloaks
	11, // longhair
	1, // circlet
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
	if (g_openVR && *g_openVR && duration > 0) {
		BSOpenVR *openVR = *g_openVR;
		openVR->TriggerHapticPulse(hand, duration);
	}
}


void HapticsManager::QueueHapticEvent(float startStrength, float endStrength, float duration)
{
	HapticEvent hapticEvent;
	hapticEvent.startStrength = startStrength;
	hapticEvent.endStrength = endStrength;
	hapticEvent.duration = duration;
	hapticEvent.isNew = true;

	{
		std::scoped_lock lock(eventsLock);
		events.push_back(hapticEvent);
	}
}

void HapticsManager::QueueHapticPulse(float strength)
{
	QueueHapticEvent(strength, strength, *g_deltaTime * 2.0f);
}

void HapticsManager::Loop()
{
	while (true) {
		{
			std::scoped_lock lock(eventsLock);

			size_t numEvents = events.size();
			if (numEvents > 0) {
				double currentTime = GetTime();

				// Just play the last event that was added
				HapticEvent &lastEvent = events[numEvents - 1];

				if (lastEvent.isNew) {
					lastEvent.isNew = false;
					lastEvent.startTime = currentTime;
				}

				float strength;
				if (lastEvent.duration == 0) {
					strength = lastEvent.startStrength;
				}
				else {
					// Simple lerp from start to end strength over duration
					double elapsedTime = currentTime - lastEvent.startTime;
					strength = lerp(lastEvent.startStrength, lastEvent.endStrength, min(1.0f, elapsedTime / lastEvent.duration));
				}
				TriggerHapticPulse(strength);

				// Cleanup events that are past their duration
				auto end = std::remove_if(events.begin(), events.end(),
					[currentTime](HapticEvent &evnt) { return currentTime - evnt.startTime >= evnt.duration; }
				);
				events.erase(end, events.end());
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5)); // TriggerHapticPulse can only be called once every 5ms
	}
}


void Grabber::TriggerCollisionHaptics(float mass, float separatingVelocity)
{
	float massComponent = Config::options.collisionMassProportionalHapticStrength * max(0.0f, powf(mass, Config::options.collisionHapticMassExponent));
	float speedComponent = Config::options.collisionSpeedProportionalHapticStrength * separatingVelocity;
	float hapticStrength = Config::options.collisionBaseHapticStrength + speedComponent + massComponent;
	hapticStrength = min(1.0f, hapticStrength);

	if (state == State::HeldBody && (dampingState == DampingState::Damped || dampingState == DampingState::TryLeaveDamped)) {
		hapticStrength *= Config::options.dampedCollisionHapticStrengthMultiplier;
		hapticStrength = max(Config::options.collisionBaseHapticStrength, hapticStrength); // don't let it go below the base strength
		hapticStrength = min(1.0f, hapticStrength);
	}

	haptics.QueueHapticEvent(hapticStrength, hapticStrength, Config::options.collisionHapticDuration);
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

	disableDropEvents = false;
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
	selectedObject.hitExtraList = nullptr;
	selectedObject.isDisconnected = false;

	disableDropEvents = false;

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
		PlayShader(objHandle, node, shader, selectedObject.isActor);
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
		StopShader(objHandle, node, shader, selectedObject.isActor);
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


std::unordered_set<bhkRigidBody *> g_nearbyBodies; // prevent duplicates due to multiple contact points
void Grabber::StartNearbyDamping(bhkWorld &world)
{
	nearbyBodies.clear();
	nearbyBodyMap.clear();
	g_nearbyBodies.clear();

	cdPointCollector.reset();

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

				if (VectorLength(HkVectorToNiPoint(motion.m_linearVelocity)) < Config::options.nearbyGrabMaxLinearVelocity &&
					VectorLength(HkVectorToNiPoint(motion.m_angularVelocity)) < Config::options.nearbyGrabMaxAngularVelocity) {
					if (g_nearbyBodies.count(bRigidBody) == 0) {
						g_nearbyBodies.insert(bRigidBody);
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


NiPoint3 GetClosestPointToRigidbody(bhkWorld &world, bhkRigidBody *rigidBody, bhkSimpleShapePhantom *sphere, const NiPoint3 &initialTranslation, const NiPoint3 &start)
{
	hkpCollidable *collidable = &rigidBody->hkBody->m_collidable;

	NiPoint3 centerOfMass = HkVectorToNiPoint(rigidBody->hkBody->m_motion.m_motionState.m_transform.m_translation);
	NiPoint3 foundPoint = centerOfMass; // Fallback to the center of the object

	hkpCollisionDispatcher *dispatcher = world.world->m_collisionDispatcher;
	if (dispatcher) {
		auto sphereShape = (hkpConvexShape *)sphere->phantom->m_collidable.m_shape;

		hkpShapeType shapeType = collidable->m_shape->m_type;
		hkpCollisionDispatcher::GetClosestPointsFunc closestPointsFunc = dispatcher->getGetClosestPointsFunc(shapeType, sphereShape->m_type);
		if (closestPointsFunc) {
			// Position the object in the direction of the palm for better grab results
			rigidBody->hkBody->m_motion.m_motionState.m_transform.m_translation = NiPointToHkVector(initialTranslation);

			const int maxIterations = 5;
			int numIterations = 0;

			// Save sphere properties so we can change them and restore them later
			UInt32 filterInfoBefore = sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo;
			float radiusBefore = sphereShape->getRadius();
			hkVector4 translationBefore = sphere->phantom->m_motionState.getTransform().getTranslation();

			sphere->phantom->m_motionState.m_transform.m_translation = NiPointToHkVector(start);
			sphereShape->m_radius = VectorLength(initialTranslation - start) + 0.02f; // Add some fudge
			sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = 0x2C;

			float closestDistance;
			// Do a series of closest points queries, starting with the object just within a sphere, and progressively shrink the radius to being just outside the object
			do {
				numIterations++;

				cdPointCollector.reset();
				closestPointsFunc(*collidable, sphere->phantom->m_collidable, *world.world->m_collisionInput, cdPointCollector);

				NiPoint3 closestPoint;
				closestDistance = (std::numeric_limits<float>::max)();
				if (cdPointCollector.m_hits.size() > 0) {
					for (auto pair : cdPointCollector.m_hits) {
						hkContactPoint &contactPoint = pair.second;
						float dist = contactPoint.getDistance();
						if (dist < closestDistance) {
							closestPoint = HkVectorToNiPoint(pair.second.getPosition());
							closestDistance = dist;
						}
					}

					// We want the point we output to be relative to where it actually is, rather than where we move it when computing closest points
					foundPoint = closestPoint + (centerOfMass - initialTranslation);
				}

				if (closestDistance < 0) {
					sphereShape->m_radius += closestDistance - 0.02f;
					sphereShape->m_radius = max(0.01f, sphereShape->m_radius);
				}
			} while (closestDistance < 0 && numIterations < maxIterations);

			_MESSAGE("%d external grab closest point iterations", numIterations);

			sphere->phantom->m_motionState.m_transform.m_translation = translationBefore;
			sphereShape->m_radius = radiusBefore;
			sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = filterInfoBefore;

			rigidBody->hkBody->m_motion.m_motionState.m_transform.m_translation = NiPointToHkVector(centerOfMass); // Restore the object's position
		}
	}

	return foundPoint;
}


bool Grabber::FindCloseObject(bhkWorld *world, bool allowGrab, const Grabber &other, const NiPoint3 &hkPalmPos, const NiPoint3 &castDirection, const bhkSimpleShapePhantom *sphere,
	NiPointer<TESObjectREFR> &closestObj, NiPointer<bhkRigidBody> &closestRigidBody, hkVector4 &closestPoint)
{
	if (!allowGrab) {
		return false;
	}

	auto sphereShape = (hkpConvexShape *)sphere->phantom->m_collidable.m_shape;
	// Save sphere properties so we can change them and restore them later
	UInt32 filterInfoBefore = sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo;
	float radiusBefore = sphereShape->getRadius();
	hkVector4 translationBefore = sphere->phantom->m_motionState.getTransform().getTranslation();

	sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = 0x2C;
	sphereShape->m_radius = Config::options.nearCastRadius;
	sphere->phantom->m_motionState.m_transform.m_translation = NiPointToHkVector(hkPalmPos);

	NiPoint3 targetPos = hkPalmPos + castDirection * Config::options.nearCastDistance;
	linearCastInput.m_to = NiPointToHkVector(targetPos);
	cdPointCollector.reset();

	bool otherObjectIsGrabbable = other.CanOtherGrab();

	NiPointer<TESObjectREFR> pulledObj;
	UInt32 pulledHandle = pulledObject.handle;
	LookupREFRByHandle(pulledHandle, pulledObj);

	world->worldLock.LockForRead();
	hkpWorld_LinearCast(world->world, &sphere->phantom->m_collidable, &linearCastInput, &cdPointCollector, nullptr);

	// Process result of cast
	float closestDistance = (std::numeric_limits<float>::max)();
	for (auto pair : cdPointCollector.m_hits) {
		auto collidable = static_cast<hkpCollidable *>(pair.first);
		hkpRigidBody *rigidBody = hkpGetRigidBody(collidable);
		if (!rigidBody || !rigidBody->m_userData) {
			continue; // No rigidbody -> no movement :/
		}
		NiPointer<TESObjectREFR> ref = FindCollidableRef(collidable);
		if (ref && ref != *g_thePlayer) {
			if (IsAllowedCollidable(collidable) || (collidable == other.selectedObject.collidable && otherObjectIsGrabbable) || (ref->baseForm && ref->baseForm->formType == kFormType_Projectile)) {
				if (ref->baseForm->formType == kFormType_Projectile) {
					auto impactData = *(void **)((UInt64)ref.m_pObject + 0x98);
					if (!impactData) {
						// Only grab projectiles that are not mid flight
						continue;
					}
				}
				// Get distance from the hit on the collidable to the ray
				NiPoint3 hit = HkVectorToNiPoint(pair.second.getPosition());
				NiPoint3 handToHit = hit - hkPalmPos;
				NiPoint3 handToHitAlongRay = castDirection * DotProduct(handToHit, castDirection); // project above vector onto ray
				float dist = VectorLength(handToHit - handToHitAlongRay); // distance from hit location to closest point on the ray
				if (dist < closestDistance) {
					closestObj = ref;
					closestRigidBody = (bhkRigidBody *)rigidBody->m_userData;
					closestPoint = pair.second.getPosition();
					closestDistance = dist;
				}
			}
		}
	}

	bool found = closestDistance != (std::numeric_limits<float>::max)();
	if (!found && pulledObj) {
		// No object found normally - do a check for the pulled object in a wider radius
		sphereShape->m_radius = Config::options.widePullGrabRadius;
		specificPairCollector.reset();
		specificPairCollector.m_target = &pulledObject.rigidBody->hkBody->m_collidable;
		hkpWorld_GetPenetrations(world->world, &sphere->phantom->m_collidable, world->world->m_collisionInput, &specificPairCollector);

		if (specificPairCollector.m_foundTarget) {
			closestObj = pulledObj;
			closestRigidBody = pulledObject.rigidBody;
			closestPoint = pulledObject.rigidBody->hkBody->m_motion.m_motionState.m_transform.m_translation;
			found = true;
		}
	}

	world->worldLock.UnlockRead();

	sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = filterInfoBefore;
	sphereShape->m_radius = radiusBefore;
	sphere->phantom->m_motionState.m_transform.m_translation = translationBefore;

	return found;
}


bool Grabber::FindFarObject(bhkWorld *world, const Grabber &other, const NiPoint3 &hkPalmPos, const NiPoint3 &castDirection, const NiPoint3 &hkHmdPos, const NiPoint3 &hmdForward, const bhkSimpleShapePhantom *sphere,
	NiPointer<TESObjectREFR> &closestObj, NiPointer<bhkRigidBody> &closestRigidBody, hkVector4 &closestPoint)
{
	NiPoint3 hkTargetPos = hkPalmPos + castDirection * Config::options.farCastDistance;

	NiPoint3 hitPosition = { hkTargetPos.x, hkTargetPos.y, hkTargetPos.z };

	// First, raycast in the pointing direction
	rayHitCollector.reset();
	rayCastInput.m_filterInfo = ((UInt32)playerCollisionGroup << 16) | 0x28;
	rayCastInput.m_from = NiPointToHkVector(hkPalmPos);
	rayCastInput.m_to = NiPointToHkVector(hkTargetPos);
	world->worldLock.LockForRead();
	hkpWorld_CastRay(world->world, &rayCastInput, &rayHitCollector);
	world->worldLock.UnlockRead();
	if (rayHitCollector.m_doesHitExist) {
		// If raycast hit, we want to linearcast only up to the ray hit location
		NiPoint3 handToTarget = hkTargetPos - hkPalmPos;
		hitPosition = hkPalmPos + (handToTarget * rayHitCollector.m_closestHitInfo.m_hitFraction);// -(VectorNormalized(handToTarget) * Config::options.castRadius);
	}

	auto sphereShape = (hkpConvexShape *)sphere->phantom->m_collidable.m_shape;

	// Save sphere properties so we can change them and restore them later
	UInt32 filterInfoBefore = sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo;
	float radiusBefore = sphereShape->getRadius();
	hkVector4 translationBefore = sphere->phantom->m_motionState.getTransform().getTranslation();

	sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = 0x2C;
	sphereShape->m_radius = Config::options.farCastRadius;
	sphere->phantom->m_motionState.m_transform.m_translation = NiPointToHkVector(hkPalmPos);

	// Now, linearcast up to the point the raycast hit, or up to the limit if it's empty space
	// 'CustomPick2' layer, to pick up projectiles, because ONLY THIS GODDAMN LAYER can collide with projectiles. This filterinfo will collide with everything.
	linearCastInput.m_to = NiPointToHkVector(hitPosition);
	cdPointCollector.reset();

	world->worldLock.LockForRead();
	hkpWorld_LinearCast(world->world, &sphere->phantom->m_collidable, &linearCastInput, &cdPointCollector, nullptr);

	// Process result of cast
	float closestDistance = (std::numeric_limits<float>::max)();
	for (auto pair : cdPointCollector.m_hits) {
		auto collidable = static_cast<hkpCollidable *>(pair.first);
		if (other.HasExclusiveObject() && collidable == other.selectedObject.collidable) {
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
				NiPoint3 handToHit = hit - hkPalmPos;
				NiPoint3 handToHitAlongRay = castDirection * DotProduct(handToHit, castDirection); // project above vector onto ray
				float dist = VectorLength(handToHit - handToHitAlongRay); // distance from hit location to closest point on the ray
				if (dist < closestDistance && DotProduct(VectorNormalized(hit - hkHmdPos), hmdForward) >= Config::options.requiredCastDotProduct) {
					closestObj = ref;
					closestRigidBody = bRigidBody;
					closestPoint = pair.second.getPosition();
					closestDistance = dist;
				}
			}
		}
	}

	world->worldLock.UnlockRead();

	sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = filterInfoBefore;
	sphereShape->m_radius = radiusBefore;
	sphere->phantom->m_motionState.m_transform.m_translation = translationBefore;

	return closestDistance != (std::numeric_limits<float>::max)();
}


hkTransform Grabber::ComputeHandCollisionTransform(NiAVObject *handNode)
{
	hkTransform transform;
	NiPoint3 handCollisionBoxOffset = Config::options.handCollisionBoxOffset;
	if (isLeft) handCollisionBoxOffset.x *= -1;
	float havokWorldScale = *g_havokWorldScale;
	transform.m_translation = NiPointToHkVector((handNode->m_worldTransform * (handCollisionBoxOffset / havokWorldScale)) * havokWorldScale);
	NiMatrixToHkMatrix(handNode->m_worldTransform.rot, transform.m_rotation);

	return transform;
}


hkTransform Grabber::ComputeWeaponCollisionTransform(bhkRigidBody *existingWeaponCollision)
{
	PlayerCharacter *player = *g_thePlayer;
	hkTransform transform = existingWeaponCollision->hkBody->getTransform();
	if (g_isVrikPresent) {
		// If using VRIK, the weapon is actually a bit offset. Use the actual position of the weapon from vrik from last frame. That's the best we can do.
		static BSFixedString weaponNodeName("WEAPON");
		static BSFixedString shieldNodeName("SHIELD");
		bool useLeft = isLeft;
		if (*g_leftHandedMode) useLeft = !useLeft;
		BSFixedString &handWeaponNodeName = useLeft ? shieldNodeName : weaponNodeName;
		NiPointer<NiAVObject> weaponNode = player->GetNiRootNode(0)->GetObjectByName(&handWeaponNodeName.data);
		if (weaponNode) {
			NiTransform &nodeTransform = weaponNode->m_oldWorldTransform;
			transform.m_translation = NiPointToHkVector(nodeTransform.pos * *g_havokWorldScale);
			NiMatrixToHkMatrix(nodeTransform.rot, transform.m_rotation);
		}
	}

	return transform;
}


void Grabber::CreateHandCollision(bhkWorld *world)
{
	bhkBoxShape *handShape = (bhkBoxShape *)Heap_Allocate(sizeof(bhkBoxShape));
	if (!handShape) return;

	hkVector4 halfExtents = NiPointToHkVector(Config::options.handCollisionBoxHalfExtents);
	bhkBoxShape_ctor(handShape, &halfExtents);
	handShape->materialId = skinMaterialId;
	((hkpBoxShape*)handShape->shape)->m_radius = Config::options.handCollisionBoxRadius;

	bhkRigidBodyCinfo cInfo;
	bhkRigidBodyCinfo_ctor(&cInfo);

	UInt32 filterInfo = ((UInt32)playerCollisionGroup << 16) | 56; // player group, our custom layer
	filterInfo |= (1 << 15); // set bit 15 to collide with same group that also has bit 15

	UInt8 ragdollBits = (UInt8)(isLeft ? CollisionInfo::RagdollLayer::LeftHand : CollisionInfo::RagdollLayer::RightHand);
	filterInfo |= (ragdollBits << 8);

	cInfo.collisionFilterInfo = filterInfo;
	cInfo.hkCinfo.m_collisionFilterInfo = filterInfo;
	cInfo.shape = handShape->shape;
	cInfo.hkCinfo.m_shape = handShape->shape;
	cInfo.hkCinfo.m_motionType = hkpMotion::MotionType::MOTION_KEYFRAMED;
	cInfo.hkCinfo.m_enableDeactivation = false;
	cInfo.hkCinfo.m_solverDeactivation = hkpRigidBodyCinfo::SolverDeactivation::SOLVER_DEACTIVATION_OFF;
	cInfo.hkCinfo.m_qualityType = hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED; // Could use KEYFRAMED_REPORTING to have its collisions trigger callbacks?

	NiPointer<NiAVObject> handNode = GetHandNode();
	if (handNode) {
		hkTransform transform = ComputeHandCollisionTransform(handNode);
		cInfo.hkCinfo.m_position = transform.m_translation;
		cInfo.hkCinfo.m_rotation.setFromRotationSimd(transform.m_rotation);
	}
	else {
		cInfo.hkCinfo.m_position = NiPointToHkVector((*g_thePlayer)->loadedState->node->m_worldTransform.pos * *g_havokWorldScale); // Place it where the player is
	}

	bhkRigidBody *handRigidBody = (bhkRigidBody *)Heap_Allocate(sizeof(bhkRigidBody));
	if (handRigidBody) {
		bhkRigidBody_ctor(handRigidBody, &cInfo);

		bhkRigidBody_setActivated(handRigidBody, true);
		hkpWorld_AddEntity(world->world, handRigidBody->hkBody, HK_ENTITY_ACTIVATION_DO_ACTIVATE);

		handBody = handRigidBody;
	}
}


void Grabber::RemoveHandCollision(bhkWorld *world)
{
	hkBool ret;
	hkpWorld_RemoveEntity(world->world, &ret, handBody->hkBody);
	handBody = nullptr;
}


void Grabber::UpdateHandCollision(NiAVObject *handNode, bhkWorld *world)
{
	hkpRigidBody *handCollBody = handBody->hkBody;
	bool wasCollisionDisabled = (handCollBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 14 & 1) != 0;

	if (state == State::HeldBody && selectedObject.isActor) {
		// Don't have the hand collide while we're holding a body
		handCollBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= (1 << 14); // turns collision off
		if (!wasCollisionDisabled) {
			BSWriteLocker lock(&world->worldLock);
			hkpWorld_UpdateCollisionFilterOnEntity(world->world, handCollBody, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
		}
	}
	else {
		handCollBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= ~(1 << 14);
		if (wasCollisionDisabled) {
			BSWriteLocker lock(&world->worldLock);
			hkpWorld_UpdateCollisionFilterOnEntity(world->world, handCollBody, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
		}
	}

	// Set collision group for the hand collision every frame. The player collision changes sometimes, e.g. when getting on/off a horse
	handCollBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= (0x0000ffff); // zero out collision group
	handCollBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt32)playerCollisionGroup << 16); // set collision group to player group

	// Put our hand collision where we want it
	hkTransform transform = ComputeHandCollisionTransform(handNode);
	hkQuaternion desiredQuat;
	desiredQuat.setFromRotationSimd(transform.m_rotation);
	hkpKeyFrameUtility_applyHardKeyFrame(transform.m_translation, desiredQuat, 1.0f / *g_deltaTime, handCollBody);
}


void Grabber::CreateWeaponCollision(bhkWorld *world)
{
	if (!Config::options.enableWeaponCollision) return;

	PlayerCharacter *player = *g_thePlayer;
	
	UInt64 dataOffset = 0x710;
	if (isLeft) dataOffset += sizeof(VRMeleeData);
	VRMeleeData *meleeData = (VRMeleeData *)((UInt64)player + dataOffset);

	NiPointer<NiNode> collisionNode = meleeData->collisionNode;
	if (!collisionNode) return;

	NiPointer<bhkRigidBody> rigidBody = GetRigidBody(collisionNode);
	if (!rigidBody) return;

	hkpRigidBody *hkBody = rigidBody->hkBody;
	if (!hkBody) return;

	const hkpShape *shape = hkBody->m_collidable.m_shape;
	if (!shape) return;

	bhkShape *bShape = (bhkShape *)shape->m_userData;
	if (!bShape) return;

	NiCloningProcess cloningProcess = NiCloningProcess();
	cloningProcess.scale = NiPoint3(1.0f, 1.0f, 1.0f) / *g_fMeleeWeaponHavokScale; // Undo the scaling of the original shape done when creating it

	if (g_vrikInterface) {
		double handSize = g_vrikInterface->getSettingDouble("handSize"); // 0.85 is the default
		cloningProcess.scale *= handSize;
	}

	bhkShape *clonedShape = (bhkShape *)NiObject_Clone(bShape, &cloningProcess);

	bhkRigidBodyCinfo cInfo;
	bhkRigidBodyCinfo_ctor(&cInfo);

	UInt32 filterInfo = ((UInt32)playerCollisionGroup << 16) | 56; // player group, our custom layer
	filterInfo |= (1 << 15); // set bit 15 to collide with same group that also has bit 15

	UInt8 ragdollBits = (UInt8)(isLeft ? CollisionInfo::RagdollLayer::LeftHand : CollisionInfo::RagdollLayer::RightHand);
	filterInfo |= (ragdollBits << 8);

	filterInfo |= (1 << 14); // Initially, turn collision off - we turn it on in the Update() if needed but don't spawn it in enabled

	cInfo.collisionFilterInfo = filterInfo;
	cInfo.hkCinfo.m_collisionFilterInfo = filterInfo;
	cInfo.shape = clonedShape->shape;
	cInfo.hkCinfo.m_shape = clonedShape->shape;
	cInfo.hkCinfo.m_motionType = hkpMotion::MotionType::MOTION_KEYFRAMED;
	cInfo.hkCinfo.m_enableDeactivation = false;
	cInfo.hkCinfo.m_solverDeactivation = hkpRigidBodyCinfo::SolverDeactivation::SOLVER_DEACTIVATION_OFF;
	cInfo.hkCinfo.m_mass = 0.0f;
	cInfo.hkCinfo.m_qualityType = hkpCollidableQualityType::HK_COLLIDABLE_QUALITY_KEYFRAMED; // Could use KEYFRAMED_REPORTING to have its collisions trigger callbacks?

	hkTransform transform = ComputeWeaponCollisionTransform(rigidBody);
	cInfo.hkCinfo.m_position = transform.m_translation;
	cInfo.hkCinfo.m_rotation.setFromRotationSimd(transform.m_rotation);

	bhkRigidBody *clonedBody = (bhkRigidBody *)Heap_Allocate(sizeof(bhkRigidBody));
	if (clonedBody) {
		bhkRigidBody_ctor(clonedBody, &cInfo);

		bhkRigidBody_setActivated(clonedBody, true);
		hkpWorld_AddEntity(world->world, clonedBody->hkBody, HK_ENTITY_ACTIVATION_DO_ACTIVATE);

		clonedFromBody = rigidBody;
		weaponBody = clonedBody;
	}
}


void Grabber::RemoveWeaponCollision(bhkWorld *world)
{
	if (!Config::options.enableWeaponCollision) return;

	if (!weaponBody) return;

	hkBool ret;
	hkpWorld_RemoveEntity(world->world, &ret, weaponBody->hkBody);
	weaponBody = nullptr;
	clonedFromBody = nullptr;
}


void Grabber::UpdateWeaponCollision()
{
	if (!Config::options.enableWeaponCollision) return;

	PlayerCharacter *player = *g_thePlayer;

	UInt64 dataOffset = 0x710;
	if (isLeft) dataOffset += sizeof(VRMeleeData);
	VRMeleeData *meleeData = (VRMeleeData *)((UInt64)player + dataOffset);

	NiPointer<NiNode> collisionNode = meleeData->collisionNode;
	if (collisionNode) {
		NiPointer<bhkRigidBody> rigidBody = GetRigidBody(collisionNode);
		if (rigidBody) {
			if (rigidBody != clonedFromBody) {
				NiPointer<bhkWorld> world = meleeData->world;
				if (world) {
					BSWriteLocker(&world->worldLock);
					RemoveWeaponCollision(world);
					CreateWeaponCollision(world);
				}
			}

			TESForm *equippedItem = *g_leftHandedMode ? player->GetEquippedObject(!isLeft) : player->GetEquippedObject(isLeft);
			bool isUnarmed = equippedItem == nullptr;
			if (equippedItem) {
				TESObjectWEAP *equippedWeapon = DYNAMIC_CAST(equippedItem, TESForm, TESObjectWEAP);
				if (equippedWeapon) {
					UInt8 type = equippedWeapon->type();
					if (type == TESObjectWEAP::GameData::kType_HandToHandMelee || type == TESObjectWEAP::GameData::kType_H2H) {
						isUnarmed = true;
					}
				}
			}

			bool wasCollisionDisabled = (weaponBody->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 14 & 1) != 0;

			if (isUnarmed || !player->actorState.IsWeaponDrawn()) {
				// Do not enable weapon collision when unarmed or no weapon drawn
				// We DO still want to move it though, since once we enable it again we don't want a huge jump
				if (!wasCollisionDisabled) {
					weaponBody->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= (1 << 14); // turns collision off
					bhkWorldObject_UpdateCollisionFilter(weaponBody);
				}
			}
			else {
				if (wasCollisionDisabled) {
					weaponBody->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= ~(1 << 14);
					bhkWorldObject_UpdateCollisionFilter(weaponBody);
				}
			}

			// Set collision group for the hand collision every frame. The player collision changes sometimes, e.g. when getting on/off a horse
			weaponBody->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= (0x0000ffff); // zero out collision group
			weaponBody->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt32)playerCollisionGroup << 16); // set collision group to player group

			hkTransform transform = ComputeWeaponCollisionTransform(rigidBody);
			hkQuaternion desiredQuat;
			desiredQuat.setFromRotationSimd(transform.m_rotation);
			hkpKeyFrameUtility_applyHardKeyFrame(transform.m_translation, desiredQuat, 1.0f / *g_deltaTime, weaponBody->hkBody);
		}
	}
}


void Grabber::PlayPhysicsSound(const NiPoint3 &location, bool loud)
{
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
			BGSMaterialType *skinMaterial = GetMaterialType(skinMaterialId);
			BGSMaterialType *stoneMaterial = GetMaterialType(stoneMaterialId);
			if (material) {
				BGSImpactData *impactData = nullptr;
				if (material->impactDataSet) {
					auto impactDataSet = DYNAMIC_CAST(material->impactDataSet, TESForm, BGSImpactDataSet);
					if (impactDataSet) {
						if (skinMaterial) {
							impactData = BGSImpactDataSet_GetImpactData(impactDataSet, skinMaterial);
						}

						if (!impactData) {
							if (stoneMaterial) {
								impactData = BGSImpactDataSet_GetImpactData(impactDataSet, stoneMaterial);
							}
						}
					}
				}
				else {
					// No impact data set for the material on the shape... try a lookup in the other direction
					if (skinMaterial && skinMaterial->impactDataSet) {
						auto impactDataSet = DYNAMIC_CAST(skinMaterial->impactDataSet, TESForm, BGSImpactDataSet);
						if (impactDataSet) {
							impactData = BGSImpactDataSet_GetImpactData(impactDataSet, material);
						}
					}

					if (!impactData) {
						if (stoneMaterial && stoneMaterial->impactDataSet) {
							auto impactDataSet = DYNAMIC_CAST(stoneMaterial->impactDataSet, TESForm, BGSImpactDataSet);
							if (impactDataSet) {
								impactData = BGSImpactDataSet_GetImpactData(impactDataSet, material);
							}
						}
					}
				}

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


bool Grabber::ShouldUsePhysicsBasedGrab(NiNode *root, NiAVObject *node, TESForm *baseForm)
{
	if (Config::options.forcePhysicsGrab) return true;

	// Ragdolls, arrows (their collision gets offset for some reason when keyframed) and objects with constraints (books, skulls with jaws, wagons with wheels, etc. - physics goes crazy when keyframed) use physics based motion
	bool usePhysicsBasedGrab = DoesNodeHaveConstraint(root, node);// || IsSkinnedToNode(root, node);
	return selectedObject.isActor || usePhysicsBasedGrab || (baseForm && baseForm->formType == kFormType_Ammo);
}


bool Grabber::TransitionHeld(Grabber &other, bhkWorld &world, const NiPoint3 &hkPalmPos, const NiPoint3 &palmDirection, const NiPoint3 &closestPoint, float havokWorldScale, const NiAVObject *handNode, TESObjectREFR *selectedObj, NiTransform *initialTransform, bool playSound)
{
	bool wereFingersSet = false;

	NiPointer<NiAVObject> collidableNode = FindCollidableNode(selectedObject.collidable);
	NiPointer<NiNode> objRoot = selectedObj->GetNiNode();
	if (collidableNode && objRoot) {
		StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

		NiPoint3 palmPos = hkPalmPos / havokWorldScale;

		float mass = NiAVObject_GetMass(collidableNode, 0);
		float hapticStrength = min(1.0f, Config::options.grabBaseHapticStrength + Config::options.grabProportionalHapticStrength * max(0.0f, powf(mass, Config::options.grabHapticMassExponent)));
		haptics.QueueHapticEvent(hapticStrength, hapticStrength, Config::options.grabHapticFadeTime);

		grabbedTime = g_currentFrameTime;
		rolloverDisplayTime = g_currentFrameTime;

		NiPoint3 ptPos = closestPoint / havokWorldScale; // in skyrim coords
		//NiPoint3 normal = HkVectorToNiPoint(closestPoint.m_separatingNormal); // vec from sphere center to point

		// Cancel a collision reset from pulling if we're grabbing the object
		bool shouldMoveHandBack = false;
		if (pulledObject.handle == selectedObject.handle) {
			shouldMoveHandBack = true;
			EndPull();
		}
		else if (other.pulledObject.handle == selectedObject.handle) {
			shouldMoveHandBack = true;
			other.EndPull();
		}

		if (selectedObject.isImpactedProjectile) { // It's an embedded projectile, i.e. stuck in a wall etc.
			auto rigidBody = GetFirstRigidBody(objRoot);
			if (rigidBody) {
				// Do not use selectedObject.collidable here, as sometimes we end up grabbing the phantom shape of the projectile instead of the 3D one
				auto collidable = &rigidBody->hkBody->m_collidable;
				// Projectiles do not interact with collision usually. We need to change the filter to make them interact.
				collidable->m_broadPhaseHandle.m_collisionFilterInfo = (((UInt32)playerCollisionGroup) << 16) | 5; // player collision group, 'weapon' collision layer
				// Projectiles have 'Fixed' motion type by default, making them unmovable
				bhkRigidBody_setMotionType(rigidBody, hkpMotion::MotionType::MOTION_DYNAMIC, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);
			}
		}

		if (playSound) {
			PlayPhysicsSound(palmPos, Config::options.useLoudSoundGrab);
		}

		if (g_vrikInterface && Config::options.disableHeadBobbingWhileGrabbed && isHeadBobbingSavedCount++ == 0) {
			savedHeadBobbingHeight = g_vrikInterface->getSettingDouble("headBobbingHeight");
			g_vrikInterface->setSettingDouble("headBobbingHeight", 0.0);
		}

		bool usePhysicsGrab = ShouldUsePhysicsBasedGrab(objRoot, collidableNode, selectedObj->baseForm);

		StartNearbyDamping(world);

		if (selectedObject.isActor) {
			if (Config::options.overrideBodyCollision) {
				CollisionInfo::SetCollisionGroupDownstream(objRoot, playerCollisionGroup, collisionMapState);
			}
		}
		else {
			CollisionInfo::SetCollisionInfoDownstream(objRoot, playerCollisionGroup, collisionMapState);
		}

		NiTransform originalTransform = collidableNode->m_worldTransform;
		NiTransform adjustedTransform = originalTransform;

		if (initialTransform) {
			//UpdateKeyframedNode(collidableNode, *initialTransform);
			adjustedTransform = *initialTransform;
		}
		else if (shouldMoveHandBack) {
			adjustedTransform.pos += (palmDirection * Config::options.pulledGrabHandAdjustDistance) / havokWorldScale;
			//UpdateKeyframedNode(collidableNode, newTransform);
		}

		std::vector<TriangleData> triangles; // tris are in worldspace

		std::unordered_set<NiAVObject *> nodesToSkinTo;
		if (selectedObject.isActor) {
			// For ragdolls, skin to all nodes
			GetDownstreamNodes(objRoot, nodesToSkinTo);
		}
		else {
			// Skin only to nodes that are the collision node we grabbed or attached to it
			GetDownstreamNodesNoCollision(collidableNode, nodesToSkinTo);
		}

		double t = GetTime();
		 GetSkinnedTriangles(objRoot, triangles, nodesToSkinTo);
		_MESSAGE("Time spent skinning: %.3f ms", (GetTime() - t) * 1000);

		t = GetTime();
		GetTriangles(objRoot, triangles);
		_MESSAGE("Time spent transforming triangles: %.3f ms", (GetTime() - t) * 1000);

		//DumpVertices(skinnedTriangleLists);

		NiPoint3 triPos, triNormal;
		float closestDist = (std::numeric_limits<float>::max)();
		t = GetTime();
		NiPoint3 startPt = palmPos + originalTransform.pos - adjustedTransform.pos;
		bool success = GetClosestPointOnGraphicsGeometryToLine(triangles, startPt, palmDirection, &triPos, &triNormal, &closestDist);

		if (success) {
			// We've got a point on the graphics geometry

			ptPos = triPos;

			NiPoint3 palmToPoint = ptPos - palmPos;

			if (g_vrikInterface) {
				double handSize = g_vrikInterface->getSettingDouble("handSize");
				float handScale = handSize / 0.85; // 0.85 is the vrik default hand size

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

				auto FingerCheck = [this, player, &fingerNormalsWorldspace, &fingerZeroAngleVecsWorldspace, handScale, &triangles, &originalTransform, &palmToPoint]
				(int fingerIndex) -> float
				{
					// TODO: We should probably save the finger positions in hand-space when generating finger curves, instead of just using the 1st person finger position
					NiPointer<NiAVObject> startFinger = player->GetNiRootNode(1)->GetObjectByName(&fingerNodeNames[fingerIndex][0].data);
					if (startFinger) {
						NiPoint3 zeroAngleVectorWorldspace = fingerZeroAngleVecsWorldspace[fingerIndex];
						NiPoint3 normalWorldspace = fingerNormalsWorldspace[fingerIndex];

						NiPoint3 startFingerPos = startFinger->m_worldTransform.pos;
						startFingerPos += palmToPoint; // Move the finger up to where the hand would be if it was already holding the object

						_MESSAGE("finger %d", fingerIndex);

						float curveValOrAngle; // If negative, it's an angle. Otherwise curveVal
						bool intersects = GetIntersections(triangles, fingerIndex, handScale, startFingerPos, normalWorldspace, zeroAngleVectorWorldspace,
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

				// Update transform to snap to the hand
				// Not needed since now we update the finger position manually, and doing it this way breaks skinning since we already skinned the object
				//UpdateKeyframedNode(collidableNode, desiredTransform);

				std::array<float, 5> fingerData;
				for (int i = 0; i < fingerData.size(); i++) {
					fingerData[i] = FingerCheck(i);
				}

				// Reset to original transform
				//UpdateKeyframedNode(collidableNode, originalTransform);

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
				wereFingersSet = true;
				//g_vrikInterface->setFingerRange(isLeft, fingerRanges[0], 1, fingerRanges[1], 1, fingerRanges[2], 1, fingerRanges[3], 1, fingerRanges[4], 1);
			}

			_MESSAGE("Geometry processing time: %.3f ms", (GetTime() - t) * 1000);
		}

		NiTransform inverseHand;
		handNode->m_worldTransform.Invert(inverseHand);

		if (usePhysicsGrab) {
			//bhkCollisionObject *collisionObject = GetCollisionObject(collidableNode);
			//collisionObject->flags &= ~(1 << 7); // no kSyncOnUpdate

			// Sync up the collision's transform with the node's
			hkVector4 hkPos = NiPointToHkVector(collidableNode->m_worldTransform.pos * havokWorldScale);
			NiQuaternion nodeRotation;
			NiMatrixToNiQuaternion(nodeRotation, collidableNode->m_worldTransform.rot);
			hkQuaternion hkQuat = NiQuatToHkQuat(nodeRotation);
			hkQuat.normalize();
			selectedObject.rigidBody->setPositionAndRotation(hkPos, hkQuat);

			// Use havok object pos / rot since we set that while holding it, and it can be slightly off from the ninode pos
			hkTransform &hkTransform = selectedObject.rigidBody->hkBody->m_motion.m_motionState.m_transform;
			NiTransform desiredHavokTransform = collidableNode->m_worldTransform;
			desiredHavokTransform.pos = HkVectorToNiPoint(hkTransform.m_translation) / havokWorldScale;
			HkMatrixToNiMatrix(hkTransform.m_rotation, desiredHavokTransform.rot);

			desiredHavokTransform.pos += palmPos - ptPos;
			desiredHavokTransformHandSpace = inverseHand * desiredHavokTransform;
		}
		else {
			selectedObject.savedMotionType = selectedObject.rigidBody->hkBody->m_motion.m_type;
			selectedObject.savedQuality = selectedObject.collidable->m_broadPhaseHandle.m_objectQualityType;

			bhkRigidBody_setMotionType(selectedObject.rigidBody, hkpMotion::MotionType::MOTION_KEYFRAMED, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY);
		}

		NiTransform desiredNodeTransform = collidableNode->m_worldTransform;
		desiredNodeTransform = collidableNode->m_worldTransform;
		desiredNodeTransform.pos += palmPos - ptPos;
		desiredNodeTransformHandSpace = inverseHand * desiredNodeTransform;

		dampingState = DampingState::Undamped;

		HiggsPluginAPI::TriggerGrabbedCallbacks(isLeft, selectedObj);

		state = usePhysicsGrab ? State::HeldBody : State::HeldInit;

		if (state == State::HeldBody) {
			heldTime = g_currentFrameTime; // we set this only when going into HeldBody and Held, not HeldInit
		}
	}
	else {
		state = State::Idle;
	}

	return wereFingersSet;
}


void Grabber::TransitionPreGrab(TESObjectREFR *selectedObj, bool isExternal)
{
	StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

	UInt32 droppedObjHandle = SpawnEquippedSelectedObject(selectedObj, 0);

	NiPointer<TESObjectREFR> droppedObj;
	if (droppedObjHandle != *g_invalidRefHandle && LookupREFRByHandle(droppedObjHandle, droppedObj)) {
		grabbedTime = g_currentFrameTime;

		Deselect();
		Select(droppedObj);

		isExternalGrab = isExternal;
		state = State::PreGrabItem;
	}
	else {
		state = State::Idle;
	}
}


bool Grabber::IsObjectDepositable(TESObjectREFR *refr, NiAVObject *hmdNode, const NiPoint3 &handPos) const
{
	if (selectedObject.isActor) return false;

	TESForm *baseForm = refr->baseForm;
	if (!baseForm || !baseForm->IsPlayable()) return false;

	auto book = DYNAMIC_CAST(baseForm, TESForm, TESObjectBOOK);
	if (book && (book->data.flags & TESObjectBOOK::Data::kType_CantBeTaken)) {
		return false;
	}

	float speed = 0;
	for (auto velocity : controllerVelocities) {
		speed += VectorLength(velocity);
	}
	speed /= std::size(controllerVelocities);

	if (speed < Config::options.shoulderVelocityThreshold) {
		NiPoint3 rightShoulderPos = hmdNode->m_worldTransform * Config::options.rightShoulderHmdOffset;
		float rightShoulderDistance = VectorLength(handPos - rightShoulderPos);
		if (rightShoulderDistance < Config::options.rightShoulderRadius) {
			return true;
		}
		else {
			NiPoint3 leftShoulderPos = hmdNode->m_worldTransform * Config::options.leftShoulderHmdOffset;
			float leftShoulderDistance = VectorLength(handPos - leftShoulderPos);
			if (leftShoulderDistance < Config::options.leftShoulderRadius) {
				return true;
			}
		}
	}
	
	return false;
}


bool Grabber::IsObjectConsumable(TESObjectREFR *refr, NiAVObject *hmdNode, const NiPoint3 &palmPos) const
{
	TESForm *baseForm = refr->baseForm;
	if (!baseForm || !baseForm->IsPlayable() || (baseForm->formType != kFormType_Potion && baseForm->formType != kFormType_Ingredient && baseForm->formType != kFormType_Book)) {
		return false;
	}

	auto potion = DYNAMIC_CAST(baseForm, TESForm, AlchemyItem);
	if (potion && (potion->IsPoison() && !Config::options.enableDrinkPoison)) {
		return false;
	}

	auto book = DYNAMIC_CAST(baseForm, TESForm, TESObjectBOOK);
	if (book && (book->data.flags & TESObjectBOOK::Data::kType_CantBeTaken)) {
		return false;
	}

	float speed = 0;
	for (auto velocity : controllerVelocities) {
		speed += VectorLength(velocity);
	}
	speed /= std::size(controllerVelocities);

	if (speed < Config::options.mouthVelocityThreshold) {
		NiPoint3 mouthPos = hmdNode->m_worldTransform * Config::options.mouthHmdOffset;
		float mouthDistance = VectorLength(palmPos - mouthPos);
		if (mouthDistance < Config::options.mouthRadius) {
			return true;
		}
	}

	return false;
}


UInt32 Grabber::SpawnEquippedSelectedObject(TESObjectREFR *selectedObj, float zOffsetWhenNotDisconnected)
{
	UInt32 droppedObjHandle = *g_invalidRefHandle;

	Actor *actor = DYNAMIC_CAST(selectedObj, TESObjectREFR, Actor);
	if (actor) {
		// Drop the armor
		TESForm *wornForm = selectedObject.hitForm;
		BaseExtraList *wornExtraData = selectedObject.hitExtraList;
		if (wornForm) {
			TESBoundObject *item = DYNAMIC_CAST(wornForm, TESForm, TESBoundObject);
			if (item) {
				// pump armor form / extra data into actor->RemoveItem (vfunc 0x56)
				// DropObject is vfunc 0xCD
				// Actor::RemoveItem is at 0x607F60

				UInt64 *vtbl = *((UInt64 **)actor);
				if (selectedObject.isDisconnected) {
					// For dropped weapons/shields, make the drop pos / rot equal to where it was before
					NiPoint3 dropLoc = selectedObject.hitNode->m_worldTransform.pos;
					NiPoint3 dropRot = MatrixToEuler(selectedObject.hitNode->m_worldTransform.rot);
					((Actor_RemoveItem)(vtbl[0x56]))(actor, &droppedObjHandle, item, 1, 3, wornExtraData, nullptr, &dropLoc, &dropRot);
					//((Actor_DropObject)(vtbl[0xCD]))(actor, &droppedObjHandle, item, wornExtraData, 1, &dropLoc, &dropRot);
				}
				else {
					NiPoint3 dropLoc = selectedObject.hitNode->m_worldTransform.pos + NiPoint3(0, 0, zOffsetWhenNotDisconnected); // move it up a bit to not collide with the ragdoll too much
					((Actor_RemoveItem)(vtbl[0x56]))(actor, &droppedObjHandle, item, 1, 3, wornExtraData, nullptr, &dropLoc, nullptr);
					//((Actor_DropObject)(vtbl[0xCD]))(actor, &droppedObjHandle, item, wornExtraData, 1, &dropLoc, nullptr);
				}
			}
		}
	}

	return droppedObjHandle;
}


bool Grabber::TransitionGrabExternal(TESObjectREFR *refr)
{
	if (CanGrabObject() && refr) {
		_MESSAGE("External grab");

		if (state == State::SelectedClose) {
			if (g_vrikInterface) {
				g_vrikInterface->restoreFingers(isLeft);
			}
		}

		if (state == State::SelectedClose || state == State::SelectedFar || state == State::SelectionLocked) {
			StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);
		}

		Deselect();
		Select(refr);

		grabbedTime = g_currentFrameTime;

		state = State::GrabExternal;

		return true;
	}

	return false;
}


void Grabber::GrabExternalObject(Grabber &other, bhkWorld &world, TESObjectREFR *selectedObj, NiNode *objRoot, NiAVObject *collidableNode, NiAVObject *handNode, bhkSimpleShapePhantom *sphere, const NiPoint3 &hkPalmPos, const NiPoint3 &palmVector, float havokWorldScale)
{
	selectedObject.point = collidableNode->m_worldTransform.pos; // Fallback to the center of the object

	// Position the object in the direction of the palm for better grab results
	NiTransform initialTransform = collidableNode->m_worldTransform;
	initialTransform.pos = (hkPalmPos + palmVector * 1.0f) / havokWorldScale;

	TransitionHeld(other, world, hkPalmPos, palmVector, selectedObject.point, havokWorldScale, handNode, selectedObj, &initialTransform);

	if (state == State::HeldInit) {
		// Set the transform here to kind of skip the HeldInit state
		NiTransform newTransform = handNode->m_worldTransform * desiredNodeTransformHandSpace;
		UpdateKeyframedNode(collidableNode, newTransform);
	}
}


void Grabber::SetPulledDuration(const NiPoint3 &hkPalmPos, const NiPoint3 &objPoint)
{
	float distance = VectorLength(hkPalmPos - objPoint);

	pullDuration = Config::options.pullDurationA + Config::options.pullDurationB * expf(-Config::options.pullDurationC * distance);

	pulledExpireTime = pullDuration + 1.0f;
}


NiPointer<NiAVObject> Grabber::GetHandNode()
{
	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->GetNiNode()) return nullptr;

	return isLeft ? player->unk3F0[PlayerCharacter::Node::kNode_LeftHandBone] : player->unk3F0[PlayerCharacter::Node::kNode_RightHandBone];
}


void Grabber::Update(Grabber &other, bool allowGrab, NiNode *playerWorldNode, bhkWorld *world)
{
	//_MESSAGE("%s:, pose update", name);

	PlayerCharacter *player = *g_thePlayer;
	if (!player || !player->GetNiNode()) return;

	NiPointer<NiAVObject> handNode = GetHandNode();
	if (!handNode) return;

	handTransform = handNode->m_worldTransform; // Save the old hand transform - we restore it later

	float havokWorldScale = *g_havokWorldScale;

	NiPoint3 handPos = handNode->m_worldTransform.pos;
	NiPoint3 hkHandPos = handPos * havokWorldScale;

	NiPoint3 palmVectorHandspace = Config::options.palmVector;
	if (isLeft) palmVectorHandspace.x *= -1;
	NiPoint3 palmVector = VectorNormalized(handNode->m_worldTransform.rot * palmVectorHandspace);

	NiPoint3 pointingVectorHandspace = Config::options.pointingVector;
	if (isLeft) pointingVectorHandspace.x *= -1;
	NiPoint3 pointingVector = VectorNormalized(handNode->m_worldTransform.rot * pointingVectorHandspace);

	NiPointer<NiAVObject> hmdNode = player->unk3F0[PlayerCharacter::Node::kNode_HmdNode];
	if (!hmdNode) return;

	NiPoint3 hmdPos = hmdNode->m_worldTransform.pos;

	// Skyrim coords: +x: right vector, +y: forward vector, +z: up vector
	NiPoint3 hmdForward = { hmdNode->m_worldTransform.rot.data[0][1], hmdNode->m_worldTransform.rot.data[1][1], hmdNode->m_worldTransform.rot.data[2][1] };

	static BSFixedString comName("NPC COM [COM ]");
	NiPointer<NiAVObject> comNode = player->GetNiRootNode(0)->GetObjectByName(&comName.data);
	if (!comNode) {
		_MESSAGE("No COM [COM ] node on player");
		return;
	}
	auto comRigidBody = GetRigidBody(comNode);
	if (comRigidBody) {
		playerCollisionGroup = comRigidBody->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 16;
	}
	else {
		_MESSAGE("COM node has no collision object");
		return;
	}


	//if (!isLeft) {
	//	UpdateGenerateFingerCurve(handNodeName, fingerNodeNames);
	//}


	UpdateHandCollision(handNode, world);

	UpdateWeaponCollision();

	// Update velocities to this frame
	NiPoint3 playerVelocityWorldspace = (player->pos - prevPlayerPosWorldspace) / *g_deltaTime;

	playerVelocitiesWorldspace.pop_back();
	playerVelocitiesWorldspace.push_front(playerVelocityWorldspace);

	NiPoint3 avgPlayerVelocityWorldspace = std::accumulate(playerVelocitiesWorldspace.begin(), playerVelocitiesWorldspace.end(), NiPoint3()) / playerVelocitiesWorldspace.size();

	if (g_currentFrameTime - pulledTime > pulledExpireTime) {
		EndPull();
	}

	bhkSimpleShapePhantom *sphere = *g_pickSphere;
	if (!sphere) return;

	NiPoint3 palmPos = handNode->m_worldTransform * palmPosHandspace;
	NiPoint3 hkPalmPos = palmPos * havokWorldScale;

	auto sphereShape = (hkpConvexShape *)sphere->phantom->m_collidable.m_shape;
	// Save sphere properties so we can change them and restore them later
	UInt32 filterInfoBefore = sphere->phantom->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo;
	float radiusBefore = sphereShape->getRadius();
	hkVector4 translationBefore = sphere->phantom->m_motionState.getTransform().getTranslation();

	if (externalGrabRequested) {
		TransitionGrabExternal(externalGrabRequestedObject);
		externalGrabRequested = false;
		externalGrabRequestedObject = nullptr;
	}

	if (state == State::GrabFromOtherHand) {
		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
			bool wereFingersSet = TransitionHeld(other, *world, hkPalmPos, palmVector, selectedObject.point, havokWorldScale, handNode, selectedObj);
			if (!wereFingersSet && g_vrikInterface) {
				g_vrikInterface->restoreFingers(isLeft);
			}
		}
		else {
			if (g_vrikInterface) {
				g_vrikInterface->restoreFingers(isLeft);
			}

			state = State::Idle;
		}
	}

	if (state == State::GrabExternal) {
		if (g_currentFrameTime - grabbedTime > Config::options.lootSpawnInTime) { // wait for the item to spawn in
			state = State::Idle;
		}
		else {
			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				NiPointer<NiNode> objRoot = selectedObj->GetNiNode();
				if (objRoot && objRoot->m_parent) {
					auto rigidBody = GetFirstRigidBody(objRoot);
					if (rigidBody) {
						hkpCollidable *collidable = &rigidBody->hkBody->m_collidable;
						NiPointer<NiAVObject> collidableNode = FindCollidableNode(collidable);
						if (collidableNode) {
							selectedObject.rigidBody = rigidBody;
							selectedObject.collidable = &rigidBody->hkBody->m_collidable;

							GrabExternalObject(other, *world, selectedObj, objRoot, collidableNode, handNode, sphere, hkPalmPos, palmVector, havokWorldScale);

							grabRequested = false;
							wasObjectGrabbed = true;
						}
					}
				}
			}
			else {
				state = State::Idle;
			}
		}
	}

	if (state == State::Idle || state == State::SelectedClose || state == State::SelectedFar) {

		// See if there's something near the hand to pick up
		NiPointer<TESObjectREFR> closestObj;
		NiPointer<bhkRigidBody> closestRigidBody;
		hkVector4 closestPoint;

		bool isSelectedNear = FindCloseObject(world, allowGrab, other, hkPalmPos, palmVector, sphere,
			closestObj, closestRigidBody, closestPoint);

		if (!isSelectedNear) {
			// Nothing close by the hand. Check for stuff pointing from the palm
			if (allowGrab) {
				FindFarObject(world, other, hkPalmPos, pointingVector, hmdPos * havokWorldScale, hmdForward, sphere,
					closestObj, closestRigidBody, closestPoint);
			}
		}

		// Check if we should select something new. If yes, stay in a selected state but select the new object
		bool isSelectedThisFrame = false;
		UInt32 prevSelectedHandle = selectedObject.handle;
		if (closestObj) {
			State newState = state; // We only want to set state after the collidable is updated, for threading reasons

			// We save this while the casts hit so that during fade time when the casts don't hit, we still have a point to use.
			selectedObject.point = HkVectorToNiPoint(closestPoint);

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
			NiPointer<NiAVObject> nodeOnWhichToPlayShader = nullptr;
			Actor *actor = DYNAMIC_CAST(closestObj, TESObjectREFR, Actor);
			bool breakStickiness = false;

			if (actor) {
				NiPointer<NiAVObject> hitNode = FindCollidableNode(&closestRigidBody->hkBody->m_collidable);
				if (hitNode) {
					BipedModel *biped = actor->GetBipedSmall();
					if (biped) {
						Biped *bipedData = biped->bipedData;
						if (bipedData) {
							int hitIndex = -1;
							TESForm *hitForm = nullptr;
							bool isDisconnected = false;
							for (int i = 0; i < equippedWeaponSlotBase; i++) {
								// For skinned armor, the nodes are not attached to the skeleton. Find nodes the armor is skinned to and see if one of them was hit
								NiPointer<NiAVObject> geomNode = bipedData->unk10[i].object;
								if (geomNode) {
									TESForm *armorForm = bipedData->unk10[i].armor;
									if (armorForm && armorForm->IsPlayable()) {
										// Now check if we actually hit a node the armor is skinned to
										bool isArmorDisconnected = DoesNodeHaveNode(geomNode, hitNode); // This is mainly for shields that are off the body
										if (IsSkinnedToNode(geomNode, hitNode) || isArmorDisconnected) {
											if (hitIndex == -1 || CompareBipedIndices(hitIndex, i)) {
												hitIndex = i;
												hitForm = armorForm;
												isDisconnected = isArmorDisconnected;
											}
										}
									}
								}
							}
							for (int i = equippedWeaponSlotBase; i < 42; i++) {
								// For equipped weapons, the nodes are attached to the skeleton. Find the nearest parent that has collision and see if it was hit
								NiPointer<NiAVObject> geomNode = bipedData->unk10[i].object;
								bool hasCollision = false;
								if (geomNode) {
									if (DoesNodeHaveNode(geomNode, hitNode)) {
										TESForm *form = bipedData->unk10[i].armor;
										if (form && form->IsPlayable()) {
											// We collided with the weapon, so it must not be attached to the character
											hitIndex = i;
											hitForm = form;
											isDisconnected = true;
											break;
										}
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
											TESForm *form = bipedData->unk10[i].armor;
											if (form && form->IsPlayable()) {
												hitIndex = i;
												hitForm = form;
												isDisconnected = false;
												break;
											}
										}
									}
								}
							}

							if (hitForm && (!Config::options.disableLooting || isDisconnected)) {
								// Make sure the armor we hit is actually equipped. When nothing is equipped, there can still be 'naked' armor in the biped data that's not really equipped armor.

								ExtraContainerChanges* containerChanges = static_cast<ExtraContainerChanges*>(actor->extraData.GetByType(kExtraData_ContainerChanges));
								if (containerChanges) {
									MatchByForm matcher(hitForm);
									EquipData equipData;

									if (hitIndex == 9) { // 9 == shield / left-hand weapon
										equipData = containerChanges->FindEquipped(matcher, false, true);
										if (!equipData.pForm) {
											// Sometimes it's not actually WornLeft...
											equipData = containerChanges->FindEquipped(matcher, true, true);
										}
									}
									else {
										equipData = containerChanges->FindEquipped(matcher, true, true);
									}

									if (equipData.pForm) {
										Biped::Data *hitBipedData = &bipedData->unk10[hitIndex];
										auto hitArmor = DYNAMIC_CAST(hitBipedData->armor, TESForm, TESObjectARMO);
										// If it's armor, make sure it has a name. If it doesn't, it could be FEC, or who knows...
										if (!hitArmor || *hitArmor->fullName.name.data) {
											nodeOnWhichToPlayShader = hitBipedData->object;
											selectedObject.hitForm = hitForm;
											selectedObject.hitExtraList = equipData.pExtraData;
											selectedObject.hitNode = hitNode;
											selectedObject.isDisconnected = isDisconnected;
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

						// Playing the shader is fine if the other hand holds it if it's an actor's armor that's getting selected
						PlaySelectionEffect(selectedObject.handle, nodeOnWhichToPlayShader);
						selectedObject.shaderNode = nodeOnWhichToPlayShader;
					}
				}
			}
			else if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				// No (actor) node selected but refr is still selected

				NiPointer<NiNode> objRoot = selectedObj->GetNiNode();
				if (objRoot) {
					if (selectedObject.handle != prevSelectedHandle) {
						// New refr is selected. Stopping the shader for the previous ref is done earlier.

						selectedObject.shaderNode = nullptr;
						selectedObject.hitNode = nullptr;
						selectedObject.hitForm = nullptr;

						if (!selectedObject.isActor) {
							NiPointer<NiAVObject> hitNode = FindCollidableNode(&closestRigidBody->hkBody->m_collidable);
							if (hitNode) {
								if (!IsSkinnedToNode(objRoot, hitNode)) {
									selectedObject.shaderNode = hitNode;
								}
							}

							if (closestRigidBody != other.selectedObject.rigidBody || !other.CanOtherGrab()) {
								PlaySelectionEffect(selectedObject.handle, selectedObject.shaderNode);
							}
						}
					}
					else if (selectedObject.shaderNode) {
						if (selectedObject.isActor) {
							// Node was selected before (selectedObject.shaderNode), but not now (nodeOnWhichToPlayShader). Stop the shader on that node.
							if (Config::options.disableLooting || breakStickiness) {
								StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

								//PlayShader(selectedObject.handle, nullptr, shader);

								selectedObject.shaderNode = nullptr;
								selectedObject.hitNode = nullptr;
								selectedObject.hitForm = nullptr;
							}
						}
						else {
							NiPointer<NiAVObject> hitNode = FindCollidableNode(&closestRigidBody->hkBody->m_collidable);
							if (hitNode != selectedObject.shaderNode) {
								// Moved nodes on the same reference
								StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

								selectedObject.shaderNode = nullptr;
								selectedObject.hitNode = nullptr;
								selectedObject.hitForm = nullptr;

								if (!IsSkinnedToNode(objRoot, hitNode)) {
									selectedObject.shaderNode = hitNode;
								}

								if (closestRigidBody != other.selectedObject.rigidBody || !other.CanOtherGrab()) {
									PlaySelectionEffect(selectedObject.handle, selectedObject.shaderNode);
								}
							}
						}
					}
					else {
						// Same refr, no node selected before (skinned?)
						NiPointer<NiAVObject> hitNode = FindCollidableNode(&closestRigidBody->hkBody->m_collidable);
						if (hitNode && !IsSkinnedToNode(objRoot, hitNode)) {
							// Only replay the shader if we now have a specific node to play on
							StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

							selectedObject.hitNode = nullptr;
							selectedObject.hitForm = nullptr;
							selectedObject.shaderNode = hitNode;

							if (closestRigidBody != other.selectedObject.rigidBody || !other.CanOtherGrab()) {
								PlaySelectionEffect(selectedObject.handle, selectedObject.shaderNode);
							}
						}
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
							NiPoint3 hkObjPos = HkVectorToNiPoint(translation);
							NiPoint3 relObjPos = hkObjPos - hkHandPos;

							rolloverDisplayTime = g_currentFrameTime;
							initialGrabbedObjRelativePosition = relObjPos;
							pulledPointOffset = selectedObject.point - hkObjPos;

							float hapticStrength = Config::options.selectionLockedStartHapticStrength;
							haptics.QueueHapticEvent(hapticStrength, hapticStrength, Config::options.selectionLockedStartHapticDuration);

							state = State::SelectionLocked;
						}
						else if (state == State::SelectedClose) {
							if (selectedObject.rigidBody == other.selectedObject.rigidBody && other.CanOtherGrab()) {
								if (selectedObject.isActor && selectedObject.hitForm) {
									if (g_vrikInterface) {
										g_vrikInterface->restoreFingers(isLeft);
									}

									rolloverDisplayTime = g_currentFrameTime;

									state = State::LootOtherHand;
								}
								else {
									// Grabbing the object from the other hand - make the other hand drop it and wait
									other.disableDropEvents = true;
									other.idleDesired = true;
									state = State::GrabFromOtherHand;
								}
							}
							else {
								if (selectedObject.hitForm && selectedObject.isDisconnected) {
									// Grabbing a weapon or something that's part of a body
									if (g_vrikInterface) {
										g_vrikInterface->restoreFingers(isLeft);
									}

									TransitionPreGrab(selectedObj);
								}
								else {
									// Grabbing a regular object, not from the other hand or off of a body
									bool wereFingersSet = TransitionHeld(other, *world, hkPalmPos, palmVector, selectedObject.point, havokWorldScale, handNode, selectedObj);
									if (!wereFingersSet && g_vrikInterface) {
										g_vrikInterface->restoreFingers(isLeft);
									}
								}
							}
						}

						grabRequested = false; // Set to false only here, so that you can hold the trigger until the cast hits something valid
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

	if (state == State::SelectionLocked || state == State::HeldInit || state == State::Held || state == State::HeldBody || state == State::LootOtherHand) {

		grabRequested = false; // Consume grab event here as we cannot grab from any of these states and don't want to grab immediately upon exiting them

		// Check if we should drop the object
		if (releaseRequested) {
			releaseRequested = false;
			wasObjectGrabbed = false;
			idleDesired = true;
		}

		if (!allowGrab) {
			idleDesired = true;
		}

		if (idleDesired) {
			idleDesired = false;

			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				NiPointer<NiNode> objRoot = selectedObj->GetNiNode();
				if (objRoot) {
					if (state == State::SelectionLocked || state == State::LootOtherHand) {
						if (state == State::SelectionLocked) {
							float hapticStrength = Config::options.selectionLockedEndHapticStrength;
							haptics.QueueHapticEvent(hapticStrength, hapticStrength, Config::options.selectionLockedEndHapticDuration);
						}

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

						NiPoint3 velocityHandComponent;
						if (largestIndex == 0) {
							// Max is the first value
							velocityHandComponent = controllerVelocities[0];
						}
						else if (largestIndex == controllerVelocities.size() - 1) {
							// Max is the last value
							velocityHandComponent = controllerVelocities[largestIndex];
						}
						else {
							// Regular case - avg 3 values centered at the peak
							velocityHandComponent = (controllerVelocities[largestIndex - 1] + controllerVelocities[largestIndex] + controllerVelocities[largestIndex + 1]) / 3;
						}

						if (VectorLength(velocityHandComponent) > Config::options.throwVelocityThreshold) {
							velocityHandComponent *= Config::options.throwVelocityBoostFactor;
						}

						NiPoint3 velocityPlayerComponent = avgPlayerVelocityWorldspace * havokWorldScale;

						NiPoint3 totalVelocity = velocityPlayerComponent + velocityHandComponent; // add the player velocity

						bool velocityAboveThreshold = VectorLength(totalVelocity) > Config::options.throwVelocityThreshold;
						bool collideWithHandWhenLettingGo = !velocityAboveThreshold;

						if (state == State::HeldBody) {
							if (selectedObject.isActor) {
								if (Config::options.overrideBodyCollision) {
									ResetCollisionGroupDownstream(objRoot, collisionMapState, nullptr);
								}
							}
							else {
								ResetCollisionInfoDownstream(objRoot, collisionMapState, nullptr, collideWithHandWhenLettingGo);
							}
						}
						if (state == State::Held || state == State::HeldInit) {
							ResetCollisionInfoDownstream(objRoot, collisionMapState, selectedObject.collidable, collideWithHandWhenLettingGo); // skip the node we grabbed, we handle that below
							ResetCollisionInfoKeyframed(selectedObject.rigidBody, selectedObject.savedMotionType, selectedObject.savedQuality, collisionMapState, collideWithHandWhenLettingGo);
						}

						bhkRigidBody_setActivated(selectedObject.rigidBody, true);
						selectedObject.rigidBody->hkBody->m_motion.m_linearVelocity = NiPointToHkVector(totalVelocity);

						if ((state == State::Held || state == State::HeldBody) && IsObjectConsumable(selectedObj, hmdNode, palmPos) && !disableDropEvents) {
							// Object dropped at the mouth

							TESForm *baseForm = selectedObj->baseForm;

							HiggsPluginAPI::TriggerConsumedCallbacks(isLeft, baseForm); // Do this before activating it so that a callback could still get the held object

							UInt32 count = BSExtraList_GetCount(&selectedObj->extraData);
							TESObjectREFR_Activate(selectedObj, player, 0, 0, count, false);

							if (baseForm) {
								if (baseForm->formType != kFormType_Book) {
									papyrusActor::EquipItemEx(player, baseForm, 0, false, true);
								}
								else {
									auto book = DYNAMIC_CAST(baseForm, TESForm, TESObjectBOOK);
									if (book && book->data.flags & TESObjectBOOK::Data::kType_Spell) {
										EquipManager *equipManager = EquipManager::GetSingleton();
										if (equipManager) {
											ExtraContainerChanges* containerChanges = static_cast<ExtraContainerChanges*>(player->extraData.GetByType(kExtraData_ContainerChanges));
											ExtraContainerChanges::Data* containerData = containerChanges ? containerChanges->data : nullptr;
											if (containerData) {
												InventoryEntryData *entryData = containerData->FindItemEntry(book);
												if (entryData) {
													EquipManager_EquipEntryData(equipManager, player, entryData, nullptr);
													if (TESObjectBOOK_LearnSpell(book, player)) {
														UInt64 *vtbl = *((UInt64 **)player);
														UInt32 droppedObjHandle = *g_invalidRefHandle;
														BaseExtraList *extraList = entryData->extendDataList ? entryData->extendDataList->GetNthItem(0) : nullptr;
														((Actor_RemoveItem)(vtbl[0x56]))(player, &droppedObjHandle, book, 1, 0, extraList, nullptr, nullptr, nullptr);
													}
												}
											}
										}
									}
								}
							}

							haptics.QueueHapticEvent(Config::options.mouthDropHapticStrength, 0, Config::options.mouthDropHapticFadeTime);
						}
						else if ((state == State::Held || state == State::HeldBody) && IsObjectDepositable(selectedObj, hmdNode, handPos) && !disableDropEvents) {
							// Object deposited in the shoulder

							UInt32 count = BSExtraList_GetCount(&selectedObj->extraData);

							TESForm *baseForm = selectedObj->baseForm;

							HiggsPluginAPI::TriggerStashedCallbacks(isLeft, baseForm); // Do this before activating it so that a callback could still get the held object

							if (Config::options.skipActivateBooks && baseForm && baseForm->formType == kFormType_Book) {
								// PickUpObject is vfunc 0xCE

								// PickUpObject is unsafe for random objects - some stuff will be 'picked up' but not show up in inventory, like moveablestatics.
								// Some stuff like containers go into the inventory but then when dropped again they essentially are reset with new items.
								// We do want to directly pick up books though, since otherwise we get the book prompt.
								UInt64 *vtbl = *((UInt64 **)player);
								((Actor_PickUpObject)(vtbl[0xCE]))(player, selectedObj, count, false, true);
							}
							else {
								TESObjectREFR_Activate(selectedObj, player, 0, 0, count, false);
							}

							haptics.QueueHapticEvent(Config::options.shoulderDropHapticStrength, 0, Config::options.shoulderDropHapticFadeTime);
						}
						else {
							float mass = NiAVObject_GetMass(FindCollidableNode(selectedObject.collidable), 0);
							float hapticStrength = min(1.0f, Config::options.grabBaseHapticStrength + Config::options.grabProportionalHapticStrength * max(0.0f, powf(mass, Config::options.grabHapticMassExponent)));
							haptics.QueueHapticEvent(hapticStrength, 0, Config::options.grabHapticFadeTime);

							if (!disableDropEvents) {
								//ObjectReference_SetActorCause(VM_REGISTRY, 0, selectedObj, player); // doesn't make people that your object hit get mad at you unfortunately

								PlayPhysicsSound(palmPos, Config::options.useLoudSoundDrop);

								HiggsPluginAPI::TriggerDroppedCallbacks(isLeft, selectedObj);
							}
						}
					}
				}
			}

			if (state == State::HeldInit || state == State::Held || state == State::HeldBody) {
				// Do all this stuff even if the refr has been deleted / 3d unloaded

				ResetNearbyDamping();

				if (g_vrikInterface) {
					g_vrikInterface->restoreFingers(isLeft);

					if (--isHeadBobbingSavedCount == 0) {
						g_vrikInterface->setSettingDouble("headBobbingHeight", savedHeadBobbingHeight);
					}
				}
			}

			Deselect();
		}
	}

	if (state == State::SelectionLocked) {
		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
			NiPointer<NiNode> objRoot = selectedObj->GetNiNode();
			if (objRoot) {
				hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;

				auto TransitionPulled = [this, &other, &objRoot, &hkPalmPos, motion, selectedObj, handNode, havokWorldScale]()
				{
					StopSelectionEffect(selectedObject.handle, selectedObject.shaderNode);

					pulledTime = g_currentFrameTime;

					if (selectedObject.hitForm) {
						// Trying to pull armor off a body

						UInt32 droppedObjHandle = SpawnEquippedSelectedObject(selectedObj, 20);

						NiPointer<TESObjectREFR> droppedObj;
						if (droppedObjHandle != *g_invalidRefHandle && LookupREFRByHandle(droppedObjHandle, droppedObj)) {
							if (!selectedObject.isDisconnected) {
								pulledPointOffset = { 0, 0, 0 }; // point offset doesn't make sense when we are pulling something other than what we had selected
							}

							Deselect();
							Select(droppedObj);

							state = State::PrePullItem;
						}
						else {
							state = State::Idle;
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
								auto rigidBody = GetFirstRigidBody(objRoot);
								if (rigidBody) {
									// Do not use grabbedObject.collidable here, as sometimes we end up grabbing the phantom shape of the projectile instead of the 3D one
									auto collidable = &rigidBody->hkBody->m_collidable;
									// Projectiles do not interact with collision usually. We need to change the filter to make them interact.
									collidable->m_broadPhaseHandle.m_collisionFilterInfo = (((UInt32)playerCollisionGroup) << 16) | 5; // player collision group, 'weapon' collision layer
									// Projectiles have 'Fixed' motion type by default, making them unmovable
									bhkRigidBody_setMotionType(rigidBody, hkpMotion::MotionType::MOTION_DYNAMIC, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);
								}
							}

							CollisionInfo::SetCollisionInfoDownstream(objRoot, playerCollisionGroup, CollisionInfo::State::Unheld);
						}

						float hapticStrength = min(1.0f, Config::options.selectionLockedBaseHapticStrength + Config::options.selectionLockedProportionalHapticStrength);
						haptics.QueueHapticEvent(hapticStrength, 0, Config::options.pullHapticFadeTime);

						NiPoint3 hkObjPos = HkVectorToNiPoint(motion->m_motionState.m_transform.m_translation);
						NiPoint3 objPoint = (hkObjPos + pulledPointOffset) * *g_inverseHavokWorldScale;
						PlayPhysicsSound(objPoint, Config::options.useLoudSoundPull);

						SetPulledDuration(hkPalmPos, objPoint * havokWorldScale);

						HiggsPluginAPI::TriggerPulledCallbacks(isLeft, selectedObj);

						state = State::Pulled;
					}
				};

				NiPointer<TESObjectREFR> closestObj;
				NiPointer<bhkRigidBody> closestRigidBody;
				hkVector4 closestPoint;
				bool isSelectedNear = FindCloseObject(world, allowGrab, other, hkPalmPos, palmVector, sphere,
					closestObj, closestRigidBody, closestPoint);

				// Allow us to go to held if we had the thing selected from a distance and it came closer
				if (isSelectedNear && closestRigidBody == selectedObject.rigidBody) {
					if (selectedObject.hitForm && selectedObject.isDisconnected) {
						// Grabbing a weapon or something that's part of a body
						TransitionPreGrab(selectedObj);
					}
					else {
						// Grabbing a regular object, not from the other hand or off of a body
						TransitionHeld(other, *world, hkPalmPos, palmVector, HkVectorToNiPoint(closestPoint), havokWorldScale, handNode, selectedObj);
					}
				}

				if (!isSelectedNear) {
					hkVector4 translation = motion->m_motionState.m_transform.m_translation;
					NiPoint3 hkObjPos = HkVectorToNiPoint(translation);
					NiPoint3 relObjPos = hkObjPos - hkHandPos;

					NiPoint3 controllerVelocity = controllerVelocities[0];
					float controllerSpeedDirectionalized = DotProduct(controllerVelocity, VectorNormalized(-relObjPos));

					if (controllerSpeedDirectionalized > Config::options.pullSpeedThreshold) {
						if (IsObjectPullable()) {
							TransitionPulled();
						}
					}
				}
			}
			else {
				state = State::Idle;
			}
		}
		else {
			state = State::Idle;
		}
	}

	if (state == State::LootOtherHand) {
		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
			NiPointer<NiNode> objRoot = selectedObj->GetNiNode();
			if (objRoot) {
				if (other.HasHeldObject() && other.selectedObject.rigidBody == selectedObject.rigidBody) {
					// Other hand is still holding this object

					hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;

					NiPoint3 controllerVelocity = controllerVelocities[0];
					float controllerSpeed = VectorLength(controllerVelocity);

					if (controllerSpeed > Config::options.pullSpeedThreshold) {
						float hapticStrength = min(1.0f, Config::options.selectionLockedBaseHapticStrength + Config::options.selectionLockedProportionalHapticStrength);
						haptics.QueueHapticEvent(hapticStrength, 0, Config::options.pullHapticFadeTime);

						TransitionPreGrab(selectedObj, true);
					}
					else {
						float hapticStrength = min(1.0f, Config::options.selectionLockedBaseHapticStrength + Config::options.selectionLockedProportionalHapticStrength * max(0, (controllerSpeed / Config::options.pullSpeedThreshold)));
						haptics.QueueHapticPulse(hapticStrength);
					}
				}
				else {
					// Other hand let go - we grab it then
					TransitionHeld(other, *world, hkPalmPos, palmVector, selectedObject.point, havokWorldScale, handNode, selectedObj);
				}
			}
			else {
				state = State::Idle;
			}
		}
		else {
			state = State::Idle;
		}
	}

	if (state == State::PreGrabItem) {
		if (g_currentFrameTime - grabbedTime > Config::options.lootSpawnInTime) { // wait for the item to spawn in
			state = State::Idle;
		}
		else {
			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				NiPointer<NiNode> objRoot = selectedObj->GetNiNode();
				if (objRoot && objRoot->m_parent) {
					// Transition to grabbed with the newly spawned item

					auto rigidBody = GetFirstRigidBody(objRoot);
					if (rigidBody) {
						hkpCollidable *collidable = &rigidBody->hkBody->m_collidable;
						NiPointer<NiAVObject> collidableNode = FindCollidableNode(collidable);
						if (collidableNode) {
							selectedObject.rigidBody = rigidBody;
							selectedObject.collidable = &rigidBody->hkBody->m_collidable;

							// Set owner to the player so it doesn't count as stealing
							TESObjectREFR_SetActorOwner(VM_REGISTRY, 0, selectedObj, player->baseForm);

							if (isExternalGrab) {
								GrabExternalObject(other, *world, selectedObj, objRoot, collidableNode, handNode, sphere, hkPalmPos, palmVector, havokWorldScale);
							}
							else {
								TransitionHeld(other, *world, hkPalmPos, palmVector, selectedObject.point, havokWorldScale, handNode, selectedObj);
							}
						}
					}
				}
			}
		}
	}

	if (state == State::PrePullItem) {
		if (g_currentFrameTime - pulledTime > Config::options.lootSpawnInTime) { // wait for the item to spawn in
			state = State::Idle;
		}
		else {
			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				NiPointer<NiNode> objRoot = selectedObj->GetNiNode();
				if (objRoot && objRoot->m_parent) {
					// Transition to pulled with the newly spawned item

					auto rigidBody = GetFirstRigidBody(objRoot);
					if (rigidBody) {
						selectedObject.rigidBody = rigidBody;
						selectedObject.collidable = &rigidBody->hkBody->m_collidable;

						// Set owner to the player so it doesn't count as stealing
						TESObjectREFR_SetActorOwner(VM_REGISTRY, 0, selectedObj, player->baseForm);

						// Cancel an existing pulled collision reset
						EndPull();

						pulledObject.handle = selectedObject.handle;
						pulledObject.rigidBody = selectedObject.rigidBody;

						hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;
						pulledObject.savedAngularDamping = motion->m_motionState.m_angularDamping;
						motion->m_motionState.m_angularDamping = hkHalf(Config::options.pulledAngularDamping);
						CollisionInfo::SetCollisionInfoDownstream(objRoot, playerCollisionGroup, CollisionInfo::State::Unheld);

						pulledTime = g_currentFrameTime;

						float hapticStrength = min(1.0f, Config::options.selectionLockedBaseHapticStrength + Config::options.selectionLockedProportionalHapticStrength);
						haptics.QueueHapticEvent(hapticStrength, 0, Config::options.pullHapticFadeTime);

						NiPoint3 hkObjPos = HkVectorToNiPoint(motion->m_motionState.m_transform.m_translation);
						NiPoint3 objPoint = (hkObjPos + pulledPointOffset) * *g_inverseHavokWorldScale;
						PlayPhysicsSound(objPoint, Config::options.useLoudSoundPull);

						SetPulledDuration(hkPalmPos, objPoint * havokWorldScale);

						HiggsPluginAPI::TriggerPulledCallbacks(isLeft, selectedObj);

						state = State::Pulled;
					}
				}
			}
		}
	}

	if (state == State::Pulled) {
		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
			NiPointer<NiNode> objRoot = selectedObj->GetNiNode();
			if (objRoot) {
				hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;
				NiPoint3 hkObjPos = HkVectorToNiPoint(motion->m_motionState.m_transform.m_translation);

				NiPoint3 objPoint = hkObjPos + pulledPointOffset;

				// Apply a predicted velocity to reach the destination, for a few frames after pulling starts.
				// Why for a few frames? Because then if it's next to something, it has a few frames to push it out of the way instead of just flopping right away

				double elapsedPullTime = g_currentFrameTime - pulledTime;
				if (elapsedPullTime <= Config::options.pullApplyVelocityTime) {
					if (elapsedPullTime <= Config::options.pullTrackHandTime) {
						pullTarget = hkPalmPos + NiPoint3(0, 0, Config::options.pullDestinationZOffset); // Add an extra few cm up so that the object doesn't undershoot
					}

					float duration = pullDuration - elapsedPullTime;

					NiPoint3 horizontalDelta = pullTarget - objPoint;
					horizontalDelta.z = 0;
					NiPoint3 velocity = horizontalDelta / duration;
					float verticalDelta = pullTarget.z - objPoint.z;
					velocity.z = 0.5f * 9.81f * duration + verticalDelta / duration;

					NiPointer<NiAVObject> n = FindCollidableNode(selectedObject.collidable);
					if (n && DoesNodeHaveConstraint(objRoot, n)) {
						// TODO: Set velocity for only the connected component of constrained objects containing this one, not all in the refr
						SetVelocityDownstream(objRoot, NiPointToHkVector(velocity));
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
		else {
			state = State::Idle;
		}
	}

	if (state == State::HeldInit || state == State::Held) {
		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->GetNiNode()) {
			NiPointer<NiAVObject> n = FindCollidableNode(selectedObject.collidable);
			if (n) {
				NiTransform newTransform = handNode->m_worldTransform * desiredNodeTransformHandSpace;

				if (state == State::HeldInit) {
					if (g_currentFrameTime - grabbedTime > Config::options.grabStartMaxTime) {
						// Taking too long to move the object to the hand, so just snap it there
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

						NiPoint3 deltaPlayerPos = player->pos - prevPlayerPosWorldspace;
						NiPoint3 currentPosCompensated = n->m_worldTransform.pos + deltaPlayerPos;

						NiPoint3 deltaDir = VectorNormalized(newTransform.pos - currentPosCompensated);
						NiPoint3 deltaPos = deltaDir * Config::options.grabStartSpeed * *g_deltaTime;

						bool doRotation = deltaAngle < quatAngle;
						bool doTranslation = VectorLengthSquared(deltaPos) < VectorLengthSquared(newTransform.pos - currentPosCompensated);

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
								newTransform.pos = currentPosCompensated + deltaPos;
							}

							UpdateKeyframedNode(n, newTransform);
						}
						else {
							// Both position and rotation are close enough to their final values - we're done
							heldTime = g_currentFrameTime;
							state = State::Held;
						}
					}
				}

				if (state == State::Held) {
					UpdateKeyframedNode(n, newTransform);

					if (g_currentFrameTime - heldTime > Config::options.grabFreezeNearbyVelocityTime) {
						ResetNearbyDamping();
					}

					if (IsObjectConsumable(selectedObj, hmdNode, palmPos)) {
						haptics.QueueHapticPulse(Config::options.mouthConstantHapticStrength);
					}
					else if (IsObjectDepositable(selectedObj, hmdNode, handPos)) {
						haptics.QueueHapticPulse(Config::options.shoulderConstantHapticStrength);
					}
				}
			}
		}
		else {
			if (g_vrikInterface) {
				g_vrikInterface->restoreFingers(isLeft);

				if (--isHeadBobbingSavedCount == 0) {
					g_vrikInterface->setSettingDouble("headBobbingHeight", savedHeadBobbingHeight);
				}
			}

			ResetNearbyDamping();

			state = State::Idle;
		}
	}

	if (state == State::HeldBody) {
		NiPointer<TESObjectREFR> selectedObj;
		if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->GetNiNode()) {
			NiPointer<NiAVObject> collidableNode = FindCollidableNode(selectedObject.collidable);
			if (collidableNode) {
				// Update the hand to match the object
				NiTransform heldTransform = collidableNode->m_worldTransform; // gets the scale

				if (!selectedObject.isActor) {
					hkTransform &heldHkTransform = selectedObject.rigidBody->hkBody->m_motion.m_motionState.m_transform; // try approxTransformAt() instead?
					heldTransform.pos = HkVectorToNiPoint(heldHkTransform.m_translation) / *g_havokWorldScale;
					HkMatrixToNiMatrix(heldHkTransform.m_rotation, heldTransform.rot);
				}

				NiTransform inverseDesired;
				desiredHavokTransformHandSpace.Invert(inverseDesired);

				adjustedHandTransform = heldTransform * inverseDesired;

				float maxHandDistance = Config::options.maxHandDistance / havokWorldScale;
				if (g_currentFrameTime - heldTime > Config::options.physicsGrabInitTime && VectorLength(adjustedHandTransform.pos - handTransform.pos) > maxHandDistance) {
					// Hand is too far from the actual hand's position in real life
					idleDesired = true;
					disableDropEvents = true; // Prevent stuff like eating or stashing when the object is dropped like this
				}
				else {
					// Not too far away. Update hand to object and set object velocity to target.
					if (g_isVrikPresent) {
						UpdateNodeTransformLocal(handNode, adjustedHandTransform);
						NiAVObject::ControllerUpdateContext ctx{ 0, 0 };
						NiAVObject_UpdateObjectUpwards(handNode, &ctx);
					}
					else {
						// When not using vrik, we need to update the clavicle to move the hand just as beth does, otherwise only part of the "hand" moves and we get stretched verts
						NiPointer<NiAVObject> clavicle = isLeft ? player->unk3F0[PlayerCharacter::Node::kNode_LeftCavicle] : player->unk3F0[PlayerCharacter::Node::kNode_RightCavicle];
						if (clavicle) {
							NiTransform identity;
							UpdateClavicleToTransformHand(clavicle, handNode, &adjustedHandTransform, &identity);
						}
					}

					// Update object velocity to go where we want it
					bhkRigidBody_setActivated(selectedObject.rigidBody, true);
					NiTransform newTransform = handTransform * (selectedObject.isActor ? desiredNodeTransformHandSpace : desiredHavokTransformHandSpace);
					NiPoint3 desiredPos = newTransform.pos * havokWorldScale;
					NiPoint3 playerHkVelocity = avgPlayerVelocityWorldspace * havokWorldScale;
					desiredPos += playerHkVelocity * *g_deltaTime;
					hkRotation desiredRot;
					NiMatrixToHkMatrix(newTransform.rot, desiredRot);
					hkQuaternion desiredQuat;
					desiredQuat.setFromRotationSimd(desiredRot);
					hkpKeyFrameUtility_applyHardKeyFrame(NiPointToHkVector(desiredPos), desiredQuat, 1.0f / *g_deltaTime, selectedObject.rigidBody->hkBody);

					// Now, check if in general, the object has been moving as we want it to be. If it isn't, then it's likely blocked by something and so we need to damp its velocities.

					NiPoint3 currentVelocity = HkVectorToNiPoint(selectedObject.rigidBody->hkBody->m_motion.m_linearVelocity);
					NiPoint3 currentVelocityPlayerspace = currentVelocity - playerHkVelocity;

					NiPoint3 hkPlayerPos = player->pos * havokWorldScale;
					NiPoint3 heldObjPosPlayerspace = HkVectorToNiPoint(selectedObject.rigidBody->hkBody->m_motion.m_motionState.m_sweptTransform.m_centerOfMass1) - hkPlayerPos;

					// TODO: Check directionalized velocity progress, not just magnitude?

					bool isNontrivialVelocity = VectorLength(currentVelocityPlayerspace) > Config::options.minVelocityToPotentiallyDamp;
					bool isObjectPreventedFromMoving = VectorLength(heldObjPosPlayerspace - prevHeldObjPosPlayerspace) < Config::options.minDampedRequiredVelocityProportion * VectorLength(prevHeldObjVelocityPlayerspace) * *g_deltaTime;

					bool shouldDamp = isNontrivialVelocity && isObjectPreventedFromMoving;

					if (dampingState == DampingState::Undamped) {
						if (!Config::options.disableDampedGrab && shouldDamp && (!selectedObject.isActor || !Config::options.disableDampedGrabForBodies)) {
							dampingState = DampingState::PreDamp;
							preDampTime = g_currentFrameTime;
						}
					}

					if (dampingState == DampingState::PreDamp) {
						if (shouldDamp) {
							if (g_currentFrameTime - preDampTime > Config::options.preDampVelocityTime) {
								dampingState = DampingState::Damped;
							}
						}
						else {
							dampingState = DampingState::Undamped;
						}
					}

					if (dampingState == DampingState::Damped || dampingState == DampingState::TryLeaveDamped) {
						if (dampingState == DampingState::Damped) {
							if (!shouldDamp) {
								dampingState = DampingState::TryLeaveDamped;
								tryLeaveDampedTime = g_currentFrameTime;
							}
						}
						if (dampingState == DampingState::TryLeaveDamped) {
							if (shouldDamp) {
								dampingState = DampingState::Damped;
							}
							else {
								if (g_currentFrameTime - tryLeaveDampedTime > Config::options.tryLeaveDampedTime) {
									dampingState = DampingState::Undamped;
								}
							}
						}

						float speed = powf(VectorLength(currentVelocityPlayerspace), Config::options.dampedLinearVelocityExponent) * Config::options.dampedLinearVelocityMultiplier;
						NiPoint3 newVelocityPlayerspace = VectorNormalized(currentVelocityPlayerspace) * speed;

						selectedObject.rigidBody->hkBody->m_motion.m_linearVelocity = NiPointToHkVector(newVelocityPlayerspace + playerHkVelocity);
						_MESSAGE("%s: Damped %.2f", name, speed);
						// TODO: Damp angular velocity separately from linear velocity
						NiPoint3 currentAngularVelocity = HkVectorToNiPoint(selectedObject.rigidBody->hkBody->m_motion.m_angularVelocity);
						selectedObject.rigidBody->hkBody->m_motion.m_angularVelocity = NiPointToHkVector(currentAngularVelocity * Config::options.dampedAngularVelocityMultiplier);
					}

					prevHeldObjPosPlayerspace = heldObjPosPlayerspace;
					prevHeldObjVelocityPlayerspace = HkVectorToNiPoint(selectedObject.rigidBody->hkBody->m_motion.m_linearVelocity) - playerHkVelocity; // potentially damped - need to set here after keyframe and damping has occurred

					if (IsObjectConsumable(selectedObj, hmdNode, palmPos)) {
						haptics.QueueHapticPulse(Config::options.mouthConstantHapticStrength);
					}
					else if (IsObjectDepositable(selectedObj, hmdNode, handPos)) {
						haptics.QueueHapticPulse(Config::options.shoulderConstantHapticStrength);
					}
				}
			}

			if (g_currentFrameTime - heldTime > Config::options.grabFreezeNearbyVelocityTime) {
				ResetNearbyDamping();
			}
		}
		else {
			if (g_vrikInterface) {
				g_vrikInterface->restoreFingers(isLeft);

				if (--isHeadBobbingSavedCount == 0) {
					g_vrikInterface->setSettingDouble("headBobbingHeight", savedHeadBobbingHeight);
				}
			}

			ResetNearbyDamping();

			state = State::Idle;
		}
	}

	//if (state != prevState) {
	//	_MESSAGE("%s: %d -> %d", name, prevState, state);
	//}

	prevState = state;
	prevPlayerPosWorldspace = player->pos;
}


void Grabber::ControllerStateUpdate(uint32_t unControllerDeviceIndex, vr_src::VRControllerState001_t *pControllerState, bool allowGrab)
{
	bool wasTriggerDisabledLastFrame = isTriggerDisabled;
	isTriggerDisabled = false;

	// Check inputs and calculate rising/falling edge even if a menu is open
	bool triggerDownBefore = triggerDown;
	bool gripDownBefore = gripDown;

	uint64_t triggerMask = vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_SteamVR_Trigger);
	uint64_t gripMask = vr_src::ButtonMaskFromId(vr_src::EVRButtonId::k_EButton_Grip);

	// Check if the trigger is pressed
	triggerDown = Config::options.enableTrigger && (pControllerState->ulButtonPressed & triggerMask);
	gripDown = Config::options.enableGrip && (pControllerState->ulButtonPressed & gripMask);

	bool triggerRisingEdge = triggerDown && !triggerDownBefore;
	bool triggerFallingEdge = !triggerDown && triggerDownBefore;

	bool gripRisingEdge = gripDown && !gripDownBefore;
	bool gripFallingEdge = !gripDown && gripDownBefore;

	if (MenuChecker::isGameStopped()) return;

	PlayerCharacter *pc = *g_thePlayer;

	bool isFallingEdge = (triggerFallingEdge || gripFallingEdge) && !triggerDown && !gripDown;

	if (isFallingEdge) {
		// Set this even in idle, if something gets externally grabbed and we were holding the trigger or something
		releaseRequested = true;
	}

	// Only advance states if no menus are open
	if (inputState == InputState::Idle) {
		if ((triggerRisingEdge || gripRisingEdge) && allowGrab) {
			grabRequestedTime = g_currentFrameTime;
			grabRequested = true;

			inputTrigger = false;
			inputGrip = false;

			if (triggerRisingEdge) {
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
		if (isFallingEdge) {
			if (wasObjectGrabbed) {
				inputState = InputState::Idle;
			}
			else {
				forceInputTime = g_currentFrameTime;
				inputState = InputState::Force;
			}
		}
		else {
			if (wasObjectGrabbed) {
				inputState = InputState::Block;
			}
			else {
				double currentTime = g_currentFrameTime;
				if (currentTime - grabRequestedTime <= Config::options.triggerPressedLeewayTime) {
					if (triggerRisingEdge) {
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
					forceInputTime = g_currentFrameTime;
					inputState = InputState::Force;
				}
			}
		}
	}

	if (inputState == InputState::Block) {
		if (isFallingEdge) {
			double currentTime = g_currentFrameTime;
			if (state == State::SelectionLocked && currentTime - grabRequestedTime <= Config::options.inputLeewayTime) {
				forceInputTime = g_currentFrameTime;
				inputState = InputState::Force;
			}
			else {
				inputState = InputState::Idle;
			}
		}
		else {
			if (Config::options.enableTrigger) {
				pControllerState->ulButtonPressed &= ~triggerMask;
			}

			if (Config::options.enableGrip) {
				pControllerState->ulButtonPressed &= ~gripMask;
			}
		}
	}

	if (inputState == InputState::Force) {
		if (g_currentFrameTime - forceInputTime <= Config::options.forceInputTime) {
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

	if (Config::options.disableTriggerWhenWeaponsSheathed && pc && !pc->actorState.IsWeaponDrawn() && (wasTriggerDisabledLastFrame || triggerRisingEdge)) {
		pControllerState->ulButtonPressed &= ~triggerMask;
		isTriggerDisabled = true;
	}
}


void Grabber::RestoreHandTransform()
{
	if (!g_isVrikPresent) return;

	if (state == State::HeldBody) {
		NiPointer<NiAVObject> handNode = GetHandNode();
		if (!handNode) return;

		NiTransform newHandTransform = handTransform;
		UpdateNodeTransformLocal(handNode, newHandTransform);
		NiAVObject::ControllerUpdateContext ctx{ 0, 0 };
		NiAVObject_UpdateObjectUpwards(handNode, &ctx);
	}
}


void Grabber::EndPull()
{
	NiPointer<TESObjectREFR> pulledObj;
	if (LookupREFRByHandle(pulledObject.handle, pulledObj)) {
		NiPointer<NiNode> objRoot = pulledObj->GetNiNode();
		if (objRoot) {
			CollisionInfo::ResetCollisionInfoDownstream(objRoot, CollisionInfo::State::Unheld);
			pulledObject.rigidBody->hkBody->m_motion.m_motionState.m_angularDamping = pulledObject.savedAngularDamping;
		}
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

bool Grabber::CanGrabObject() const
{
	return state == State::Idle || state == State::SelectedClose || state == State::SelectedFar || state == State::SelectionLocked;
}

bool Grabber::HasHeldObject() const
{
	return state == State::Held || state == State::HeldInit || state == State::HeldBody;
}

bool Grabber::CanOtherGrab() const
{
	return HasHeldObject();
}

bool Grabber::HasHeldKeyframed() const
{
	return state == State::Held || state == State::HeldInit;
}


bool Grabber::ShouldDisplayRollover()
{
	if (state != State::SelectedClose && state != State::SelectionLocked && state != State::Held && state != State::HeldInit && state != State::HeldBody && state != State::LootOtherHand)
		return false;

	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
		return true;
	}
	return false;
}


bool Grabber::IsSafeToClearSavedCollision() const
{
	return (
		(state == State::Idle || state == State::SelectedFar || state == State::SelectedClose)
		&& pulledObject.handle == *g_invalidRefHandle
		);
}

bool Grabber::GetActivateText(std::string &strOut)
{
	if (state != State::SelectedClose && state != State::SelectionLocked && state != State::LootOtherHand) {
		return false;
	}

	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
		TESForm *baseForm = selectedObj->baseForm;
		if (baseForm) {
			TESBoundObject *boundObject = DYNAMIC_CAST(baseForm, TESForm, TESBoundObject);
			if (boundObject) {
				PlayerCharacter *player = *g_thePlayer;

				UInt64 *vtbl = *((UInt64 **)boundObject);
				BSString textStr;
				bool showActivate = ((TESBoundObject_GetActivateText)(vtbl[0x4C]))(boundObject, selectedObj, textStr);
				char *text = textStr.m_data;
				if (showActivate && text) {
					char *color;
					if (CALL_MEMBER_FN(selectedObj, IsOffLimits)()) {
						color = "#ff0000";
					}
					else {
						color = "#ffffff";
					}

					const char * itemNameReplaced = nullptr;

					std::string verb;
					if (state == State::SelectedClose) {
						verb = Config::options.grabString;

						Grabber *other = isLeft ? g_rightGrabber : g_leftGrabber;

						if (selectedObject.isActor && (selectedObject.isDisconnected || (selectedObject.hitForm && selectedObject.rigidBody == other->selectedObject.rigidBody && other->CanOtherGrab()))) {
							TESForm *wornForm = selectedObject.hitForm;
							if (wornForm) {
								BaseExtraList *wornExtraData = selectedObject.hitExtraList;
								itemNameReplaced = GetItemName(wornForm, wornExtraData);
							}
						}
					}
					else if (state == State::SelectionLocked) {
						verb = Config::options.pullString;

						if (selectedObject.isActor) {
							verb = Config::options.lootString;
							TESForm *wornForm = selectedObject.hitForm;
							if (wornForm) {
								BaseExtraList *wornExtraData = selectedObject.hitExtraList;
								itemNameReplaced = GetItemName(wornForm, wornExtraData);
							}
						}
					}
					else if (state == State::LootOtherHand) {
						verb = Config::options.lootString;
						TESForm *wornForm = selectedObject.hitForm;
						if (wornForm) {
							BaseExtraList *wornExtraData = selectedObject.hitExtraList;
							itemNameReplaced = GetItemName(wornForm, wornExtraData);
						}
					}

					std::stringstream ss;
					ss << "<font color='" << color << "'>" << verb << "</font>\n";

					if (itemNameReplaced) {
						ss << itemNameReplaced;
						strOut = ss.str();
						return true;
					}

					if (*text) {
						std::string currentStr(text);
						std::regex e("^.*\\n");
						strOut = std::regex_replace(currentStr, e, ss.str(), std::regex_constants::format_first_only);
					}
					else {
						// Item activate text is empty... just show the "grab", "pull", whatnot
						strOut = ss.str();
					}

					return true;
				}
			}
		}
	}

	strOut = "";
	return true;
}

void Grabber::SetupRollover()
{
	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
		// Set rotation/position/scale of the hud prompt

		PlayerCharacter *player = *g_thePlayer;
		if (player) {
			NiPointer<NiAVObject> wandNode = isLeft ? player->unk3F0[PlayerCharacter::Node::kNode_LeftWandNode] : player->unk3F0[PlayerCharacter::Node::kNode_RightWandNode];
			NiPointer<NiAVObject> roomObj = player->unk3F0[PlayerCharacter::Node::kNode_RoomNode];
			NiNode *roomNode = roomObj ? roomObj->GetAsNiNode() : nullptr;
			if (roomNode && wandNode) {
				static BSFixedString rolloverNodeStr("WSActivateRollover");
				NiPointer<NiAVObject> rolloverNode = roomNode->GetObjectByName(&rolloverNodeStr.data);
				if (rolloverNode) {
					NiTransform desiredLocal;
					desiredLocal.pos = rolloverOffset;
					desiredLocal.rot = rolloverRotation;
					desiredLocal.scale = rolloverScale;

					// World transform where we would like the rollover node to be
					NiTransform desiredWorld;

					if (state == State::HeldBody) {
						NiTransform inverseHand;
						handTransform.Invert(inverseHand);
						NiTransform handToWand = inverseHand * wandNode->m_worldTransform;

						desiredWorld = adjustedHandTransform * handToWand * desiredLocal;
					}
					else {
						desiredWorld = wandNode->m_worldTransform * desiredLocal;
					}

					UpdateNodeTransformLocal(rolloverNode, desiredWorld);

					NiAVObject::ControllerUpdateContext ctx{ 0, 0 };
					NiAVObject_UpdateObjectUpwards(rolloverNode, &ctx);
				}
			}
		}
	}
}

void Grabber::SetupSelectionBeam(NiNode *spellOrigin)
{
	NiPointer<NiAVObject> handNode = GetHandNode();
	if (!handNode) return;

	NiPointer<TESObjectREFR> obj;
	if (LookupREFRByHandle(selectedObject.handle, obj)) {
		bool isOffLimits = CALL_MEMBER_FN(obj, IsOffLimits)();
		NiAVObject *arcNodeShow = isOffLimits ? spellOrigin->m_children.m_data[1] : spellOrigin->m_children.m_data[0];
		NiAVObject *arcNodeHide = isOffLimits ? spellOrigin->m_children.m_data[0] : spellOrigin->m_children.m_data[1];

		NiPoint3 palmPos = handNode->m_worldTransform * palmPosHandspace;

		float havokWorldScale = *g_havokWorldScale;

		hkVector4 translation = selectedObject.rigidBody->hkBody->m_motion.m_motionState.m_transform.m_translation;
		NiPoint3 hkObjPos = HkVectorToNiPoint(translation);

		NiPoint3 origin = palmPos;
		NiPoint3 dest = (hkObjPos + pulledPointOffset) / havokWorldScale;
		NiPoint3 destToOrigin = origin - dest;
		float distance = VectorLength(destToOrigin);
		if (distance == 0.0f) return;

		NiPoint3 destToOriginNormalized = destToOrigin / distance;

		NiMatrix33 stretcher;
		stretcher.Identity();
		stretcher.data[0][0] = Config::options.selectionBeamStretch.x;
		stretcher.data[1][1] = distance * Config::options.selectionBeamStretch.y; // stretches y (forward) vector
		stretcher.data[2][2] = Config::options.selectionBeamStretch.z;

		NiMatrix33 rot;
		NiPoint3 worldUp = { 0, 0, 1 };
		MatrixFromForwardVector(&rot, &destToOriginNormalized, &worldUp);

		rot = rot * stretcher;

		spellOrigin->m_localTransform.pos = dest;
		spellOrigin->m_localTransform.rot = rot;

		spellOrigin->m_flags &= 0xFFFFFFFE; // unhide spell origin

		arcNodeShow->m_flags &= 0xFFFFFFFE; // unhide tempArc
		arcNodeHide->m_flags |= 1; // hide noGoArc

		NiAVObject::ControllerUpdateContext ctx{ 0.0f, 0 };
		NiAVObject_UpdateObjectUpwards(spellOrigin, &ctx);
	}
}
