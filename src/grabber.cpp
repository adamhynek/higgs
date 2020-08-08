#include <numeric>

#include "skse64/GameRTTI.h"
#include "skse64/PapyrusActor.h"
#include "skse64/NiGeometry.h"
#include "skse64/GameExtraData.h"

#include "grabber.h"
#include "offsets.h"
#include "utils.h"
#include "config.h"
#include "menu_checker.h"
#include "shaders.h"
#include "math_utils.h"
#include "vrikinterface001.h"

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


// Copied from PapyrusActor.cpp
class MatchByForm : public FormMatcher
{
	TESForm * m_form;
public:
	MatchByForm(TESForm * form) : m_form(form) {}

	bool Matches(TESForm* pForm) const { return m_form == pForm; }
};


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
	std::lock_guard<std::mutex> lock(deselectLock);

	selectedObject.handle = *g_invalidRefHandle;
	selectedObject.collidable = nullptr;
	selectedObject.rigidBody = nullptr;
	selectedObject.shaderNode = nullptr;
	selectedObject.hitNode = nullptr;
	selectedObject.hitForm = nullptr;

	state = State::Idle;
}


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
	sphereShape->m_radius = 0.05f;
	sphere->phantom->m_motionState.m_transform.m_translation = NiPointToHkVector(hkPalmNodePos);

	NiPoint3 targetPos = hkPalmNodePos + castDirection * 0.1f;
	linearCastInput.m_to = NiPointToHkVector(targetPos);
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


void Grabber::TransitionHeld(bhkWorld *world, NiPoint3 &hkPalmNodePos, NiPoint3 &castDirection, hkContactPoint &closestPoint, float havokWorldScale, NiAVObject *handNode, TESObjectREFR *selectedObj)
{
	NiAVObject *n = FindCollidableNode(selectedObject.collidable);
	if (n) {
		StopShader(selectedObject.handle, selectedObject.shaderNode);

		grabbedTime = g_currentFrameTime;

		NiPoint3 ptPos = HkVectorToNiPoint(closestPoint.getPosition());
		//NiPoint3 normal = HkVectorToNiPoint(closestPoint.m_separatingNormal); // vec from sphere center to point

		// Cancel a collision reset from pulling if we're grabbing the object
		if (pulledObject.handle == selectedObject.handle) {
			EndPull();
		}

		if (selectedObject.isActor) {
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
			initialObjTransformHandSpace = inverseHand * desiredTransform;

			state = State::HeldBody;
		}
		else {
			// TODO: For e.g. books, I don't think we want the other parts of the book to collide with the hand that's holding it (with the other hand is fine). It can cause physics freakouts.
			SetCollisionInfoForAllCollisionInRefr(selectedObj, playerCollisionGroup);

			selectedObject.savedMotionType = selectedObject.rigidBody->hkBody->m_motion.m_type;
			selectedObject.savedQuality = selectedObject.collidable->m_broadPhaseHandle.m_objectQualityType;

			// Set motion type before trying to set obj transform
			bhkRigidBody_setMotionType(selectedObject.rigidBody, hkpMotion::MotionType::MOTION_KEYFRAMED, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY);


			initialGrabbedObjWorldPosition = n->m_worldTransform.pos;

			NiPoint3 triPos, triNormal;
			float closestDist = (std::numeric_limits<float>::max)();
			double t = GetTime();
			bool success = GetClosestPointOnGraphicsGeometryToLine(selectedObj->loadedState->node, hkPalmNodePos / havokWorldScale, castDirection, &triPos, &triNormal, &closestDist);
			//bool success = GetClosestPointOnGraphicsGeometry(selectedObj->loadedState->node, hkPalmNodePos / havokWorldScale, &triPos, &triNormal, &closestDist);

			NiTransform desiredTransform = n->m_worldTransform;

			if (success) {
				// We've got a point on the graphics geometry
				ptPos = triPos * havokWorldScale;

				PlayerCharacter *player = *g_thePlayer;

				//static const NiPoint3 nonThumbNormalHandspace = { 1, 0, 0 }; // points out the thumb like in a thumbs-up for the right hand
				static const NiPoint3 nonThumbNormalHandspace = VectorNormalized({ 1, 0.3, 0.2 }); // points out the thumb like in a thumbs-up for the right hand
				static const NiPoint3 nonThumbZeroAngleVectorHandspace = VectorNormalized({ 0, -0.5, 1 });

				static const NiPoint3 thumbZeroAngleVectorHandspace = VectorNormalized({ 2.2, -1.1, 1 });
				static const NiPoint3 thumbNormalHandspace = VectorNormalized({ -0.285, -0.85, -0.443 }); // points out the palm for the right hand

				NiPoint3 nonThumbNormalWorldspace = VectorNormalized(handNode->m_worldTransform.rot * nonThumbNormalHandspace);
				NiPoint3 nonThumbZeroAngleVectorWorldspace = VectorNormalized(handNode->m_worldTransform.rot * nonThumbZeroAngleVectorHandspace);

				auto FingerCheck = [this, player, handNode](TESObjectREFR *refr, int fingerIndex, NiPoint3 zeroAngleVectorHandspace, NiPoint3 normalHandspace) -> float
				{
					NiAVObject *startFinger = player->GetNiRootNode(1)->GetObjectByName(&fingerNodeNames[fingerIndex][0].data);
					NiAVObject *midFinger = player->GetNiRootNode(1)->GetObjectByName(&fingerNodeNames[fingerIndex][1].data);
					NiAVObject *endFinger = player->GetNiRootNode(1)->GetObjectByName(&fingerNodeNames[fingerIndex][2].data);

					if (startFinger && midFinger && endFinger) {
						NiPoint3 startFingerPos = startFinger->m_worldTransform.pos;
						NiPoint3 midFingerPos = midFinger->m_worldTransform.pos;
						NiPoint3 endFingerPos = endFinger->m_worldTransform.pos;

						if (isLeft) {
							zeroAngleVectorHandspace.x *= -1; // x axis is flipped for left hand
						}
						NiPoint3 zeroAngleVectorWorldspace = VectorNormalized(handNode->m_worldTransform.rot * zeroAngleVectorHandspace);

						if (isLeft) {
							// Flip the entire vector, then flip the x-axis. Equivalent: flip y/z
							normalHandspace.y *= -1;
							normalHandspace.z *= -1;
						}
						NiPoint3 normalWorldspace = VectorNormalized(handNode->m_worldTransform.rot * normalHandspace);

						float closestAngle = (std::numeric_limits<float>::max)();
						NiPoint3 intersection, normal;
						bool intersects = GetCircleIntersectionOnGraphicsGeometry(refr->loadedState->node, startFingerPos, midFingerPos, endFingerPos, normalWorldspace, zeroAngleVectorWorldspace, &intersection, &normal, &closestAngle);
						if (intersects) {
							NiPoint3 centerToIntersect = intersection - startFingerPos;
							float angle = acosf(DotProduct(VectorNormalized(centerToIntersect), zeroAngleVectorWorldspace));
							if (DotProduct(normalWorldspace, CrossProduct(zeroAngleVectorWorldspace, centerToIntersect)) < 0) {
								// Positive angles are those which curl the finger
								angle *= -1;
							}
							return angle;
						}
						else {
							// No finger intersection, so just close it completely
							return 99999999; // some large angle
						}
					}
					// Couldn't get the fingers... that's a problem
					_ERROR("Could not get finger %d", fingerIndex);
					return 0;
				};

				// Update transform to snap to the hand
				desiredTransform = n->m_worldTransform;
				desiredTransform.pos += hkPalmNodePos / havokWorldScale - triPos;
				UpdateKeyframedNodeTransform(n, desiredTransform);

				std::array<float, 5> fingerAngles;
				for (int i = 1; i < fingerAngles.size(); i++) { // No thumb
					fingerAngles[i] = FingerCheck(selectedObj, i, nonThumbZeroAngleVectorHandspace, nonThumbNormalHandspace);
				}

				int fingerWithSmallestAngle = -1;
				float smallestAngle = (std::numeric_limits<float>::max)();
				for (int i = 1; i < fingerAngles.size(); i++) {
					float angle = fingerAngles[i];
					if (angle < smallestAngle) {
						smallestAngle = angle;
						fingerWithSmallestAngle = i;
					}
				}

				// TODO: BETTER IDEA
				//
				// Derotate first, then move the hand backwards in the palm direction. Then compute palm attach pos and all fingers again...
				//
				// Do derotation for the thumb only. Then, for the rest of the fingers, instead of derotating, move the hand away from the object (or vice versa - moving the object might be easier?)
				// in the negative palm direction some amount, at which point hopefully the fingers are no longer intersecting and we can do the whole thing from scratch

				const float minAllowedAngle = 10 * 0.0174533; // 10 degrees
				if (smallestAngle < minAllowedAngle) {
					// Derotate the object to not have fingers clip through it
					// TODO: Rotate about closest pt on line that goes through the knuckles, instead of the tripos
					NiPoint3 axis = nonThumbNormalWorldspace;
					float angle = minAllowedAngle - smallestAngle;

					// First, rotate the center of the object relative to the closest point, then rotate the object itself by the angle
					NiAVObject *startFinger = player->GetNiRootNode(1)->GetObjectByName(&fingerNodeNames[fingerWithSmallestAngle][0].data);
					NiAVObject *midFinger = player->GetNiRootNode(1)->GetObjectByName(&fingerNodeNames[fingerWithSmallestAngle][1].data);
					NiAVObject *endFinger = player->GetNiRootNode(1)->GetObjectByName(&fingerNodeNames[fingerWithSmallestAngle][2].data);
					if (startFinger && midFinger && endFinger) {
						NiPoint3 startFingerPos = startFinger->m_worldTransform.pos;
						NiPoint3 midFingerPos = midFinger->m_worldTransform.pos;
						NiPoint3 endFingerPos = endFinger->m_worldTransform.pos;

						float radius = VectorLength(endFingerPos - midFingerPos) + VectorLength(midFingerPos - startFingerPos);
						NiPoint3 desiredPtPos = startFingerPos + RotateVectorByAxisAngle(nonThumbZeroAngleVectorWorldspace * radius, axis, minAllowedAngle);
						NiPoint3 actualPtPos = startFingerPos + RotateVectorByAxisAngle(nonThumbZeroAngleVectorWorldspace * radius, axis, smallestAngle);

						NiPoint3 ptToCenter = n->m_worldTransform.pos - actualPtPos;

						NiPoint3 rotatedPtToCenter = RotateVectorByAxisAngle(ptToCenter, axis, angle);
						NiPoint3 desiredPos = desiredPtPos + rotatedPtToCenter;
						//NiPoint3 ptToCenter = rotatedPtToCenter * havokWorldScale;

						//NiPoint3 desiredPos = (hkPalmNodePos + ptToCenter) / havokWorldScale; // in skyrim coords
						NiTransform desiredTransform = n->m_worldTransform;
						desiredTransform.pos = desiredPos;

						NiMatrix33 rotMatrix = MatrixFromAxisAngle(axis, angle);
						desiredTransform.rot = rotMatrix * desiredTransform.rot;

						UpdateKeyframedNodeTransform(n, desiredTransform);

						for (int i = 1; i < fingerAngles.size(); i++) {
							fingerAngles[i] += angle;
						}
					}
				}

				// Move the object away from the hand a bit
				desiredTransform = n->m_worldTransform;
				desiredTransform.pos += castDirection * 5; // ~7cm
				UpdateKeyframedNodeTransform(n, desiredTransform);

				// Compute the palm attach pos again
				NiPoint3 triPos, triNormal;
				float closestDist = (std::numeric_limits<float>::max)();
				bool success = GetClosestPointOnGraphicsGeometryToLine(selectedObj->loadedState->node, hkPalmNodePos / havokWorldScale, castDirection, &triPos, &triNormal, &closestDist);

				// Update transform to snap to the hand
				desiredTransform = n->m_worldTransform;
				desiredTransform.pos += hkPalmNodePos / havokWorldScale - triPos;
				UpdateKeyframedNodeTransform(n, desiredTransform);

				// Recompute all fingers
				for (int i = 1; i < fingerAngles.size(); i++) { // No thumb
					fingerAngles[i] = FingerCheck(selectedObj, i, nonThumbZeroAngleVectorHandspace, nonThumbNormalHandspace);
				}

				// Now that we've determined (potentially) the object's rotation, figure out the thumb
				fingerAngles[0] = FingerCheck(selectedObj, 0, thumbZeroAngleVectorHandspace, thumbNormalHandspace);

				if (g_vrikInterface) {
					std::array<float, 5> fingerRanges;
					for (int i = 0; i < fingerRanges.size(); i++) {
						float degrees = fingerAngles[i] * 57.2958;
						_MESSAGE("%d angle: %.2f", i, degrees);
						if (i == 0) {
							// Thumb's got it a bit different
							fingerRanges[i] = 1.0f - max(0, min(1, (degrees / 180.0f) / 0.75f));
						}
						else {
							fingerRanges[i] = 1.0f - max(0, min(1, degrees / 180.0f));
						}
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
			initialObjTransformHandSpace = inverseHand * desiredTransform;

			state = State::HeldInit;
		}
	}
}


void Grabber::PoseUpdate(const Grabber &other, bool allowGrab, NiNode *playerWorldNode)
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

	NiPoint3 castDirection = VectorNormalized(handNode->m_worldTransform.rot * Config::options.handAdjust);

	NiAVObject *hmdNode = playerWorldNode->GetObjectByName(&hmdNodeStr.data);
	if (!hmdNode)
		return;

	NiPoint3 hmdPos = hmdNode->m_worldTransform.pos;

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

	playerCollisionGroup = ((bhkCollisionObject *)comNode->unk040)->body->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo >> 16;

	bhkWorld *world = GetWorld(cell);
	if (!world) {
		_MESSAGE("Could not get havok world from player cell");
		return;
	}

	float havokWorldScale = *g_havokWorldScale;

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

		// Add collision object for the hand
		hkpBoxShape_ctor(handCollShape, { 0.05, 0.015, 0.075, 0 }, 0);
		hkpRigidBodyCinfo_ctor(handCollCInfo); // initialize with defaults
		handCollCInfo->m_shape = handCollShape;
		handCollCInfo->m_collisionFilterInfo = ((UInt32)playerCollisionGroup << 16) | 56; // player group, our custom layer
		handCollCInfo->m_collisionFilterInfo |= (1 << 15); // set bit 15 to collide with same group that also has bit 15
		handCollCInfo->m_motionType = hkpMotion::MotionType::MOTION_KEYFRAMED;
		handCollCInfo->m_enableDeactivation = false;
		handCollCInfo->m_solverDeactivation = hkpRigidBodyCinfo::SolverDeactivation::SOLVER_DEACTIVATION_OFF;

		hkpRigidBody_ctor(handCollBody, handCollCInfo);

		hkpWorld_AddEntity(world->world, handCollBody, HK_ENTITY_ACTIVATION_DO_ACTIVATE);

		world->worldLock.UnlockWrite();
	}

	// Put our hand collision where we want it
	NiPoint3 desiredPos = (handNode->m_worldTransform * (NiPoint3(0, -0.005, 0.08) / havokWorldScale)) * havokWorldScale;
	hkRotation desiredRot;
	NiMatrixToHkMatrix(handNode->m_worldTransform.rot, desiredRot);
	hkQuaternion desiredQuat;
	desiredQuat.setFromRotationSimd(desiredRot);
	hkpKeyFrameUtility_applyHardKeyFrame(NiPointToHkVector(desiredPos), desiredQuat, 1.0f / *g_deltaTime, handCollBody);

	NiPoint3 handVelocityWorldspace = (handPos - prevHandPosWorldspace) / *g_deltaTime;

	// Update velocities array to this frame
	for (int i = handVelocitiesWorldspace.size() - 1; i >= 1; i--) {
		handVelocitiesWorldspace[i] = handVelocitiesWorldspace[i - 1];
	}
	handVelocitiesWorldspace[0] = handVelocityWorldspace;

	NiPoint3 avgVelocityWorldspace = std::accumulate(handVelocitiesWorldspace.begin(), handVelocitiesWorldspace.end(), NiPoint3()) / handVelocitiesWorldspace.size();

	NiPoint3 handPosRoomspace = wandNode->m_localTransform.pos;
	NiPoint3 handVelocityRoomspace = (handPosRoomspace - prevHandPosRoomspace) / *g_deltaTime;

	// Update velocities array to this frame
	for (int i = handVelocitiesRoomspace.size() - 1; i >= 1; i--) {
		handVelocitiesRoomspace[i] = handVelocitiesRoomspace[i - 1];
	}
	handVelocitiesRoomspace[0] = handVelocityRoomspace;

	NiPoint3 avgVelocityRoomspace = std::accumulate(handVelocitiesRoomspace.begin(), handVelocitiesRoomspace.end(), NiPoint3()) / handVelocitiesRoomspace.size();

	// Get whatever transform takes the wand position from room space to skyrim worldspace
	NiMatrix33 localToWorldTransform = wandNode->m_worldTransform.rot * wandNode->m_localTransform.rot.Transpose();
	NiMatrix33 worldToLocalTransform = localToWorldTransform.Transpose();

	NiPoint3 localVelocityWorldspace = localToWorldTransform * avgVelocityRoomspace;

	NiPoint3 handDirectionRoomspace = worldToLocalTransform * castDirection;
	NiPoint3 deltaHandDirectionRoomspace = handDirectionRoomspace - prevHandDirectionRoomspace;
	NiPoint3 handDirectionVelocityRoomspace = deltaHandDirectionRoomspace / *g_deltaTime;

	// Update velocities array to this frame
	for (int i = handDirectionVelocities.size() - 1; i >= 1; i--) {
		handDirectionVelocities[i] = handDirectionVelocities[i - 1];
	}
	handDirectionVelocities[0] = handDirectionVelocityRoomspace;

	NiPoint3 avgDirectionVelocityRoomspace = std::accumulate(handDirectionVelocities.begin(), handDirectionVelocities.end(), NiPoint3()) / handDirectionVelocities.size();

	//if (std::string(handNodeName.data) == "NPC R Hand [RHnd]") {
	//	PrintToFile(std::to_string(VectorLength(avgDirectionVelocityRoomspace)), "hand_dir_speed_r.txt");
	//}

	NiAVObject *upperArmNode = player->GetNiRootNode(0)->GetObjectByName(&upperArmNodeName.data);
	if (!upperArmNode)
		return;

	NiPoint3 upperArmPos = upperArmNode->m_worldTransform.pos;

	if (g_currentFrameTime - pulledTime > pulledExpireTime) {
		EndPull();
	}

	bhkSimpleShapePhantom *sphere = *g_pickSphere;
	if (!sphere)
		return;

	NiAVObject *palmNode = player->GetNiRootNode(1)->GetObjectByName(&palmNodeName.data);
	if (!palmNode)
		return;
	NiPoint3 hkPalmNodePos = palmNode->m_worldTransform.pos * havokWorldScale;

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

		bool isSelectedNear = FindCloseObject(world, allowGrab, other, hkPalmNodePos, castDirection, sphere,
			&closestObj, &closestRigidBody, &closestPoint);

		if (!isSelectedNear) {
			// Nothing close by the hand. Check for stuff pointing from from the palm

			// Convert hand position from skyrim coords to havok coords
			NiPoint3 hkHmdPos = hmdPos * havokWorldScale;

			NiPoint3 hkTargetPos = hkPalmNodePos + castDirection * Config::options.castDistance;

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
			sphereShape->m_radius = Config::options.castRadius;
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
						NiPoint3 hit = HkVectorToNiPoint(pair.second.getPosition());
						NiPoint3 handToHit = hit - hkPalmNodePos;
						NiPoint3 handToHitAlongRay = castDirection * DotProduct(handToHit, castDirection); // project above vector onto ray
						float dist = VectorLength(handToHit - handToHitAlongRay); // distance from hit location to closest point on the ray
						if (dist < closestDistance && DotProduct(VectorNormalized(hit - hkHmdPos), hmdForward) >= Config::options.requiredCastDotProduct) {
							closestObj = ref;
							closestRigidBody = bRigidBody;
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
			NiPointer<TESObjectREFR> selectedObj;
			if (!LookupREFRByHandle(selectedObject.handle, selectedObj) || closestObj != selectedObj) {
				if (selectedObj) {
					// Deselect the old thing if something else was selected
					StopShader(selectedObject.handle, selectedObject.shaderNode);
					Deselect();
				}
				// select the new refr
				Select(closestObj);
				state = isSelectedNear ? State::SelectedClose : State::SelectedFar;
			}
			else if ((state == State::SelectedFar && isSelectedNear) || (state == State::SelectedClose && !isSelectedNear)) {
				// Same object is selected, but it has become near/far enough to switch states
				state = isSelectedNear ? State::SelectedClose : State::SelectedFar;
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
										bool isArmorHit = IsNodeWithinArmor(geomNode, hitNode);
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
										auto hitBipedData = &bipedData->unk10[hitIndex];
										nodeOnWhichToPlayShader = hitBipedData->object;
										selectedObject.hitForm = hitForm;
										selectedObject.hitNode = hitNode;
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
						TESEffectShader *shader;
						if (CALL_MEMBER_FN(selectedObj, IsOffLimits)()) {
							shader = itemSelectedShaderOffLimits;
						}
						else {
							shader = itemSelectedShader;
						}

						if (selectedObject.handle == prevSelectedHandle) {
							// Stop the shader on the current reference, we've changed nodes
							StopShader(selectedObject.handle, selectedObject.shaderNode);
						}
						else {
							// Stop the shader on the previous reference
							NiPointer<TESObjectREFR> prevSelectedObj;
							if (LookupREFRByHandle(prevSelectedHandle, prevSelectedObj)) {
								StopShader(prevSelectedHandle, selectedObject.shaderNode);
							}
						}
						PlayShader(selectedObject.handle, nodeOnWhichToPlayShader, shader);
						selectedObject.shaderNode = nodeOnWhichToPlayShader;
					}
				}
			}
			else if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				// No node selected but refr is still selected

				if (selectedObject.handle != prevSelectedHandle) {
					// New refr is selected. Stopping the shader for the previous ref is done earlier.
					if (!selectedObject.isActor) {
						TESEffectShader *shader;
						if (CALL_MEMBER_FN(selectedObj, IsOffLimits)()) {
							shader = itemSelectedShaderOffLimits;
						}
						else {
							shader = itemSelectedShader;
						}

						PlayShader(selectedObject.handle, nullptr, shader);
					}

					selectedObject.shaderNode = nullptr;
					selectedObject.hitNode = nullptr;
					selectedObject.hitForm = nullptr;
				}
				else if (selectedObject.shaderNode) {
					// Node was selected before (selectedObject.shaderNode), but not now (nodeOnWhichToPlayShader). Stop the shader on that node.
					if (breakStickiness) {
						StopShader(selectedObject.handle, selectedObject.shaderNode);

						TESEffectShader *shader;
						if (CALL_MEMBER_FN(selectedObj, IsOffLimits)()) {
							shader = itemSelectedShaderOffLimits;
						}
						else {
							shader = itemSelectedShader;
						}

						//PlayShader(selectedObject.handle, nullptr, shader);

						selectedObject.shaderNode = nullptr;
						selectedObject.hitNode = nullptr;
						selectedObject.hitForm = nullptr;
					}
				}
			}

			// Set selected collidable no matter what, as we can have objects with more than one collidable
			selectedObject.rigidBody = closestRigidBody;
			selectedObject.collidable = &closestRigidBody->hkBody->m_collidable;

			isSelectedThisFrame = true;
			lastSelectedTime = g_currentFrameTime;
		}

		if (state == State::SelectedClose || state == State::SelectedFar) {
			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				if (LookupREFRByHandle(selectedObject.handle, selectedObj) && !isSelectedThisFrame && g_currentFrameTime - lastSelectedTime > Config::options.selectedLeewayTime) {
					// If time has run out and nothing is selected, deselect whatever is selected
					StopShader(selectedObject.handle, selectedObject.shaderNode);
					Deselect();
				}

				// Check if we should grab the object. If yes, grab and go to GRABBED
				if (grabRequested && g_currentFrameTime - grabRequestedTime <= Config::options.triggerPressedLeewayTime) {
					if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
						if (state == State::SelectedFar) {
							hkVector4 translation = selectedObject.rigidBody->hkBody->m_motion.m_motionState.m_transform.m_translation;
							NiPoint3 hkHandPos = handPos * havokWorldScale;
							NiPoint3 hkObjPos = HkVectorToNiPoint(translation);
							NiPoint3 relObjPos = hkObjPos - hkHandPos;

							NiPoint3 forward = castDirection;
							NiPoint3 worldUp = { 0, 0, 1 };
							NiPoint3 right = CrossProduct(forward, worldUp);
							NiPoint3 up = CrossProduct(right, forward);
							initialObjPosRaySpace = { DotProduct(relObjPos, forward), DotProduct(relObjPos, right) , DotProduct(relObjPos, up) };
							selectionLockedTime = g_currentFrameTime;
							initialGrabbedObjRelativePosition = relObjPos;
							initialGrabbedObjWorldPosition = hkObjPos;
							initialHandShoulderDistance = VectorLength(handPos - upperArmPos);
							state = State::SelectionLocked;
						}
						else if (state == State::SelectedClose) {
							TransitionHeld(world, hkPalmNodePos, castDirection, closestPoint, havokWorldScale, handNode, selectedObj);
						}
						// Set to false only here, so that you can hold the trigger until the cast hits something valid
						grabRequested = false;
						wasObjectGrabbed = true; // This variable is not set to false when we push/pull the object
					}
					else {
						// Selected object no longer exists
						state = State::Idle;
					}
				}
			}
			else {
				state = State::Idle;
			}
		}

		if (releaseRequested) {
			releaseRequested = false;
			wasObjectGrabbed = false;
		}
	}

	NiPointer<TESObjectREFR> selectedObj;
	if (state == State::SelectionLocked || state == State::Pulled || state == State::PrepullItem || state == State::HeldInit || state == State::Held || state == State::HeldBody) {
		// Check if we should drop the object. If yes, go to IDLE
		if (releaseRequested) {
			releaseRequested = false;
			wasObjectGrabbed = false;
			idleDesired = true;
		}
		if (idleDesired || !allowGrab) {
			idleDesired = false;

			NiPointer<TESObjectREFR> selectedObj;
			if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
				if (state == State::Held || state == State::HeldInit) {
					if (g_vrikInterface) {
						g_vrikInterface->restoreFingers(isLeft);
					}

					ResetCollisionInfoForAllCollisionInRefr(selectedObj, selectedObject.collidable); // skip the node we grabbed, we handle that below

					UInt8 savedCollisionRefCount = GetSavedCollisionRefCount(selectedObject.rigidBody->hkBody->m_uid);
					if (savedCollisionRefCount > 0) {
						if (savedCollisionRefCount == 1) {
							UInt32 savedCollision = GetSavedCollision(selectedObject.rigidBody->hkBody->m_uid);
							// Restore only the original layer first, so it collides with everything except the player
							selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7f;
							selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (savedCollision & 0x7f);
							selectedObject.collidable->m_broadPhaseHandle.m_objectQualityType = HK_COLLIDABLE_QUALITY_CRITICAL; // Will make object collide with other things as motion type is changed
							bhkRigidBody_setMotionType(selectedObject.rigidBody, selectedObject.savedMotionType, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);

							selectedObject.collidable->m_broadPhaseHandle.m_collisionFilterInfo = savedCollision;
							selectedObject.collidable->m_broadPhaseHandle.m_objectQualityType = selectedObject.savedQuality;
							bhkRigidBody_setMotionType(selectedObject.rigidBody, selectedObject.savedMotionType, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY);
						}
						else { // > 1
							// use current collision, other hand still has it
							selectedObject.collidable->m_broadPhaseHandle.m_objectQualityType = HK_COLLIDABLE_QUALITY_CRITICAL; // Will make object collide with other things as motion type is changed
							bhkRigidBody_setMotionType(selectedObject.rigidBody, selectedObject.savedMotionType, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);

							selectedObject.collidable->m_broadPhaseHandle.m_objectQualityType = selectedObject.savedQuality;
							bhkRigidBody_setMotionType(selectedObject.rigidBody, selectedObject.savedMotionType, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY);
						}
						RemoveSavedCollision(selectedObject.rigidBody->hkBody->m_uid);
					}
				}
				if (state == State::SelectionLocked) {
					StopShader(selectedObject.handle, selectedObject.shaderNode);
				}
				if (state == State::HeldInit || state == State::Held || state == State::HeldBody) {
					// TODO: Do a bit of lookback, we probably want the velocity from some time before we actually let go of the object
					bhkRigidBody_setActivated(selectedObject.rigidBody, true);
					selectedObject.rigidBody->hkBody->m_motion.m_linearVelocity = NiPointToHkVector(avgVelocityWorldspace * 1.1f * havokWorldScale); // Boost the velocity a bit
				}
			}

			Deselect();
		}

		NiPoint3 hkHandPos = handPos * havokWorldScale;

		if (state == State::SelectionLocked) {
			if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
				hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;

				auto TransitionPulled = [this, motion, selectedObj, handNode]()
				{
					StopShader(selectedObject.handle, selectedObject.shaderNode);

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
											state = State::PrepullItem;

											selectedObject.handle = droppedObjHandle;
											selectedObject.collidable = nullptr;
											selectedObject.rigidBody = nullptr;
										}
									}
								}
							}
						}
					}
					else {
						// Not trying to pull armor off a body
						state = State::Pulled;

						if (pulledObject.handle != selectedObject.handle) {
							EndPull(); // Cancel an existing pulled collision reset if we're pulling something new

							pulledObject.handle = selectedObject.handle;
							pulledObject.rigidBody = selectedObject.rigidBody;
							pulledObject.savedAngularDamping = motion->m_motionState.m_angularDamping;
							motion->m_motionState.m_angularDamping = hkHalf(3.0f);

							if (selectedObject.isImpactedProjectile) { // It's an embedded projectile, i.e. stuck in a wall etc.
								auto collObj = (bhkCollisionObject *)selectedObj->loadedState->node->unk040;
								if (collObj) {
									// Do not use grabbedObject.collidable here, as sometimes we end up grabbing the phantom shape of the projectile instead of the 3D one
									auto collidable = &collObj->body->hkBody->m_collidable;
									// Projectiles do not interact with collision usually. We need to change the filter to make them interact.
									collidable->m_broadPhaseHandle.m_collisionFilterInfo = (((UInt32)playerCollisionGroup) << 16) | 6; // player collision group, 'projectile' collision layer
									// Projectiles have 'Fixed' motion type by default, making them unmovable
									bhkRigidBody_setMotionType(collObj->body, hkpMotion::MotionType::MOTION_DYNAMIC, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);
								}
							}

							// TODO: Using the hand collision layer means the pulled object does not collide with other npcs. We probably do want it to though.
							SetCollisionInfoForAllCollisionInRefr(selectedObj, playerCollisionGroup);
						}
					}
				};

				bool isSelectedNear = false;

				if (g_currentFrameTime - grabRequestedTime <= Config::options.triggerPressedLeewayTime) {
					NiPointer<TESObjectREFR> closestObj;
					NiPointer<bhkRigidBody> closestRigidBody;
					hkContactPoint closestPoint;

					NiAVObject *palmNode = player->GetNiRootNode(1)->GetObjectByName(&palmNodeName.data);
					NiPoint3 hkPalmNodePos = palmNode->m_worldTransform.pos * havokWorldScale;

					isSelectedNear = FindCloseObject(world, allowGrab, other, hkPalmNodePos, castDirection, sphere,
						&closestObj, &closestRigidBody, &closestPoint);

					// Allow us to go to held if we had the thing selected from a distance and it came closer within the leeway time
					if (isSelectedNear && closestRigidBody == selectedObject.rigidBody) {
						TransitionHeld(world, hkPalmNodePos, castDirection, closestPoint, havokWorldScale, handNode, selectedObj);
					}
				}

				if (!isSelectedNear) {
					hkVector4 translation = motion->m_motionState.m_transform.m_translation;
					NiPoint3 hkObjPos = HkVectorToNiPoint(translation);
					NiPoint3 relObjPos = hkObjPos - hkHandPos;

					float handSpeedInObjDirection = DotProduct(localVelocityWorldspace, VectorNormalized(relObjPos));
					//if (std::string(handNodeName.data) == "NPC R Hand [RHnd]") {
					//	PrintToFile(std::to_string(VectorLength(handDirectionVelocityRoomspace)), "hand_delta_dir_r.txt");
					//	PrintToFile(std::to_string(handSpeedInObjDirection), "hand_speed_r.txt");
					//}

					bool pull = false;
					if ((VectorLength(avgDirectionVelocityRoomspace) > Config::options.pullAngularSpeedThreshold && handSpeedInObjDirection < 0) || handSpeedInObjDirection < -Config::options.pushPullSpeedThreshold) {
						if (IsObjectPullable()) {
							pull = true;
						}
					}

					if (pull) {
						TransitionPulled();
					}
					else {
						// Hold object selected while hand doesn't point too far away from original location. If far enough, deselect.
						NiPoint3 forward = castDirection;
						NiPoint3 worldUp = { 0, 0, 1 };
						NiPoint3 right = CrossProduct(forward, worldUp);
						NiPoint3 up = CrossProduct(right, forward);
						NiPoint3 initialObjPos = forward * initialObjPosRaySpace.x + right * initialObjPosRaySpace.y + up * initialObjPosRaySpace.z;

						// Determine the position where we want the object to be
						// Essentially it's a cylinder with a radius of the original distance when we grabbed it, and a height determined by some limit
						float maxHeight = 4.0f;
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

						if (DotProduct(VectorNormalized(initialObjPos), VectorNormalized(relObjPos)) < Config::options.grabbedDotProductThreshold || abs(DotProduct(deltaPos, castDirection)) > 1.5f) {
							//grab = true;
							idleDesired = true;
						}
					}
				}
			}
			else {
				// Grabbed object doesn't exist - go back to IDLE
				state = State::Idle;
			}
		}

		if (state == State::PrepullItem) {
			if (g_currentFrameTime - pulledTime > 0.5f) { // half a second for the item to spawn in
				state = State::Idle;
			}
			else {
				if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
					// Transition to pulled with the newly spawned item

					auto collObj = (bhkCollisionObject *)selectedObj->loadedState->node->unk040;
					if (collObj) {
						selectedObject.rigidBody = collObj->body;
						selectedObject.collidable = &selectedObject.rigidBody->hkBody->m_collidable;

						// Set owner to the player so it doesn't count as stealing
						TESObjectREFR_SetActorOwner(nullptr, nullptr, selectedObj, player->baseForm);

						state = State::Pulled;
						pulledTime = g_currentFrameTime;

						// Cancel an existing pulled collision reset
						EndPull();

						pulledObject.handle = selectedObject.handle;
						pulledObject.rigidBody = selectedObject.rigidBody;

						hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;
						pulledObject.savedAngularDamping = motion->m_motionState.m_angularDamping;
						motion->m_motionState.m_angularDamping = hkHalf(3.0f);
						SetCollisionInfoForAllCollisionInRefr(selectedObj, playerCollisionGroup);
					}
				}
			}
		}

		if (state == State::Pulled) {
			if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {

				hkpMotion *motion = &selectedObject.rigidBody->hkBody->m_motion;
				NiPoint3 hkObjPos = HkVectorToNiPoint(motion->m_motionState.m_transform.m_translation);

				float distanceFactor = VectorLength(hkPalmNodePos - hkObjPos) / 5.0f;
				float duration = 0.375f + 0.375f * distanceFactor;

				pulledExpireTime = duration + 1.0f;

				// Apply a predicted velocity to reach the destination, for a few frames after pulling starts.
				// Why for a few frames? Because then if it's next to something, it has a few frames to push it out of the way instead of just flopping right away

				if (g_currentFrameTime - pulledTime <= 0.06f) { // just over 5 frames at 90fps
					NiPoint3 horizontalDelta = hkPalmNodePos - hkObjPos;
					horizontalDelta.z = 0;
					NiPoint3 velocity = horizontalDelta / duration;
					float verticalDelta = hkPalmNodePos.z - hkObjPos.z;
					velocity.z = 0.5f * 9.81f * duration + verticalDelta / duration;

					SetVelocityForAllCollisionInRefr(selectedObj, NiPointToHkVector(velocity));
				}
				else {
					idleDesired = true;
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
					NiTransform newTransform = handNode->m_worldTransform * initialObjTransformHandSpace;

					if (state == State::HeldInit) {
						if (g_currentFrameTime - grabbedTime > 1.0f) {
							// It shouldn't take more than a second to move the object to the hand. If it does for some reason, just snap it there.
							state = State::Held;
						}
						else {
							// TODO: Interpolate not just pos, but rot too...?
							NiPoint3 delta = VectorNormalized(newTransform.pos - n->m_worldTransform.pos) * Config::options.grabStartSpeed * *g_deltaTime;
							if (VectorLengthSquared(delta) < VectorLengthSquared(newTransform.pos - n->m_worldTransform.pos)) {
								// If not close enough, move the object closer to the hand at some velocity
								newTransform.pos = n->m_worldTransform.pos + delta;
							}
							else {
								state = State::Held;
							}
						}
					}

					UpdateKeyframedNodeTransform(n, newTransform);
				}
			}
			else {
				if (g_vrikInterface) {
					g_vrikInterface->restoreFingers(isLeft);
				}
				state = State::Idle;
			}
		}

		if (state == State::HeldBody) {
			if (LookupREFRByHandle(selectedObject.handle, selectedObj) && selectedObj->loadedState && selectedObj->loadedState->node) {
				NiAVObject *n = FindCollidableNode(selectedObject.collidable);
				if (n) {
					bhkRigidBody_setActivated(selectedObject.rigidBody, true);
					NiTransform newTransform = handNode->m_worldTransform * initialObjTransformHandSpace;
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
	}

	//if (state != prevState) {
	//	_MESSAGE("%s: %d -> %d", name, prevState, state);
	//}

	prevState = state;
	prevHandPosWorldspace = handPos;
	prevHandPosRoomspace = handPosRoomspace;
	prevHandDirectionRoomspace = handDirectionRoomspace;
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
		ResetCollisionInfoForAllCollisionInRefr(pulledObj);
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
	if (state != State::SelectedClose && state != State::Pulled && state != State::SelectionLocked && state != State::Held && state != State::HeldInit && state != State::HeldBody)
		return false;

	if (state == State::SelectedClose) {
		// Even when no piece of armor selected, still show rollover when we'd be grabbing the body
		return true;
	}
	return IsObjectPullable();
}


bool Grabber::IsSafeToClearSavedCollision()
{
	return (state != State::Held && state != State::HeldInit && pulledObject.handle == *g_invalidRefHandle);
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


void Grabber::SetupRollover(NiAVObject *rolloverNode, bool isLeftHanded)
{
	NiPointer<TESObjectREFR> selectedObj;
	if (LookupREFRByHandle(selectedObject.handle, selectedObj)) {
		// Give the hud with info about the object you're floating

		// First, change rotation/position/scale of the hud prompt
		rolloverNode->m_localTransform.pos = rolloverOffset;
		rolloverNode->m_localTransform.rot = rolloverRotation;
		rolloverNode->m_localTransform.scale = rolloverScale;
	}
}
