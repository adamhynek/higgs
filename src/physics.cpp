#include <sstream>

#include "physics.h"
#include "offsets.h"

#include "skse64/NiNodes.h"


CdPointCollector::CdPointCollector()
{
	reset();
}

void CdPointCollector::reset()
{
	m_earlyOutDistance = 1.0f;
	m_hits.clear(); // TODO: Shrink to fit?
}

void CdPointCollector::addCdPoint(const hkpCdPoint& point)
{
	// Note: If this collector is being used for *linear casts* then for optimization 
	// purposes you should set the m_earlyOutDistance to:
	// - 0.0 if you want to get no more hits
	// - point.m_contact.getDistance() if you only want to get closer hits
	// - don't set, if you want to get all hits.

	hkpCdBody *cdBody = const_cast<hkpCdBody *>(&point.m_cdBodyB);
	while (cdBody->m_parent) {
		cdBody = const_cast<hkpCdBody *>(cdBody->m_parent);
	}
	//_MESSAGE("Hit: %x", cdBody->m_shape ? cdBody->m_shape->m_type : HK_SHAPE_INVALID);

	m_hits.push_back(std::make_pair(cdBody, point.getContact()));
	//m_earlyOutDistance = point.m_contact.getDistance(); // Only accept closer hits after this
}

CdBodyPairCollector::CdBodyPairCollector()
{
	reset();
}

void CdBodyPairCollector::reset()
{
	m_earlyOut = false;
	m_hits.clear(); // TODO: Shrink to fit?
}

void CdBodyPairCollector::addCdBodyPair(const hkpCdBody& bodyA, const hkpCdBody& bodyB)
{
	// Note: for optimization purposes this should set the m_earlyOut:
	// - true if you want to get no more hits
	// - false if you want to get more hits (which is the default)

	hkpCdBody *cdBody = const_cast<hkpCdBody *>(&bodyB);
	while (cdBody->m_parent) {
		cdBody = const_cast<hkpCdBody *>(cdBody->m_parent);
	}
	//_MESSAGE("Hit: %x", cdBody->m_shape ? cdBody->m_shape->m_type : HK_SHAPE_INVALID);

	m_hits.push_back(cdBody);
}

RayHitCollector::RayHitCollector()
{
	reset();
}

void RayHitCollector::reset()
{
	m_earlyOutHitFraction = 1.0f;
	m_doesHitExist = false;
}

void RayHitCollector::addRayHit(const hkpCdBody& cdBody, const hkpShapeRayCastCollectorOutput& hitInfo)
{
	// Note: for optimization purposes this should set the m_earlyOutHitFraction to:
	// - 0.0 if you want to get no more hits
	// - 1.0 if you want to get all hits (constructor initializes this value to 1.0 by default)
	// - output.m_hitFraction if you only want to get closer hits than one just found

	//while (cdBody->m_parent) {
	//	cdBody = cdBody->m_parent;
	//}
	//_MESSAGE("Raycast hit: %x", cdBody->m_shape ? cdBody->m_shape->m_type : HK_SHAPE_INVALID);

	//m_closestCollidable = cdBody;
	m_closestHitInfo = hitInfo;
	m_doesHitExist = true;
	m_earlyOutHitFraction = hitInfo.m_hitFraction; // Only accept closer hits after this
}

AllRayHitCollector::AllRayHitCollector()
{
	reset();
}

void AllRayHitCollector::reset()
{
	m_earlyOutHitFraction = 1.0f;
	m_hits.clear(); // TODO: Shrink to fit?
}

void AllRayHitCollector::addRayHit(const hkpCdBody& cdBody, const hkpShapeRayCastCollectorOutput& hitInfo)
{
	// Note: for optimization purposes this should set the m_earlyOutHitFraction to:
	// - 0.0 if you want to get no more hits
	// - 1.0 if you want to get all hits (constructor initializes this value to 1.0 by default)
	// - output.m_hitFraction if you only want to get closer hits than one just found

	//while (cdBody->m_parent) {
	//	cdBody = cdBody->m_parent;
	//}
	//_MESSAGE("Raycast hit: %x", cdBody->m_shape ? cdBody->m_shape->m_type : HK_SHAPE_INVALID);

	//m_closestCollidable = cdBody;

	hkpCdBody *body = const_cast<hkpCdBody *>(&cdBody);
	while (body->m_parent) {
		body = const_cast<hkpCdBody *>(body->m_parent);
	}

	m_hits.push_back(std::make_pair(body, hitInfo));
	//m_earlyOutHitFraction = hitInfo->m_hitFraction; // Only accept closer hits after this
}


namespace CollisionInfo
{
	// Map havok entity id -> (saved collisionfilterinfo, state of saved collision)
	std::unordered_map<UInt32, CollisionMapEntry> collisionInfoIdMap;

	void ClearCollisionMap()
	{
		// Should only be called when you're sure there should be nothing in the map
		if (collisionInfoIdMap.size() > 0) {
			collisionInfoIdMap.clear();
		}
	}

	void SetCollisionInfoDownstream(NiAVObject *obj, UInt32 collisionGroup, State reason)
	{
		if (obj->unk040) {
			auto collObj = (bhkCollisionObject *)obj->unk040;
			hkpRigidBody *entity = collObj->body->hkBody;
			if (entity->m_world) {
				hkpCollidable *collidable = &entity->m_collidable;

				// Save collisionfilterinfo by entity id
				UInt32 entityId = entity->m_uid;
				if (collisionInfoIdMap.count(entityId) != 0) {
					// Already in the map - state transition

					auto[savedInfo, savedState] = collisionInfoIdMap[entityId];

					if ((savedState == State::HeldLeft && reason == State::HeldRight) ||
						(savedState == State::HeldRight && reason == State::HeldLeft)) {
						// One hand holds the object -> 2 hands hold it

						collisionInfoIdMap[entityId] = { savedInfo, State::HeldBoth };

						bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
						world->worldLock.LockForWrite();

						UInt8 ragdollBits = (UInt8)RagdollLayer::SkipBoth;
						collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
						collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (ragdollBits << 8);

						hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

						world->worldLock.UnlockWrite();
					}
					else if (savedState == State::Unheld && (reason == State::HeldLeft || reason == State::HeldRight)) {
						// No hand holds it -> one hand holds it
						// I don't think this case is actually possible, since I try to reset a pull when grabbing, but it's here just in case.

						collisionInfoIdMap[entityId] = { savedInfo, reason };

						bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
						world->worldLock.LockForWrite();

						// We don't set the layer when pulling, so now that we're grabbing the object we need to set it.
						collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7F; // clear out layer
						collidable->m_broadPhaseHandle.m_collisionFilterInfo |= 56; // our custom layer

						UInt8 ragdollBits = (UInt8)(reason == State::HeldLeft ? RagdollLayer::SkipLeft : RagdollLayer::SkipRight);
						collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
						collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (ragdollBits << 8);

						hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

						world->worldLock.UnlockWrite();
					}
					else {
						//std::stringstream ss;
						//ss << "Invalid collision state transition: " << (UInt8)savedState << " -> " << (UInt8)reason;
						//ASSERT_STR(false, ss.str().c_str());
					}
				}
				else {
					// Not in the map yet - init to the necessary state

					collisionInfoIdMap[entityId] = { collidable->m_broadPhaseHandle.m_collisionFilterInfo, reason };

					bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
					world->worldLock.LockForWrite();

					collidable->m_broadPhaseHandle.m_collisionFilterInfo &= 0x0000FFFF;
					collidable->m_broadPhaseHandle.m_collisionFilterInfo |= collisionGroup << 16;

					// set bit 15. This way it won't collide with the player, but _will_ collide with other objects that also have bit 15 set (i.e. other things we pick up).
					collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (1 << 15); // Why bit 15? It's just the way the collision works.

					if (reason == State::HeldLeft || reason == State::HeldRight) {
						// Our collision layer negates collisions with other characters
						collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7F; // clear out layer
						collidable->m_broadPhaseHandle.m_collisionFilterInfo |= 56; // our custom layer

						UInt8 ragdollBits = (UInt8)(reason == State::HeldLeft ? RagdollLayer::SkipLeft : RagdollLayer::SkipRight);
						collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
						collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (ragdollBits << 8);
					}
					else if (reason == State::Unheld) {
						// When pulling, don't set the layer yet. This way it will still collide with other characters.

						UInt8 ragdollBits = (UInt8)RagdollLayer::SkipNone;
						collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
						collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (ragdollBits << 8);
					}
					else {
						std::stringstream ss;
						ss << "Invalid initial collision state: " << (UInt8)reason;
						ASSERT_STR(false, ss.str().c_str());
					}

					hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

					world->worldLock.UnlockWrite();
				}
			}
		}

		NiNode *node = obj->GetAsNiNode();
		if (node) {
			for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
				auto child = node->m_children.m_data[i];
				if (child) {
					SetCollisionInfoDownstream(child, collisionGroup, reason);
				}
			}
		}
	}

	void SetCollisionInfoForAllCollisionInRefr(TESObjectREFR *refr, UInt32 collisionGroup, State reason)
	{
		if (refr->loadedState && refr->loadedState->node) {
			SetCollisionInfoDownstream(refr->loadedState->node, collisionGroup, reason);
		}
	}

	void ResetCollisionInfoKeyframed(bhkRigidBody *entity, hkpMotion::MotionType motionType, hkInt8 quality, State reason)
	{
		UInt32 entityId = entity->hkBody->m_uid;
		if (collisionInfoIdMap.count(entityId) != 0) {
			auto[savedInfo, savedState] = collisionInfoIdMap[entityId];

			if (savedState == State::HeldBoth && (reason == State::HeldRight || reason == State::HeldLeft)) {
				// Two hands hold the object -> one hand lets go, so the other hand holds it

				State otherHand = reason == State::HeldRight ? State::HeldLeft : State::HeldRight;
				collisionInfoIdMap[entityId] = { savedInfo, otherHand };

				// TODO: Lock world for this line?
				UInt8 ragdollBits = (UInt8)(otherHand == State::HeldLeft ? RagdollLayer::SkipLeft : RagdollLayer::SkipRight);
				entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
				entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= (ragdollBits << 8);

				// use current collision, other hand still has it
				entity->hkBody->m_collidable.m_broadPhaseHandle.m_objectQualityType = HK_COLLIDABLE_QUALITY_CRITICAL; // Will make object collide with other things as motion type is changed
				bhkRigidBody_setMotionType(entity, motionType, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);

				entity->hkBody->m_collidable.m_broadPhaseHandle.m_objectQualityType = quality;
				bhkRigidBody_setMotionType(entity, motionType, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY);
			}
			else if ((savedState == State::HeldLeft && reason == State::HeldLeft) ||
				(savedState == State::HeldRight && reason == State::HeldRight) ||
				(savedState == State::Unheld && reason == State::Unheld)) {
				// One hand holds the object -> one hand lets go of the object, so none hold it
				// Or, no hand was holding, and none is holding now

				collisionInfoIdMap.erase(entityId);

				// Restore only the original layer and ragdoll bits first, so it collides with everything except the player (but still the hands)
				entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7f;
				entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= (savedInfo & 0x7f);
				entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
				entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt8)RagdollLayer::SkipNone << 8);
				entity->hkBody->m_collidable.m_broadPhaseHandle.m_objectQualityType = HK_COLLIDABLE_QUALITY_CRITICAL; // Will make object collide with other things as motion type is changed
				bhkRigidBody_setMotionType(entity, motionType, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);

				entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = savedInfo;
				entity->hkBody->m_collidable.m_broadPhaseHandle.m_objectQualityType = quality;
				bhkRigidBody_setMotionType(entity, motionType, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY);
			}
			else {
				//std::stringstream ss;
				//ss << "Invalid collision reset state transition: " << (int)savedState << " -> " << (int)reason;
				//ASSERT_STR(false, ss.str().c_str());
			}
		}
		else {
			ASSERT_STR(false, "Collision keyframed reset attempted on item that isn't in the collision map");
		}
	}

	void ResetCollisionInfoDownstream(NiAVObject *obj, State reason, hkpCollidable *skipNode)
	{
		if (obj->unk040) {
			auto collObj = (bhkCollisionObject *)obj->unk040;
			hkpRigidBody *entity = collObj->body->hkBody;
			if (entity->m_world) {
				hkpCollidable *collidable = &entity->m_collidable;
				if (collidable != skipNode) {
					UInt32 entityId = entity->m_uid;

					if (collisionInfoIdMap.count(entityId) != 0) {
						auto[savedInfo, savedState] = collisionInfoIdMap[entityId];

						if (savedState == State::HeldBoth && (reason == State::HeldRight || reason == State::HeldLeft)) {
							// Two hands hold the object -> one hand lets go, so the other hand holds it

							State otherHand = reason == State::HeldRight ? State::HeldLeft : State::HeldRight;
							collisionInfoIdMap[entityId] = { savedInfo, otherHand };

							bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
							world->worldLock.LockForWrite();

							UInt8 ragdollBits = (UInt8)(otherHand == State::HeldLeft ? RagdollLayer::SkipLeft : RagdollLayer::SkipRight);
							collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
							collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (ragdollBits << 8);

							hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

							world->worldLock.UnlockWrite();
						}
						else if ((savedState == State::HeldLeft && reason == State::HeldLeft) ||
							(savedState == State::HeldRight && reason == State::HeldRight) ||
							(savedState == State::Unheld && reason == State::Unheld)) {
							// One hand holds the object -> one hand lets go of the object, so none hold it
							// Or, no hand was holding, and none is holding now

							collisionInfoIdMap.erase(entityId);

							bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
							world->worldLock.LockForWrite();

							// Restore only the original layer and ragdoll bits first, so it collides with everything except the player (but still the hands)
							collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7f;
							collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (savedInfo & 0x7f);
							collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
							collidable->m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt8)RagdollLayer::SkipNone << 8);
							hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

							// Do not do a full check. What that means is it won't colide with the player until they stop colliding.
							collidable->m_broadPhaseHandle.m_collisionFilterInfo = savedInfo;
							hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

							world->worldLock.UnlockWrite();
						}
						else {
							//std::stringstream ss;
							//ss << "Invalid collision reset state transition: " << (int)savedState << " -> " << (int)reason;
							//ASSERT_STR(false, ss.str().c_str());
						}
					}
					else {
						ASSERT_STR(false, "Collision reset attempted on item that isn't in the collision map");
					}
				}
			}
		}

		NiNode *node = obj->GetAsNiNode();
		if (node) {
			for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
				auto child = node->m_children.m_data[i];
				if (child) {
					ResetCollisionInfoDownstream(child, reason, skipNode);
				}
			}
		}
	}

	void ResetCollisionInfoForAllCollisionInRefr(TESObjectREFR *refr, State reason, hkpCollidable *skipNode)
	{
		if (refr->loadedState && refr->loadedState->node) {
			ResetCollisionInfoDownstream(refr->loadedState->node, reason, skipNode);
		}
	}
}


void SetVelocityDownstream(NiAVObject *obj, hkVector4 velocity)
{
	if (obj->unk040) {
		auto collObj = (bhkCollisionObject *)obj->unk040;
		bhkRigidBody *bRigidBody = collObj->body;
		hkpRigidBody *rigidBody = bRigidBody->hkBody;
		if (rigidBody->m_world) {
			hkpMotion *motion = &rigidBody->m_motion;

			bhkRigidBody_setActivated(bRigidBody, true);
			motion->m_linearVelocity = velocity;
		}
	}

	NiNode *node = obj->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				SetVelocityDownstream(child, velocity);
			}
		}
	}
}

void SetVelocityForAllCollisionInRefr(TESObjectREFR *refr, hkVector4 velocity)
{
	if (refr->loadedState && refr->loadedState->node) {
		SetVelocityDownstream(refr->loadedState->node, velocity);
	}
}
