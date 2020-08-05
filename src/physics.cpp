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


// Map havok entity id -> (saved collisionfilterinfo, "refcount" of sorts)
std::unordered_map<UInt32, std::pair<UInt32, UInt8>> collisionInfoIdMap;

void ClearCollisionMap()
{
	// Should only be called when you're sure there should be nothing in the map
	if (collisionInfoIdMap.size() > 0) {
		collisionInfoIdMap.clear();
	}
}

UInt32 GetSavedCollision(UInt32 id)
{
	try {
		return collisionInfoIdMap.at(id).first;
	}
	catch (std::out_of_range) {
		// do not have saved filter info for this entity... it must have been added somehow between saving and reseting
		return 0;
	}
}

UInt32 GetSavedCollisionRefCount(UInt32 id)
{
	try {
		return collisionInfoIdMap.at(id).second;
	}
	catch (std::out_of_range) {
		// do not have saved filter info for this entity... it must have been added somehow between saving and reseting
		return 0;
	}
}

void RemoveSavedCollision(UInt32 id)
{
	try {
		auto val = collisionInfoIdMap.at(id);
		UInt8 count = val.second;
		UInt32 collisionFilterInfo = val.first;
		if (count == 1) {
			// Other hand is not affecting this entity
			collisionInfoIdMap.erase(id);
		}
		else {
			// Other hand is still affecting this entity - 'decref'
			collisionInfoIdMap[id] = { collisionFilterInfo, count - 1 };
		}
	}
	catch (std::out_of_range) {
		// do not have saved filter info for this entity... it must have been added somehow between saving and reseting
	}
}


void SetCollisionInfoDownstream(NiAVObject *obj, UInt32 collisionGroup)
{
	if (obj->unk040) {
		auto collObj = (bhkCollisionObject *)obj->unk040;
		hkpRigidBody *entity = collObj->body->hkBody;
		if (entity->m_world) {
			hkpCollidable *collidable = &entity->m_collidable;

			// Save collisionfilterinfo by entity id
			UInt32 entityId = entity->m_uid;
			UInt8 count = GetSavedCollisionRefCount(entityId);
			if (count > 0) {
				// other hand already did the job - 'incRef'
				UInt32 savedInfo = GetSavedCollision(entityId);
				collisionInfoIdMap[entityId] = { savedInfo, count + 1 };
			}
			else {
				// Other hand hasn't affected this yet. Set its collision info
				collisionInfoIdMap[entityId] = { collidable->m_broadPhaseHandle.m_collisionFilterInfo, 1 };

				bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
				world->worldLock.LockForWrite();

				collidable->m_broadPhaseHandle.m_collisionFilterInfo &= 0x0000FFFF;
				collidable->m_broadPhaseHandle.m_collisionFilterInfo |= collisionGroup << 16;
				collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7F; // clear out layer
				collidable->m_broadPhaseHandle.m_collisionFilterInfo |= 56; // our custom layer
				// set bit 15. This way it won't collide with the player, but _will_ collide with other objects that also have bit 15 set (i.e. other things we pick up).
				collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (1 << 15); // Why bit 15? It's just the way the collision works.

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
				SetCollisionInfoDownstream(child, collisionGroup);
			}
		}
	}
}

void SetCollisionInfoForAllCollisionInRefr(TESObjectREFR *refr, UInt32 collisionGroup)
{
	if (refr->loadedState && refr->loadedState->node) {
		SetCollisionInfoDownstream(refr->loadedState->node, collisionGroup);
	}
}


void ResetCollisionInfoDownstream(NiAVObject *obj, hkpCollidable *skipNode)
{
	if (obj->unk040) {
		auto collObj = (bhkCollisionObject *)obj->unk040;
		hkpRigidBody *entity = collObj->body->hkBody;
		if (entity->m_world) {
			hkpCollidable *collidable = &entity->m_collidable;
			if (collidable != skipNode) {
				UInt32 entityId = entity->m_uid;
				UInt8 refCount = GetSavedCollisionRefCount(entityId);
				if (refCount > 0) {
					if (refCount == 1) {
						// Only actually reset collision info if the other hand isn't involved
						UInt32 savedCollision = GetSavedCollision(entityId);

						bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
						world->worldLock.LockForWrite();

						// Restore only the original layer first, so it collides with everything except the player
						collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7f;
						collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (savedCollision & 0x7f);
						hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

						// Do not do a full check. What that means is it won't colide with the player until they stop colliding.
						collidable->m_broadPhaseHandle.m_collisionFilterInfo = savedCollision;
						hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

						world->worldLock.UnlockWrite();
					}
					RemoveSavedCollision(entityId);
				}
			}
		}
	}

	NiNode *node = obj->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				ResetCollisionInfoDownstream(child, skipNode);
			}
		}
	}
}

void ResetCollisionInfoForAllCollisionInRefr(TESObjectREFR *refr, hkpCollidable *skipNode)
{
	if (refr->loadedState && refr->loadedState->node) {
		ResetCollisionInfoDownstream(refr->loadedState->node, skipNode);
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
