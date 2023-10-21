#include <sstream>

#include "RE/offsets.h"
#include "physics.h"
#include "utils.h"
#include "pluginapi.h"
#include "hand.h"
#include "config.h"
#include "main.h"

#include "skse64/NiNodes.h"
#include "skse64/gamethreads.h"
#include "skse64/GameRTTI.h"


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

SpecificPointCollector::SpecificPointCollector()
{
    reset();
}

void SpecificPointCollector::reset()
{
    m_earlyOutDistance = 1.0f;
    m_foundTarget = false;
    m_target = nullptr;
}

void SpecificPointCollector::addCdPoint(const hkpCdPoint& point)
{
    hkpCdBody *cdBody = const_cast<hkpCdBody *>(&point.m_cdBodyB);
    while (cdBody->m_parent) {
        cdBody = const_cast<hkpCdBody *>(cdBody->m_parent);
    }

    if (cdBody == m_target) {
        m_foundTarget = true;
        m_contactPoint = point.getContact();
        m_earlyOutDistance = 0.f;
    }
}

AnyPointCollector::AnyPointCollector()
{
    reset();
}

void AnyPointCollector::reset()
{
    m_earlyOutDistance = 1.0f;
    m_anyHits = false;
}

void AnyPointCollector::addCdPoint(const hkpCdPoint &point)
{
    if (point.getContact().getDistance() > maxDistance) return;

    m_anyHits = true;
    m_earlyOutDistance = 0.f;
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

SpecificPairCollector::SpecificPairCollector()
{
    reset();
}

void SpecificPairCollector::reset()
{
    m_earlyOut = false;
    m_foundTarget = false;
    m_target = nullptr;
}

void SpecificPairCollector::addCdBodyPair(const hkpCdBody& bodyA, const hkpCdBody& bodyB)
{
    // Note: for optimization purposes this should set the m_earlyOut:
    // - true if you want to get no more hits
    // - false if you want to get more hits (which is the default)

    hkpCdBody *cdBody = const_cast<hkpCdBody *>(&bodyB);
    while (cdBody->m_parent) {
        cdBody = const_cast<hkpCdBody *>(cdBody->m_parent);
    }

    if (cdBody == m_target) {
        m_foundTarget = true;
        m_earlyOut = true;
    }
}

AnyPairCollector::AnyPairCollector()
{
    reset();
}

void AnyPairCollector::reset()
{
    m_earlyOut = false;
    m_anyHits = false;
}

void AnyPairCollector::addCdBodyPair(const hkpCdBody &bodyA, const hkpCdBody &bodyB)
{
    m_anyHits = true;
    m_earlyOut = true;
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


void IslandDeactivationListener::islandActivatedCallback(hkpSimulationIsland* island) {}

void IslandDeactivationListener::islandDeactivatedCallback(hkpSimulationIsland* island)
{
    int numEntities = island->m_entities.getSize();
    if (numEntities <= 0) return;

    if (g_shadowUpdateFrame == *g_currentFrameCounter) {
        return; // We're already doing an update this frame
    }

    auto SetShadowsToUpdateThisFrame = []() {
        //_MESSAGE("Island deactived on frame %d", *g_currentFrameCounter);
        if (g_savedShadowUpdateFrameDelay == -1) {
            g_savedShadowUpdateFrameDelay = *g_iShadowUpdateFrameDelay;
        }

        // These are values within the game
        *g_nextShadowUpdateFrameCount = *g_currentFrameCounter;
        *g_iShadowUpdateFrameDelay = 1;

        // These are mine, used to keep track of when we want to stop updating
        g_shadowUpdateFrame = *g_currentFrameCounter;
        g_numShadowUpdates = 1;
    };

    if (numEntities > Config::options.maxNumEntitiesPerSimulationIslandToCheck) {
        SetShadowsToUpdateThisFrame();
        return;
    }

    bool anyToProcess = false;
    for (hkpEntity *entity : island->m_entities) {
        if (IsMoveableEntity(entity) &&
            ((entity->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f) != 0x21) && // 0x21 == L_BIPED_NO_CC
            (entity->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f) != 0x20 // 0x20 == L_DEADBIP
            ) {
            anyToProcess = true;
            break;
        }
    }

    if (anyToProcess) {
        PlayerCharacter *player = *g_thePlayer;
        float havokWorldScale = *g_havokWorldScale;
        NiPoint3 playerPos = player->pos * havokWorldScale;

        bool isAnyWithinRadius = false;
        for (hkpEntity *entity : island->m_entities) {
            NiPoint3 pos = HkVectorToNiPoint(entity->m_motion.getTransform().m_translation);
            float distFromPlayer = VectorLength(pos - playerPos);
            if (distFromPlayer < Config::options.maxDistanceOfSimulationIslandToUpdate) {
                isAnyWithinRadius = true;
                break;
            }
        }

        if (isAnyWithinRadius) {
            SetShadowsToUpdateThisFrame();
        }
    }
}


struct CreateDetectionEventTask : TaskDelegate
{
    static CreateDetectionEventTask * Create(ActorProcessManager *ownerProcess, Actor *owner, NiPoint3 position, int soundLevel, TESObjectREFR *source) {
        CreateDetectionEventTask * cmd = new CreateDetectionEventTask;
        if (cmd)
        {
            cmd->ownerProcess = ownerProcess;
            cmd->owner = owner;
            cmd->position = position;
            cmd->soundLevel = soundLevel;
            cmd->source = source;
        }
        return cmd;
    }
    virtual void Run() {
        CreateDetectionEvent(ownerProcess, owner, &position, soundLevel, source);
    }
    virtual void Dispose() {
        delete this;
    }

    ActorProcessManager *ownerProcess;
    Actor* owner;
    NiPoint3 position;
    int soundLevel;
    TESObjectREFR *source;
};

inline bool IsLeftRigidBody(hkpRigidBody *rigidBody)
{
    bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
    if (!wrapper) return false;
    return wrapper == g_leftHand->handBody || wrapper == g_leftHand->weaponBody || (g_leftHand->HasHeldObject() && wrapper == g_leftHand->selectedObject.rigidBody);
}

inline bool IsWeaponRigidBody(hkpRigidBody *rigidBody)
{
    bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
    if (!wrapper) return false;
    return wrapper == g_leftHand->weaponBody || wrapper == g_rightHand->weaponBody;
}

inline bool IsHandRigidBody(hkpRigidBody *rigidBody)
{
    bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
    if (!wrapper) return false;
    return wrapper == g_leftHand->handBody || wrapper == g_rightHand->handBody;
}

inline bool IsHeldRigidBody(hkpRigidBody *rigidBody)
{
    bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
    if (!wrapper) return false;
    return (g_leftHand->HasHeldObject() && wrapper == g_leftHand->selectedObject.rigidBody) ||
        (g_rightHand->HasHeldObject() && wrapper == g_rightHand->selectedObject.rigidBody);
}

inline bool IsHiggsRigidBody(hkpRigidBody *rigidBody)
{
    if ((rigidBody->getCollidable()->getBroadPhaseHandle()->getCollisionFilterInfo() & 0x7f) != 56) {
        return false;
    }
    return IsHandRigidBody(rigidBody) || IsWeaponRigidBody(rigidBody) || IsHeldRigidBody(rigidBody);
}

std::mutex PhysicsListener::handLocks[2]{};

void TriggerCollisionHaptics(float inverseMass, float speed, bool isLeft) {
    float mass = inverseMass ? 1.0f / inverseMass : 10000.0f;

    if (g_rightHand->IsTwoHanding() || g_leftHand->IsTwoHanding()) {
        // Both hands are holding an equipped weapon - play haptics for both
        g_leftHand->TriggerCollisionHaptics(mass, speed);
        g_rightHand->TriggerCollisionHaptics(mass, speed);
        HiggsPluginAPI::TriggerCollisionCallbacks(true, mass, speed);
        HiggsPluginAPI::TriggerCollisionCallbacks(false, mass, speed);
    }
    else {
        if (isLeft) {
            g_leftHand->TriggerCollisionHaptics(mass, speed);
        }
        else {
            g_rightHand->TriggerCollisionHaptics(mass, speed);
        }

        HiggsPluginAPI::TriggerCollisionCallbacks(isLeft, mass, speed);
    }
};

void TriggerCollisionHapticsUsingHandSpeed(float inverseMass, bool isLeft) {
    float mass = inverseMass ? 1.0f / inverseMass : 10000.0f;

    if (g_rightHand->IsTwoHanding() || g_leftHand->IsTwoHanding()) {
        // Both hands are holding an equipped weapon - play haptics for both
        float speed = g_rightHand->avgPlayerSpeedWorldspace + max(g_rightHand->controllerData.avgSpeed, g_leftHand->controllerData.avgSpeed);
        g_leftHand->TriggerCollisionHaptics(mass, speed);
        g_rightHand->TriggerCollisionHaptics(mass, speed);
        HiggsPluginAPI::TriggerCollisionCallbacks(true, mass, speed);
        HiggsPluginAPI::TriggerCollisionCallbacks(false, mass, speed);
    }
    else {
        if (isLeft) {
            float speed = g_leftHand->avgPlayerSpeedWorldspace + g_leftHand->controllerData.avgSpeed;
            g_leftHand->TriggerCollisionHaptics(mass, speed);
            HiggsPluginAPI::TriggerCollisionCallbacks(isLeft, mass, speed);
        }
        else {
            float speed = g_rightHand->avgPlayerSpeedWorldspace + g_rightHand->controllerData.avgSpeed;
            g_rightHand->TriggerCollisionHaptics(mass, speed);
            HiggsPluginAPI::TriggerCollisionCallbacks(isLeft, mass, speed);
        }
    }
}

void PhysicsListener::RegisterHandCollision(hkpRigidBody *body, float separatingVelocity, bool isLeft)
{
    auto &collidedBodies = handData[isLeft].collidedBodies;
    if (auto it = collidedBodies.find(body); it == collidedBodies.end()) {
        // It's not in the map yet
        std::unique_lock lock(handLocks[isLeft]);
        float collisionVelocity = separatingVelocity;
        if (it = collidedBodies.find(body); it != collidedBodies.end()) {
            // It's been added between when we checked above and when we acquired the lock
            collisionVelocity = max(separatingVelocity, it->second.velocity);
        }
        collidedBodies[body] = { *g_currentFrameCounter, body->getMassInv(), collisionVelocity };
    }
    else {
        // It's already in the map, so just update the collided frame
        it->second.collidedFrame = *g_currentFrameCounter;
    }
}

void PhysicsListener::contactPointCallback(const hkpContactPointEvent& evnt)
{
    if (evnt.m_contactPointProperties->m_flags & hkContactPointMaterial::FlagEnum::CONTACT_IS_DISABLED) {
        // Early out
        return;
    }

    hkpRigidBody *rigidBodyA = evnt.m_bodies[0];
    hkpRigidBody *rigidBodyB = evnt.m_bodies[1];

    UInt32 layerA = rigidBodyA->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
    UInt32 layerB = rigidBodyB->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo & 0x7f;
    if (layerA != 56 && layerB != 56) return; // Every collision we care about involves a body with our custom layer (hand, held object...)

    // Ensure full manifold callbacks for any higgs collisions
    evnt.m_contactMgr->m_contactPointCallbackDelay = 0;

    float separatingVelocity = fabs(hkpContactPointEvent_getSeparatingVelocity(evnt));

    if (evnt.m_contactPointProperties->wasUsed() && evnt.m_contactPoint->getDistance() < Config::options.collisionMaxInitialContactPointDistance) {
        if (IsHiggsRigidBody(rigidBodyA)) {
            RegisterHandCollision(rigidBodyB, separatingVelocity, IsLeftRigidBody(rigidBodyA));
        }

        if (IsHiggsRigidBody(rigidBodyB)) {
            RegisterHandCollision(rigidBodyA, separatingVelocity, IsLeftRigidBody(rigidBodyB));
        }
    }

    if (evnt.m_contactPointProperties->isPotential()) {
        if (separatingVelocity < Config::options.collisionMinHapticSpeed) {
            return;
        }

        if (IsHiggsRigidBody(rigidBodyA)) {
            bool isLeft = IsLeftRigidBody(rigidBodyA);
            TriggerCollisionHaptics(rigidBodyB->getMassInv(), separatingVelocity, isLeft);
        }

        if (IsHiggsRigidBody(rigidBodyB)) {
            bool isLeft = IsLeftRigidBody(rigidBodyB);
            TriggerCollisionHaptics(rigidBodyA->getMassInv(), separatingVelocity, isLeft);
        }
    }

    /*
    TESObjectREFR *ref = GetRefFromCollidable(&otherBody->m_collidable);
    hkContactPoint *contactPoint = evnt.m_contactPoint;
    if (ref && contactPoint) {
        PlayerCharacter *player = *g_thePlayer;
        NiPoint3 position = HkVectorToNiPoint(contactPoint->getPosition()) * *g_inverseHavokWorldScale;
        // Very Loud == 200, Silent == 0, Normal == 50, Loud == 100
        int soundLevel = 50;
        g_taskInterface->AddTask(CreateDetectionEventTask::Create(player->processManager, player, position, soundLevel, ref));
    }
    */
}

void PhysicsListener::postSimulationCallback(hkpWorld* world)
{
    int currentFrame = *g_currentFrameCounter;

    for (int isLeft = 0; isLeft < 2; isLeft++) { // for each hand
        auto &collidedBodies = handData[isLeft].collidedBodies;
        auto &previousCollidedBodies = handData[isLeft].prevCollidedBodies;

        for (auto it = collidedBodies.begin(); it != collidedBodies.end();) {
            auto[body, collisionData] = *it;

            if (currentFrame - collisionData.collidedFrame < Config::options.collisionMaxInactiveFramesToConsiderActive) {
                // Collision is active
                if (!previousCollidedBodies.count(body)) {
                    previousCollidedBodies.insert(body);
                    // No used contact points for this body last frame, but yes this frame
                    TriggerCollisionHaptics(collisionData.inverseMass, collisionData.velocity, isLeft);
                }
            }
            else {
                // Collision is inactive
                if (auto it = previousCollidedBodies.find(body); it != previousCollidedBodies.end()) {
                    // There were used contact points for this body last frame, but not anymore
                    previousCollidedBodies.erase(it);
                }
            }

            if (currentFrame - collisionData.collidedFrame > Config::options.collisionMaxInactiveFramesBeforeCleanup) {
                it = collidedBodies.erase(it);
            }
            else {
                ++it;
            }
        }

        for (auto it = previousCollidedBodies.begin(); it != previousCollidedBodies.end();) {
            hkpRigidBody *body = *it;
            if (!collidedBodies.count(body)) {
                it = previousCollidedBodies.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    g_rightEntityCollisionListener.PostSimulationUpdate();
    g_leftEntityCollisionListener.PostSimulationUpdate();
}


EntityCollisionListener g_rightEntityCollisionListener(false);
EntityCollisionListener g_leftEntityCollisionListener(true);

void EntityCollisionListener::RegisterCollision(hkpRigidBody *body)
{
    if (auto it = collidedBodies.find(body); it == collidedBodies.end()) {
        // It's not in the map yet
        std::unique_lock lock(collisionLock);
        collidedBodies[body] = *g_currentFrameCounter;
    }
    else {
        // It's already in the map, so just update the collided frame
        it->second = *g_currentFrameCounter;
    }
}

void EntityCollisionListener::contactPointCallback(const hkpContactPointEvent &evnt)
{
    if (evnt.m_contactPointProperties->m_flags & hkContactPointMaterial::FlagEnum::CONTACT_IS_DISABLED) {
        // Early out
        return;
    }

    if (evnt.m_contactPointProperties->wasUsed() && evnt.m_contactPoint->getDistance() < Config::options.collisionMaxInitialContactPointDistance) {
        RegisterCollision(evnt.m_source == hkpContactPointEvent::SOURCE_A ? evnt.m_bodies[0] : evnt.m_bodies[1]);
    }
}

void EntityCollisionListener::PostSimulationUpdate()
{
    for (auto it = collidedBodies.begin(); it != collidedBodies.end();) {
        auto [body, collidedFrame] = *it;

        if (*g_currentFrameCounter - collidedFrame > Config::options.collisionMaxInactiveFramesToConsiderActive) {
            // Collision is inactive
            it = collidedBodies.erase(it);
        }
        else {
            // Collision is active
            ++it;
        }
    }
}

bool EntityCollisionListener::IsColliding()
{
    return collidedBodies.size() > 0;
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
        auto rigidBody = GetRigidBody(obj);
        if (rigidBody) {
            hkpRigidBody *entity = rigidBody->hkBody;
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
                        {
                            BSWriteLocker lock(&world->worldLock);

                            UInt8 ragdollBits = (UInt8)RagdollLayer::SkipBoth;
                            collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
                            collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (ragdollBits << 8);

                            hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
                        }
                    }
                    else if (savedState == State::Unheld && (reason == State::HeldLeft || reason == State::HeldRight)) {
                        // No hand holds it -> one hand holds it
                        // I don't think this case is actually possible, since I try to reset a pull when grabbing, but it's here just in case.

                        collisionInfoIdMap[entityId] = { savedInfo, reason };

                        bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
                        {
                            BSWriteLocker lock(&world->worldLock);

                            // We don't set the layer when pulling, so now that we're grabbing the object we need to set it.
                            collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7F; // clear out layer
                            collidable->m_broadPhaseHandle.m_collisionFilterInfo |= 56; // our custom layer

                            UInt8 ragdollBits = (UInt8)(reason == State::HeldLeft ? RagdollLayer::SkipLeft : RagdollLayer::SkipRight);
                            collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
                            collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (ragdollBits << 8);

                            hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
                        }
                    }
                }
                else {
                    // Not in the map yet - init to the necessary state

                    collisionInfoIdMap[entityId] = { collidable->m_broadPhaseHandle.m_collisionFilterInfo, reason };

                    bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
                    {
                        BSWriteLocker lock(&world->worldLock);

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

                        hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
                    }
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

    void SetCollisionGroupDownstream(NiAVObject *obj, UInt32 collisionGroup, State reason)
    {
        auto rigidBody = GetRigidBody(obj);
        if (rigidBody) {
            hkpRigidBody *entity = rigidBody->hkBody;
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
                    }
                    else if (savedState == State::Unheld && (reason == State::HeldLeft || reason == State::HeldRight)) {
                        // No hand holds it -> one hand holds it
                        collisionInfoIdMap[entityId] = { savedInfo, reason };
                    }
                }
                else {
                    // Not in the map yet - init to the necessary state

                    collisionInfoIdMap[entityId] = { collidable->m_broadPhaseHandle.m_collisionFilterInfo, reason };

                    bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
                    {
                        BSWriteLocker lock(&world->worldLock);

                        collidable->m_broadPhaseHandle.m_collisionFilterInfo &= 0x0000FFFF;
                        collidable->m_broadPhaseHandle.m_collisionFilterInfo |= collisionGroup << 16;

                        hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
                    }
                }
            }
        }

        NiNode *node = obj->GetAsNiNode();
        if (node) {
            for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
                auto child = node->m_children.m_data[i];
                if (child) {
                    SetCollisionGroupDownstream(child, collisionGroup, reason);
                }
            }
        }
    }

    void ResetCollisionInfoKeyframed(bhkRigidBody *entity, hkpMotion::MotionType motionType, hkInt8 quality, State reason, bool collideAll, bool collideNone)
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

                if (collideAll) {
                    entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
                    entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt8)RagdollLayer::SkipNone << 8);
                }
                else if (collideNone) {
                    entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
                    entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt8)RagdollLayer::SkipBoth << 8);
                }

                entity->hkBody->m_collidable.m_broadPhaseHandle.m_objectQualityType = HK_COLLIDABLE_QUALITY_CRITICAL; // Will make object collide with other things as motion type is changed
                bhkRigidBody_setMotionType(entity, motionType, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK);

                entity->hkBody->m_collidable.m_broadPhaseHandle.m_collisionFilterInfo = savedInfo;
                entity->hkBody->m_collidable.m_broadPhaseHandle.m_objectQualityType = quality;
                bhkRigidBody_setMotionType(entity, motionType, HK_ENTITY_ACTIVATION_DO_ACTIVATE, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY);
            }
        }
    }

    void ResetCollisionInfoDownstream(NiAVObject *obj, State reason, hkpCollidable *skipNode, bool collideAll, bool collideNone)
    {
        auto rigidBody = GetRigidBody(obj);
        if (rigidBody) {
            hkpRigidBody *entity = rigidBody->hkBody;
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
                            {
                                BSWriteLocker lock(&world->worldLock);

                                UInt8 ragdollBits = (UInt8)(otherHand == State::HeldLeft ? RagdollLayer::SkipLeft : RagdollLayer::SkipRight);
                                collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
                                collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (ragdollBits << 8);

                                hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
                            }
                        }
                        else if ((savedState == State::HeldLeft && reason == State::HeldLeft) ||
                            (savedState == State::HeldRight && reason == State::HeldRight) ||
                            (savedState == State::Unheld && reason == State::Unheld)) {
                            // One hand holds the object -> one hand lets go of the object, so none hold it
                            // Or, no hand was holding, and none is holding now

                            collisionInfoIdMap.erase(entityId);

                            bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
                            {
                                BSWriteLocker lock(&world->worldLock);

                                // Restore only the original layer and ragdoll bits first, so it collides with everything except the player (but still the hands)
                                collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7f;
                                collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (savedInfo & 0x7f);

                                if (collideAll) {
                                    collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
                                    collidable->m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt8)RagdollLayer::SkipNone << 8);
                                }
                                else if (collideNone) {
                                    collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~(0x1f << 8);
                                    collidable->m_broadPhaseHandle.m_collisionFilterInfo |= ((UInt8)RagdollLayer::SkipBoth << 8);
                                }

                                hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

                                // Do not do a full check. What that means is it won't colide with the player until they stop colliding.
                                collidable->m_broadPhaseHandle.m_collisionFilterInfo = savedInfo;
                                hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
                            }
                        }
                    }
                }
            }
        }

        NiNode *node = obj->GetAsNiNode();
        if (node) {
            for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
                auto child = node->m_children.m_data[i];
                if (child) {
                    ResetCollisionInfoDownstream(child, reason, skipNode, collideAll);
                }
            }
        }
    }

    void ResetCollisionGroupDownstream(NiAVObject *obj, State reason, hkpCollidable *skipNode)
    {
        auto rigidBody = GetRigidBody(obj);
        if (rigidBody) {
            hkpRigidBody *entity = rigidBody->hkBody;
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
                        }
                        else if ((savedState == State::HeldLeft && reason == State::HeldLeft) ||
                            (savedState == State::HeldRight && reason == State::HeldRight) ||
                            (savedState == State::Unheld && reason == State::Unheld)) {
                            // One hand holds the object -> one hand lets go of the object, so none hold it
                            // Or, no hand was holding, and none is holding now

                            collisionInfoIdMap.erase(entityId);

                            bhkWorld *world = (bhkWorld *)static_cast<ahkpWorld *>(entity->m_world)->m_userData;
                            {
                                BSWriteLocker lock(&world->worldLock);

                                // Do not do a full check. What that means is it won't colide with the player until they stop colliding.
                                collidable->m_broadPhaseHandle.m_collisionFilterInfo = savedInfo;
                                hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
                            }
                        }
                    }
                }
            }
        }

        NiNode *node = obj->GetAsNiNode();
        if (node) {
            for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
                auto child = node->m_children.m_data[i];
                if (child) {
                    ResetCollisionGroupDownstream(child, reason, skipNode);
                }
            }
        }
    }
}


void AddHiggsCollisionLayer(bhkWorld *world)
{
    // Create our own layer in the first unused vanilla layer (56)
    bhkCollisionFilter *worldFilter = (bhkCollisionFilter *)world->world->m_collisionFilter;
    worldFilter->layerBitfields[56] = g_interface001.higgsLayerBitfield;
    worldFilter->layerNames[56] = BSFixedString("L_HIGGSCOLLISION");
    // Set whether other layers should collide with our new layer
    ReSyncLayerBitfields(worldFilter, 56);
}

void EnsureHiggsCollisionLayer(bhkWorld *world)
{
    bhkCollisionFilter *worldFilter = (bhkCollisionFilter *)world->world->m_collisionFilter;
    UInt64 currentHiggsBitfield = worldFilter->layerBitfields[56];
    if (currentHiggsBitfield != g_interface001.higgsLayerBitfield) {
        BSWriteLocker lock(&world->worldLock);
        worldFilter->layerBitfields[56] = g_interface001.higgsLayerBitfield;
        ReSyncLayerBitfields(worldFilter, 56);
    }
}

void ReSyncLayerBitfields(bhkCollisionFilter *filter, UInt8 layer)
{
    UInt64 bitfield = filter->layerBitfields[layer];
    for (int i = 0; i < 64; i++) { // 56 layers in vanilla
        if ((bitfield >> i) & 1) {
            filter->layerBitfields[i] |= ((UInt64)1 << layer);
        }
        else {
            filter->layerBitfields[i] &= ~((UInt64)1 << layer);
        }
    }
}

void ApplyHardKeyframeVelocityClamped(const hkVector4& nextPosition, const hkQuaternion& nextOrientation, hkReal invDeltaTime, bhkRigidBody* body)
{
    hkpRigidBody *hkBody = body->hkBody;
    hkpKeyFrameUtility_applyHardKeyFrame(nextPosition, nextOrientation, invDeltaTime, hkBody);

    if (VectorLength(HkVectorToNiPoint(hkBody->getLinearVelocity())) > bhkRigidBody_GetMaxLinearVelocityMetersPerSecond(body)) {
        if (ahkpWorld *world = body->GetHavokWorld_2()) {
            if (bhkWorld *worldWrapper = world->m_userData) {
                BSWriteLocker lock(&worldWrapper->worldLock);
                hkpRigidBody_setPosition(hkBody, nextPosition);
            }
        }
    }
    if (VectorLength(HkVectorToNiPoint(hkBody->getAngularVelocity())) > bhkRigidBody_GetMaxAngularVelocity(body)) {
        if (ahkpWorld *world = body->GetHavokWorld_2()) {
            if (bhkWorld *worldWrapper = world->m_userData) {
                BSWriteLocker lock(&worldWrapper->worldLock);
                hkpRigidBody_setRotation(hkBody, nextOrientation);
            }
        }
    }
}

void SetVelocityDownstream(NiAVObject *obj, hkVector4 velocity)
{
    auto bRigidBody = GetRigidBody(obj);
    if (bRigidBody) {
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

void SetAngularVelocityDownstream(NiAVObject *obj, hkVector4 velocity)
{
    auto bRigidBody = GetRigidBody(obj);
    if (bRigidBody) {
        hkpRigidBody *rigidBody = bRigidBody->hkBody;
        if (rigidBody->m_world) {
            hkpMotion *motion = &rigidBody->m_motion;

            bhkRigidBody_setActivated(bRigidBody, true);
            motion->m_angularVelocity = velocity;
        }
    }

    NiNode *node = obj->GetAsNiNode();
    if (node) {
        for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
            auto child = node->m_children.m_data[i];
            if (child) {
                SetAngularVelocityDownstream(child, velocity);
            }
        }
    }
}

void ApplyHardKeyframeDownstream(NiAVObject *obj, hkVector4 pos, hkQuaternion rot, hkReal invDeltaTime)
{
    auto bRigidBody = GetRigidBody(obj);
    if (bRigidBody) {
        hkpRigidBody *rigidBody = bRigidBody->hkBody;
        if (rigidBody->m_world) {
            hkpMotion *motion = &rigidBody->m_motion;

            bhkRigidBody_setActivated(bRigidBody, true);
            hkpKeyFrameUtility_applyHardKeyFrame(pos, rot, invDeltaTime, rigidBody);
        }
    }

    NiNode *node = obj->GetAsNiNode();
    if (node) {
        for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
            auto child = node->m_children.m_data[i];
            if (child) {
                ApplyHardKeyframeDownstream(child, pos, rot, invDeltaTime);
            }
        }
    }
}

void hkpWorld_removeContactListener(hkpWorld *_this, hkpContactListener* worldListener)
{
    hkArray<hkpContactListener *> &listeners = _this->m_contactListeners;

    for (int i = 0; i < listeners.getSize(); i++) {
        hkpContactListener *listener = listeners[i];
        if (listener == worldListener) {
            listeners[i] = nullptr;
            return;
        }
    }
}

float hkpContactPointEvent_getSeparatingVelocity(const hkpContactPointEvent &_this)
{
    if (_this.m_separatingVelocity)
    {
        return *_this.m_separatingVelocity;
    }
    else
    {
        return hkpSimpleContactConstraintUtil_calculateSeparatingVelocity(_this.m_bodies[0], _this.m_bodies[1], _this.m_bodies[0]->getCenterOfMassInWorld(), _this.m_bodies[1]->getCenterOfMassInWorld(), _this.m_contactPoint);
    }
}

bool IsColliding(const hkpRigidBody *rigidBody, float tolerance)
{
    hkpWorld *world = rigidBody->getWorld();
    if (!world) return false;

    const hkpCollidable *collidable = rigidBody->getCollidable();

    static AnyPointCollector collector{};
    collector.reset();
    collector.maxDistance = tolerance;

    hkpWorld_GetClosestPoints(world, collidable, world->getCollisionInput(), &collector);
    return collector.m_anyHits;
}

