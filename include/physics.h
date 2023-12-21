#pragma once

#include <unordered_set>
#include <vector>

#include "RE/havok.h"

#include <Physics/Collide/Shape/Query/hkpRayHitCollector.h>
#include <Physics/Collide/Shape/Query/hkpShapeRayCastCollectorOutput.h>
#include <Physics/Collide/Agent/Query/hkpCdPointCollector.h>
#include <Physics/Collide/Agent/Query/hkpCdBodyPairCollector.h>
#include <Physics/Collide/Agent/Collidable/hkpCdPoint.h>
#include <Physics/Dynamics/Collide/ContactListener/hkpContactListener.h>
#include <Physics/Dynamics/World/Listener/hkpIslandActivationListener.h>
#include <Physics/Dynamics/World/Listener/hkpWorldPostSimulationListener.h>

#include "skse64/GameReferences.h"


struct RayHitCollector : public hkpRayHitCollector
{
public:
    RayHitCollector();
    inline void reset();
    void addRayHit(const hkpCdBody& cdBody, const hkpShapeRayCastCollectorOutput& hitInfo) override;

    hkpShapeRayCastCollectorOutput m_closestHitInfo;
    bool m_doesHitExist = false;
    //const hkpCdBody *m_closestCollidable = nullptr;
};

struct AllRayHitCollector : public hkpRayHitCollector
{
public:
    AllRayHitCollector();
    inline void reset();
    void addRayHit(const hkpCdBody& cdBody, const hkpShapeRayCastCollectorOutput& hitInfo) override;

    std::vector<std::pair<hkpCdBody *, hkpShapeRayCastCollectorOutput>> m_hits;
};

struct CdPointCollector : public hkpCdPointCollector
{
    CdPointCollector();
    void addCdPoint(const hkpCdPoint& point) override;
    inline void reset() override;

    std::vector<std::pair<hkpCdBody *, hkContactPoint>> m_hits;
};

struct SpecificPointCollector : public hkpCdPointCollector
{
    SpecificPointCollector();
    void addCdPoint(const hkpCdPoint& point) override;
    void reset() override;

    hkpCdBody *m_target = nullptr;
    hkContactPoint m_contactPoint;
    bool m_foundTarget = false;
};

struct AnyPointCollector : public hkpCdPointCollector
{
    AnyPointCollector();
    void addCdPoint(const hkpCdPoint& point) override;
    void reset() override;

    float maxDistance = 0.f;
    bool m_anyHits = false;
};

struct CdBodyPairCollector : public hkpCdBodyPairCollector
{
    CdBodyPairCollector();
    void addCdBodyPair(const hkpCdBody& bodyA, const hkpCdBody& bodyB) override;
    void reset() override;

    std::vector<hkpCdBody *> m_hits;
};

struct SpecificPairCollector : public hkpCdBodyPairCollector
{
    SpecificPairCollector();
    void addCdBodyPair(const hkpCdBody& bodyA, const hkpCdBody& bodyB) override;
    void reset() override;

    hkpCdBody *m_target = nullptr;
    bool m_foundTarget = false;
};

struct AnyPairCollector : public hkpCdBodyPairCollector
{
    AnyPairCollector();
    void addCdBodyPair(const hkpCdBody& bodyA, const hkpCdBody& bodyB) override;
    void reset() override;

    bool m_anyHits = false;
};;

struct IslandDeactivationListener : public hkpIslandActivationListener
{
    void islandActivatedCallback(hkpSimulationIsland* island) override;
    void islandDeactivatedCallback(hkpSimulationIsland* island) override;
};

enum class HandIndex : UInt32 {
    Right = 0,
    Left = 1,
    Both = 2
};

struct PhysicsListener : public hkpContactListener, hkpWorldPostSimulationListener
{
    void contactPointCallback(const hkpContactPointEvent& evnt) override;
    void postSimulationCallback(hkpWorld* world) override;

    void RegisterHandCollision(hkpRigidBody *body, float separatingVelocity, HandIndex handIndex);
    void RegisterHandCollision(hkpRigidBody *body, float separatingVelocity, bool isLeft);

    void DisableContactsTemporarily(hkpRigidBody *bodyA, hkpRigidBody *bodyB, double duration);
    void HandleIgnoredContact(const hkpContactPointEvent &evnt);

    struct HandCollisionData
    {
        struct RigidBodyCollisionData {
            int collidedFrame;
            float inverseMass;
            float velocity;
        };

        std::unordered_map<hkpRigidBody *, RigidBodyCollisionData> collidedBodies{};
        std::unordered_set<hkpRigidBody *> prevCollidedBodies{}; // rigidbodies that were collided with last frame
    };

    static std::mutex handLocks[2];
    HandCollisionData handData[2];

    NiPointer<bhkWorld> world = nullptr;

    struct IgnoreContactPointData {
        hkpRigidBody *body;
        double startTime;
        double ignoreTime;
    };
    std::unordered_map<hkpRigidBody *, IgnoreContactPointData> ignoreContactPointData{};
};
extern PhysicsListener g_physicsListener;

struct EntityCollisionListener : public hkpContactListener
{
    EntityCollisionListener(bool isLeft) : isLeft(isLeft) {}

    void contactPointCallback(const hkpContactPointEvent& evnt) override;

    void PostSimulationUpdate();
    void RegisterCollision(hkpRigidBody *body);
    bool IsColliding();

    bool isLeft;
    std::mutex collisionLock;
    std::unordered_map<hkpRigidBody *, int> collidedBodies{};
};
extern EntityCollisionListener g_rightEntityCollisionListener;
extern EntityCollisionListener g_leftEntityCollisionListener;

namespace CollisionInfo
{
    // 5-bit ragdoll layer. If bit 15 is set, then if layer differs by 1 we don't collide. If layer differs by !=1 we do.
    enum class RagdollLayer : UInt8
    {
        SkipNone = 0, // Collides with both hands + held objects
        SkipRight = 2, // Collides with all except right hand
        RightHand = 3, // Used for right hand
        SkipBoth = 4, // Collides with all except right/left hand
        LeftHand = 5, // Used for left hand
        SkipLeft = 6 // Collides with all except left hand
    };
}

void AddHiggsCollisionLayer(bhkWorld *world);
void EnsureHiggsCollisionLayer(bhkWorld *world);
void ReSyncLayerBitfields(bhkCollisionFilter *filter, UInt8 layer);

void ApplyHardKeyframeVelocityClamped(const hkVector4& nextPosition, const hkQuaternion& nextOrientation, hkReal invDeltaTime, bhkRigidBody* body);

void SetVelocityDownstream(NiAVObject *obj, hkVector4 velocity);
void SetAngularVelocityDownstream(NiAVObject *obj, hkVector4 velocity);
void ApplyHardKeyframeDownstream(NiAVObject *obj, hkVector4 pos, hkQuaternion rot, hkReal invDeltaTime);

void hkpWorld_removeContactListener(hkpWorld *_this, hkpContactListener* worldListener);
float hkpContactPointEvent_getSeparatingVelocity(const hkpContactPointEvent &_this);

bool IsColliding(const hkpRigidBody *rigidBody, float tolerance = 0.005f);

