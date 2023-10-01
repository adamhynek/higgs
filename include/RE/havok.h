#pragma once

#include <Common/Base/hkBase.h>
#include <Physics/Dynamics/Entity/hkpRigidBody.h>
#include <Physics/Dynamics/Phantom/hkpSimpleShapePhantom.h>
#include <Physics/Collide/Agent/hkpProcessCollisionInput.h>
#include <Physics/Collide/Filter/hkpCollisionFilter.h>
#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics/Collide/Agent/Collidable/hkpCdPoint.h>
#include <Physics/Collide/Shape/Query/hkpShapeRayCastCollectorOutput.h>
#include <Physics/Collide/Shape/Compound/Tree/Mopp/hkpMoppBvTreeShape.h>
#include <Physics/Dynamics/Collide/ContactListener/hkpContactPointEvent.h>
#include <Physics/Utilities/CharacterControl/CharacterProxy/hkpCharacterProxy.h>
#include <Physics/Utilities/CharacterControl/CharacterProxy/hkpCharacterProxyListener.h>
#include <Physics/Utilities/CharacterControl/CharacterRigidBody/hkpCharacterRigidBodyListener.h>
#include <Physics/Utilities/CharacterControl/CharacterRigidBody/hkpCharacterRigidBody.h>
#include <Physics/Utilities/CharacterControl/StateMachine/hkpCharacterContext.h>

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"

#include "havok_ref_ptr.h"


namespace RE
{
    template <typename T>
    struct hkArray
    {
        enum
        {
            CAPACITY_MASK = int(0x3FFFFFFF),
            FLAG_MASK = int(0xC0000000),
            DONT_DEALLOCATE_FLAG = int(0x80000000), // Indicates that the storage is not the array's to delete
            FORCE_SIGNED = -1
        };

        T *m_data = nullptr; // 00
        int m_size = 0; // 08
        int m_capacityAndFlags = 0; // 0C

        inline int getCapacity() const
        {
            return (m_capacityAndFlags & static_cast<int>(CAPACITY_MASK));
        }

        inline ~hkArray()
        {
            clearAndDeallocate();
        }

        void clear();
        void clearAndDeallocate();
        void pushBack(const T &t);
    };

}

enum class HavokProperty : hkUint32
{
    Node = 1, // NiAVObject
    CollisionObject = 2, // bhkCollisionObject
    Character = 1000, // Character
    CharacterController = 1002, // bhkCharacterController
    TelekinesisDamageMult = 314159,
    TelekinesisMass = 314160,
};

struct bhkCollisionFilter : hkpCollisionFilter
{
    UInt32 unk48;
    UInt32 nextCollisionGroup; // 4C - this gets incremented when something gets added to the world - used for unique collision groups?
    UInt32 bipedBitfields[32]; // 50 - About 0x18 of them seem to be actually filled in
    UInt32 layerCollisionGroups[64]; // D0 - if zero, use the counter from the collision filter (4C) as the collision group - afaik only 3 have non-zero entries: 9 (trees), 11 (water), and 13 (terrain)
    UInt64 layerBitfields[64]; // 1D0 - only 56 are valid in vanilla - these are used to determine which layers collide with each other
    UInt64 triggerBitfield1; // 3D0 - bit x determines if layer x should be tested against if a charcontroller with bit 15 set and bit 7 unset collides with it. Mostly used for triggers/traps
    UInt64 triggerBitfield2; // 3D8 - similar to above, but it's not used in the collision filter comparison, so not sure where it's actually used
    BSFixedString layerNames[64]; // 3E0 - only 56 are non-null
};
static_assert(offsetof(bhkCollisionFilter, nextCollisionGroup) == 0x4C);
static_assert(offsetof(bhkCollisionFilter, bipedBitfields) == 0x50);
static_assert(offsetof(bhkCollisionFilter, layerCollisionGroups) == 0xD0);
static_assert(offsetof(bhkCollisionFilter, layerBitfields) == 0x1D0);
static_assert(offsetof(bhkCollisionFilter, layerNames) == 0x3E0);

struct ahkpWorld : hkpWorld
{
    struct bhkWorld * m_userData; // 430
};
static_assert(offsetof(ahkpWorld, m_userData) == 0x430);

struct bhkRefObject : NiObject
{
    virtual void SetHavokObject(hkReferencedObject *object); // 25
    virtual void AddOrRemoveReference(bool add); // 26
};

struct bhkSerializable : bhkRefObject
{
    virtual ahkpWorld * GetHavokWorld_1(); // 27
    virtual ahkpWorld * GetHavokWorld_2(); // 28
    virtual void	  MoveToWorld(struct bhkWorld *world); // 29
    virtual void	  RemoveFromCurrentWorld(); // 2A
    virtual void	  DestroyCinfo(bool destroy); // 2B
    virtual void	  Unk_2C(void); // 2C
    virtual void	  Unk_2D(void); // 2D
    virtual void	  InitHavokFromCinfo(void *cInfo); // 2E
    virtual void *    GetOrCreateCinfo(bool &didCreateCinfo); // 2F
    virtual void	  Destroy(); // 30
    virtual void	  Unk_31(void *cInfo); // 31
};

struct bhkWorld : bhkSerializable
{
    virtual void Update(bool unk);  // 32
    virtual bool CastRay(hkpWorldRayCastInput& input); // 33 - actually takes in an extended hkpWorldRayCastInput that has the raycast output and stuff shoved at the end
    virtual bool HasSimulationIslands();  // 34
    virtual void Unk_35(void);  // 35
    virtual void Unk_36(void);  // 36

    RE::hkRefPtr<ahkpWorld> world; // 10
    UInt8 unk18[0xC598 - 0x18];
    BSReadWriteLock worldLock; // C598
    BSReadWriteLock graphLock; // C5A0
    // C530 is tArray<GraphPhysicsStepListener>
    // C570 is bhkConstraintProjector
    // C5C0 is TESTrapListener
    // C5C8 is BGSAcousticSpaceListener
    // C5D0 is hkpSuspendInactiveAgentsUtil
    // C5D8 is some sort of counter
};
static_assert(offsetof(bhkWorld, world) == 0x10);
static_assert(offsetof(bhkWorld, worldLock) == 0xC598);

struct bhkShape : bhkSerializable
{
    RE::hkRefPtr<hkpShape> shape; // 10
    UInt64 unk18; // == 0?
    UInt32 materialId; // 20
    UInt32 pad28;
};
static_assert(sizeof(bhkShape) == 0x28);

struct bhkSphereRepShape : bhkShape {};

struct bhkConvexShape : bhkSphereRepShape {};

struct bhkBoxShape : bhkConvexShape {};
static_assert(sizeof(bhkBoxShape) == 0x28);

struct bhkConstraint : bhkSerializable
{
    RE::hkRefPtr<hkpConstraintInstance> constraint; // 10
    struct hkConstraintCinfo *cinfo; // 18
};

struct bhkWorldObject : bhkSerializable
{
    virtual void Unk_32(void); // 32
};

struct bhkEntity : bhkWorldObject
{

};

struct bhkRigidBody : bhkEntity
{
    virtual hkVector4 & getPosition(hkVector4 &position); // 33
    virtual hkVector4 & getRotation(hkQuaternion &rotation); // 34
    virtual void setPosition(hkVector4 &position); // 35
    virtual void setRotation(hkQuaternion &rotation); // 36
    virtual void setPositionAndRotation(hkVector4 &pos, hkQuaternion &rot); // 37
    virtual hkVector4 & getCenterOfMassLocal(hkVector4 &centerOfMassLocal); // 38
    virtual hkVector4 & getCenterOfMassInWorld(hkVector4 &centerOfMassWorld); // 39
    virtual hkTransform & getTransform(hkTransform &transform); // 3A
    virtual void getAabbWorldspace(hkAabb &aabb); // 3B
    virtual void Unk_3C(void); // 3C

    RE::hkRefPtr<hkpRigidBody> hkBody; // 10
    UInt64 unk18;
    UInt8 flags; // at least first byte are some flags? bit 2 is set -> has constraints?
    tArray<NiPointer<bhkConstraint>> constraints; // 28
};
static_assert(offsetof(bhkRigidBody, constraints) == 0x28);
static_assert(sizeof(bhkRigidBody) == 0x40);

struct bhkRigidBodyT : bhkRigidBody
{
    hkQuaternion rotation; // 40
    hkVector4 translation; // 50
};
static_assert(offsetof(bhkRigidBodyT, rotation) == 0x40);
static_assert(offsetof(bhkRigidBodyT, translation) == 0x50);
static_assert(sizeof(bhkRigidBodyT) == 0x60);

struct NiCollisionObject : NiObject
{
    virtual void SetNode(NiAVObject* node); // 25
    virtual void Update(NiAVObject::ControllerUpdateContext *ctx); // 26
    // These next 3 all return immediately
    virtual void Unk_27(void); // 27 - { return; }
    virtual void Unk_28(void); // 28 - { return; }
    virtual void Unk_29(void); // 29 - { return; }

    NiAVObject * node; // 10 - points back to the NiAVObject pointing to this
};

struct bhkNiCollisionObject : NiCollisionObject
{
    virtual void GetLinearVelocity(void); // 2A
    virtual void UpdateNodeTransformsFromCollision(void) = 0; // 2B
    virtual void UpdateCollisionFromNodeTransform(void) = 0; // 2C
    virtual void ZeroOutSmallVelocities(void) = 0; // 2D
    virtual void SetMotionType(void) = 0; // 2E
    virtual void IsFixedOrKeyframed(void); // 2F
    virtual void Unk_30(void); // 30 - { return 1; }

    UInt32 flags; // 18 - flag 8 -> use blended pos (for bhkBlendCollisionObject) instead of node pos
    UInt32 pad1C; // 1C
    NiPointer<bhkWorldObject> body; // 20
};
static_assert(offsetof(bhkNiCollisionObject, body) == 0x20);

struct bhkCollisionObject : bhkNiCollisionObject
{
};

struct bhkBlendCollisionObject : bhkCollisionObject
{
    float blendStrength; // 28 - this affects how intensely to go from rigidBody position to node position. 0 means strictly follow rigidbody, 1 means strictly follow node.
    float unk2C;
    UInt32 motionType; // 30
    bhkWorld *world; // 38
    UInt32 unk40;
};
static_assert(sizeof(bhkBlendCollisionObject) == 0x48);

struct bhkSimpleShapePhantom : NiRefObject
{
    RE::hkRefPtr<hkpSimpleShapePhantom> phantom; // 10
};

struct bhkRigidBodyCinfo
{
    UInt32 collisionFilterInfo; // 00 - initd to 0
    hkpShape *shape; // 08 - initd to 0
    UInt8 unk10; // initd to 1
    UInt64 unk18; // initd to 0
    UInt32 unk20; // initd to 0
    float unk24; // initd to -0
    UInt8 unk28; // initd to 1
    UInt16 unk2A; // initd to -1 - quality type?
    hkpRigidBodyCinfo hkCinfo; // 30 - size == 0xE0
};
static_assert(offsetof(bhkRigidBodyCinfo, shape) == 0x08);
static_assert(offsetof(bhkRigidBodyCinfo, hkCinfo) == 0x30);
static_assert(sizeof(bhkRigidBodyCinfo) == 0x110);

struct hkConstraintCinfo
{
    hkConstraintCinfo();
    ~hkConstraintCinfo();

    void *vtbl = 0; // 00
    RE::hkRefPtr<hkpConstraintData> constraintData = nullptr; // 08
    hkpConstraintInstance::ConstraintPriority priority = hkpConstraintInstance::ConstraintPriority::PRIORITY_INVALID; // 10
    UInt32 pad14 = 0;
    hkpRigidBody *rigidBodyA = nullptr; // 18
    hkpRigidBody *rigidBodyB = nullptr; // 20
};

typedef void(*_hkConstraintCinfo_CreateConstraintData)(hkConstraintCinfo *); // vfunc 4

struct hkBallAndSocketConstraintCinfo : hkConstraintCinfo
{
    hkBallAndSocketConstraintCinfo(hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, NiPoint3 &pivotA, NiPoint3 &pivotB);

    // These are not necessarily 16 byte aligned
    float pivotB[4]; // 28
    float pivotA[4]; // 38
};

struct bhkGroupConstraint : bhkConstraint
{
    UInt32 collisionGroup; // 20
    UInt32 pad24;
};
static_assert(sizeof(bhkGroupConstraint) == 0x28);


struct bhkCharacterController : NiRefObject
{
    virtual void  GetPositionImpl(hkVector4 &a_pos, bool a_applyCenterOffset) const = 0; // 02
    virtual void  SetPositionImpl(const hkVector4 &a_pos, bool a_applyCenterOffset, bool a_forceWarp) = 0; // 03
    virtual void  GetTransformImpl(hkTransform &a_tranform) const = 0; // 04
    virtual void  SetTransformImpl(const hkTransform &a_tranform) = 0; // 05
    virtual void  GetLinearVelocityImpl(hkVector4 &a_velocity) const = 0; // 06
    virtual void  SetLinearVelocityImpl(const hkVector4 &a_velocity) = 0; // 07
    virtual void  GetCollisionFilterInfo(std::uint32_t &a_collisionFilterInfo) const = 0; // 08
    virtual void  SetCollisionFilterInfo(std::uint32_t filterInfo) = 0; // 09
    virtual void  Unk_0A(void) = 0; // 0A
    virtual void  Integrate(void) = 0; // 0B
    virtual void  FireMoveFinishEvent(void) = 0; // 0C
    virtual void  CheckSupportImpl() = 0; // 0D
    virtual void  MoveToWorld(bhkWorld *newWorld) = 0; // 0E
    virtual bhkWorld *GetbhkWorld(void) = 0; // 0F
    virtual hkpWorldObject *GetHavokWorldObject(void) = 0; // 10
    virtual float GetVDBAlpha() const = 0; // 11
    virtual void  Unk_12(void) = 0; // 12
    virtual void  SetTransformThroughAngularVelocity(hkTransform &a_transform) = 0;

    struct UpdateData
    {
        float deltaTime; // 00
        NiPoint3 rot; // 04 - euler
        NiPoint3 targetPos; // 10
    };

    struct CollisionEvent
    {
        NiPointer<bhkRigidBody> body; // 00
        // All 3 of these are multiplied by inverseHavokWorldScale
        NiPoint3 position; // 08
        NiPoint3 separatingNormal; // 14
        NiPoint3 bodyVelocity; // 20
    };
    static_assert(sizeof(CollisionEvent) == 0x30);

    UInt8 unk10[0x70 - 0x10];
    hkVector4                                        forwardVec;                 // 070
    hkStepInfo                                       stepInfo;                   // 080
    hkVector4                                        outVelocity;                // 090
    hkVector4                                        initialVelocity;            // 0A0
    hkVector4                                        velocityMod;                // 0B0
    hkVector4                                        direction;                  // 0C0
    hkVector4                                        rotCenter;                  // 0D0
    hkVector4                                        pushDelta;                  // 0E0
    hkVector4                                        fakeSupportStart;           // 0F0
    hkVector4                                        up;                         // 100
    hkVector4                                        supportNorm;                // 110
    UInt8                                            collisionBound[0x150 - 0x120];             // 120
    UInt8                                            bumperCollisionBound[0x180 - 0x150];       // 150
    std::uint64_t                                    unk180;                     // 180
    std::uint64_t                                    unk188;                     // 188
    struct bhkICharOrientationController *orientationCtrl;            // 190
    std::uint64_t                                    pad198;                     // 198
    hkpSurfaceInfo                                   surfaceInfo;                // 1A0
    hkpCharacterContext                              context;                    // 1E0
    UInt32                                           flags;                      // 218
    hkpCharacterStateType                            wantState;                  // 21C
    float                                            velocityTime;               // 220
    float                                            rotMod;                     // 224
    float                                            rotModTime;                 // 228
    float                                            calculatePitchTimer;        // 22C
    float                                            acrobatics;                 // 230
    float                                            center;                     // 234
    float                                            waterHeight;                // 238
    float                                            jumpHeight;                 // 23C
    float                                            fallStartHeight;            // 240
    float                                            fallTime;                   // 244
    float                                            gravity;                    // 248
    float                                            pitchAngle;                 // 24C
    float                                            rollAngle;                  // 250
    float                                            pitchMult;                  // 254
    float                                            scale;                      // 258
    float                                            swimFloatHeight;            // 25C
    float                                            actorHeight;                // 260
    float                                            speedPct;                   // 264
    std::uint32_t                                    pushCount;                  // 268
    std::uint32_t                                    unk26C;                     // 26C
    std::uint64_t                                    unk270;                     // 270
    std::uint64_t                                    unk278;                     // 278
    NiPointer<bhkShape>                              shapes[2];                  // 280
    std::uint64_t                                    unk290;                     // 290
    std::uint64_t                                    unk298;                     // 298
    std::uint64_t                                    unk2A0;                     // 2A0
    std::uint64_t                                    unk2A8;                     // 2A8
    RE::hkRefPtr<hkpRigidBody>                       supportBody;                // 2B0
    float                                            bumpedForce;                // 2B8
    std::uint32_t                                    pad2BC;                     // 2BC
    RE::hkRefPtr<hkpRigidBody>                       bumpedBody;                 // 2C0
    RE::hkRefPtr<hkpRigidBody>                       bumpedCharCollisionObject;  // 2C8
    UInt8                                            unk2D0[0x300 - 0x2D0];      // 2D0 - BSTHashMap<bhkRigidBody, CollisionEvent>
    std::uint64_t                                    unk300;                     // 300
    std::uint64_t                                    unk308;                     // 308
    std::uint64_t                                    unk310;                     // 310
    std::uint64_t                                    unk318;                     // 318
    std::uint64_t                                    unk320;                     // 320
    std::uint64_t                                    unk328;                     // 328
};
static_assert(offsetof(bhkCharacterController, context) == 0x1E0);
static_assert(sizeof(bhkCharacterController) == 0x330);

class bhkCharacterPointCollector : public hkpAllCdPointCollector
{
    UInt64 unk220;  // 220
    UInt64 unk228;  // 228
    UInt64 unk230;  // 230
    UInt64 unk238;  // 238
};
static_assert(sizeof(bhkCharacterPointCollector) == 0x240);

struct bhkCharacterProxy : bhkSerializable
{
    RE::hkRefPtr<hkpCharacterProxy> characterProxy; // 10
    UInt64 unk18;
    bhkCharacterPointCollector ignoredCollisionStartCollector; // 020
};

struct bhkCharacterRigidBody : bhkSerializable
{
    RE::hkRefPtr <hkpCharacterRigidBody> characterRigidBody; // 10
    UInt64 unk18;
    bhkRigidBody *rigidBody; // 20
    NiAVObject *unk28; // 28 - MarkerX ??
    bhkCharacterPointCollector ignoredCollisionStartCollector;  // 30
};
static_assert(offsetof(bhkCharacterRigidBody, ignoredCollisionStartCollector) == 0x30);

struct bhkCharProxyController :
    hkpCharacterProxyListener, // 000
    bhkCharacterController // 010
{
    bhkCharacterProxy proxy; // 340
};
static_assert(offsetof(bhkCharProxyController, proxy) == 0x340);

struct bhkCharRigidBodyController :
    bhkCharacterController, // 00
    hkpCharacterRigidBodyListener // 330
{
    bhkCharacterRigidBody characterRigidBody; // 340
};
static_assert(offsetof(bhkCharRigidBodyController, characterRigidBody) == 0x340);


hkMemoryRouter &hkGetMemoryRouter();
inline void *hkHeapAlloc(int numBytes) { return hkGetMemoryRouter().heap().blockAlloc(numBytes); }

template <typename T>
T * hkAllocReferencedObject() {
    T *allocated = (T *)hkHeapAlloc(sizeof(T));
    allocated->m_memSizeAndFlags = sizeof(T);
    return allocated;
}

bhkGroupConstraint *CreateBallAndSocketConstraint(hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, NiPoint3 &pivotA, NiPoint3 &pivotB);
bhkGroupConstraint *CreateBallAndSocketConstraint2(hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, NiPoint3 &pivotA, NiPoint3 &pivotB);
bhkGroupConstraint *CreateGrabConstraint(hkpRigidBody *rigidBodyA, hkpRigidBody *rigidBodyB, NiTransform &transformA, NiTransform &transformB);

NiPointer<bhkCharacterController> GetCharacterController(Actor *actor);
NiPointer<bhkCharRigidBodyController> GetCharRigidBodyController(Actor *actor);
NiPointer<bhkCharProxyController> GetCharProxyController(Actor *actor);
