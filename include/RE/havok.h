#pragma once

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"

#include <Common/Base/hkBase.h>
#include <Physics/Collide/Filter/hkpCollisionFilter.h>
#include <Physics/Dynamics/Entity/hkpRigidBody.h>
#include <Physics/Dynamics/Phantom/hkpSimpleShapePhantom.h>
#include <Physics/Collide/Agent/hkpProcessCollisionInput.h>
#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics/Collide/Agent/Collidable/hkpCdPoint.h>
#include <Physics/Collide/Shape/Query/hkpShapeRayCastCollectorOutput.h>
#include <Physics/Collide/Shape/Compound/Tree/Mopp/hkpMoppBvTreeShape.h>


struct bhkShape : NiRefObject
{
	hkpShape *shape; // 10
	UInt64 unk18; // == 0?
	UInt32 materialId; // 20
	UInt32 pad28;
};
static_assert(sizeof(bhkShape) == 0x28);

struct bhkCollisionFilter : hkpCollisionFilter
{
	UInt32 unk48;
	UInt32 unkCounter; // 4C - this gets incremented when something gets added to the world - is it used for unique collision groups?
	UInt32 bipedBitfields[32]; // 50 - About 18 of them seem to be actually filled in
	UInt32 layerCollisionGroups[64]; // D0 - if zero, use the counter from the collision filter as the collision group - afaik only 3 have non-zero entries: 9 (trees), 11 (water), and 13 (terrain)
	UInt64 layerBitfields[64]; // 1D0 - only 56 are valid in vanilla - these are used to determine which layers collide with each other
	UInt64 todo2[2]; // 3D0
	BSFixedString layerNames[64]; // 3E0 - only 56 are non-null
};
static_assert(offsetof(bhkCollisionFilter, unkCounter) == 0x4C);
static_assert(offsetof(bhkCollisionFilter, bipedBitfields) == 0x50);
static_assert(offsetof(bhkCollisionFilter, layerCollisionGroups) == 0xD0);
static_assert(offsetof(bhkCollisionFilter, layerBitfields) == 0x1D0);
static_assert(offsetof(bhkCollisionFilter, layerNames) == 0x3E0);

struct ahkpWorld : hkpWorld
{
	struct bhkWorld * m_userData; // 430
};
static_assert(offsetof(ahkpWorld, m_userData) == 0x430);

// bhkWorld pointer (exteriors) is at reloc<0x1f850d0>
// function that gets it from a TESObjectCELL is at reloc<276A90>

// Address of pointer that points to the bhkWorld pointer
// RelocAddr<bhkWorld ***> BHKWORLD(0x1f850d0); - world for _tamriel outside_ is here - does not work for interiors

struct bhkWorld : NiRefObject
{
	ahkpWorld * world; // 10
	UInt8 unk18[0xC598 - 0x18];
	BSReadWriteLock worldLock; // C598
	// C570 is bhkConstraintProjector
	// C5C0 is TESTrapListener
	// C5C8 is BGSAcousticSpaceListener
	// C5D0 is hkpSuspendInactiveAgentsUtil
	// C5D8 is some sort of counter
};
static_assert(offsetof(bhkWorld, world) == 0x10);
static_assert(offsetof(bhkWorld, worldLock) == 0xC598);

struct bhkRefObject : NiObject
{
	virtual void Unk_25(void); // 25
	virtual void AddOrRemoveReference(void); // 26
};

struct bhkSerializable : bhkRefObject
{
	virtual hkpWorld* GetHavokWorld_1(); // 27
	virtual hkpWorld* GetHavokWorld_2(void); // 28
	virtual void	  MoveToWorld(void); // 29
	virtual void	  RemoveFromCurrentWorld(void); // 2A
	virtual void	  Unk_2B(void); // 2B
	virtual void	  Unk_2C(void); // 2C
	virtual void	  Unk_2D(void); // 2D
	virtual void	  Unk_2E(void) = 0; // 2E
	virtual void	  GetSerializable(void) = 0; // 2F
	virtual void	  Unk_30(void); // 30
	virtual void	  Unk_31(void); // 31
};

struct bhkConstraint : bhkSerializable
{
	hkpConstraintInstance *constraint; // 10
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
	virtual void getPosition(void); // 33
	virtual void getRotation(void); // 34
	virtual void setPosition(void); // 35
	virtual void setRotation(void); // 36
	virtual void setPositionAndRotation(hkVector4 &pos, hkQuaternion &rot); // 37
	virtual void getCenterOfMassLocal(void); // 38
	virtual void getCenterOfMassInWorld(void); // 39
	virtual void getTransform(void); // 3A
	virtual void getAabb(void); // 3B
	virtual void Unk_3C(void); // 3C

	hkpRigidBody * hkBody; // 10
	UInt16 flags; // 18 - flags? if or'd with 0x20 (bit 5), it makes havok sim more stable - actually, that might apply to the flags in bhkCollisionObject?
	UInt64 unk20; // at least first byte are some flags? bit 2 is set -> has constraints?
	tArray<bhkConstraint *> constraints; // 28
};
static_assert(offsetof(bhkRigidBody, constraints) == 0x28);
static_assert(sizeof(bhkRigidBody) == 0x40);

struct bhkRigidBodyT : bhkRigidBody
{
	// I'm really not sure about these. The first one likes to be { 0, 0, 0, 1 } which seems to be a quat.
	hkQuaternion unkRot;
	hkVector4 unkPos;
};
static_assert(offsetof(bhkRigidBodyT, unkRot) == 0x40);
static_assert(offsetof(bhkRigidBodyT, unkPos) == 0x50);

struct NiCollisionObject : NiObject
{
	virtual void Unk_25(void); // 25
	virtual void UpdateWithContext(void); // 26
	virtual void Unk_27(void); // 27
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

	UInt64 flags; // 18 - bit 3 is set => we should update rotation of NiNode? // kDebugDisplay = 1 << 4, TODO: TRY THIS???
	NiPointer<bhkWorldObject> body; // 20
};
static_assert(offsetof(bhkNiCollisionObject, body) == 0x20);

struct bhkCollisionObject : bhkNiCollisionObject
{
};


struct bhkSimpleShapePhantom : NiRefObject
{
	hkpSimpleShapePhantom * phantom; // 10
};
