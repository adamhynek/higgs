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
	UInt64 unk48;
	UInt32 bipedBitfields[18]; // 50 - could be more than 18, I'm not exactly sure. The max is 32 (5 bits)
	UInt64 unk[39]; // 98
	UInt64 layerBitfields[64]; // 1D0 - only 56 are valid in vanilla
	UInt64 todo2[2]; // 3D0
	BSFixedString layerNames[64]; // 3E0 - only 56 are non-null
};
static_assert(offsetof(bhkCollisionFilter, bipedBitfields) == 0x50);
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
};
static_assert(offsetof(bhkWorld, world) == 0x10);
static_assert(offsetof(bhkWorld, worldLock) == 0xC598);

struct bhkConstraint : NiRefObject
{
	hkpConstraintInstance *constraint; // 10
};

struct bhkWorldObject : NiObject
{

};

struct bhkEntity : bhkWorldObject
{

};

struct bhkRigidBody : bhkEntity
{
	hkpRigidBody * hkBody; // 10
	UInt16 flags; // 18 - flags? if or'd with 0x20 (bit 5), it makes havok sim more stable
	UInt64 unk20; // at least first byte are some flags?
	bhkConstraint **constraints; // 28 - ptr to array of constraint ptrs. Size is at 0x38
	UInt64 unk30;
	UInt32 numConstraints; // 38
	// pretty sure size is 40
};
static_assert(offsetof(bhkRigidBody, numConstraints) == 0x38);

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
	NiAVObject * node; // 10 - points back to the NiAVObject pointing to this
};

struct bhkCollisionObject : NiCollisionObject
{
	UInt64 unk18; // bit 3 is set => we should update rotation of NiNode?
	NiPointer<bhkWorldObject> body; // 20
};
static_assert(offsetof(bhkCollisionObject, body) == 0x20);

struct bhkSimpleShapePhantom : NiRefObject
{
	hkpSimpleShapePhantom * phantom; // 10
};

/*
struct hkbStateMachine : hkReferencedObject
{
	// These from hkbBindable
	void * m_variableBindingSet; // 10
	hkArray m_cachedBindables; // 18
	hkBool m_areBindablesCached;
	hkBool m_hasEnableChanged;

	// These from hkbNode
	void * m_userData; // 30
	char * m_name; // 38
	UInt16 m_id; // 40
	SInt8 m_cloneState;
	UInt8 m_type;
	void * m_nodeInfo; // 48

	UInt8 unk50[0x90 - 0x50];

	hkArray m_states; // 90
	void * m_wildcardTransitions; // A0 - ptr to TransitionInfoArray
	void * m_stateIdToIndexMap; // A8 - ptr to hkPointerMap<int, int>
	hkArray m_activeTransitions; // B0
	hkArray m_transitionFlags; // C0
	hkArray m_wildcardTransitionFlags; // D0
	hkArray m_delayedTransitions; // E0
	float m_timeInState; // F0
	float m_lastLocalTime; // F4
	int m_previousStateId; // F8
	int m_nextStartStateIndexOverride; // FC

	// Not 100% on these ones
	hkBool m_stateOrTransitionChanged; // 100
	hkBool m_echoNextUpdate; // 101
	hkBool m_hasEventlessWildcardTransitions; // 102
};
static_assert(offsetof(hkbStateMachine, m_states) == 0x90);
static_assert(offsetof(hkbStateMachine, m_nextStartStateIndexOverride) == 0xFC);

struct hkbBehaviorGraph : hkReferencedObject
{
	// These from hkbBindable
	void * m_variableBindingSet; // 10
	hkArray m_cachedBindables; // 18
	hkBool m_areBindablesCached;
	hkBool m_hasEnableChanged;

	// These from hkbNode
	void * m_userData; // 30
	char * m_name; // 38
	UInt16 m_id; // 40
	SInt8 m_cloneState;
	UInt8 m_type;
	void * m_nodeInfo; // 48

	hkArray m_uniqueIdPool; // 50
	UInt64 pad60;
	hkArray m_mirroredExternalIdMap; // 68
	void * m_pseudoRandomGenerator; // 78
	void * m_rootGenerator; // 80 - hkbStateMachine, at least for the player
	void * m_data; // 88

	UInt64 pad90;

	// Dunno about these 3
	hkbBehaviorGraph * m_template; // 98
	void * m_rootGeneratorClone; // A0
	hkArray * m_activeNodes; // A8 - ptr to array of hkbNodeInfo*

	void * m_globalTransitionData; // B0
	void * m_eventIdMap; // B8
	void * m_attributeIdMap; // C0
	void * m_variableIdMap; // C8
	void * m_characterPropertyIdMap; // D0
	void * m_variableValueSet; // D8

	UInt64 unkE0;
	UInt64 unkE8;
	UInt64 unkF0;
	UInt64 unkF8;
	UInt64 unk100;
	hkArray unk108;
	hkArray unk118;
	UInt32 unk120;
	hkBool m_isActive; // 12C - I _think_ this is the right place
};
static_assert(offsetof(hkbBehaviorGraph, m_userData) == 0x30);
static_assert(offsetof(hkbBehaviorGraph, m_rootGenerator) == 0x80);
static_assert(offsetof(hkbBehaviorGraph, m_globalTransitionData) == 0xB0);
static_assert(offsetof(hkbBehaviorGraph, m_isActive) == 0x12C);

*/