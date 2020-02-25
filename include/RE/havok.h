#pragma once

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"


typedef char hkBool;

// All shape types. The dispatcher has only to implement at least the types that can be used as secondary types
enum hkpShapeType
{
	//
	// Shape types from HK_SHAPE_INVALID to HK_SHAPE_MAX_ID_SPU are supported on the SPU.
	//

	HK_SHAPE_INVALID = 0,

	// 
	//	The abstract base shapes
	//

	//
	// Special convex shapes, which get their private agents for better performance on
	// PPU only.  All use predictive GSK agent on SPU.
	//

	// hkpSphereShape type.
	HK_SHAPE_SPHERE,
	// The first real shape.
	HK_FIRST_SHAPE_TYPE = HK_SHAPE_SPHERE,
	// hkpCylinderShape type.
	HK_SHAPE_CYLINDER,
	// hkpTriangleShape type.
	HK_SHAPE_TRIANGLE,
	// hkpBoxShape type.
	HK_SHAPE_BOX,
	// hkpCapsuleShape type.
	HK_SHAPE_CAPSULE,
	// hkpConvexVerticesShape type.
	HK_SHAPE_CONVEX_VERTICES,

	//
	// Special shape collections that are solved on the SPU.
	//

	// All shapes which inherit from hkpShapeCollection have this as an alternate type.
	HK_SHAPE_COLLECTION,
	// All shapes which inherit from hkpBvTreeShape have this as an alternate type.
	HK_SHAPE_BV_TREE,
	// hkpListShape type.
	HK_SHAPE_LIST,
	// hkpMoppBvTreeShape type.
	HK_SHAPE_MOPP,
	// hkpConvexTranslateShape type.
	HK_SHAPE_CONVEX_TRANSLATE,
	// hkpConvexTransformShape type.
	HK_SHAPE_CONVEX_TRANSFORM,
	// hkpSampledHeightFieldShape type.
	HK_SHAPE_SAMPLED_HEIGHT_FIELD,
	// hkpExtendedMeshShape type.
	HK_SHAPE_EXTENDED_MESH,
	// hkpTransformShape type.
	HK_SHAPE_TRANSFORM,

	//
	// Shape types from HK_SHAPE_MAX_ID_SPU to HK_SHAPE_MAX_ID are NOT supported on the SPU.
	//

	// Last SPU support shape type
	HK_SHAPE_MAX_ID_SPU = HK_SHAPE_TRANSFORM + 1,
	// All shapes which inherit from hkpConvexShape have this as an alternate type.
	HK_SHAPE_CONVEX,
	// DEPRECATED - hkpPackedConvexVerticesShape type. 
	HK_SHAPE_PACKED_CONVEX_VERTICES,
	// DEPRECATED - hkpMoppEmbeddedShape type.
	HK_SHAPE_MOPP_EMBEDDED,
	// DEPRECATED - hkpConvexPieceShape type.
	HK_SHAPE_CONVEX_PIECE,

	//
	//	hkpShapeCollection implementations
	//

	// hkpMultiSphereShape type.
	HK_SHAPE_MULTI_SPHERE,
	// hkpConvexListShape, a List of convex pieces which are treated as a single convex object if possible.
	HK_SHAPE_CONVEX_LIST,
	// A shape collection which only returns triangles as child shapes, e.g. hkpMeshShape.
	HK_SHAPE_TRIANGLE_COLLECTION,

	// 
	// Special shapes
	// 

	// hkpMultiRayShape type.
	HK_SHAPE_MULTI_RAY,
	// hkpHeightFieldShape type.
	HK_SHAPE_HEIGHT_FIELD,
	// hkpSphereRepShape type.
	HK_SHAPE_SPHERE_REP,
	// hkpBvShape type.
	HK_SHAPE_BV,
	// hkpPlaneShape type.
	HK_SHAPE_PLANE,

	//
	//	Single shapes which are processed by unary agents.
	//

	// hkpPhantomCallbackShape type.
	HK_SHAPE_PHANTOM_CALLBACK,


	//
	//	user shapes
	//

	HK_SHAPE_USER0,
	HK_SHAPE_USER1,
	HK_SHAPE_USER2,

	//	The end of the shape type list.
	HK_SHAPE_MAX_ID,
	// All shape flag, used by the hkpCollisionDispatcher.
	HK_SHAPE_ALL = -1
};

// Must be aligned to 16 bytes (128 bits) as it's a simd type
__declspec(align(16)) struct hkVector4
{
	float x;
	float y;
	float z;
	float w;
};
static_assert(sizeof(hkVector4) == 0x10);

struct hkArray
{
	void * m_data;
	int m_size;
	int m_capacityAndFlags;
};
static_assert(sizeof(hkArray) == 0x10);

struct hkTransform
{
	float m_rotation[12]; // 00 - 3x4 matrix, 3 rows of hkVector4
	hkVector4 m_translation; // 30
};
static_assert(sizeof(hkTransform) == 0x40);

struct hkSweptTransform
{
	hkVector4 m_centerOfMass0; // 00
	hkVector4 m_centerOfMass1; // 10
	hkVector4 m_rotation0; // 20 - Quaternion
	hkVector4 m_rotation1; // 30 - Quaternion
	hkVector4 m_centerOfMassLocal; // 40 - Often all 0's
};
static_assert(sizeof(hkSweptTransform) == 0x50);

struct bhkBoxShape
{
	void * vtbl; // 00
	// These 2 inherited from NiRefObject, I _think_
	volatile SInt32	m_uiRefCount;	// 08
	UInt32	pad0C;	// 0C

	struct hkpBoxShape * hkBoxShape; // 10 - points to hkpBoxShape

	UInt64 unk18; // == 0?
	UInt64 unk20;
};
static_assert(sizeof(bhkBoxShape) == 0x28);

struct hkpShape
{
	void * vtbl; // 00
	// These 3 inherited from hkReferencedObject
	UInt16 m_memSizeAndFlags; // 08
	SInt16 m_referenceCount; // 0A
	UInt32 pad0C; // 0C

	void * m_userData; // 10
	UInt32 m_type; // 18
	UInt32 pad1C; // 1C
};
static_assert(sizeof(hkpShape) == 0x20);

struct hkpBoxShape : public hkpShape
{
	// m_type == 4, m_userData points to bhkBoxShape
	float m_radius; // 20
	UInt32 pad24;
	UInt64 unk28;
	hkVector4 m_halfExtents; // 30 - I believe the value of w doesn't matter here
};
static_assert(sizeof(hkpBoxShape) == 0x40);

struct hkpConvexShape : public hkpShape
{
	float m_radius; // 20
};

struct hkpShapeRayCastCollectorOutput
{
	hkVector4 m_normal;
	float m_hitFraction;
	int m_extraInfo;
	int m_pad[2];
};
static_assert(sizeof(hkpShapeRayCastCollectorOutput) == 0x20);

struct hkpTypedBroadPhaseHandle
{
	// Inherited from hkpBroadPhaseHandle
	UInt32 m_id; // 00

	SInt8 m_type; // 04
	SInt8 m_ownerOffset; // 05
	SInt8 m_objectQualityType; // 06
	UInt8 pad07;
	UInt32 m_collisionFilterInfo; // 08
};
static_assert(sizeof(hkpTypedBroadPhaseHandle) == 0x0C);

struct hkpCdBody
{
	hkpShape * m_shape; // 00
	UInt32 m_shapeKey; // 08
	UInt32 pad0C;
	void * m_motion; // 10
	hkpCdBody * m_parent; // 18
};
static_assert(sizeof(hkpCdBody) == 0x20);

struct hkpCollidable : public hkpCdBody
{
	struct BoundingVolumeData
	{
		UInt32 m_min[3]; // 00
		UInt8 m_expansionMin[3]; // 0C
		UInt8 m_expansionShift; // 0F
		UInt32 m_max[3]; // 10
		UInt8 m_expansionMax[3]; // 1C
		UInt8 m_padding; // 1F
		UInt16 m_numChildShapeAabbs; // 20
		UInt16 m_capacityChildShapeAabbs; // 22
		UInt32 pad24;
		void * m_childShapeAabbs; // 28 - it's a hkAabbUint32 *
		UInt32 * m_childShapeKeys; // 30
	};
	static_assert(sizeof(BoundingVolumeData) == 0x38);

	SInt8 m_ownerOffset; // 20
	SInt8 m_forceCollideOntoPpu; // 21
	SInt16 m_shapeSizeOnSpu; // 22
	hkpTypedBroadPhaseHandle m_broadPhaseHandle; // 24
	BoundingVolumeData m_boundingVolumeData; // 30
	float m_allowedPenetrationDepth; // 68
	UInt32 pad6C;
};
static_assert(sizeof(hkpCollidable) == 0x70);

struct hkpLinkedCollidable : public hkpCollidable
{
	hkArray m_collisionEntries; // 70
};
static_assert(sizeof(hkpLinkedCollidable) == 0x80);

class hkpRayHitCollector
{
public:
	virtual void addRayHit(const hkpCdBody * cdBody, const hkpShapeRayCastCollectorOutput * hitInfo) = 0;
	virtual inline ~hkpRayHitCollector() { }

public:
	//void *vtbl; // 00
	float m_earlyOutHitFraction; // 08
};

struct hkpWorldRayCastInput
{
	hkVector4 m_from; // 00
	hkVector4 m_to; // 10
	hkBool m_enableShapeCollectionFilter; // 20
	UInt32 m_filterInfo; // 24

	inline hkpWorldRayCastInput() : m_enableShapeCollectionFilter(false), m_filterInfo(0) {}
};

struct hkpWorldRayCaster
{
	void * vtbl; // 00
	hkpWorldRayCastInput * m_input; // 08
	void * m_filter; // 10
	void * m_collectorBase; // 18
	int m_collectorStriding; //20
	UInt32 pad24;
	UInt64 extra[7]; // 28
};
static_assert(sizeof(hkpWorldRayCaster) == 0x60);

struct hkContactPoint
{
	inline float getDistance() const { return m_separatingNormal.w; };

	hkVector4 m_position; // 00
	hkVector4 m_separatingNormal; // 10
	hkVector4 m_separatingNormal_again; // 20 - It appears to be here twice for some reason - maybe sometimes it's different?
};
static_assert(sizeof(hkContactPoint) == 0x30);

struct hkpCdPoint
{
	hkContactPoint m_contact; // 00
	hkpCdBody * m_cdBodyA; // 30
	hkpCdBody * m_cdBodyB; // 40
};

struct hkpCdPointCollector
{
	virtual inline ~hkpCdPointCollector() {}
	virtual void addCdPoint(const hkpCdPoint& point) = 0;
	virtual inline void reset() { m_earlyOutDistance = 1.0f; }

	//void * vtbl; // 00
	float m_earlyOutDistance; // 08
};

struct hkpLinearCastInput
{
	hkVector4 m_to; // 00
	float m_maxExtraPenetration; // 10
	float m_startPointTolerance; // 14

	inline hkpLinearCastInput() : m_maxExtraPenetration(0.01f), m_startPointTolerance(0.01f) {}
};

struct hkpCdBodyPairCollector
{
	virtual inline ~hkpCdBodyPairCollector() {}
	virtual void addCdBodyPair(const hkpCdBody& bodyA, const hkpCdBody& bodyB) = 0;
	virtual inline void reset() { m_earlyOut = false; }

	//void * vtbl; // 00
	hkBool m_earlyOut; // 08
};

// Function that compares 2 filter infos is at reloc<0xE2BA10>

struct hkpCollisionFilter
{
	void * vtbl; // 00
	// These 3 inherited from hkReferencedObject
	UInt16 m_memSizeAndFlags; // 08
	SInt16 m_referenceCount; // 0A
	UInt32 pad0C; // 0C

	// more...
};

struct hkpCollisionInput
{

};

struct hkpProcessCollisionInput : public hkpCollisionInput
{
	// Pointer to this struct can be gotten from hkpWorld
};

struct hkpSimulation
{

};

struct hkpSimulationIsland
{

};

// bhkWorld pointer is at reloc<0x1f850d0>
// function that gets it (using a TESObjectCELL at rcx) is at reloc<276A90>

struct ahkpWorld
{
	void * vtbl; // 00
	// These 3 inherited from hkReferencedObject
	UInt16 m_memSizeAndFlags; // 08
	SInt16 m_referenceCount; // 0A
	UInt32 pad0C; // 0C

	hkpSimulation * m_simulation; // 10 - hkpContinuousSimulation
	UInt64 unk18; // pointer to string? (??)
	hkVector4 m_gravity; // 20
	hkpSimulationIsland * m_fixedIsland; // 30
	struct hkpRigidBody * m_fixedRigidBody; // 38
	hkArray m_activeSimulationIslands; // 40
	hkArray m_inactiveSimulationIslands; // 50
	hkArray m_dirtySimulationIslands; // 60
	void * m_maintenanceMgr; // 70 - hkRefPtr?
	void * m_memoryWatchDog; // 78
	bool assertOnRunningOutOfSolverMemory; // 80
	UInt8 pad081;
	UInt16 pad082;
	UInt32 pad084;
	void * broadPhase; // 88
	void * kdTreeManager; // 90
	bool autoUpdateTree; // 98
	UInt8 pad099;
	UInt16 pad09A;
	UInt32 pad09C;
	void * m_broadPhaseDispatcher; // A0
	void * m_phantomBroadPhaseListener; // A8
	void * m_entityEntityBroadPhaseListener; // B0
	void * m_broadPhaseBorderListener; // B8
	void * m_multithreadedSimulationJobData; // C0
	hkpProcessCollisionInput * m_collisionInput; // C8
	hkpCollisionFilter * m_collisionFilter;  // D0
	void * m_collisionDispatcher; // D8
	void * m_convexListFilter; // E0

	// way more... todo
};
static_assert(offsetof(ahkpWorld, m_broadPhaseDispatcher) == 0xA0);

struct bhkWorld
{
	void * vtbl; // 00
	// These 2 inherited from NiRefObject, I _think_
	volatile SInt32	m_uiRefCount;	// 08
	UInt32	pad0C;	// 0C

	ahkpWorld * world; // 10
};
static_assert(offsetof(bhkWorld, world) == 0x10);

struct hkMotionState
{
	hkTransform m_transform; // 00
	hkSweptTransform m_sweptTransform; // 40

	hkVector4 m_deltaAngle; // 90
	float m_objectRadius; // A0
	float m_linearDamping; // A4
	float m_angularDamping; // A8
	// These next 2 are hkUFloat8, 8-bit floats
	UInt8 m_maxLinearVelocity; // AC
	UInt8 m_maxAngularVelocity; // AD
	UInt8 m_deactivationClass; // AE
	UInt8 padAF;
};
static_assert(sizeof(hkMotionState) == 0xB0);

struct hkpMotion
{
	void * vtbl; // 00
	// These 3 inherited from hkReferencedObject
	UInt16 m_memSizeAndFlags; // 08
	SInt16 m_referenceCount; // 0A
	UInt32 pad0C; // 0C

	UInt8 m_motionType; // 10
	UInt8 m_deactivationIntegrateCounter; // 11
	UInt16 m_deactivationNumInactiveFrames[2]; // 12
	UInt16 pad16;
	UInt64 unk18; // Looks like padding

	hkMotionState m_motionState; // 20

	hkVector4 m_inertiaAndMassInv; // D0
	hkVector4 m_linearVelocity; // E0
	hkVector4 m_angularVelocity; // F0
	hkVector4 m_deactivationRefPosition[2]; // 100
	UInt32 m_deactivationRefOrientation[2]; // 120
	void * m_savedMotion; // 128 - hkpMaxSizeMotion * - can be null
	UInt16 m_savedQualityTypeIndex; // 130
	UInt16 m_gravityFactor; // 132
	UInt32 pad134;
	UInt64 pad138;
};
static_assert(sizeof(hkpMotion) == 0x140);

struct hkpRigidBody
{
	void * vtbl; // 00
	// These 3 inherited from hkReferencedObject
	UInt16 m_memSizeAndFlags; // 08
	SInt16 m_referenceCount; // 0A
	UInt32 pad0C; // 0C

	ahkpWorld * world; // 10

	struct bhkRigidBodyT * gameRigidBody; // 18 - user data - points back to below struct

	hkpLinkedCollidable m_collidable; // 20
	UInt64 todo[18]; // A0
	hkpSimulationIsland * island; // 130 - I hope it's nice there
	UInt64 unk138;
	UInt64 unk140;
	UInt64 unk148;

	hkpMotion motion; // 150
	// more...
};
static_assert(offsetof(hkpRigidBody, motion) == 0x150);

struct bhkRigidBodyT
{
	void * vtbl; // 00
	// These 2 inherited from NiRefObject
	volatile SInt32	m_uiRefCount;	// 08
	UInt32	pad0C;	// 0C

	hkpRigidBody * hkBody; // 10 - points to above struct
	// more?
};

struct bhkCollisionObject
{
	void * vtbl; // 00
	// These 2 inherited from NiRefObject
	volatile SInt32	m_uiRefCount;	// 08
	UInt32	pad0C;	// 0C

	NiNode * node; // 10 - points back to the NiNode pointing to this
	UInt64 unk18; // == 0x81?
	bhkRigidBodyT * body; // 20
	// more?
};
static_assert(offsetof(bhkCollisionObject, body) == 0x20);

struct hkpShapePhantom
{
	// From hkReferencedObject
	virtual ~hkpShapePhantom();
	virtual void getClassType();
	virtual void calcContentStatistics();
	// From hkpWorldObject
	virtual void setShape();
	virtual void getMotionState();
	// From hkpPhantom
	virtual void getType();
	virtual void calcAabb();
	virtual void addOverlappingCollidable();
	virtual void isOverlappingCollidableAdded();
	virtual void removeOverlappingCollidable();
	virtual void ensureDeterministicOrder();
	virtual void clone();
	virtual void updateShapeCollectionFilter();
	virtual void deallocateInternalArrays();
	// From hkpShapePhantom
	virtual void setPositionAndLinearCast(const hkVector4& position, const hkpLinearCastInput& input, hkpCdPointCollector& castCollector, hkpCdPointCollector* startCollector);
	virtual void setTransformAndLinearCast(const hkTransform& transform, const hkpLinearCastInput& input, hkpCdPointCollector& castCollector, hkpCdPointCollector* startCollector);
	virtual void getClosestPoints();
	virtual void getPenetrations();
	// Missing a vfunc... must have added one in a later havok version? - TODO
};

struct hkpSimpleShapePhantom : public hkpShapePhantom
{
	//void * vtbl; // 00
	// These 3 inherited from hkReferencedObject
	UInt16 m_memSizeAndFlags; // 08
	SInt16 m_referenceCount; // 0A
	UInt32 pad0C; // 0C

	ahkpWorld * world; // 10

	void * userData; // 18

	hkpLinkedCollidable m_collidable; // 20

	UInt64 todo[10];

	hkMotionState m_motionState; // F0

	// more...
};
static_assert(offsetof(hkpSimpleShapePhantom, m_motionState) == 0xF0);

struct bhkSimpleShapePhantom
{
	void * vtbl; // 00
	volatile SInt32	m_uiRefCount;	// 08
	UInt32	pad0C;	// 0C

	hkpSimpleShapePhantom * phantom; // 10
};
