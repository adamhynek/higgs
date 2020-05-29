#pragma once

#include "skse64_common/Relocation.h"
#include "skse64/PapyrusVM.h"
#include "skse64/GameReferences.h"


typedef char hkBool;
typedef SInt16 hkHalf;

// All shape types. The dispatcher has only to implement at least the types that can be used as secondary types
enum hkpShapeType : UInt32
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
	HK_SHAPE_ALL = (UInt32)-1
};

enum MotionType : UInt8
{
	MOTION_INVALID,

	// A fully-simulated, movable rigid body. At construction time the engine checks
	// the input inertia and selects MOTION_SPHERE_INERTIA or MOTION_BOX_INERTIA as
	// appropriate.
	MOTION_DYNAMIC,

	// Simulation is performed using a sphere inertia tensor. (A multiple of the
	// Identity matrix). The highest value of the diagonal of the rigid body's
	// inertia tensor is used as the spherical inertia.
	MOTION_SPHERE_INERTIA,

	// Simulation is performed using a box inertia tensor. The non-diagonal elements
	// of the inertia tensor are set to zero. This is slower than the
	// MOTION_SPHERE_INERTIA motions, however it can produce more accurate results,
	// especially for long thin objects.
	MOTION_BOX_INERTIA,

	// Simulation is not performed as a normal rigid body. During a simulation step,
	// the velocity of the rigid body is used to calculate the new position of the
	// rigid body, however the velocity is NOT updated. The user can keyframe a rigid
	// body by setting the velocity of the rigid body to produce the desired keyframe
	// positions. The hkpKeyFrameUtility class can be used to simply apply keyframes
	// in this way. The velocity of a keyframed rigid body is NOT changed by the
	// application of impulses or forces. The keyframed rigid body has an infinite
	// mass when viewed by the rest of the system.
	MOTION_KEYFRAMED,

	// This motion type is used for the static elements of a game scene, e.g., the
	// landscape. Fixed rigid bodies are treated in a special way by the system. They
	// have the same effect as a rigid body with a motion of type MOTION_KEYFRAMED
	// and velocity 0, however they are much faster to use, incurring no simulation
	// overhead, except in collision with moving bodies.
	MOTION_FIXED,

	// A box inertia motion which is optimized for thin boxes and has less stability problems
	MOTION_THIN_BOX_INERTIA,

	// A specialized motion used for character controllers
	MOTION_CHARACTER,

	MOTION_MAX_ID
};

enum hkpCollidableQualityType : SInt8
{
	// Invalid or unassigned type. If you add a hkpRigidBody to the hkpWorld,
	// this type automatically gets converted to either
	// HK_COLLIDABLE_QUALITY_FIXED, HK_COLLIDABLE_QUALITY_KEYFRAMED or HK_COLLIDABLE_QUALITY_DEBRIS
	HK_COLLIDABLE_QUALITY_INVALID = -1,

	// Use this for fixed bodies.
	HK_COLLIDABLE_QUALITY_FIXED = 0,

	// Use this for moving objects with infinite mass.
	HK_COLLIDABLE_QUALITY_KEYFRAMED,

	// Use this for all your debris objects.
	HK_COLLIDABLE_QUALITY_DEBRIS,

	// Use this for debris objects that should have simplified TOI collisions with fixed/landscape objects.
	HK_COLLIDABLE_QUALITY_DEBRIS_SIMPLE_TOI,

	// Use this for moving bodies, which should not leave the world,
	// but you rather prefer those objects to tunnel through the world than
	// dropping frames because the engine .
	HK_COLLIDABLE_QUALITY_MOVING,

	// Use this for all objects, which you cannot afford to tunnel through
	// the world at all.
	HK_COLLIDABLE_QUALITY_CRITICAL,

	// Use this for very fast objects.
	HK_COLLIDABLE_QUALITY_BULLET,

	// For user. If you want to use this, you have to modify hkpCollisionDispatcher::initCollisionQualityInfo()
	HK_COLLIDABLE_QUALITY_USER,

	// Use this for rigid body character controllers.
	HK_COLLIDABLE_QUALITY_CHARACTER,

	// Use this for moving objects with infinite mass which should report contact points and TOI-collisions against all other bodies, including other fixed and keyframed bodies.
	//
	// Note that only non-TOI contact points are reported in collisions against debris-quality objects.
	HK_COLLIDABLE_QUALITY_KEYFRAMED_REPORTING,

	// End of this list
	HK_COLLIDABLE_QUALITY_MAX
};

enum hkpUpdateCollisionFilterOnEntityMode
{
	// Do a full check
	HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK,

	// Recheck filter but only check for disabled entity-entity collisions which have been enabled before
	HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY
};

enum hkpUpdateCollectionFilterMode
{
	// Assume that no single shapes in a shape collections changed their filter status
	HK_UPDATE_COLLECTION_FILTER_IGNORE_SHAPE_COLLECTIONS,

	// Recheck all subshapes in a shape collection
	HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS
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

struct hkMatrix3
{
	// w of each column is just padding
	hkVector4 m_col0;
	hkVector4 m_col1;
	hkVector4 m_col2;
};

struct hkRotation : hkMatrix3
{

};

struct hkTransform
{
	hkRotation m_rotation; // 00
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

struct hkReferencedObject
{
	virtual ~hkReferencedObject() {};
	virtual void getClassType() {};
	virtual void calcContentStatistics() {};

	UInt16 m_memSizeAndFlags; // 08
	SInt16 m_referenceCount; // 0A
	UInt32 pad0C;
};

struct bhkBoxShape : NiRefObject
{
	struct hkpBoxShape * hkBoxShape; // 10 - points to hkpBoxShape

	UInt64 unk18; // == 0?
	UInt64 unk20;
};
static_assert(sizeof(bhkBoxShape) == 0x28);

struct hkpShape : hkReferencedObject
{
	void * m_userData; // 10
	hkpShapeType m_type; // 18
	UInt32 pad1C; // 1C
};
static_assert(sizeof(hkpShape) == 0x20);

struct hkpConvexShape : public hkpShape
{
	float m_radius; // 20
};

struct hkpBoxShape : public hkpConvexShape
{
	hkVector4 m_halfExtents; // 30 - I believe the value of w doesn't matter here
};
static_assert(sizeof(hkpBoxShape) == 0x40);

struct hkpTypedBroadPhaseHandle
{
	// Inherited from hkpBroadPhaseHandle
	UInt32 m_id; // 00

	SInt8 m_type; // 04
	SInt8 m_ownerOffset; // 05
	hkpCollidableQualityType m_objectQualityType; // 06
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
	inline void* getOwner() const
	{
		return const_cast<char*>(reinterpret_cast<const char*>(this) + m_ownerOffset);
	}

	inline int getType() const
	{
		return m_broadPhaseHandle.m_type;
	}

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

struct hkpShapeRayCastCollectorOutput
{
	inline void reset() {
		m_hitFraction = 1.0f;
		m_shapeKey = -1;
		m_extraInfo = -1;
	};

	hkVector4 m_normal;
	float m_hitFraction;
	int m_extraInfo;
	UInt32 m_shapeKey;
	int m_pad;
};

struct hkpShapeRayCastOutput : public hkpShapeRayCastCollectorOutput
{
	UInt32 m_shapeKeys[8];
	int m_shapeKeyIndex = -1;
};

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

	inline hkpWorldRayCastInput(UInt32 filterInfo = 0, hkBool enableShapeCollectionFilter = false)
		: m_enableShapeCollectionFilter(enableShapeCollectionFilter), m_filterInfo(filterInfo)
	{}
};

struct hkpShapeRayCastInput
{
	hkVector4 m_from;
	hkVector4 m_to;
	UInt32 m_filterInfo = 0;
	void * m_rayShapeCollectionFilter = nullptr;
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
};
static_assert(sizeof(hkContactPoint) == 0x20);

struct hkpCdPoint
{
	hkContactPoint m_contact; // 00
	hkVector4 m_unweldedNormal; // 20
	hkpCdBody * m_cdBodyA; // 30
	hkpCdBody * m_cdBodyB; // 38
};
static_assert(sizeof(hkpCdPoint) == 0x40);

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

struct hkpRayShapeCollectionFilter : hkReferencedObject
{

};

// Function that compares 2 filter infos is at reloc<0xE2BA10>

struct hkpCollisionFilter : hkpRayShapeCollectionFilter
{
	// more...
};

struct bhkCollisionFilter : hkpCollisionFilter
{
	char todo[0x40]; // 10
	UInt32 bipedBitfields[18]; // 50 - could be more than 18, I'm not exactly sure. The max is 32 (5 bits)
	UInt64 unk[39]; // 98
	UInt64 layerBitfields[64]; // 1D0 - only 56 are valid in vanilla
	UInt64 todo2[2]; // 3D0
	char * layerNames[64]; // 3E0 - only 56 are non-null
};
static_assert(offsetof(bhkCollisionFilter, bipedBitfields) == 0x50);
static_assert(offsetof(bhkCollisionFilter, layerBitfields) == 0x1D0);
static_assert(offsetof(bhkCollisionFilter, layerNames) == 0x3E0);

struct hkpCollisionInput
{
	struct hkpCollisionDispatcher * m_dispatcher; // 00
	UInt32 m_weldClosestPoints; // 08
	UInt32 m_forceAcceptContactPoints; // 0C
	float m_tolerance; // 10
	UInt32 pad14;
	hkpCollisionFilter * m_filter; // 18
	struct hkpConvexListFilter * m_convexListFilter; // 20
	UInt32 m_createPredictiveAgents; // 28
	UInt32 pad2C;
	// Aabb32Info
	hkVector4 m_bitOffsetLow; // 30
	hkVector4 m_bitOffsetHigh; // 40
	hkVector4 m_bitScale; // 50
};
static_assert(sizeof(hkpCollisionInput) == 0x60);

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

struct hkpCollisionDispatcher : hkReferencedObject
{
	enum {
		HK_MAX_RESPONSE_TYPE = 8,
		HK_MAX_SHAPE_TYPE = 32,
		HK_MAX_AGENT2_TYPES = 64,
		HK_MAX_AGENT3_TYPES = 18,
		HK_MAX_COLLISION_QUALITIES = 8
	};

	typedef void (*GetClosestPointsFunc)   (const hkpCdBody *bodyA, const hkpCdBody *bodyB, const hkpCollisionInput *input, hkpCdPointCollector *output);

	struct AgentFuncs
	{
		void * m_createFunc;
		void * m_getPenetrationsFunc;
		GetClosestPointsFunc m_getClosestPointFunc;
		void * m_linearCastFunc;
		hkBool m_isFlipped;
		hkBool m_isPredictive;
	};

	inline GetClosestPointsFunc getGetClosestPointsFunc(hkpShapeType typeA, hkpShapeType typeB) const
	{
		int idx = m_agent2Types[typeA][typeB];
		return m_agent2Func[idx].m_getClosestPointFunc;
	}

	void * m_defaultCollisionAgent;

	void * m_contactMgrFactory[HK_MAX_RESPONSE_TYPE][HK_MAX_RESPONSE_TYPE];

	__declspec(align(16)) UInt32 m_hasAlternateType[HK_MAX_SHAPE_TYPE];

	int	m_numAgent2Types;
	__declspec(align(16)) UInt8 m_agent2Types[HK_MAX_SHAPE_TYPE][HK_MAX_SHAPE_TYPE];
	UInt8 m_agent2TypesPred[HK_MAX_SHAPE_TYPE][HK_MAX_SHAPE_TYPE];
	AgentFuncs m_agent2Func[HK_MAX_AGENT2_TYPES];

	// more...
};
static_assert(offsetof(hkpCollisionDispatcher, m_numAgent2Types) == 0x2A0);

// bhkWorld pointer (exteriors) is at reloc<0x1f850d0>
// function that gets it from a TESObjectCELL is at reloc<276A90>

struct ahkpWorld : hkReferencedObject
{
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
	bhkCollisionFilter * m_collisionFilter;  // D0
	hkpCollisionDispatcher * m_collisionDispatcher; // D8
	void * m_convexListFilter; // E0

	// way more... todo
};
static_assert(offsetof(ahkpWorld, m_broadPhaseDispatcher) == 0xA0);

// Address of pointer that points to the bhkWorld pointer
// RelocAddr<bhkWorld ***> BHKWORLD(0x1f850d0); - world for _tamriel outside_ is here - does not work for interiors

struct bhkWorld : NiRefObject
{
	ahkpWorld * world; // 10
};
static_assert(offsetof(bhkWorld, world) == 0x10);

struct hkMotionState
{
	hkTransform m_transform; // 00
	hkSweptTransform m_sweptTransform; // 40

	hkVector4 m_deltaAngle; // 90
	float m_objectRadius; // A0
	hkHalf m_linearDamping; // A4
	hkHalf m_angularDamping; // A6
	hkHalf m_timeFactor; // A8
	// These next 2 are hkUFloat8, 8-bit floats
	UInt8 m_maxLinearVelocity; // AA
	UInt8 m_maxAngularVelocity; // AB
	UInt8 m_deactivationClass; // AC
	UInt8 padAD[3];
};
static_assert(sizeof(hkMotionState) == 0xB0);

struct hkpMotion : hkReferencedObject
{
	// vfunc 12 is setposition
	// 13 is setrotation
	// 14 is setpositionandrotation
	// 15 is settransform

	MotionType m_motionType; // 10
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

struct hkpRigidBodyCinfo
{
	UInt32 m_collisionFilterInfo;
	const hkpShape* m_shape;
	void * m_localFrame;
	UInt8 m_collisionResponse;
	UInt16 m_contactPointCallbackDelay;
	hkVector4 m_position;
	hkVector4 m_rotation; // Quaternion
	hkVector4 m_linearVelocity;
	hkVector4 m_angularVelocity;
	hkMatrix3 m_inertiaTensor;
	hkVector4 m_centerOfMass;
	float m_mass;
	float m_linearDamping;
	float m_angularDamping;
	float m_timeFactor;
	float m_gravityFactor;
	float m_friction;
	float m_rollingFrictionMultiplier;
	float m_restitution;
	float m_maxLinearVelocity;
	float m_maxAngularVelocity;
	float m_allowedPenetrationDepth;
	MotionType m_motionType;
	hkBool m_enableDeactivation;
	UInt8 m_solverDeactivation;
	hkpCollidableQualityType m_qualityType;
	UInt8 m_autoRemoveLevel;
	UInt8 m_responseModifierFlags;
	SInt8 m_numShapeKeysInContactPointProperties;
	hkBool m_forceCollideOntoPpu;
};

struct hkpWorldObject : hkReferencedObject
{
	enum BroadPhaseType
	{
		BROAD_PHASE_INVALID,
		BROAD_PHASE_ENTITY, // hkpEntity.
		BROAD_PHASE_PHANTOM, // hkpPhantom.
		BROAD_PHASE_BORDER, // hkpBroadPhaseBorder's objects (AABB phantoms).
		BROAD_PHASE_MAX_ID
	};
};

struct hkpEntity : hkpWorldObject
{

};

struct hkpRigidBody : hkpEntity
{
	ahkpWorld * world; // 10

	struct bhkRigidBody * gameRigidBody; // 18 - user data - points back to below struct

	hkpLinkedCollidable m_collidable; // 20
	UInt64 unkA0;
	UInt64 unkA8;
	UInt64 unkB0;
	hkArray m_properties; // B8
	UInt8 unkC8[0x130 - 0xC8]; // C8
	hkpSimulationIsland * island; // 130 - I hope it's nice there
	UInt64 unk138;
	UInt64 unk140;
	UInt64 unk148;

	hkpMotion motion; // 150
	// more...
	char unk[0x2D0 - 0x290]; // 290
};
static_assert(offsetof(hkpRigidBody, m_properties) == 0xB8);
static_assert(offsetof(hkpRigidBody, motion) == 0x150);
static_assert(sizeof(hkpRigidBody) == 0x2D0); // 2D0 is how much the game allocates for a rigidbody

struct bhkEntity : NiRefObject
{

};

struct bhkRigidBody : bhkEntity
{
	hkpRigidBody * hkBody; // 10
	UInt16 flags; // 18 - flags? if or'd with 0x20 (bit 5), it makes havok sim more stable
	UInt64 unk20; // at least first byte are some flags?
	void *constraints; // 28
	UInt64 unk30;
	UInt32 numConstraints; // 38
};
static_assert(offsetof(bhkRigidBody, numConstraints) == 0x38);

struct bhkRigidBodyT : bhkRigidBody
{

};

struct bhkCollisionObject : NiRefObject
{
	NiNode * node; // 10 - points back to the NiNode pointing to this
	UInt64 unk18; // bit 3 is set => we should update rotation of NiNode?
	bhkRigidBody * body; // 20
	// more?
};
static_assert(offsetof(bhkCollisionObject, body) == 0x20);

struct hkpShapePhantom : hkReferencedObject
{
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

struct hkpSimpleShapePhantom : hkpShapePhantom
{
	ahkpWorld * world; // 10

	void * userData; // 18

	hkpLinkedCollidable m_collidable; // 20

	UInt64 todo[10];

	hkMotionState m_motionState; // F0

	// more...
};
static_assert(offsetof(hkpSimpleShapePhantom, m_motionState) == 0xF0);

struct bhkSimpleShapePhantom : NiRefObject
{
	hkpSimpleShapePhantom * phantom; // 10
};


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

inline hkpWorldObject* hkGetWorldObject(const hkpCollidable* collidable)
{
	return reinterpret_cast<hkpWorldObject*>(const_cast<void*>(collidable->getOwner()));
}

inline hkpRigidBody* hkpGetRigidBody(const hkpCollidable* collidable)
{
	if (collidable->getType() == hkpWorldObject::BROAD_PHASE_ENTITY)
	{
		return static_cast<hkpRigidBody*>(hkGetWorldObject(collidable));
	}
	return nullptr;
}


typedef void(*_hkpMotion_setPositionAndRotation)(hkpMotion *_this, const hkVector4& position, const hkVector4& rotation); // rotation is hkQuaternion
typedef void(*_hkpMotion_setTransform)(hkpMotion *_this, const hkTransform& transform);
typedef hkBool(*_hkpShape_castRayImpl)(hkpShape *_this, const hkpShapeRayCastInput& input, hkpShapeRayCastOutput& output);
