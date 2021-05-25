#pragma once

#include <vector>
#include "RE/havok.h"

#include <Physics/Collide/Shape/Query/hkpRayHitCollector.h>
#include <Physics/Collide/Shape/Query/hkpShapeRayCastCollectorOutput.h>
#include <Physics/Collide/Agent/Query/hkpCdPointCollector.h>
#include <Physics/Collide/Agent/Query/hkpCdBodyPairCollector.h>
#include <Physics/Collide/Agent/Collidable/hkpCdPoint.h>
#include <Physics/Dynamics/Collide/ContactListener/hkpContactListener.h>

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

struct ContactListener : public hkpContactListener
{
	void contactPointCallback(const hkpContactPointEvent& evnt) override;

	NiPointer<bhkWorld> world = nullptr;
};

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

	enum class State : UInt8
	{
		Unheld,
		HeldRight,
		HeldLeft,
		HeldBoth
	};

	struct CollisionMapEntry
	{
		UInt32 filterInfo;
		State state;
	};

	void ClearCollisionMap();
	void SetCollisionInfoDownstream(NiAVObject *obj, UInt32 collisionGroup, State reason);
	void SetCollisionGroupDownstream(NiAVObject *obj, UInt32 collisionGroup, State reason);
	void ResetCollisionInfoDownstream(NiAVObject *obj, State reaon, hkpCollidable *skipNode = nullptr, bool collideAll = true);
	void ResetCollisionGroupDownstream(NiAVObject *obj, State reason, hkpCollidable *skipNode);
	void ResetCollisionInfoKeyframed(bhkRigidBody *entity, hkpMotion::MotionType motionType, hkInt8 quality, State reason, bool collideAll = true);
}

void AddCustomCollisionLayer(bhkWorld *world);

void SetVelocityDownstream(NiAVObject *obj, hkVector4 velocity);
void SetAngularVelocityDownstream(NiAVObject *obj, hkVector4 velocity);
void ApplyHardKeyframeDownstream(NiAVObject *obj, hkVector4 pos, hkQuaternion rot, hkReal invDeltaTime);

float hkpContactPointEvent_getSeparatingVelocity(const hkpContactPointEvent &_this);
