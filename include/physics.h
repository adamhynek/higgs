#pragma once

#include <vector>
#include "RE/havok.h"

#include <Physics/Collide/Shape/Query/hkpRayHitCollector.h>
#include <Physics/Collide/Shape/Query/hkpShapeRayCastCollectorOutput.h>
#include <Physics/Collide/Agent/Query/hkpCdPointCollector.h>
#include <Physics/Collide/Agent/Query/hkpCdBodyPairCollector.h>
#include <Physics/Collide/Agent/Collidable/hkpCdPoint.h>


struct RayHitCollector : public _hkpRayHitCollector
{
public:
	RayHitCollector();
	inline void reset();
	void addRayHit(const hkpCdBody& cdBody, const _hkpShapeRayCastCollectorOutput& hitInfo) override;

	_hkpShapeRayCastCollectorOutput m_closestHitInfo;
	bool m_doesHitExist = false;
	//const hkpCdBody *m_closestCollidable = nullptr;
};

struct AllRayHitCollector : public _hkpRayHitCollector
{
public:
	AllRayHitCollector();
	inline void reset();
	void addRayHit(const hkpCdBody& cdBody, const _hkpShapeRayCastCollectorOutput& hitInfo) override;

	std::vector<std::pair<hkpCdBody *, _hkpShapeRayCastCollectorOutput>> m_hits;
};

struct CdPointCollector : public _hkpCdPointCollector
{
	CdPointCollector();
	void addCdPoint(const hkpCdPoint& point) override;
	inline void reset() override;

	std::vector<std::pair<hkpCdBody *, hkContactPoint>> m_hits;
};

struct CdBodyPairCollector : public _hkpCdBodyPairCollector
{
	CdBodyPairCollector();
	void addCdBodyPair(const hkpCdBody& bodyA, const hkpCdBody& bodyB) override;
	void reset() override;

	std::vector<hkpCdBody *> m_hits;
};
