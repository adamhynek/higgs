#include "physics.h"
#include "offsets.h"


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

	hkpCdBody *cdBody = point.m_cdBodyB;
	while (cdBody->m_parent) {
		cdBody = cdBody->m_parent;
	}
	//_MESSAGE("Hit: %x", cdBody->m_shape ? cdBody->m_shape->m_type : HK_SHAPE_INVALID);

	m_hits.push_back(std::make_pair(cdBody, point.m_contact));
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
		cdBody = cdBody->m_parent;
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

void RayHitCollector::addRayHit(const hkpCdBody * cdBody, const hkpShapeRayCastCollectorOutput * hitInfo)
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
	m_closestHitInfo = *hitInfo;
	m_doesHitExist = true;
	m_earlyOutHitFraction = hitInfo->m_hitFraction; // Only accept closer hits after this
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

void AllRayHitCollector::addRayHit(const hkpCdBody * cdBody, const hkpShapeRayCastCollectorOutput * hitInfo)
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

	hkpCdBody *body = const_cast<hkpCdBody *>(cdBody);
	while (body->m_parent) {
		body = body->m_parent;
	}

	m_hits.push_back(std::make_pair(body, *hitInfo));
	//m_earlyOutHitFraction = hitInfo->m_hitFraction; // Only accept closer hits after this
}
