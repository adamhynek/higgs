#pragma once

#include "RE/havok.h"

#include "skse64/NiObjects.h"


struct Triangle
{
	UInt16 vertexIndices[3];
};
static_assert(sizeof(Triangle) == 0x06);

struct Intersection
{
	NiPoint3 pt; // intersection pt
	NiPoint3 pt2;
	BSTriShape *node; // the trishape where the intersected triangle resides
	Triangle tri; // triangle that was intersected
	bool hasPt2;
};

namespace MathUtils
{
	struct Result
	{
		float sqrDistance;
		// barycentric coordinates for triangle.v[3]
		float parameter[3];
		NiPoint3 closest;
	};

	Result GetClosestPointOnTriangle(NiPoint3 const& point, Triangle const& triangle, uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset);
}

inline float VectorLengthSquared(NiPoint3 vec) { return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z; }
inline float VectorLength(NiPoint3 vec) { return sqrtf(VectorLengthSquared(vec)); }
inline float DotProduct(NiPoint3 vec1, NiPoint3 vec2) { return vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z; }
NiPoint3 VectorNormalized(NiPoint3 vec);
NiPoint3 CrossProduct(NiPoint3 vec1, NiPoint3 vec2);
NiMatrix33 MatrixFromAxisAngle(NiPoint3 axis, float theta);
NiPoint3 MatrixToEuler(NiMatrix33 &mat);
NiMatrix33 EulerToMatrix(NiPoint3 euler);
NiPoint3 RotateVectorByAxisAngle(NiPoint3 vector, NiPoint3 axis, float angle);
void NiMatrixToHkMatrix(NiMatrix33 &niMat, hkMatrix3 &hkMat);
void HkMatrixToNiMatrix(hkMatrix3 &hkMat, NiMatrix33 &niMat);
inline NiPoint3 HkVectorToNiPoint(const hkVector4 &vec) { return { vec.getQuad().m128_f32[0], vec.getQuad().m128_f32[1], vec.getQuad().m128_f32[2] }; };
inline hkVector4 NiPointToHkVector(NiPoint3 &pt) { return { pt.x, pt.y, pt.z, 0 }; };
float Determinant33(const NiMatrix33 &m);
NiPoint3 QuadraticFromPoints(NiPoint2 p1, NiPoint2 p2, NiPoint2 p3);

bool GetIntersections(NiAVObject *root, NiPoint3 center, NiPoint3 point1, NiPoint3 point2, float tipLength, NiPoint3 normal, NiPoint3 zeroAngleVector, NiPoint3 palmDirection,
	NiPoint3 *closestPos, NiPoint3 *closestNormal, float *furthestDistanceSoFar, float *bestPointAngle);
bool GetDiskIntersectionOnGraphicsGeometry(std::vector<Intersection> &intersections, NiAVObject *root, NiPoint3 center, NiPoint3 point1, NiPoint3 point2, float tipLength, NiPoint3 normal, NiPoint3 zeroAngleVector,
	NiPoint3 *closestPos, NiPoint3 *closestNormal, float *furthestDistanceSoFar, float *bestPointAngle);
bool GetClosestPointOnGraphicsGeometry(NiAVObject *root, NiPoint3 point, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar);
bool GetClosestPointOnGraphicsGeometryToLine(NiAVObject *root, NiPoint3 point, NiPoint3 direction, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar);
