#pragma once

#include <vector>

#include "RE/havok.h"

#include "skse64/NiObjects.h"


static const float g_minAllowedFingerAngle = 0;// 5 * 0.0174533; // 5 degrees

struct Triangle
{
	UInt16 vertexIndices[3];
};
static_assert(sizeof(Triangle) == 0x06);

struct TriangleData
{
	inline TriangleData(Triangle tri, uintptr_t vertices, UInt32 posOffset, UInt8 vertexSize) {
		uintptr_t vert = (vertices + tri.vertexIndices[0] * vertexSize);
		v0 = *(NiPoint3 *)(vert + posOffset);
		vert = (vertices + tri.vertexIndices[1] * vertexSize);
		v1 = *(NiPoint3 *)(vert + posOffset);
		vert = (vertices + tri.vertexIndices[2] * vertexSize);
		v2 = *(NiPoint3 *)(vert + posOffset);
	}

	inline TriangleData(Triangle tri, const std::vector<NiPoint3> &vertices) {
		v0 = vertices[tri.vertexIndices[0]];
		v1 = vertices[tri.vertexIndices[1]];
		v2 = vertices[tri.vertexIndices[2]];
	}

	TriangleData() : v0(), v1(), v2() {}

	inline void ApplyTransform(NiTransform &transform) {
		v0 = transform * v0;
		v1 = transform * v1;
		v2 = transform * v2;
	}

	NiPoint3 v0;
	NiPoint3 v1;
	NiPoint3 v2;
};

struct Intersection
{
	float angle; // angle of the fingertip at intersection pt
};

struct _OldIntersection
{
	BSTriShape *node; // the trishape where the intersected triangle resides
	Triangle tri; // triangle that was intersected
	float angle; // angle of the fingertip at intersection pt
};

struct Point2
{
	float x;
	float y;

	Point2();
	Point2(float X, float Y) : x(X), y(Y) { };

	Point2 Point2::operator- () const;
	Point2 Point2::operator+ (const Point2& pt) const;

	Point2 Point2::operator- (const Point2& pt) const;

	Point2& Point2::operator+= (const Point2& pt);
	Point2& Point2::operator-= (const Point2& pt);

	// Scalar operations
	Point2 Point2::operator* (float scalar) const;
	Point2 Point2::operator/ (float scalar) const;

	Point2& Point2::operator*= (float scalar);
	Point2& Point2::operator/= (float scalar);
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

	Result GetClosestPointOnTriangle(const NiPoint3 &point, const TriangleData &triangle);
}

inline float VectorLengthSquared(const NiPoint3 &vec) { return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z; }
inline float VectorLengthSquared(const Point2 &vec) { return vec.x*vec.x + vec.y*vec.y; }
inline float VectorLength(const NiPoint3 &vec) { return sqrtf(VectorLengthSquared(vec)); }
inline float VectorLength(const Point2 &vec) { return sqrtf(VectorLengthSquared(vec)); }
inline float DotProduct(const NiPoint3 &vec1, const NiPoint3 &vec2) { return vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z; }
inline float DotProduct(const Point2 &vec1, const Point2 &vec2) { return vec1.x*vec2.x + vec1.y*vec2.y; }
inline float DotProduct(const NiQuaternion &q1, const NiQuaternion &q2) { return q1.m_fW*q2.m_fW + q1.m_fX*q2.m_fX + q1.m_fY*q2.m_fY + q1.m_fZ*q2.m_fZ; }
inline float QuaternionLength(const NiQuaternion &q) { return sqrtf(DotProduct(q, q)); }
NiPoint3 VectorNormalized(const NiPoint3 &vec);
NiPoint3 CrossProduct(const NiPoint3 &vec1, const NiPoint3 &vec2);
NiMatrix33 MatrixFromAxisAngle(const NiPoint3 &axis, float theta);
NiPoint3 MatrixToEuler(const NiMatrix33 &mat);
NiMatrix33 EulerToMatrix(const NiPoint3 &euler);
NiPoint3 RotateVectorByAxisAngle(const NiPoint3 &vector, const NiPoint3 &axis, float angle);
void NiMatrixToHkMatrix(const NiMatrix33 &niMat, hkMatrix3 &hkMat);
void HkMatrixToNiMatrix(const hkMatrix3 &hkMat, NiMatrix33 &niMat);
NiMatrix33 QuaternionToMatrix(const NiQuaternion &q);
inline NiQuaternion HkQuatToNiQuat(const hkQuaternion &quat) { return { quat.m_vec(3), quat.m_vec(0), quat.m_vec(1), quat.m_vec(2) }; }
inline hkQuaternion NiQuatToHkQuat(const NiQuaternion &quat) { return hkQuaternion(quat.m_fX, quat.m_fY, quat.m_fZ, quat.m_fW); }
inline NiPoint3 HkVectorToNiPoint(const hkVector4 &vec) { return { vec.getQuad().m128_f32[0], vec.getQuad().m128_f32[1], vec.getQuad().m128_f32[2] }; }
inline hkVector4 NiPointToHkVector(const NiPoint3 &pt) { return { pt.x, pt.y, pt.z, 0 }; };
NiQuaternion QuaternionIdentity();
NiQuaternion QuaternionNormalized(const NiQuaternion &q);
NiQuaternion QuaternionMultiply(const NiQuaternion &qa, const NiQuaternion &qb);
NiQuaternion QuaternionMultiply(const NiQuaternion &q, float multiplier);
NiQuaternion QuaternionInverse(const NiQuaternion &q);
inline float QuaternionAngle(const NiQuaternion &qa, const NiQuaternion &qb) { return 2.0f * acosf(abs(DotProduct(qa, qb))); }
NiQuaternion slerp(const NiQuaternion &qa, const NiQuaternion &qb, double t);
inline NiPoint3 lerp(const NiPoint3 &a, const NiPoint3 &b, float t) { return a * (1.0f - t) + b * t; }
inline float lerp(float a, float b, float t) { return a * (1.0f - t) + b * t; }
float Determinant33(const NiMatrix33 &m);
NiPoint3 QuadraticFromPoints(const NiPoint2 &p1, const NiPoint2 &p2, const NiPoint2 &p3);

void GetSkinnedTriangles(NiAVObject *root, std::vector<TriangleData> &triangles);
void GetTriangles(NiAVObject *root, std::vector<TriangleData> &triangles);

bool GetIntersections(const std::vector<TriangleData> &triangles, int fingerIndex, float handScale, const NiPoint3 &center, const NiPoint3 &normal, const NiPoint3 &zeroAngleVector,
	float *outAngle);
void GetFingerIntersectionOnGraphicsGeometry(std::vector<Intersection> &tipIntersections, std::vector<Intersection> &outerIntersections, std::vector<Intersection> &innerIntersections,
	const std::vector<TriangleData> &triangles,
	int fingerIndex, float handScale, const NiPoint3 &center, const NiPoint3 &normal, const NiPoint3 &zeroAngleVector);
bool GetClosestPointOnGraphicsGeometry(NiAVObject *root, const NiPoint3 &point, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar);
bool GetClosestPointOnGraphicsGeometryToLine(const std::vector<TriangleData> &triangles, const NiPoint3 &point, const NiPoint3 &direction, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar);
