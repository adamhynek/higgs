#include "math_utils.h"
#include "utils.h"
#include "config.h"
#include "triangle_triangle_intersection.h"
#include "PQP/TriDist.h"
#include "finger_curves.h"

#include "skse64/NiGeometry.h"

#include <array>
#include <unordered_set>


NiPoint3 VectorNormalized(const NiPoint3 &vec)
{
	float length = VectorLength(vec);
	return length ? vec / length : NiPoint3();
}

NiPoint3 CrossProduct(const NiPoint3 &vec1, const NiPoint3 &vec2)
{
	NiPoint3 result;
	result.x = vec1.y * vec2.z - vec1.z * vec2.y;
	result.y = vec1.z * vec2.x - vec1.x * vec2.z;
	result.z = vec1.x * vec2.y - vec1.y * vec2.x;
	return result;
}

NiMatrix33 MatrixFromAxisAngle(const NiPoint3 &axis, float theta)
{
	NiPoint3 a = axis;
	float cosTheta = cosf(theta);
	float sinTheta = sinf(theta);
	NiMatrix33 result;

	result.data[0][0] = cosTheta + a.x*a.x*(1 - cosTheta);
	result.data[0][1] = a.x*a.y*(1 - cosTheta) - a.z*sinTheta;
	result.data[0][2] = a.x*a.z*(1 - cosTheta) + a.y*sinTheta;

	result.data[1][0] = a.y*a.x*(1 - cosTheta) + a.z*sinTheta;
	result.data[1][1] = cosTheta + a.y*a.y*(1 - cosTheta);
	result.data[1][2] = a.y*a.z*(1 - cosTheta) - a.x*sinTheta;

	result.data[2][0] = a.z*a.x*(1 - cosTheta) - a.y*sinTheta;
	result.data[2][1] = a.z*a.y*(1 - cosTheta) + a.x*sinTheta;
	result.data[2][2] = cosTheta + a.z*a.z*(1 - cosTheta);

	return result;
}

NiPoint3 MatrixToEuler(const NiMatrix33 &mat)
{
	// Thanks DavidJCobb
	NiPoint3 output(0, 0, 0);

	float fY = asin(((float)(SInt32)(-mat.arr[2] * 1000000)) / 1000000);
	float fCY = cos(fY);
	float fCYTest = ((float)(SInt32)(fCY * 100)) / 100;
	float fTX, fTY;
	if (fCY && abs(fCY) >= 0.00000011920929 && fCYTest) {
		fTX = mat.arr[8] / fCY;
		fTY = mat.arr[5] / fCY;
		output.x = atan2(fTY, fTX);
		fTX = mat.arr[0] / fCY;
		fTY = mat.arr[1] / fCY;
		output.z = atan2(fTY, fTX);
	}
	else {
		output.x = 0;
		fTX = mat.arr[4];
		fTY = mat.arr[3];
		output.z = -atan2(fTY, fTX);
	}
	output.y = fY;
	return output;
}

NiMatrix33 EulerToMatrix(const NiPoint3 &euler)
{
	// Thanks DavidJCobb
	NiMatrix33 output;
	//
	float _x = euler.x;
	float _y = euler.y;
	float _z = euler.z;

	float fSinX = sin(_x);
	float fSinY = sin(_y);
	float fSinZ = sin(_z);
	float fCosX = cos(_x);
	float fCosY = cos(_y);
	float fCosZ = cos(_z);
	//
	// Build the matrix.
	//
	output.data[0][0] = fCosY * fCosZ; // 1,1
	output.data[0][1] = fCosY * fSinZ;
	output.data[0][2] = -fSinY;
	output.data[1][0] = fSinX * fSinY * fCosZ - fCosX * fSinZ; // 2,1
	output.data[1][1] = fSinX * fSinY * fSinZ + fCosX * fCosZ;
	output.data[1][2] = fSinX * fCosY;
	output.data[2][0] = fCosX * fSinY * fCosZ + fSinX * fSinZ; // 3,1
	output.data[2][1] = fCosX * fSinY * fSinZ - fSinX * fCosZ;
	output.data[2][2] = fCosX * fCosY;
	//
	return output;
};

NiPoint3 RotateVectorByAxisAngle(const NiPoint3 &vector, const NiPoint3 &axis, float angle)
{
	// Rodrigues' rotation formula
	float cosTheta = cosf(angle);
	return vector * cosTheta + (CrossProduct(axis, vector) * sinf(angle)) + axis * DotProduct(axis, vector) * (1.0f - cosTheta);
}

void NiMatrixToHkMatrix(const NiMatrix33 &niMat, hkMatrix3 &hkMat)
{
	hkMat.setCols({ niMat.data[0][0], niMat.data[1][0], niMat.data[2][0], 0 },
		{ niMat.data[0][1], niMat.data[1][1], niMat.data[2][1], 0 },
		{ niMat.data[0][2], niMat.data[1][2], niMat.data[2][2], 0 });
}

void HkMatrixToNiMatrix(const hkMatrix3 &hkMat, NiMatrix33 &niMat)
{
	hkVector4 col0, col1, col2;
	hkMat.getCols(col0, col1, col2);

	niMat.data[0][0] = col0(0);
	niMat.data[1][0] = col0(1);
	niMat.data[2][0] = col0(2);

	niMat.data[0][1] = col1(0);
	niMat.data[1][1] = col1(1);
	niMat.data[2][1] = col1(2);

	niMat.data[0][2] = col2(0);
	niMat.data[1][2] = col2(1);
	niMat.data[2][2] = col2(2);
}

NiMatrix33 QuaternionToMatrix(const NiQuaternion &q)
{
	double sqw = q.m_fW*q.m_fW;
	double sqx = q.m_fX*q.m_fX;
	double sqy = q.m_fY*q.m_fY;
	double sqz = q.m_fZ*q.m_fZ;

	NiMatrix33 m;

	// invs (inverse square length) is only required if quaternion is not already normalised
	double invs = 1 / (sqx + sqy + sqz + sqw);
	m.data[0][0] = (sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
	m.data[1][1] = (-sqx + sqy - sqz + sqw)*invs;
	m.data[2][2] = (-sqx - sqy + sqz + sqw)*invs;

	double tmp1 = q.m_fX*q.m_fY;
	double tmp2 = q.m_fZ*q.m_fW;
	m.data[1][0] = 2.0 * (tmp1 + tmp2)*invs;
	m.data[0][1] = 2.0 * (tmp1 - tmp2)*invs;

	tmp1 = q.m_fX*q.m_fZ;
	tmp2 = q.m_fY*q.m_fW;
	m.data[2][0] = 2.0 * (tmp1 - tmp2)*invs;
	m.data[0][2] = 2.0 * (tmp1 + tmp2)*invs;
	tmp1 = q.m_fY*q.m_fZ;
	tmp2 = q.m_fX*q.m_fW;
	m.data[2][1] = 2.0 * (tmp1 + tmp2)*invs;
	m.data[1][2] = 2.0 * (tmp1 - tmp2)*invs;

	return m;
}

NiQuaternion QuaternionIdentity()
{
	return { 1, 0, 0, 0 };
}

NiQuaternion QuaternionNormalized(const NiQuaternion &q)
{
	float length = QuaternionLength(q);
	if (length) {
		return QuaternionMultiply(q, 1.0f / length);
	}
	return QuaternionIdentity();
}

NiQuaternion QuaternionMultiply(const NiQuaternion &qa, const NiQuaternion &qb)
{
	NiQuaternion multiple;
	multiple.m_fW = qa.m_fW * qb.m_fW - qa.m_fX * qb.m_fX - qa.m_fY * qb.m_fY - qa.m_fZ * qb.m_fZ;
	multiple.m_fX = qa.m_fW * qb.m_fX + qa.m_fX * qb.m_fW + qa.m_fY * qb.m_fZ - qa.m_fZ * qb.m_fY;
	multiple.m_fY = qa.m_fW * qb.m_fY - qa.m_fX * qb.m_fZ + qa.m_fY * qb.m_fW + qa.m_fZ * qb.m_fX;
	multiple.m_fZ = qa.m_fW * qb.m_fZ + qa.m_fX * qb.m_fY - qa.m_fY * qb.m_fX + qa.m_fZ * qb.m_fW;
	return multiple;
}

NiQuaternion QuaternionMultiply(const NiQuaternion &q, float multiplier)
{
	NiQuaternion multiple;
	multiple.m_fW = q.m_fW * multiplier;
	multiple.m_fX = q.m_fX * multiplier;
	multiple.m_fY = q.m_fY * multiplier;
	multiple.m_fZ = q.m_fZ * multiplier;
	return multiple;
}

NiQuaternion QuaternionInverse(const NiQuaternion &q)
{
	NiQuaternion inverse;
	float normSquared = q.m_fW*q.m_fW + q.m_fX*q.m_fX + q.m_fY*q.m_fY + q.m_fZ*q.m_fZ;
	if (!normSquared)
		normSquared = 1;
	inverse.m_fW = q.m_fW / normSquared;
	inverse.m_fX = -q.m_fX / normSquared;
	inverse.m_fY = -q.m_fY / normSquared;
	inverse.m_fZ = -q.m_fZ / normSquared;
	return inverse;
}

NiQuaternion slerp(const NiQuaternion &qa, const NiQuaternion &qb, double t)
{
	// quaternion to return
	NiQuaternion qm;
	// Calculate angle between them.
	double cosHalfTheta = DotProduct(qa, qb);
	// if qa=qb or qa=-qb then theta = 0 and we can return qa
	if (abs(cosHalfTheta) >= 0.9995) {
		qm.m_fW = qa.m_fW;
		qm.m_fX = qa.m_fX;
		qm.m_fY = qa.m_fY;
		qm.m_fZ = qa.m_fZ;
		return qm;
	}

	// If the dot product is negative, slerp won't take
	// the shorter path. Note that qb and -qb are equivalent when
	// the negation is applied to all four components. Fix by 
	// reversing one quaternion.
	NiQuaternion q2 = qb;
	if (cosHalfTheta < 0) {
		q2.m_fW *= -1;
		q2.m_fX *= -1;
		q2.m_fY *= -1;
		q2.m_fZ *= -1;
		cosHalfTheta *= -1;
	}

	// Calculate temporary values.
	double halfTheta = acos(cosHalfTheta);
	double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);
	// if theta = 180 degrees then result is not fully defined
	// we could rotate around any axis normal to qa or qb
	if (fabs(sinHalfTheta) < 0.001) { // fabs is floating point absolute
		qm.m_fW = (qa.m_fW * 0.5 + q2.m_fW * 0.5);
		qm.m_fX = (qa.m_fX * 0.5 + q2.m_fX * 0.5);
		qm.m_fY = (qa.m_fY * 0.5 + q2.m_fY * 0.5);
		qm.m_fZ = (qa.m_fZ * 0.5 + q2.m_fZ * 0.5);
		return qm;
	}
	double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
	double ratioB = sin(t * halfTheta) / sinHalfTheta;
	//calculate Quaternion.
	qm.m_fW = (qa.m_fW * ratioA + q2.m_fW * ratioB);
	qm.m_fX = (qa.m_fX * ratioA + q2.m_fX * ratioB);
	qm.m_fY = (qa.m_fY * ratioA + q2.m_fY * ratioB);
	qm.m_fZ = (qa.m_fZ * ratioA + q2.m_fZ * ratioB);
	return qm;
}

float Determinant33(const NiMatrix33 &m)
{
	float a = m.data[0][0];
	float b = m.data[0][1];
	float c = m.data[0][2];
	float d = m.data[1][0];
	float e = m.data[1][1];
	float f = m.data[1][2];
	float g = m.data[2][0];
	float h = m.data[2][1];
	float i = m.data[2][2];
	return a * (e*i - f * h) - b * (d*i - f * g) + c * (d*h - e * g);
}

NiPoint3 QuadraticFromPoints(const NiPoint2 &p1, const NiPoint2 &p2, const NiPoint2 &p3)
{
	// Fit a quadratic to the 3 given points, and return the coefficients of x^2, x^1, x^0
	float x1sqr = p1.x*p1.x;
	float x2sqr = p2.x*p2.x;
	float x3sqr = p3.x*p3.x;

	float inverseDet = x1sqr * (p2.x - p3.x) - p1.x * (x2sqr - x3sqr) + (x2sqr*p3.x - x3sqr * p2.x);
	if (abs(inverseDet) <= 0.0001f) {
		return NiPoint3();
	}
	inverseDet = 1.0f / inverseDet;

	NiMatrix33 i; // inverse

	i.data[0][0] = p2.x - p3.x;
	i.data[0][1] = p3.x - p1.x;
	i.data[0][2] = p1.x - p2.x;

	i.data[1][0] = x3sqr - x2sqr;
	i.data[1][1] = x1sqr - x3sqr;
	i.data[1][2] = x2sqr - x1sqr;

	i.data[2][0] = x2sqr * p3.x - x3sqr * p2.x;
	i.data[2][1] = x3sqr * p1.x - x1sqr * p3.x;
	i.data[2][2] = x1sqr * p2.x - x2sqr * p1.x;

	i = i * inverseDet;

	NiPoint3 yVals = { p1.y, p2.y, p3.y };
	return i * yVals;
}

Point2::Point2()
{
	x = 0.0f;
	y = 0.0f;
}

Point2 Point2::operator- () const
{
	return Point2(-x, -y);
}

Point2 Point2::operator+ (const Point2& pt) const
{
	return Point2(x + pt.x, y + pt.y);
}

Point2 Point2::operator- (const Point2& pt) const
{
	return Point2(x - pt.x, y - pt.y);
}

Point2& Point2::operator+= (const Point2& pt)
{
	x += pt.x;
	y += pt.y;
	return *this;
}
Point2& Point2::operator-= (const Point2& pt)
{
	x -= pt.x;
	y -= pt.y;
	return *this;
}

// Scalar operations
Point2 Point2::operator* (float scalar) const
{
	return Point2(scalar * x, scalar * y);
}
Point2 Point2::operator/ (float scalar) const
{
	float invScalar = 1.0f / scalar;
	return Point2(invScalar * x, invScalar * y);
}

Point2& Point2::operator*= (float scalar)
{
	x *= scalar;
	y *= scalar;
	return *this;
}
Point2& Point2::operator/= (float scalar)
{
	float invScalar = 1.0f / scalar;
	x *= invScalar;
	y *= invScalar;
	return *this;
}


namespace MathUtils
{
	Result GetClosestPointOnTriangle(const NiPoint3 &point, const Triangle &triangle, uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset)
	{
		// Taken from https://www.geometrictools.com/GTE//Mathematics/DistPointTriangleExact.h and adapted

		uintptr_t vert = (vertices + triangle.vertexIndices[0] * vertexStride);
		NiPoint3 pos0 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[1] * vertexStride);
		NiPoint3 pos1 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[2] * vertexStride);
		NiPoint3 pos2 = *(NiPoint3 *)(vert + vertexPosOffset);

		NiPoint3 diff = point - pos0;
		NiPoint3 edge0 = pos1 - pos0;
		NiPoint3 edge1 = pos2 - pos0;
		float a00 = DotProduct(edge0, edge0);
		float a01 = DotProduct(edge0, edge1);
		float a11 = DotProduct(edge1, edge1);
		float b0 = -DotProduct(diff, edge0);
		float b1 = -DotProduct(diff, edge1);
		float det = a00 * a11 - a01 * a01;
		float t0 = a01 * b1 - a11 * b0;
		float t1 = a01 * b0 - a00 * b1;

		if (t0 + t1 <= det)
		{
			if (t0 < 0)
			{
				if (t1 < 0)  // region 4
				{
					if (b0 < 0)
					{
						t1 = 0;
						if (-b0 >= a00)  // V1
						{
							t0 = 1;
						}
						else  // E01
						{
							t0 = -b0 / a00;
						}
					}
					else
					{
						t0 = 0;
						if (b1 >= 0)  // V0
						{
							t1 = 0;
						}
						else if (-b1 >= a11)  // V2
						{
							t1 = 1;
						}
						else  // E20
						{
							t1 = -b1 / a11;
						}
					}
				}
				else  // region 3
				{
					t0 = 0;
					if (b1 >= 0)  // V0
					{
						t1 = 0;
					}
					else if (-b1 >= a11)  // V2
					{
						t1 = 1;
					}
					else  // E20
					{
						t1 = -b1 / a11;
					}
				}
			}
			else if (t1 < 0)  // region 5
			{
				t1 = 0;
				if (b0 >= 0)  // V0
				{
					t0 = 0;
				}
				else if (-b0 >= a00)  // V1
				{
					t0 = 1;
				}
				else  // E01
				{
					t0 = -b0 / a00;
				}
			}
			else  // region 0, interior
			{
				float invDet = 1 / det;
				t0 *= invDet;
				t1 *= invDet;
			}
		}
		else
		{
			float tmp0, tmp1, numer, denom;

			if (t0 < 0)  // region 2
			{
				tmp0 = a01 + b0;
				tmp1 = a11 + b1;
				if (tmp1 > tmp0)
				{
					numer = tmp1 - tmp0;
					denom = a00 - (float)2 * a01 + a11;
					if (numer >= denom)  // V1
					{
						t0 = 1;
						t1 = 0;
					}
					else  // E12
					{
						t0 = numer / denom;
						t1 = 1 - t0;
					}
				}
				else
				{
					t0 = 0;
					if (tmp1 <= 0)  // V2
					{
						t1 = 1;
					}
					else if (b1 >= 0)  // V0
					{
						t1 = 0;
					}
					else  // E20
					{
						t1 = -b1 / a11;
					}
				}
			}
			else if (t1 < 0)  // region 6
			{
				tmp0 = a01 + b1;
				tmp1 = a00 + b0;
				if (tmp1 > tmp0)
				{
					numer = tmp1 - tmp0;
					denom = a00 - (float)2 * a01 + a11;
					if (numer >= denom)  // V2
					{
						t1 = 1;
						t0 = 0;
					}
					else  // E12
					{
						t1 = numer / denom;
						t0 = 1 - t1;
					}
				}
				else
				{
					t1 = 0;
					if (tmp1 <= 0)  // V1
					{
						t0 = 1;
					}
					else if (b0 >= 0)  // V0
					{
						t0 = 0;
					}
					else  // E01
					{
						t0 = -b0 / a00;
					}
				}
			}
			else  // region 1
			{
				numer = a11 + b1 - a01 - b0;
				if (numer <= 0)  // V2
				{
					t0 = 0;
					t1 = 1;
				}
				else
				{
					denom = a00 - (float)2 * a01 + a11;
					if (numer >= denom)  // V1
					{
						t0 = 1;
						t1 = 0;
					}
					else  // 12
					{
						t0 = numer / denom;
						t1 = 1 - t0;
					}
				}
			}
		}

		Result result;
		result.parameter[0] = 1 - t0 - t1;
		result.parameter[1] = t0;
		result.parameter[2] = t1;
		result.closest = pos0 + (edge0 * t0) + (edge1 * t1);
		diff = point - result.closest;
		result.sqrDistance = DotProduct(diff, diff);
		return result;
	}

	bool RayIntersectsTriangle(const NiPoint3 &rayOrigin,
		const NiPoint3 &rayVector,
		const Triangle &triangle,
		NiPoint3 &outIntersectionPoint,
		uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset)
	{
		const float EPSILON = 0.0000001;

		uintptr_t vert = (vertices + triangle.vertexIndices[0] * vertexStride);
		NiPoint3 vertex0 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[1] * vertexStride);
		NiPoint3 vertex1 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[2] * vertexStride);
		NiPoint3 vertex2 = *(NiPoint3 *)(vert + vertexPosOffset);

		NiPoint3 edge1, edge2, h, s, q;
		float a, f, u, v;
		edge1 = vertex1 - vertex0;
		edge2 = vertex2 - vertex0;
		h = CrossProduct(rayVector, edge2);
		a = DotProduct(edge1, h);
		if (a > -EPSILON && a < EPSILON)
			return false;    // This ray is parallel to this triangle.
		f = 1.0 / a;
		s = rayOrigin - vertex0;
		u = f * DotProduct(s, h);
		if (u < 0.0 || u > 1.0)
			return false;
		q = CrossProduct(s, edge1);
		v = f * DotProduct(rayVector, q);
		if (v < 0.0 || u + v > 1.0)
			return false;
		// At this stage we can compute t to find out where the intersection point is on the line.
		float t = f * DotProduct(edge2, q);
		if (t > EPSILON) // ray intersection
		{
			outIntersectionPoint = rayOrigin + rayVector * t;
			return true;
		}
		else // This means that there is a line intersection but not a ray intersection.
			return false;
	}

	bool GetClosestPointOnTriangleToLine(const NiPoint3 &rayOrigin,
		const NiPoint3 &rayVector,
		const Triangle &triangle,
		NiPoint3 &outIntersectionPoint,
		float &outSqrDistance,
		bool &outIntersects,
		uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset)
	{
		const float EPSILON = 0.0000001;

		uintptr_t vert = (vertices + triangle.vertexIndices[0] * vertexStride);
		NiPoint3 vertex0 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[1] * vertexStride);
		NiPoint3 vertex1 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[2] * vertexStride);
		NiPoint3 vertex2 = *(NiPoint3 *)(vert + vertexPosOffset);

		bool intersects = true;
		NiPoint3 edge1, edge2, h, s, q;
		float a, f, u, v;
		edge1 = vertex1 - vertex0;
		edge2 = vertex2 - vertex0;
		h = CrossProduct(rayVector, edge2);
		a = DotProduct(edge1, h);
		if (a > -EPSILON && a < EPSILON)
			return false;    // This ray is parallel to this triangle.
		f = 1.0 / a;
		s = rayOrigin - vertex0;
		u = f * DotProduct(s, h);
		if (u < 0.0 || u > 1.0)
			intersects = false;
		q = CrossProduct(s, edge1);
		v = f * DotProduct(rayVector, q);
		if (v < 0.0 || u + v > 1.0)
			intersects = false;

		// At this stage we can compute t to find out where the intersection point is on the line.
		float t = f * DotProduct(edge2, q);
		//if (t > EPSILON) // ray intersection
		//{
		NiPoint3 intersectPoint = rayOrigin + rayVector * t;
		Result result = GetClosestPointOnTriangle(intersectPoint, triangle, vertices, vertexStride, vertexPosOffset);
		outIntersectionPoint = result.closest;
		outSqrDistance = result.sqrDistance;
		outIntersects = intersects;
		return true;
		//}
		//else // This means that there is a line intersection but not a ray intersection.
		//	return false;
	}

	NiPoint3 GetClosestPointOnLineSegment(const NiPoint3 &start, const NiPoint3 &end, const NiPoint3 &point)
	{
		float l2 = VectorLengthSquared(end - start);  // i.e. |w-v|^2
		if (l2 == 0.0) return start;   // v == w case
		// Consider the line extending the segment, parameterized as v + t (w - v).
		// We find projection of point p onto the line. 
		// It falls where t = [(p-v) . (w-v)] / |w-v|^2
		// We clamp t from [0,1] to handle points outside the segment vw.
		float t = max(0, min(1, DotProduct(point - start, end - start) / l2));
		NiPoint3 projection = start + (end - start) * t;  // Projection falls on the segment
		return projection;
	}

	Point2 GetClosestPointOnLineSegment(const Point2 &start, const Point2 &end, const Point2 &point)
	{
		float l2 = VectorLengthSquared(end - start);
		if (l2 == 0.0) return start;
		float t = max(0, min(1, DotProduct(point - start, end - start) / l2));
		Point2 projection = start + (end - start) * t;
		return projection;
	}

	NiPoint3 GetFurthestPointOnLineSegment(const NiPoint3 &start, const NiPoint3 &end, const NiPoint3 &point)
	{
		float l2 = VectorLengthSquared(end - start);
		if (l2 == 0.0) return start;   // v == w case

		return VectorLengthSquared(start - point) >= VectorLengthSquared(end - point) ? start : end;
	}

	float LineSegmentLineSegmentDistance(const Point2 &a, const Point2 &b, const Point2 &c, const Point2 &d)
	{
		return min(
			VectorLength(GetClosestPointOnLineSegment(c, d, a) - a),
			min(
				VectorLength(GetClosestPointOnLineSegment(c, d, b) - b),
				min(
					VectorLength(GetClosestPointOnLineSegment(a, b, c) - c),
					VectorLength(GetClosestPointOnLineSegment(a, b, d) - d)
				)
			)
		);
	}

	// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
	// intersect the intersection point may be stored in the floats i_x and i_y.
	bool LineSegmentIntersectsLineSegment(const Point2 &p0, const Point2 &p1, const Point2 &p2, const Point2 &p3, Point2 *intersection)
	{
		Point2 s1 = p1 - p0;
		Point2 s2 = p3 - p2;

		float det = (-s2.x * s1.y + s1.x * s2.y);
		if (fabsf(det) < 0.0000001)
			return false; // line segments are parallel

		float s = (-s1.y * (p0.x - p2.x) + s1.x * (p0.y - p2.y)) / det;
		float t = (s2.x * (p0.y - p2.y) - s2.y * (p0.x - p2.x)) / det;

		if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
		{
			// Collision detected
			if (intersection) {
				*intersection = p0 + s1 * t;
			}
			return true;
		}

		return false; // No collision
	}

	bool LinePlaneIntersection(NiPoint3 &contact, const NiPoint3 &ray, const NiPoint3 &rayOrigin,
		const NiPoint3 &normal, const NiPoint3 &coord)
	{
		// get d value
		float d = DotProduct(normal, coord);

		float normalDotRay = DotProduct(normal, ray);
		if (normalDotRay < 0.0000001) {
			return false; // No intersection, the line is parallel to the plane
		}

		// Compute the X value for the directed line ray intersecting the plane
		float x = (d - DotProduct(normal, rayOrigin)) / normalDotRay;

		// output contact point
		contact = rayOrigin + ray * x; //Make sure your ray vector is normalized
		return true;
	}

	bool PlaneIntersectsLineSegment(const NiPoint3 &planePoint, const NiPoint3 &planeNormal, const NiPoint3 &segmentStart, const NiPoint3 &segmentFinish, NiPoint3 &outPoint)
	{
		NiPoint3 edge = segmentFinish - segmentStart;
		float edgeLength = VectorLength(edge);
		NiPoint3 ray = edgeLength ? edge / edgeLength : NiPoint3();

		// get d value, where plane eq: n*x = d
		float d = DotProduct(planeNormal, planePoint);

		float normalDotRay = DotProduct(planeNormal, ray);
		if (abs(normalDotRay) < 0.0000001) {
			return false; // No intersection, the line is parallel to the plane
		}

		// Compute the X value for the directed line ray intersecting the plane
		float x = (d - DotProduct(planeNormal, segmentStart)) / normalDotRay;

		// output contact point
		if (x < 0 || x > edgeLength) {
			// intersection point is not on the line segment
			return false;
		}

		outPoint = segmentStart + ray * x;
		return true;
	}

	// Return # of intersection points (0, 1, or 2)
	int CircleIntersectsTriangle(const NiPoint3 &circleCenter,
		const NiPoint3 &circleNormal,
		float circleRadius,
		const Triangle &triangle,
		NiPoint3 &outIntersectionPoint1,
		NiPoint3 &outIntersectionPoint2,
		uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset)
	{
		uintptr_t vert = (vertices + triangle.vertexIndices[0] * vertexStride);
		NiPoint3 vertex0 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[1] * vertexStride);
		NiPoint3 vertex1 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[2] * vertexStride);
		NiPoint3 vertex2 = *(NiPoint3 *)(vert + vertexPosOffset);

		// Check each triangle edge for intersection. Either none intersect, or 2 of them do.
		NiPoint3 edge1Intersection, edge2Intersection, edge3Intersection;
		bool edge1Intersects = PlaneIntersectsLineSegment(circleCenter, circleNormal, vertex0, vertex1, edge1Intersection);
		bool edge2Intersects = PlaneIntersectsLineSegment(circleCenter, circleNormal, vertex0, vertex2, edge2Intersection);
		if (!edge1Intersects && !edge2Intersects) {
			return 0; // Impossible for 2 edges to intersect at this point
		}
		bool edge3Intersects = PlaneIntersectsLineSegment(circleCenter, circleNormal, vertex1, vertex2, edge3Intersection);

		int numIntersections = edge1Intersects + edge2Intersects + edge3Intersects;
		if (numIntersections < 2) {
			return 0;
		}

		// p: start point of intersection line; d: direction vector of intersection line
		NiPoint3 d, p;
		float edgeLength;
		if (edge1Intersects && edge2Intersects) {
			NiPoint3 edge = edge2Intersection - edge1Intersection;
			edgeLength = VectorLength(edge);
			d = edgeLength ? edge / edgeLength : NiPoint3();
			p = edge1Intersection - circleCenter; // Make the circle center the origin, it makes the math cleaner
		}
		else if (edge1Intersects && edge3Intersects) {
			NiPoint3 edge = edge3Intersection - edge1Intersection;
			edgeLength = VectorLength(edge);
			d = edgeLength ? edge / edgeLength : NiPoint3();
			p = edge1Intersection - circleCenter;
		}
		else if (edge2Intersects && edge3Intersects) {
			NiPoint3 edge = edge3Intersection - edge2Intersection;
			edgeLength = VectorLength(edge);
			d = edgeLength ? edge / edgeLength : NiPoint3();
			p = edge2Intersection - circleCenter;
		}
		else {
			// Impossible
			ASSERT(false);
			return 0;
		}

		// Solve quadratic for pt that is on the intersection line as well as on the circle (dist to circle center == radius)
		float pDotd = DotProduct(p, d);
		float dLength2 = VectorLengthSquared(d);
		float pLength2 = VectorLengthSquared(p);
		float discriminant = pDotd * pDotd - dLength2 * (pLength2 - circleRadius * circleRadius);
		if (discriminant < 0) {
			return 0;
		}
		float t1 = (-pDotd + sqrtf(discriminant)) / dLength2;
		float t2 = (-pDotd - sqrtf(discriminant)) / dLength2;

		bool t1IsOnSegment = t1 >= 0 && t1 <= edgeLength;
		bool t2IsOnSegment = t2 >= 0 && t2 <= edgeLength;

		if (t1IsOnSegment && t2IsOnSegment) {
			// This is rare... the circle intersects the triangle at 2 points
			outIntersectionPoint1 = circleCenter + p + d * t1; // Add back the circle center after making it the origin
			outIntersectionPoint2 = circleCenter + p + d * t2;
			return 2;
		}
		else if (t1IsOnSegment) {
			// t1 is on the segment
			outIntersectionPoint1 = circleCenter + p + d * t1;
			return 1;
		}
		else if (t2IsOnSegment) {
			// t2 is on the segment
			outIntersectionPoint1 = circleCenter + p + d * t2;
			return 1;
		}

		return 0;
	}

	// Return # of intersection points (0, 1, or 2)
	int DiskIntersectsTriangle(const NiPoint3 &diskCenter,
		const NiPoint3 &diskNormal,
		float diskRadius,
		const Triangle &triangle,
		NiPoint3 &outIntersectionPoint1,
		NiPoint3 &outIntersectionPoint2,
		uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset)
	{
		uintptr_t vert = (vertices + triangle.vertexIndices[0] * vertexStride);
		NiPoint3 vertex0 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[1] * vertexStride);
		NiPoint3 vertex1 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[2] * vertexStride);
		NiPoint3 vertex2 = *(NiPoint3 *)(vert + vertexPosOffset);

		// Check each triangle edge for intersection. Either none intersect, or 2 of them do.
		NiPoint3 edge1Intersection, edge2Intersection, edge3Intersection;
		bool edge1Intersects = PlaneIntersectsLineSegment(diskCenter, diskNormal, vertex0, vertex1, edge1Intersection);
		bool edge2Intersects = PlaneIntersectsLineSegment(diskCenter, diskNormal, vertex0, vertex2, edge2Intersection);
		if (!edge1Intersects && !edge2Intersects) {
			return 0; // Impossible for 2 edges to intersect at this point
		}
		bool edge3Intersects = PlaneIntersectsLineSegment(diskCenter, diskNormal, vertex1, vertex2, edge3Intersection);

		int numIntersections = edge1Intersects + edge2Intersects + edge3Intersects;
		if (numIntersections < 2) {
			return 0;
		}

		// p: start point of intersection line; d: direction vector of intersection line
		NiPoint3 d, p;
		float edgeLength;
		if (edge1Intersects && edge2Intersects) {
			NiPoint3 edge = edge2Intersection - edge1Intersection;
			edgeLength = VectorLength(edge);
			d = edgeLength ? edge / edgeLength : NiPoint3();
			p = edge1Intersection - diskCenter; // Make the circle center the origin, it makes the math cleaner
		}
		else if (edge1Intersects && edge3Intersects) {
			NiPoint3 edge = edge3Intersection - edge1Intersection;
			edgeLength = VectorLength(edge);
			d = edgeLength ? edge / edgeLength : NiPoint3();
			p = edge1Intersection - diskCenter;
		}
		else if (edge2Intersects && edge3Intersects) {
			NiPoint3 edge = edge3Intersection - edge2Intersection;
			edgeLength = VectorLength(edge);
			d = edgeLength ? edge / edgeLength : NiPoint3();
			p = edge2Intersection - diskCenter;
		}
		else {
			// Impossible
			return 0;
		}

		NiPoint3 furthestPoint = GetFurthestPointOnLineSegment(p, p + d * edgeLength, { 0, 0, 0 }); // circle center is the origin, so {0, 0, 0}
		if (VectorLength(furthestPoint) > diskRadius) {
			// Furthest point is past the end of the disk. See if there is another point at the edge of the disk.

			// Solve quadratic for pt that is on the intersection line as well as on the circle (dist to circle center == radius)
			float pDotd = DotProduct(p, d);
			float dLength2 = VectorLengthSquared(d);
			float pLength2 = VectorLengthSquared(p);
			float discriminant = pDotd * pDotd - dLength2 * (pLength2 - diskRadius * diskRadius);
			if (discriminant < 0) {
				return 0;
			}
			float t1 = (-pDotd + sqrtf(discriminant)) / dLength2;
			float t2 = (-pDotd - sqrtf(discriminant)) / dLength2;

			bool t1IsOnSegment = t1 >= 0 && t1 <= edgeLength;
			bool t2IsOnSegment = t2 >= 0 && t2 <= edgeLength;

			if (t1IsOnSegment && t2IsOnSegment) {
				// This is rare... the circle intersects the triangle at 2 points
				outIntersectionPoint1 = diskCenter + p + d * t1; // Add back the circle center after making it the origin
				outIntersectionPoint2 = diskCenter + p + d * t2;
				return 2;
			}
			else if (t1IsOnSegment) {
				// t1 is on the segment
				outIntersectionPoint1 = diskCenter + p + d * t1;
				return 1;
			}
			else if (t2IsOnSegment) {
				// t2 is on the segment
				outIntersectionPoint1 = diskCenter + p + d * t2;
				return 1;
			}
		}
		else {
			// Intersection line is entirely inside the disk
			outIntersectionPoint1 = diskCenter + furthestPoint;
			return 1;
		}

		return 0;
	}

	// Return # of intersection points (0, 1, or 2)
	int FingerIntersectsTriangle(int fingerIndex,
		const NiPoint3 &diskCenter,
		const NiPoint3 &diskNormal,
		const NiPoint3 &zeroAngleVector,
		float scale,
		const Triangle &triangle,
		NiPoint3 &outIntersectionPoint1,
		NiPoint3 &outIntersectionPoint2,
		uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset)
	{
		uintptr_t vert = (vertices + triangle.vertexIndices[0] * vertexStride);
		NiPoint3 vertex0 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[1] * vertexStride);
		NiPoint3 vertex1 = *(NiPoint3 *)(vert + vertexPosOffset);
		vert = (vertices + triangle.vertexIndices[2] * vertexStride);
		NiPoint3 vertex2 = *(NiPoint3 *)(vert + vertexPosOffset);

		// Check each triangle edge for intersection. Either none intersect, or 2 of them do.
		NiPoint3 edge1Intersection, edge2Intersection, edge3Intersection;
		bool edge1Intersects = PlaneIntersectsLineSegment(diskCenter, diskNormal, vertex0, vertex1, edge1Intersection);
		bool edge2Intersects = PlaneIntersectsLineSegment(diskCenter, diskNormal, vertex0, vertex2, edge2Intersection);
		if (!edge1Intersects && !edge2Intersects) {
			return 0; // Impossible for 2 edges to intersect at this point
		}
		bool edge3Intersects = PlaneIntersectsLineSegment(diskCenter, diskNormal, vertex1, vertex2, edge3Intersection);

		int numIntersections = edge1Intersects + edge2Intersects + edge3Intersects;
		if (numIntersections < 2) {
			return 0;
		}

		// p: start point of intersection line; d: direction vector of intersection line
		NiPoint3 d, p;
		float edgeLength;
		if (edge1Intersects && edge2Intersects) {
			NiPoint3 edge = edge2Intersection - edge1Intersection;
			edgeLength = VectorLength(edge);
			d = edgeLength ? edge / edgeLength : NiPoint3();
			p = edge1Intersection - diskCenter; // Make the circle center the origin, it makes the math cleaner
		}
		else if (edge1Intersects && edge3Intersects) {
			NiPoint3 edge = edge3Intersection - edge1Intersection;
			edgeLength = VectorLength(edge);
			d = edgeLength ? edge / edgeLength : NiPoint3();
			p = edge1Intersection - diskCenter;
		}
		else if (edge2Intersects && edge3Intersects) {
			NiPoint3 edge = edge3Intersection - edge2Intersection;
			edgeLength = VectorLength(edge);
			d = edgeLength ? edge / edgeLength : NiPoint3();
			p = edge2Intersection - diskCenter;
		}
		else {
			// Impossible
			return 0;
		}

		NiPoint3 edgeStart = p;
		NiPoint3 edgeEnd = p + d * edgeLength;


		float startAngle = acosf(DotProduct(VectorNormalized(edgeStart), zeroAngleVector));
		if (DotProduct(diskNormal, CrossProduct(zeroAngleVector, edgeStart)) < 0) {
			// Positive angles are those which curl the finger
			startAngle *= -1;
		}

		float endAngle = acosf(DotProduct(VectorNormalized(edgeEnd), zeroAngleVector));
		if (DotProduct(diskNormal, CrossProduct(zeroAngleVector, edgeEnd)) < 0) {
			endAngle *= -1;
		}

		bool crossesBehind = false;
		float angleCorrection = 0;
		float smallestAngle = min(startAngle, endAngle);
		if (max(startAngle, endAngle) < g_minAllowedFingerAngle) {
			// Both ends of the edge are behind the finger. Derotate.
			angleCorrection = g_minAllowedFingerAngle - smallestAngle;

			startAngle += angleCorrection;
			endAngle += angleCorrection;
		}
		else {
			if (smallestAngle < g_minAllowedFingerAngle) {
				// If one end of the edge is behind the finger and the other isn't, then derotate only if the finger missed in the normal case.
				crossesBehind = true;
			}
		}

		bool startSmaller = startAngle < endAngle;
		float smallerAngle = startSmaller ? startAngle : endAngle;
		float largerAngle = startSmaller ? endAngle : startAngle;

		float smallerLength = startSmaller ? VectorLength(edgeStart) : VectorLength(edgeEnd);
		float largerLength = startSmaller ? VectorLength(edgeEnd) : VectorLength(edgeStart);

		Point2 smallerPt = Point2(cosf(smallerAngle), sinf(smallerAngle)) * smallerLength;
		Point2 largerPt = Point2(cosf(largerAngle), sinf(largerAngle)) * largerLength;

		float smallerRadius = -1.0f, largerRadius = -1.0f;
		SavedFingerData fingerData;
		int smallerIndex = LookupFingerByAngle(fingerIndex, smallerAngle, &fingerData);		
		smallerRadius = fingerData.fingerLength;
		smallerRadius /= scale;

		int largerIndex = LookupFingerByAngle(fingerIndex, largerAngle, &fingerData);
		largerRadius = fingerData.fingerLength;
		largerRadius /= scale;		

		if (smallerLength > smallerRadius || largerLength > largerRadius) {
			// Edge is not entirely within the curve area. Now we only care if it intersects the curve.

			//_MESSAGE("%.3f\t%.2f %.2f", VectorLength(smallerPt - largerPt), smallerAngle * 57.2958f, largerAngle * 57.2958f);

			auto &fingerVals = g_fingerCurveVals[fingerIndex];

			int end = min(largerIndex, std::size(fingerVals) - 2);
			for (int i = smallerIndex; i <= end; i++) {
				float angle = fingerVals[i].angle;
				float length = fingerVals[i].fingerLength;
				length /= scale;

				float nextAngle = fingerVals[i + 1].angle;
				float nextLength = fingerVals[i + 1].fingerLength;
				nextLength /= scale;

				Point2 currentPt = Point2(cosf(angle), sinf(angle)) * length;
				Point2 nextPt = Point2(cosf(nextAngle), sinf(nextAngle)) * nextLength;

				Point2 intersection;
				//_MESSAGE("%.3f", LineSegmentLineSegmentDistance(smallerPt, largerPt, currentPt, nextPt));
				if (LineSegmentIntersectsLineSegment(smallerPt, largerPt, currentPt, nextPt, &intersection) ||
					LineSegmentLineSegmentDistance(smallerPt, largerPt, currentPt, nextPt) <= 0.015f) {

					NiPoint3 triNormal = VectorNormalized(CrossProduct(vertex1 - vertex0, vertex2 - vertex1));

					NiPoint3 currentPtDiskspace = VectorNormalized(RotateVectorByAxisAngle(zeroAngleVector, diskNormal, angle - angleCorrection)) * length;
					NiPoint3 nextPtDiskspace = VectorNormalized(RotateVectorByAxisAngle(zeroAngleVector, diskNormal, nextAngle - angleCorrection)) * nextLength;
					NiPoint3 tangent = nextPtDiskspace - currentPtDiskspace;

					if (DotProduct(triNormal, tangent) <= 0) {
						// Front face of the triangle was intersected CCW around the circle
						// TODO: Use precise intersection point instead of the current angle / length
						_MESSAGE("pt 1: %.2f\t%.2f", angle, angleCorrection);
						outIntersectionPoint1 = diskCenter + currentPtDiskspace;
						return 1;
					}
				}
			}
		}
		else {
			// Intersection line is entirely inside the disk. Miss.
		}

		
		// If one end of the edge is behind the finger and the other isn't, then derotate only if the finger missed in the normal case.
		if (crossesBehind) {
			angleCorrection = g_minAllowedFingerAngle - smallestAngle;

			startAngle += angleCorrection;
			endAngle += angleCorrection;

			bool startSmaller = startAngle < endAngle;
			float smallerAngle = startSmaller ? startAngle : endAngle;
			float largerAngle = startSmaller ? endAngle : startAngle;

			float smallerLength = startSmaller ? VectorLength(edgeStart) : VectorLength(edgeEnd);
			float largerLength = startSmaller ? VectorLength(edgeEnd) : VectorLength(edgeStart);

			Point2 smallerPt = Point2(cosf(smallerAngle), sinf(smallerAngle)) * smallerLength;
			Point2 largerPt = Point2(cosf(largerAngle), sinf(largerAngle)) * largerLength;

			float smallerRadius = -1.0f, largerRadius = -1.0f;
			SavedFingerData fingerData;
			int smallerIndex = LookupFingerByAngle(fingerIndex, smallerAngle, &fingerData);
			smallerRadius = fingerData.fingerLength;
			smallerRadius /= scale;

			int largerIndex = LookupFingerByAngle(fingerIndex, largerAngle, &fingerData);
			largerRadius = fingerData.fingerLength;
			largerRadius /= scale;

			if (smallerLength > smallerRadius || largerLength > largerRadius) {
				// Edge is not entirely within the curve area. Now we only care if it intersects the curve.

				auto &fingerVals = g_fingerCurveVals[fingerIndex];

				int end = min(largerIndex, std::size(fingerVals) - 2);
				for (int i = smallerIndex; i <= end; i++) {
					float angle = fingerVals[i].angle;
					float length = fingerVals[i].fingerLength;
					length /= scale;

					float nextAngle = fingerVals[i + 1].angle;
					float nextLength = fingerVals[i + 1].fingerLength;
					nextLength /= scale;

					Point2 currentPt = Point2(cosf(angle), sinf(angle)) * length;
					Point2 nextPt = Point2(cosf(nextAngle), sinf(nextAngle)) * nextLength;

					Point2 intersection;
					if (LineSegmentIntersectsLineSegment(smallerPt, largerPt, currentPt, nextPt, &intersection) ||
						LineSegmentLineSegmentDistance(smallerPt, largerPt, currentPt, nextPt) <= 0.015f) {

						NiPoint3 triNormal = VectorNormalized(CrossProduct(vertex1 - vertex0, vertex2 - vertex1));

						NiPoint3 currentPtDiskspace = VectorNormalized(RotateVectorByAxisAngle(zeroAngleVector, diskNormal, angle - angleCorrection)) * length;
						NiPoint3 nextPtDiskspace = VectorNormalized(RotateVectorByAxisAngle(zeroAngleVector, diskNormal, nextAngle - angleCorrection)) * nextLength;
						NiPoint3 tangent = nextPtDiskspace - currentPtDiskspace;

						if (DotProduct(triNormal, tangent) <= 0) {
							// Front face of the triangle was intersected CCW around the circle
							// TODO: Use precise intersection point instead of the current angle / length
							_MESSAGE("derot pt 1: %.2f\t%.2f", angle, angleCorrection);
							outIntersectionPoint1 = diskCenter + currentPtDiskspace;
							return 1;
						}
					}
				}
			}
			else {
				// Intersection line is entirely inside the disk. Miss.
			}
		}

		return 0;
	}
}


void GetDiskIntersectionOnGraphicsGeometry(std::vector<Intersection> &intersections, int fingerIndex, NiAVObject *root, const NiPoint3 &center, const NiPoint3 &point1, const NiPoint3 &point2, const NiPoint3 &normal, const NiPoint3 &zeroAngleVector)
{
	BSTriShape *geom = root->GetAsBSTriShape();
	if (geom) {
		UInt16 numTris = geom->unk198;
		UInt16 numVerts = geom->numVertices;
		BSGeometryData *geomData = geom->geometryData;
		if (!geomData) {
			// Probably skinned mesh. TODO: Deal with skinned mesh
			return;
		}

		_MESSAGE("%d tris", numTris);

		auto tris = (Triangle *)geomData->triangles;
		uintptr_t verts = (uintptr_t)(geomData->vertices);

		UInt64 vertexDesc = geom->vertexDesc;
		VertexFlags vertexFlags = NiSkinPartition::GetVertexFlags(vertexDesc);
		UInt8 vertexSize = (vertexDesc & 0xF) * 4;

		if ((vertexFlags & VertexFlags::VF_VERTEX) && verts && numVerts > 0 && tris && numTris > 0) {
			UInt32 posOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_POSITION);

			NiTransform nodeTransform = geom->m_worldTransform;

			// Input transformations
			NiTransform inverseTransform;
			nodeTransform.Invert(inverseTransform);
			NiPoint3 centerInNodeSpace = inverseTransform * center;
			NiPoint3 point1InNodeSpace = inverseTransform * point1;
			NiPoint3 point2InNodeSpace = inverseTransform * point2;

			NiMatrix33 inverseRot = nodeTransform.rot.Transpose();
			NiPoint3 zeroAngleVectorNodespace = inverseRot * zeroAngleVector;
			NiPoint3 normalNodespace = inverseRot * normal;

			for (int i = 0; i < numTris; i++) {
				Triangle tri = tris[i];
				// get closest point on triangle to given point
				NiPoint3 intersectionPoint1, intersectionPoint2;
				int numIntersections = MathUtils::FingerIntersectsTriangle(fingerIndex, centerInNodeSpace, normalNodespace, zeroAngleVectorNodespace, nodeTransform.scale, tri, intersectionPoint1, intersectionPoint2, verts, vertexSize, posOffset);
				//int numIntersections = MathUtils::DiskIntersectsTriangle(centerInNodeSpace, normalNodespace, radius, tri, intersectionPoint1, intersectionPoint2, verts, vertexSize, posOffset);
				//int numIntersections = MathUtils::CircleIntersectsTriangle(centerInNodeSpace, normalNodespace, radius, tri, intersectionPoint1, intersectionPoint2, verts, vertexSize, posOffset);
				if (numIntersections == 0) {
					continue;
				}

				if (numIntersections == 2) {
					intersections.push_back({ nodeTransform * intersectionPoint1, nodeTransform * intersectionPoint2, geom, tri, true });
				}
				else { // numIntersections == 1
					intersections.push_back({ nodeTransform * intersectionPoint1, { 0, 0, 0 }, geom, tri, false });
				}
			}
		}
		return;
	}
	NiNode *node = root->GetAsNiNode();
	if (node) {
		if (node->GetAsNiSwitchNode()) {
			// NiSwitchNode: Return the first valid child
			for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
				auto child = node->m_children.m_data[i];
				if (child) {
					GetDiskIntersectionOnGraphicsGeometry(intersections, fingerIndex, child, center, point1, point2, normal, zeroAngleVector);
					return;
				}
			}
			return;
		}
		else {
			for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
				auto child = node->m_children.m_data[i];
				if (child) {
					GetDiskIntersectionOnGraphicsGeometry(intersections, fingerIndex, child, center, point1, point2, normal, zeroAngleVector);
				}
			}
			return;
		}
	}
	return;
}


NiPoint3 GetClosestPointOnIntersection(const NiPoint3 &point, const Intersection &intersection)
{
	BSTriShape *geom = intersection.node;
	Triangle tri = intersection.tri;

	BSGeometryData *geomData = geom->geometryData;

	uintptr_t verts = (uintptr_t)(geomData->vertices);

	UInt64 vertexDesc = geom->vertexDesc;
	UInt8 vertexSize = (vertexDesc & 0xF) * 4;

	UInt32 posOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_POSITION);

	NiTransform nodeTransform = geom->m_worldTransform;

	MathUtils::Result result = MathUtils::GetClosestPointOnTriangle(point, tri, verts, vertexSize, posOffset);

	return nodeTransform * result.closest;
}

std::array<NiPoint3, 3> GetVertices(const Intersection &intersection)
{
	BSTriShape *geom = intersection.node;
	Triangle tri = intersection.tri;

	BSGeometryData *geomData = geom->geometryData;

	uintptr_t verts = (uintptr_t)(geomData->vertices);

	UInt64 vertexDesc = geom->vertexDesc;
	UInt8 vertexSize = (vertexDesc & 0xF) * 4;

	UInt32 posOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_POSITION);

	NiTransform nodeTransform = geom->m_worldTransform;

	// get vertices in world space
	uintptr_t vert = (verts + tri.vertexIndices[0] * vertexSize);
	NiPoint3 pos0 = nodeTransform * *(NiPoint3 *)(vert + posOffset);
	vert = (verts + tri.vertexIndices[1] * vertexSize);
	NiPoint3 pos1 = nodeTransform * *(NiPoint3 *)(vert + posOffset);
	vert = (verts + tri.vertexIndices[2] * vertexSize);
	NiPoint3 pos2 = nodeTransform * *(NiPoint3 *)(vert + posOffset);

	return { pos0, pos1, pos2 };
}

bool DoesAnyVertexMatch(const std::array<NiPoint3, 3> &verts1, const std::array<NiPoint3, 3> &verts2)
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (VectorLengthSquared(verts1[i] - verts2[j]) < 0.00001f) {
				return true;
			}
		}
	}
	return false;
}

bool TriangleIntersectsTriangle(const std::array<NiPoint3, 3> &verts1, const std::array<NiPoint3, 3> &verts2)
{
	float *p1 = (float *)&verts1[0];
	float *q1 = (float *)&verts1[1];
	float *r1 = (float *)&verts1[2];

	float *p2 = (float *)&verts2[0];
	float *q2 = (float *)&verts2[1];
	float *r2 = (float *)&verts2[2];

	return tri_tri_overlap_test_3d(p1, q1, r1, p2, q2, r2);
}

float TriangleTriangleDistance(const std::array<NiPoint3, 3> &verts1, const std::array<NiPoint3, 3> &verts2)
{
	float P[3], Q[3];
	auto S = (float(*)[3])&verts1;
	auto T = (float(*)[3])&verts2;
	return TriDist(P, Q, S, T);
}

void _VisitGraphNodes(std::unordered_map<Intersection *, std::unordered_set<Intersection *>> &graph, Intersection *intersection, std::function<void(Intersection *)> visitor, std::unordered_set<Intersection *> &visited)
{
	visitor(intersection);
	visited.insert(intersection);

	if (graph.count(intersection) == 0) {
		return;
	}

	for (Intersection *edge : graph[intersection]) {
		if (visited.count(edge) == 0) {
			_VisitGraphNodes(graph, edge, visitor, visited);
		}
	}
}

void VisitGraphNodes(std::unordered_map<Intersection *, std::unordered_set<Intersection *>> &graph, Intersection *intersection, std::function<void(Intersection *)> visitor)
{
	std::unordered_set<Intersection *> visited;
	_VisitGraphNodes(graph, intersection, visitor, visited);
}

bool GetIntersections(NiAVObject *root, int fingerIndex, const NiPoint3 &center, const NiPoint3 &point1, const NiPoint3 &point2, const NiPoint3 &normal, const NiPoint3 &zeroAngleVector, const NiPoint3 &palmPos, const NiPoint3 &palmDirection,
	NiPoint3 *closestPos)
{
	std::vector<Intersection> intersections;

	// Populate intersections
	GetDiskIntersectionOnGraphicsGeometry(intersections, fingerIndex, root, center, point1, point2, normal, zeroAngleVector);

	if (intersections.size() == 0) {
		return false;
	}

	// Construct a graph where V = all intersected triangles, and E = connections of adjacent triangles

	std::unordered_map<Intersection *, std::unordered_set<Intersection *>> graph; // adjacency list: map intersection -> intersections adjacent to it

	for (int i = 0; i < intersections.size(); i++) {
		Intersection *intersection = &intersections[i];

		std::array<NiPoint3, 3> vertices = GetVertices(*intersection);

		for (int j = i + 1; j < intersections.size(); j++) {
			Intersection *otherIntersection = &intersections[j];

			std::array<NiPoint3, 3> otherVertices = GetVertices(*otherIntersection);

			//if (DoesAnyVertexMatch(vertices, otherVertices) || TriangleIntersectsTriangle(vertices, otherVertices) || TriangleTriangleDistance(vertices, otherVertices) < 1.5f) {
				// Add an edge between these two intersections
				if (graph.count(intersection) == 0) {
					graph[intersection] = std::unordered_set<Intersection *>();
				}
				graph[intersection].insert(otherIntersection);

				if (graph.count(otherIntersection) == 0) {
					graph[otherIntersection] = std::unordered_set<Intersection *>();
				}
				graph[otherIntersection].insert(intersection);
			//}
		}
	}

	// Now find the closest intersection to the finger root

	Intersection *closestIntersection = nullptr;
	float closestDist = (std::numeric_limits<float>::max)();

	NiPoint3 curlDirection = CrossProduct(normal, zeroAngleVector);
	for (Intersection &intersection : intersections) {
		NiPoint3 pt = GetClosestPointOnIntersection(center, intersection);
		float dist = VectorLengthSquared(pt - center);

		if (dist < closestDist) {
			std::array<NiPoint3, 3> verts = GetVertices(intersection);

			NiPoint3 triNormal = VectorNormalized(CrossProduct(verts[1] - verts[0], verts[2] - verts[1]));

			//if (DotProduct(triNormal, curlDirection) <= 0) {
				// Front face of the triangle faces the line
				closestDist = dist;
				closestIntersection = &intersection;
			//}
		}
	}

	if (!closestIntersection) {
		return false;
	}

	// Now go through the connected component of the above, and pick the one with the largest radius
	// Allow backfaces when traversing the connected component, but only choose a final triangle if it is a front face

	float smallestEnergy = (std::numeric_limits<float>::max)();
	NiPoint3 furthestIntersection;
	auto visit = [fingerIndex, &furthestIntersection, &smallestEnergy, &center, &normal, &zeroAngleVector, &point1, &point2](Intersection *intersection) {
		NiPoint3 centerToIntersect = intersection->pt - center;
		float dist;
		NiPoint3 pt;
		if (intersection->hasPt2) {
			// Both pts have the same dist, so choose based on smaller angle
			float angle1 = acosf(DotProduct(VectorNormalized(intersection->pt - center), zeroAngleVector));
			float angle2 = acosf(DotProduct(VectorNormalized(intersection->pt2 - center), zeroAngleVector));

			if (angle1 <= angle2) {
				dist = VectorLength(intersection->pt - center);
				pt = intersection->pt;
			}
			else {
				dist = VectorLength(intersection->pt2 - center);
				pt = intersection->pt2;
			}
		}
		else {
			dist = VectorLength(centerToIntersect);
			pt = intersection->pt;
		}

		centerToIntersect = pt - center;
		float angle = acosf(DotProduct(VectorNormalized(centerToIntersect), zeroAngleVector));

		float degrees = angle * 57.2958f;

		SavedFingerData fingerData;
		LookupFingerByAngle(fingerIndex, angle, &fingerData);
		float radius = fingerData.fingerLength;
		float radiusRatio = 1.0f - VectorLength(centerToIntersect) / radius;

		_MESSAGE("radius ratio: %.3f", radiusRatio);
		_MESSAGE("angle: %.2f", degrees);

		float energy = radiusRatio * 0.7f + (degrees / 180.0f) * 0.3f;

		if (energy < smallestEnergy) {
			std::array<NiPoint3, 3> verts = GetVertices(*intersection);

			NiPoint3 triNormal = VectorNormalized(CrossProduct(verts[1] - verts[0], verts[2] - verts[1]));

			NiPoint3 tangentAtIntersection = CrossProduct(normal, centerToIntersect);
			//if (DotProduct(triNormal, tangentAtIntersection) <= 0) {
				// Front face of the triangle was intersected CCW around the circle
				smallestEnergy = energy;
				furthestIntersection = pt;
			//}
		}
	};
	VisitGraphNodes(graph, closestIntersection, visit);

	if (smallestEnergy == (std::numeric_limits<float>::max)()) {
		// No valid tris from the ones in the connected component
		return false;
	}

	*closestPos = furthestIntersection;
	return true;
}


bool GetClosestPointOnGraphicsGeometry(NiAVObject *root, const NiPoint3 &point, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar)
{
	BSTriShape *geom = root->GetAsBSTriShape();
	if (geom) {
		UInt16 numTris = geom->unk198;
		UInt16 numVerts = geom->numVertices;
		BSGeometryData *geomData = geom->geometryData;
		if (!geomData) {
			// Probably skinned mesh. TODO: Deal with skinned mesh
			return false;
		}

		_MESSAGE("%d tris", numTris);

		auto tris = (Triangle *)geomData->triangles;
		uintptr_t verts = (uintptr_t)(geomData->vertices);

		UInt64 vertexDesc = geom->vertexDesc;
		VertexFlags vertexFlags = NiSkinPartition::GetVertexFlags(vertexDesc);
		UInt8 vertexSize = (vertexDesc & 0xF) * 4;

		if ((vertexFlags & VertexFlags::VF_VERTEX) && verts && numVerts > 0 && tris && numTris > 0) {
			UInt32 posOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_POSITION);

			NiTransform inverseTransform;
			geom->m_worldTransform.Invert(inverseTransform);
			NiPoint3 pointInNodeSpace = inverseTransform * point;

			int closestTri = -1;
			NiPoint3 closestTriPos;
			float closestDistance = *closestDistanceSoFar;

			for (int i = 0; i < numTris; i++) {
				Triangle tri = tris[i];
				// get closest point on triangle to given point
				MathUtils::Result result = MathUtils::GetClosestPointOnTriangle(pointInNodeSpace, tri, verts, vertexSize, posOffset);
				float distance = result.sqrDistance;
				if (distance < closestDistance) {
					closestDistance = distance;
					closestTriPos = result.closest;
					closestTri = i;
				}
			}

			if (closestTri >= 0) {
				*closestDistanceSoFar = closestDistance;
				*closestPos = geom->m_worldTransform * closestTriPos;

				Triangle closestTriangle = tris[closestTri];
				uintptr_t vert = (verts + closestTriangle.vertexIndices[0] * vertexSize);
				NiPoint3 pos0 = geom->m_worldTransform * *(NiPoint3 *)(vert + posOffset);
				vert = (verts + closestTriangle.vertexIndices[1] * vertexSize);
				NiPoint3 pos1 = geom->m_worldTransform * *(NiPoint3 *)(vert + posOffset);
				vert = (verts + closestTriangle.vertexIndices[2] * vertexSize);
				NiPoint3 pos2 = geom->m_worldTransform * *(NiPoint3 *)(vert + posOffset);

				*closestNormal = VectorNormalized(CrossProduct(pos1 - pos0, pos2 - pos1));

				return true;
			}
		}
		return false;
	}
	NiNode *node = root->GetAsNiNode();
	if (node) {
		if (node->GetAsNiSwitchNode()) {
			// NiSwitchNode: Return the first valid child
			for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
				auto child = node->m_children.m_data[i];
				if (child) {
					return GetClosestPointOnGraphicsGeometry(child, point, closestPos, closestNormal, closestDistanceSoFar);
				}
			}
		}
		else {
			bool success = false;
			for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
				auto child = node->m_children.m_data[i];
				if (child) {
					if (GetClosestPointOnGraphicsGeometry(child, point, closestPos, closestNormal, closestDistanceSoFar)) {
						success = true;
					}
				}
			}
			return success;
		}
	}
	return false;
}

bool GetClosestPointOnGraphicsGeometryToLine(NiAVObject *root, const NiPoint3 &point, const NiPoint3 &direction, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar)
{
	BSTriShape *geom = root->GetAsBSTriShape();
	if (geom) {
		UInt16 numTris = geom->unk198;
		UInt16 numVerts = geom->numVertices;
		BSGeometryData *geomData = geom->geometryData;
		if (!geomData) {
			// Probably skinned mesh. TODO: Deal with skinned mesh
			return false;
		}

		_MESSAGE("%d tris", numTris);

		auto tris = (Triangle *)geomData->triangles;
		uintptr_t verts = (uintptr_t)(geomData->vertices);

		UInt64 vertexDesc = geom->vertexDesc;
		VertexFlags vertexFlags = NiSkinPartition::GetVertexFlags(vertexDesc);
		UInt8 vertexSize = (vertexDesc & 0xF) * 4;

		if ((vertexFlags & VertexFlags::VF_VERTEX) && verts && numVerts > 0 && tris && numTris > 0) {
			UInt32 posOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_POSITION);

			NiTransform inverseTransform;
			geom->m_worldTransform.Invert(inverseTransform);
			NiPoint3 pointInNodeSpace = inverseTransform * point;
			NiPoint3 directionInNodeSpace = geom->m_worldTransform.rot.Transpose() * direction;

			int closestTri = -1;
			NiPoint3 closestTriPos;
			float closestDistance = *closestDistanceSoFar;

			float lateralWeight = Config::options.grabLateralWeight;
			float directionalWeight = Config::options.grabDirectionalWeight;

			for (int i = 0; i < numTris; i++) {
				Triangle tri = tris[i];
				// get closest point on the triangle to given ray starting at point in direction
				NiPoint3 closestPoint;
				float closestDistSquared;
				bool intersects;
				MathUtils::GetClosestPointOnTriangleToLine(pointInNodeSpace, directionInNodeSpace, tri, closestPoint, closestDistSquared, intersects, verts, vertexSize, posOffset);
				NiPoint3 pointToClosest = closestPoint - pointInNodeSpace;
				NiPoint3 pointToClosestAlongDirection = directionInNodeSpace * DotProduct(pointToClosest, directionInNodeSpace);
				float directionalDistance = VectorLength(pointToClosestAlongDirection);
				float lateralDistance = VectorLength(pointToClosest - pointToClosestAlongDirection);
				float distance = directionalWeight * directionalDistance*directionalDistance + lateralWeight * lateralDistance*lateralDistance;
				if (distance < closestDistance) {
					uintptr_t vert = (verts + tri.vertexIndices[0] * vertexSize);
					NiPoint3 pos0 = *(NiPoint3 *)(vert + posOffset);
					vert = (verts + tri.vertexIndices[1] * vertexSize);
					NiPoint3 pos1 = *(NiPoint3 *)(vert + posOffset);
					vert = (verts + tri.vertexIndices[2] * vertexSize);
					NiPoint3 pos2 = *(NiPoint3 *)(vert + posOffset);

					NiPoint3 triNormal = VectorNormalized(CrossProduct(pos1 - pos0, pos2 - pos1));

					if (DotProduct(triNormal, directionInNodeSpace) <= 0) {
						// Front face of the triangle faces the line
						closestDistance = distance;
						closestTriPos = closestPoint;
						closestTri = i;
					}
				}
			}

			if (closestTri >= 0) {
				*closestDistanceSoFar = closestDistance;
				*closestPos = geom->m_worldTransform * closestTriPos;

				Triangle closestTriangle = tris[closestTri];
				uintptr_t vert = (verts + closestTriangle.vertexIndices[0] * vertexSize);
				NiPoint3 pos0 = geom->m_worldTransform * *(NiPoint3 *)(vert + posOffset);
				vert = (verts + closestTriangle.vertexIndices[1] * vertexSize);
				NiPoint3 pos1 = geom->m_worldTransform * *(NiPoint3 *)(vert + posOffset);
				vert = (verts + closestTriangle.vertexIndices[2] * vertexSize);
				NiPoint3 pos2 = geom->m_worldTransform * *(NiPoint3 *)(vert + posOffset);

				*closestNormal = VectorNormalized(CrossProduct(pos1 - pos0, pos2 - pos1));

				return true;
			}
		}
		return false;
	}
	NiNode *node = root->GetAsNiNode();
	if (node) {
		if (node->GetAsNiSwitchNode()) {
			// NiSwitchNode: Return the first valid child
			for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
				auto child = node->m_children.m_data[i];
				if (child) {
					return GetClosestPointOnGraphicsGeometryToLine(child, point, direction, closestPos, closestNormal, closestDistanceSoFar);
				}
			}
		}
		else {
			bool success = false;
			for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
				auto child = node->m_children.m_data[i];
				if (child) {
					if (GetClosestPointOnGraphicsGeometryToLine(child, point, direction, closestPos, closestNormal, closestDistanceSoFar)) {
						success = true;
					}
				}
			}
			return success;
		}
	}
	return false;
}