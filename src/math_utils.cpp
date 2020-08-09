#include "math_utils.h"
#include "utils.h"
#include "config.h"

#include "skse64/NiGeometry.h"


NiPoint3 VectorNormalized(NiPoint3 vec)
{
	float length = VectorLength(vec);
	return length ? vec / length : NiPoint3();
}

NiPoint3 CrossProduct(NiPoint3 vec1, NiPoint3 vec2)
{
	NiPoint3 result;
	result.x = vec1.y * vec2.z - vec1.z * vec2.y;
	result.y = vec1.z * vec2.x - vec1.x * vec2.z;
	result.z = vec1.x * vec2.y - vec1.y * vec2.x;
	return result;
}

NiMatrix33 MatrixFromAxisAngle(NiPoint3 axis, float theta)
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

NiPoint3 MatrixToEuler(NiMatrix33 &mat)
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

NiMatrix33 EulerToMatrix(NiPoint3 euler)
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

NiPoint3 RotateVectorByAxisAngle(NiPoint3 vector, NiPoint3 axis, float angle)
{
	// Rodrigues' rotation formula
	return vector * cosf(angle) + (CrossProduct(axis, vector) * sinf(angle)) + axis * DotProduct(axis, vector) * (1 - cosf(angle));
}

void NiMatrixToHkMatrix(NiMatrix33 &niMat, hkMatrix3 &hkMat)
{
	hkMat.setCols({ niMat.data[0][0], niMat.data[1][0], niMat.data[2][0], 0 },
		{ niMat.data[0][1], niMat.data[1][1], niMat.data[2][1], 0 },
		{ niMat.data[0][2], niMat.data[1][2], niMat.data[2][2], 0 });
}

void HkMatrixToNiMatrix(hkMatrix3 &hkMat, NiMatrix33 &niMat)
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

NiPoint3 QuadraticFromPoints(NiPoint2 p1, NiPoint2 p2, NiPoint2 p3)
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


namespace MathUtils
{
	Result GetClosestPointOnTriangle(NiPoint3 const& point, Triangle const& triangle, uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset)
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

	bool RayIntersectsTriangle(NiPoint3 rayOrigin,
		NiPoint3 rayVector,
		Triangle &triangle,
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

	bool GetClosestPointOnTriangleToLine(NiPoint3 rayOrigin,
		NiPoint3 rayVector,
		Triangle &triangle,
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

	bool LinePlaneIntersection(NiPoint3& contact, NiPoint3 ray, NiPoint3 rayOrigin,
		NiPoint3 normal, NiPoint3 coord) {
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

	bool PlaneIntersectsLineSegment(NiPoint3 planePoint, NiPoint3 planeNormal, NiPoint3 segmentStart, NiPoint3 segmentFinish, NiPoint3 &outPoint)
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
	int CircleIntersectsTriangle(NiPoint3 circleCenter,
		NiPoint3 circleNormal,
		float circleRadius,
		Triangle &triangle,
		NiPoint3 &outIntersectionPoint1,
		NiPoint3 &outIntersectionPoint2,
		uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset)
	{
		const float EPSILON = 0.0000001;

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
			return false;
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
}


bool GetCircleIntersectionOnGraphicsGeometry(NiAVObject *root, NiPoint3 center, NiPoint3 point1, NiPoint3 point2, NiPoint3 normal, NiPoint3 zeroAngleVector,
	NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar)
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

			// Compute radius after transforming points, as things can be scaled up/down
			float radius = VectorLength(point2InNodeSpace - point1InNodeSpace) + VectorLength(point1InNodeSpace - centerInNodeSpace); // Add em up as if the finger was straightned out
			_MESSAGE("r: %.2f", radius);

			int closestTri = -1;
			NiPoint3 closestTriPos;
			float closestDistance = *closestDistanceSoFar;

			for (int i = 0; i < numTris; i++) {
				Triangle tri = tris[i];
				// get closest point on triangle to given point
				NiPoint3 intersectionPoint1, intersectionPoint2;
				int numIntersections = MathUtils::CircleIntersectsTriangle(centerInNodeSpace, normalNodespace, radius, tri, intersectionPoint1, intersectionPoint2, verts, vertexSize, posOffset);
				if (numIntersections == 0) {
					continue;
				}

				NiPoint3 intersectionPoint;
				NiPoint3 centerToIntersect;
				float angle;
				if (numIntersections == 2) {
					// Pick the point with the smaller angle
					NiPoint3 centerToIntersect1 = intersectionPoint1 - centerInNodeSpace;
					float angle1 = acosf(DotProduct(VectorNormalized(centerToIntersect1), zeroAngleVectorNodespace));
					NiPoint3 centerToIntersect2 = intersectionPoint2 - centerInNodeSpace;
					float angle2 = acosf(DotProduct(VectorNormalized(centerToIntersect2), zeroAngleVectorNodespace));
					bool angle1Smaller = angle1 < angle2;
					intersectionPoint = angle1Smaller ? intersectionPoint1 : intersectionPoint2;
					angle = angle1Smaller ? angle1 : angle2;
					centerToIntersect = angle1Smaller ? centerToIntersect1 : centerToIntersect2;
				}
				else { // numIntersections == 1
					intersectionPoint = intersectionPoint1;
					centerToIntersect = intersectionPoint - centerInNodeSpace;
					angle = acosf(DotProduct(VectorNormalized(centerToIntersect), zeroAngleVectorNodespace));
				}
				
				if (angle < closestDistance) {
					uintptr_t vert = (verts + tri.vertexIndices[0] * vertexSize);
					NiPoint3 pos0 = *(NiPoint3 *)(vert + posOffset);
					vert = (verts + tri.vertexIndices[1] * vertexSize);
					NiPoint3 pos1 = *(NiPoint3 *)(vert + posOffset);
					vert = (verts + tri.vertexIndices[2] * vertexSize);
					NiPoint3 pos2 = *(NiPoint3 *)(vert + posOffset);

					NiPoint3 triNormal = VectorNormalized(CrossProduct(pos1 - pos0, pos2 - pos1));

					NiPoint3 tangentAtIntersection = CrossProduct(normalNodespace, centerToIntersect);
					if (DotProduct(triNormal, tangentAtIntersection) <= 0) {
						// Front face of the triangle was intersected CCW around the circle
						closestDistance = angle;
						closestTriPos = intersectionPoint;
						closestTri = i;
					}
				}
			}

			if (closestTri >= 0) {
				*closestDistanceSoFar = closestDistance;
				*closestPos = nodeTransform * closestTriPos;

				Triangle closestTriangle = tris[closestTri];
				uintptr_t vert = (verts + closestTriangle.vertexIndices[0] * vertexSize);
				NiPoint3 pos0 = nodeTransform * *(NiPoint3 *)(vert + posOffset);
				vert = (verts + closestTriangle.vertexIndices[1] * vertexSize);
				NiPoint3 pos1 = nodeTransform * *(NiPoint3 *)(vert + posOffset);
				vert = (verts + closestTriangle.vertexIndices[2] * vertexSize);
				NiPoint3 pos2 = nodeTransform * *(NiPoint3 *)(vert + posOffset);

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
					return GetCircleIntersectionOnGraphicsGeometry(child, center, point1, point2, normal, zeroAngleVector, closestPos, closestNormal, closestDistanceSoFar);
				}
			}
		}
		else {
			bool success = false;
			for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
				auto child = node->m_children.m_data[i];
				if (child) {
					if (GetCircleIntersectionOnGraphicsGeometry(child, center, point1, point2, normal, zeroAngleVector, closestPos, closestNormal, closestDistanceSoFar)) {
						success = true;
					}
				}
			}
			return success;
		}
	}
	return false;
}

bool GetClosestPointOnGraphicsGeometry(NiAVObject *root, NiPoint3 point, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar)
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

bool GetClosestPointOnGraphicsGeometryToLine(NiAVObject *root, NiPoint3 point, NiPoint3 direction, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar)
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