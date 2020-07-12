#pragma once

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"
#include "common/ITimer.h"
#include "RE/havok.h"


#define VM_REGISTRY (*g_skyrimVM)->GetClassRegistry()


extern ITimer g_timer;
extern double g_currentFrameTime;
//extern double g_deltaTime;


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

NiAVObject * GetHighestParent(NiAVObject *node);
void updateTransformTree(NiAVObject * root, NiAVObject::ControllerUpdateContext *ctx);

NiAVObject * GetTorsoNode(Actor *actor);

UInt32 GetFullFormID(const ModInfo * modInfo, UInt32 formLower);

bool IsAllowedCollidable(const hkpCollidable *collidable);

bool IsTwoHanded(const TESObjectWEAP *weap);
bool IsBow(const TESObjectWEAP *weap);
std::pair<bool, bool> AreEquippedItemsValid(Actor *actor);

double GetTime();

void PrintVector(const NiPoint3 &p);
void PrintSceneGraph(NiAVObject *node);
void PrintToFile(std::string entry, std::string filename);

//float hkHalfToFloat(hkHalf half);
//hkHalf floatToHkHalf(float half);

bool DoesNodeHaveNode(NiAVObject *haystack, NiAVObject *target);
bool DoesRefrHaveNode(TESObjectREFR *ref, NiAVObject *node);
bool IsNodeWithinArmor(NiAVObject *armorNode, NiAVObject *target);

void ClearCollisionMap();
UInt32 GetSavedCollision(UInt32 id);
UInt32 GetSavedCollisionRefCount(UInt32 id);
void RemoveSavedCollision(UInt32 id);
void SetCollisionInfoForAllCollisionInRefr(TESObjectREFR *refr, UInt32 collisionGroup);
void SetCollisionInfoDownstream(NiAVObject *obj, UInt32 collisionGroup);
void ResetCollisionInfoForAllCollisionInRefr(TESObjectREFR *refr, hkpCollidable *skipNode = nullptr);
void ResetCollisionInfoDownstream(NiAVObject *obj, hkpCollidable *skipNode = nullptr);

typedef void(*_RemoveItem)(TESObjectREFR *_this, UInt32 *outHandle, TESBoundObject* a_item, SInt32 a_count, UInt32 a_reason, BaseExtraList* a_extraList, TESObjectREFR* a_moveToRef, const NiPoint3* a_dropLoc, const NiPoint3* a_rotate);

typedef void(*_Update3DPosition)(TESObjectREFR *_this, bool warp);

struct Triangle
{
	UInt16 vertexIndices[3];
};
static_assert(sizeof(Triangle) == 0x06);


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

bool GetClosestPointOnGraphicsGeometry(NiAVObject *root, NiPoint3 point, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar);

void SetVelocityDownstream(NiAVObject *obj, hkVector4 velocity);
void SetVelocityForAllCollisionInRefr(TESObjectREFR *refr, hkVector4 velocity);
