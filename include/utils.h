#pragma once

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"
#include "common/ITimer.h"
#include "RE/havok.h"


#define VM_REGISTRY (*g_skyrimVM)->GetClassRegistry()


extern ITimer g_timer;
extern double g_currentFrameTime;
extern double g_deltaTime;


inline float VectorLength(NiPoint3 vec) { return sqrtf(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z); }
inline float DotProduct(NiPoint3 vec1, NiPoint3 vec2) { return vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z; }
NiPoint3 VectorNormalized(NiPoint3 vec);
NiPoint3 CrossProduct(NiPoint3 vec1, NiPoint3 vec2);
NiMatrix33 MatrixFromAxisAngle(NiPoint3 axis, float theta);
float Determinant33(const NiMatrix33 &m);
NiPoint3 QuadraticFromPoints(NiPoint2 p1, NiPoint2 p2, NiPoint2 p3);

NiAVObject * GetHighestParent(NiAVObject *node);
void updateTransformTree(NiAVObject * root);

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

float hkHalfToFloat(hkHalf half);
hkHalf floatToHkHalf(float half);
