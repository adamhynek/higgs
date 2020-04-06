#pragma once

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"
#include "RE/havok.h"


#define VM_REGISTRY (*g_skyrimVM)->GetClassRegistry()


float VectorLength(NiPoint3 vec);
NiPoint3 VectorNormalized(NiPoint3 vec);
float DotProduct(NiPoint3 vec1, NiPoint3 vec2);
NiPoint3 CrossProduct(NiPoint3 vec1, NiPoint3 vec2);
NiMatrix33 MatrixFromAxisAngle(NiPoint3 axis, float theta);
NiPoint3 MatrixToEulerAngles(const NiMatrix33 &m);

NiAVObject * GetHighestParent(NiAVObject *node);
void updateTransformTree(NiAVObject * root);

NiAVObject * GetTorsoNode(Actor *actor);

UInt32 GetFullFormID(const ModInfo * modInfo, UInt32 formLower);

bool IsSelectable(const TESForm *form);
bool IsAllowedCollidable(const hkpCollidable *collidable);

bool IsTwoHanded(const TESObjectWEAP *weap);
bool IsBow(const TESObjectWEAP *weap);
std::pair<bool, bool> AreEquippedItemsValid(Actor *actor);

long long GetTime();

void PrintVector(const NiPoint3 &p);
void PrintSceneGraph(NiAVObject *node);
