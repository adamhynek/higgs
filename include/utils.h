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
NiPoint3 MatrixToEulerAngles(NiMatrix33 &m);

NiAVObject * GetHighestParent(NiAVObject *node);
void updateTransformTree(NiAVObject * root);

float GetActorInverseMass(Actor *actor);
NiAVObject * GetTorsoNode(Actor *actor);

UInt32 GetFullFormID(const ModInfo * modInfo, UInt32 formLower);

bool IsSelectable(TESForm *form);
bool IsAllowedCollidable(hkpCollidable *collidable);

bool IsTwoHanded(TESObjectWEAP *weap);
bool IsBow(TESObjectWEAP *weap);
std::pair<bool, bool> AreEquippedItemsValid(Actor *actor);

long long GetTime();

void PrintVector(NiPoint3 &p);
