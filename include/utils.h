#pragma once

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"


float VectorLength(NiPoint3 vec);

NiPoint3 VectorNormalized(NiPoint3 vec);

float DotProduct(NiPoint3 vec1, NiPoint3 vec2);

NiPoint3 CrossProduct(NiPoint3 vec1, NiPoint3 vec2);

NiMatrix33 MatrixFromAxisAngle(NiPoint3 axis, float theta);

NiAVObject * GetHighestParent(NiAVObject *node);

UInt32 GetFullFormID(const ModInfo * modInfo, UInt32 formLower);

bool IsSelectable(TESForm *form);
