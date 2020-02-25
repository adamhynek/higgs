#include "utils.h"


float VectorLength(NiPoint3 vec)
{
	return sqrtf(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

NiPoint3 VectorNormalized(NiPoint3 vec)
{
	float length = VectorLength(vec);
	return length ? vec / length : NiPoint3(0, 0, 0);
}

float DotProduct(NiPoint3 vec1, NiPoint3 vec2)
{
	return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
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

NiAVObject * GetHighestParent(NiAVObject *node)
{
	if (!node->m_parent) {
		return node;
	}
	return GetHighestParent(node->m_parent);
}

UInt32 GetFullFormID(const ModInfo * modInfo, UInt32 formLower)
{
	return (modInfo->modIndex << 24) | formLower;
}

bool IsSelectable(TESForm *form)
{
	switch (form->formType)
	{
	case kFormType_Weapon:
	case kFormType_Misc:
	case kFormType_Ingredient:
	case kFormType_Armor:
	case kFormType_Ammo:
	case kFormType_Book:
	case kFormType_ScrollItem:
	case kFormType_Potion:
	case kFormType_SoulGem:
	case kFormType_Key: // unverified - TODO
	case kFormType_Projectile: // For arrows stuck in i.e. a wall. Requires some fiddling to actually interact with physics.
	//case kFormType_Light: // Works for torch, but don't want arbitrary lights to be selectable
		return true;
	default:
		return false;
	}
}
