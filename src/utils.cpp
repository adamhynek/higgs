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
