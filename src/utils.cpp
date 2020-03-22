#include <chrono>
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

NiPoint3 MatrixToEulerAngles(NiMatrix33 &m)
{
	NiPoint3 output(0, 0, 0);
	float fY = asin(((float)(SInt32)(-m.arr[2] * 1000000)) / 1000000);
	float fCY = cos(fY);
	float fCYTest = ((float)(SInt32)(fCY * 100)) / 100;
	float fTX, fTY;
	if (fCY && abs(fCY) >= 0.00000011920929 && fCYTest) {
		fTX = m.arr[8] / fCY;
		fTY = m.arr[5] / fCY;
		output.x = atan2(fTY, fTX);
		fTX = m.arr[0] / fCY;
		fTY = m.arr[1] / fCY;
		output.z = atan2(fTY, fTX);
	}
	else {
		output.x = 0;
		fTX = m.arr[4]; // Setting X to zero simplifies this element to: 0*sinY*sinZ + 1*cosZ
		fTY = m.arr[3]; // Setting X to zero simplifies this element to: 0*sinY*cosZ - 1*sinZ
		output.z = -atan2(fTY, fTX); // atan(sinZ/cosZ)
	}
	output.y = fY;
	return output;
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

// This function is currently unused, in favor of just selecting by motion type
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
	//case kFormType_MovableStatic: - a lot of them work, but stuff like campfires are not actually movable
	case kFormType_Key: // unverified - TODO
	case kFormType_Projectile: // Arrows stuck in a wall, projectiles mid air...
	//case kFormType_Light: // Torch, but don't want arbitrary lights to be selectable
	//case kFormType_Flora: // Coin bags, but also mushrooms and stuff you can't actually move
		return true;
	default:
		return false;
	}
}

bool IsAllowedCollidable(hkpCollidable *collidable)
{
	auto motion = reinterpret_cast<hkpMotion *>((UInt64)collidable->m_motion - offsetof(hkpMotion, m_motionState));
	// Motion types we allow are: Dynamic, SphereInertia, BoxInertia, ThinBoxInertia
	return (motion->m_motionType == 1 || motion->m_motionType == 2 || motion->m_motionType == 3 || motion->m_motionType == 6);
}

NiAVObject * GetTorsoNode(Actor *actor)
{
	TESRace *race = actor->race;
	BGSBodyPartData *partData = race->bodyPartData;
	if (partData) {
		auto torsoData = partData->part[0];
		if (torsoData && torsoData->unk08.data) {
			NiAVObject *actorNode = actor->GetNiNode();
			if (actorNode) {
				NiAVObject *torsoNode = actorNode->GetObjectByName(&torsoData->unk08.data);
				if (torsoNode) {
					return torsoNode;
				}
			}
		}
	}
	return nullptr;
}

void updateTransformTree(NiAVObject * root)
{
	NiAVObject::ControllerUpdateContext ctx;
	ctx.flags = 0; // set to 1 if you want to force it to update old world transforms again but you probably shouldnt
	ctx.delta = 0;

	root->UpdateWorldData(&ctx);

	auto node = root->GetAsNiNode();

	if (node)
	{
		for (int i = 0; i < node->m_children.m_arrayBufLen; ++i)
		{
			auto child = node->m_children.m_data[i];
			if (child) updateTransformTree(child);
		}
	}
}

float GetActorInverseMass(Actor *actor)
{
	TESRace *race = actor->race;
	if (race && race->bodyPartData && race->bodyPartData->part[0] && race->bodyPartData->part[0]->unk00) {
		BSFixedString nodeWithMassName = race->bodyPartData->part[0]->unk00;
		NiAVObject *nodeWithMass = actor->loadedState->node->GetObjectByName(&nodeWithMassName.data);
		if (nodeWithMass) {
			auto collWithMass = (bhkCollisionObject *)nodeWithMass->unk040;
			if (collWithMass) {
				return collWithMass->body->hkBody->motion.m_inertiaAndMassInv.w;
			}
		}
	}
	return -1.0f;
}

long long GetTime()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void PrintVector(NiPoint3 &p)
{
	_MESSAGE("%.2f, %.2f, %.2f", p.x, p.y, p.z);
}
