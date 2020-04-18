#include <chrono>
#include <list>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <regex>

#include "skse64/GameRTTI.h"
#include "skse64/NiExtraData.h"

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

NiPoint3 MatrixToEulerAngles(const NiMatrix33 &m)
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

bool IsAllowedCollidable(const hkpCollidable *collidable)
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

	if (node) {
		for (int i = 0; i < node->m_children.m_arrayBufLen; ++i) {
			auto child = node->m_children.m_data[i];
			if (child) updateTransformTree(child);
		}
	}
}

bool IsTwoHanded(const TESObjectWEAP *weap)
{
	switch (weap->gameData.type) {
	case TESObjectWEAP::GameData::kType_2HA:
	case TESObjectWEAP::GameData::kType_2HS:
	case TESObjectWEAP::GameData::kType_CBow:
	case TESObjectWEAP::GameData::kType_CrossBow:
	case TESObjectWEAP::GameData::kType_Staff:
	case TESObjectWEAP::GameData::kType_Staff2:
	case TESObjectWEAP::GameData::kType_TwoHandAxe:
	case TESObjectWEAP::GameData::kType_TwoHandSword:
		return true;
	default:
		return false;
	}
}

bool IsBow(const TESObjectWEAP *weap)
{
	UInt8 type = weap->gameData.type;
	return (type == TESObjectWEAP::GameData::kType_Bow || type == TESObjectWEAP::GameData::kType_Bow2);
}

std::pair<bool, bool> AreEquippedItemsValid(Actor *actor)
{
	if (!actor->actorState.IsWeaponDrawn()) {
		return std::make_pair(true, true);
	}

	TESForm *mainhandItem = actor->GetEquippedObject(false);
	TESForm *offhandItem = actor->GetEquippedObject(true);

	bool isMainValid = false, isOffhandValid = false;

	if (!mainhandItem || mainhandItem->formType == kFormType_Spell) {
		isMainValid = true;
	}
	TESObjectWEAP *weap = DYNAMIC_CAST(mainhandItem, TESForm, TESObjectWEAP);
	if (weap) {
		if (IsTwoHanded(weap)) {
			return std::make_pair(false, true); // Main hand holds the weapon, offhand is 'free' in VR
		}
		else if (IsBow(weap)) {
			return std::make_pair(true, false); // For bows, the main hand holds the arrow, offhand holds the bow
		}
	}
	if (!offhandItem || offhandItem->formType == kFormType_Spell) {
		isOffhandValid = true;
	}
	return std::make_pair(isMainValid, isOffhandValid);
}

ITimer g_timer;
double g_currentFrameTime;
double g_deltaTime;
double GetTime()
{
	return g_timer.GetElapsedTime();
}

void PrintVector(const NiPoint3 &p)
{
	_MESSAGE("%.2f, %.2f, %.2f", p.x, p.y, p.z);
}

bool VisitNodes(NiAVObject  *parent, std::function<bool(NiAVObject*, int)> functor, int depth = 0)
{
	if (!parent) return false;
	NiNode * node = parent->GetAsNiNode();
	if (node) {
		if (functor(parent, depth))
			return true;

		//auto children = NINODE_CHILDREN(node);
		auto children = &node->m_children;
		for (UInt32 i = 0; i < children->m_emptyRunStart; i++) {
			NiAVObject * object = children->m_data[i];
			if (object) {
				if (VisitNodes(object, functor, depth + 1))
					return true;
			}
		}
	}
	else if (functor(parent, depth))
		return true;

	return false;
}

std::string PrintNodeToString(NiAVObject *avObj, int depth)
{
	std::stringstream avInfoStr;
	avInfoStr << std::string(depth, ' ') << avObj->GetRTTI()->name;
	if (avObj->m_name) {
		avInfoStr << " " << avObj->m_name;
	}
	if (avObj->unk040) {
		auto coll = (bhkCollisionObject *)avObj->unk040;
		avInfoStr.precision(5);
		avInfoStr << " m=" << (1.0f / coll->body->hkBody->motion.m_inertiaAndMassInv.w);
	}
	if (avObj->m_extraData && avObj->m_extraDataLen > 0) {
		avInfoStr << " { ";
		for (int i = 0; i < avObj->m_extraDataLen; i++) {
			auto extraData = avObj->m_extraData[i];
			auto boolExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiBooleanExtraData);
			if (boolExtraData) {
				avInfoStr << extraData->m_pcName << " (bool): " << boolExtraData->m_data << "; ";
				continue;
			}
			auto intExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiIntegerExtraData);
			if (intExtraData) {
				avInfoStr << extraData->m_pcName << " (int): " << intExtraData->m_data << "; ";
				continue;
			}
			auto stringExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiStringExtraData);
			if (stringExtraData) {
				avInfoStr << extraData->m_pcName << " (str): " << stringExtraData->m_pString << "; ";
				continue;
			}
			auto floatExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiFloatExtraData);
			if (floatExtraData) {
				avInfoStr << extraData->m_pcName << " (flt): " << floatExtraData->m_data << "; ";
				continue;
			}
			auto binaryExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiBinaryExtraData);
			if (binaryExtraData) {
				avInfoStr << extraData->m_pcName << " (bin): " << binaryExtraData->m_data << "; ";
				continue;
			}
			auto floatsExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiFloatsExtraData);
			if (floatsExtraData) {
				std::stringstream extrasStr;
				extrasStr << extraData->m_pcName << " (flts): ";
				for (int j = 0; j < floatsExtraData->m_size; j++) {
					if (j != 0)
						extrasStr << ", ";
					extrasStr << floatsExtraData->m_data[j];
				}
				avInfoStr << extrasStr.str() << "; ";
				continue;
			}
			auto intsExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiIntegersExtraData);
			if (intsExtraData) {
				std::stringstream extrasStr;
				extrasStr << extraData->m_pcName << " (ints): ";
				for (int j = 0; j < intsExtraData->m_size; j++) {
					if (j != 0)
						extrasStr << ", ";
					extrasStr << intsExtraData->m_data[j];
				}
				avInfoStr << extrasStr.str() << "; ";
				continue;
			}
			auto strsExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiStringsExtraData);
			if (strsExtraData) {
				std::stringstream extrasStr;
				extrasStr << extraData->m_pcName << " (strs): ";
				for (int j = 0; j < strsExtraData->m_size; j++) {
					if (j != 0)
						extrasStr << ", ";
					extrasStr << strsExtraData->m_data[j];
				}
				avInfoStr << extrasStr.str() << "; ";
				continue;
			}
			auto vecExtraData = DYNAMIC_CAST(extraData, NiExtraData, NiVectorExtraData);
			if (vecExtraData) {
				std::stringstream extrasStr;
				extrasStr << extraData->m_pcName << " (vec): ";
				for (int j = 0; j < 4; j++) {
					if (j != 0)
						extrasStr << ", ";
					extrasStr << vecExtraData->m_vector[j];
				}
				avInfoStr << extrasStr.str() << "; ";
				continue;
			}
			auto fgAnimExtraData = DYNAMIC_CAST(extraData, NiExtraData, BSFaceGenAnimationData);
			if (fgAnimExtraData) {
				avInfoStr << extraData->m_pcName << " (facegen anim); ";
				continue;
			}
			auto fgModelExtraData = DYNAMIC_CAST(extraData, NiExtraData, BSFaceGenModelExtraData);
			if (fgModelExtraData) {
				avInfoStr << extraData->m_pcName << " (facegen model); ";
				continue;
			}
			auto fgBaseMorphExtraData = DYNAMIC_CAST(extraData, NiExtraData, BSFaceGenBaseMorphExtraData);
			if (fgBaseMorphExtraData) {
				avInfoStr << extraData->m_pcName << " (facegen basemorph); ";
				continue;
			}
			avInfoStr << extraData->m_pcName << "; ";
		}
		avInfoStr << "}";
	}

	return std::regex_replace(avInfoStr.str(), std::regex("\\n"), " ");
}

bool PrintNodes(NiAVObject *avObj, int depth)
{
	gLog.Message(PrintNodeToString(avObj, depth).c_str());
	return false;
}

void PrintSceneGraph(NiAVObject *node)
{
	VisitNodes(node, PrintNodes);
}

void PrintToFile(std::string entry, std::string filename)
{
	std::ofstream file;
	file.open(filename);
	file << entry << std::endl;
	file.close();
}
