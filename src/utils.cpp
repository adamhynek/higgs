#include <chrono>
#include <list>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <regex>

#include "skse64/GameRTTI.h"
#include "skse64/NiExtraData.h"
#include "skse64/NiGeometry.h"

#include "utils.h"
#include "offsets.h"
#include "config.h"


ITimer g_timer;
double g_currentFrameTime;
//double g_deltaTime;
double GetTime()
{
	return g_timer.GetElapsedTime();
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
	hkpRigidBody *rb = hkpGetRigidBody(collidable);
	if (!rb)
		return false;

	auto motion = &rb->m_motion;
	// Motion types we allow are: Dynamic, SphereInertia, BoxInertia, ThinBoxInertia
	return (motion->m_type == 1 || motion->m_type == 2 || motion->m_type == 3 || motion->m_type == 6);
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

void updateTransformTree(NiAVObject * root, NiAVObject::ControllerUpdateContext *ctx)
{
	root->UpdateWorldData(ctx);

	auto node = root->GetAsNiNode();

	if (node) {
		for (int i = 0; i < node->m_children.m_arrayBufLen; ++i) {
			auto child = node->m_children.m_data[i];
			if (child) updateTransformTree(child, ctx);
		}
	}
}

void UpdateKeyframedNodeTransform(NiAVObject *node, const NiTransform &transform)
{
	NiTransform inverseParent;
	node->m_parent->m_worldTransform.Invert(inverseParent);

	node->m_localTransform = inverseParent * transform;
	NiAVObject::ControllerUpdateContext ctx;
	ctx.flags = 0x2000; // makes havok sim more stable?
	ctx.delta = 0;
	NiAVObject_UpdateObjectUpwards(node, &ctx);
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
		avInfoStr << " m=" << (1.0f / coll->body->hkBody->m_motion.getMassInv().getReal());
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

std::ofstream _file;
bool DumpNodes(NiAVObject *avObj, int depth)
{
	_file << PrintNodeToString(avObj, depth).c_str() << std::endl;
	return false;
}

void PrintSceneGraph(NiAVObject *node)
{
	_file.open("scenegraph.log");
	//VisitNodes(node, PrintNodes);
	VisitNodes(node, DumpNodes);
	_file.close();
}

void PrintToFile(std::string entry, std::string filename)
{
	std::ofstream file;
	file.open(filename);
	file << entry << std::endl;
	file.close();
}

/*
float hkHalfToFloat(hkHalf half)
{
	union {
		int i;
		float f;
	} u;
	u.i = (half << 16);
	return u.f;
}

hkHalf floatToHkHalf(float f)
{
	int t = ((const int*)&f)[0];
	return SInt16(t >> 16);
}
*/

bool DoesNodeHaveNode(NiAVObject *haystack, NiAVObject *target)
{
	if (haystack == target) {
		return true;
	}

	NiNode *node = haystack->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			NiAVObject *child = node->m_children.m_data[i];
			if (child) {
				if (DoesNodeHaveNode(child, target)) {
					return true;
				}
			}
		}
	}
	return false;
}

bool DoesRefrHaveNode(TESObjectREFR *ref, NiAVObject *node)
{
	if (!node || !ref || !ref->loadedState || !ref->loadedState->node) {
		return false;
	}

	return DoesNodeHaveNode(ref->loadedState->node, node);
}

bool IsNodeWithinArmor(NiAVObject *armorNode, NiAVObject *target)
{
	BSGeometry *geom = armorNode->GetAsBSGeometry();
	if (geom) {
		NiSkinInstancePtr skinInstance = geom->m_spSkinInstance;
		if (skinInstance) {
			NiSkinDataPtr skinData = skinInstance->m_spSkinData;
			if (skinData) {
				UInt32 numBones = *(UInt32*)((UInt64)skinData.m_pObject + 0x58);
				for (int i = 0; i < numBones; i++) {
					NiAVObject *bone = skinInstance->m_ppkBones[i];
					if (bone) {
						if (bone == target) {
							return true;
						}
					}
				}
			}
		}
		return false;
	}
	NiNode *node = armorNode->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				if (IsNodeWithinArmor(child, target)) {
					return true;
				}
			}
		}
		return false;
	}
	_WARNING("Armor node is not geometry and has no children: %", armorNode->m_name ? armorNode->m_name : "");
	return false;
}
