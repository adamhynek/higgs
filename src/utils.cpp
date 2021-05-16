#include <chrono>
#include <list>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <regex>
#include <map>

#include "skse64/GameRTTI.h"
#include "skse64/NiExtraData.h"
#include "skse64/NiGeometry.h"
#include "skse64/HashUtil.h"

#include "utils.h"
#include "offsets.h"
#include "config.h"
#include "grabber.h"


RelocPtr<UInt64> unk_141E703BC(0x1E703BC);
RelocPtr<UInt64> unk_141E703B8(0x1E703B8);


ITimer g_timer;
double g_currentFrameTime;
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
	return (
		motion->m_type == hkpMotion::MotionType::MOTION_DYNAMIC ||
		motion->m_type == hkpMotion::MotionType::MOTION_SPHERE_INERTIA ||
		motion->m_type == hkpMotion::MotionType::MOTION_BOX_INERTIA ||
		motion->m_type == hkpMotion::MotionType::MOTION_THIN_BOX_INERTIA
		);
}

bool HasGeometryChildren(NiAVObject *obj)
{
	NiNode *node = obj->GetAsNiNode();
	if (!node) {
		return false;
	}

	for (int i = 0; i < node->m_children.m_arrayBufLen; ++i) {
		auto child = node->m_children.m_data[i];
		if (child && child->GetAsBSGeometry()) {
			return true;
		}
	}

	return false;
}

bool DoesEntityHaveConstraint(NiAVObject *root, bhkRigidBody *entity)
{
	auto rigidBody = GetRigidBody(root);
	if (rigidBody) {
		for (int i = 0; i < rigidBody->constraints.count; i++) {
			bhkConstraint *constraint = rigidBody->constraints.entries[i];
			if (constraint->constraint->getEntityA() == entity->hkBody || constraint->constraint->getEntityB() == entity->hkBody) {
				return true;
			}
		}
	}

	NiNode *rootNode = root->GetAsNiNode();
	if (rootNode) {
		for (int i = 0; i < rootNode->m_children.m_emptyRunStart; i++) {
			NiPointer<NiAVObject> child = rootNode->m_children.m_data[i];
			if (child) {
				if (DoesEntityHaveConstraint(child, entity)) {
					return true;
				}
			}
		}
	}

	return false;
}

bool DoesNodeHaveConstraint(NiNode *rootNode, NiAVObject *node)
{
	bhkRigidBody *entity = GetRigidBody(node);
	if (!entity) {
		return false;
	}

	if (entity->constraints.count > 0) {
		// Easy case: it's a master entity
		return true;
	}

	return DoesEntityHaveConstraint(rootNode, entity);
}

NiPointer<NiAVObject> GetTorsoNode(Actor *actor)
{
	TESRace *race = actor->race;
	BGSBodyPartData *partData = race->bodyPartData;
	if (partData) {
		auto torsoData = partData->part[0];
		if (torsoData && torsoData->unk08.data) {
			NiPointer<NiNode> actorNode = actor->GetNiNode();
			if (actorNode) {
				NiPointer<NiAVObject> torsoNode = actorNode->GetObjectByName(&torsoData->unk08.data);
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

void UpdateNodeTransformLocal(NiAVObject *node, const NiTransform &worldTransform)
{
	// Given world transform, set the necessary local transform

	NiPointer<NiNode> parent = node->m_parent;
	if (parent) {
		NiTransform inverseParent;
		node->m_parent->m_worldTransform.Invert(inverseParent);

		node->m_localTransform = inverseParent * worldTransform;
	}
	else {
		node->m_localTransform = worldTransform;
	}
}

void UpdateBoneMatrices(NiAVObject *obj)
{
	BSGeometry *geom = obj->GetAsBSGeometry();
	if (geom) {
		NiSkinInstance *skinInstance = geom->m_spSkinInstance;
		if (skinInstance) {
			skinInstance->unk38 = -1; // This is the frameID. UpdateBoneMatrices only updates the bone matrices if the frameID is not equal to the current frame.
			NiSkinInstance_UpdateBoneMatrices(skinInstance, obj->m_worldTransform);
		}
	}

	NiNode *node = obj->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			NiAVObject *child = node->m_children.m_data[i];
			if (child) {
				UpdateBoneMatrices(child);
			}
		}
	}
}

void UpdateKeyframedNode(NiAVObject *node, NiTransform &transform)
{
	UpdateNodeTransformLocal(node, transform);
	
	NiAVObject::ControllerUpdateContext ctx;
	ctx.flags = 0x2000; // makes havok sim more stable
	ctx.delta = 0;
	NiAVObject_UpdateObjectUpwards(node, &ctx);
	
	UpdateBoneMatrices(node);

	//UpdateNodeTransformLocal(node, transform);

	//// UpdateWithContext
	/*
	// UpdateCollisionFromNodeTransform
	NiAVObject_RecalculateWorldTransform(node);

	NiQuaternion rot;
	NiMatrixToNiQuaternion(rot, transform.rot);
	NiPointer<bhkRigidBody> rigidBody = GetRigidBody(node);
	if (rigidBody) {
		bhkRigidBody_setActivated(rigidBody, 1);
		bhkRigidBody_MoveToPositionAndRotation(rigidBody, transform.pos, rot);
	}
	//

	NiAVObject_RecalculateWorldTransform(node);
	////
	*/
	/*
	bhkCollisionObject *collisionObject = GetCollisionObject(node);
	if (collisionObject) {
		bhkCollisionObject_SetNodeTransformsFromWorldTransform(collisionObject, transform);
	}
	*/

	ShadowSceneNode_UpdateNodeList(*g_shadowSceneNode, node, false);
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
		return { true, true };
	}

	TESForm *mainhandItem = actor->GetEquippedObject(false);
	TESForm *offhandItem = actor->GetEquippedObject(true);

	return { !mainhandItem, !offhandItem };

	/*
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
	*/
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

		auto children = &node->m_children;
		for (UInt32 i = 0; i < children->m_emptyRunStart; i++) {
			NiPointer<NiAVObject> object = children->m_data[i];
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
	BSGeometry *geom = avObj->GetAsBSGeometry();
	if (geom) {
		if (geom->m_spSkinInstance) {
			avInfoStr << " [skinned]";
		}
	}
	auto rigidBody = GetRigidBody(avObj);
	if (rigidBody) {
		avInfoStr.precision(5);
		avInfoStr << " m=" << (1.0f / rigidBody->hkBody->m_motion.getMassInv().getReal());
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

void DumpVertices(std::vector<TriangleData> &triangles)
{
	_file.open("vertices.log");

	std::vector<NiPoint3> vertices;
	for (TriangleData &triangle : triangles) {
		bool contains = false;
		for (NiPoint3 vertex : vertices) {
			if (vertex.x == triangle.v0.x && vertex.y == triangle.v0.y && vertex.z == triangle.v0.z) {
				contains = true;
			}
		}
		if (!contains) {
			vertices.push_back(triangle.v0);
		}

		contains = false;
		for (NiPoint3 vertex : vertices) {
			if (vertex.x == triangle.v1.x && vertex.y == triangle.v1.y && vertex.z == triangle.v1.z) {
				contains = true;
			}
		}
		if (!contains) {
			vertices.push_back(triangle.v1);
		}

		contains = false;
		for (NiPoint3 vertex : vertices) {
			if (vertex.x == triangle.v2.x && vertex.y == triangle.v2.y && vertex.z == triangle.v2.z) {
				contains = true;
			}
		}
		if (!contains) {
			vertices.push_back(triangle.v2);
		}
	}

	for (NiPoint3 vertex : vertices) {
		_file << vertex.x << ',' << vertex.y << ',' << vertex.z << ';';
	}

	_file.close();
}

bhkCollisionObject * GetCollisionObject(NiAVObject *obj)
{
	if (!obj->unk040) return nullptr;

	auto niCollObj = ((NiCollisionObject *)obj->unk040);
	auto collObj = DYNAMIC_CAST(niCollObj, NiCollisionObject, bhkCollisionObject);
	return collObj;
}

NiPointer<bhkRigidBody> GetRigidBody(NiAVObject *obj)
{
	auto collObj = GetCollisionObject(obj);
	if (collObj) {
		NiPointer<bhkWorldObject> worldObj = collObj->body;
		auto rigidBody = DYNAMIC_CAST(worldObj, bhkWorldObject, bhkRigidBody);
		if (rigidBody) {
			return rigidBody;
		}
	}

	return nullptr;
}

bool DoesNodeHaveNode(NiAVObject *haystack, NiAVObject *target)
{
	if (haystack == target) {
		return true;
	}

	NiNode *node = haystack->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			NiPointer<NiAVObject> child = node->m_children.m_data[i];
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
	if (!node || !ref) {
		return false;
	}

	NiPointer<NiNode> rootObj = ref->GetNiNode();
	if (!rootObj) {
		return false;
	}

	return DoesNodeHaveNode(rootObj, node);
}

bool IsSkinnedToNodes(NiAVObject *skinnedRoot, const std::unordered_set<NiAVObject *> &targets)
{
	// Check if skinnedRoot is skinned to target
	BSGeometry *geom = skinnedRoot->GetAsBSGeometry();
	if (geom) {
		NiSkinInstancePtr skinInstance = geom->m_spSkinInstance;
		if (skinInstance) {
			NiSkinDataPtr skinData = skinInstance->m_spSkinData;
			if (skinData) {
				for (int i = 0; i < skinData->m_uiBones; i++) {
					NiPointer<NiAVObject> bone = skinInstance->m_ppkBones[i];
					if (bone) {
						if (targets.count(bone) != 0) {
							return true;
						}
					}
				}
			}
		}
		return false;
	}
	NiNode *node = skinnedRoot->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				if (IsSkinnedToNodes(child, targets)) {
					return true;
				}
			}
		}
		return false;
	}
	return false;
}

void GetDownstreamNodes(NiAVObject *root, std::unordered_set<NiAVObject *> &targets)
{
	if (!root) return;

	targets.insert(root);

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				GetDownstreamNodes(child, targets);
			}
		}
	}
}

void GetDownstreamNodesNoCollision(NiAVObject *root, std::unordered_set<NiAVObject *> &targets)
{
	if (!root) return;

	// Populate targets with the entire subtree but stop when we hit a node with collision

	targets.insert(root);

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				if (!GetRigidBody(child)) {
					GetDownstreamNodesNoCollision(child, targets);
				}
			}
		}
	}
}

std::unordered_set<NiAVObject *> targetNodeSet;
bool IsSkinnedToNode(NiAVObject *skinnedRoot, NiAVObject *target)
{
	GetDownstreamNodesNoCollision(target, targetNodeSet);

	bool result = IsSkinnedToNodes(skinnedRoot, targetNodeSet);

	targetNodeSet.clear();
	return result;
}

NiPointer<bhkRigidBody> GetFirstRigidBody(NiAVObject *root)
{
	auto rigidBody = GetRigidBody(root);
	if (rigidBody) {
		return rigidBody;
	}

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				return GetFirstRigidBody(child);
			}
		}
	}

	return nullptr;
}

UInt32 PlaySoundAtNode(BGSSoundDescriptorForm *sound, NiAVObject *node, const NiPoint3 &location)
{
	UInt32 formId = sound->formID;

	SoundData soundData;
	BSAudioManager_InitSoundData(*g_audioManager, &soundData, formId, 16);
	if (soundData.id == -1) {
		return 0;
	}

	SoundData_SetPosition(&soundData, location.x, location.y, location.z);
	SoundData_SetNode(&soundData, node);
	if (SoundData_Play(&soundData)) {
		return soundData.id;
	}

	return 0;
}

const char * GetItemName(TESForm *form, BaseExtraList *extraList)
{
	if (!form || !extraList)
		return 0;

	const char * name = extraList->GetDisplayName(form);

	// No name in extra data? Use base form name
	if (!name)
	{
		TESFullName* pFullName = DYNAMIC_CAST(form, TESForm, TESFullName);
		if (pFullName)
			name = pFullName->name.data;
	}

	return name;
}

// Taken from PapyrusActor.cpp
SInt32 GetItemId(TESForm * form, BaseExtraList * extraList)
{
	const char *name = GetItemName(form, extraList);
	if (!name) {
		return 0;
	}

	return (SInt32)HashUtil::CRC32(name, form->formID & 0x00FFFFFF);
}

// Ripped from PapyrusActor.cpp
class MatchBySlot : public FormMatcher
{
	UInt32 m_mask;
public:
	MatchBySlot(UInt32 slot) :
		m_mask(slot)
	{

	}

	bool Matches(TESForm* pForm) const {
		if (pForm) {
			BGSBipedObjectForm* pBip = DYNAMIC_CAST(pForm, TESForm, BGSBipedObjectForm);
			if (pBip) {
				return (pBip->data.parts & m_mask) != 0;
			}
		}
		return false;
	}
};

EquipData GetWornItem(Actor* thisActor, UInt32 mask)
{
	ExtraContainerChanges* containerChanges = static_cast<ExtraContainerChanges*>(thisActor->extraData.GetByType(kExtraData_ContainerChanges));
	if (!containerChanges)
		return { nullptr, nullptr };

	MatchBySlot matcher(mask);
	return containerChanges->FindEquipped(matcher);
}

Grabber * GetGrabberToShowRolloverFor()
{
	bool displayLeft = g_leftGrabber->ShouldDisplayRollover();
	bool displayRight = g_rightGrabber->ShouldDisplayRollover();

	bool isRolloverSet = displayRight || displayLeft;
	if (isRolloverSet) {
		if (displayRight && displayLeft) {
			// Pick whichever hand grabbed last
			if (g_leftGrabber->rolloverDisplayTime > g_rightGrabber->rolloverDisplayTime) {
				return g_leftGrabber;
			}
			else {
				return g_rightGrabber;
			}
		}
		else if (displayRight) {
			return g_rightGrabber;
		}
		else if (displayLeft) {
			return g_leftGrabber;
		}
	}
	
	return nullptr;
}

void SetSelectedHandles(bool isLeftHanded, UInt32 handle)
{
	// Now set all the places I could find that get set to the handle of the pointed at object usually
	CrosshairPickData *pickData = *g_pickData;
	if (pickData) {
		if (isLeftHanded) {
			pickData->leftHandle1 = handle;
			pickData->leftHandle2 = handle;
			pickData->leftHandle3 = handle;
		}
		else {
			pickData->rightHandle1 = handle;
			pickData->rightHandle2 = handle;
			pickData->rightHandle3 = handle;
		}

		//PlayerCharacter *player = *g_thePlayer;
		//if (player) {
			// This flag is used to tell the game to refresh the rollover during the PlayerCharacter update
		//	*((UInt8 *)player + 0x12D5) |= 0x20;
		//}
	}
}

void ReplaceBSString(BSString &replacee, std::string &replacer)
{
	Heap_Free(replacee.m_data);

	size_t len = replacer.length() + 1;
	replacee.m_data = (char*)Heap_Allocate(len);
	strcpy_s(replacee.m_data, len, replacer.c_str());
	replacee.m_dataLen = len;
	replacee.m_bufLen = len;
}
