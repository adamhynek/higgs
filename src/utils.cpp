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
#include "config.h"
#include "hand.h"


UInt32 g_bipedObjectPriorities[] = { // lower is better
	4, // head
	3, // hair
	10, // body
	6, // hands
	7, // forearms
	2, // amulet
	5, // ring
	8, // feet
	9, // calves
	0, // shield
	13, // tail - seems to be used by cloaks
	11, // longhair
	1, // circlet
	12, // ears
	84, // 14
	85, // 15
	14, // 16 - seems to be used by cloaks as well
	86, // 17
	87, // 18
	88, // 19
	89, // decapitatehead
	90, // decapitate
	91, // 22
	92, // 23
	93, // 24
	94, // 25
	95, // 26
	96, // 27
	97, // 28
	98, // 29
	99, // 30
	100  // fx01
};
bool IsBipedIndexHigherPriority(int indexInQuestion, int indexToBeat)
{
	// Return true if fist index beats second index
	return g_bipedObjectPriorities[indexInQuestion] < g_bipedObjectPriorities[indexToBeat];
}

// Copied from PapyrusActor.cpp
class MatchByForm : public FormMatcher
{
	TESForm *m_form;
public:
	MatchByForm(TESForm *form) : m_form(form) {}

	bool Matches(TESForm *pForm) const { return m_form == pForm; }
};

std::tuple<EquipData, int> GetEquipDataForBipedObject(Actor *actor, Biped *bipedData, TESForm *matchForm, int bipedIndex)
{
	EquipData equipData{ nullptr, nullptr };

	if (ExtraContainerChanges *containerChanges = static_cast<ExtraContainerChanges *>(actor->extraData.GetByType(kExtraData_ContainerChanges))) {
		MatchByForm matcher(matchForm);

		if (bipedIndex == 9) { // 9 == shield / left-hand weapon
			equipData = containerChanges->FindEquipped(matcher, false, true);
			if (!equipData.pForm) {
				// Sometimes it's not actually WornLeft...
				equipData = containerChanges->FindEquipped(matcher, true, true);
			}
		}
		else {
			equipData = containerChanges->FindEquipped(matcher, true, true);
		}

		if (equipData.pForm) {
			Biped::Data *hitBipedData = &bipedData->unk10[bipedIndex];
			TESObjectARMO *hitArmor = DYNAMIC_CAST(hitBipedData->armor, TESForm, TESObjectARMO);
			// If it's armor, make sure it has a name. If it doesn't, it could be FEC, or who knows...
			if (!hitArmor || *hitArmor->fullName.name.data) {
				return { equipData, ContainerChanges_GetCount(containerChanges->data, equipData.pForm) };
			}
		}
	}
	return { { nullptr, nullptr }, 0 };
};

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

bool IsMoveableEntity(hkpEntity *entity)
{
	hkpMotion *motion = &entity->m_motion;
	return IsMotionTypeMoveable(motion->m_type);
}

bool IsObjectSelectable(hkpRigidBody *rigidBody, TESObjectREFR *ref)
{
	if (IsMoveableEntity(rigidBody)) return true;

	if (TESForm *baseForm = ref->baseForm) {
		if (baseForm->formType == kFormType_Projectile ||
			(baseForm->formType == kFormType_Weapon && rigidBody->m_motion.m_type == hkpMotion::MotionType::MOTION_KEYFRAMED)) {
			return true;
		}
	}

	return false;
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

NiTransform GetLocalTransformForDesiredWorldTransform(NiAVObject *node, const NiTransform &worldTransform, bool useOldParentTransform)
{
	if (NiPointer<NiNode> parent = node->m_parent) {
		NiTransform inverseParent = InverseTransform(useOldParentTransform ? node->m_parent->m_oldWorldTransform : node->m_parent->m_worldTransform);
		return inverseParent * worldTransform;
	}
	return worldTransform;
}

void UpdateNodeTransformLocal(NiAVObject *node, const NiTransform &worldTransform)
{
	// Given world transform, set the necessary local transform
	node->m_localTransform = GetLocalTransformForDesiredWorldTransform(node, worldTransform);
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
	ctx.flags = 0x2000; // Use velocity for moving the collision object - this won't actually move it until the next sim
	ctx.delta = 0;
	NiAVObject_UpdateNode(node, &ctx); // This will set the collision object's velocity as well

	bhkCollisionObject *collisionObject = GetCollisionObject(node);
	if (collisionObject) {
		bhkBlendCollisionObject *blendCollisionObject = DYNAMIC_CAST(collisionObject, bhkCollisionObject, bhkBlendCollisionObject);
		if (blendCollisionObject) {
			// The bhkBlendCollisionObject update function does not have a case where it checks if it's keyframed (and thus would do a node->collision update instead of a collision->node update) like the bhkCollisionObject update does.
			// So, I need to do it myself.
			NiPointer<bhkRigidBody> rigidBody = GetRigidBody(node);
			if (rigidBody) {
				rigidBody->flags |= (1 << 6); // I'm not 100% sure what the true purpose of this flag is, but the bhkBlendCollisionObject update function skips updating the node from the collision if it's set, which is handy for me.
				UpdateNodeTransformLocal(node, transform);
				blendCollisionObject->UpdateCollisionFromNodeTransform();
			}
		}
	}

	NiPointer<bhkRigidBody> rigidBody = GetRigidBody(node);
	if (rigidBody) {
		bhkRigidBody_setActivated(rigidBody, 1);

		bhkRigidBodyT *rigidBodyT = DYNAMIC_CAST(rigidBody, bhkRigidBody, bhkRigidBodyT);
		if (rigidBodyT) {
			// bhkRigidBodyT means the collision object is offset from the node. Bethesda didn't code it correctly in the node update when using the velocity flag for this case, so I have to do it myself.

			NiTransform rigidBodyLocalTransform;
			rigidBodyLocalTransform.pos = HkVectorToNiPoint(rigidBodyT->translation) * *g_inverseHavokWorldScale;
			rigidBodyLocalTransform.rot = QuaternionToMatrix(HkQuatToNiQuat(rigidBodyT->rotation));

			NiTransform rigidBodyTransform = transform * rigidBodyLocalTransform;

			NiPoint3 pos = rigidBodyTransform.pos;
			NiQuaternion rot = MatrixToQuaternion(rigidBodyTransform.rot);

			//bhkRigidBody_MoveToPositionAndRotation(rigidBody, pos, rot); // This doesn't work because this function does stuff like read the collision object's center of mass without compensating for bhkRigidBodyT transformations...
			ApplyHardKeyframeVelocityClamped(NiPointToHkVector(pos * *g_havokWorldScale), NiQuatToHkQuat(rot), 1.0f / *g_deltaTime, rigidBody);
		}
	}
	
	UpdateBoneMatrices(node); // Update skinned geometry on the object so that it's not a frame behind

	ShadowSceneNode_UpdateNodeList(*g_shadowSceneNode, node, false); // Gets shadows to update since keyframed nodes are not "dynamic" and so the game doesn't think they can move

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
}

bool IsTwoHanded(const TESObjectWEAP *weap)
{
	switch (weap->gameData.type) {
	case TESObjectWEAP::GameData::kType_CrossBow:
	case TESObjectWEAP::GameData::kType_TwoHandAxe:
	case TESObjectWEAP::GameData::kType_TwoHandSword:
		return true;
	default:
		return false;
	}
}

bool IsTwoHandable(const TESObjectWEAP *weap)
{
	UInt8 type = weap->gameData.type;
	if (type == TESObjectWEAP::GameData::kType_OneHandDagger && !Config::options.allowDaggerTwoHanding) return false;

	// Basically, not unarmed or bow
	switch (type) {
	case TESObjectWEAP::GameData::kType_OneHandSword:
	case TESObjectWEAP::GameData::kType_OneHandDagger:
	case TESObjectWEAP::GameData::kType_OneHandAxe:
	case TESObjectWEAP::GameData::kType_OneHandMace:
	case TESObjectWEAP::GameData::kType_CrossBow:
	case TESObjectWEAP::GameData::kType_TwoHandAxe:
	case TESObjectWEAP::GameData::kType_TwoHandSword:
	case TESObjectWEAP::GameData::kType_Staff:
		return true;
	default:
		return false;
	}
}

bool IsBow(const TESObjectWEAP *weap)
{
	UInt8 type = weap->gameData.type;
	return (type == TESObjectWEAP::GameData::kType_Bow);
}

TESObjectWEAP * GetEquippedWeapon(Actor *actor, bool isOffhand)
{
	TESForm *equippedObject = actor->GetEquippedObject(isOffhand);
	if (equippedObject) {
		return DYNAMIC_CAST(equippedObject, TESForm, TESObjectWEAP);
	}
	return nullptr;
}

SpellItem *GetEquippedSpell(Actor *actor, bool isOffhand)
{
	TESForm *form = actor->GetEquippedObject(isOffhand);
	if (form) {
		return DYNAMIC_CAST(form, TESForm, SpellItem);
	}
	return nullptr;
}

bool IsUnarmed(TESForm *equippedObject)
{
	if (!equippedObject) return true;

	TESObjectWEAP *equippedWeapon = DYNAMIC_CAST(equippedObject, TESForm, TESObjectWEAP);
	if (equippedWeapon && equippedWeapon->type() == TESObjectWEAP::GameData::kType_HandToHandMelee) {
		return true;
	}

	return false;
}

void PrintVector(const NiPoint3 &p)
{
	_MESSAGE("%.2f, %.2f, %.2f", p.x, p.y, p.z);
}

void PrintQuat(const NiQuaternion &q)
{
	_MESSAGE("%f, %f, %f, %f", q.m_fW, q.m_fX, q.m_fY, q.m_fZ);
}

std::set<std::string, std::less<>> SplitStringToSet(const std::string &s, char delim)
{
	std::set<std::string, std::less<>> result;
	std::stringstream ss(s);
	std::string item;

	while (getline(ss, item, delim)) {
		trim(item);
		result.insert(item);
	}

	return result;
}

bool VisitNodes(NiAVObject  *parent, std::function<bool(NiAVObject*, int)> functor, int depth)
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

NiPointer<bhkCollisionObject> GetCollisionObject(NiAVObject *obj)
{
	if (!obj->unk040) return nullptr;

	if (NiPointer<NiCollisionObject> niCollObj = (NiCollisionObject *)obj->unk040) {
		if (NiPointer<bhkCollisionObject> collObj = DYNAMIC_CAST(niCollObj, NiCollisionObject, bhkCollisionObject)) {
			return collObj;
		}
	}

	return nullptr;
}

NiPointer<bhkRigidBody> GetRigidBody(NiAVObject *obj)
{
	if (NiPointer<bhkCollisionObject> collObj = GetCollisionObject(obj)) {
		if (NiPointer<bhkWorldObject> worldObj = collObj->body) {
			if (NiPointer<bhkRigidBody> rigidBody = DYNAMIC_CAST(worldObj, bhkWorldObject, bhkRigidBody)) {
				return rigidBody;
			}
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
	if (BSGeometry *geom = skinnedRoot->GetAsBSGeometry()) {
		if (NiSkinInstancePtr skinInstance = geom->m_spSkinInstance) {
			if (NiSkinDataPtr skinData = skinInstance->m_spSkinData) {
				for (int i = 0; i < skinData->m_uiBones; i++) {
					if (NiPointer<NiAVObject> bone = skinInstance->m_ppkBones[i]) {
						if (targets.count(bone) != 0) {
							return true;
						}
					}
				}
			}
		}

		return false;
	}

	if (NiNode *node = skinnedRoot->GetAsNiNode()) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			if (NiAVObject *child = node->m_children.m_data[i]) {
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

	if (NiNode *node = root->GetAsNiNode()) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			if (NiAVObject *child = node->m_children.m_data[i]) {
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

	if (NiNode *node = root->GetAsNiNode()) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			if (NiAVObject *child = node->m_children.m_data[i]) {
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
	if (NiPointer<bhkRigidBody> rigidBody = GetRigidBody(root)) {
		return rigidBody;
	}

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			if (NiAVObject *child = node->m_children.m_data[i]) {
				if (NiPointer<bhkRigidBody> rigidBody = GetFirstRigidBody(child)) {
					return rigidBody;
				}
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

Hand * GetHandToShowRolloverFor()
{
	bool displayLeft = g_leftHand->ShouldDisplayRollover(*g_rightHand);
	bool displayRight = g_rightHand->ShouldDisplayRollover(*g_leftHand);

	bool isRolloverSet = displayRight || displayLeft;
	if (isRolloverSet) {
		if (displayRight && displayLeft) {
			// Pick whichever hand grabbed last
			if (g_leftHand->rolloverDisplayTime > g_rightHand->rolloverDisplayTime) {
				return g_leftHand;
			}
			else {
				return g_rightHand;
			}
		}
		else if (displayRight) {
			return g_rightHand;
		}
		else if (displayLeft) {
			return g_leftHand;
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

void SetGeometryAlphaDownstream(NiAVObject *root, float alpha)
{
	BSGeometry *geom = DYNAMIC_CAST(root, NiAVObject, BSGeometry);
	if (geom) {
		NiPointer<NiProperty> geomProperty = geom->m_spEffectState;
		if (geomProperty) {
			BSShaderProperty *shaderProperty = DYNAMIC_CAST(geomProperty, NiProperty, BSShaderProperty);
			if (shaderProperty) {
				BSShaderMaterial *material = shaderProperty->material;
				if (material) {
					BSEffectShaderMaterial *effectShaderMaterial = DYNAMIC_CAST(material, BSShaderMaterial, BSEffectShaderMaterial);
					if (effectShaderMaterial) {
						*(float *)((UInt64)effectShaderMaterial + 0x54) = alpha;
						//*(float *)&shaderProperty->unk18 = alpha;
					}
				}
			}
		}
	}

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				SetGeometryAlphaDownstream(child, alpha);
			}
		}
	}
}

NiPointer<NiAVObject> GetClosestParentWithCollision(NiAVObject *node)
{
	NiPointer<NiAVObject> nodeWithCollision = node;
	while (nodeWithCollision) {
		if (NiPointer<bhkRigidBody> rigidBody = GetRigidBody(nodeWithCollision)) {
			if (rigidBody->hkBody->m_world) {
				return nodeWithCollision;
			}
		}
		nodeWithCollision = nodeWithCollision->m_parent;
	}
	return nullptr;
}

NiPointer<BSFlattenedBoneTree> GetFlattenedBoneTree(NiAVObject *root)
{
	if (!root) return nullptr;

	BSFlattenedBoneTree *boneTree = DYNAMIC_CAST(root, NiAVObject, BSFlattenedBoneTree);
	if (boneTree) return boneTree;

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				return GetFlattenedBoneTree(child);
			}
		}
	}

	return nullptr;
}

NiPointer<BSFlattenedBoneTree> GetFlattenedBoneTree(TESObjectREFR *refr)
{
	return GetFlattenedBoneTree(refr->GetNiNode());
}

NiAVObject * GetNodeMatchingBoneTreeTransform(BSFlattenedBoneTree *tree, NiTransform *worldTransform)
{
	for (int i = 0; i < tree->numBones; i++) {
		BSFlattenedBoneTree::BoneEntry &entry = tree->boneEntries[i];
		if (&entry.world == worldTransform) {
			if (NiAVObject *node = entry.node) {
				return node;
			}
			else if (BSFixedString nodeName = entry.nodeName) {
				return tree->GetObjectByName(&nodeName.data);
			}
		}
	}

	return nullptr;
}

void ModSpeedMult(Actor *actor, float amount)
{
	// actor->RestoreActorValue(kTemporary, kSpeedMult, amount)
	get_vfunc<_ActorValueOwner_RestoreActorValue>(&actor->actorValueOwner, 6)(&actor->actorValueOwner, 1, 30, amount);

	// Need to cycle carry weight to make speed change take effect
	// actor->RestoreActorValue(kTemporary, kCarryWeight, amount)
	get_vfunc<_ActorValueOwner_RestoreActorValue>(&actor->actorValueOwner, 6)(&actor->actorValueOwner, 1, 32, 0.1f);
	get_vfunc<_ActorValueOwner_RestoreActorValue>(&actor->actorValueOwner, 6)(&actor->actorValueOwner, 1, 32, -0.1f);
}

void ConsumeSpellBook(PlayerCharacter *player, TESObjectBOOK *book)
{
	if (EquipManager *equipManager = EquipManager::GetSingleton()) {
		ExtraContainerChanges *containerChanges = static_cast<ExtraContainerChanges *>(player->extraData.GetByType(kExtraData_ContainerChanges));
		ExtraContainerChanges::Data *containerData = containerChanges ? containerChanges->data : nullptr;
		if (containerData) {
			if (InventoryEntryData *entryData = containerData->FindItemEntry(book)) {
				EquipManager_EquipEntryData(equipManager, player, entryData, nullptr);
				if (TESObjectBOOK_LearnSpell(book, player)) {
					UInt32 droppedObjHandle = *g_invalidRefHandle;
					BaseExtraList *extraList = entryData->extendDataList ? entryData->extendDataList->GetNthItem(0) : nullptr;
					get_vfunc<Actor_RemoveItem>(player, 0x56)(player, &droppedObjHandle, book, 1, 0, extraList, nullptr, nullptr, nullptr);
				}
			}
		}
	}
}
