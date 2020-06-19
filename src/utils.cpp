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


ITimer g_timer;
double g_currentFrameTime;
//double g_deltaTime;
double GetTime()
{
	return g_timer.GetElapsedTime();
}

NiPoint3 VectorNormalized(NiPoint3 vec)
{
	float length = VectorLength(vec);
	return length ? vec / length : NiPoint3();
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

void NiMatrixToHkMatrix(NiMatrix33 &niMat, hkMatrix3 &hkMat)
{
	hkMat.setCols({ niMat.data[0][0], niMat.data[1][0], niMat.data[2][0], 0 },
		{ niMat.data[0][1], niMat.data[1][1], niMat.data[2][1], 0 },
		{ niMat.data[0][2], niMat.data[1][2], niMat.data[2][2], 0 });
}

float Determinant33(const NiMatrix33 &m)
{
	float a = m.data[0][0];
	float b = m.data[0][1];
	float c = m.data[0][2];
	float d = m.data[1][0];
	float e = m.data[1][1];
	float f = m.data[1][2];
	float g = m.data[2][0];
	float h = m.data[2][1];
	float i = m.data[2][2];
	return a * (e*i - f*h) - b * (d*i - f*g) + c * (d*h - e*g);
}

NiPoint3 QuadraticFromPoints(NiPoint2 p1, NiPoint2 p2, NiPoint2 p3)
{
	// Fit a quadratic to the 3 given points, and return the coefficients of x^2, x^1, x^0
	float x1sqr = p1.x*p1.x;
	float x2sqr = p2.x*p2.x;
	float x3sqr = p3.x*p3.x;

	float inverseDet = x1sqr * (p2.x - p3.x) - p1.x * (x2sqr - x3sqr) + (x2sqr*p3.x - x3sqr*p2.x);
	if (abs(inverseDet) <= 0.0001f) {
		return NiPoint3();
	}
	inverseDet = 1.0f / inverseDet;

	NiMatrix33 i; // inverse

	i.data[0][0] = p2.x - p3.x;
	i.data[0][1] = p3.x - p1.x;
	i.data[0][2] = p1.x - p2.x;

	i.data[1][0] = x3sqr - x2sqr;
	i.data[1][1] = x1sqr - x3sqr;
	i.data[1][2] = x2sqr - x1sqr;

	i.data[2][0] = x2sqr*p3.x - x3sqr*p2.x;
	i.data[2][1] = x3sqr*p1.x - x1sqr*p3.x;
	i.data[2][2] = x1sqr*p2.x - x2sqr*p1.x;

	i = i * inverseDet;

	NiPoint3 yVals = { p1.y, p2.y, p3.y };
	return i * yVals;
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

// Map havok entity id -> (saved collisionfilterinfo, "refcount" of sorts)
std::unordered_map<UInt32, std::pair<UInt32, UInt8>> collisionInfoIdMap;

void ClearCollisionMap()
{
	// Should only be called when you're sure there should be nothing in the map
	if (collisionInfoIdMap.size() > 0) {
		collisionInfoIdMap.clear();
	}
}

UInt32 GetSavedCollision(UInt32 id)
{
	try {
		return collisionInfoIdMap.at(id).first;
	}
	catch (std::out_of_range) {
		// do not have saved filter info for this entity... it must have been added somehow between saving and reseting
		return 0;
	}
}

UInt32 GetSavedCollisionRefCount(UInt32 id)
{
	try {
		return collisionInfoIdMap.at(id).second;
	}
	catch (std::out_of_range) {
		// do not have saved filter info for this entity... it must have been added somehow between saving and reseting
		return 0;
	}
}

void RemoveSavedCollision(UInt32 id)
{
	try {
		auto val = collisionInfoIdMap.at(id);
		UInt8 count = val.second;
		UInt32 collisionFilterInfo = val.first;
		if (count == 1) {
			// Other hand is not affecting this entity
			collisionInfoIdMap.erase(id);
		}
		else {
			// Other hand is still affecting this entity - 'decref'
			collisionInfoIdMap[id] = { collisionFilterInfo, count - 1 };
		}
	}
	catch (std::out_of_range) {
		// do not have saved filter info for this entity... it must have been added somehow between saving and reseting
	}
}


void SetCollisionInfoDownstream(NiAVObject *obj, UInt32 collisionGroup)
{
	if (obj->unk040) {
		auto collObj = (bhkCollisionObject *)obj->unk040;
		hkpRigidBody *entity = collObj->body->hkBody;
		if (entity->m_world) {
			hkpCollidable *collidable = &entity->m_collidable;

			// Save collisionfilterinfo by entity id
			UInt32 entityId = entity->m_uid;
			UInt8 count = GetSavedCollisionRefCount(entityId);
			if (count > 0) {
				// other hand already did the job - 'incRef'
				UInt32 savedInfo = GetSavedCollision(entityId);
				collisionInfoIdMap[entityId] = { savedInfo, count + 1 };
			}
			else {
				// Other hand hasn't affected this yet. Set its collision info
				collisionInfoIdMap[entityId] = { collidable->m_broadPhaseHandle.m_collisionFilterInfo, 1 };

				collidable->m_broadPhaseHandle.m_collisionFilterInfo &= 0x0000FFFF;
				collidable->m_broadPhaseHandle.m_collisionFilterInfo |= collisionGroup << 16;
				collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7F; // clear out layer
				collidable->m_broadPhaseHandle.m_collisionFilterInfo |= 56; // our custom layer
				// set bit 15. This way it won't collide with the player, but _will_ collide with other objects that also have bit 15 set (i.e. other things we pick up).
				collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (1 << 15); // Why bit 15? It's just the way the collision works.

				hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
			}
		}
	}

	NiNode *node = obj->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				SetCollisionInfoDownstream(child, collisionGroup);
			}
		}
	}
}

void SetCollisionInfoForAllCollisionInRefr(TESObjectREFR *refr, UInt32 collisionGroup)
{
	if (refr->loadedState && refr->loadedState->node) {
		SetCollisionInfoDownstream(refr->loadedState->node, collisionGroup);
	}
}


void ResetCollisionInfoDownstream(NiAVObject *obj, hkpCollidable *skipNode)
{
	if (obj->unk040) {
		auto collObj = (bhkCollisionObject *)obj->unk040;
		hkpRigidBody *entity = collObj->body->hkBody;
		hkpCollidable *collidable = &entity->m_collidable;
		if (collidable != skipNode) {
			UInt32 entityId = entity->m_uid;
			UInt8 refCount = GetSavedCollisionRefCount(entityId);
			if (refCount > 0) {
				if (refCount == 1) {
					// Only actually reset collision info if the other hand isn't involved
					UInt32 savedCollision = GetSavedCollision(entityId);

					// Restore only the original layer first, so it collides with everything except the player
					collidable->m_broadPhaseHandle.m_collisionFilterInfo &= ~0x7f;
					collidable->m_broadPhaseHandle.m_collisionFilterInfo |= (savedCollision & 0x7f);
					hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

					// Do not do a full check. What that means is it won't colide with the player until they stop colliding.
					collidable->m_broadPhaseHandle.m_collisionFilterInfo = savedCollision;
					hkpWorld_UpdateCollisionFilterOnEntity(entity->m_world, entity, HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
				}
				RemoveSavedCollision(entityId);
			}
		}
	}

	NiNode *node = obj->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			auto child = node->m_children.m_data[i];
			if (child) {
				ResetCollisionInfoDownstream(child, skipNode);
			}
		}
	}
}

void ResetCollisionInfoForAllCollisionInRefr(TESObjectREFR *refr, hkpCollidable *skipNode)
{
	if (refr->loadedState && refr->loadedState->node) {
		ResetCollisionInfoDownstream(refr->loadedState->node, skipNode);
	}
}
