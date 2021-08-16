#pragma once

#include <unordered_set>

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"
#include "skse64/GameExtraData.h"
#include "common/ITimer.h"

#include "RE/havok.h"
#include "RE/offsets.h"
#include "math_utils.h"


#define VM_REGISTRY (*g_skyrimVM)->GetClassRegistry()


extern ITimer g_timer;
extern double g_currentFrameTime;
//extern double g_deltaTime;

NiAVObject * GetHighestParent(NiAVObject *node);
void updateTransformTree(NiAVObject * root, NiAVObject::ControllerUpdateContext *ctx);
NiTransform GetLocalTransform(NiAVObject *node, const NiTransform &worldTransform, bool useOldParentTransform = false);
void UpdateNodeTransformLocal(NiAVObject *node, const NiTransform &worldTransform);
void UpdateKeyframedNode(NiAVObject *node, NiTransform &transform);
void UpdateBoneMatrices(NiAVObject *obj);

NiPointer<NiAVObject> GetTorsoNode(Actor *actor);

UInt32 GetFullFormID(const ModInfo * modInfo, UInt32 formLower);

inline bool IsMotionTypeMoveable(UInt8 motionType) {
	return (
		motionType == hkpMotion::MotionType::MOTION_DYNAMIC ||
		motionType == hkpMotion::MotionType::MOTION_SPHERE_INERTIA ||
		motionType == hkpMotion::MotionType::MOTION_BOX_INERTIA ||
		motionType == hkpMotion::MotionType::MOTION_THIN_BOX_INERTIA
		);
}
bool IsMoveableEntity(hkpEntity *entity);
bool IsObjectSelectable(hkpRigidBody *rigidBody, TESObjectREFR *ref);

bool HasGeometryChildren(NiAVObject *obj);

bool IsTwoHanded(const TESObjectWEAP *weap);
bool IsTwoHandable(const TESObjectWEAP *weap);
bool IsBow(const TESObjectWEAP *weap);
TESObjectWEAP * GetEquippedWeapon(Actor *actor, bool isOffhand);
bool IsUnarmed(TESForm *equippedObject);
std::pair<bool, bool> AreEquippedItemsValid(Actor *actor);

bool IsBipedIndexHigherPriority(int indexInQuestion, int indexToBeat);

double GetTime();

void PrintVector(const NiPoint3 &p);
void PrintQuat(const NiQuaternion &q);
void PrintSceneGraph(NiAVObject *node);
void PrintToFile(std::string entry, std::string filename);
void DumpVertices(std::vector<TriangleData> &triangles);

//float hkHalfToFloat(hkHalf half);
//hkHalf floatToHkHalf(float half);

bhkCollisionObject * GetCollisionObject(NiAVObject *obj);
NiPointer<bhkRigidBody> GetRigidBody(NiAVObject *obj);
bool DoesNodeHaveConstraint(NiNode *rootNode, NiAVObject *node);
bool DoesNodeHaveNode(NiAVObject *haystack, NiAVObject *target);
bool DoesRefrHaveNode(TESObjectREFR *ref, NiAVObject *node);
void GetDownstreamNodes(NiAVObject *root, std::unordered_set<NiAVObject *> &targets);
void GetDownstreamNodesNoCollision(NiAVObject *root, std::unordered_set<NiAVObject *> &targets);
bool IsSkinnedToNode(NiAVObject *skinnedRoot, NiAVObject *target);
NiPointer<bhkRigidBody> GetFirstRigidBody(NiAVObject *root);
UInt32 PlaySoundAtNode(BGSSoundDescriptorForm *sound, NiAVObject *node, const NiPoint3 &location);
const char * GetItemName(TESForm *form, BaseExtraList *extraList);
SInt32 GetItemId(TESForm * form, BaseExtraList * extraList);
EquipData GetWornItem(Actor* thisActor, UInt32 mask);
struct Hand * GetHandToShowRolloverFor();
void SetSelectedHandles(bool isLeftHanded, UInt32 handle);
void ReplaceBSString(BSString &replacee, std::string &replacer);
void SetGeometryAlphaDownstream(NiAVObject *root, float alpha);
