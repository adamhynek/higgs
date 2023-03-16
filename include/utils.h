#pragma once

#include <set>

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

inline void set_vtbl(void *object, void *vtbl) { *((void **)object) = ((void *)(vtbl)); }
inline UInt64 * get_vtbl(void *object) { return *((UInt64 **)object); }

inline void set_vfunc(void *object, UInt64 index, std::uintptr_t vfunc)
{
	UInt64 *vtbl = get_vtbl(object);
	vtbl[index] = vfunc;
}

template<class T>
inline T get_vfunc(void *object, UInt64 index)
{
	UInt64 *vtbl = get_vtbl(object);
	return (T)(vtbl[index]);
}

NiAVObject * GetHighestParent(NiAVObject *node);
NiTransform GetLocalTransformForDesiredWorldTransform(NiAVObject *node, const NiTransform &worldTransform, bool useOldParentTransform = false);
void UpdateNodeTransformLocal(NiAVObject *node, const NiTransform &worldTransform);
NiTransform GetRigidBodyTLocalTransform(bhkRigidBody *rigidBody);
void UpdateKeyframedNode(NiAVObject *node, NiTransform &transform);
void UpdateBoneMatrices(NiAVObject *obj);

inline VRMeleeData * GetVRMeleeData(bool isLeft) { return (VRMeleeData *)((UInt64)*g_thePlayer + 0x710 + (isLeft ? sizeof(VRMeleeData) : 0)); };

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
TESObjectWEAP *GetEquippedWeapon(Actor *actor, bool isOffhand);
TESObjectARMO *GetEquippedShield(Actor *actor, bool isOffhand);
SpellItem *GetEquippedSpell(Actor *actor, bool isOffhand);
bool IsUnarmed(TESForm *equippedObject);

bool IsBipedIndexHigherPriority(int indexInQuestion, int indexToBeat);
std::tuple<EquipData, int> GetEquipDataForBipedObject(Actor *actor, Biped *bipedData, TESForm *matchForm, int bipedIndex);

double GetTime();

bool VisitNodes(NiAVObject  *parent, std::function<bool(NiAVObject*, int)> functor, int depth = 0);

inline void ltrim(std::string &s) { s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !::isspace(ch); })); }
inline void rtrim(std::string &s) { s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !::isspace(ch); }).base(), s.end()); }
inline void trim(std::string &s) { ltrim(s); rtrim(s); }
std::set<std::string, std::less<>> SplitStringToSet(const std::string &s, char delim);

void PrintVector(const NiPoint3 &p);
void PrintQuat(const NiQuaternion &q);
void PrintSceneGraph(NiAVObject *node);
void PrintToFile(std::string entry, std::string filename);
void DumpVertices(std::vector<TriangleData> &triangles);

//float hkHalfToFloat(hkHalf half);
//hkHalf floatToHkHalf(float half);

NiPointer<bhkCollisionObject> GetCollisionObject(NiAVObject *obj);
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
NiPointer<NiAVObject> GetClosestParentWithCollision(NiAVObject *node);
NiPointer<BSFlattenedBoneTree> GetFlattenedBoneTree(NiAVObject *root);
NiPointer<BSFlattenedBoneTree> GetFlattenedBoneTree(TESObjectREFR *refr);
NiAVObject * GetNodeMatchingBoneTreeTransform(BSFlattenedBoneTree *tree, NiTransform *worldTransform);
void ModSpeedMult(Actor *actor, float amount);
void ConsumeSpellBook(PlayerCharacter *player, TESObjectBOOK *book);
inline TESRace * Actor_GetRace(Actor *actor) { return *((TESRace **)((UInt64)actor + 0x1F0)); }

struct SnapTurnState
{
	bool isSnapTurning; // 00
	float targetAngle; // 04
	float currentAngle; // 08
	UInt32 currentFrame; // 0C
	float turningSpeed; // 10
};
inline SnapTurnState &PlayerCharacter_GetSnapTurnState(PlayerCharacter *player) { return *(SnapTurnState *)((UInt64)player + 0x970); }

inline UInt32 GetCollisionLayer(UInt32 collisionFilterInfo) { return collisionFilterInfo & 0x7f; }
inline void SetCollisionLayer(hkUint32 &collisionFilterInfo, UInt32 layer) {
	collisionFilterInfo &= ~(0x7f); // zero out layer
	collisionFilterInfo |= (layer & 0x7f); // set layer
}
inline UInt32 GetCollisionLayer(hkpRigidBody *rigidBody) { return GetCollisionLayer(rigidBody->getCollisionFilterInfo()); }
inline void SetCollisionLayer(hkpRigidBody *rigidBody, UInt32 layer) { return SetCollisionLayer(rigidBody->getCollidableRw()->getBroadPhaseHandle()->m_collisionFilterInfo, layer); }

inline UInt32 GetPartNumber(UInt32 collisionFilterInfo) { return (collisionFilterInfo >> 8) & 0x1f; }
inline void SetPartNumber(hkUint32 &collisionFilterInfo, UInt32 partNumber) {
	collisionFilterInfo &= ~(0x1f00); // zero out part number
	collisionFilterInfo |= (partNumber & 0x1f) << 8; // set part number
}
