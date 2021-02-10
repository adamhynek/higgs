#pragma once

#include <unordered_set>

#include "RE/havok.h"

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"
#include "skse64/GameExtraData.h"
#include "common/ITimer.h"


#define VM_REGISTRY (*g_skyrimVM)->GetClassRegistry()


extern ITimer g_timer;
extern double g_currentFrameTime;
//extern double g_deltaTime;

NiAVObject * GetHighestParent(NiAVObject *node);
void updateTransformTree(NiAVObject * root, NiAVObject::ControllerUpdateContext *ctx);
void UpdateNodeTransformLocal(NiAVObject *node, const NiTransform &worldTransform);
void UpdateKeyframedNode(NiAVObject *node, const NiTransform &transform);

NiPointer<NiAVObject> GetTorsoNode(Actor *actor);

UInt32 GetFullFormID(const ModInfo * modInfo, UInt32 formLower);

bool IsAllowedCollidable(const hkpCollidable *collidable);
bool HasGeometryChildren(NiAVObject *obj);

bool IsTwoHanded(const TESObjectWEAP *weap);
bool IsBow(const TESObjectWEAP *weap);
std::pair<bool, bool> AreEquippedItemsValid(Actor *actor);

double GetTime();

void PrintVector(const NiPoint3 &p);
void PrintSceneGraph(NiAVObject *node);
void PrintToFile(std::string entry, std::string filename);

//float hkHalfToFloat(hkHalf half);
//hkHalf floatToHkHalf(float half);

NiPointer<bhkRigidBody> GetRigidBody(NiAVObject *obj);
bool DoesNodeHaveConstraint(NiNode *rootNode, NiAVObject *node);
bool DoesNodeHaveNode(NiAVObject *haystack, NiAVObject *target);
bool DoesRefrHaveNode(TESObjectREFR *ref, NiAVObject *node);
bool IsSkinnedToNode(NiAVObject *skinnedRoot, NiAVObject *target);
void GetAllSkinnedNodes(NiAVObject *root, std::unordered_set<NiAVObject *> &skinnedNodes);
NiPointer<bhkRigidBody> GetFirstRigidBody(NiAVObject *root);
UInt32 PlaySoundAtNode(BGSSoundDescriptorForm *sound, NiAVObject *node, const NiPoint3 &location);
const char * GetItemName(TESForm *form, BaseExtraList *extraList);
SInt32 GetItemId(TESForm * form, BaseExtraList * extraList);
EquipData GetWornItem(Actor* thisActor, UInt32 mask);
struct Grabber * GetGrabberToShowRolloverFor();
void SetSelectedHandles(bool isLeftHanded, UInt32 handle);
void ReplaceBSString(BSString &replacee, std::string &replacer);

typedef void(*Actor_RemoveItem)(TESObjectREFR *_this, UInt32 *outHandle, TESBoundObject* a_item, SInt32 a_count, UInt32 a_reason, BaseExtraList* a_extraList, TESObjectREFR* a_moveToRef, const NiPoint3* a_dropLoc, const NiPoint3* a_rotate);
typedef void(*Actor_PickUpObject)(Actor *_this, TESObjectREFR* a_object, std::int32_t a_count, bool a_arg3, bool a_playSound); // arg3 == false
typedef void(*Actor_DropObject)(Actor *_this, UInt32 *outHandle, const TESBoundObject* a_object, BaseExtraList* a_extraList, std::int32_t a_count, const NiPoint3* a_dropLoc, const NiPoint3* a_rotate);
typedef void(*Actor_GetLinearVelocity)(Actor *_this, NiPoint3 &velocity);
typedef bool(*TESBoundObject_GetActivateText)(TESBoundObject *_this, TESObjectREFR* activator, BSString& text);

typedef void(*_Update3DPosition)(TESObjectREFR *_this, bool warp);
