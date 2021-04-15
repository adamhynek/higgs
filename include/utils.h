#pragma once

#include <unordered_set>

#include "skse64/NiNodes.h"
#include "skse64/GameData.h"
#include "skse64/GameExtraData.h"
#include "common/ITimer.h"

#include "RE/havok.h"
#include "math_utils.h"
#include "offsets.h"


#define VM_REGISTRY (*g_skyrimVM)->GetClassRegistry()


extern ITimer g_timer;
extern double g_currentFrameTime;
//extern double g_deltaTime;

NiAVObject * GetHighestParent(NiAVObject *node);
void updateTransformTree(NiAVObject * root, NiAVObject::ControllerUpdateContext *ctx);
void UpdateNodeTransformLocal(NiAVObject *node, const NiTransform &worldTransform);
void UpdateKeyframedNode(NiAVObject *node, NiTransform &transform);
void UpdateBoneMatrices(NiAVObject *obj);

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
void DumpVertices(std::vector<TriangleData> &triangles);

//float hkHalfToFloat(hkHalf half);
//hkHalf floatToHkHalf(float half);

bhkCollisionObject * GetCollisionObject(NiAVObject *obj);
NiPointer<bhkRigidBody> GetRigidBody(NiAVObject *obj);
bool DoesNodeHaveConstraint(NiNode *rootNode, NiAVObject *node);
bool DoesNodeHaveNode(NiAVObject *haystack, NiAVObject *target);
bool DoesRefrHaveNode(TESObjectREFR *ref, NiAVObject *node);
void GetDownstreamNodesNoCollision(NiAVObject *root, std::unordered_set<NiAVObject *> &targets);
bool IsSkinnedToNode(NiAVObject *skinnedRoot, NiAVObject *target);
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

extern RelocPtr<UInt64> unk_141E703BC;
extern RelocPtr<UInt64> unk_141E703B8;

class NiCloningProcess
{
public:
	NiCloningProcess() {
		unk18 = unk_141E703BC;
		unk48 = unk_141E703B8;
	}

	~NiCloningProcess() {
		CleanupCloneList1((uintptr_t)&unk38);
		CleanupCloneList2((uintptr_t)&unk08);
	}

	UInt64 unk00 = 0;
	UInt64 unk08 = 0; // Start of clone list 1?
	UInt64 unk10 = 0;
	UInt64 * unk18; // initd to RelocAddr(0x1E703BC)
	UInt64 unk20 = 0;
	UInt64 unk28 = 0;
	UInt64 unk30 = 0;
	UInt64 unk38 = 0; // Start of clone list 2?
	UInt64 unk40 = 0;
	UInt64 * unk48; // initd to RelocAddr(0x1E703B8)
	UInt64 unk50 = 0;
	UInt64 unk58 = 0;
	UInt8 copyType = 1; // 60 - CopyType - default 1
	UInt8 m_eAffectedNodeRelationBehavior = 0; // 61 - CloneRelationBehavior - default 0
	UInt8 m_eDynamicEffectRelationBehavior = 0; // 62 - CloneRelationBehavior - default 0
	char m_cAppendChar = '$'; // 64 - default '$'
	NiPoint3 scale = { 1.0f, 1.0f, 1.0f }; // 0x68 - default {1, 1, 1}
};

struct VRMeleeData
{
	UInt64 unk00;
	UInt64 unk08;
	NiPointer<bhkWorld> world; // 10
	NiPointer<NiNode> collisionNode; // 18
	NiPointer<NiAVObject> offsetNode; // 20
	UInt64 unk28; // default == 3?
	NiPoint3 position; // 30
	tArray<NiPoint3> unk40;
	tArray<NiPoint3> unk58;
	tArray<NiPoint3> unk70;
	UInt64 unk88;
	UInt64 unk80;
	UInt64 unk98;
	float unkA0; // linearVelocityThreshold when right, 0 when right?
	float linearVelocityThreshold; // A4
	float impactConfirmRumbleIntensity; // A8
	float impactConfirmRumbleDuration; // AC
	float impactRumbleIntensity; // B0
	float impactRumbleDuration; // B4
	float meleeForceMultLinear; // B8
	float unkBC; // default byte 0?
	float unkC0; // default 0
	float cooldown; // C4 - gets set to the cooldown, then ticks down, can (and will) get negative - default 0
	UInt32 unkC8; // default 0
	UInt32 unkCC;
};
static_assert(offsetof(VRMeleeData, collisionNode) == 0x18);
static_assert(offsetof(VRMeleeData, linearVelocityThreshold) == 0xA4);
static_assert(sizeof(VRMeleeData) == 0xD0);
