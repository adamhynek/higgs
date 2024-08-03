#pragma once

#include "skse64_common/Relocation.h"
#include "skse64/NiTypes.h"
#include "skse64/GameTypes.h"
#include "skse64/NiNodes.h"
#include "skse64/GameMenus.h"

#include "Physics/Collide/Query/Collector/PointCollector/hkpAllCdPointCollector.h"

#include "RE/havok.h"


typedef void(*_CleanupCloneList)(uintptr_t _this);
extern RelocAddr<_CleanupCloneList> CleanupCloneList1;
extern RelocAddr<_CleanupCloneList> CleanupCloneList2;

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
    UInt8 pad63;
    char m_cAppendChar = '$'; // 64 - default '$'
    NiPoint3 scale = { 1.0f, 1.0f, 1.0f }; // 0x68 - default {1, 1, 1}
};
static_assert(offsetof(NiCloningProcess, m_cAppendChar) == 0x64);
static_assert(offsetof(NiCloningProcess, scale) == 0x68);

struct VRMeleeData
{
    enum class SwingDirection : UInt32 {
        kNone = 0,
        kDown = 1,
        kLeft = 3,
        kRight = 4,
        kUp = 6
    };

    UInt64 unk00; // something to do with cooldown multipliers
    UInt64 unk08;
    NiPointer<bhkWorld> world; // 10
    NiPointer<NiNode> collisionNode; // 18
    NiPointer<NiAVObject> offsetNode; // 20
    UInt32 unk28; // default == 3? - could be an isLeft sort of thing
    UInt32 currentArrayOffset; // 2C - this is the offset in the arrays of the value for the current frame
    NiPoint3 position; // 30
    tArray<float> wandPositionDiffLengths; // 40 - lengths of values in subsequent array
    tArray<NiPoint3> wandPositionDiffs; // 58 - diffs of currentframe - lastframe of wandPositionsRoomspace
    tArray<NiPoint3> wandPositionsRoomspace; // 70
    tArray<Actor *> sweepActors; // 88 - which actors has the current swing hit. Only applies if the player has the 'Sweep' perk.
    float unkA0; // linearVelocityThreshold when main hand, 0 when offhand
    float linearVelocityThreshold; // A4
    float impactConfirmRumbleIntensity; // A8
    float impactConfirmRumbleDuration; // AC
    float impactRumbleIntensity; // B0
    float impactRumbleDuration; // B4
    float meleeForceMultLinear; // B8
    bool enableCollision; // BC - this value is read from and the collision node's collision is enabled/disabled
    bool applyImpulseOnHit; // BD - default true
    SwingDirection swingDirection; // C0 - default 0
    float cooldown; // C4 - gets set to the cooldown, then ticks down, can (and will) get negative - default 0
    float powerAttackCooldown; // C8 - default 0
    UInt32 unkCC;
};
static_assert(offsetof(VRMeleeData, collisionNode) == 0x18);
static_assert(offsetof(VRMeleeData, linearVelocityThreshold) == 0xA4);
static_assert(sizeof(VRMeleeData) == 0xD0);

struct CrosshairPickData
{
    UInt32 unk00;
    UInt32 leftHandle1; // 04
    UInt32 rightHandle1; // 08
    UInt32 unk0C;
    UInt32 leftHandle2; // 10
    UInt32 rightHandle2; // 14
    UInt32 unk18;
    UInt32 leftHandle3; // 1C
    UInt32 rightHandle3; // 20
    UInt8 unk24[0x50 - 0x24];
    bhkRigidBody *rigidBodies[2]; // 50 - slot depends on hand
    UInt8 unk60[0x78 - 0x60];
    NiPointer<bhkSimpleShapePhantom> sphere; // 78
};
static_assert(offsetof(CrosshairPickData, rigidBodies) == 0x50);
static_assert(offsetof(CrosshairPickData, sphere) == 0x78);

struct SoundData
{
    UInt32 id = -1; // 00
    UInt8 unk04 = 0;
    UInt32 unk08 = 0; // set to 1 when playing
};
static_assert(sizeof(SoundData) == 0x0C);

struct BGSImpactManager
{
    void *vtbl; // 00
    BSTEventSink<void> combatImpactEventSink; // 08 - BSTEventSink<BGSCombatImpactEvent>
    BSTEventSink<void> collisionSoundEventSink; // 10 - BSTEventSink<BGSCollisionSoundEvent>
    // sound event passes in ptr to skyrimhavokmaterial id in rdx (actually it's the 2 material ids, then at 0x14 (I think?) is the magnitude of the impact or something - it's used to determine which sound to play, high or low)
};

struct BSFlattenedBoneTree : NiNode
{
    struct BoneEntry
    {
        NiTransform local; // 00
        NiTransform world; // 34
        UInt16 unk68;
        UInt16 unk6A;
        UInt16 unk6C;
        UInt16 unk6E;
        NiAVObject *node; // 70
        BSFixedString nodeName; // 78
    };
    static_assert(sizeof(BoneEntry) == 0x80);

    UInt32 numBones; // 150
    UInt32 unk154; // 154
    BoneEntry * boneEntries; // 158
};

struct WorldSpaceMenu : IMenu
{
    virtual void SetHandle(UInt32 handle);
    virtual void Unk_0A();
    virtual void SetupMenuNode();
    virtual NiNode * GetMenuParentNode();
    virtual void InitializeNode();
};

struct BGSAttackData : NiRefObject
{
    struct AttackData  // ATKD
    {
        enum class AttackFlag : UInt32
        {
            kNone = 0,
            kIgnoreWeapon = 1 << 0,
            kBashAttack = 1 << 1,
            kPowerAttack = 1 << 2,
            kChargeAttack = 1 << 3,
            kRotatingAttack = 1 << 4,
            kContinuousAttack = 1 << 5,
            kOverrideData = (UInt32)1 << 31
        };

        // members
        float                                       damageMult;     // 00
        float                                       attackChance;   // 04
        SpellItem *attackSpell;    // 08
        UInt32										flags;          // 10
        float                                       attackAngle;    // 14
        float                                       strikeAngle;    // 18
        float                                       staggerOffset;  // 1C
        BGSKeyword *attackType;     // 20
        float                                       knockDown;      // 28
        float                                       recoveryTime;   // 2C
        float                                       staminaMult;    // 30
        std::uint32_t                               pad34;          // 34
    };
    static_assert(sizeof(AttackData) == 0x38);

    BSFixedString event;  // 10 - ATKE
    AttackData    data;   // 18 - ATKD
};
static_assert(sizeof(BGSAttackData) == 0x50);

struct HitData
{
    enum class Flag : UInt32
    {
        kBlocked = 1 << 0,
        kBlockWithWeapon = 1 << 1,
        kBlockCandidate = 1 << 2,
        kCritical = 1 << 3,
        kCriticalOnDeath = 1 << 4,
        kFatal = 1 << 5,
        kDismemberLimb = 1 << 6,
        kExplodeLimb = 1 << 7,
        kCrippleLimb = 1 << 8,
        kDisarm = 1 << 9,
        kDisableWeapon = 1 << 10,
        kSneakAttack = 1 << 11,
        kIgnoreCritical = 1 << 12,
        kPredictDamage = 1 << 13,
        //kPredictBaseDamage = 1 << 14,
        kBash = 1 << 14,
        kTimedBash = 1 << 15,
        kPowerAttack = 1 << 16,

        kOffhand = 1 << 17,

        kMeleeAttack = 1 << 18,
        kRicochet = 1 << 19,
        kExplosion = 1 << 20
    };

    // members
    NiPoint3                 hitPosition;             // 00
    NiPoint3                 hitDirection;            // 0C
    UInt32                   aggressor;               // 18
    UInt32                   target;                  // 1C
    UInt32                   sourceRef;               // 20
    std::uint32_t            pad24;                   // 24
    NiPointer<BGSAttackData> attackData;              // 28
    TESObjectWEAP *weapon;                  // 30
    std::uint64_t            unk38;                   // 38
    std::uint64_t            unk40;                   // 40
    std::uint32_t            unk48;                   // 48
    float                    healthDamage;            // 4C
    float                    totalDamage;             // 50
    float                    physicalDamage;          // 54
    float                    targetedLimbDamage;      // 58
    float                    percentBlocked;          // 5C
    float                    resistedPhysicalDamage;  // 60
    float                    resistedTypedDamage;     // 64
    float                    stagger;                 // 68
    float                    sneakAttackBonus;        // 6C
    float                    bonusHealthDamageMult;   // 70
    float                    pushBack;                // 74
    float                    reflectedDamage;         // 78
    float                    criticalDamageMult;      // 7C
    UInt32                   flags;                   // 80
    std::uint32_t            equipIndex;              // 84
    std::uint32_t            material;                // 88
    std::uint32_t            damageLimb;              // 8C
};
static_assert(sizeof(HitData) == 0x90);

typedef void(*Actor_RemoveItem)(TESObjectREFR *_this, UInt32 *outHandle, TESBoundObject* a_item, SInt32 a_count, UInt32 a_reason, BaseExtraList* a_extraList, TESObjectREFR* a_moveToRef, const NiPoint3* a_dropLoc, const NiPoint3* a_rotate);
typedef TESAmmo * (*Actor_GetCurrentAmmo)(Actor *_this);
typedef void(*Actor_PickUpObject)(Actor *_this, TESObjectREFR* a_object, std::int32_t a_count, bool a_arg3, bool a_playSound); // arg3 == false
typedef void(*Actor_DropObject)(Actor *_this, UInt32 *outHandle, const TESBoundObject* a_object, BaseExtraList* a_extraList, std::int32_t a_count, const NiPoint3* a_dropLoc, const NiPoint3* a_rotate);
typedef void(*Actor_GetLinearVelocity)(Actor *_this, NiPoint3 &velocity);
typedef bool(*TESBoundObject_GetActivateText)(TESBoundObject *_this, TESObjectREFR* activator, BSString& text);
typedef bool(*_ActorValueOwner_RestoreActorValue)(ActorValueOwner *_this, UInt32 modifier, UInt64 actorValue, float value);
typedef bool(*_Projectile_UpdateImpactFromCollector)(Projectile *_this, hkpAllCdPointCollector *collector);
