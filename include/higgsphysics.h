#ifndef HIGGSPHYSICS_H
#define HIGGSPHYSICS_H
#include "higgshandapi.h"
#include "higgstransactions.h"
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push, 8)

/* Independent C ABI. This file does not extend or replace IHiggsInterface001. */
#include <stdint.h>

#if defined(_WIN32)
#define HIGGS_SYNERGY_CALL __cdecl
#else
#define HIGGS_SYNERGY_CALL
#endif


#define HIGGS_SYNERGY_ABI_VERSION 1u
#define HIGGS_SYNERGY_CAP_SCOPED_RESTRICTIONS UINT64_C(1)
#define HIGGS_SYNERGY_MAX_CLIENTS 64u
#define HIGGS_SYNERGY_MAX_LEASES 256u
#define HIGGS_SYNERGY_MIN_LEASE_MS 1u
#define HIGGS_SYNERGY_MAX_LEASE_MS 60000u

typedef uint32_t HIGGS_SynergyStatus;
typedef uint64_t HIGGS_SynergyClient;
typedef uint64_t HIGGS_SynergyLease;

enum {
    HIGGS_SYNERGY_OK = 0,
    HIGGS_SYNERGY_INVALID_ARGUMENT = 1,
    HIGGS_SYNERGY_UNSUPPORTED_VERSION = 2,
    HIGGS_SYNERGY_BUFFER_TOO_SMALL = 3,
    HIGGS_SYNERGY_INVALID_CLIENT = 4,
    HIGGS_SYNERGY_INVALID_LEASE = 5,
    HIGGS_SYNERGY_WRONG_OWNER = 6,
    HIGGS_SYNERGY_NOT_READY = 7,
    HIGGS_SYNERGY_STALE_WORLD = 8,
    HIGGS_SYNERGY_CAPACITY = 9,
    HIGGS_SYNERGY_INTERNAL_ERROR = 10
};

/* Physical hands; these never mean dominant/off hand. */
enum {
    HIGGS_SYNERGY_HAND_LEFT = 1u,
    HIGGS_SYNERGY_HAND_RIGHT = 2u,
    HIGGS_SYNERGY_HAND_BOTH = 3u
};

enum {
    HIGGS_SYNERGY_SUPPRESS_HAND_INTERACTIONS = 1u,
    HIGGS_SYNERGY_SUPPRESS_WEAPON_COLLISION = 2u,
    HIGGS_SYNERGY_FORCE_WEAPON_COLLISION = 4u
};

typedef struct HIGGS_SynergyClientInfo {
    uint32_t structSize;
    uint32_t reserved; /* Must be zero. */
    char name[64];    /* Copied, nonempty, NUL terminated; descriptive, not authority. */
} HIGGS_SynergyClientInfo;

typedef struct HIGGS_SynergyLeaseRequest {
    uint32_t structSize;
    uint32_t physicalHands;
    uint32_t flags;
    uint32_t durationMs;
    uint64_t expectedWorldGeneration;
} HIGGS_SynergyLeaseRequest;

typedef struct HIGGS_SynergyRenewRequest {
    uint32_t structSize;
    uint32_t durationMs;
    uint64_t expectedWorldGeneration;
    HIGGS_SynergyLease lease;
} HIGGS_SynergyRenewRequest;

typedef struct HIGGS_SynergyLeaseInfo {
    uint32_t structSize;
    uint32_t reserved;
    HIGGS_SynergyLease lease;
    uint64_t worldGeneration;
    uint64_t expiresAtMs; /* Provider monotonic clock; compare to snapshot sampledAtMs. */
} HIGGS_SynergyLeaseInfo;

typedef struct HIGGS_SynergyHandState {
    uint32_t physicalHand;
    uint32_t handInteractionsDisabled;
    uint32_t weaponCollisionDisabled;
    uint32_t forceWeaponCollisionEnabled;
    uint32_t legacyHandDisabled;
    uint32_t legacyWeaponCollisionDisabled;
    uint32_t legacyForceWeaponCollisionEnabled;
    uint32_t scopedFlags;
    uint32_t activeScopedLeases;
} HIGGS_SynergyHandState;

typedef struct HIGGS_SynergyEffectiveState {
    uint32_t structSize;
    uint32_t worldReady;
    uint64_t stateGeneration;
    uint64_t worldGeneration;
    uint64_t sampledAtMs;
    HIGGS_SynergyHandState hands[2]; /* [0] physical left, [1] physical right. */
} HIGGS_SynergyEffectiveState;

typedef struct HIGGS_SynergyInterface {
    uint32_t structSize;
    uint32_t abiVersion;
    uint64_t capabilities;
    uint32_t maxClients;
    uint32_t maxLeases;
    uint32_t minLeaseMs;
    uint32_t maxLeaseMs;
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *RegisterClient)(const HIGGS_SynergyClientInfo*, HIGGS_SynergyClient*);
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *UnregisterClient)(HIGGS_SynergyClient);
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *AcquireLease)(HIGGS_SynergyClient, const HIGGS_SynergyLeaseRequest*, HIGGS_SynergyLeaseInfo*);
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *RenewLease)(HIGGS_SynergyClient, const HIGGS_SynergyRenewRequest*, HIGGS_SynergyLeaseInfo*);
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *ReleaseLease)(HIGGS_SynergyClient, HIGGS_SynergyLease);
    HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *CopyEffectiveState)(HIGGS_SynergyClient, HIGGS_SynergyEffectiveState*);
} HIGGS_SynergyInterface;
/* Resolve this symbol with GetProcAddress on higgs_vr.dll. Only exact v1 is accepted.
   tableSize is the caller's buffer capacity. No output is written on query failure.
   Record outputs need structSize initialized; BUFFER_TOO_SMALL sets its required size.
   Pointers are caller-owned and retained only for this call. Handles are runtime only.
   Stop/join the client's callers, then UnregisterClient before unloading its module. */
typedef HIGGS_SynergyStatus (HIGGS_SYNERGY_CALL *HIGGS_QuerySynergyInterfaceFn)(uint32_t version, uint32_t tableSize, HIGGS_SynergyInterface*);
HIGGS_SynergyStatus HIGGS_SYNERGY_CALL HIGGS_QuerySynergyInterface(uint32_t version, uint32_t tableSize, HIGGS_SynergyInterface* outInterface);

#define HIGGS_HAND_OPERATIONS_ABI_VERSION 1u
#define HIGGS_HAND_OP_CAP_STATIC_WORLD_SUPPORT UINT64_C(1)
#define HIGGS_HAND_OP_CAP_PHYSICAL_HAND_TARGET UINT64_C(2)
#define HIGGS_HAND_OP_CAP_GRIP_TRANSFORM UINT64_C(4)
#define HIGGS_HAND_OP_CAP_DRIVE_CAP UINT64_C(8)
#define HIGGS_HAND_OP_CAP_HAPTIC UINT64_C(16)
#define HIGGS_HAND_OP_CAP_HAND_GEOMETRY UINT64_C(32)
#define HIGGS_HAND_OP_CAP_SECONDARY_GRIP_TRANSFORM UINT64_C(64)
    /* With capability64, GRIP_TRANSFORM also accepts an active game-weapon
       secondary support: itemRefId is its weapon base form, grant is mandatory,
       and handGeneration must match that exact secondary support observation.
       The per-Hand desired transform is independent of the main-hand grip.
       Confirmation follows native actuation; restoration requires the same
       support start/weapon/generation and the still-matching last written value. */
    enum
    {
        HIGGS_OP_WORLD_SUPPORT = 1,
        HIGGS_OP_PHYSICAL_HAND_TARGET = 2,
        HIGGS_OP_GRIP_TRANSFORM = 3,
        HIGGS_OP_DRIVE_CAP = 4,
        HIGGS_OP_HAPTIC = 5,
        HIGGS_OP_HAND_GEOMETRY = 6
    };
    enum
    {
        HIGGS_OP_ACTIVE = 1,
        HIGGS_OP_APPLIED = 2
    };
    typedef struct HIGGS_HandOperation
    {
        uint32_t structSize, kind, physicalHand, itemRefId;
        uint64_t epoch, handGeneration;
        HIGGS_HandGrant grant;
        uint32_t durationMs, reserved;
        float position[3]; /* support ray endpoint / physical target world position;
                              grip transform: item-node position in physical-hand space. */
        float rotation[9]; /* row major unit rotation for target/grip; support captures current rotation. */
        float scale;       /* physical target/grip, finite >0. */
        float maxDistance; /* world units: support reach, target/support break distance, 1..200. */
        float linearForceCap, angularForceCap; /* existing grab motors' native force units, >=0. */
        float hapticStrength;                  /* existing HIGGS actuator units, 0..1; durationMs<=1000. */
        uint32_t reserved2;
        float geometryHalfExtents[3]; /* meters, each 0.005..0.5, existing box shape only. */
        uint32_t reserved3;
    } HIGGS_HandOperation;
    typedef struct HIGGS_HandOperationState
    {
        uint32_t structSize, flags;
        HIGGS_HandRequestId requestId;
        HIGGS_HandClient owner;
        HIGGS_HandGrant grant;
        uint64_t epoch, supportGeneration, observedAtNs;
        HIGGS_HandOperation operation; /* validated native target, support hit and normal below. */
        float supportNormal[3];
        uint32_t reserved;
        float supportPosition[3];
        uint32_t reserved2; /* actual world-space ray hit, not wrist origin. */
    } HIGGS_HandOperationState;
    typedef struct HIGGS_HandOperationsInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *RequestOperation)(HIGGS_HandClient, const HIGGS_HandOperation *,
                                                            HIGGS_HandRequestId *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *EndOperation)(HIGGS_HandClient, HIGGS_HandRequestId, uint64_t epoch);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyOperation)(HIGGS_HandClient, HIGGS_HandRequestId,
                                                         HIGGS_HandOperationState *);
    } HIGGS_HandOperationsInterface;
    /* Closed primitives, source-integrated native adapter. Static support validates
     an actual raycast hit on the named fixed reference body, retains it, and pins
     HIGGS's existing keyframed hand collision target. No player movement, dynamic
     support, NPC endpoint, skeletal IK goal, custom geometry or new Havok constraint.
     Support/target require an empty physical hand. Ending/expiry removes only this
     operation's target. VRIK remains an independent visible pose owner.
     Accepted -> committed means native validation; PHYSICAL_CONFIRMED only after
     the existing hand actuator consumes the target / grip / drive policy. Haptics
     confirm queue acceptance, never measured hardware delivery. Runtime behavior requires validation in the game.
     Requires matching owned grant, or atomically acquires a new bounded grant.
     Grip/drive effects stop on hold generation change and are restored at the next
     safe main update; only one of each kind per hand may be active. Copy may return
     inactive historical state; ACTIVE/APPLIED are separate flags. Any thread may
     request/copy/end; native dispatch and retained endpoint validation run in HIGGS
     main update. Existing hand-body keyframe phase applies physical targets.
     Observe action results through HandInterface; records retained in its bounded
     256-request history. No new arbitrary control kinds may be registered. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandOperationsInterfaceFn)(uint32_t, uint32_t,
                                                                                    HIGGS_HandOperationsInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandOperationsInterface(uint32_t, uint32_t,
                                                                        HIGGS_HandOperationsInterface *);

enum
    {
        HIGGS_SUPPORT_MOVING_WORLD = 1,
        HIGGS_SUPPORT_PLANCK_HAND = 2,
        HIGGS_SUPPORT_PLAYER_WEAPON = 3,
        HIGGS_CONSTRAINT_HELD_TO_WORLD = 4,
        HIGGS_CONSTRAINT_HELD_TO_PLANCK = 5
    };
    enum
    {
        HIGGS_SUPPORT_PENDING = 1,
        HIGGS_SUPPORT_ACTIVE = 2,
        HIGGS_SUPPORT_ENDING = 3,
        HIGGS_SUPPORT_RELEASED = 4,
        HIGGS_SUPPORT_BROKEN = 5
    };
    enum
    {
        HIGGS_SUPPORT_TARGET_VALID = 1,
        HIGGS_SUPPORT_NATIVE_APPLIED = 2,
        HIGGS_SUPPORT_CONSTRAINT_ATTACHED = 4,
        HIGGS_SUPPORT_FORCE_OBSERVED = 8,
        HIGGS_SUPPORT_FORCE_SATURATED = 16,
        HIGGS_SUPPORT_DRIVE_LIMITED = 32
    };
    enum
    {
        HIGGS_SUPPORT_BREAK_OWNER = 1,
        HIGGS_SUPPORT_BREAK_EPOCH = 2,
        HIGGS_SUPPORT_BREAK_ENDPOINT = 3,
        HIGGS_SUPPORT_BREAK_DISTANCE = 4,
        HIGGS_SUPPORT_BREAK_CONSTRAINT = 5,
        HIGGS_SUPPORT_BREAK_TIMEOUT = 6
    };
    typedef struct HIGGS_ProviderBodyRef
    {
        uint64_t epoch, actorGeneration, bodyGeneration;
        uint32_t actorHandle, bodyIndex;
    } HIGGS_ProviderBodyRef;
    typedef struct HIGGS_SupportRequest
    {
        uint32_t structSize, kind, physicalHand, durationMs;
        uint64_t epoch, handGeneration;
        HIGGS_HandGrant grant;
        uint32_t referenceFormId, sourceItemRefId;
        HIGGS_ProviderBodyRef providerBody;
        uint64_t expectedGeometryRevision;
        uint32_t maxObservationAgeMs, reserved;
        float targetLocalPoint[3], sourceLocalPoint[3]; /* meters in the respective rigid-body frames. */
        float targetLocalRotation[9];                   /* unit body-from-hand rotation, or target constraint frame. */
        float maxDistanceWorld, linearForceCap, angularForceCap;
    } HIGGS_SupportRequest;
    typedef struct HIGGS_SupportState
    {
        uint32_t structSize, stage, flags, reason;
        HIGGS_HandRequestId requestId;
        HIGGS_HandClient owner;
        HIGGS_HandGrant grant;
        uint64_t epoch, endpointGeneration, geometryRevision, requestRevision, appliedRevision, validatedAtNs;
        HIGGS_SupportRequest request;
        float anchorPosition[3], handPosition[3], handRotation[9], handScale;
        float effectiveLinearCap, effectiveAngularCap, linearForce, angularTorque;
    } HIGGS_SupportState;
    typedef struct HIGGS_SupportUpdate
    {
        uint32_t structSize, reserved;
        uint64_t epoch, expectedRevision;
        float targetLocalPoint[3];
        float targetLocalRotation[9];
    } HIGGS_SupportUpdate;
    typedef struct HIGGS_SupportInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *RequestSupport)(HIGGS_HandClient, const HIGGS_SupportRequest *,
                                                          HIGGS_HandRequestId *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *UpdateSupport)(HIGGS_HandClient, HIGGS_HandRequestId,
                                                         const HIGGS_SupportUpdate *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *EndSupport)(HIGGS_HandClient, HIGGS_HandRequestId, uint64_t);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopySupport)(HIGGS_HandClient, HIGGS_HandRequestId, HIGGS_SupportState *);
    } HIGGS_SupportInterface;
    /* Finite physical primitives, no arbitrary drivers. Capability bit(kind-1).
       Dynamic world support follows a retained non-NPC body-local anchor. NPC hand
       support and held-item constraints acquire an independent HIGGS-owned PLANCK
       endpoint lease and use its graph/world-locked borrowed-body transaction.
       Player weapon support reuses HIGGS's existing secondary grip actuator.
       Constraints act only on the matching held item body; PLANCK owns NPC physics.
       End is asynchronous: RELEASED means actual owned constraint removal, not just
       accepted cancellation. Target getters fail closed on stale validation age or
       endpoint invalidation. No player locomotion, wounds, fall or skeletal IK policy.
       Update changes only local grip frame within the same body/epoch/generation;
       requested/applied revisions distinguish proposals from actual native use.
       Force/torque fields require FORCE_OBSERVED; caps alone are not measurements.
       Source grips remain held when a dependent support/constraint ends. Physical
       release is separate. All native mutations use HIGGS main maintenance; compatible
       PLANCK interop additionally requires its observed actor-maintenance thread.
       Exact v1 sized POD tables, shared hand clients and runtime-only identities. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QuerySupportInterfaceFn)(uint32_t, uint32_t,
                                                                             HIGGS_SupportInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QuerySupportInterface(uint32_t, uint32_t, HIGGS_SupportInterface *);

#define HIGGS_PHYSICAL_POLICY_ABI_VERSION 2u
#define HIGGS_POLICY_CAP_PROPERTIES UINT64_C(1)
#define HIGGS_POLICY_CAP_GRAB_MODE UINT64_C(2)
#define HIGGS_POLICY_CAP_FINGER_SUPPRESSION UINT64_C(4)
#define HIGGS_POLICY_CAP_CONTROLLER_NODE_CULL UINT64_C(8)
    enum
    {
        HIGGS_POLICY_PROPERTIES = 1,
        HIGGS_POLICY_GRAB_MODE = 2,
        HIGGS_POLICY_PRESENTATION = 3
    };
    enum
    {
        HIGGS_PROPERTY_MASS = 1,
        HIGGS_PROPERTY_INERTIA = 2,
        HIGGS_PROPERTY_CENTER_OF_MASS = 4
    };
    enum
    {
        HIGGS_GRAB_FORCE_PHYSICS = 1,
        HIGGS_GRAB_IGNORE_GLOBAL_FORCE = 2
    };
    enum
    {
        HIGGS_PRESENTATION_SUPPRESS_FINGERS = 1,
        HIGGS_PRESENTATION_CULL_CONTROLLER = 2
    };
    enum
    {
        HIGGS_POLICY_PENDING = 1,
        HIGGS_POLICY_ACTIVE = 2,
        HIGGS_POLICY_ENDING = 3,
        HIGGS_POLICY_ENDED = 4,
        HIGGS_POLICY_BROKEN = 5
    };
    enum
    {
        HIGGS_POLICY_APPLIED = 1,
        HIGGS_POLICY_RESTORED = 2,
        HIGGS_POLICY_EXTERNAL_WRITE = 4,
        HIGGS_POLICY_RESTORED_MASS = 8,
        HIGGS_POLICY_RESTORED_INERTIA = 16,
        HIGGS_POLICY_RESTORED_COM = 32
    };
    typedef struct HIGGS_PhysicalPolicyRequest
    {
        uint32_t structSize, kind, physicalHand, durationMs;
        uint64_t epoch, handGeneration;
        HIGGS_HandGrant grant;
        HIGGS_ItemIdentity item;
        uint32_t fields, grabMode, presentation, reserved;
        float massMultiplier, inertiaMultiplier, centerOfMassOffset[3];
        uint32_t reserved2;
    } HIGGS_PhysicalPolicyRequest;
    typedef struct HIGGS_PhysicalPolicyValues
    {
        float mass, inertia[9], centerOfMass[3];
    } HIGGS_PhysicalPolicyValues;
    typedef struct HIGGS_PhysicalPolicyState
    {
        uint32_t structSize, stage, flags, reason;
        HIGGS_HandRequestId policy;
        HIGGS_HandClient owner;
        uint64_t epoch, expiresAtNs, revision, appliedRevision, bodyGeneration;
        int32_t effectivePriority;
        uint32_t appliedFields;
        HIGGS_PhysicalPolicyRequest request;
        HIGGS_PhysicalPolicyValues original, effective;
    } HIGGS_PhysicalPolicyState;
    typedef struct HIGGS_PhysicalPolicyInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *Request)(HIGGS_HandClient, const HIGGS_PhysicalPolicyRequest *,
                                                   HIGGS_HandRequestId *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *Update)(HIGGS_HandClient, HIGGS_HandRequestId, uint64_t expectedRevision,
                                                  const HIGGS_PhysicalPolicyRequest *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *End)(HIGGS_HandClient, HIGGS_HandRequestId, uint64_t epoch);
        HIGGS_HandStatus(HIGGS_HAND_CALL *Copy)(HIGGS_HandClient, HIGGS_HandRequestId, HIGGS_PhysicalPolicyState *);
    } HIGGS_PhysicalPolicyInterface;
    /* Effect leases never acquire input grants. Properties require exact currently
       held native item/hand generation. Mass and inertia are a COUPLED exclusive
       resource: setMass may alter inertia, so the native owner sets/readbacks both.
       COM is a separate exclusive resource. Multipliers use one captured baseline;
       COM offsets are body-local meters. Updates use revision CAS and renew a live
       lease; they cannot revive expiry/hold/model/world loss. Restore only components
       whose values still match the owner's last write; preserve the foreign
       counterpart through coupled setters and report per-component restoration.
       Grab mode only overrides the global ForcePhysicsGrab preference for that hand;
       actor/constraint safety still forces physics. Presentation claims only HIGGS
       finger-animation suppression and the existing controller-node subtree cull
       bit. It does NOT hide VRIK skinned hand meshes or drive body/finger bones.
       Same resource conflicts return CONFLICT; there is no hidden priority preemption.
       Effective user priority is diagnostic, separate from input arbitration.
       End is asynchronous until native restoration. Client/epoch/tracking/optional
       grant loss end effects. Native capabilities absent in portable builds.
       Exact version2 copied POD; existing client and sized-output rules apply. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryPhysicalPolicyInterfaceFn)(uint32_t, uint32_t,
                                                                                    HIGGS_PhysicalPolicyInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryPhysicalPolicyInterface(uint32_t, uint32_t,
                                                                        HIGGS_PhysicalPolicyInterface *);

enum
    {
        HIGGS_HAPTIC_QUEUED = 1,
        HIGGS_HAPTIC_ACTIVE = 2,
        HIGGS_HAPTIC_COMPLETED = 3,
        HIGGS_HAPTIC_CANCELLED = 4
    };
    typedef struct HIGGS_HapticRequest
    {
        uint32_t structSize, physicalHand;
        uint64_t epoch, contextKey;
        uint32_t delayMs, durationMs;
        float startStrength, endStrength;
    } HIGGS_HapticRequest;
    typedef struct HIGGS_HapticState
    {
        uint32_t structSize, stage;
        uint64_t requestId, epoch;
        HIGGS_HandClient owner;
        uint32_t physicalHand, submittedSamples, suppressedSamples, unavailableSamples;
        int32_t effectivePriority;
        uint32_t reserved;
        uint64_t dueAtNs, endsAtNs, lastSubmittedAtNs;
    } HIGGS_HapticState;
    typedef struct HIGGS_HapticsInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *RequestHaptic)(HIGGS_HandClient, const HIGGS_HapticRequest *, uint64_t *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CancelHaptic)(HIGGS_HandClient, uint64_t);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyHaptic)(HIGGS_HandClient, uint64_t, HIGGS_HapticState *);
    } HIGGS_HapticsInterface;
    /* One coordinated output per physical hand, including all legacy HIGGS calls.
       Higher configured client priority wins; equal-priority envelopes combine by
       maximum strength, independent of submission order. Legacy HIGGS uses the
       configurable client name HIGGS with fallback 0. contextKey!=0 replaces only
       that owner's same-hand/same-context request. No hand/input grant is acquired.
       Delays <=5000ms, durations 1..5000ms, strengths [0,1]. Client/world revocation
       and cancellation remove scheduled effects; an issued hardware pulse cannot be
       recalled. submittedSamples confirms native API submission only, not delivery.
       Queued/active requests bounded256; oldest terminal records may be evicted.
       Exact copied v1; same client/table lifetime and sized-output rules as hand API. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHapticsInterfaceFn)(uint32_t, uint32_t,
                                                                             HIGGS_HapticsInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHapticsInterface(uint32_t, uint32_t, HIGGS_HapticsInterface *);

#define HIGGS_HAND_PARTS_ABI_VERSION 2u
#define HIGGS_HAND_PART_LIMIT 8u
#define HIGGS_HAND_SOURCE_LIMIT 17u
enum { HIGGS_PART_BOX = 1 };
enum { HIGGS_PART_PENDING = 1, HIGGS_PART_ACTIVE = 2, HIGGS_PART_ENDING = 3,
       HIGGS_PART_RELEASED = 4, HIGGS_PART_BROKEN = 5 };
enum { HIGGS_PART_REASON_OWNER = 1, HIGGS_PART_REASON_EPOCH = 2,
       HIGGS_PART_REASON_TRACKING = 3, HIGGS_PART_REASON_MODEL = 4,
       HIGGS_PART_REASON_TIMEOUT = 5, HIGGS_PART_REASON_NATIVE = 6 };
enum { HIGGS_PART_BODY_PRESENT = 1, HIGGS_PART_COLLISION_ENABLED = 2,
       HIGGS_PART_KEYFRAME_ISSUED = 4, HIGGS_PART_ACTUAL_POSE = 8 };
enum { HIGGS_PART_SOURCE_VALID = 1 };
typedef struct HIGGS_HandPartSource {
    uint32_t sourceId, flags;
    float position[3], rotation[9], modelScale;
} HIGGS_HandPartSource;
typedef struct HIGGS_HandPartSources {
    uint32_t structSize, physicalHand, sourceCount, reserved;
    uint64_t epoch, sampleId, observedAtNs, modelGeneration, handGeneration;
    uint64_t trackingGeneration, trackingObservationId, trackedAtNs;
    HIGGS_HandPartSource sources[HIGGS_HAND_SOURCE_LIMIT];
} HIGGS_HandPartSources;
typedef struct HIGGS_HandPartDefinition {
    uint32_t partId, shape, sourceId, reserved;
    float localPosition[3], localRotation[9], halfExtentsMetres[3];
} HIGGS_HandPartDefinition;
typedef struct HIGGS_HandPartSetRequest {
    uint32_t structSize, physicalHand, partCount, durationMs;
    uint64_t epoch, sampleId, modelGeneration, expectedRevision, setId;
    HIGGS_HandPartDefinition parts[HIGGS_HAND_PART_LIMIT];
} HIGGS_HandPartSetRequest;
typedef struct HIGGS_HandPartBody {
    uint32_t partId, flags, bodyKey, reserved;
    uint64_t bodyGeneration;
    float position[3], rotation[9], targetPosition[3], targetRotation[9];
} HIGGS_HandPartBody;
typedef struct HIGGS_HandPartSetState {
    uint32_t structSize, stage, reason, physicalHand;
    uint64_t owner, setId, epoch, requestRevision, appliedRevision,
        modelGeneration, sourceSampleId, observedAtNs, expiresAtNs;
    uint32_t bodyCount, reserved;
    HIGGS_HandPartBody bodies[HIGGS_HAND_PART_LIMIT];
} HIGGS_HandPartSetState;

typedef struct HIGGS_HandPartContact {
    uint32_t structSize, reserved;
    uint64_t contactSequence;
    HIGGS_HandPartAttribution part;
} HIGGS_HandPartContact;
typedef struct HIGGS_HandPartsInterface {
    uint32_t structSize, abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopySources)(HIGGS_HandClient, uint32_t, HIGGS_HandPartSources*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *SetParts)(HIGGS_HandClient, const HIGGS_HandPartSetRequest*, HIGGS_HandPartSetState*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *EndParts)(HIGGS_HandClient, uint64_t, uint64_t);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyParts)(HIGGS_HandClient, uint64_t, HIGGS_HandPartSetState*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *ResolveBody)(HIGGS_HandClient, uint64_t, uint32_t, uint64_t, HIGGS_HandPartAttribution*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyContactAttribution)(HIGGS_HandClient, uint64_t, uint64_t, HIGGS_HandPartContact*);
} HIGGS_HandPartsInterface;
/* Copied POD v1. Capabilities: bit0 fitted BOX parts, bit1 bone sources, bit2
   resolved-palm-local sources, bit3 live body attribution, bit4 contact attribution.
   CopyContactAttribution matches the exact ObservationInterface contact sequence
   and epoch, from a bounded512 ring; missing/evicted records return NOT_FOUND.
   It retains the identity at that contact even after the native body was removed.
   No thumb/palm replacement,
   capsule factory, arbitrary driver, provider collision-filter or damage policy.
   One exclusive provider set per physical hand; 1..8 unique nonzero part IDs.
   Source0 is the actual resolved HIGGS palm body; source1 is the skeletal hand;
   sources2..16 are thumb/index/middle/ring/little joints0..2 in that order.
   Sources have world positions in game units and proper row-major rotations.
   localPosition uses game units in source axes, with NO implicit model scaling;
   fitted half extents are metres. modelScale is evidence for the provider's fitter.
   SetParts copies a proposal matching the current source epoch/sample/model, max
   age100ms. setId=expectedRevision=0 creates; updates require both exact values.
   Updating renews a 1..60000ms lease. HIGGS owns keyframing, movement compensation,
   native filter policy and teardown; PPB owns fitting. Collision follows HIGGS's
   palm enablement, so holding/two-hand transitions do not create a second actuator.
   ACTIVE/appliedRevision means native bodies and keyframe commands were applied,
   NOT that physics reached the requested pose. ACTUAL_POSE and target are separate.
   Geometry-size changes recreate only the changed native body, changing its
   bodyGeneration. bodyKey is observed Havok UID, epoch-scoped; bodyGeneration is
   this API's nonreused runtime lifetime. ResolveBody requires both for ABA safety.
   EndParts is asynchronous; RELEASED/BROKEN confirms owned body removal. Loading,
   model replacement, tracking loss, timeout and unregister invalidate the set.
   No callbacks run under the service registry lock. Never persist runtime IDs. */
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandPartsInterfaceFn)(uint32_t,uint32_t,HIGGS_HandPartsInterface*);
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandPartsInterface(uint32_t,uint32_t,HIGGS_HandPartsInterface*);

#define HIGGS_OWN_CONTACT_VERSION 1u
#define HIGGS_OWN_CONTACT_CAP_HELD UINT64_C(1)
#define HIGGS_OWN_CONTACT_CAP_EQUIPPED_UID UINT64_C(2)
#define HIGGS_OWN_CONTACT_CAP_EQUIPPED_COLLIDER UINT64_C(4)
enum { HIGGS_CONTACT_ITEM_HELD=1, HIGGS_CONTACT_ITEM_EQUIPPED=2 };
enum { HIGGS_CONTACT_CLASS_WEAPON=1,HIGGS_CONTACT_CLASS_SHIELD=2,HIGGS_CONTACT_CLASS_OTHER=4 };
enum { HIGGS_COSMETIC_SOUND=1,HIGGS_COSMETIC_HAPTIC=2 };
enum { HIGGS_COSMETIC_ACTIVE=1,HIGGS_COSMETIC_ENDED=2 };
enum { HIGGS_COSMETIC_VERIFIED_PAIR=1,HIGGS_COSMETIC_SERVICE_NO_DAMAGE=2,HIGGS_COSMETIC_NATIVE_RESPONSE_DISABLED=4,HIGGS_COSMETIC_PLANCK_HIT_PATH_SKIPPED=8 };
enum { HIGGS_CONTACT_SOURCE_AVAILABLE=1,HIGGS_CONTACT_SOURCE_ABSENT=2,HIGGS_CONTACT_SOURCE_UNSUPPORTED=3,HIGGS_CONTACT_SOURCE_AMBIGUOUS=4,HIGGS_CONTACT_SOURCE_NOT_READY=5 };
enum { HIGGS_CONTACT_PROVENANCE_HELD_REFERENCE=1,HIGGS_CONTACT_PROVENANCE_EQUIPPED_UID=2,HIGGS_CONTACT_PROVENANCE_EQUIPPED_COLLIDER=4 };
typedef struct HIGGS_ContactItem {
    uint32_t structSize,physicalHand,source,itemClass,baseFormId,worldRefId,provenance,reserved;
    uint64_t itemInstance,itemGeneration,bodyGeneration,handGeneration;
} HIGGS_ContactItem;
typedef struct HIGGS_CosmeticContactPolicy {
    uint32_t structSize,feedbackKinds;
    uint64_t epoch,leaseId,expectedRevision;
    uint32_t leftSources,rightSources,leftClasses,rightClasses;
    float minRelativeSpeed,minClosingSpeed;
    uint32_t cooldownMs,leaseMs,maxObservationAgeMs,provenanceMask,reserved;
} HIGGS_CosmeticContactPolicy;
typedef struct HIGGS_CosmeticContactState {
    uint32_t structSize,stage;
    uint64_t leaseId,owner,epoch,revision,expiresAtNs;
    uint32_t reason,reserved;
} HIGGS_CosmeticContactState;
typedef struct HIGGS_CosmeticContact {
    uint32_t structSize,flags;
    uint64_t sequence,epoch,leaseId,policyRevision,producerObservation,producerObservedAtNs,observedAtNs,mainFrame,physicsTick;
    HIGGS_ContactItem left,right;
    float positionMetres[3],rightToLeftNormal[3],leftPointVelocity[3],rightPointVelocity[3],relativeSpeed,closingSpeed;
    uint32_t feedbackKinds,reserved;
} HIGGS_CosmeticContact;
typedef struct HIGGS_CosmeticContactQuery {
    uint32_t structSize,reserved;
    uint64_t epoch,afterSequence,leaseId;
    uint32_t feedbackKinds,reserved2;
} HIGGS_CosmeticContactQuery;
typedef struct HIGGS_ContactItemFrame {
    uint32_t structSize,itemCount;
    uint64_t epoch,observationId,observedAtNs;
    uint32_t sources[4]; /* left-held,left-equipped,right-held,right-equipped */
    HIGGS_ContactItem items[4];
} HIGGS_ContactItemFrame;
typedef struct HIGGS_CosmeticContactInterface {
    uint32_t structSize,abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus(HIGGS_HAND_CALL *RequestPolicy)(HIGGS_HandClient,const HIGGS_CosmeticContactPolicy*,HIGGS_CosmeticContactState*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *EndPolicy)(HIGGS_HandClient,uint64_t,uint64_t);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyPolicy)(HIGGS_HandClient,uint64_t,HIGGS_CosmeticContactState*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyContacts)(HIGGS_HandClient,const HIGGS_CosmeticContactQuery*,HIGGS_CosmeticContact*,uint32_t,uint32_t*);
    HIGGS_HandStatus(HIGGS_HAND_CALL *CopyItems)(HIGGS_HandClient,HIGGS_ContactItemFrame*);
} HIGGS_CosmeticContactInterface;
/* Passive, opt-in cosmetic evidence only. No input grant, damage, solver change,
   audio or hardware pulse is produced by requesting a policy. Source masks use
   HELD=1 / EQUIPPED=2; class masks are WEAPON=1 / SHIELD=2 / OTHER=4.
   Identities name HIGGS-owned contact-item lifetimes, not serialized inventory IDs.
   Both endpoints have proved provider-owned bodies and nonreused generations.
   Provenance masks explicitly select retained HELD_REFERENCE episodes, existing
   unambiguous EQUIPPED_UID evidence, or HIGGS-owned EQUIPPED_COLLIDER episodes.
   Missing/duplicate worn UID cannot satisfy UID provenance; a policy may instead
   opt into collider provenance. A collider episode is not inventory identity.
   Old/model/world/hand changes and unsupported equipment fail closed. Sources
   and positions are sampled observations, not virtual IK goals. Publication
   age bounds are not proof of fresh hardware callbacks or controller tracking.
   Producer velocities are measured at the SAME contact point (including angular
   motion), in world-axis metres/s. Normal points right item -> left item.
   Closing speed is max(0,-dot(left-right velocity,normal)); relative speed is
   |left-right|, never the maximum of independent body speeds.
   Consumer polls in its safe phase, selects sound/assets, and schedules haptics
   through HIGGS's ordinary haptics service. A copied event is admitted evidence,
   not proof that sound/pulse was delivered. SERVICE_NO_DAMAGE means this service
   applies no damage; it does not certify a caller's separate combat outcome.
   The other flags report PLANCK's actual response-disable/hit-path branch.
   mainFrame/physicsTick are producer-assigned observations, not service timers.
   Coverage is contacts observed by the connected PLANCK listener, not all world
   manifolds. Capability bits name endpoint provenance, not contact enumeration.
   Old/ended/replaced-source events are not returned. CopyContacts retains at
   most 512 newest admitted events across leases; afterSequence is exclusive,
   and overflow drops oldest events. Required count is the current matching
   count (not a total-ever count); short capacity returns BUFFER_TOO_SMALL.
   CopyItems includes its producer time and availability of each source; callers
   must inspect these, not infer freshness from an earlier successful copy.
   Histories/leases are bounded; no external callback is retained.
   Exact v1 sized PODs, packing8, no native pointer in this gameplay interface. */
typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryCosmeticContactInterfaceFn)(uint32_t,uint32_t,HIGGS_CosmeticContactInterface*);
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryCosmeticContactInterface(uint32_t,uint32_t,HIGGS_CosmeticContactInterface*);

#pragma pack(pop)
#ifdef __cplusplus
}
#endif
#endif
