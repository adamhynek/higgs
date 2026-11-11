#ifndef HIGGS_HAND_OPERATIONS_H
#define HIGGS_HAND_OPERATIONS_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C"
{
#endif
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
#pragma pack(push, 8)
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
#pragma pack(pop)
    /* Closed primitives, source-integrated native adapter. Static support validates
     an actual raycast hit on the named fixed reference body, retains it, and pins
     HIGGS's existing keyframed hand collision target. No player movement, dynamic
     support, NPC endpoint, skeletal IK goal, custom geometry or new Havok constraint.
     Support/target require an empty physical hand. Ending/expiry removes only this
     operation's target. VRIK remains an independent visible pose owner.
     Accepted -> committed means native validation; PHYSICAL_CONFIRMED only after
     the existing hand actuator consumes the target / grip / drive policy. Haptics
     confirm queue acceptance, never measured hardware delivery. Native source has
     not been compiled/qualified without the compatible Havok dependency.
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
#ifdef __cplusplus
}
#endif
#endif
