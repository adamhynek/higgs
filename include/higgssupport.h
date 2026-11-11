#ifndef HIGGS_SUPPORT_H
#define HIGGS_SUPPORT_H
#include "higgshandapi.h"
#ifdef __cplusplus
extern "C"
{
#endif
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
#pragma pack(push, 8)
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
#pragma pack(pop)
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
#ifdef __cplusplus
}
#endif
#endif
