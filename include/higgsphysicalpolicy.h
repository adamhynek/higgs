#ifndef HIGGS_PHYSICAL_POLICY_H
#define HIGGS_PHYSICAL_POLICY_H
#define HIGGS_PHYSICAL_POLICY_ABI_VERSION 2u
#include "higgstransactions.h"
#ifdef __cplusplus
extern "C"
{
#endif
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
#pragma pack(push, 8)
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
#pragma pack(pop)
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
       Exact version1 copied POD; existing client and sized-output rules apply. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryPhysicalPolicyInterfaceFn)(uint32_t, uint32_t,
                                                                                    HIGGS_PhysicalPolicyInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryPhysicalPolicyInterface(uint32_t, uint32_t,
                                                                        HIGGS_PhysicalPolicyInterface *);
#ifdef __cplusplus
}
#endif
#endif
