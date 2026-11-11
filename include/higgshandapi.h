#ifndef HIGGSHANDAPI_H
#define HIGGSHANDAPI_H
#include "higgsabitypes.h"
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push, 8)

/* Additive, copied C ABI. Official001, restrictions v1 and motion v1 are unchanged. */
#define HIGGS_HAND_ABI_VERSION 2u
#define HIGGS_HAND_CAP_SNAPSHOT UINT64_C(1)
#define HIGGS_HAND_CAP_INPUT_BROKER UINT64_C(2)
#define HIGGS_HAND_CAP_OWNED_GRANTS UINT64_C(4)
#define HIGGS_HAND_CAP_EXECUTOR_RECEIPTS UINT64_C(8)
#define HIGGS_HAND_CAP_PHYSICAL_REQUESTS UINT64_C(16)
#define HIGGS_HAND_CAP_INTERACTION_EVENTS UINT64_C(32)
    /* occupancy is meaningful ONLY when both HELD_KNOWN and EQUIPMENT_KNOWN are set. */
    enum
    {
        HIGGS_OCCUPANCY_EMPTY = 0,
        HIGGS_OCCUPANCY_GAME_EQUIPPED = 1,
        HIGGS_OCCUPANCY_HIGGS_HELD = 2,
        HIGGS_OCCUPANCY_BOTH = 3
    };
    enum
    {
        HIGGS_AVAILABILITY_UNKNOWN = 0,
        HIGGS_AVAILABILITY_READY = 1,
        HIGGS_AVAILABILITY_RESERVED = 2,
        HIGGS_AVAILABILITY_UNAVAILABLE = 3
    };
    enum
    {
        HIGGS_OBS_HELD_KNOWN = 1,
        HIGGS_OBS_EQUIPMENT_KNOWN = 2,
        HIGGS_OBS_PALM_VALID = 4,
        HIGGS_OBS_TRACKING_VALID = 8,
        HIGGS_OBS_SECONDARY_SUPPORT = 16,
        HIGGS_OBS_NATIVE_GRIP_TARGET = 32,
        HIGGS_OBS_NATIVE_TRIGGER_TARGET = 64
    };
    enum
    {
        HIGGS_HAND_REASON_NONE = 0,
        HIGGS_HAND_REASON_OBSERVATION = 1,
        HIGGS_HAND_REASON_RESTRICTION = 2,
        HIGGS_HAND_REASON_MENU = 4,
        HIGGS_HAND_REASON_TRACKING = 8,
        HIGGS_HAND_REASON_EXISTING_USE = 16
    };
    enum
    {
        HIGGS_INPUT_GRIP = 1,
        HIGGS_INPUT_TRIGGER = 2
    };
    enum
    {
        HIGGS_INPUT_DOWN = 1,
        HIGGS_INPUT_UP = 2
    };
    enum
    {
        HIGGS_DECISION_USER_PRIORITY = 1u,
        HIGGS_DECISION_CLIENT_PRIORITY = 2u,
        HIGGS_DECISION_REGISTRATION_ORDER = 4u, /* reserved for earlier drafts */
        HIGGS_DECISION_CANDIDATE_ORDER = 8u, /* offers of the same recognizer for one provider */
        HIGGS_DECISION_STABLE_ID = 16u,
        HIGGS_DECISION_NATIVE = 32u
    };
    enum
    {
        HIGGS_ACTION_ACCEPTED = 1,
        HIGGS_ACTION_COMMITTED = 2,
        HIGGS_ACTION_COMPLETED = 3,
        HIGGS_ACTION_REJECTED = 4,
        HIGGS_ACTION_CANCELLED = 5
    };
    enum
    {
        HIGGS_RECEIPT_INVENTORY_COMMITTED = 1,
        HIGGS_RECEIPT_PHYSICAL_CONFIRMED = 2,
        HIGGS_RECEIPT_COMPENSATED = 4
    };
    enum
    {
        HIGGS_EVENT_GRABBED = 1,
        HIGGS_EVENT_RELEASED = 2,
        HIGGS_EVENT_SECONDARY_STARTED = 3,
        HIGGS_EVENT_SECONDARY_ENDED = 4,
        HIGGS_EVENT_ACTION_RESULT = 5,
        HIGGS_EVENT_RESET = 6,
        HIGGS_EVENT_CONSUMPTION_REQUESTED = 7,
        HIGGS_EVENT_STASH_REQUESTED = 8,
        HIGGS_EVENT_INPUT_RELEASED = 9, HIGGS_EVENT_INPUT_CANCELLED = 10
    };
    enum
    {
        HIGGS_PHYSICAL_GRAB = 1,
        HIGGS_PHYSICAL_RELEASE = 2,
        HIGGS_PHYSICAL_SECONDARY_SUPPORT = 3 /* existing HIGGS close weapon selection only. */
    };

    typedef struct HIGGS_HandClientInfo
    {
        uint32_t structSize;
        int32_t userPriority;
        char name[64]; /* Unique stable client/INI ID, trimmed and case-insensitive; excludes ',' and '='. */
    } HIGGS_HandClientInfo;
    typedef struct HIGGS_HandRecord
    {
        uint32_t physicalHand, observationMask, occupancy, availability;
        uint32_t unavailableReasons, heldRefId, equippedFormId, reserved;
        uint64_t generation, poseObservationId, poseObservedAtNs;
        HIGGS_HandGrant grant; /* Public reservation correlation, never a mutation grant. */
        HIGGS_HandClient owner; /* Public provider label, never a client credential. */
        float palmPosition[3]; /* Skyrim world units, world axes, resolved HIGGS palm. */
        float palmRotation[9]; /* row-major, no VRIK final-pose guarantee. */
    } HIGGS_HandRecord;
    typedef struct HIGGS_HandSnapshot
    {
        uint32_t structSize, worldReady;
        uint64_t epoch, generation, observationId, sampledAtNs;
        HIGGS_HandRecord hands[2]; /* physical left, right; copied together after hand update. */
    } HIGGS_HandSnapshot;
    typedef struct HIGGS_HandInput
    {
        uint32_t structSize, physicalHand, button, edge;
        uint64_t inputId, epoch, handGeneration, poseObservationId, observedAtNs;
        HIGGS_HandSnapshot snapshot; /* Exact retained observation used for recognition. */
    } HIGGS_HandInput;
    typedef struct HIGGS_ActionCandidate
    {
        uint32_t structSize, physicalHands;
        uint64_t inputId, epoch, handGeneration;
        HIGGS_HandExecutor executor;
        uint64_t actionId, contextId, contextGeneration;
        uint32_t grantDurationMs, reserved; /* 1..60000. Priority comes from client registration. */
    } HIGGS_ActionCandidate;
    typedef struct HIGGS_ActionRequest
    {
        uint32_t structSize, physicalHands;
        HIGGS_HandRequestId requestId;
        HIGGS_HandClient owner;
        HIGGS_HandExecutor executor;
        HIGGS_HandGrant grant;
        uint64_t epoch, inputId, actionId, contextId, contextGeneration, poseObservationId;
    } HIGGS_ActionRequest;
    typedef struct HIGGS_ActionResult
    {
        uint32_t structSize, stage;
        HIGGS_HandRequestId requestId;
        HIGGS_HandExecutor executor;
        uint64_t epoch, inputId;
        uint32_t receiptFlags, itemRefId, quantity, reason;
    } HIGGS_ActionResult;
    typedef struct HIGGS_InputDecision
    {
        uint32_t structSize, consumed;
        uint64_t inputId, epoch;
        HIGGS_HandClient owner; /* Public provider label. */
        HIGGS_HandGrant grant; /* Public reservation correlation. */
        HIGGS_HandRequestId requestId;
        int32_t effectivePriority;
        uint32_t reason;
    } HIGGS_InputDecision;
    typedef struct HIGGS_HandEvent
    {
        uint32_t structSize, type, physicalHand, itemRefId;
        uint64_t eventId, epoch, observationId, poseObservationId, observedAtNs;
        HIGGS_ActionResult result; /* ACTION_RESULT, plus original press/request identifiers on input termination. */
        HIGGS_HandRecord hand;     /* exact observation for grab/release/support events. */
    } HIGGS_HandEvent;
    typedef struct HIGGS_PhysicalRequest
    {
        uint32_t structSize, kind, physicalHand, itemRefId;
        uint64_t epoch, handGeneration;
        HIGGS_HandGrant grant;
        uint32_t timeoutMs, reserved;
    } HIGGS_PhysicalRequest;
    typedef void(HIGGS_HAND_CALL *HIGGS_RecognizerCallback)(const HIGGS_HandInput *, void *);
    typedef void(HIGGS_HAND_CALL *HIGGS_ExecutorCallback)(const HIGGS_ActionRequest *, void *);
    typedef void(HIGGS_HAND_CALL *HIGGS_HandEventCallback)(const HIGGS_HandEvent *, void *);
    typedef struct HIGGS_HandInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *RegisterClient)(const HIGGS_HandClientInfo *, HIGGS_HandClient *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *UnregisterClient)(HIGGS_HandClient);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopySnapshot)(HIGGS_HandClient, HIGGS_HandSnapshot *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *SubscribeRecognizer)(HIGGS_HandClient, HIGGS_RecognizerCallback, void *,
                                                               HIGGS_HandRegistration *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RegisterExecutor)(HIGGS_HandClient, HIGGS_ExecutorCallback, void *,
                                                            HIGGS_HandExecutor *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *SubscribeEvents)(HIGGS_HandClient, HIGGS_HandEventCallback, void *,
                                                           HIGGS_HandRegistration *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RemoveRegistration)(HIGGS_HandClient, uint64_t);
        HIGGS_HandStatus(HIGGS_HAND_CALL *SubmitCandidate)(HIGGS_HandClient, const HIGGS_ActionCandidate *,
                                                           HIGGS_HandRequestId *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *ReportResult)(HIGGS_HandClient, const HIGGS_ActionResult *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyActionResult)(HIGGS_HandClient, HIGGS_HandRequestId,
                                                            HIGGS_ActionResult *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyInputDecision)(HIGGS_HandClient, uint64_t, HIGGS_InputDecision *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *ReleaseGrant)(HIGGS_HandClient, HIGGS_HandGrant);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RequestPhysical)(HIGGS_HandClient, const HIGGS_PhysicalRequest *,
                                                           HIGGS_HandRequestId *);
    } HIGGS_HandInterface;
    /* All copied reads and registration/receipt calls serialize and may run on any
     thread. Recognizers/executors run synchronously on the controller callback
     thread BEFORE HIGGS defaults. Recognizers may ONLY read/submit candidates or
     mutate registrations; inventory mutation belongs to the winning executor.
     SubmitCandidate is valid only inside that client's recognizer for this input.
     External executors must queue engine work to their safe phase and report actual
     results; callbacks do not authorize unsafe engine calls. No pointers are retained
     except registered callbacks/context, which must remain alive until drainage.
     RemoveRegistration/UnregisterClient from outside ALL provider callbacks drain
     in-flight calls before OK. From any provider callback removal deactivates at
     once and returns PENDING: call again outside callbacks for confirmed drainage.
     A removed token remains a tombstone until drainage is confirmed. Registering
     during dispatch takes effect on the NEXT input/event. Never unload on PENDING.
     Epoch changes invalidate grants/pending requests; registration survives loads.
     Deadlines use provider steady clock. Consumers must renew by a new action after
     expiry; no hardware timestamp or callback-silence freshness is promised.
     Physical requests are queued, dispatched in HIGGS's main update and completed
     only after an actual matching hold transition. Acceptance is not completion.
     Successful outputs copy known bytes only; failures leave output unchanged except
     BUFFER_TOO_SMALL sets structSize. No C++ exceptions may cross callbacks.
     Exact v2 table only; sizes/layout immutable. Stop callers before DLL unload. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandInterfaceFn)(uint32_t, uint32_t, HIGGS_HandInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandInterface(uint32_t version, uint32_t tableSize,
                                                              HIGGS_HandInterface *);

#define HIGGS_HAND_ROUTING_ABI_VERSION 2u
    typedef struct HIGGS_HandRoutingInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        /* Executor owner explicitly authorizes another client's recognizer. Hands is
           a mask, not a grant. Authority is checked again at selection and dispatch. */
        HIGGS_HandStatus(HIGGS_HAND_CALL *AuthorizeRecognizer)(HIGGS_HandClient executorOwner, HIGGS_HandExecutor,
                                                               uint64_t recognizerPublicLabel,
                                                               uint32_t physicalHands);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RevokeRecognizer)(HIGGS_HandClient executorOwner, HIGGS_HandExecutor,
                                                            uint64_t recognizerPublicLabel);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CanRoute)(HIGGS_HandClient recognizerOwner, HIGGS_HandExecutor,
                                                    uint32_t physicalHands);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CancelRecognizedAction)(HIGGS_HandClient recognizerOwner, HIGGS_HandRequestId,
                                                                  uint64_t epoch, uint32_t reason);
    } HIGGS_HandRoutingInterface;
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandRoutingInterfaceFn)(uint32_t, uint32_t,
                                                                                 HIGGS_HandRoutingInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandRoutingInterface(uint32_t, uint32_t, HIGGS_HandRoutingInterface *);
/* Routed grant/request owner is the assigned executor's client. Recognizer and
   executor may copy results; only executor owner may report/release/use grant.
   A narrower mask requires Revoke then Authorize. Revocation invalidates pending uncommitted routed actions, preserving a
   completed physical hold. It does not release a held item or undo inventory. */

#define HIGGS_GRANT_MUTATION_ABI_VERSION 2u
    typedef struct HIGGS_GrantedMutationRequest
    {
        uint32_t structSize, commitStage; /* COMMITTED or COMPLETED; recorded before fence exit. */
        HIGGS_HandRequestId requestId;
        HIGGS_HandExecutor executor;
        uint64_t epoch;
        HIGGS_HandGrant grant;
    } HIGGS_GrantedMutationRequest;
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_GrantedMutationCallback)(const HIGGS_ActionRequest *,
                                                                             const HIGGS_HandSnapshot *, void *);
    typedef struct HIGGS_GrantMutationInterface
    {
        uint32_t structSize, abiVersion;
        HIGGS_HandStatus(HIGGS_HAND_CALL *WithAuthorizedAction)(HIGGS_HandClient, const HIGGS_GrantedMutationRequest *,
                                                                HIGGS_GrantedMutationCallback, void *,
                                                                HIGGS_ActionResult *);
    } HIGGS_GrantMutationInterface;
    /* Public snapshot IDs never authorize a mutation. The addressed request retains its own grant. Exact v2. Runs synchronously only on HIGGS's observed main-maintenance thread.
       Validates assigned executor/request/epoch and every currently owned hand grant
       under the native mutation fence. Callback runs OUTSIDE the broker state mutex;
       no revocation, expiry or world reset may invalidate authority until it returns.
       Same-grant reentrant revoke/remove returns PENDING. Return OK only after the
       domain mutation actually commits (e.g. saved placement CAS); HIGGS records the
       requested stage before releasing the fence. Output is that committed receipt.
       Callback failure/exception cannot promise rollback of its arbitrary effects.
       Inventory transfer/consumption uses TransactionsInterface and its item/permit
       rules; this fence itself creates no item, inventory or physical receipt flag.
       No callback/context is retained. On failure output remains untouched except
       BUFFER_TOO_SMALL sets its required structSize. Stop callers before DLL unload. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryGrantMutationInterfaceFn)(uint32_t, uint32_t,
                                                                                   HIGGS_GrantMutationInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryGrantMutationInterface(uint32_t, uint32_t,
                                                                       HIGGS_GrantMutationInterface *);

#define HIGGS_AUTHORIZED_DISPATCH_ABI_VERSION 2u
    typedef struct HIGGS_AuthorizedDispatchRequest
    {
        uint32_t structSize, reserved;
        HIGGS_HandRequestId requestId;
        HIGGS_HandExecutor executor;
        uint64_t epoch;
        HIGGS_HandGrant grant;
    } HIGGS_AuthorizedDispatchRequest;
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_AuthorizedDispatchCallback)(const HIGGS_ActionRequest *,
                                                                            const HIGGS_HandSnapshot *, void *);
    typedef struct HIGGS_AuthorizedDispatchInterface
    {
        uint32_t structSize, abiVersion;
        HIGGS_HandStatus(HIGGS_HAND_CALL *Invoke)(HIGGS_HandClient, const HIGGS_AuthorizedDispatchRequest *,
                                                HIGGS_AuthorizedDispatchCallback, void *);
    } HIGGS_AuthorizedDispatchInterface;
    /* Public snapshot IDs never authorize a mutation. The addressed request retains its own grant. Exact v2, independently negotiated. Invoke runs only on HIGGS's observed
       main-maintenance thread. It checks the selected executor/request/epoch,
       input context and every currently owned, known and available hand resource
       under the native mutation fence, then claims this request once BEFORE
       calling outside the broker state mutex. Duplicate invocation is CONFLICT,
       including reentry and after callback failure/exception. The callback is
       synchronous; no pointer/context is retained. It may queue ordinary child
       physical/transaction requests using the grant. Same-grant revocation,
       removal and ReportResult return PENDING while the callback is entered.
       Callback status is returned (exception becomes INTERNAL_ERROR); invocation
       creates no commit/completion/receipt, rejection or rollback promise. Report
       factual results after return or use the existing child transaction's facts.
       WithAuthorizedAction remains the separate actual-domain-commit fence.
       Requests are copied before callback entry. Stop callers before DLL unload. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryAuthorizedDispatchInterfaceFn)(
        uint32_t, uint32_t, HIGGS_AuthorizedDispatchInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryAuthorizedDispatchInterface(
        uint32_t, uint32_t, HIGGS_AuthorizedDispatchInterface *);
#ifdef __cplusplus
static_assert(sizeof(HIGGS_AuthorizedDispatchRequest) == 40, "authorized dispatch request ABI");
static_assert(sizeof(HIGGS_AuthorizedDispatchInterface) == 16, "authorized dispatch table ABI (x64)");
#endif

enum
    {
        HIGGS_USE_HAND_GOALS = 1,
        HIGGS_USE_GAME_EQUIPMENT = 2,
        HIGGS_USE_HELD_OBJECT = 3,
        HIGGS_USE_SECONDARY_SUPPORT = 4,
        HIGGS_USE_WORLD_SUPPORT = 5,
        HIGGS_USE_CONSTRAINT = 6,
        HIGGS_USE_CONSUMPTION = 7
    };
    typedef struct HIGGS_HandUseRequest
    {
        uint32_t structSize, kind, physicalHands, durationMs;
        uint64_t epoch, contextId, contextGeneration;
        HIGGS_HandExecutor executor;
    } HIGGS_HandUseRequest;
    typedef struct HIGGS_HandUseState
    {
        uint32_t structSize, kind, active, reserved;
        uint64_t epoch, expiresAtNs;
        HIGGS_ActionRequest authority;
    } HIGGS_HandUseState;
    typedef struct HIGGS_HandUseInterface
    {
        uint32_t structSize, abiVersion;
        uint64_t capabilities;
        HIGGS_HandStatus(HIGGS_HAND_CALL *RequestHandUse)(HIGGS_HandClient, const HIGGS_HandUseRequest *,
                                                          HIGGS_HandUseState *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *RenewHandUse)(HIGGS_HandClient, HIGGS_HandRequestId, uint64_t, uint32_t,
                                                        HIGGS_HandUseState *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *CopyHandUse)(HIGGS_HandClient, HIGGS_HandRequestId, HIGGS_HandUseState *);
        HIGGS_HandStatus(HIGGS_HAND_CALL *EndHandUse)(HIGGS_HandClient, HIGGS_HandRequestId, uint64_t);
    } HIGGS_HandUseInterface;
    /* Externally initiated continuous uses (activation, contact or domain events)
       require no fabricated controller press. This finite catalogue authorizes hand
       resources and reserves all named hands atomically; acceptance is NOT physical
       actuation. Apply supported goals/grips/constraints through their own receipts.
       The assigned generic executor is notified once in HIGGS main maintenance with
       inputId=0, actionId=kind and copied context identity. Use GrantMutationInterface
       for a fenced domain-state commit. Renew validates current ownership and cannot
       revive expired, reset, unregistered or tracking-invalid grants. Ending removes
       this use's grants/effects, never implicitly releases an already held item.
       This is a reservation primitive, not permission for a custom physical driver.
       Durations1..60000ms, exact copied v1, existing shared-client/registration rules. */
    typedef HIGGS_HandStatus(HIGGS_HAND_CALL *HIGGS_QueryHandUseInterfaceFn)(uint32_t, uint32_t,
                                                                             HIGGS_HandUseInterface *);
    HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandUseInterface(uint32_t, uint32_t, HIGGS_HandUseInterface *);

#define HIGGS_HAND_USE_PROPOSALS_ABI_VERSION 1u
#define HIGGS_HAND_USE_PROPOSALS_CAP_PRIORITY_ROUNDS UINT64_C(1)
enum {
    HIGGS_USE_PROPOSAL_PENDING = 1,
    HIGGS_USE_PROPOSAL_ACCEPTED = 2,
    HIGGS_USE_PROPOSAL_REJECTED = 3,
    HIGGS_USE_PROPOSAL_CANCELLED = 4
};
typedef struct HIGGS_HandUseProposalRequest {
    uint32_t structSize, deadlineMs;
    uint64_t expectedHandGenerations[2];
    HIGGS_HandUseRequest use;
} HIGGS_HandUseProposalRequest;

typedef struct HIGGS_HandUseProposalState {
    uint32_t structSize, stage, reason, decisionFlags;
    uint64_t proposalId, roundId;
    int32_t effectivePriority;
    uint32_t reserved;
    HIGGS_HandUseState use;
} HIGGS_HandUseProposalState;

typedef struct HIGGS_HandUseProposalsInterface {
    uint32_t structSize, abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus (HIGGS_HAND_CALL *Propose)(HIGGS_HandClient, const HIGGS_HandUseProposalRequest *, uint64_t *);
    HIGGS_HandStatus (HIGGS_HAND_CALL *Copy)(HIGGS_HandClient, uint64_t, HIGGS_HandUseProposalState *);
    HIGGS_HandStatus (HIGGS_HAND_CALL *Cancel)(HIGGS_HandClient, uint64_t, uint64_t);
} HIGGS_HandUseProposalsInterface;
/* Nonbutton admission only; pending proposals carry NO grant or mutation authority.
 * The next HIGGS main maintenance selects a disjoint set in effective user-priority
 * order, then stable client ID, use kind, context ID and generation. Both hands are all-or-none.
 * Existing uses are never preempted. A pending proposal is one-shot, not a retry.
 * Deadline1..1000ms bounds collection; use.durationMs begins on acceptance.
 * All requested generations must be current at proposal AND selection. UI/load,
 * tracking/eligibility changes, cancellation, executor/client removal invalidate it.
 * Accepted proposalId is the ordinary HandUse requestId. Renew/End/Resize and the
 * existing executor/receipt protocol apply. Cancel only cancels pending proposals;
 * use EndHandUse after acceptance. Copy preserves the admission decision even after
 * a session ends; use.active reports current authority. Results share the bounded
 * broker history and can be evicted. At most8pending proposals per client, one per
 * client/kind/contextId/contextGeneration. Reentrant proposals wait for later main
 * maintenance. Input arbitration and already acquired legacy immediate HandUse
 * are separate admission paths; no cross-frame or global default-action claim.
 * Exact v1 copied POD; existing client and sized-output rules apply. */
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandUseProposalsInterface(uint32_t, uint32_t, HIGGS_HandUseProposalsInterface *);
typedef HIGGS_HandStatus (HIGGS_HAND_CALL *HIGGS_QueryHandUseProposalsInterfaceFn)(uint32_t, uint32_t, HIGGS_HandUseProposalsInterface *);
#ifdef __cplusplus
static_assert(sizeof(HIGGS_HandUseProposalRequest) == 72, "hand-use proposal request ABI");
static_assert(sizeof(HIGGS_HandUseProposalState) == 160, "hand-use proposal state ABI");
static_assert(sizeof(HIGGS_HandUseProposalsInterface) == 40, "hand-use proposal table ABI (x64)");
#endif

typedef struct HIGGS_HandUseTransitionRequest {
    uint32_t structSize,physicalHands,durationMs,reserved;
    uint64_t epoch,requestId,expectedRevision,expectedHandGenerations[2];
} HIGGS_HandUseTransitionRequest;

typedef struct HIGGS_HandUseTransitionState {
    uint32_t structSize,reserved;
    uint64_t revision;
    HIGGS_HandUseState use;
} HIGGS_HandUseTransitionState;

typedef struct HIGGS_HandUseTransitionInterface {
    uint32_t structSize,abiVersion;
    uint64_t capabilities;
    HIGGS_HandStatus (HIGGS_HAND_CALL *Copy)(HIGGS_HandClient,uint64_t,HIGGS_HandUseTransitionState*);
    HIGGS_HandStatus (HIGGS_HAND_CALL *Resize)(HIGGS_HandClient,const HIGGS_HandUseTransitionRequest*,HIGGS_HandUseTransitionState*);
} HIGGS_HandUseTransitionInterface;
/* Atomic live-session footprint changes. Copy supplies the footprint revision;
 * Hand.CopySnapshot supplies generations for all hands in the requested footprint.
 * Successful Resize preserves request/grant/context/kind and the retained hands'
 * authority; it changes the entire footprint or nothing. Every Resize advances
 * revision, including same-footprint renewal. Old HandUse.Renew does not change it.
 * No priority preemption: an added hand must be eligible now. Removed hands must
 * have no live broker child/native/retained/actuator work under this grant: end it
 * and await its actual drainage first. Refusal leaves existing authority intact.
 * Existing held items are never implicitly released. UI permits renew/reduction,
 * but not expansion. Expiry/load/tracking loss cannot be revived by Resize.
 * Separate HandUseProposals supplies nonbutton admission rounds. Resize is not
 * proposal arbitration or a transferable/delegated grant.
 * Exact v1 copied POD, ordinary client/output rules. Capability bit0: resize. */
HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandUseTransitionInterface(uint32_t,uint32_t,HIGGS_HandUseTransitionInterface*);
typedef HIGGS_HandStatus (HIGGS_HAND_CALL *HIGGS_QueryHandUseTransitionInterfaceFn)(uint32_t,uint32_t,HIGGS_HandUseTransitionInterface*);
#ifdef __cplusplus
static_assert(sizeof(HIGGS_HandUseTransitionRequest)==56,"HandUse transition request ABI");
static_assert(sizeof(HIGGS_HandUseTransitionState)==136,"HandUse transition state ABI");
static_assert(sizeof(HIGGS_HandUseTransitionInterface)==32,"HandUse transition table ABI (x64)");
#endif

#pragma pack(pop)
#ifdef __cplusplus
}
#endif
#endif
