#ifndef HIGGS_HAND_USE_PROPOSALS_H
#define HIGGS_HAND_USE_PROPOSALS_H
#include "higgshanduse.h"
#ifdef __cplusplus
extern "C" {
#endif
#define HIGGS_HAND_USE_PROPOSALS_ABI_VERSION 1u
#define HIGGS_HAND_USE_PROPOSALS_CAP_PRIORITY_ROUNDS UINT64_C(1)
enum {
    HIGGS_USE_PROPOSAL_PENDING = 1,
    HIGGS_USE_PROPOSAL_ACCEPTED = 2,
    HIGGS_USE_PROPOSAL_REJECTED = 3,
    HIGGS_USE_PROPOSAL_CANCELLED = 4
};
#pragma pack(push, 8)
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
#pragma pack(pop)
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
}
static_assert(sizeof(HIGGS_HandUseProposalRequest) == 72, "hand-use proposal request ABI");
static_assert(sizeof(HIGGS_HandUseProposalState) == 160, "hand-use proposal state ABI");
static_assert(sizeof(HIGGS_HandUseProposalsInterface) == 40, "hand-use proposal table ABI (x64)");
#endif
#endif
