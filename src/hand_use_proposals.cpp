#include "hand_abi_boundary.h"
#include "hand_broker.h"
#include <algorithm>
#include <cstring>
#include <tuple>
namespace HiggsHands {
HIGGS_HandStatus Broker::ProposeHandUse(uint64_t owner, const HIGGS_HandUseProposalRequest &r,
                                       uint64_t &out, Clock now)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    const auto &use = r.use;
    if (r.structSize != sizeof(r) || !r.deadlineMs || r.deadlineMs > 1000 ||
        use.structSize != sizeof(use) || use.kind < HIGGS_USE_HAND_GOALS || use.kind > HIGGS_USE_CONSUMPTION ||
        !use.physicalHands || (use.physicalHands & ~3u) || !use.durationMs || use.durationMs > 60000)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (use.epoch != snapshot_.epoch) return HIGGS_HAND_STALE;
    const auto executor = RegistrationLocked(use.executor);
    if (!executor || !executor->active || executor->owner != owner || executor->kind != 2)
        return HIGGS_HAND_WRONG_OWNER;
    if (!EligibleLocked(use.physicalHands)) return HIGGS_HAND_CONFLICT;
    for (unsigned i = 0; i < 2; ++i)
        if ((use.physicalHands & (1u << i)) &&
            (!r.expectedHandGenerations[i] || r.expectedHandGenerations[i] != snapshot_.hands[i].generation))
            return HIGGS_HAND_STALE;
    unsigned pendingCount = 0;
    for (const auto &a : actions_)
        if (a.useProposal.structSize && !a.result.stage && a.request.owner == owner) {
            if (a.useProposal.use.kind == use.kind && a.request.contextId == use.contextId &&
                a.request.contextGeneration == use.contextGeneration) return HIGGS_HAND_CONFLICT;
            ++pendingCount;
        }
    if (pendingCount >= 8) return HIGGS_HAND_CAPACITY;
    const auto a = NewActionLocked();
    if (!a) return HIGGS_HAND_CAPACITY;
    a->useProposal = r;
    a->request.structSize = sizeof(a->request);
    a->request.physicalHands = use.physicalHands;
    a->request.requestId = nextToken_++;
    a->request.owner = owner;
    a->request.executor = use.executor;
    a->request.epoch = use.epoch;
    a->request.actionId = use.kind;
    a->request.contextId = use.contextId;
    a->request.contextGeneration = use.contextGeneration;
    a->inputContextGeneration = contextGeneration_;
    a->expires = now + uint64_t(r.deadlineMs) * 1000000;
    out = a->request.requestId;
    return HIGGS_HAND_OK;
}

void Broker::ResolveHandUseProposalsLocked(Clock now)
{
    std::array<Action *, 256> pending{};
    size_t count = 0;
    for (auto &a : actions_)
        if (a.useProposal.structSize && !a.result.stage) {
            const auto client = ClientLocked(a.request.owner);
            bool overridden = false;
            a.proposalPriority = client ? PriorityLocked(*client, overridden) : 0;
            a.proposalDecisionFlags = overridden ? HIGGS_DECISION_USER_PRIORITY : HIGGS_DECISION_CLIENT_PRIORITY;
            a.proposalDecisionFlags |= HIGGS_DECISION_STABLE_ID;
            pending[count++] = &a;
        }
    if (!count) return;
    const auto round = nextUseRound_++;
    std::sort(pending.begin(), pending.begin() + count, [&](const Action *left, const Action *right) {
        if (left->proposalPriority != right->proposalPriority) return left->proposalPriority > right->proposalPriority;
        const auto &a = left->request, &b = right->request;
        if (a.owner != b.owner) return ClientKeyLocked(a.owner) < ClientKeyLocked(b.owner);
        return std::tie(a.actionId, a.contextId, a.contextGeneration) < std::tie(b.actionId, b.contextId, b.contextGeneration);
    });
    for (size_t n = 0; n < count; ++n) {
        auto &a = *pending[n];
        a.proposalRound = round;
        uint32_t reason = HIGGS_HAND_OK;
        const auto executor = RegistrationLocked(a.request.executor);
        if (!ClientLocked(a.request.owner)) reason = HIGGS_HAND_INVALID_CLIENT;
        else if (!executor || !executor->active) reason = HIGGS_HAND_NOT_FOUND;
        else if (a.request.epoch != snapshot_.epoch || a.inputContextGeneration != contextGeneration_)
            reason = HIGGS_HAND_STALE;
        for (unsigned i = 0; !reason && i < 2; ++i)
            if ((a.request.physicalHands & (1u << i)) &&
                a.useProposal.expectedHandGenerations[i] != snapshot_.hands[i].generation)
                reason = HIGGS_HAND_STALE;
        if (!reason && !EligibleLocked(a.request.physicalHands)) reason = HIGGS_HAND_CONFLICT;
        if (reason) { FinishLocked(a,HIGGS_ACTION_REJECTED,reason); continue; }
        AcceptHandUseLocked(a, a.useProposal.use, now);
    }
}

HIGGS_HandStatus Broker::CopyHandUseProposal(uint64_t owner, uint64_t id,
                                           HIGGS_HandUseProposalState &out, Clock now)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    const auto a = ActionLocked(id);
    if (!a || !a->useProposal.structSize) return HIGGS_HAND_NOT_FOUND;
    if (a->request.owner != owner) return HIGGS_HAND_WRONG_OWNER;
    out = {};
    out.structSize = sizeof(out);
    out.stage = a->sessionKind ? HIGGS_USE_PROPOSAL_ACCEPTED : !a->result.stage ? HIGGS_USE_PROPOSAL_PENDING :
        a->result.stage == HIGGS_ACTION_REJECTED ? HIGGS_USE_PROPOSAL_REJECTED : HIGGS_USE_PROPOSAL_CANCELLED;
    // Admission and the later session outcome are separate facts.
    out.reason = a->sessionKind ? 0 : a->result.reason;
    out.proposalId = id;
    out.roundId = a->proposalRound;
    out.effectivePriority = a->proposalPriority;
    out.decisionFlags = a->proposalDecisionFlags;
    out.use.structSize = sizeof(out.use);
    if (a->sessionKind)
        out.use = HandUseState(*a);
    return HIGGS_HAND_OK;
}

HIGGS_HandStatus Broker::CancelHandUseProposal(uint64_t owner, uint64_t id, uint64_t epoch)
{
    auto phase = NativePhase();
    {
        std::lock_guard lock(mutex_);
        if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
        const auto a = ActionLocked(id);
        if (!a || !a->useProposal.structSize) return HIGGS_HAND_NOT_FOUND;
        if (a->request.owner != owner) return HIGGS_HAND_WRONG_OWNER;
        if (epoch != snapshot_.epoch || epoch != a->request.epoch) return HIGGS_HAND_STALE;
        if (a->sessionKind) return HIGGS_HAND_CONFLICT;
        if (!a->result.stage) FinishLocked(*a,HIGGS_ACTION_CANCELLED,0);
    }
    FlushEvents();
    return HIGGS_HAND_OK;
}
} // namespace HiggsHands
namespace {

HIGGS_HandStatus HIGGS_HAND_CALL Propose(uint64_t owner,const HIGGS_HandUseProposalRequest *r,uint64_t *out) {
    if (!r || !out || r->structSize != sizeof(*r)) return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& id) {
        return HiggsHands::Service().ProposeHandUse(owner,*r,id,HiggsHands::Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t owner,uint64_t id,HIGGS_HandUseProposalState *out) {
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    if (out->structSize < sizeof(*out)) { out->structSize=sizeof(*out); return HIGGS_HAND_BUFFER_TOO_SMALL; }
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::Service().CopyHandUseProposal(owner,id,value,HiggsHands::Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Cancel(uint64_t owner,uint64_t id,uint64_t epoch) {
    return HiggsAbi::Guard([&] { return HiggsHands::Service().CancelHandUseProposal(owner,id,epoch); });
}
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandUseProposalsInterface(uint32_t version,uint32_t bytes,
                                                                               HIGGS_HandUseProposalsInterface *out) {
    if (version != HIGGS_HAND_USE_PROPOSALS_ABI_VERSION) return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out)) return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_HandUseProposalsInterface table{sizeof(table),version,HIGGS_HAND_USE_PROPOSALS_CAP_PRIORITY_ROUNDS,
                                               Propose,Copy,Cancel};
    std::memcpy(out,&table,sizeof(table)); return HIGGS_HAND_OK;
}
