#include "hand_abi_boundary.h"
#include "hand_broker.h"
#include <algorithm>
#include <cstring>
#include <tuple>
namespace HiggsHands {
bool Broker::SharedRoundsLocked() const
{
    for (const auto &c : clients_) if (c.active && c.sharedRounds) return true;
    for (const auto &a : actions_) if (a.sharedPending && !a.result.stage) return true;
    return false;
}
HIGGS_HandStatus Broker::EnableSharedRounds(uint64_t owner)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    const auto client = ClientLocked(owner);
    if (!client) return HIGGS_HAND_INVALID_CLIENT;
    if (client->sharedRounds) return HIGGS_HAND_OK;
    for (const auto &hand : down_) for (bool down : hand) if (down) return HIGGS_HAND_WRONG_PHASE;
    client->sharedRounds = true;
    return HIGGS_HAND_OK;
}
void Broker::SetNativeTargetKeys(uint32_t hand, uint64_t grip, uint64_t trigger)
{
    if (hand != HIGGS_HAND_LEFT && hand != HIGGS_HAND_RIGHT) return;
    std::lock_guard lock(mutex_);
    nativeTargetKeys_[hand == HIGGS_HAND_LEFT ? 0 : 1][0] = grip;
    nativeTargetKeys_[hand == HIGGS_HAND_LEFT ? 0 : 1][1] = trigger;
}
bool Broker::NativeAcquisitionPendingLocked(uint32_t hands) const
{
    for (unsigned h = 0; h < 2; ++h)
        if ((hands & (1u << h)) && !snapshot_.hands[h].heldRefId &&
            !(snapshot_.hands[h].observationMask & HIGGS_OBS_SECONDARY_SUPPORT))
            for (const auto &active : activeSequences_[h])
                if (active.record.nativeAction && active.record.requestId) {
                    for (const auto &a : actions_)
                        if (a.request.requestId == active.record.requestId && a.nativeCandidate == 1 && !a.nativeAcquired &&
                            a.selected && a.result.stage == HIGGS_ACTION_ACCEPTED) return true;
                }
    return false;
}
HIGGS_HandStatus Broker::ProposeOrderedHandUse(uint64_t owner, const HIGGS_HandUseProposalRequest &r,
                                             uint64_t action, uint64_t target, uint64_t &out, Clock now)
{
    auto phase = NativePhase();
    if (!action) return HIGGS_HAND_INVALID_ARGUMENT;
    {
        std::lock_guard lock(mutex_);
        const auto client = ClientLocked(owner);
        if (!client) return HIGGS_HAND_INVALID_CLIENT;
        if (!client->sharedRounds) return HIGGS_HAND_WRONG_PHASE;
    }
    const auto status = ProposeHandUse(owner, r, out, now);
    if (!status) {
        std::lock_guard lock(mutex_);
        const auto a = ActionLocked(out);
        a->request.actionId = action;
        a->stableTarget = target;
    }
    return status;
}
HIGGS_HandStatus Broker::NativeRegionAdmission(uint32_t hand, uint64_t epoch, uint32_t ref, uint32_t action, Clock now)
{
    if ((hand != HIGGS_HAND_LEFT && hand != HIGGS_HAND_RIGHT) || !ref || (action != 2 && action != 3))
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    if (!SharedRoundsLocked()) return HIGGS_HAND_OK;
    const auto hi = hand == HIGGS_HAND_LEFT ? 0u : 1u;
    const auto &h = snapshot_.hands[hi];
    if (epoch != snapshot_.epoch || ref != h.heldRefId) return HIGGS_HAND_STALE;
    for (const auto &a : actions_)
        if (a.nativeCandidate == action && a.sourceHand == hand && a.request.epoch == epoch &&
            a.stableTarget == ref && a.inputHandGenerations[hi] == h.generation) {
            if (!a.result.stage) return HIGGS_HAND_PENDING;
            return a.result.stage == HIGGS_ACTION_ACCEPTED && a.grantActive &&
                a.inputContextGeneration == contextGeneration_ && h.grant == a.request.grant ?
                    HIGGS_HAND_OK : HIGGS_HAND_CONFLICT;
        }
    if (!EligibleLocked(hand)) return HIGGS_HAND_CONFLICT;
    const auto a = NewActionLocked();
    if (!a) return HIGGS_HAND_CAPACITY;
    a->sharedPending = true; a->nativeCandidate = action; a->sourceHand = hand; a->stableTarget = ref;
    a->request = {sizeof(HIGGS_ActionRequest),hand,nextToken_++,0,0,0,epoch,0,action,ref,h.generation,h.poseObservationId};
    a->inputContextGeneration = contextGeneration_;
    a->inputHandGenerations[hi] = h.generation;
    a->expires = now + UINT64_C(1000000000);
    return HIGGS_HAND_PENDING;
}
HIGGS_HandStatus Broker::ProposeHandUseResize(uint64_t owner, const HIGGS_HandUseTransitionRequest &r,
                                            uint64_t action, uint64_t target, uint64_t &out, Clock now)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    const auto client = ClientLocked(owner);
    if (!client) return HIGGS_HAND_INVALID_CLIENT;
    if (!client->sharedRounds) return HIGGS_HAND_WRONG_PHASE;
    if (r.structSize != sizeof(r) || !action || r.reserved || !r.expectedRevision ||
        !r.physicalHands || (r.physicalHands & ~3u) || !r.durationMs || r.durationMs > 60000)
        return HIGGS_HAND_INVALID_ARGUMENT;
    const auto parent = ActionLocked(r.requestId);
    if (!parent || !parent->sessionKind) return HIGGS_HAND_NOT_FOUND;
    if (parent->request.owner != owner) return HIGGS_HAND_WRONG_OWNER;
    if (!parent->grantActive || r.epoch != snapshot_.epoch || r.epoch != parent->request.epoch)
        return HIGGS_HAND_STALE;
    if (parent->sessionRevision != r.expectedRevision) return HIGGS_HAND_CONFLICT;
    const auto added = r.physicalHands & ~parent->request.physicalHands;
    if (!added) return HIGGS_HAND_INVALID_ARGUMENT;
    if (!EligibleLocked(added)) return HIGGS_HAND_CONFLICT;
    unsigned count = 0;
    for (const auto &a : actions_) if (!a.result.stage && a.request.owner == owner &&
        (a.useProposal.structSize || a.resizeProposal.structSize)) ++count;
    if (count >= 8) return HIGGS_HAND_CAPACITY;
    for (unsigned i=0;i<2;++i)
        if ((r.physicalHands & (1u << i)) && r.expectedHandGenerations[i] != snapshot_.hands[i].generation)
            return HIGGS_HAND_STALE;
    const auto a = NewActionLocked();
    if (!a) return HIGGS_HAND_CAPACITY;
    a->resizeProposal = r; a->resizingProposal = true; a->sharedPending = true; a->stableTarget = target;
    a->request = {sizeof(HIGGS_ActionRequest),added,nextToken_++,owner,parent->request.executor,0,
                  r.epoch,0,action,parent->request.contextId,parent->request.contextGeneration,0};
    a->inputContextGeneration = contextGeneration_;
    for (unsigned i=0;i<2;++i) a->inputHandGenerations[i] = r.expectedHandGenerations[i];
    a->expires = now + UINT64_C(1000000000);
    out = a->request.requestId;
    return HIGGS_HAND_OK;
}
void Broker::FinishNativeRegionAction(uint32_t hand, bool executed)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    for (auto &a : actions_)
        if (a.nativeCandidate >= 2 && a.sourceHand == hand && a.grantActive) {
            a.grantActive = false;
            FinishLocked(a,executed ? HIGGS_ACTION_COMPLETED : HIGGS_ACTION_CANCELLED,0);
        }
    RefreshLocked();
}
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
            if (!a.resizingProposal && a.useProposal.use.kind == use.kind && a.request.contextId == use.contextId &&
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
    a->stableTarget = use.contextId;
    a->inputContextGeneration = contextGeneration_;
    a->expires = now + uint64_t(r.deadlineMs) * 1000000;
    out = a->request.requestId;
    return HIGGS_HAND_OK;
}

void Broker::ResolveHandUseProposalsLocked(Clock now)
{
    const bool shared = SharedRoundsLocked();
    std::array<Action *, 256> pending{};
    size_t count = 0;
    for (auto &a : actions_)
        if ((a.useProposal.structSize || a.sharedPending) && !a.result.stage) {
            const auto client = ClientLocked(a.request.owner);
            bool overridden = false;
            Client native{};
            std::memcpy(native.info.name, "higgs", 6);
            a.proposalPriority = a.nativeCandidate ? ActionPriorityLocked(native, a.request.actionId, overridden) :
                client ? shared ? ActionPriorityLocked(*client, a.request.actionId, overridden) :
                                  PriorityLocked(*client, overridden) : 0;
            a.proposalDecisionFlags = overridden ? HIGGS_DECISION_USER_PRIORITY : HIGGS_DECISION_CLIENT_PRIORITY;
            a.proposalDecisionFlags |= HIGGS_DECISION_STABLE_ID;
            pending[count++] = &a;
        }
    if (!count) return;
    const auto round = nextUseRound_++;
    const auto key = [&](const Action &a) {
        return std::make_tuple(a.nativeCandidate ? std::string_view("higgs") : ClientKeyLocked(a.request.owner),
                               a.request.actionId, a.stableTarget, shared ? UINT64_C(0) : a.request.contextGeneration);
    };
    std::sort(pending.begin(), pending.begin() + count, [&](const Action *left, const Action *right) {
        if (left->proposalPriority != right->proposalPriority) return left->proposalPriority > right->proposalPriority;
        return key(*left) < key(*right);
    });
    uint32_t selectedHands = 0;
    for (size_t n = 0; n < count; ++n) {
        auto &a = *pending[n];
        a.proposalRound = round;
        a.sharedPending = false;
        uint32_t reason = HIGGS_HAND_OK;
        const auto executor = RegistrationLocked(a.request.executor);
        if (!a.nativeCandidate && !ClientLocked(a.request.owner)) reason = HIGGS_HAND_INVALID_CLIENT;
        else if (!a.nativeCandidate && (!executor || !executor->active)) reason = HIGGS_HAND_NOT_FOUND;
        else if (a.inputRegistration && (!RegistrationLocked(a.inputRegistration) ||
                 !RegistrationLocked(a.inputRegistration)->active || RegistrationLocked(a.inputRegistration)->kind != 5))
            reason = HIGGS_HAND_NOT_FOUND;
        else if (a.request.epoch != snapshot_.epoch || a.inputContextGeneration != contextGeneration_)
            reason = HIGGS_HAND_STALE;
        if (shared && a.nativeCandidate != 1 &&
            ((n && key(a) == key(*pending[n-1])) || (n+1 < count && key(a) == key(*pending[n+1]))))
            reason = HIGGS_HAND_CONFLICT;
        for (unsigned i = 0; !reason && i < 2; ++i)
            if ((a.request.physicalHands & (1u << i)) &&
                (a.useProposal.structSize && !a.resizingProposal ? a.useProposal.expectedHandGenerations[i] : a.inputHandGenerations[i]) !=
                    snapshot_.hands[i].generation)
                reason = HIGGS_HAND_STALE;
        if (!reason && a.request.inputId) {
            const auto hi = a.sourceHand == HIGGS_HAND_LEFT ? 0u : 1u;
            if (!down_[hi][InputControlSlot(a.sourceButton)]) reason = HIGGS_HAND_STALE;
            if (!a.nativeCandidate && !RoutedLocked(a.request.executor, a.recognizerOwner, a.request.physicalHands))
                reason = HIGGS_HAND_WRONG_OWNER;
            if (a.nativeCandidate && nativeTargetKeys_[hi][a.sourceButton == HIGGS_INPUT_GRIP ? 0 : 1] != a.stableTarget)
                reason = HIGGS_HAND_STALE;
        }
        bool nativeAlias = false;
        for (size_t previous = 0; previous < n; ++previous)
            if (a.nativeCandidate == 1 && pending[previous]->nativeCandidate == 1 && pending[previous]->selected &&
                pending[previous]->request.physicalHands == a.request.physicalHands && key(a) == key(*pending[previous]))
                nativeAlias = true;
        if (!reason && !nativeAlias && (selectedHands & a.request.physicalHands)) reason = HIGGS_HAND_CONFLICT;
        if (!reason && !nativeAlias && !EligibleLocked(a.request.physicalHands)) reason = HIGGS_HAND_CONFLICT;
        if (reason) { FinishLocked(a,HIGGS_ACTION_REJECTED,reason); continue; }
        selectedHands |= a.request.physicalHands;
        if (a.resizingProposal) {
            HIGGS_HandUseTransitionState state{};
            const auto status = ResizeHandUseLocked(a.request.owner,a.resizeProposal,state,now,true);
            a.selected = !status;
            FinishLocked(a,status ? HIGGS_ACTION_REJECTED : HIGGS_ACTION_ACCEPTED,status);
            if (status) selectedHands &= ~a.request.physicalHands;
        }
        else if (a.useProposal.structSize) AcceptHandUseLocked(a, a.useProposal.use, now);
        else {
            a.selected = true;
            a.grantActive = a.nativeCandidate != 1;
            if (a.grantActive) a.request.grant = nextToken_++;
            FinishLocked(a,HIGGS_ACTION_ACCEPTED,0);
            if (a.request.inputId) {
                const auto hi = a.sourceHand == HIGGS_HAND_LEFT ? 0u : 1u;
                auto &active = activeSequences_[hi][InputControlSlot(a.sourceButton)];
                active.owner = a.request.owner;
                active.recognizer = a.recognizerOwner;
                active.record = {sizeof(HIGGS_InputSequence), HIGGS_SEQUENCE_HELD, a.sourceHand, a.sourceButton,
                    a.nativeCandidate ? 1u : 0u, 0, a.request.epoch, a.request.inputId, 0, a.request.requestId,
                    a.request.executor, GrantCorrelationLocked(a.request.grant), a.request.actionId};
                auto &decision = decisions_[a.request.inputId % decisions_.size()];
                decision.owner = a.request.owner; decision.grant = a.request.grant;
                decision.requestId = a.request.requestId; decision.effectivePriority = a.proposalPriority;
                decision.reason = a.proposalDecisionFlags | (a.nativeCandidate ? HIGGS_DECISION_NATIVE : 0);
                if (a.inputSpec.structSize) {
                    a.recognition.stage = a.inputSpec.gesture ? HIGGS_RECOGNITION_WAITING : HIGGS_RECOGNITION_MATCHED;
                    if (!a.inputSpec.gesture) {
                        a.recognition.recognizedAtNs = now;
                        a.recognition.recognizedFrame = a.recognition.initialFrame;
                    }
                }
            }
            RefreshLocked();
        }
    }
    for (size_t n = 0; n < count; ++n) {
        const auto &a = *pending[n];
        if (a.request.inputId) {
            auto &decision = decisions_[a.request.inputId % decisions_.size()];
            if (decision.inputId == a.request.inputId) decision.reason &= ~HIGGS_DECISION_PENDING;
        }
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
    if (!a || (!a->useProposal.structSize && !a->resizeProposal.structSize)) return HIGGS_HAND_NOT_FOUND;
    if (a->request.owner != owner) return HIGGS_HAND_WRONG_OWNER;
    out = {};
    out.structSize = sizeof(out);
    out.stage = (a->sessionKind || a->selected) ? HIGGS_USE_PROPOSAL_ACCEPTED : !a->result.stage ? HIGGS_USE_PROPOSAL_PENDING :
        a->result.stage == HIGGS_ACTION_REJECTED ? HIGGS_USE_PROPOSAL_REJECTED : HIGGS_USE_PROPOSAL_CANCELLED;
    // Admission and the later session outcome are separate facts.
    out.reason = (a->sessionKind || a->selected) ? 0 : a->result.reason;
    out.proposalId = id;
    out.roundId = a->proposalRound;
    out.effectivePriority = a->proposalPriority;
    out.decisionFlags = a->proposalDecisionFlags;
    out.use.structSize = sizeof(out.use);
    if (a->sessionKind)
        out.use = HandUseState(*a);
    else if (a->resizingProposal && a->selected)
        if (const auto parent = ActionLocked(a->resizeProposal.requestId)) out.use = HandUseState(*parent);
    return HIGGS_HAND_OK;
}

HIGGS_HandStatus Broker::CancelHandUseProposal(uint64_t owner, uint64_t id, uint64_t epoch)
{
    auto phase = NativePhase();
    {
        std::lock_guard lock(mutex_);
        if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
        const auto a = ActionLocked(id);
        if (!a || (!a->useProposal.structSize && !a->resizeProposal.structSize)) return HIGGS_HAND_NOT_FOUND;
        if (a->request.owner != owner) return HIGGS_HAND_WRONG_OWNER;
        if (epoch != snapshot_.epoch || epoch != a->request.epoch) return HIGGS_HAND_STALE;
        if (a->sessionKind || a->selected) return HIGGS_HAND_CONFLICT;
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
HIGGS_HandStatus HIGGS_HAND_CALL Enable(uint64_t owner) {
    return HiggsAbi::Guard([&] { return HiggsHands::Service().EnableSharedRounds(owner); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Ordered(uint64_t owner,const HIGGS_HandUseProposalRequest *r,
                                       uint64_t action,uint64_t target,uint64_t *out) {
    if (!r || !out || r->structSize != sizeof(*r)) return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out,[&](auto &id) {
        return HiggsHands::Service().ProposeOrderedHandUse(owner,*r,action,target,id,HiggsHands::Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Resize(uint64_t owner,const HIGGS_HandUseTransitionRequest *r,
                                      uint64_t action,uint64_t target,uint64_t *out) {
    if (!r || !out || r->structSize != sizeof(*r)) return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out,[&](auto &id) {
        return HiggsHands::Service().ProposeHandUseResize(owner,*r,action,target,id,HiggsHands::Now());
    });
}
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandUseProposalsInterface(uint32_t version,uint32_t bytes,
                                                                               HIGGS_HandUseProposalsInterface *out) {
    if (version != HIGGS_HAND_USE_PROPOSALS_ABI_VERSION && version != 2) return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    if (version == 2) {
        if (bytes < sizeof(HIGGS_SharedHandUseProposalsInterface)) return HIGGS_HAND_BUFFER_TOO_SMALL;
        const HIGGS_SharedHandUseProposalsInterface table{
            {sizeof(table),version,HIGGS_HAND_USE_PROPOSALS_CAP_PRIORITY_ROUNDS | HIGGS_HAND_USE_PROPOSALS_CAP_SHARED_WINDOWS,
             Propose,Copy,Cancel},Enable,Ordered,Resize};
        std::memcpy(out,&table,sizeof(table)); return HIGGS_HAND_OK;
    }
    if (bytes < sizeof(*out)) return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_HandUseProposalsInterface table{sizeof(table),version,HIGGS_HAND_USE_PROPOSALS_CAP_PRIORITY_ROUNDS,
                                               Propose,Copy,Cancel};
    std::memcpy(out,&table,sizeof(table)); return HIGGS_HAND_OK;
}
