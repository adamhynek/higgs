#include "hand_abi_boundary.h"
#include "hand_broker.h"
#include <cstring>
#include <limits>
namespace HiggsHands
{
HIGGS_HandUseState Broker::HandUseState(const Action &a)
{
    return {sizeof(HIGGS_HandUseState), a.sessionKind, a.grantActive ? 1u : 0u, 0,
            a.request.epoch, a.expires, a.request};
}
void Broker::AcceptHandUseLocked(Action &a, const HIGGS_HandUseRequest &use, Clock now)
{
    a.request.grant = nextToken_++;
    a.selected = a.grantActive = true;
    a.sessionKind = use.kind;
    a.sessionRevision = 1;
    a.expires = now + uint64_t(use.durationMs) * 1000000;
    FinishLocked(a, HIGGS_ACTION_ACCEPTED, 0);
    RefreshLocked();
}
HIGGS_HandStatus Broker::HandUse(uint64_t owner, const HIGGS_HandUseRequest &r, HIGGS_HandUseState &out, uint64_t now)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (r.structSize < sizeof(r) || r.kind < HIGGS_USE_HAND_GOALS || r.kind > HIGGS_USE_CONSUMPTION ||
        !r.physicalHands || (r.physicalHands & ~3u) || !r.durationMs || r.durationMs > 60000)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (r.epoch != snapshot_.epoch)
        return HIGGS_HAND_STALE;
    if (SharedRoundsLocked())
        return HIGGS_HAND_WRONG_PHASE;
    auto executor = RegistrationLocked(r.executor);
    if (!executor || !executor->active || executor->owner != owner || executor->kind != 2)
        return HIGGS_HAND_WRONG_OWNER;
    if (!EligibleLocked(r.physicalHands))
        return HIGGS_HAND_CONFLICT;
    // A late immediate request must not jump an already collecting proposal round.
    for (const auto &pending : actions_)
        if (pending.useProposal.structSize && !pending.result.stage &&
            (pending.request.physicalHands & r.physicalHands))
            return HIGGS_HAND_CONFLICT;
    auto action = NewActionLocked();
    if (!action)
        return HIGGS_HAND_CAPACITY;
    action->request.structSize = sizeof(action->request);
    action->request.requestId = nextToken_++;
    action->request.owner = owner;
    action->request.executor = r.executor;
    action->request.physicalHands = r.physicalHands;
    action->request.epoch = r.epoch;
    action->request.actionId = r.kind;
    action->request.contextId = r.contextId;
    action->request.contextGeneration = r.contextGeneration;
    AcceptHandUseLocked(*action, r, now);
    out = HandUseState(*action);
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::ReadHandUse(uint64_t owner, uint64_t id, uint64_t epoch, uint32_t renew,
                                     HIGGS_HandUseState &out, uint64_t now)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto a = ActionLocked(id);
    if (!a || !a->sessionKind)
        return HIGGS_HAND_NOT_FOUND;
    if (a->request.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    if (epoch && epoch != snapshot_.epoch)
        return HIGGS_HAND_STALE;
    if (renew)
    {
        if (renew > 60000)
            return HIGGS_HAND_INVALID_ARGUMENT;
        if (a->request.epoch != snapshot_.epoch || !a->grantActive)
            return HIGGS_HAND_STALE;
        auto executor = RegistrationLocked(a->request.executor);
        if (!executor || !executor->active)
            return HIGGS_HAND_NOT_FOUND;
        for (const auto &h : snapshot_.hands)
            if (h.physicalHand & a->request.physicalHands)
                if (h.grant != a->request.grant || h.owner != owner || !(h.observationMask & HIGGS_OBS_TRACKING_VALID))
                    return HIGGS_HAND_STALE;
        a->expires = now + static_cast<uint64_t>(renew) * 1000000;
    }
    out = HandUseState(*a);
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::EndHandUse(uint64_t owner, uint64_t id, uint64_t epoch)
{
    auto phase = NativePhase();
    uint64_t grant = 0;
    {
        std::lock_guard lock(mutex_);
        if (!ClientLocked(owner))
            return HIGGS_HAND_INVALID_CLIENT;
        auto a = ActionLocked(id);
        if (!a || !a->sessionKind)
            return HIGGS_HAND_NOT_FOUND;
        if (a->request.owner != owner)
            return HIGGS_HAND_WRONG_OWNER;
        if (epoch != a->request.epoch || epoch != snapshot_.epoch)
            return HIGGS_HAND_STALE;
        grant = a->request.grant;
    }
    return Release(owner, grant);
}
void Broker::DrainHandUses()
{
    auto phase = NativePhase();
    if (drainingHandUses_) return;
    struct DrainScope {
        bool &active;
        explicit DrainScope(bool &value) : active(value) { active = true; }
        ~DrainScope() { active = false; }
    } draining(drainingHandUses_);
    if (!GameplayContext())
        return;
    std::array<HIGGS_ActionRequest, 256> work{};
    size_t count = 0;
    {
        std::lock_guard lock(mutex_);
        ExpireLocked(Now());
        ResolveHandUseProposalsLocked(Now());
        for (auto &a : actions_)
            if ((a.sessionKind || (a.proposalRound && a.request.inputId && !a.nativeCandidate &&
                                  a.recognition.stage != HIGGS_RECOGNITION_WAITING)) &&
                a.grantActive && !a.sessionNotified && !a.dispatched)
            {
                work[count++] = a.request;
            }
    }
    for (size_t i = 0; i < count; ++i)
    {
        std::shared_ptr<Registration> executor;
        {
            std::lock_guard lock(mutex_);
            // A previous executor may have taken long enough for another lease
            // to expire. Do not notify it with authority that is already stale.
            ExpireLocked(Now());
            auto a = ActionLocked(work[i].requestId);
            if (contextMenus_ || !a || a->sessionNotified || !a->grantActive || a->result.stage >= HIGGS_ACTION_COMPLETED)
                continue;
            a->sessionNotified = true;
            a->dispatched = true;
            work[i] = a->request;
            executor = RegistrationLocked(a->request.executor);
        }
        InvokeSessionExecutor(executor, work[i]);
    }
    FlushEvents();
}
HIGGS_HandStatus Broker::CopyHandUseTransition(uint64_t owner, uint64_t id,
                                               HIGGS_HandUseTransitionState &out, uint64_t now)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    const auto a = ActionLocked(id);
    if (!a || !a->sessionKind) return HIGGS_HAND_NOT_FOUND;
    if (a->request.owner != owner) return HIGGS_HAND_WRONG_OWNER;
    out = {};
    out.structSize = sizeof(out);
    out.revision = a->sessionRevision;
    out.use = HandUseState(*a);
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::ResizeHandUse(uint64_t owner, const HIGGS_HandUseTransitionRequest &r,
                                      HIGGS_HandUseTransitionState &out, uint64_t now)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    return ResizeHandUseLocked(owner,r,out,now);
}
HIGGS_HandStatus Broker::ResizeHandUseLocked(uint64_t owner, const HIGGS_HandUseTransitionRequest &r,
                                            HIGGS_HandUseTransitionState &out, uint64_t now, bool selected)
{
    ExpireLocked(now);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    if (r.structSize != sizeof(r) || r.reserved || !r.physicalHands || (r.physicalHands & ~3u) ||
        !r.durationMs || r.durationMs > 60000 || !r.expectedRevision)
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto a = ActionLocked(r.requestId);
    if (!a || !a->sessionKind) return HIGGS_HAND_NOT_FOUND;
    if (a->request.owner != owner) return HIGGS_HAND_WRONG_OWNER;
    if (r.epoch != snapshot_.epoch || r.epoch != a->request.epoch || !a->grantActive ||
        worldChangePending_ || !snapshot_.worldReady)
        return HIGGS_HAND_STALE;
    if (r.expectedRevision != a->sessionRevision) return HIGGS_HAND_CONFLICT;
    if (a->sessionRevision == (std::numeric_limits<uint64_t>::max)()) return HIGGS_HAND_CAPACITY;
    const auto executor = RegistrationLocked(a->request.executor);
    if (!executor || !executor->active) return HIGGS_HAND_NOT_FOUND;
    if (a->nativeStarted) return HIGGS_HAND_CONFLICT;
    const auto added = r.physicalHands & ~a->request.physicalHands;
    const auto removed = a->request.physicalHands & ~r.physicalHands;
    if (added && !selected && SharedRoundsLocked()) return HIGGS_HAND_WRONG_PHASE;
    if (added && !EligibleLocked(added)) return HIGGS_HAND_CONFLICT;
    for (const auto &h : snapshot_.hands) {
        if (h.physicalHand & a->request.physicalHands)
            if (h.grant != a->request.grant || h.owner != owner || !(h.observationMask & HIGGS_OBS_TRACKING_VALID))
                return HIGGS_HAND_STALE;
        if (h.physicalHand & r.physicalHands)
            if (!r.expectedHandGenerations[h.physicalHand == 1 ? 0 : 1] ||
                h.generation != r.expectedHandGenerations[h.physicalHand == 1 ? 0 : 1])
                return HIGGS_HAND_STALE;
    }
    // A child may retain actual native work after its action became terminal.
    // Refuse shrinking until both its authority and physical ownership drain.
    for (const auto &child : actions_)
        if (&child != a && child.request.epoch == r.epoch &&
            ((child.request.grant == a->request.grant && (child.request.physicalHands & removed)) ||
             (child.dependencyGrant == a->request.grant && (child.dependencyHands & removed))) &&
            (child.grantActive || child.operationActive || child.nativeStarted || child.retained || child.ownedActuator ||
             (child.selected && child.result.stage < HIGGS_ACTION_COMPLETED)))
            return HIGGS_HAND_CONFLICT;
    a->request.physicalHands = r.physicalHands;
    a->expires = now + static_cast<uint64_t>(r.durationMs) * 1000000;
    ++a->sessionRevision;
    RefreshLocked();
    out = {};
    out.structSize = sizeof(out);
    out.revision = a->sessionRevision;
    out.use = HandUseState(*a);
    return HIGGS_HAND_OK;
}
} // namespace HiggsHands
namespace
{

HIGGS_HandStatus HIGGS_HAND_CALL Request(uint64_t owner, const HIGGS_HandUseRequest *r, HIGGS_HandUseState *out)
{
    if (!r || r->structSize < sizeof(*r))
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto s = HiggsAbi::CheckSized(out);
    if (s)
        return s;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::Service().HandUse(owner, *r, value, HiggsHands::Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Renew(uint64_t owner, uint64_t id, uint64_t epoch, uint32_t duration,
                                       HIGGS_HandUseState *out)
{
    auto s = HiggsAbi::CheckSized(out);
    if (s)
        return s;
    if (!duration)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::Service().ReadHandUse(owner, id, epoch, duration, value, HiggsHands::Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t owner, uint64_t id, HIGGS_HandUseState *out)
{
    auto s = HiggsAbi::CheckSized(out);
    if (s)
        return s;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::Service().ReadHandUse(owner, id, 0, 0, value, HiggsHands::Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL End(uint64_t owner, uint64_t id, uint64_t epoch)
{
    return HiggsAbi::Guard([&] { return HiggsHands::Service().EndHandUse(owner, id, epoch); });
}
HIGGS_HandStatus HIGGS_HAND_CALL CopyTransition(uint64_t owner,uint64_t id,HIGGS_HandUseTransitionState *out)
{
    const auto status = HiggsAbi::CheckSized(out); if (status) return status;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::Service().CopyHandUseTransition(owner,id,value,HiggsHands::Now());
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL ResizeTransition(uint64_t owner,const HIGGS_HandUseTransitionRequest *r,
                                                HIGGS_HandUseTransitionState *out)
{
    if (!r || r->structSize != sizeof(*r)) return HIGGS_HAND_INVALID_ARGUMENT;
    const auto status=HiggsAbi::CheckSized(out); if (status) return status;
    return HiggsAbi::PublishOnSuccess(out, [&](auto& value) {
        return HiggsHands::Service().ResizeHandUse(owner,*r,value,HiggsHands::Now());
    });
}
} // namespace
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandUseTransitionInterface(uint32_t version,uint32_t bytes,
                                                                               HIGGS_HandUseTransitionInterface *out)
{
    if (version != 1) return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out)) return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_HandUseTransitionInterface table{sizeof(table),1,1,CopyTransition,ResizeTransition};
    *out=table; return HIGGS_HAND_OK;
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHandUseInterface(uint32_t version, uint32_t bytes,
                                                                        HIGGS_HandUseInterface *out)
{
    if (version != 1)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_HandUseInterface table{sizeof(table), 1, 1, Request, Renew, Copy, End};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
