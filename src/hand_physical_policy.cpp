#include "hand_physical_policy.h"
#include "hand_broker.h"
#include "hand_transactions.h"
#include <cmath>
#include <cstring>
namespace HiggsHands
{
namespace
{
bool Terminal(uint32_t stage)
{
    return stage == HIGGS_POLICY_ENDED || stage == HIGGS_POLICY_BROKEN;
}
bool Applied(HIGGS_PhysicalPolicyState &state)
{
    uint32_t required = state.request.kind == 1   ? state.request.fields
                        : state.request.kind == 2 ? 1u
                                                  : state.request.presentation;
    if ((state.appliedFields & required) == required)
    {
        state.flags |= HIGGS_POLICY_APPLIED;
        return true;
    }
    state.flags &= ~HIGGS_POLICY_APPLIED;
    return false;
}
uint32_t PropertyResources(uint32_t fields)
{
    return (fields & 3 ? 3u : 0u) | (fields & 4);
}
uint64_t Required(const HIGGS_PhysicalPolicyRequest &r)
{
    return r.kind == HIGGS_POLICY_PROPERTIES ? HIGGS_POLICY_CAP_PROPERTIES
           : r.kind == HIGGS_POLICY_GRAB_MODE
               ? HIGGS_POLICY_CAP_GRAB_MODE
               : ((r.presentation & HIGGS_PRESENTATION_SUPPRESS_FINGERS ? HIGGS_POLICY_CAP_FINGER_SUPPRESSION : 0) |
                  (r.presentation & HIGGS_PRESENTATION_CULL_CONTROLLER ? HIGGS_POLICY_CAP_CONTROLLER_NODE_CULL : 0));
}
} // namespace
HIGGS_HandStatus Broker::ReserveEffectAuthority(uint64_t owner, uint64_t epoch, uint32_t duration,
                                                HIGGS_ActionRequest &out, uint64_t now,
                                                uint64_t dependencyGrant, uint32_t dependencyHands)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (epoch != snapshot_.epoch)
        return HIGGS_HAND_STALE;
    if (!snapshot_.worldReady || worldChangePending_ || contextMenus_)
        return HIGGS_HAND_NOT_READY;
    if (!duration || duration > 60000)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if ((dependencyGrant == 0) != (dependencyHands == 0) || (dependencyHands & ~3u))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (dependencyGrant && !InputEligibleLocked(dependencyHands, owner, dependencyGrant))
        return HIGGS_HAND_WRONG_OWNER;
    auto action = NewActionLocked();
    if (!action)
        return HIGGS_HAND_CAPACITY;
    action->request.structSize = sizeof(action->request);
    action->request.requestId = nextToken_++;
    action->request.owner = owner;
    action->request.epoch = epoch;
    action->selected = action->domainOwned = action->retained = true;
    action->dependencyGrant = dependencyGrant;
    action->dependencyHands = dependencyHands;
    action->expires = now + uint64_t(duration) * 1000000;
    FinishLocked(*action, HIGGS_ACTION_ACCEPTED, 0);
    out = action->request;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::RenewEffectAuthority(uint64_t owner, uint64_t id, uint32_t duration, uint64_t now)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    auto action = ActionLocked(id);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (!action || action->request.owner != owner || !action->domainOwned || action->request.physicalHands ||
        action->request.grant || !action->retained)
        return HIGGS_HAND_WRONG_OWNER;
    if (action->request.epoch != snapshot_.epoch || now >= action->expires || worldChangePending_ ||
        !snapshot_.worldReady)
        return HIGGS_HAND_STALE;
    if (!duration || duration > 60000)
        return HIGGS_HAND_INVALID_ARGUMENT;
    action->expires = now + uint64_t(duration) * 1000000;
    return HIGGS_HAND_OK;
}
PhysicalPolicyService &PhysicalPolicies()
{
    static PhysicalPolicyService service(Service());
    return service;
}
PhysicalPolicyService::Slot *PhysicalPolicyService::Find(uint64_t id)
{
    for (auto &slot : slots_)
        if (slot.state.policy == id && id)
            return &slot;
    return nullptr;
}
void PhysicalPolicyService::Install(Driver driver, void *context, uint64_t caps)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    driver_ = driver;
    context_ = context;
    capabilities_ = caps;
}
uint64_t PhysicalPolicyService::Capabilities()
{
    std::lock_guard lock(mutex_);
    return capabilities_;
}
HIGGS_HandStatus PhysicalPolicyService::Validate(uint64_t owner, const HIGGS_PhysicalPolicyRequest &r, uint64_t now)
{
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (r.structSize < sizeof(r) || r.kind < 1 || r.kind > 3 || (r.physicalHand != 1 && r.physicalHand != 2) ||
        !r.durationMs || r.durationMs > 60000 || r.reserved || r.reserved2)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (r.kind == HIGGS_POLICY_PROPERTIES)
    {
        if (!r.fields || (r.fields & ~7u) || r.grabMode || r.presentation ||
            r.item.kind != HIGGS_ITEM_WORLD_REFERENCE || !r.item.instance || !std::isfinite(r.massMultiplier) ||
            !std::isfinite(r.inertiaMultiplier) || r.massMultiplier < .01f || r.massMultiplier > 100 ||
            r.inertiaMultiplier < .01f || r.inertiaMultiplier > 100)
            return HIGGS_HAND_INVALID_ARGUMENT;
        for (float value : r.centerOfMassOffset)
            if (!std::isfinite(value) || std::fabs(value) > 2)
                return HIGGS_HAND_INVALID_ARGUMENT;
        if (!Transactions().IdentityCurrent(r.item))
            return HIGGS_HAND_STALE;
    }
    else
    {
        if (r.fields || r.item.instance || r.massMultiplier || r.inertiaMultiplier || r.centerOfMassOffset[0] ||
            r.centerOfMassOffset[1] || r.centerOfMassOffset[2])
            return HIGGS_HAND_INVALID_ARGUMENT;
        if (r.kind == HIGGS_POLICY_GRAB_MODE)
        {
            if (r.presentation || (r.grabMode != 1 && r.grabMode != 2))
                return HIGGS_HAND_INVALID_ARGUMENT;
        }
        else if (r.grabMode || !r.presentation || (r.presentation & ~3u))
            return HIGGS_HAND_INVALID_ARGUMENT;
    }
    if (!driver_ || (capabilities_ & Required(r)) != Required(r))
        return HIGGS_HAND_UNSUPPORTED;
    HIGGS_HandSnapshot snapshot{};
    auto status = broker_.Copy(owner, snapshot, now);
    if (status)
        return status;
    if (snapshot.epoch != r.epoch)
        return HIGGS_HAND_STALE;
    const auto &hand = snapshot.hands[r.physicalHand - 1];
    if (hand.generation != r.handGeneration)
        return HIGGS_HAND_STALE;
    if (!snapshot.worldReady || !(hand.observationMask & HIGGS_OBS_TRACKING_VALID))
        return HIGGS_HAND_NOT_READY;
    if (r.grant && (hand.grant != r.grant || hand.owner != owner))
        return HIGGS_HAND_WRONG_OWNER;
    if (r.kind == 1 && (!(hand.observationMask & HIGGS_OBS_HELD_KNOWN) || hand.heldRefId != r.item.worldRefId))
        return HIGGS_HAND_CONFLICT;
    return HIGGS_HAND_OK;
}
bool PhysicalPolicyService::Conflict(const HIGGS_PhysicalPolicyRequest &r, uint64_t except)
{
    for (const auto &slot : slots_)
    {
        const auto &state = slot.state;
        if (!state.policy || state.policy == except || Terminal(state.stage) || state.epoch != r.epoch)
            continue;
        const auto &other = state.request;
        if (r.kind == 1 && other.kind == 1 && r.item.instance == other.item.instance &&
            (PropertyResources(r.fields) & PropertyResources(other.fields)))
            return true;
        if (r.physicalHand == other.physicalHand && r.kind == other.kind &&
            (r.kind == 2 || (r.kind == 3 && (r.presentation & other.presentation))))
            return true;
    }
    return false;
}
HIGGS_HandStatus PhysicalPolicyService::Request(uint64_t owner, const HIGGS_PhysicalPolicyRequest &r, uint64_t &out,
                                                uint64_t now)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    auto status = Validate(owner, r, now);
    if (status)
        return status;
    if (Conflict(r))
        return HIGGS_HAND_CONFLICT;
    Slot *slot = nullptr;
    for (auto &value : slots_)
        if (!value.state.policy)
        {
            slot = &value;
            break;
        }
    if (!slot)
        for (auto &value : slots_)
            if (Terminal(value.state.stage) && (!slot || value.state.policy < slot->state.policy))
                slot = &value;
    if (!slot)
        return HIGGS_HAND_CAPACITY;
    HIGGS_ActionRequest authority{};
    status = broker_.ReserveEffectAuthority(owner, r.epoch, r.durationMs, authority, now,
                                          r.grant, r.grant ? r.physicalHand : 0);
    if (status)
        return status;
    *slot = {};
    auto &state = slot->state;
    state.structSize = sizeof(state);
    state.stage = HIGGS_POLICY_PENDING;
    state.policy = authority.requestId;
    state.owner = owner;
    state.epoch = r.epoch;
    state.expiresAtNs = now + uint64_t(r.durationMs) * 1000000;
    state.revision = 1;
    state.request = r;
    broker_.EffectivePriority(owner, state.effectivePriority);
    out = state.policy;
    return HIGGS_HAND_OK;
}
bool PhysicalPolicyService::Alive(const Slot &slot, uint64_t now)
{
    const auto &state = slot.state;
    if (now >= state.expiresAtNs || !broker_.DomainAlive(state.owner, state.policy, now))
        return false;
    auto snapshot = broker_.CopyPublished();
    const auto &hand = snapshot.hands[state.request.physicalHand - 1];
    return snapshot.worldReady && (hand.observationMask & HIGGS_OBS_TRACKING_VALID) &&
           (!state.request.grant || (hand.grant == state.request.grant && hand.owner == state.owner)) &&
           (state.request.kind != 1 ||
            (hand.generation == state.request.handGeneration && hand.heldRefId == state.request.item.worldRefId &&
             Transactions().IdentityCurrent(state.request.item)));
}
HIGGS_HandStatus PhysicalPolicyService::Update(uint64_t owner, uint64_t id, uint64_t revision,
                                               const HIGGS_PhysicalPolicyRequest &r, uint64_t now)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    auto slot = Find(id);
    if (!slot)
        return HIGGS_HAND_NOT_FOUND;
    auto &state = slot->state;
    if (state.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    if (Terminal(state.stage) || state.stage == HIGGS_POLICY_ENDING || !Alive(*slot, now) || state.revision != revision)
        return HIGGS_HAND_STALE;
    if (r.kind != state.request.kind || r.physicalHand != state.request.physicalHand ||
        r.fields != state.request.fields || r.presentation != state.request.presentation ||
        r.grant != state.request.grant || r.item.instance != state.request.item.instance)
        return HIGGS_HAND_CONFLICT;
    auto status = Validate(owner, r, now);
    if (status)
        return status;
    if (!broker_.GameplayContext()) {
        const auto &before = state.request;
        const auto &a = r.item; const auto &b = before.item;
        if (r.epoch != before.epoch || r.handGeneration != before.handGeneration || r.grabMode != before.grabMode ||
            r.massMultiplier != before.massMultiplier || r.inertiaMultiplier != before.inertiaMultiplier ||
            a.structSize != b.structSize || a.kind != b.kind || a.epoch != b.epoch || a.instance != b.instance ||
            a.generation != b.generation || a.provider != b.provider || a.providerKey != b.providerKey ||
            a.instanceDataVersion != b.instanceDataVersion || a.validFields != b.validFields ||
            a.baseFormId != b.baseFormId || a.worldRefId != b.worldRefId || a.quantity != b.quantity)
            return HIGGS_HAND_NOT_READY;
        for (unsigned i = 0; i < 3; ++i)
            if (r.centerOfMassOffset[i] != before.centerOfMassOffset[i]) return HIGGS_HAND_NOT_READY;
    }
    status = broker_.RenewEffectAuthority(owner, id, r.durationMs, now);
    if (status)
        return status;
    state.request = r;
    ++state.revision;
    state.expiresAtNs = now + uint64_t(r.durationMs) * 1000000;
    state.flags &= ~HIGGS_POLICY_APPLIED;
    state.appliedFields = 0;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus PhysicalPolicyService::End(uint64_t owner, uint64_t id, uint64_t epoch)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto slot = Find(id);
    if (!slot)
        return HIGGS_HAND_NOT_FOUND;
    if (slot->state.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    if (slot->state.epoch != epoch || epoch != broker_.Epoch())
        return HIGGS_HAND_STALE;
    if (!Terminal(slot->state.stage))
        slot->state.stage = HIGGS_POLICY_ENDING;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus PhysicalPolicyService::Copy(uint64_t owner, uint64_t id, HIGGS_PhysicalPolicyState &out)
{
    std::lock_guard lock(mutex_);
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto slot = Find(id);
    if (!slot)
        return HIGGS_HAND_NOT_FOUND;
    if (slot->state.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    out = slot->state;
    return HIGGS_HAND_OK;
}
void PhysicalPolicyService::Drain(uint64_t now, uint32_t endingHand)
{
    auto phase = broker_.NativePhase();
    for (size_t index = 0; index < slots_.size(); ++index)
    {
        HIGGS_PhysicalPolicyState state{};
        Driver driver = nullptr;
        void *context = nullptr;
        {
            std::lock_guard lock(mutex_);
            auto &slot = slots_[index];
            if (!slot.state.policy || Terminal(slot.state.stage) ||
                (endingHand &&
                 (slot.state.request.physicalHand != endingHand || slot.state.stage != HIGGS_POLICY_ENDING)))
                continue;
            if (!Alive(slot, now))
            {
                slot.state.stage = HIGGS_POLICY_ENDING;
                slot.state.reason = HIGGS_HAND_STALE;
            }
            state = slot.state;
            driver = driver_;
            context = context_;
        }
        const bool cleanup = state.stage == HIGGS_POLICY_ENDING;
        if (!broker_.EnterDomainMutation(state.policy, cleanup))
            continue;
        HIGGS_HandStatus status = HIGGS_HAND_INTERNAL_ERROR;
        try
        {
            status = driver ? driver(state, cleanup, context) : HIGGS_HAND_NOT_READY;
        }
        catch (...)
        {
        }
        broker_.ExitDomainMutation(state.policy);
        std::lock_guard lock(mutex_);
        auto slot = Find(state.policy);
        if (!slot)
            continue;
        if (cleanup)
        {
            if (status == HIGGS_HAND_OK)
            {
                state.stage = state.reason ? HIGGS_POLICY_BROKEN : HIGGS_POLICY_ENDED;
                slot->state = state;
                broker_.RetainRequest(state.policy, false);
                broker_.CancelNativeRequest(state.owner, state.policy);
            }
            continue;
        }
        if (status == HIGGS_HAND_PENDING || status == HIGGS_HAND_NOT_READY)
            continue;
        if (status != HIGGS_HAND_OK)
        {
            slot->state.flags |= state.flags;
            slot->state.stage = HIGGS_POLICY_ENDING;
            slot->state.reason = status;
            continue;
        }
        if (slot->state.stage == HIGGS_POLICY_ENDING)
            continue;
        state.stage = HIGGS_POLICY_ACTIVE;
        if (Applied(state))
        {
            state.appliedRevision = state.revision;
            broker_.DomainResult(state.policy, HIGGS_ACTION_COMPLETED, 0);
        }
        slot->state = state;
    }
}
void PhysicalPolicyService::EndHeld(uint32_t hand)
{
    std::lock_guard lock(mutex_);
    for (auto &slot : slots_)
        if (slot.state.policy && !Terminal(slot.state.stage) && slot.state.request.kind == 1 &&
            slot.state.request.physicalHand == hand)
            slot.state.stage = HIGGS_POLICY_ENDING;
}
uint32_t PhysicalPolicyService::GrabMode(uint32_t hand, uint64_t now)
{
    std::lock_guard lock(mutex_);
    for (auto &slot : slots_)
        if (slot.state.request.kind == 2 && slot.state.request.physicalHand == hand &&
            slot.state.stage == HIGGS_POLICY_ACTIVE && Alive(slot, now))
        {
            slot.state.appliedFields = 1;
            if (Applied(slot.state))
            {
                slot.state.appliedRevision = slot.state.revision;
                broker_.DomainResult(slot.state.policy, HIGGS_ACTION_COMPLETED, 0);
            }
            return slot.state.request.grabMode;
        }
    return 0;
}
bool PhysicalPolicyService::SuppressFingers(uint32_t hand, uint64_t now)
{
    std::lock_guard lock(mutex_);
    for (auto &slot : slots_)
        if (slot.state.request.kind == 3 && (slot.state.request.presentation & HIGGS_PRESENTATION_SUPPRESS_FINGERS) &&
            slot.state.request.physicalHand == hand && slot.state.stage == HIGGS_POLICY_ACTIVE && Alive(slot, now))
        {
            slot.state.appliedFields |= HIGGS_PRESENTATION_SUPPRESS_FINGERS;
            if (Applied(slot.state))
            {
                slot.state.appliedRevision = slot.state.revision;
                broker_.DomainResult(slot.state.policy, HIGGS_ACTION_COMPLETED, 0);
            }
            return true;
        }
    return false;
}
bool PhysicsGrabPreference(bool left, bool configured)
{
    auto mode = PhysicalPolicies().GrabMode(left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, Now());
    return mode == HIGGS_GRAB_FORCE_PHYSICS ? true : mode == HIGGS_GRAB_IGNORE_GLOBAL_FORCE ? false : configured;
}
bool SuppressHiggsFingers(bool left)
{
    return PhysicalPolicies().SuppressFingers(left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, Now());
}
void PreparePhysicalPolicyRelease(bool left)
{
    auto hand = left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT;
    PhysicalPolicies().EndHeld(hand);
    PhysicalPolicies().Drain(Now(), hand);
}
} // namespace HiggsHands
