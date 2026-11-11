#include "hand_support.h"
#include "hand_broker.h"
#include <cmath>
#include <cstring>
namespace HiggsHands
{
namespace
{
bool Ended(uint32_t stage)
{
    return stage == HIGGS_SUPPORT_RELEASED || stage == HIGGS_SUPPORT_BROKEN;
}
bool Constraint(uint32_t kind)
{
    return kind == HIGGS_CONSTRAINT_HELD_TO_WORLD || kind == HIGGS_CONSTRAINT_HELD_TO_PLANCK;
}
bool Point(const float *p, size_t n)
{
    for (size_t i = 0; i < n; ++i)
        if (!std::isfinite(p[i]))
            return false;
    return true;
}
bool Rotation(const float *r)
{
    if (!Point(r, 9))
        return false;
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
        {
            float d = 0;
            for (size_t k = 0; k < 3; ++k)
                d += r[i * 3 + k] * r[j * 3 + k];
            if (std::fabs(d - (i == j ? 1.f : 0.f)) > .001f)
                return false;
        }
    return r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
               r[2] * (r[3] * r[7] - r[4] * r[6]) >
           .999f;
}
} // namespace
SupportService &Supports()
{
    static SupportService service(Service());
    return service;
}
SupportService::Slot *SupportService::Find(uint64_t id)
{
    for (auto &slot : slots_)
        if (slot.state.requestId == id && id)
            return &slot;
    return nullptr;
}
void SupportService::Install(Driver driver, void *context, uint64_t caps)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    driver_ = driver;
    context_ = context;
    capabilities_ = caps;
}
uint64_t SupportService::Capabilities()
{
    std::lock_guard lock(mutex_);
    return capabilities_;
}
HIGGS_HandStatus SupportService::Request(uint64_t owner, const HIGGS_SupportRequest &r, uint64_t &out, uint64_t now)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (r.structSize < sizeof(r) || r.reserved || r.kind < 1 || r.kind > 5 ||
        (r.physicalHand != 1 && r.physicalHand != 2) || !r.durationMs || r.durationMs > 60000 ||
        !r.maxObservationAgeMs || r.maxObservationAgeMs > 1000 || !Point(r.targetLocalPoint, 3) ||
        !Point(r.sourceLocalPoint, 3) || !Rotation(r.targetLocalRotation) || !std::isfinite(r.maxDistanceWorld) ||
        r.maxDistanceWorld < 1 || r.maxDistanceWorld > 200 || !std::isfinite(r.linearForceCap) ||
        !std::isfinite(r.angularForceCap) || r.linearForceCap < 0 || r.angularForceCap < 0)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (!driver_ || !(capabilities_ & (UINT64_C(1) << (r.kind - 1))))
        return HIGGS_HAND_UNSUPPORTED;
    HIGGS_HandSnapshot snapshot{};
    auto status = broker_.Copy(owner, snapshot, now);
    if (status)
        return status;
    if (r.epoch != snapshot.epoch)
        return HIGGS_HAND_STALE;
    const auto &h = snapshot.hands[r.physicalHand == 1 ? 0 : 1];
    if (h.generation != r.handGeneration)
        return HIGGS_HAND_STALE;
    if (Constraint(r.kind))
    {
        if (!r.sourceItemRefId || h.heldRefId != r.sourceItemRefId)
            return HIGGS_HAND_CONFLICT;
    }
    else if (h.heldRefId || (h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT))
        return HIGGS_HAND_CONFLICT;
    if ((r.kind == HIGGS_SUPPORT_MOVING_WORLD || r.kind == HIGGS_CONSTRAINT_HELD_TO_WORLD) && !r.referenceFormId)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if ((r.kind == HIGGS_SUPPORT_PLANCK_HAND || r.kind == HIGGS_CONSTRAINT_HELD_TO_PLANCK) &&
        (!r.providerBody.epoch || !r.providerBody.actorHandle || !r.providerBody.actorGeneration ||
         !r.providerBody.bodyGeneration))
        return HIGGS_HAND_INVALID_ARGUMENT;
    for (const auto &slot : slots_)
        if (slot.state.requestId && !Ended(slot.state.stage) && slot.state.request.physicalHand == r.physicalHand)
            return HIGGS_HAND_CONFLICT;
    Slot *slot = nullptr;
    for (auto &candidate : slots_)
        if (!candidate.state.requestId)
        {
            slot = &candidate;
            break;
        }
    if (!slot)
        for (auto &candidate : slots_)
            if (Ended(candidate.state.stage) && (!slot || candidate.state.requestId < slot->state.requestId))
                slot = &candidate;
    if (!slot)
        return HIGGS_HAND_CAPACITY;
    uint32_t hands = r.physicalHand;
    if (Constraint(r.kind))
        for (const auto &other : snapshot.hands)
            if (other.heldRefId == h.heldRefId)
                hands |= other.physicalHand;
    HIGGS_ActionRequest authority{};
    status = broker_.ReserveDomain(owner, 0, hands, r.epoch, r.grant, r.durationMs, authority, now);
    if (status)
        return status;
    status = broker_.ClaimActuator(authority.requestId, Constraint(r.kind) ? 2u : 1u);
    if (status)
    {
        broker_.RetainRequest(authority.requestId, false);
        broker_.CancelNativeRequest(owner, authority.requestId);
        if (!r.grant)
            broker_.Release(owner, authority.grant);
        return status;
    }
    *slot = {};
    slot->ownGrant = !r.grant;
    slot->deadline = now + static_cast<uint64_t>(r.durationMs) * 1000000;
    auto &state = slot->state;
    state.structSize = sizeof(state);
    state.stage = HIGGS_SUPPORT_PENDING;
    state.requestId = authority.requestId;
    state.owner = owner;
    state.grant = authority.grant;
    state.epoch = r.epoch;
    state.request = r;
    state.requestRevision = 1;
    out = state.requestId;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus SupportService::Update(uint64_t owner, uint64_t id, const HIGGS_SupportUpdate &update)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto slot = Find(id);
    if (!slot)
        return HIGGS_HAND_NOT_FOUND;
    auto &s = slot->state;
    if (s.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    if (update.structSize < sizeof(update) || update.reserved || !Point(update.targetLocalPoint, 3) ||
        !Rotation(update.targetLocalRotation))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (update.epoch != s.epoch || update.epoch != broker_.Epoch() || update.expectedRevision != s.requestRevision ||
        Ended(s.stage) || s.stage == HIGGS_SUPPORT_ENDING || !broker_.DomainAlive(owner, id, Now()))
        return HIGGS_HAND_STALE;
    if (!broker_.GameplayContext()) {
        for (unsigned i = 0; i < 3; ++i)
            if (update.targetLocalPoint[i] != s.request.targetLocalPoint[i]) return HIGGS_HAND_NOT_READY;
        for (unsigned i = 0; i < 9; ++i)
            if (update.targetLocalRotation[i] != s.request.targetLocalRotation[i]) return HIGGS_HAND_NOT_READY;
    }
    std::memcpy(s.request.targetLocalPoint, update.targetLocalPoint, sizeof(update.targetLocalPoint));
    std::memcpy(s.request.targetLocalRotation, update.targetLocalRotation, sizeof(update.targetLocalRotation));
    ++s.requestRevision;
    s.flags &= ~HIGGS_SUPPORT_NATIVE_APPLIED;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus SupportService::End(uint64_t owner, uint64_t id, uint64_t epoch)
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
    if (epoch != slot->state.epoch || epoch != broker_.Epoch())
        return HIGGS_HAND_STALE;
    if (!Ended(slot->state.stage))
        slot->state.stage = HIGGS_SUPPORT_ENDING;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus SupportService::Copy(uint64_t owner, uint64_t id, HIGGS_SupportState &out)
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
void SupportService::Break(uint64_t id, uint32_t reason)
{
    std::lock_guard lock(mutex_);
    if (auto slot = Find(id))
        if (!Ended(slot->state.stage))
        {
            slot->state.reason = reason;
            slot->state.stage = HIGGS_SUPPORT_ENDING;
        }
}
void SupportService::BreakHand(uint32_t hand, uint32_t reason)
{
    std::lock_guard lock(mutex_);
    for (auto &slot : slots_)
        if (slot.state.request.physicalHand == hand && !Ended(slot.state.stage))
        {
            slot.state.stage = HIGGS_SUPPORT_ENDING;
            slot.state.reason = reason;
        }
}
void SupportService::Drain(uint64_t now, uint32_t endingHand)
{
    auto phase = broker_.NativePhase();
    for (size_t index = 0; index < slots_.size(); ++index)
    {
        HIGGS_SupportState state{};
        Driver driver = nullptr;
        void *context = nullptr;
        {
            std::lock_guard lock(mutex_);
            auto &slot = slots_[index];
            if (!slot.state.requestId || Ended(slot.state.stage) ||
                (endingHand &&
                 (slot.state.request.physicalHand != endingHand || slot.state.stage != HIGGS_SUPPORT_ENDING)))
                continue;
            if (slot.state.epoch != broker_.Epoch() ||
                !broker_.DomainAlive(slot.state.owner, slot.state.requestId, now) || now >= slot.deadline)
            {
                slot.state.stage = HIGGS_SUPPORT_ENDING;
                slot.state.reason =
                    slot.state.epoch != broker_.Epoch() ? HIGGS_SUPPORT_BREAK_EPOCH : HIGGS_SUPPORT_BREAK_OWNER;
            }
            state = slot.state;
            driver = driver_;
            context = context_;
        }
        const bool ending = state.stage == HIGGS_SUPPORT_ENDING;
        if (!broker_.EnterDomainMutation(state.requestId, ending))
            continue;
        HIGGS_HandStatus status = HIGGS_HAND_INTERNAL_ERROR;
        try
        {
            status = driver ? driver(state, ending, context) : HIGGS_HAND_NOT_READY;
        }
        catch (...)
        {
        }
        broker_.ExitDomainMutation(state.requestId);
        std::lock_guard lock(mutex_);
        auto slot = Find(state.requestId);
        if (!slot)
            continue;
        if (ending)
        {
            if (status == HIGGS_HAND_OK)
            {
                state.stage = state.reason ? HIGGS_SUPPORT_BROKEN : HIGGS_SUPPORT_RELEASED;
                slot->state = state;
                broker_.UnclaimActuator(state.requestId);
                broker_.RetainRequest(state.requestId, false);
                broker_.CancelNativeRequest(state.owner, state.requestId);
                if (slot->ownGrant)
                    broker_.Release(state.owner, state.grant);
            }
            continue;
        }
        if (status == HIGGS_HAND_PENDING || status == HIGGS_HAND_NOT_READY)
            continue;
        if (status != HIGGS_HAND_OK)
        {
            slot->state.stage = HIGGS_SUPPORT_ENDING;
            slot->state.reason = HIGGS_SUPPORT_BREAK_ENDPOINT;
            continue;
        }
        state.stage = HIGGS_SUPPORT_ACTIVE;
        state.validatedAtNs = now;
        if (state.flags & HIGGS_SUPPORT_NATIVE_APPLIED)
            state.appliedRevision = state.requestRevision;
        slot->state = state;
        if (state.flags & (HIGGS_SUPPORT_NATIVE_APPLIED | HIGGS_SUPPORT_CONSTRAINT_ATTACHED))
            broker_.DomainResult(state.requestId, HIGGS_ACTION_COMPLETED, HIGGS_RECEIPT_PHYSICAL_CONFIRMED, 0, 0);
    }
}
bool SupportService::Goal(uint32_t hand, HIGGS_SupportState &out, uint64_t now)
{
    std::lock_guard lock(mutex_);
    for (auto &slot : slots_)
    {
        auto &s = slot.state;
        if (s.request.physicalHand != hand || s.stage != HIGGS_SUPPORT_ACTIVE ||
            !(s.flags & HIGGS_SUPPORT_TARGET_VALID))
            continue;
        if (now < s.validatedAtNs ||
            now - s.validatedAtNs > static_cast<uint64_t>(s.request.maxObservationAgeMs) * 1000000 ||
            !broker_.DomainAlive(s.owner, s.requestId, now))
            return false;
        out = s;
        return true;
    }
    return false;
}
void SupportService::Applied(uint64_t id, uint64_t now)
{
    std::lock_guard lock(mutex_);
    if (auto slot = Find(id))
        if (slot->state.stage == HIGGS_SUPPORT_ACTIVE && now >= slot->state.validatedAtNs &&
            now - slot->state.validatedAtNs <= uint64_t(slot->state.request.maxObservationAgeMs) * 1000000)
        {
            slot->state.flags |= HIGGS_SUPPORT_NATIVE_APPLIED;
            slot->state.appliedRevision = slot->state.requestRevision;
            broker_.DomainResult(id, HIGGS_ACTION_COMPLETED, HIGGS_RECEIPT_PHYSICAL_CONFIRMED, 0, 0);
        }
}
bool CopySupportTarget(bool left, float position[3], float rotation[9], float &scale)
{
    HIGGS_SupportState state{};
    if (!Supports().Goal(left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, state, Now()))
        return false;
    std::memcpy(position, state.handPosition, sizeof(state.handPosition));
    std::memcpy(rotation, state.handRotation, sizeof(state.handRotation));
    scale = state.handScale;
    return true;
}
void ConfirmSupportTarget(bool left)
{
    HIGGS_SupportState state{};
    if (Supports().Goal(left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, state, Now()))
        Supports().Applied(state.requestId, Now());
}
void PrepareSupportRelease(bool left)
{
    const uint32_t hand = left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT;
    Supports().BreakHand(hand, HIGGS_SUPPORT_BREAK_OWNER);
    Supports().Drain(Now(), hand);
}
void DrainNativeSupport()
{
    Supports().Drain(Now());
}
} // namespace HiggsHands
namespace
{
template <class F> HIGGS_HandStatus Guard(F f) noexcept
{
    try
    {
        return f();
    }
    catch (...)
    {
        return HIGGS_HAND_INTERNAL_ERROR;
    }
}
HIGGS_HandStatus HIGGS_HAND_CALL Request(uint64_t owner, const HIGGS_SupportRequest *r, uint64_t *out)
{
    if (!r || r->structSize < sizeof(*r) || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return Guard([&] {
        uint64_t value = 0;
        auto s = HiggsHands::Supports().Request(owner, *r, value, HiggsHands::Now());
        if (!s)
            *out = value;
        return s;
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Update(uint64_t owner, uint64_t id, const HIGGS_SupportUpdate *u)
{
    if (!u || u->structSize < sizeof(*u))
        return HIGGS_HAND_INVALID_ARGUMENT;
    return Guard([&] { return HiggsHands::Supports().Update(owner, id, *u); });
}
HIGGS_HandStatus HIGGS_HAND_CALL End(uint64_t owner, uint64_t id, uint64_t epoch)
{
    return Guard([&] { return HiggsHands::Supports().End(owner, id, epoch); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t owner, uint64_t id, HIGGS_SupportState *out)
{
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (out->structSize < sizeof(*out))
    {
        out->structSize = sizeof(*out);
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    }
    return Guard([&] {
        HIGGS_SupportState value{};
        auto status = HiggsHands::Supports().Copy(owner, id, value);
        if (!status)
            std::memcpy(out, &value, sizeof(value));
        return status;
    });
}
} // namespace
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QuerySupportInterface(uint32_t version, uint32_t bytes,
                                                                        HIGGS_SupportInterface *out)
{
    if (version != 1)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_SupportInterface table{sizeof(table), 1,   HiggsHands::Supports().Capabilities(), Request, Update,
                                       End,           Copy};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
