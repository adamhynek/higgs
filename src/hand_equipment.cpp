#include "higgsequipment.h"
#include "hand_broker.h"
#include <array>
#include <algorithm>
#include <mutex>

namespace
{
using namespace HiggsHands;
constexpr uint32_t layouts = HIGGS_EQUIP_ORDINARY | HIGGS_EQUIP_MULTI_PLACE | HIGGS_EQUIP_PROXY;
struct Entry
{
    HIGGS_EquipmentOutcome outcome{};
    uint64_t owner = 0;
    bool pendingBackend = false;
};
std::mutex equipmentMutex;
std::array<Entry, 256> history{};
size_t nextEntry = 0;

bool ValidItem(const HIGGS_EquipmentItem &i)
{
    if (i.state == HIGGS_EQUIP_EMPTY)
        return !i.originalForm && !i.equippedForm && !i.visualForm && !i.instanceProvider && !i.instance &&
               !i.generation;
    return i.state == HIGGS_EQUIP_ITEM && i.originalForm && i.equippedForm && i.instanceProvider && i.instance &&
           i.generation;
}
bool Matches(const HIGGS_EquipmentItem &a, const HIGGS_EquipmentItem &b)
{
    return a.state == b.state && a.originalForm == b.originalForm && a.equippedForm == b.equippedForm &&
           a.visualForm == b.visualForm && a.instanceProvider == b.instanceProvider && a.instance == b.instance &&
           a.generation == b.generation;
}
bool ValidTransition(const HIGGS_EquipmentTransition &r)
{
    if (r.structSize != sizeof(r) || !r.physicalHands || (r.physicalHands & ~3u) || !r.placeCount ||
        r.placeCount > HIGGS_EQUIP_MAX_PLACES || !r.requiredCapabilities || (r.requiredCapabilities & ~layouts) ||
        (r.placeCount > 1 && !(r.requiredCapabilities & HIGGS_EQUIP_MULTI_PLACE)))
        return false;
    bool changed = false;

    for (unsigned i = 0; i < r.placeCount; ++i)
    {
        const auto &p = r.places[i];
        if (p.placeKind == HIGGS_EQUIP_PLACE_BIPED)
        {
            if (p.placeId < 30 || p.placeId > 61)
                return false;
        }
        else if ((p.placeKind != HIGGS_EQUIP_PLACE_GAME_HAND && p.placeKind != HIGGS_EQUIP_PLACE_PHYSICAL_HAND) ||
                 (p.placeId != 1 && p.placeId != 2))
            return false;
        if (p.placeKind == HIGGS_EQUIP_PLACE_PHYSICAL_HAND && !(r.physicalHands & p.placeId))
            return false;
        if (!ValidItem(p.before) || !ValidItem(p.after))
            return false;
        changed |= !Matches(p.before, p.after);
        // This service cannot infer main/off hand or a biped attachment from handedness.
        if (p.placeKind != HIGGS_EQUIP_PLACE_PHYSICAL_HAND && r.physicalHands != HIGGS_HAND_BOTH)
            return false;
        if ((p.before.equippedForm != p.before.originalForm || p.after.equippedForm != p.after.originalForm) &&
            !(r.requiredCapabilities & HIGGS_EQUIP_PROXY))
            return false;
        for (unsigned j = 0; j < i; ++j)
            if (p.placeKind == r.places[j].placeKind && p.placeId == r.places[j].placeId)
                return false;
    }
    return changed;
}
Entry *Find(uint64_t epoch, uint64_t request)
{
    for (auto &entry : history)
        if (entry.outcome.epoch == epoch && entry.outcome.requestId == request)
            return &entry;
    return nullptr;
}
bool ValidEvidence(const HIGGS_EquipmentEvidence &e, uint32_t count)
{
    if (e.structSize != sizeof(e) || (e.flags & ~HIGGS_EQUIP_BACKEND_SETTLED) || e.placeCount != count ||
        e.observedMask >> count)
        return false;
    for (unsigned i = 0; i < count; ++i)
        if ((e.observedMask & (1u << i)) && !ValidItem(e.places[i]))
            return false;
    return true;
}
bool MatchesLayout(const HIGGS_EquipmentOutcome &o, const HIGGS_EquipmentEvidence &e, bool after)
{
    if (e.observedMask != ((1u << o.placeCount) - 1))
        return false;
    for (unsigned i = 0; i < o.placeCount; ++i)
        if (!Matches(e.places[i], after ? o.expected[i].after : o.expected[i].before))
            return false;
    return true;
}
void ObserveOutcome(HIGGS_EquipmentOutcome &o, const HIGGS_EquipmentEvidence &e, bool backendSettled)
{
    o.after = e;
    if (MatchesLayout(o, e, true))
    {
        o.stage = backendSettled ? HIGGS_EQUIP_CONFIRMED : HIGGS_EQUIP_AWAITING;
        o.reason = backendSettled ? HIGGS_HAND_OK : HIGGS_HAND_PENDING;
    }
    else if (MatchesLayout(o, e, false))
    {
        o.stage = backendSettled ? HIGGS_EQUIP_ORIGINAL_OBSERVED : HIGGS_EQUIP_AWAITING;
        o.reason = backendSettled ? HIGGS_HAND_OK : HIGGS_HAND_PENDING;
    }
    else
    {
        o.stage = HIGGS_EQUIP_RECOVERY_REQUIRED;
        o.reason = HIGGS_HAND_CONFLICT;
    }
}
bool Terminal(uint32_t stage)
{
    return stage == HIGGS_EQUIP_CONFIRMED || stage == HIGGS_EQUIP_DECLINED ||
           stage == HIGGS_EQUIP_ORIGINAL_OBSERVED;
}
struct Work
{
    HIGGS_EquipmentTransition request;
    HIGGS_EquipmentBackend backend;
    uint64_t owner;
};

HIGGS_HandStatus HIGGS_HAND_CALL Execute(const HIGGS_ActionRequest *selected, const HIGGS_HandSnapshot *,
                                         void *context)
{
    const auto &work = *static_cast<Work *>(context);
    auto r = work.request;
    r.authority.grant = 0; // The readback/backend receives correlation, never the executor credential.
    if ((selected->physicalHands & r.physicalHands) != r.physicalHands)
        return HIGGS_HAND_WRONG_OWNER;
    Entry *entry = nullptr;
    {
        std::lock_guard lock(equipmentMutex);
        for (size_t n = 0; n < history.size(); ++n)
        {
            auto &candidate = history[(nextEntry + n) % history.size()];
            if (!candidate.outcome.requestId || Terminal(candidate.outcome.stage) ||
                candidate.outcome.epoch != selected->epoch)
            {
                nextEntry = (nextEntry + n + 1) % history.size();
                entry = &candidate;
                break;
            }
        }
        if (!entry)
            return HIGGS_HAND_CAPACITY;
        *entry = {};
        entry->owner = work.owner;
        auto &o = entry->outcome;
        o.structSize = sizeof(o);
        o.stage = HIGGS_EQUIP_CHECKING;
        o.placeCount = r.placeCount;
        o.epoch = selected->epoch;
        o.requestId = selected->requestId;
        o.executor = selected->executor;
        std::copy_n(r.places, r.placeCount, o.expected);
    }
    auto read = [&](HIGGS_EquipmentEvidence &e) {
        e = {};
        e.structSize = sizeof(e);
        e.placeCount = r.placeCount;
        try
        {
            return work.backend.Observe(&r, &e, work.backend.context);
        }
        catch (...)
        {
            return HIGGS_HandStatus(HIGGS_HAND_INTERNAL_ERROR);
        }
    };
    HIGGS_EquipmentEvidence before{};
    auto status = read(before);
    {
        std::lock_guard lock(equipmentMutex);
        auto &o = entry->outcome;
        if (status || !ValidEvidence(before, r.placeCount) || !MatchesLayout(o, before, false))
        {
            o.stage = HIGGS_EQUIP_DECLINED;
            if (!status && ValidEvidence(before, r.placeCount))
                o.before = before;
            o.reason = status ? status : HIGGS_HAND_STALE;
            return HIGGS_HAND_OK;
        }
        o.before = before;
    }
    if (!Service().MutationAdmissionReady() || !Service().ClientValid(work.owner))
    {
        std::lock_guard lock(equipmentMutex);
        entry->outcome.stage = HIGGS_EQUIP_DECLINED;
        entry->outcome.reason = HIGGS_HAND_STALE;
        return HIGGS_HAND_OK;
    }
    try
    {
        status = work.backend.Apply(&r, work.backend.context);
    }
    catch (...)
    {
        status = HIGGS_HAND_INTERNAL_ERROR;
    }
    HIGGS_EquipmentEvidence after{};
    const auto observation = read(after);
    std::lock_guard lock(equipmentMutex);
    auto &o = entry->outcome;
    entry->pendingBackend = status == HIGGS_HAND_PENDING;
    o.stage = HIGGS_EQUIP_AWAITING;
    o.reason = status ? status : observation;
    if (observation == HIGGS_HAND_OK && ValidEvidence(after, r.placeCount))
    {
        o.after = after;
        if (MatchesLayout(o, after, true))
            ObserveOutcome(o, after, status != HIGGS_HAND_PENDING);
        else if (status != HIGGS_HAND_PENDING)
            ObserveOutcome(o, after, status != HIGGS_HAND_PENDING);
    }
    else if (status != HIGGS_HAND_PENDING && observation != HIGGS_HAND_PENDING)
    {
        o.stage = HIGGS_EQUIP_RECOVERY_REQUIRED;
    }
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus HIGGS_HAND_CALL Request(uint64_t owner, const HIGGS_EquipmentTransition *r,
                                         const HIGGS_EquipmentBackend *backend)
{
    if (!r || !ValidTransition(*r))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (!backend)
        return HIGGS_HAND_UNSUPPORTED;
    if (backend->structSize != sizeof(*backend) || !backend->Observe || !backend->Apply ||
        (backend->capabilities & ~layouts))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if ((backend->capabilities & r->requiredCapabilities) != r->requiredCapabilities)
        return HIGGS_HAND_UNSUPPORTED;
    try
    {
        Work work{*r, *backend, owner};
        return Service().InvokeAuthorized(owner, work.request.authority, Execute, &work);
    }
    catch (...)
    {
        return HIGGS_HAND_INTERNAL_ERROR;
    }
}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t owner, uint64_t epoch, uint64_t id, HIGGS_EquipmentOutcome *out)
{
    if (!epoch || !id || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (out->structSize < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    try
    {
        auto phase = Service().NativePhase();
        if (!Service().ClientValid(owner))
            return HIGGS_HAND_INVALID_CLIENT;
        const auto current = Service().Epoch();
        uint64_t executorOwner;
        {
            std::lock_guard lock(equipmentMutex);
            auto entry = Find(epoch, id);
            if (!entry)
                return HIGGS_HAND_NOT_FOUND;
            *out = entry->outcome;
            executorOwner = entry->owner;
        }
        if (!Terminal(out->stage) && (current != epoch || !Service().ClientValid(executorOwner)))
        {
            out->stage = HIGGS_EQUIP_RECOVERY_REQUIRED;
            out->reason = HIGGS_HAND_STALE;
        }
        return HIGGS_HAND_OK;
    }
    catch (...)
    {
        return HIGGS_HAND_INTERNAL_ERROR;
    }
}
HIGGS_HandStatus HIGGS_HAND_CALL Report(uint64_t owner, uint64_t epoch, uint64_t id,
                                        const HIGGS_EquipmentEvidence *evidence)
{
    if (!epoch || !id || !evidence)
        return HIGGS_HAND_INVALID_ARGUMENT;
    try
    {
        auto phase = Service().NativePhase();
        if (!Service().ClientValid(owner))
            return HIGGS_HAND_INVALID_CLIENT;
        if (Service().Epoch() != epoch)
            return HIGGS_HAND_STALE;
        std::lock_guard lock(equipmentMutex);
        auto entry = Find(epoch, id);
        if (!entry)
            return HIGGS_HAND_NOT_FOUND;
        if (entry->owner != owner)
            return HIGGS_HAND_WRONG_OWNER;
        if (!ValidEvidence(*evidence, entry->outcome.placeCount))
            return HIGGS_HAND_INVALID_ARGUMENT;
        if (entry->outcome.stage != HIGGS_EQUIP_AWAITING && entry->outcome.stage != HIGGS_EQUIP_RECOVERY_REQUIRED)
            return HIGGS_HAND_WRONG_PHASE;
        if (evidence->flags & HIGGS_EQUIP_BACKEND_SETTLED)
            entry->pendingBackend = false;
        ObserveOutcome(entry->outcome, *evidence, !entry->pendingBackend);
        return HIGGS_HAND_OK;
    }
    catch (...)
    {
        return HIGGS_HAND_INTERNAL_ERROR;
    }
}
} // namespace
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryEquipmentInterface(uint32_t version, uint32_t bytes,
                                                                          HIGGS_EquipmentInterface *out)
{
    if (version != 1)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    *out = {sizeof(*out), 1, 0, Request, Copy, Report};
    return HIGGS_HAND_OK;
}
