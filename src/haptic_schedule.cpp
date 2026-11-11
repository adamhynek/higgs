#include "haptic_schedule.h"
#include "hand_broker.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
namespace HiggsHands
{
HapticSchedule &Haptics()
{
    static HapticSchedule schedule(Service());
    return schedule;
}
void HapticSchedule::Refresh(uint64_t now)
{
    const auto snapshot = broker_.CopyPublished();
    epoch_ = snapshot.epoch;
    for (auto &e : events_)
        if (e.state.stage == HIGGS_HAPTIC_QUEUED || e.state.stage == HIGGS_HAPTIC_ACTIVE)
        {
            const auto &hand = snapshot.hands[e.state.physicalHand == HIGGS_HAND_LEFT ? 0 : 1];
            if (e.state.epoch != epoch_ || !snapshot.worldReady ||
                (e.state.owner && !broker_.ClientValid(e.state.owner)) ||
                !(hand.observationMask & HIGGS_OBS_TRACKING_VALID) ||
                (e.operation && !broker_.OperationActive(e.operation, now)))
                e.state.stage = HIGGS_HAPTIC_CANCELLED;
            else if (now >= e.state.endsAtNs)
                e.state.stage = HIGGS_HAPTIC_COMPLETED;
        }
}
HIGGS_HandStatus HapticSchedule::Request(uint64_t owner, const HIGGS_HapticRequest &r, uint64_t &out, uint64_t now,
                                         uint64_t operation, bool legacy)
{
    std::unique_lock<std::recursive_mutex> phase;
    if (owner)
        phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    Refresh(now);
    if ((!owner && !legacy) || (owner && !broker_.ClientValid(owner)))
        return HIGGS_HAND_INVALID_CLIENT;
    if (r.structSize < sizeof(r) || (r.physicalHand != HIGGS_HAND_LEFT && r.physicalHand != HIGGS_HAND_RIGHT) ||
        r.delayMs > 5000 || !r.durationMs || r.durationMs > 5000 || !std::isfinite(r.startStrength) ||
        !std::isfinite(r.endStrength) || r.startStrength < 0 || r.startStrength > 1 || r.endStrength < 0 ||
        r.endStrength > 1)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (!epoch_ || r.epoch != epoch_)
        return HIGGS_HAND_STALE;
    const auto snapshot = broker_.CopyPublished();
    if (!snapshot.worldReady ||
        !(snapshot.hands[r.physicalHand == HIGGS_HAND_LEFT ? 0 : 1].observationMask & HIGGS_OBS_TRACKING_VALID))
        return HIGGS_HAND_NOT_READY;
    Event *slot = nullptr;
    Event *replacement = nullptr;
    if (r.contextKey)
        for (auto &e : events_)
            if (e.state.owner == owner && e.request.physicalHand == r.physicalHand &&
                e.request.contextKey == r.contextKey && e.state.stage < HIGGS_HAPTIC_COMPLETED)
            {
                replacement = &e;
                break;
            }
    if (!slot)
        for (auto &e : events_)
            if (!e.state.requestId)
            {
                slot = &e;
                if (!slot)
                    slot = replacement;
                break;
            }
    if (!slot)
        for (auto &e : events_)
            if (e.state.stage >= HIGGS_HAPTIC_COMPLETED && (!slot || e.state.requestId < slot->state.requestId))
                slot = &e;
    if (!slot)
        return HIGGS_HAND_CAPACITY;
    if (r.contextKey)
        for (auto &e : events_)
            if (e.state.owner == owner && e.request.physicalHand == r.physicalHand &&
                e.request.contextKey == r.contextKey && e.state.stage < HIGGS_HAPTIC_COMPLETED)
                e.state.stage = HIGGS_HAPTIC_CANCELLED;
    *slot = {};
    slot->request = r;
    slot->operation = operation;
    auto &s = slot->state;
    s.structSize = sizeof(s);
    s.requestId = next_++;
    s.epoch = epoch_;
    s.owner = owner;
    s.physicalHand = r.physicalHand;
    s.stage = HIGGS_HAPTIC_QUEUED;
    s.dueAtNs = now + static_cast<uint64_t>(r.delayMs) * 1000000;
    s.endsAtNs = s.dueAtNs + static_cast<uint64_t>(r.durationMs) * 1000000;
    broker_.EffectivePriority(owner, s.effectivePriority);
    out = s.requestId;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus HapticSchedule::Cancel(uint64_t owner, uint64_t id)
{
    auto phase = broker_.NativePhase();
    std::lock_guard lock(mutex_);
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    for (auto &e : events_)
        if (e.state.requestId == id)
        {
            if (e.state.owner != owner)
                return HIGGS_HAND_WRONG_OWNER;
            if (e.state.stage < HIGGS_HAPTIC_COMPLETED)
                e.state.stage = HIGGS_HAPTIC_CANCELLED;
            return HIGGS_HAND_OK;
        }
    return HIGGS_HAND_NOT_FOUND;
}
HIGGS_HandStatus HapticSchedule::Copy(uint64_t owner, uint64_t id, HIGGS_HapticState &out, uint64_t now)
{
    std::lock_guard lock(mutex_);
    Refresh(now);
    if (!broker_.ClientValid(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    for (auto &e : events_)
        if (e.state.requestId == id)
        {
            if (e.state.owner != owner)
                return HIGGS_HAND_WRONG_OWNER;
            out = e.state;
            return HIGGS_HAND_OK;
        }
    return HIGGS_HAND_NOT_FOUND;
}
HapticOutput HapticSchedule::Sample(uint32_t hand, uint64_t now)
{
    std::lock_guard lock(mutex_);
    Refresh(now);
    if (hand != HIGGS_HAND_LEFT && hand != HIGGS_HAND_RIGHT)
        return {};
    auto &output = outputs_[hand == HIGGS_HAND_LEFT ? 0 : 1];
    output = {};
    output.value.hand = hand;
    output.value.atNs = now;
    output.value.serial = next_++;
    int32_t priority = (std::numeric_limits<int32_t>::min)();
    for (auto &e : events_)
        if (e.state.stage < HIGGS_HAPTIC_COMPLETED && e.state.physicalHand == hand && now >= e.state.dueAtNs)
        {
            int32_t value = 0;
            if (!broker_.EffectivePriority(e.state.owner, value))
                continue;
            e.state.effectivePriority = value;
            priority = (std::max)(priority, value);
        }
    for (auto &e : events_)
        if (e.state.requestId && e.state.stage < HIGGS_HAPTIC_COMPLETED && e.state.physicalHand == hand &&
            now >= e.state.dueAtNs)
        {
            if (e.state.effectivePriority != priority)
            {
                ++e.state.suppressedSamples;
                continue;
            }
            e.state.stage = HIGGS_HAPTIC_ACTIVE;
            const double t =
                static_cast<double>(now - e.state.dueAtNs) / static_cast<double>(e.state.endsAtNs - e.state.dueAtNs);
            const auto strength =
                static_cast<float>(e.request.startStrength + (e.request.endStrength - e.request.startStrength) * t);
            output.value.strength = (std::max)(output.value.strength, strength);
            output.requests[output.count++] = e.state.requestId;
        }
    return output.value;
}
void HapticSchedule::Submitted(const HapticOutput &sample, bool accepted)
{
    std::lock_guard lock(mutex_);
    if (sample.hand != HIGGS_HAND_LEFT && sample.hand != HIGGS_HAND_RIGHT)
        return;
    auto &output = outputs_[sample.hand == HIGGS_HAND_LEFT ? 0 : 1];
    if (output.value.serial != sample.serial || !sample.serial || output.reported)
        return;
    output.reported = true;
    for (size_t i = 0; i < output.count; ++i)
        for (auto &e : events_)
            if (e.state.requestId == output.requests[i] && e.state.stage < HIGGS_HAPTIC_COMPLETED)
            {
                if (accepted)
                {
                    ++e.state.submittedSamples;
                    e.state.lastSubmittedAtNs = sample.atNs;
                }
                else
                    ++e.state.unavailableSamples;
            }
}
void HapticSchedule::Legacy(uint32_t hand, float start, float end, float seconds, uint64_t now)
{
    if (!std::isfinite(seconds) || seconds < 0)
        return;
    HIGGS_HapticRequest r{};
    r.structSize = sizeof(r);
    r.physicalHand = hand;
    r.epoch = broker_.Epoch();
    r.durationMs = static_cast<uint32_t>((std::max)(1.f, (std::min)(seconds * 1000.f, 5000.f)));
    r.startStrength = std::clamp(start, 0.f, 1.f);
    r.endStrength = std::clamp(end, 0.f, 1.f);
    uint64_t ignored = 0;
    Request(0, r, ignored, now, 0, true);
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
HIGGS_HandStatus HIGGS_HAND_CALL Request(uint64_t owner, const HIGGS_HapticRequest *r, uint64_t *out)
{
    if (!r || r->structSize < sizeof(*r) || !out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    return Guard([&] {
        uint64_t value = 0;
        auto s = HiggsHands::Haptics().Request(owner, *r, value, HiggsHands::Now());
        if (!s)
            *out = value;
        return s;
    });
}
HIGGS_HandStatus HIGGS_HAND_CALL Cancel(uint64_t owner, uint64_t request)
{
    return Guard([&] { return HiggsHands::Haptics().Cancel(owner, request); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Copy(uint64_t owner, uint64_t request, HIGGS_HapticState *out)
{
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (out->structSize < sizeof(*out))
    {
        out->structSize = sizeof(*out);
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    }
    return Guard([&] {
        HIGGS_HapticState value{};
        auto s = HiggsHands::Haptics().Copy(owner, request, value, HiggsHands::Now());
        if (!s)
            std::memcpy(out, &value, sizeof(value));
        return s;
    });
}
} // namespace
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryHapticsInterface(uint32_t version, uint32_t bytes,
                                                                        HIGGS_HapticsInterface *out)
{
    if (version != 1)
        return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out))
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_HapticsInterface table{sizeof(table), 1, 1, Request, Cancel, Copy};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
