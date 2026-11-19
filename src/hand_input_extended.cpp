#include "hand_input_extended.h"
#include "hand_broker.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
namespace HiggsHands
{
namespace
{
thread_local uint64_t controllerFrames[2]{};
}
uint64_t CurrentControllerFrameId(uint32_t hand) noexcept
{
    return hand >= 1 && hand <= 2 ? controllerFrames[hand - 1] : 0;
}
uint64_t KnownControlMask() noexcept
{
    return UINT64_C(0xff) | (UINT64_C(1) << 31) | (UINT64_C(31) << 32);
}
uint64_t ActionControlMask() noexcept
{
    return KnownControlMask() & ~((UINT64_C(1) << HIGGS_CONTROL_SYSTEM) | (UINT64_C(1) << HIGGS_CONTROL_PROXIMITY));
}
uint32_t InputControlId(uint32_t button) noexcept
{
    return button == HIGGS_INPUT_GRIP                                           ? HIGGS_CONTROL_GRIP
           : button == HIGGS_INPUT_TRIGGER                                      ? HIGGS_CONTROL_TRIGGER
           : button >= kAuxiliaryInputBase && button < kAuxiliaryInputBase + 64 ? button - kAuxiliaryInputBase
                                                                                : UINT32_MAX;
}
uint32_t InputControlSlot(uint32_t button) noexcept
{
    return button == HIGGS_INPUT_GRIP                                           ? 0u
           : button == HIGGS_INPUT_TRIGGER                                      ? 1u
           : button >= kAuxiliaryInputBase && button < kAuxiliaryInputBase + 64 ? 2u + button - kAuxiliaryInputBase
                                                                                : UINT32_MAX;
}
HIGGS_HandStatus ValidateInputCandidate(const HIGGS_InputCandidate &c) noexcept
{
    if (c.structSize != sizeof(c) || c.gesture > HIGGS_GESTURE_MULTI_TAP ||
        c.defaultPolicy != HIGGS_DEFAULT_DISCARD_AFTER_SELECTION || c.candidate.structSize != sizeof(c.candidate))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (c.gesture == HIGGS_GESTURE_IMMEDIATE)
        return !c.deadlineMs && !c.holdMs && !c.maxTapMs && !c.tapGapMs && !c.tapCount ? HIGGS_HAND_OK
                                                                                       : HIGGS_HAND_INVALID_ARGUMENT;
    if (!c.deadlineMs || c.deadlineMs > 60000 || c.deadlineMs > c.candidate.grantDurationMs)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (c.gesture == HIGGS_GESTURE_HOLD)
        return c.holdMs && c.holdMs < c.deadlineMs && !c.maxTapMs && !c.tapGapMs && !c.tapCount
                   ? HIGGS_HAND_OK
                   : HIGGS_HAND_INVALID_ARGUMENT;
    return !c.holdMs && c.maxTapMs && c.maxTapMs <= c.deadlineMs && c.tapCount && c.tapCount <= 8 &&
                   (c.tapCount == 1 ? !c.tapGapMs : (c.tapGapMs && c.tapGapMs <= c.deadlineMs))
               ? HIGGS_HAND_OK
               : HIGGS_HAND_INVALID_ARGUMENT;
}
namespace
{
bool Valid(const HIGGS_ControllerFrame &f)
{
    if (f.structSize != sizeof(f) || f.physicalHand < 1 || f.physicalHand > 2 || !f.epoch || !f.observedAtNs ||
        f.frameId || f.axisValidMask > 31 || f.status > HIGGS_INPUT_FRAME_DISCONNECTED || f.reserved ||
        (f.flags & ~3u) || !(f.flags & HIGGS_INPUT_NO_HARDWARE_TIME))
        return false;
    if (f.status == HIGGS_INPUT_FRAME_READY && (!f.deviceIndex || f.deviceIndex >= 64))
        return false;
    if (f.status != HIGGS_INPUT_FRAME_READY && (f.pressed || f.touched || f.axisValidMask))
        return false;
    for (unsigned i = 0; i < 5; ++i)
    {
        if (f.axisTypes[i] > HIGGS_AXIS_UNKNOWN ||
            ((f.axisValidMask & (1u << i)) && (!f.axisTypes[i] || f.axisTypes[i] >= HIGGS_AXIS_UNKNOWN)))
            return false;
        for (float v : f.axes[i])
            if ((f.axisValidMask & (1u << i)) ? (!std::isfinite(v) || v < -1.001f || v > 1.001f) : (v != 0))
                return false;
    }
    return true;
}
} // namespace
void InputFrameStore::EpochLocked(uint64_t epoch)
{
    if (!epoch || epoch <= epoch_)
        return;
    epoch_ = epoch;
    latest_ = {};
    history_ = {};
    pressedAt_ = {};
    suppressed_[0] = suppressed_[1] = 0;
    count_ = cursor_ = 0;
}
void InputFrameStore::Epoch(uint64_t epoch)
{
    std::lock_guard lock(mutex_);
    EpochLocked(epoch);
}
bool InputFrameStore::Publish(HIGGS_ControllerFrame &frame, std::array<HIGGS_InputEvent, 96> &events, uint32_t &count)
{
    if (!Valid(frame))
        return false;
    std::lock_guard lock(mutex_);
    EpochLocked(frame.epoch);
    if (frame.epoch != epoch_ || nextFrame_ == (std::numeric_limits<uint64_t>::max)())
        return false;
    const auto hand = frame.physicalHand - 1;
    auto previous = latest_[hand];
    if (previous.frameId && frame.observedAtNs < previous.observedAtNs)
        return false;
    if (previous.deviceIndex != frame.deviceIndex)
        previous = {};
    const bool baseline = !previous.frameId;
    if (baseline)
        frame.flags |= HIGGS_INPUT_BASELINE;
    frame.frameId = nextFrame_++;
    count = 0;
    const auto emit = [&](uint32_t id, uint32_t kind, uint32_t axis = UINT32_MAX) {
        auto &e = events[count++];
        e = {};
        e.structSize = sizeof(e);
        e.physicalHand = frame.physicalHand;
        e.controlId = id;
        e.kind = kind;
        e.axisIndex = axis;
        e.frame = frame;
        if (pressedAt_[hand][id] && frame.observedAtNs >= pressedAt_[hand][id])
            e.heldDurationNs = frame.observedAtNs - pressedAt_[hand][id];
    };
    for (unsigned id = 0; id < 64; ++id)
        if (KnownControlMask() & (UINT64_C(1) << id))
        {
            const auto bit = UINT64_C(1) << id;
            const bool before = (previous.pressed & bit) != 0, after = (frame.pressed & bit) != 0;
            if (after && !before)
            {
                pressedAt_[hand][id] = frame.observedAtNs;
                emit(id, baseline ? HIGGS_RAW_HELD : HIGGS_RAW_PRESS);
            }
            else if (before && !after)
            {
                emit(id, HIGGS_RAW_RELEASE);
                pressedAt_[hand][id] = 0;
            }
            else if (after)
                emit(id, HIGGS_RAW_HELD);
            if (!baseline && (previous.touched & bit) != (frame.touched & bit))
                emit(id, (frame.touched & bit) ? HIGGS_RAW_TOUCH : HIGGS_RAW_UNTOUCH);
        }
    for (unsigned i = 0; i < 5; ++i)
        if ((previous.axisValidMask & (1u << i)) != (frame.axisValidMask & (1u << i)) ||
            previous.axisTypes[i] != frame.axisTypes[i] || previous.axes[i][0] != frame.axes[i][0] ||
            previous.axes[i][1] != frame.axes[i][1])
            emit(32 + i, HIGGS_RAW_AXIS, i);
    latest_[hand] = frame;
    suppressed_[hand] = 0;
    history_[cursor_] = frame;
    cursor_ = (cursor_ + 1) % history_.size();
    if (count_ < history_.size())
        ++count_;
    return true;
}
HIGGS_HandStatus InputFrameStore::Copy(uint64_t epoch, HIGGS_InputSnapshot &out)
{
    std::lock_guard lock(mutex_);
    if (epoch != epoch_)
        return HIGGS_HAND_STALE;
    out = {};
    out.structSize = sizeof(out);
    out.epoch = epoch_;
    out.hands[0] = latest_[0];
    out.hands[1] = latest_[1];
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus InputFrameStore::History(const HIGGS_InputFrameQuery &q, HIGGS_ControllerFrame *output,
                                          uint32_t capacity, uint32_t &count)
{
    if (q.structSize != sizeof(q) || !q.epoch || !q.physicalHands || (q.physicalHands & ~3u) ||
        capacity > history_.size() || (capacity && !output))
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto rows = std::make_unique<std::array<HIGGS_ControllerFrame, 128>>();
    std::lock_guard lock(mutex_);
    if (q.epoch != epoch_)
        return HIGGS_HAND_STALE;
    uint32_t found = 0;
    for (size_t i = 0; i < count_; ++i)
        if (history_[i].frameId > q.afterFrame && (history_[i].physicalHand & q.physicalHands))
            (*rows)[found++] = history_[i];
    if (capacity < found)
    {
        count = found;
        return HIGGS_HAND_BUFFER_TOO_SMALL;
    }
    std::sort(rows->begin(), rows->begin() + found, [](const auto &a, const auto &b) { return a.frameId < b.frameId; });
    for (unsigned i = 0; i < found; ++i)
        output[i] = (*rows)[i];
    count = found;
    return HIGGS_HAND_OK;
}
bool InputFrameStore::Latest(uint32_t hand, HIGGS_ControllerFrame &out)
{
    if (hand < 1 || hand > 2)
        return false;
    std::lock_guard lock(mutex_);
    out = latest_[hand - 1];
    return out.frameId != 0;
}
void InputFrameStore::RetainSuppression(uint32_t hand, uint64_t frameId, uint64_t mask)
{
    if (hand < 1 || hand > 2)
        return;
    std::lock_guard lock(mutex_);
    if (latest_[hand - 1].frameId == frameId)
        suppressed_[hand - 1] = mask;
}
uint64_t InputFrameStore::Suppression(uint32_t hand, uint64_t frameId)
{
    if (hand < 1 || hand > 2)
        return 0;
    std::lock_guard lock(mutex_);
    return latest_[hand - 1].frameId == frameId ? suppressed_[hand - 1] : 0;
}
InputFrameStore &InputFrames()
{
    static InputFrameStore value;
    return value;
}
void BeginControllerFrame(HIGGS_ControllerFrame frame)
{
    if (frame.physicalHand >= 1 && frame.physicalHand <= 2)
        controllerFrames[frame.physicalHand - 1] = 0;
    try
    {
        std::array<HIGGS_InputEvent, 96> events{};
        uint32_t count = 0;
        if (!InputFrames().Publish(frame, events, count))
            return;
        controllerFrames[frame.physicalHand - 1] = frame.frameId;
        Service().ObserveContextFrame(frame);
        if (frame.flags & HIGGS_INPUT_BASELINE)
            Service().PrimeInputFrame(frame);
        Service().NotifyInputObservations(events.data(), count);
        const uint64_t mask =
            ActionControlMask() & ~((UINT64_C(1) << HIGGS_CONTROL_GRIP) | (UINT64_C(1) << HIGGS_CONTROL_TRIGGER));
        uint64_t consumed = 0;
        for (unsigned id = 0; id < 64; ++id)
            if (mask & (UINT64_C(1) << id))
                if (Service().Input(frame.physicalHand, kAuxiliaryInputBase + id,
                                    (frame.pressed & (UINT64_C(1) << id)) != 0, frame.observedAtNs))
                    consumed |= UINT64_C(1) << id;
        InputFrames().RetainSuppression(frame.physicalHand, frame.frameId, consumed);
    }
    catch (...)
    { /* immutable frame failure does not escape a controller callback */
    }
}
void FinishControllerFrame(uint32_t hand, uint64_t &pressed, uint64_t &touched)
{
    try
    {
        HIGGS_ControllerFrame frame{};
        uint64_t mask = Service().InputSuppressionMask(hand);
        if (InputFrames().Latest(hand, frame) && frame.frameId == CurrentControllerFrameId(hand))
        {
            Service().AdvanceInputRecognitions(frame);
            mask |= Service().InputSuppressionMask(hand) | InputFrames().Suppression(hand, frame.frameId);
        }
        uint64_t pressedMask = mask, touchedMask = mask;
        Service().ContextSuppression(hand, CurrentControllerFrameId(hand), pressedMask, touchedMask);
        pressed &= ~pressedMask;
        touched &= ~touchedMask;
    }
    catch (...)
    {
    }
}
} // namespace HiggsHands
