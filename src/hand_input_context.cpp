#include "hand_broker.h"
#include <cstring>
namespace HiggsHands {
HIGGS_InputContextState Broker::ContextStateLocked() const
{
    HIGGS_InputContextState state{};
    state.structSize = sizeof(state);
    state.kind = contextMenus_ ? HIGGS_CONTEXT_UI : HIGGS_CONTEXT_GAMEPLAY;
    state.activeMenus = contextMenus_;
    state.flags = HIGGS_CONTEXT_NO_REPLAY;
    state.epoch = snapshot_.epoch;
    state.generation = contextGeneration_;
    state.changedAtNs = contextChangedAt_;
    for (unsigned i = 0; i < 2; ++i) {
        const auto &hand = contextHands_[i];
        state.hands[i] = {hand.frame, hand.drainPressed, hand.drainTouched};
        if (hand.drainPressed || hand.drainTouched || !hand.known) state.flags |= HIGGS_CONTEXT_DRAINING;
    }
    return state;
}
void Broker::SetInputContext(uint32_t menus, Clock now)
{
    auto phase = NativePhase();
    {
        std::lock_guard lock(mutex_);
        if (menus == contextMenus_) return;
        const auto previous = ContextStateLocked();
        contextMenus_ = menus;
        ++contextGeneration_;
        contextChangedAt_ = now;
        for (auto &hand : contextHands_) {
            hand.drainPressed |= hand.known ? hand.pressed : ActionControlMask();
            hand.drainTouched |= hand.known ? hand.touched : ActionControlMask();
        }
        // Possession, committed effects, sessions and entered mutations survive.
        // A dispatched external executor owns its eventual receipt/recovery path.
        for (auto &a : actions_)
            if (a.request.requestId && a.result.stage < HIGGS_ACTION_COMMITTED && !a.nativeStarted &&
                !a.sessionKind && a.physical.kind != HIGGS_PHYSICAL_RELEASE &&
                (!a.dispatched || a.recognition.stage == HIGGS_RECOGNITION_WAITING)) {
                a.grantActive = false;
                a.operationActive = false;
                if (a.recognition.stage == HIGGS_RECOGNITION_WAITING) a.recognition.stage = HIGGS_RECOGNITION_CANCELLED;
                FinishLocked(a, HIGGS_ACTION_CANCELLED, HIGGS_HAND_NOT_READY);
            }
        RefreshLocked();
        pendingContexts_.emplace_back(previous, ContextStateLocked());
    }
    FlushEvents();
}
void Broker::ObserveContextFrame(const HIGGS_ControllerFrame &frame)
{
    if (frame.physicalHand < 1 || frame.physicalHand > 2) return;
    std::lock_guard lock(mutex_);
    if (frame.epoch != snapshot_.epoch) return;
    auto &hand = contextHands_[frame.physicalHand - 1];
    const auto mask = ActionControlMask();
    const auto pressed = frame.pressed & mask, touched = frame.touched & mask;
    if (!hand.known || hand.device != frame.deviceIndex || (frame.flags & HIGGS_INPUT_BASELINE)) {
        hand.drainPressed |= pressed;
        hand.drainTouched |= touched;
    }
    // Save the union BEFORE clearing released bits: its matching Up is drained too.
    HIGGS_InputContextFrame stamp{sizeof(stamp), frame.physicalHand,
        contextMenus_ ? uint32_t(HIGGS_CONTEXT_UI) : uint32_t(HIGGS_CONTEXT_GAMEPLAY), HIGGS_CONTEXT_NO_REPLAY,
        frame.epoch, frame.frameId, contextGeneration_, hand.drainPressed, hand.drainTouched};
    if (stamp.drainPressed || stamp.drainTouched) stamp.flags |= HIGGS_CONTEXT_DRAINING;
    contextFrames_[frame.frameId % contextFrames_.size()] = stamp;
    hand.frame = frame.frameId;
    hand.device = frame.deviceIndex;
    hand.known = frame.status == HIGGS_INPUT_FRAME_READY;
    hand.touchGrip = (frame.flags & HIGGS_INPUT_GRIP_USES_TOUCH) != 0;
    hand.pressed = pressed;
    hand.touched = touched;
    hand.drainPressed &= pressed;
    hand.drainTouched &= touched;
}
bool Broker::GameplayInputLocked(uint32_t physicalHand, uint32_t button) const
{
    if (contextMenus_ || physicalHand < 1 || physicalHand > 2) return false;
    const auto &hand = contextHands_[physicalHand - 1];
    const auto control = InputControlId(button);
    if (control >= 64) return false;
    const auto &frame = contextFrames_[hand.frame % contextFrames_.size()];
    const auto mask = hand.frame && frame.frameId == hand.frame && frame.generation == contextGeneration_
        ? (control == HIGGS_CONTROL_GRIP && hand.touchGrip ? frame.drainTouched : frame.drainPressed)
        : (control == HIGGS_CONTROL_GRIP && hand.touchGrip ? hand.drainTouched : hand.drainPressed);
    return !(mask & (UINT64_C(1) << control));
}
bool Broker::GameplayInput(uint32_t hand, uint32_t button)
{
    std::lock_guard lock(mutex_);
    return GameplayInputLocked(hand, button);
}
bool Broker::GameplayContext() { std::lock_guard lock(mutex_); return !contextMenus_; }
bool Broker::RebaseNativeInput(uint32_t hand) {
    std::lock_guard lock(mutex_);
    if (hand < 1 || hand > 2) return true;
    auto &generation = contextHands_[hand - 1].nativeGeneration;
    // Zero means no native observation in this lifetime, not a clean legacy
    // Hand history. Menus/load can change without a controller callback between.
    const bool changed = generation != contextGeneration_;
    generation = contextGeneration_;
    return changed;
}
void Broker::ContextSuppression(uint32_t physicalHand, uint64_t frame, uint64_t &pressed, uint64_t &touched)
{
    std::lock_guard lock(mutex_);
    if (physicalHand < 1 || physicalHand > 2) return;
    const auto &stamp = contextFrames_[frame % contextFrames_.size()];
    if (frame && stamp.frameId == frame && stamp.physicalHand == physicalHand) {
        pressed |= stamp.drainPressed;
        touched |= stamp.drainTouched;
    }
    // A menu can open reentrantly after BeginControllerFrame.
    pressed |= contextHands_[physicalHand - 1].drainPressed;
    touched |= contextHands_[physicalHand - 1].drainTouched;
}
HIGGS_HandStatus Broker::CopyInputContext(uint64_t owner, HIGGS_InputContextState &out)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    out = ContextStateLocked();
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::CopyInputFrameContext(uint64_t owner, uint64_t frame, HIGGS_InputContextFrame &out)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    const auto &stamp = contextFrames_[frame % contextFrames_.size()];
    if (!frame || stamp.frameId != frame) return HIGGS_HAND_NOT_FOUND;
    if (stamp.epoch != snapshot_.epoch) return HIGGS_HAND_STALE;
    out = stamp;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::ContextEvents(uint64_t owner, HIGGS_InputContextCallback cb, void *context, uint64_t &out)
{
    if (!cb) return HIGGS_HAND_INVALID_ARGUMENT;
    auto phase = NativePhase();
    const auto status = Add(owner, 6, nullptr, nullptr, nullptr, context, out);
    if (!status) { std::lock_guard lock(mutex_); RegistrationLocked(out)->inputContext = cb; }
    return status;
}
}
namespace {
template<class T, class F> HIGGS_HandStatus Copy(T *out, F f) noexcept {
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    if (out->structSize < sizeof(T)) { out->structSize = sizeof(T); return HIGGS_HAND_BUFFER_TOO_SMALL; }
    try { T value{}; const auto status = f(value); if (!status) *out = value; return status; }
    catch (...) { return HIGGS_HAND_INTERNAL_ERROR; }
}
HIGGS_HandStatus HIGGS_HAND_CALL Context(uint64_t owner, HIGGS_InputContextState *out) {
    return Copy(out, [&](auto &value) { return HiggsHands::Service().CopyInputContext(owner, value); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Frame(uint64_t owner, uint64_t frame, HIGGS_InputContextFrame *out) {
    return Copy(out, [&](auto &value) { return HiggsHands::Service().CopyInputFrameContext(owner, frame, value); });
}
HIGGS_HandStatus HIGGS_HAND_CALL Subscribe(uint64_t owner, HIGGS_InputContextCallback cb, void *context, uint64_t *out) {
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    try { uint64_t token = 0; const auto status = HiggsHands::Service().ContextEvents(owner, cb, context, token); if (!status) *out = token; return status; }
    catch (...) { return HIGGS_HAND_INTERNAL_ERROR; }
}
}
extern "C" HIGGS_HandStatus HIGGS_HAND_CALL HIGGS_QueryInputContextInterface(uint32_t version, uint32_t bytes, HIGGS_InputContextInterface *out) {
    if (version != HIGGS_INPUT_CONTEXT_ABI_VERSION) return HIGGS_HAND_UNSUPPORTED_VERSION;
    if (!out) return HIGGS_HAND_INVALID_ARGUMENT;
    if (bytes < sizeof(*out)) return HIGGS_HAND_BUFFER_TOO_SMALL;
    const HIGGS_InputContextInterface table{sizeof(table), HIGGS_INPUT_CONTEXT_ABI_VERSION, 7, Context, Frame, Subscribe};
    std::memcpy(out, &table, sizeof(table));
    return HIGGS_HAND_OK;
}
