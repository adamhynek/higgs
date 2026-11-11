#include "hand_broker.h"
#include "hand_observations.h"
#include "own_contact.h"
#include "native_regions.h"
#include <algorithm>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <tuple>

namespace HiggsHands
{
namespace
{
thread_local Broker *callbackBroker = nullptr;
thread_local uint64_t callbackOwner = 0, recognitionInput = 0;
thread_local uint32_t recognitionHand = 0;
thread_local uint64_t recognitionGeneration = 0;
thread_local const HIGGS_InputEvent *extendedRecognition = nullptr;
thread_local const HIGGS_InputCandidate *extendedSubmission = nullptr;
thread_local bool recognitionFreshDown = false;
thread_local uint64_t extendedRegistration = 0;
thread_local uint32_t callbackDepth = 0;
thread_local Broker *flushingBroker = nullptr;
thread_local uint64_t transferExecutor = 0;
struct DeferredWorldDrain
{
    Broker &broker;
    ~DeferredWorldDrain()
    {
        broker.ApplyDeferredWorld();
    }
};
struct CallbackScope
{
    Broker *oldBroker = callbackBroker;
    uint64_t oldOwner = callbackOwner, oldInput = recognitionInput;
    const HIGGS_InputEvent *oldExtended = extendedRecognition;
    uint64_t oldExtendedRegistration = extendedRegistration;
    bool oldFreshDown = recognitionFreshDown;
    CallbackScope(Broker *b, uint64_t owner, uint64_t input = 0)
    {
        callbackBroker = b;
        callbackOwner = owner;
        recognitionInput = input;
        extendedRecognition = nullptr;
        extendedRegistration = 0;
        recognitionFreshDown = false;
        ++callbackDepth;
    }
    ~CallbackScope()
    {
        --callbackDepth;
        callbackBroker = oldBroker;
        callbackOwner = oldOwner;
        recognitionInput = oldInput;
        extendedRecognition = oldExtended;
        extendedRegistration = oldExtendedRegistration;
        recognitionFreshDown = oldFreshDown;
    }
};
bool Terminal(uint32_t stage)
{
    return stage >= HIGGS_ACTION_COMPLETED;
}
bool Hand(uint32_t hand)
{
    return hand == HIGGS_HAND_LEFT || hand == HIGGS_HAND_RIGHT;
}
std::string Name(std::string value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        return {};
    value = value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
    for (auto &ch : value)
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    return value;
}
} // namespace
Clock Now()
{
    return static_cast<Clock>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());
}
Broker &Service()
{
    static Broker service;
    return service;
}
bool Broker::ConfigurePriorities(const std::string &settings)
{
    std::vector<std::pair<std::string, int32_t>> parsed;
    size_t start = 0;
    while (start < settings.size())
    {
        const auto end = settings.find(',', start);
        const auto entry = settings.substr(start, end == std::string::npos ? end : end - start);
        const auto equals = entry.find('=');
        if (equals == std::string::npos)
            return false;
        const auto name = Name(entry.substr(0, equals)), number = Name(entry.substr(equals + 1));
        if (name.empty() || name.size() > 85 || parsed.size() >= 64)
            return false;
        const auto slash = name.find('/');
        if (slash != std::string::npos) {
            const auto action = std::string_view(name).substr(slash+1);
            uint64_t id = 0;
            const auto actionEnd = action.data() + action.size();
            const auto conversion = std::from_chars(action.data(),actionEnd,id);
            if (!slash || slash>63 || !id || conversion.ec!=std::errc{} || conversion.ptr!=actionEnd ||
                std::to_string(id)!=action) return false;
        } else if (name.size()>63) return false;
        int32_t priority = 0;
        auto result = std::from_chars(number.data(), number.data() + number.size(), priority);
        if (result.ec != std::errc{} || result.ptr != number.data() + number.size())
            return false;
        for (const auto &prior : parsed)
            if (prior.first == name)
                return false;
        parsed.emplace_back(name, priority);
        if (end == std::string::npos)
            break;
        start = end + 1;
        if (start == settings.size())
            return false;
    }
    std::lock_guard lock(mutex_);
    priorities_ = std::move(parsed);
    return true;
}
int32_t Broker::PriorityLocked(const Client &client, bool &overridden) const
{
    const auto name = Name(client.info.name);
    for (const auto &priority : priorities_)
        if (priority.first == name)
        {
            overridden = true;
            return priority.second;
        }
    overridden = false;
    return client.info.userPriority;
}
int32_t Broker::ActionPriorityLocked(const Client &client, uint64_t action, bool &overridden) const
{
    const auto key = Name(client.info.name) + "/" + std::to_string(action);
    for (const auto &entry : priorities_)
        if (entry.first == key) { overridden = true; return entry.second; }
    return PriorityLocked(client, overridden);
}
bool Broker::EffectivePriority(uint64_t owner, int32_t &value)
{
    std::lock_guard lock(mutex_);
    Client legacy{};
    const Client *client = nullptr;
    if (!owner)
    {
        const char name[] = "HIGGS";
        std::memcpy(legacy.info.name, name, sizeof(name));
        client = &legacy;
    }
    else
        client = ClientLocked(owner);
    if (!client)
        return false;
    bool overridden = false;
    value = PriorityLocked(*client, overridden);
    return true;
}
uint64_t Broker::GrantCorrelationLocked(uint64_t grant) const
{
    if (!grant) return 0;
    uint64_t first = UINT64_MAX;
    for (const auto &action : actions_)
        if (action.request.grant == grant && action.request.requestId)
            first = (std::min)(first, action.request.requestId);
    return first == UINT64_MAX ? 0 : first | (UINT64_C(1) << 63);
}
void Broker::ObserveRecordLocked(HIGGS_HandRecord &h)
{
    h.grant = GrantCorrelationLocked(h.grant);
    h.owner = ClientLabelLocked(h.owner);
}
uint64_t Broker::ClientLabel(uint64_t id)
{
    std::lock_guard lock(mutex_);
    return ClientLabelLocked(id);
}
uint64_t Broker::ClientLabelLocked(uint64_t id)
{
    const auto owner = ClientLocked(id);
    return owner ? owner->label : 0;
}
uint64_t Broker::ClientForLabel(uint64_t label)
{
    std::lock_guard lock(mutex_);
    for(const auto &client:clients_)if(client.active && label && client.label==label)return client.id;
    return 0;
}
HIGGS_HandSnapshot Broker::ObservationLocked(HIGGS_HandSnapshot snapshot)
{
    for (auto &h : snapshot.hands) ObserveRecordLocked(h);
    return snapshot;
}
HIGGS_HandStatus Broker::ObservedSnapshot(uint64_t owner, HIGGS_HandSnapshot &out, Clock now)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    ExpireLocked(now);
    out = ObservationLocked(snapshot_);
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::ObservedDecision(uint64_t owner, uint64_t id, HIGGS_InputDecision &out)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    const auto &decision = decisions_[id % decisions_.size()];
    if (!id || decision.inputId != id) return HIGGS_HAND_NOT_FOUND;
    out = decision;
    HIGGS_HandRecord h{};
    h.owner = out.owner;
    h.grant = out.grant;
    ObserveRecordLocked(h);
    out.owner = h.owner;
    out.grant = h.grant;
    return HIGGS_HAND_OK;
}
void Broker::EndSequenceLocked(size_t hand, size_t control, uint64_t up, uint32_t reason)
{
    auto &active = activeSequences_[hand][control];
    auto &route = active.record;
    if (!route.pressInputId) return;
    route.phase = up ? HIGGS_SEQUENCE_RELEASED : HIGGS_SEQUENCE_CANCELLED;
    route.terminalInputId = up;
    route.reason = reason;
    pastSequences_[nextPastSequence_++ % pastSequences_.size()] = route;
    if (active.owner) {
        HIGGS_HandEvent event{};
        event.structSize = sizeof(event);
        event.type = up ? HIGGS_EVENT_INPUT_RELEASED : HIGGS_EVENT_INPUT_CANCELLED;
        event.eventId = nextEvent_++;
        event.physicalHand = route.physicalHand;
        event.epoch = route.epoch;
        event.observationId = snapshot_.observationId;
        event.observedAtNs = Now();
        event.result.structSize = sizeof(event.result);
        event.result.requestId = route.requestId;
        event.result.inputId = route.pressInputId;
        event.result.epoch = route.epoch;
        event.result.executor = route.executor;
        event.result.reason = reason;
        pendingEvents_.emplace_back(active.owner,event);
    }
    active = {};
}
void Broker::CancelSequencesLocked(size_t hand, uint32_t reason)
{
    for (size_t i = 0; i < kInputControlSlots; ++i) EndSequenceLocked(hand, i, 0, reason);
}
void Broker::CancelRoutesLocked(uint64_t executor, uint64_t owner, uint64_t recognizer, uint32_t reason)
{
    for (size_t h = 0; h < activeSequences_.size(); ++h)
        for (size_t i = 0; i < kInputControlSlots; ++i) {
            const auto &r = activeSequences_[h][i];
            if ((!executor || r.record.executor == executor) && (!owner || r.owner == owner) &&
                (!recognizer || r.recognizer == recognizer)) EndSequenceLocked(h,i,0,reason);
        }
}
void Broker::AcceptNativePress(uint32_t hand, uint32_t button)
{
    if (!Hand(hand) || (button != HIGGS_INPUT_GRIP && button != HIGGS_INPUT_TRIGGER)) return;
    std::lock_guard lock(mutex_);
    const size_t h = hand == HIGGS_HAND_LEFT ? 0 : 1;
    const size_t b = button == HIGGS_INPUT_GRIP ? 0 : 1;
    const auto id = nativeFallbackPress_[h][b];
    auto &decision = decisions_[id % decisions_.size()];
    auto &route = activeSequences_[h][b].record;
    if (!id || route.pressInputId || decision.inputId != id || decision.consumed ||
        !down_[h][b] || snapshot_.epoch != decision.epoch || !snapshot_.worldReady || contextMenus_ ||
        decisionContexts_[id % decisions_.size()] != contextGeneration_) return;
    decision.reason |= HIGGS_DECISION_NATIVE;
    route = {sizeof(route), HIGGS_SEQUENCE_HELD, hand, button, 1, 0, decision.epoch, id, 0, 0, 0, 0, 1};
}

HIGGS_HandStatus Broker::CopySequence(uint64_t owner, uint64_t press, HIGGS_InputSequence &out)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    if (!press) return HIGGS_HAND_INVALID_ARGUMENT;
    for (const auto &row : activeSequences_)
        for (const auto &r : row)
            if (r.record.pressInputId == press) { out = r.record; return HIGGS_HAND_OK; }
    for (const auto &r : pastSequences_)
        if (r.pressInputId == press) { out = r; return HIGGS_HAND_OK; }
    return HIGGS_HAND_NOT_FOUND;
}
HIGGS_HandStatus Broker::CheckSelectedAction(uint64_t owner, uint64_t id, uint64_t epoch, uint32_t hands,
                                            uint64_t &correlation, Clock now)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    const auto action = ActionLocked(id);
    if (!id || !hands || (hands & ~HIGGS_HAND_BOTH)) return HIGGS_HAND_INVALID_ARGUMENT;
    if (!action) return HIGGS_HAND_NOT_FOUND;
    if (owner != action->request.owner && owner != action->recognizerOwner) return HIGGS_HAND_WRONG_OWNER;
    if (epoch != snapshot_.epoch || epoch != action->request.epoch) return HIGGS_HAND_STALE;
    ExpireLocked(now);
    if (!action->selected || !action->grantActive || (action->request.physicalHands & hands) != hands ||
        !RoutedLocked(action->request.executor,action->recognizerOwner,hands) ||
        !GrantReadyLocked(action->request.owner,action->request.grant)) return HIGGS_HAND_CONFLICT;
    correlation = GrantCorrelationLocked(action->request.grant);
    return HIGGS_HAND_OK;
}
Broker::Client *Broker::ClientLocked(uint64_t id)
{
    for (auto &c : clients_)
        if (c.id == id && c.active)
            return &c;
    return nullptr;
}
std::string_view Broker::ClientKeyLocked(uint64_t id)
{
    const auto client = ClientLocked(id);
    return client ? std::string_view(client->info.name) : std::string_view{};
}
std::shared_ptr<Broker::Registration> Broker::RegistrationLocked(uint64_t id)
{
    for (auto &r : registrations_)
        if (r->id == id)
            return r;
    return {};
}
Broker::Action *Broker::ActionLocked(uint64_t id)
{
    for (auto &a : actions_)
        if (id && a.request.requestId == id)
            return &a;
    return nullptr;
}
Broker::Action *Broker::NewActionLocked()
{
    for (auto &a : actions_)
        if (!a.request.requestId)
        {
            a = {};
            return &a;
        }
    Action *oldest = nullptr;
    for (auto &a : actions_)
        if (!a.grantActive && !a.operationActive && !a.nativeStarted && !a.retained && Terminal(a.result.stage) &&
            (!oldest || a.request.requestId < oldest->request.requestId))
            oldest = &a;
    if (oldest)
        *oldest = {};
    return oldest;
}
HIGGS_HandStatus Broker::Register(const HIGGS_HandClientInfo &info, uint64_t &out)
{
    if (info.structSize < sizeof(info) || !info.name[0] || !std::memchr(info.name, 0, sizeof(info.name)))
        return HIGGS_HAND_INVALID_ARGUMENT;
    std::lock_guard lock(mutex_);
    const auto name = Name(info.name);
    if (name.empty() || name == "higgs" || name.find_first_of(",=/") != std::string::npos)
        return HIGGS_HAND_INVALID_ARGUMENT;
    size_t active = 0;
    for (const auto &c : clients_) {
        if (!c.active) continue;
        if (name == c.info.name) return HIGGS_HAND_CONFLICT;
        ++active;
    }
    if (active >= 64)
        return HIGGS_HAND_CAPACITY;
    Client c{};
    if(nextClientLabel_==UINT64_MAX || nextToken_ >= (UINT64_C(1)<<63))return HIGGS_HAND_CAPACITY;
    c.label=nextClientLabel_++;
    c.id = nextToken_++;
    c.info = info;
    std::memcpy(c.info.name, name.c_str(), name.size() + 1);
    auto it = std::find_if(clients_.begin(), clients_.end(), [](const Client &v) { return !v.active; });
    if (it == clients_.end())
        clients_.push_back(c);
    else
        *it = c;
    out = c.id;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::Unregister(uint64_t owner)
{
    std::lock_guard serial(dispatch_);
    std::unique_lock lock(mutex_);
    for (const auto &a : actions_)
        if (a.request.owner == owner && a.nativeStarted)
            return HIGGS_HAND_PENDING;
    auto c = ClientLocked(owner);
    bool tombstone = false;
    for (auto &r : registrations_)
        if (r->owner == owner)
            tombstone = true;
    if (!c && !tombstone)
        return HIGGS_HAND_INVALID_CLIENT;
    if (owner == drawBoundaryOwner_) {
        drawBoundaryOwner_ = drawBoundaryId_ = 0;
        for (auto &client : clients_) client.drawUntil = 0;
    }
    CancelRoutesLocked(0,owner,0,HIGGS_HAND_INVALID_CLIENT);
    CancelRoutesLocked(0,0,owner,HIGGS_HAND_INVALID_CLIENT);
    if (c)
        c->active = false;
    for (auto &r : registrations_)
        if (r->owner == owner)
            r->active = false;
    for (auto &a : actions_)
        if (a.request.owner == owner ||
            (a.recognition.stage == HIGGS_RECOGNITION_WAITING && a.recognizerOwner == owner))
        {
            a.grantActive = false;
            a.operationActive = false;
            if (!Terminal(a.result.stage))
                FinishLocked(a, HIGGS_ACTION_CANCELLED, HIGGS_HAND_INVALID_CLIENT);
        }
    RefreshLocked();
    if (callbackDepth)
        return HIGGS_HAND_PENDING;
    drained_.wait(lock, [&] {
        for (auto &r : registrations_)
            if (r->owner == owner && r->inFlight)
                return false;
        return true;
    });
    registrations_.erase(
        std::remove_if(registrations_.begin(), registrations_.end(), [&](const auto &r) { return r->owner == owner; }),
        registrations_.end());
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::Add(uint64_t owner, uint32_t kind, HIGGS_RecognizerCallback rec, HIGGS_ExecutorCallback exec,
                             HIGGS_HandEventCallback ev, void *ctx, uint64_t &out)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (registrations_.size() >= 256)
        return HIGGS_HAND_CAPACITY;
    auto r = std::make_shared<Registration>();
    r->id = nextToken_++;
    r->owner = owner;
    r->kind = kind;
    r->recognizer = rec;
    r->executor = exec;
    r->events = ev;
    r->context = ctx;
    registrations_.push_back(r);
    out = r->id;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::Recognizer(uint64_t o, HIGGS_RecognizerCallback cb, void *c, uint64_t &out)
{
    return cb ? Add(o, 1, cb, nullptr, nullptr, c, out) : HIGGS_HAND_INVALID_ARGUMENT;
}
HIGGS_HandStatus Broker::Executor(uint64_t o, HIGGS_ExecutorCallback cb, void *c, uint64_t &out)
{
    return cb ? Add(o, 2, nullptr, cb, nullptr, c, out) : HIGGS_HAND_INVALID_ARGUMENT;
}
HIGGS_HandStatus Broker::Events(uint64_t o, HIGGS_HandEventCallback cb, void *c, uint64_t &out)
{
    return cb ? Add(o, 3, nullptr, nullptr, cb, c, out) : HIGGS_HAND_INVALID_ARGUMENT;
}
HIGGS_HandStatus Broker::Remove(uint64_t owner, uint64_t token)
{
    std::lock_guard serial(dispatch_);
    std::unique_lock lock(mutex_);
    auto r = RegistrationLocked(token);
    if (!r)
        return HIGGS_HAND_NOT_FOUND;
    if (r->owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    for (const auto &action : actions_)
        if (action.nativeStarted && action.request.executor == token)
            return HIGGS_HAND_PENDING;
    r->active = false;
    if (r->kind == 2) CancelRoutesLocked(token,owner,0,HIGGS_HAND_NOT_FOUND);
    for (auto &a : actions_)
        if ((a.request.executor == token ||
             (a.recognition.stage == HIGGS_RECOGNITION_WAITING && a.inputRegistration == token)) &&
            !Terminal(a.result.stage))
        {
            a.grantActive = false;
            FinishLocked(a, HIGGS_ACTION_CANCELLED, HIGGS_HAND_NOT_FOUND);
        }
    RefreshLocked();
    if (callbackDepth)
        return HIGGS_HAND_PENDING;
    drained_.wait(lock, [&] { return r->inFlight == 0; });
    registrations_.erase(std::remove(registrations_.begin(), registrations_.end(), r), registrations_.end());
    return HIGGS_HAND_OK;
}
std::vector<std::shared_ptr<Broker::Registration>> Broker::List(uint32_t kind)
{
    std::lock_guard lock(mutex_);
    std::vector<std::shared_ptr<Registration>> result;
    for (auto &r : registrations_)
        if (r->active && r->kind == kind)
            result.push_back(r);
    return result;
}
bool Broker::Enter(const std::shared_ptr<Registration> &r)
{
    std::lock_guard lock(mutex_);
    if (!r->active || !ClientLocked(r->owner))
        return false;
    ++r->inFlight;
    return true;
}
bool Broker::InTransferCallback(uint64_t owner, uint64_t executor)
{
    return callbackBroker == this && callbackOwner == owner && transferExecutor == executor;
}
bool Broker::InvokeTransfer(uint64_t owner, uint64_t token, const HIGGS_TransferExecution &execution)
{
    auto phase = NativePhase();
    DeferredWorldDrain deferredWorldDrain{*this};
    std::shared_ptr<Registration> r;
    {
        std::lock_guard lock(mutex_);
        r = RegistrationLocked(token);
        if (!r || r->kind != 4 || r->owner != owner || !r->transfer)
            return false;
        if (auto a = ActionLocked(execution.requestId)) {
            a->nativeStarted = true;
            a->dispatched = true;
        }
    }
    if (!Enter(r))
    {
        std::lock_guard lock(mutex_);
        if (auto a = ActionLocked(execution.requestId))
            a->nativeStarted = false;
        return false;
    }
    bool success = true;
    {
        CallbackScope scope(this, owner);
        const auto previous = transferExecutor;
        transferExecutor = token;
        try
        {
            r->transfer(&execution, r->context);
        }
        catch (...)
        {
            success = false;
        }
        transferExecutor = previous;
    }
    Leave(r);
    uint64_t completedGrant = 0;
    {
        std::lock_guard lock(mutex_);
        if (auto a = ActionLocked(execution.requestId))
        {
            a->nativeStarted = false;
            if (a->domainOwned && a->grantActive && a->result.stage >= HIGGS_ACTION_COMPLETED)
                completedGrant = a->request.grant;
        }
        ExpireLocked(Now());
    }
    if (completedGrant)
        Release(owner, completedGrant);
    ApplyDeferredWorld();
    return success;
}
void Broker::Leave(const std::shared_ptr<Registration> &r)
{
    std::lock_guard lock(mutex_);
    --r->inFlight;
    drained_.notify_all();
}
void Broker::InvokeSessionExecutor(const std::shared_ptr<Registration> &r, const HIGGS_ActionRequest &request)
{
    if (!r || !Enter(r))
        return;
    {
        CallbackScope scope(this, r->owner);
        try
        {
            r->executor(&request, r->context);
        }
        catch (...)
        {
            CancelNativeRequest(r->owner, request.requestId);
        }
    }
    Leave(r);
}
void Broker::QueueResultLocked(const Action &a)
{
    HIGGS_HandEvent e{};
    e.structSize = sizeof(e);
    e.type = HIGGS_EVENT_ACTION_RESULT;
    e.eventId = nextEvent_++;
    e.epoch = a.request.epoch;
    e.observationId = snapshot_.observationId;
    e.result = a.result;
    pendingEvents_.emplace_back(0,e);
}
void Broker::FinishLocked(Action &a, uint32_t stage, uint32_t reason)
{
    a.result.structSize = sizeof(a.result);
    a.result.requestId = a.request.requestId;
    a.result.executor = a.request.executor;
    a.result.epoch = a.request.epoch;
    a.result.inputId = a.request.inputId;
    a.result.stage = stage;
    a.result.reason = reason;
    QueueResultLocked(a);
}
void Broker::QueueEventLocked(uint32_t type, uint32_t hand, uint32_t ref, Clock now, const HIGGS_HandRecord *state)
{
    HIGGS_HandEvent e{};
    e.structSize = sizeof(e);
    e.type = type;
    e.physicalHand = hand;
    e.itemRefId = ref;
    e.eventId = nextEvent_++;
    e.epoch = snapshot_.epoch;
    e.observationId = snapshot_.observationId;
    e.observedAtNs = now;
    if (state)
    {
        e.hand = *state;
        ObserveRecordLocked(e.hand);
        e.poseObservationId = state->poseObservationId;
    }
    if (e.type == HIGGS_EVENT_RELEASED) ObserveRelease(e, ObservationLocked(snapshot_));
    pendingEvents_.emplace_back(0,e);
}
void Broker::FlushEvents()
{
    if (flushingBroker == this)
        return;
    std::lock_guard dispatch(dispatch_);
    struct Restore
    {
        Broker *previous;
        ~Restore()
        {
            flushingBroker = previous;
        }
    } restore{flushingBroker};
    flushingBroker = this;
    // Nested receipts append to the next batch. Registrations created by callbacks
    // never join the event currently in progress.
    for (size_t rounds = 0; rounds < 256; ++rounds)
    {
        std::vector<std::pair<uint64_t,HIGGS_HandEvent>> events;
        std::vector<std::pair<HIGGS_InputContextState, HIGGS_InputContextState>> contexts;
        {
            std::lock_guard lock(mutex_);
            events.swap(pendingEvents_);
            contexts.swap(pendingContexts_);
        }
        if (events.empty() && contexts.empty())
            break;
        for (auto &[recipient,e] : events)
        {
            auto list = List(3);
            for (auto &r : list)
                if ((!recipient || r->owner == recipient) && Enter(r))
                {
                    CallbackScope scope(this, r->owner);
                    try
                    {
                        r->events(&e, r->context);
                    }
                    catch (...)
                    {
                    }
                    Leave(r);
                }
        }
        for (auto &e : contexts)
            for (auto &r : List(6))
                if (Enter(r)) {
                    CallbackScope scope(this, r->owner);
                    try { r->inputContext(&e.first, &e.second, r->context); } catch (...) {}
                    Leave(r);
                }
    }
}
void Broker::ExpireLocked(Clock now)
{
    for (auto &a : actions_)
        if (a.request.requestId && a.expires && now >= a.expires && !a.nativeStarted)
        {
            a.grantActive = false;
            a.operationActive = false;
            if (!Terminal(a.result.stage))
                FinishLocked(a, HIGGS_ACTION_CANCELLED, HIGGS_HAND_STALE);
        }
    RefreshLocked();
}
void Broker::RefreshLocked()
{
    for (auto &h : snapshot_.hands)
    {
        h.unavailableReasons = nativeUnavailableReasons_[h.physicalHand == HIGGS_HAND_LEFT ? 0 : 1];
        for (const auto &owned : actions_)
            if (owned.request.epoch == snapshot_.epoch &&
                ((owned.ownedActuator && (owned.request.physicalHands & h.physicalHand)) ||
                 (owned.dependencyGrant && owned.retained && (owned.dependencyHands & h.physicalHand))))
            {
                bool authority = false;
                for (const auto &grant : actions_)
                    if (grant.grantActive && grant.request.grant ==
                            (owned.dependencyGrant ? owned.dependencyGrant : owned.request.grant) &&
                        grant.request.owner == owned.request.owner)
                        authority = true;
                if (!authority)
                    h.unavailableReasons |= HIGGS_HAND_REASON_EXISTING_USE;
            }
        h.grant = 0;
        h.owner = 0;
        const bool known =
            (h.observationMask &
             (HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN | HIGGS_OBS_PALM_VALID | HIGGS_OBS_TRACKING_VALID)) ==
            (HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN | HIGGS_OBS_PALM_VALID | HIGGS_OBS_TRACKING_VALID);
        h.availability = !snapshot_.worldReady || !known
                             ? HIGGS_AVAILABILITY_UNKNOWN
                             : (h.unavailableReasons ? HIGGS_AVAILABILITY_UNAVAILABLE : HIGGS_AVAILABILITY_READY);
        if (h.availability == HIGGS_AVAILABILITY_READY &&
            (h.heldRefId || (h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT)))
            h.availability = HIGGS_AVAILABILITY_RESERVED;
        for (auto &a : actions_)
            if (a.grantActive && (a.request.physicalHands & h.physicalHand))
            {
                h.grant = a.request.grant;
                h.owner = a.request.owner;
                if (h.availability == HIGGS_AVAILABILITY_READY || h.availability == HIGGS_AVAILABILITY_RESERVED)
                    h.availability = HIGGS_AVAILABILITY_RESERVED;
                break;
            }
    }
}
bool Broker::EligibleLocked(uint32_t hands) const
{
    if (worldChangePending_ || contextMenus_)
        return false;
    if (!snapshot_.worldReady || !hands || (hands & ~3u))
        return false;
    for (const auto &h : snapshot_.hands)
        if (h.physicalHand & hands)
        {
            if (h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT)
                return false;
            if (h.availability == HIGGS_AVAILABILITY_UNKNOWN || h.availability == HIGGS_AVAILABILITY_UNAVAILABLE ||
                h.grant)
                return false;
        }
    return true;
}
bool Broker::GrantReadyLocked(uint64_t owner, uint64_t grant) const
{
    for (const auto &action : actions_)
        if (action.grantActive && action.request.owner == owner && action.request.grant == grant &&
            (!action.request.inputId || action.sessionKind || action.result.stage >= HIGGS_ACTION_COMMITTED ||
             action.inputContextGeneration == contextGeneration_) &&
            action.recognition.stage != HIGGS_RECOGNITION_WAITING)
            return true;
    return false;
}
bool Broker::RoutedLocked(uint64_t executor, uint64_t recognizer, uint32_t hands)
{
    auto r = RegistrationLocked(executor);
    if (!r || !r->active || !ClientLocked(r->owner) || !ClientLocked(recognizer))
        return false;
    if (r->owner == recognizer)
        return true;
    for (const auto &route : r->routes)
        if (route.first == recognizer && (hands & ~route.second) == 0)
            return true;
    return false;
}
HIGGS_HandStatus Broker::Authorize(uint64_t owner, uint64_t executor, uint64_t recognizer, uint32_t hands)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner) || !ClientLocked(recognizer))
        return HIGGS_HAND_INVALID_CLIENT;
    auto r = RegistrationLocked(executor);
    if (!r || !r->active || r->kind != 2)
        return HIGGS_HAND_NOT_FOUND;
    if (r->owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    if (!hands || (hands & ~3u))
        return HIGGS_HAND_INVALID_ARGUMENT;
    for (auto &route : r->routes)
        if (route.first == recognizer)
        {
            // Revoke explicitly before narrowing. Revoke cancels pending routed uses.
            if (route.second & ~hands) return HIGGS_HAND_CONFLICT;
            route.second = hands;
            return HIGGS_HAND_OK;
        }
    if (r->routes.size() >= 64)
        return HIGGS_HAND_CAPACITY;
    r->routes.emplace_back(recognizer, hands);
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::Revoke(uint64_t owner, uint64_t executor, uint64_t recognizer)
{
    std::lock_guard serial(dispatch_);
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto r = RegistrationLocked(executor);
    if (!r)
        return HIGGS_HAND_NOT_FOUND;
    if (r->owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    for (const auto &action : actions_)
        if (action.nativeStarted && action.request.executor == executor && action.recognizerOwner == recognizer)
            return HIGGS_HAND_PENDING;
    CancelRoutesLocked(executor,owner,recognizer,HIGGS_HAND_WRONG_OWNER);
    r->routes.erase(std::remove_if(r->routes.begin(), r->routes.end(),
                                   [&](const auto &route) { return route.first == recognizer; }),
                    r->routes.end());
    for (auto &a : actions_)
        if (a.request.executor == executor && a.recognizerOwner == recognizer &&
            a.result.stage < HIGGS_ACTION_COMMITTED)
        {
            a.grantActive = false;
            FinishLocked(a, HIGGS_ACTION_CANCELLED, HIGGS_HAND_WRONG_OWNER);
        }
    RefreshLocked();
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::CanRoute(uint64_t recognizer, uint64_t executor, uint32_t hands)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(recognizer))
        return HIGGS_HAND_INVALID_CLIENT;
    if (!hands || (hands & ~3u))
        return HIGGS_HAND_INVALID_ARGUMENT;
    return RoutedLocked(executor, recognizer, hands) ? HIGGS_HAND_OK : HIGGS_HAND_WRONG_OWNER;
}
HIGGS_HandStatus Broker::CancelRecognized(uint64_t recognizer, uint64_t request, uint64_t epoch, uint32_t reason)
{
    std::lock_guard serial(dispatch_);
    {
        std::lock_guard lock(mutex_);
        if (!ClientLocked(recognizer))
            return HIGGS_HAND_INVALID_CLIENT;
        auto a = ActionLocked(request);
        if (!a)
            return HIGGS_HAND_NOT_FOUND;
        if (a->recognizerOwner != recognizer)
            return HIGGS_HAND_WRONG_OWNER;
        if (a->nativeStarted)
            return HIGGS_HAND_PENDING;
        if (a->request.epoch != epoch || epoch != snapshot_.epoch)
            return HIGGS_HAND_STALE;
        if (a->result.stage >= HIGGS_ACTION_COMMITTED)
            return HIGGS_HAND_CONFLICT;
        a->grantActive = false;
        FinishLocked(*a, HIGGS_ACTION_CANCELLED, reason);
        RefreshLocked();
    }
    FlushEvents();
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::Copy(uint64_t owner, HIGGS_HandSnapshot &out, Clock now)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    ExpireLocked(now);
    out = snapshot_;
    out.structSize = sizeof(out);
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::Submit(uint64_t owner, const HIGGS_ActionCandidate &c, uint64_t &out, Clock now, uint64_t stableTarget)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (contextMenus_)
        return HIGGS_HAND_NOT_READY;
    if (callbackBroker != this || callbackOwner != owner || !recognitionInput || recognitionInput != c.inputId)
        return HIGGS_HAND_WRONG_PHASE;
    if (c.structSize < sizeof(c) || c.reserved || !c.grantDurationMs || c.grantDurationMs > 60000 || !c.physicalHands ||
        (c.physicalHands & ~3u))
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto exec = RegistrationLocked(c.executor);
    if (!exec || !exec->active || exec->kind != 2)
        return HIGGS_HAND_NOT_FOUND;
    if (!RoutedLocked(c.executor, owner, c.physicalHands))
        return HIGGS_HAND_WRONG_OWNER;
    if (c.epoch != snapshot_.epoch)
        return HIGGS_HAND_STALE;
    const auto &decision = decisions_[c.inputId % decisions_.size()];
    if (decision.inputId != c.inputId)
        return HIGGS_HAND_STALE;
    if (decisionContexts_[c.inputId % decisionContexts_.size()] != contextGeneration_)
        return HIGGS_HAND_STALE;
    // A consumed Down owns its matching Up even if the executor already ended
    // the hand grant. Up observers may inspect; they cannot consume it again.
    const auto parentGrant = extendedSubmission ? extendedSubmission->existingGrant : 0;
    if (decision.consumed && !(parentGrant && recognitionFreshDown))
        return HIGGS_HAND_CONFLICT;
    // The generation belongs to the triggering physical hand; both-hand candidates
    // still validate both resources against the current coherent snapshot below.
    if (c.handGeneration != recognitionGeneration || !(c.physicalHands & recognitionHand))
        return HIGGS_HAND_STALE;
    ExpireLocked(now);
    if (!InputEligibleLocked(c.physicalHands, exec->owner, parentGrant))
        return HIGGS_HAND_CONFLICT;
    if (nextToken_ == (std::numeric_limits<uint64_t>::max)())
        return HIGGS_HAND_CAPACITY;
    auto a = NewActionLocked();
    if (!a)
        return HIGGS_HAND_CAPACITY;
    a->candidate = c;
    a->stableTarget = stableTarget;
    a->recognizerOwner = owner;
    a->inputRegistration = extendedRegistration;
    a->inputContextGeneration = contextGeneration_;
    a->request = {};
    a->request.structSize = sizeof(a->request);
    a->request.requestId = nextToken_++;
    a->request.owner = exec->owner;
    a->request.executor = c.executor;
    a->request.epoch = c.epoch;
    a->request.inputId = c.inputId;
    a->request.actionId = c.actionId;
    a->request.contextId = c.contextId;
    a->request.contextGeneration = c.contextGeneration;
    a->request.physicalHands = c.physicalHands;
    a->expires = now + static_cast<Clock>(c.grantDurationMs) * 1000000;
    if (extendedSubmission)
    {
        a->inputSpec = *extendedSubmission;
        a->recognition.structSize = sizeof(a->recognition);
        a->recognition.requestId = a->request.requestId;
        a->recognition.epoch = c.epoch;
        a->recognition.inputId = c.inputId;
        a->recognition.gesture = extendedSubmission->gesture;
        a->recognition.initialFrame = extendedRecognition->frame;
        a->inputControl = extendedRecognition->controlId;
        a->inputTouch = (extendedRecognition->flags & HIGGS_INPUT_EFFECTIVE_SEMANTIC) &&
                        (extendedRecognition->frame.flags & HIGGS_INPUT_GRIP_USES_TOUCH) &&
                        a->inputControl == HIGGS_CONTROL_GRIP;
        if (parentGrant)
            for (const auto &parent : actions_)
                if (parent.grantActive && parent.request.grant == parentGrant && parent.expires &&
                    parent.expires < a->expires)
                    a->expires = parent.expires;
        for (unsigned h = 0; h < 2; ++h)
            a->inputHandGenerations[h] = extendedRecognition->snapshot.hands[h].generation;
        if (extendedSubmission->gesture)
        {
            a->inputExecutionExpires = a->expires;
            a->recognition.deadlineAtNs =
                (std::min)(a->expires, now + static_cast<uint64_t>(extendedSubmission->deadlineMs) * 1000000);
            a->expires = a->recognition.deadlineAtNs;
        }
    }
    out = a->request.requestId;
    return HIGGS_HAND_OK;
}
bool Broker::InputEligibleLocked(uint32_t hands, uint64_t owner, uint64_t parentGrant, bool maintainingRecognition) const
{
    if (contextMenus_)
        return false;
    if (!parentGrant)
        return EligibleLocked(hands);
    // A waiting recognition may maintain its own reservation but cannot mint
    // new work. Every admission under a parent needs current mutation authority.
    if (!maintainingRecognition && !GrantReadyLocked(owner, parentGrant))
        return false;
    if (worldChangePending_ || !snapshot_.worldReady || !hands || (hands & ~3u))
        return false;
    bool parent = false;
    for (const auto &a : actions_)
        if (a.grantActive && a.request.grant == parentGrant && a.request.owner == owner &&
            a.request.epoch == snapshot_.epoch && (hands & ~a.request.physicalHands) == 0)
            parent = true;
    if (!parent)
        return false;
    for (const auto &h : snapshot_.hands)
        if (h.physicalHand & hands)
            if (h.grant != parentGrant || h.owner != owner || h.availability != HIGGS_AVAILABILITY_RESERVED ||
                h.unavailableReasons)
                return false;
    return true;
}
HIGGS_HandStatus Broker::InputRecognizer(uint64_t owner, const HIGGS_InputRecognizerSpec &spec, uint64_t &out)
{
    if (spec.structSize != sizeof(spec) || !spec.physicalHands || (spec.physicalHands & ~3u) || !spec.controlMask ||
        (spec.controlMask & ~KnownControlMask()) || spec.eventMask > 63 || spec.reserved || !spec.callback)
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto registration = std::make_shared<Registration>();
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (registrations_.size() >= 256 || nextToken_ == (std::numeric_limits<uint64_t>::max)())
        return HIGGS_HAND_CAPACITY;
    registration->id = nextToken_++;
    registration->owner = owner;
    registration->kind = 5;
    registration->input = spec;
    registrations_.push_back(registration);
    out = registration->id;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::SubmitInputCandidate(uint64_t owner, const HIGGS_InputCandidate &candidate, uint64_t &out,
                                              Clock now)
{
    if (!extendedRecognition || !(extendedRecognition->flags & HIGGS_INPUT_ACTIONABLE))
        return HIGGS_HAND_WRONG_PHASE;
    if (extendedRecognition->frame.flags & HIGGS_INPUT_BASELINE)
        return HIGGS_HAND_NOT_READY;
    const auto validation = ValidateInputCandidate(candidate);
    if (validation)
        return validation;
    HIGGS_ControllerFrame latest{};
    if (!InputFrames().Latest(extendedRecognition->physicalHand, latest) ||
        latest.frameId != extendedRecognition->frame.frameId)
        return HIGGS_HAND_STALE;
    if ((candidate.gesture || candidate.existingGrant) && !recognitionFreshDown)
        return HIGGS_HAND_WRONG_PHASE;
    if (candidate.existingGrant)
    {
        std::lock_guard lock(mutex_);
        for (const auto &a : actions_)
            if (a.grantActive && a.request.grant == candidate.existingGrant &&
                a.recognition.stage == HIGGS_RECOGNITION_WAITING && a.inputControl == extendedRecognition->controlId)
                return HIGGS_HAND_CONFLICT;
    }
    const auto *previous = extendedSubmission;
    extendedSubmission = &candidate;
    HIGGS_HandStatus status;
    try
    {
        status = Submit(owner, candidate.candidate, out, now);
    }
    catch (...)
    {
        extendedSubmission = previous;
        throw;
    }
    extendedSubmission = previous;
    return status;
}
HIGGS_HandStatus Broker::CopyInputRecognition(uint64_t owner, uint64_t request, HIGGS_InputRecognition &out)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    const auto now = Now();
    ExpireLocked(now);
    const auto *a = ActionLocked(request);
    if (!a || !a->inputSpec.structSize)
        return HIGGS_HAND_NOT_FOUND;
    if (a->request.owner != owner && a->recognizerOwner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    auto value = a->recognition;
    if (!a->selected && Terminal(a->result.stage))
        value.stage = HIGGS_RECOGNITION_CANCELLED;
    else if (value.stage == HIGGS_RECOGNITION_WAITING && !a->grantActive)
        value.stage = now >= value.deadlineAtNs ? HIGGS_RECOGNITION_EXPIRED : HIGGS_RECOGNITION_CANCELLED;
    out = value;
    return HIGGS_HAND_OK;
}
void Broker::NotifyInputObservations(const HIGGS_InputEvent *events, uint32_t count)
{
    if (!events || count > 96)
        return;
    std::lock_guard serial(dispatch_);
    HIGGS_HandSnapshot snapshot;
    { std::lock_guard lock(mutex_); snapshot = ObservationLocked(snapshot_); }
    auto registrations = List(5);
    for (unsigned i = 0; i < count; ++i)
    {
        auto event = events[i];
        if (event.frame.epoch != snapshot.epoch || event.controlId >= 64 || event.kind < 1 || event.kind > 6)
            continue;
        event.snapshot = snapshot;
        for (auto &registration : registrations)
            if ((registration->input.physicalHands & event.physicalHand) &&
                (registration->input.controlMask & (UINT64_C(1) << event.controlId)) &&
                (!registration->input.eventMask || (registration->input.eventMask & (1u << (event.kind - 1)))) &&
                Enter(registration))
            {
                CallbackScope scope(this, registration->owner);
                try
                {
                    registration->input.callback(&event, registration->input.context);
                }
                catch (...)
                {
                }
                Leave(registration);
            }
    }
}
void Broker::PrimeInputFrame(const HIGGS_ControllerFrame &frame)
{
    if (!Hand(frame.physicalHand))
        return;
    std::lock_guard serial(dispatch_);
    std::lock_guard lock(mutex_);
    if (frame.epoch != snapshot_.epoch)
        return;
    const auto mask =
        ActionControlMask() & ~((UINT64_C(1) << HIGGS_CONTROL_GRIP) | (UINT64_C(1) << HIGGS_CONTROL_TRIGGER));
    for (unsigned id = 0; id < 64; ++id)
        if (mask & (UINT64_C(1) << id))
        {
            down_[frame.physicalHand - 1][2 + id] = (frame.pressed & (UINT64_C(1) << id)) != 0;
            consumed_[frame.physicalHand - 1][2 + id] = down_[frame.physicalHand - 1][2 + id];
        }
}
uint64_t Broker::InputSuppressionMask(uint32_t hand)
{
    if (!Hand(hand))
        return 0;
    std::lock_guard lock(mutex_);
    ExpireLocked(Now());
    const auto index = hand - 1;
    if (!contextMenus_ && snapshot_.hands[index].grant)
        return ActionControlMask();
    uint64_t mask = 0;
    for (unsigned id = 0; id < 64; ++id)
        if (!contextMenus_ && consumed_[index][2 + id])
            mask |= UINT64_C(1) << id;
    if (!contextMenus_ && consumed_[index][0])
        mask |= UINT64_C(1) << HIGGS_CONTROL_GRIP;
    if (!contextMenus_ && consumed_[index][1])
        mask |= UINT64_C(1) << HIGGS_CONTROL_TRIGGER;
    return mask & ActionControlMask();
}
void Broker::AdvanceInputRecognitions(const HIGGS_ControllerFrame &frame)
{
    std::lock_guard serial(dispatch_);
    std::array<HIGGS_ActionRequest, 256> work{};
    size_t count = 0;
    {
        std::lock_guard lock(mutex_);
        ExpireLocked(frame.observedAtNs);
        if (frame.epoch != snapshot_.epoch || contextMenus_)
            return;
        for (auto &a : actions_)
            if (a.recognition.stage == HIGGS_RECOGNITION_WAITING &&
                a.recognition.initialFrame.physicalHand == frame.physicalHand)
            {
                const auto cancel = [&](uint32_t stage, uint32_t reason) {
                    a.recognition.stage = stage;
                    a.grantActive = false;
                    FinishLocked(a, HIGGS_ACTION_CANCELLED, reason);
                };
                if (!a.grantActive || frame.observedAtNs >= a.recognition.deadlineAtNs)
                {
                    a.recognition.stage = frame.observedAtNs >= a.recognition.deadlineAtNs
                                              ? HIGGS_RECOGNITION_EXPIRED
                                              : HIGGS_RECOGNITION_CANCELLED;
                    continue;
                }
                const auto recognizer = RegistrationLocked(a.inputRegistration);
                bool valid = a.inputContextGeneration == contextGeneration_ && frame.status == HIGGS_INPUT_FRAME_READY && ClientLocked(a.recognizerOwner) && recognizer &&
                             recognizer->active &&
                             InputEligibleLocked(a.request.physicalHands, a.request.owner, a.request.grant, true);
                for (unsigned h = 0; h < 2; ++h)
                    if (a.request.physicalHands & (h + 1))
                        valid = valid && snapshot_.hands[h].generation == a.inputHandGenerations[h];
                if (a.inputControl == HIGGS_CONTROL_GRIP)
                    valid = valid && bool(frame.flags & HIGGS_INPUT_GRIP_USES_TOUCH) == a.inputTouch;
                if (!valid)
                {
                    cancel(HIGGS_RECOGNITION_CANCELLED, HIGGS_HAND_STALE);
                    continue;
                }
                const auto bit = UINT64_C(1) << a.inputControl;
                const bool pressed = ((a.inputTouch ? frame.touched : frame.pressed) & bit) != 0;
                bool match = false;
                if (pressed != a.inputPressed)
                {
                    a.inputPressed = pressed;
                    if (pressed)
                    {
                        if (a.inputSpec.gesture == HIGGS_GESTURE_MULTI_TAP && a.recognition.matchedTaps &&
                            frame.observedAtNs - a.inputLastReleaseAt > uint64_t(a.inputSpec.tapGapMs) * 1000000)
                        {
                            cancel(HIGGS_RECOGNITION_CANCELLED, HIGGS_HAND_STALE);
                            continue;
                        }
                        a.inputDownAt = frame.observedAtNs;
                        a.expires = a.recognition.deadlineAtNs;
                    }
                    else
                    {
                        a.inputLastReleaseAt = frame.observedAtNs;
                        if (a.inputSpec.gesture == HIGGS_GESTURE_HOLD)
                        {
                            cancel(HIGGS_RECOGNITION_CANCELLED, HIGGS_HAND_CONFLICT);
                            continue;
                        }
                        if (frame.observedAtNs - a.inputDownAt > uint64_t(a.inputSpec.maxTapMs) * 1000000)
                        {
                            cancel(HIGGS_RECOGNITION_CANCELLED, HIGGS_HAND_CONFLICT);
                            continue;
                        }
                        ++a.recognition.matchedTaps;
                        match = a.recognition.matchedTaps == a.inputSpec.tapCount;
                        if (!match)
                            a.expires = (std::min)(a.recognition.deadlineAtNs,
                                                   frame.observedAtNs + uint64_t(a.inputSpec.tapGapMs) * 1000000);
                    }
                }
                if (a.inputSpec.gesture == HIGGS_GESTURE_HOLD && pressed &&
                    frame.observedAtNs - a.inputDownAt >= uint64_t(a.inputSpec.holdMs) * 1000000)
                    match = true;
                if (a.inputSpec.gesture == HIGGS_GESTURE_MULTI_TAP && !pressed && a.recognition.matchedTaps &&
                    frame.observedAtNs - a.inputLastReleaseAt > uint64_t(a.inputSpec.tapGapMs) * 1000000)
                {
                    cancel(HIGGS_RECOGNITION_CANCELLED, HIGGS_HAND_STALE);
                    continue;
                }
                if (match)
                {
                    a.recognition.stage = HIGGS_RECOGNITION_MATCHED;
                    a.recognition.recognizedAtNs = frame.observedAtNs;
                    a.recognition.recognizedFrame = frame;
                    a.expires = a.inputExecutionExpires;
                    work[count++] = a.request;
                }
            }
        RefreshLocked();
    }
    for (size_t i = 0; i < count; ++i)
    {
        std::shared_ptr<Registration> executor;
        {
            std::lock_guard lock(mutex_);
            const auto *action = ActionLocked(work[i].requestId);
            if (!action || !action->grantActive || contextMenus_ || action->inputContextGeneration != contextGeneration_ || action->recognition.stage != HIGGS_RECOGNITION_MATCHED)
                continue;
            ActionLocked(work[i].requestId)->dispatched = true;
            executor = RegistrationLocked(action->request.executor);
        }
        if (executor && Enter(executor))
        {
            CallbackScope scope(this, executor->owner);
            try
            {
                executor->executor(&work[i], executor->context);
            }
            catch (...)
            {
                std::lock_guard lock(mutex_);
                if (auto *action = ActionLocked(work[i].requestId))
                {
                    action->grantActive = false;
                    FinishLocked(*action, HIGGS_ACTION_REJECTED, HIGGS_HAND_INTERNAL_ERROR);
                    RefreshLocked();
                }
            }
            Leave(executor);
        }
    }
    FlushEvents();
}
bool Broker::Input(uint32_t hand, uint32_t button, bool down, Clock now)
{
    const auto control = InputControlId(button), slot = InputControlSlot(button);
    if (!Hand(hand) || slot >= kInputControlSlots || control >= 64 || !(ActionControlMask() & (UINT64_C(1) << control)))
        return false;
    std::lock_guard dispatch(dispatch_);
    const size_t hi = hand == HIGGS_HAND_LEFT ? 0 : 1, bi = slot;
    HIGGS_HandInput input{};
    bool wasConsumed = false;
    bool freshDown = false;
    uint64_t contextGeneration = 0;
    {
        std::lock_guard lock(mutex_);
        ExpireLocked(now);
        if (!GameplayInputLocked(hand, button)) {
            down_[hi][bi] = down;
            consumed_[hi][bi] = false;
            const auto &contextHand = contextHands_[hi];
            const auto &stamp = contextFrames_[contextHand.frame % contextFrames_.size()];
            const uint64_t mask = control == HIGGS_CONTROL_GRIP && contextHand.touchGrip ?
                (stamp.frameId == contextHand.frame ? stamp.drainTouched : contextHand.drainTouched) :
                (stamp.frameId == contextHand.frame ? stamp.drainPressed : contextHand.drainPressed);
            return (mask & (UINT64_C(1) << control)) != 0;
        }
        contextGeneration = contextGeneration_;
        if (down_[hi][bi] == down)
        {
            if (down && snapshot_.hands[hi].grant)
                consumed_[hi][bi] = true;
            return consumed_[hi][bi] || snapshot_.hands[hi].grant != 0;
        }
        if (nextInput_ == (std::numeric_limits<uint64_t>::max)())
        {
            down_[hi][bi] = down;
            consumed_[hi][bi] = down;
            return true;
        }
        freshDown = down && !down_[hi][bi];
        down_[hi][bi] = down;
        wasConsumed = consumed_[hi][bi] || (down && snapshot_.hands[hi].grant != 0);
        input.structSize = sizeof(input);
        input.physicalHand = hand;
        input.button = button;
        input.edge = down ? HIGGS_INPUT_DOWN : HIGGS_INPUT_UP;
        input.inputId = nextInput_++;
        input.epoch = snapshot_.epoch;
        input.handGeneration = snapshot_.hands[hi].generation;
        input.poseObservationId = snapshot_.hands[hi].poseObservationId;
        input.observedAtNs = now;
        input.snapshot = ObservationLocked(snapshot_);
        auto &d = decisions_[input.inputId % decisions_.size()];
        d = {};
        d.structSize = sizeof(d);
        d.inputId = input.inputId;
        d.epoch = input.epoch;
        d.consumed = wasConsumed ? 1u : 0u;
        const auto sequence = activeSequences_[hi][bi].record;
        if (!down && sequence.pressInputId) {
            // A selected native press must continue into the native falling-edge
            // handler, but is closed to another recognizer, just like an owned press.
            d.consumed = 1;
            d.requestId = sequence.requestId;
            if (sequence.nativeAction) d.reason |= HIGGS_DECISION_NATIVE;
            EndSequenceLocked(hi, bi, input.inputId, 0);
        }
        decisionContexts_[input.inputId % decisionContexts_.size()] = contextGeneration;
    }
    const auto contextCurrent = [&] {
        std::lock_guard lock(mutex_);
        return !contextMenus_ && contextGeneration == contextGeneration_;
    };
    HIGGS_ControllerFrame raw{};
    InputFrames().Latest(hand, raw);
    auto list = button < kAuxiliaryInputBase ? List(1) : std::vector<std::shared_ptr<Registration>>{};
    for (auto &r : list)
        if (contextCurrent() && Enter(r))
        {
            CallbackScope scope(this, r->owner, input.inputId);
            const auto oldHand = recognitionHand;
            const auto oldGeneration = recognitionGeneration;
            recognitionHand = hand;
            recognitionGeneration = input.handGeneration;
            try
            {
                r->recognizer(&input, r->context);
            }
            catch (...)
            {
            }
            recognitionHand = oldHand;
            recognitionGeneration = oldGeneration;
            Leave(r);
        }
    if (raw.frameId && raw.frameId == CurrentControllerFrameId(hand) && raw.epoch == input.epoch &&
        raw.observedAtNs <= now && contextCurrent())
    {
        HIGGS_InputEvent event{};
        event.structSize = sizeof(event);
        event.physicalHand = hand;
        event.controlId = control;
        event.kind = down ? HIGGS_RAW_PRESS : HIGGS_RAW_RELEASE;
        event.flags =
            HIGGS_INPUT_ACTIONABLE | (button < kAuxiliaryInputBase ? uint32_t(HIGGS_INPUT_EFFECTIVE_SEMANTIC) : 0u);
        event.axisIndex = UINT32_MAX;
        event.inputId = input.inputId;
        event.frame = raw;
        event.snapshot = input.snapshot;
        auto extended = List(5);
        for (auto &r : extended)
            if ((r->input.physicalHands & hand) && (r->input.controlMask & (UINT64_C(1) << control)) &&
                (!r->input.eventMask || (r->input.eventMask & (1u << (event.kind - 1)))) && contextCurrent() && Enter(r))
            {
                CallbackScope scope(this, r->owner, input.inputId);
                const auto *oldEvent = extendedRecognition;
                const bool oldFresh = recognitionFreshDown;
                const auto oldHand = recognitionHand;
                const auto oldGeneration = recognitionGeneration;
                const auto oldRegistration = extendedRegistration;
                extendedRecognition = &event;
                extendedRegistration = r->id;
                recognitionFreshDown = freshDown;
                recognitionHand = hand;
                recognitionGeneration = input.handGeneration;
                try
                {
                    r->input.callback(&event, r->input.context);
                }
                catch (...)
                {
                }
                recognitionHand = oldHand;
                recognitionGeneration = oldGeneration;
                recognitionFreshDown = oldFresh;
                extendedRegistration = oldRegistration;
                extendedRecognition = oldEvent;
                Leave(r);
            }
    }
    std::shared_ptr<Registration> executor;
    HIGGS_ActionRequest request{};
    bool consumed = wasConsumed;
    {
        std::lock_guard lock(mutex_);
        Action *winner = nullptr;
        int32_t priority = (std::numeric_limits<int32_t>::min)();
        bool winnerOverride = false;
        uint32_t tieRules = 0;
        struct Ranked { Action *action; int32_t priority; bool overridden; };
        std::array<Ranked,256> candidates{};
        size_t count=0;
        const auto providerKey = [&](const Action &a) {
            return std::make_tuple(ClientKeyLocked(a.request.owner), a.request.actionId, a.stableTarget,
                                   ClientKeyLocked(a.recognizerOwner));
        };
        for (auto &a : actions_)
            if (a.request.inputId == input.inputId && !a.selected && !a.result.stage)
            {
                auto client = ClientLocked(a.request.owner);
                auto exec = RegistrationLocked(a.request.executor);
                const auto recognizer = a.inputRegistration ? RegistrationLocked(a.inputRegistration) : nullptr;
                bool overridden = false;
                const auto effective = client ? ActionPriorityLocked(*client, a.request.actionId, overridden) : 0;
                if (client && exec && exec->active &&
                    (!a.inputRegistration || (recognizer && recognizer->active && recognizer->kind == 5)) &&
                    RoutedLocked(a.request.executor, a.recognizerOwner, a.request.physicalHands) &&
                    contextGeneration == contextGeneration_ && !contextMenus_ &&
                    input.epoch == snapshot_.epoch && snapshot_.hands[hi].generation == input.handGeneration &&
                    InputEligibleLocked(a.request.physicalHands, a.request.owner, a.inputSpec.existingGrant))
                {
                    candidates[count++] = {&a,effective,overridden};
                }
            }
        std::sort(candidates.begin(),candidates.begin()+count,[&](const Ranked &a,const Ranked &b) {
            if(a.priority!=b.priority)return a.priority>b.priority;
            return providerKey(*a.action)<providerKey(*b.action);
        });
        for(size_t i=0;i<count;){
            size_t next=i+1;
            while(next<count && candidates[next].priority==candidates[i].priority &&
                  providerKey(*candidates[next].action)==providerKey(*candidates[i].action))++next;
            if(next==i+1){
                winner=candidates[i].action;
                priority=candidates[i].priority;
                winnerOverride=candidates[i].overridden;
                if(next<count && candidates[next].priority==priority)tieRules=HIGGS_DECISION_STABLE_ID;
                break;
            }
            i=next; // Ambiguous binding, continue to the next eligible semantic action.
        }
        uint32_t nativeTargetMask = 0;
        if (button == HIGGS_INPUT_GRIP) nativeTargetMask = HIGGS_OBS_NATIVE_GRIP_TARGET;
        if (button == HIGGS_INPUT_TRIGGER) nativeTargetMask = HIGGS_OBS_NATIVE_TRIGGER_TARGET;
        const bool nativeEligible = freshDown && !wasConsumed &&
            (snapshot_.hands[hi].observationMask & nativeTargetMask) &&
            now >= snapshot_.sampledAtNs && now - snapshot_.sampledAtNs <= UINT64_C(100000000) &&
            input.snapshot.observationId == snapshot_.observationId && input.epoch == snapshot_.epoch &&
            contextGeneration == contextGeneration_ && EligibleLocked(hand);
        Client native{};
        std::memcpy(native.info.name, "higgs", 6);
        bool nativeOverride = false;
        const int32_t nativePriority = nativeEligible ? ActionPriorityLocked(native, 1, nativeOverride) : 0;
        bool nativeSelected = nativeEligible && (!winner || nativePriority > priority ||
            (nativePriority == priority && std::string_view("higgs") < ClientKeyLocked(winner->request.owner)));
        if (nativeSelected) winner = nullptr;
        for (auto &a : actions_)
            if (a.request.inputId == input.inputId && !a.selected && !a.result.stage && &a != winner)
                FinishLocked(a, HIGGS_ACTION_REJECTED, HIGGS_HAND_CONFLICT);
        if (winner && !winner->inputSpec.existingGrant && nextToken_ == (std::numeric_limits<uint64_t>::max)())
        {
            FinishLocked(*winner, HIGGS_ACTION_REJECTED, HIGGS_HAND_CAPACITY);
            winner = nullptr;
        }
        if (winner)
        {
            winner->selected = true;
            winner->grantActive = true;
            winner->request.grant = winner->inputSpec.existingGrant ? winner->inputSpec.existingGrant : nextToken_++;
            winner->request.poseObservationId = input.poseObservationId;
            FinishLocked(*winner, HIGGS_ACTION_ACCEPTED, 0);
            request = winner->request;
            if (winner->inputSpec.structSize)
            {
                winner->recognition.stage =
                    winner->inputSpec.gesture ? HIGGS_RECOGNITION_WAITING : HIGGS_RECOGNITION_MATCHED;
                winner->inputPressed = down;
                winner->inputDownAt = now;
                if (!winner->inputSpec.gesture)
                {
                    winner->recognition.recognizedAtNs = now;
                    winner->recognition.recognizedFrame = raw;
                }
            }
            if (!winner->inputSpec.gesture)
                executor = RegistrationLocked(request.executor);
            consumed = true;
            auto &d = decisions_[input.inputId % decisions_.size()];
            d.consumed = 1;
            d.owner = request.owner;
            d.grant = request.grant;
            d.requestId = request.requestId;
            d.effectivePriority = priority;
            d.reason = (winnerOverride ? HIGGS_DECISION_USER_PRIORITY : HIGGS_DECISION_CLIENT_PRIORITY) | tieRules;
        }
        if (nativeSelected) {
            auto &d = decisions_[input.inputId % decisions_.size()];
            d.effectivePriority = nativePriority;
            d.reason = HIGGS_DECISION_NATIVE | (nativeOverride ? HIGGS_DECISION_USER_PRIORITY : HIGGS_DECISION_CLIENT_PRIORITY);
        }
        if (freshDown && (winner || nativeSelected)) {
            auto &active = activeSequences_[hi][bi];
            active.owner = request.owner;
            active.recognizer = winner ? winner->recognizerOwner : 0;
            auto &route = active.record;
            route = {sizeof(route), HIGGS_SEQUENCE_HELD, hand, button, nativeSelected ? 1u : 0u, 0,
                     input.epoch, input.inputId, 0, request.requestId, request.executor,
                     GrantCorrelationLocked(request.grant), nativeSelected ? UINT64_C(1) : request.actionId};
        }
        if (bi<2) nativeFallbackPress_[hi][bi] = freshDown && !winner && !nativeSelected && !consumed ? input.inputId : 0;
        consumed_[hi][bi] = down && consumed;
        RefreshLocked();
    }
    if (executor && Enter(executor))
    {
        {
            std::lock_guard lock(mutex_);
            auto action = ActionLocked(request.requestId);
            if (contextMenus_ || contextGeneration != contextGeneration_ || !action || !action->grantActive) {
                // Enter has acquired one callback reference; no callback is run.
                --executor->inFlight;
                drained_.notify_all();
                return consumed;
            }
            action->dispatched = true;
        }
        CallbackScope scope(this, executor->owner);
        try
        {
            executor->executor(&request, executor->context);
        }
        catch (...)
        {
            std::lock_guard lock(mutex_);
            if (auto a = ActionLocked(request.requestId))
            {
                a->grantActive = false;
                FinishLocked(*a, HIGGS_ACTION_REJECTED, HIGGS_HAND_INTERNAL_ERROR);
                RefreshLocked();
            }
        }
        Leave(executor);
    }
    FlushEvents();
    return consumed;
}
HIGGS_HandStatus Broker::Report(uint64_t owner, const HIGGS_ActionResult &result)
{
    std::lock_guard serial(dispatch_);
    {
        std::lock_guard lock(mutex_);
        ExpireLocked(Now());
        if (!ClientLocked(owner))
            return HIGGS_HAND_INVALID_CLIENT;
        auto a = ActionLocked(result.requestId);
        if (!a)
            return HIGGS_HAND_NOT_FOUND;
        if (a->domainOwned)
            return HIGGS_HAND_WRONG_PHASE;
        if (a->nativeStarted)
            return HIGGS_HAND_PENDING;
        if (a->request.owner != owner || a->request.executor != result.executor)
            return HIGGS_HAND_WRONG_OWNER;
        if (a->recognition.stage == HIGGS_RECOGNITION_WAITING &&
            (result.stage == HIGGS_ACTION_COMMITTED || result.stage == HIGGS_ACTION_COMPLETED))
            return HIGGS_HAND_WRONG_PHASE;
        if (result.structSize < sizeof(result) || result.stage < HIGGS_ACTION_COMMITTED ||
            result.stage > HIGGS_ACTION_CANCELLED || (result.receiptFlags & ~7u))
            return HIGGS_HAND_INVALID_ARGUMENT;
        if (a->request.epoch != result.epoch || result.epoch != snapshot_.epoch || a->request.inputId != result.inputId)
            return HIGGS_HAND_STALE;
        if (!a->selected || !result.executor)
            return HIGGS_HAND_WRONG_PHASE;
        if (std::memcmp(&a->result, &result, sizeof(result)) == 0)
            return HIGGS_HAND_OK;
        if (Terminal(a->result.stage))
            return HIGGS_HAND_CONFLICT;
        if (result.stage < a->result.stage || (a->result.receiptFlags & ~result.receiptFlags))
            return HIGGS_HAND_CONFLICT;
        if (a->result.stage >= HIGGS_ACTION_COMMITTED && result.stage == HIGGS_ACTION_REJECTED)
            return HIGGS_HAND_CONFLICT;
        a->result = result;
        QueueResultLocked(*a);
        if (result.stage == HIGGS_ACTION_REJECTED || result.stage == HIGGS_ACTION_CANCELLED)
            a->grantActive = false;
        RefreshLocked();
    }
    FlushEvents();
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::Result(uint64_t owner, uint64_t id, HIGGS_ActionResult &out)
{
    std::lock_guard lock(mutex_);
    ExpireLocked(Now());
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto a = ActionLocked(id);
    if (!a)
        return HIGGS_HAND_NOT_FOUND;
    if (a->request.owner != owner && a->recognizerOwner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    out = a->result;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::Decision(uint64_t owner, uint64_t id, HIGGS_InputDecision &out)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    const auto &d = decisions_[id % decisions_.size()];
    if (!id || d.inputId != id)
        return HIGGS_HAND_NOT_FOUND;
    out = d;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::Release(uint64_t owner, uint64_t grant)
{
    std::lock_guard serial(dispatch_);
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    for (const auto &a : actions_)
        if (grant && a.request.grant == grant && a.request.owner == owner && a.nativeStarted)
            return HIGGS_HAND_PENDING; // Already entered the native mutator; retry after this phase.
    bool found = false;
    for (auto &a : actions_)
        if (grant && a.request.grant == grant)
        {
            if (a.request.owner != owner)
                return HIGGS_HAND_WRONG_OWNER;
            found = true;
            a.grantActive = false;
            a.operationActive = false;
            if (!Terminal(a.result.stage))
                FinishLocked(a, HIGGS_ACTION_CANCELLED, 0);
        }
    RefreshLocked();
    return found ? HIGGS_HAND_OK : HIGGS_HAND_NOT_FOUND;
}
void Broker::ObserveWorld(uintptr_t world, bool loading, Clock now)
{
    std::lock_guard dispatch(dispatch_);
    {
        std::lock_guard lock(mutex_);
        if (world_ == world && loading_ == loading && !worldChangePending_ && !forceWorldReset_)
            return;
        for (const auto &action : actions_)
            if (action.nativeStarted)
            {
                worldChangePending_ = true;
                pendingWorld_ = world;
                pendingWorldLoading_ = loading;
                pendingWorldAt_ = now;
                return;
            }
        forceWorldReset_ = false;
        CancelSequencesLocked(0, HIGGS_HAND_STALE);
        CancelSequencesLocked(1, HIGGS_HAND_STALE);
        const auto previousContext = ContextStateLocked();
        world_ = world;
        loading_ = loading;
        const auto epoch = snapshot_.epoch + 1, generation = snapshot_.generation + 1;
        snapshot_ = {};
        snapshot_.structSize = sizeof(snapshot_);
        snapshot_.epoch = epoch;
        snapshot_.generation = generation;
        snapshot_.worldReady = world && !loading;
        snapshot_.sampledAtNs = now;
        for (auto& frame : nativeRegionFrames_) frame = {};
        for (auto& context : nativeRegionContexts_) context = 0;
        snapshot_.hands[0].physicalHand = HIGGS_HAND_LEFT;
        snapshot_.hands[1].physicalHand = HIGGS_HAND_RIGHT;
        for (auto &hand : contextHands_) hand = {};
        ++contextGeneration_;
        contextChangedAt_ = now;
        pendingContexts_.emplace_back(previousContext, ContextStateLocked());
        for (auto &a : actions_)
            if (a.request.requestId)
            {
                a.grantActive = false;
                a.operationActive = false;
                if (!Terminal(a.result.stage))
                    FinishLocked(a, HIGGS_ACTION_CANCELLED, HIGGS_HAND_STALE);
            }
        for (auto &row : down_)
            for (auto &value : row)
                value = false;
        for (auto &row : consumed_)
            for (auto &value : row)
                value = false;
        QueueEventLocked(HIGGS_EVENT_RESET, 0, 0, now, nullptr);
        RefreshLocked();
    }
    FlushEvents();
}
uint64_t Broker::Epoch() const
{
    std::lock_guard lock(mutex_);
    return snapshot_.epoch;
}
uint64_t Broker::HandGeneration(uint32_t hand) const
{
    std::lock_guard lock(mutex_);
    return snapshot_.hands[hand == HIGGS_HAND_LEFT ? 0 : 1].generation;
}
void Broker::ObserveSecondaryEpisode(uint32_t hand, uint64_t episode)
{
    if (hand != HIGGS_HAND_LEFT && hand != HIGGS_HAND_RIGHT)
        return;
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    const auto index = hand == HIGGS_HAND_LEFT ? 0 : 1;
    if (secondaryEpisodes_[index] != episode)
    {
        secondaryEpisodes_[index] = episode;
        snapshot_.hands[index].generation = nextToken_++;
    }
}
uint64_t Broker::SecondaryEpisode(uint32_t hand) const
{
    std::lock_guard lock(mutex_);
    return secondaryEpisodes_[hand == HIGGS_HAND_LEFT ? 0 : 1];
}
HIGGS_HandSnapshot Broker::CopyPublished() const
{
    std::lock_guard lock(mutex_);
    return snapshot_;
}
bool Broker::MutationAdmissionReady(bool recovery)
{
    std::lock_guard lock(mutex_);
    return snapshot_.worldReady && !worldChangePending_ && (recovery || !contextMenus_);
}
void Broker::ApplyDeferredWorld()
{
    auto phase = NativePhase();
    uintptr_t world = 0;
    bool loading = false;
    Clock at = 0;
    {
        std::lock_guard lock(mutex_);
        if (!worldChangePending_)
            return;
        for (const auto &action : actions_)
            if (action.nativeStarted)
                return;
        world = pendingWorld_;
        loading = pendingWorldLoading_;
        at = pendingWorldAt_;
        worldChangePending_ = false;
        forceWorldReset_ = true;
    }
    ObserveWorld(world, loading, at);
}
void Broker::MarkMainThread()
{
    std::lock_guard lock(mutex_);
    mainThread_ = std::this_thread::get_id();
}
HIGGS_HandStatus Broker::WithAuthorizedAction(uint64_t owner, const HIGGS_GrantedMutationRequest &r,
                                              HIGGS_GrantedMutationCallback callback, void *context,
                                              HIGGS_ActionResult &output)
{
    if (r.structSize < sizeof(r) || !callback ||
        (r.commitStage != HIGGS_ACTION_COMMITTED && r.commitStage != HIGGS_ACTION_COMPLETED))
        return HIGGS_HAND_INVALID_ARGUMENT;
    auto phase = NativePhase();
    DeferredWorldDrain deferredWorldDrain{*this};
    HIGGS_ActionRequest request{};
    HIGGS_HandSnapshot snapshot{};
    {
        std::lock_guard lock(mutex_);
        ExpireLocked(Now());
        if (!ClientLocked(owner))
            return HIGGS_HAND_INVALID_CLIENT;
        if (worldChangePending_ || contextMenus_)
            return HIGGS_HAND_NOT_READY;
        if (mainThread_ == std::thread::id{} || mainThread_ != std::this_thread::get_id())
            return HIGGS_HAND_WRONG_PHASE;
        auto a = ActionLocked(r.requestId);
        if (!a)
            return HIGGS_HAND_NOT_FOUND;
        if (a->request.owner != owner || a->request.executor != r.executor)
            return HIGGS_HAND_WRONG_OWNER;
        if (a->domainOwned || a->nativeStarted || !a->selected || a->result.stage >= HIGGS_ACTION_COMPLETED ||
            a->recognition.stage == HIGGS_RECOGNITION_WAITING)
            return HIGGS_HAND_WRONG_PHASE;
        if (a->request.inputId && a->result.stage < HIGGS_ACTION_COMMITTED &&
            a->inputContextGeneration != contextGeneration_)
            return HIGGS_HAND_STALE;
        if (r.epoch != snapshot_.epoch || a->request.epoch != r.epoch || r.grant != a->request.grant)
            return HIGGS_HAND_STALE;
        auto executor = RegistrationLocked(r.executor);
        if (!executor || !executor->active)
            return HIGGS_HAND_NOT_FOUND;
        for (const auto &hand : snapshot_.hands)
            if (hand.physicalHand & a->request.physicalHands)
                if (hand.grant != r.grant || hand.owner != owner)
                    return HIGGS_HAND_STALE;
        a->nativeStarted = true;
        request = a->request;
        snapshot = ObservationLocked(snapshot_);
    }
    HIGGS_HandStatus status = HIGGS_HAND_INTERNAL_ERROR;
    {
        CallbackScope scope(this, owner);
        try
        {
            status = callback(&request, &snapshot, context);
        }
        catch (...)
        {
        }
    }
    {
        std::lock_guard lock(mutex_);
        auto a = ActionLocked(r.requestId);
        if (!a)
            return HIGGS_HAND_INTERNAL_ERROR;
        if (status == HIGGS_HAND_OK)
        {
            FinishLocked(*a, r.commitStage, 0);
            output = a->result;
        }
        a->nativeStarted = false;
        ExpireLocked(Now());
    }
    ApplyDeferredWorld();
    FlushEvents();
    return status;
}
HIGGS_HandStatus Broker::InvokeAuthorized(uint64_t owner, const HIGGS_AuthorizedDispatchRequest &r,
                                         HIGGS_AuthorizedDispatchCallback callback, void *context)
{
    if (r.structSize != sizeof(r) || r.reserved || !r.requestId || !r.executor || !r.epoch || !r.grant || !callback)
        return HIGGS_HAND_INVALID_ARGUMENT;
    // Copy before entering caller code; caller storage is never used after entry.
    const auto invocation = r;
    auto phase = NativePhase();
    DeferredWorldDrain deferredWorldDrain{*this};
    HIGGS_ActionRequest request{};
    HIGGS_HandSnapshot snapshot{};
    {
        std::lock_guard lock(mutex_);
        ExpireLocked(Now());
        if (!ClientLocked(owner))
            return HIGGS_HAND_INVALID_CLIENT;
        if (mainThread_ == std::thread::id{} || mainThread_ != std::this_thread::get_id())
            return HIGGS_HAND_WRONG_PHASE;
        auto a = ActionLocked(invocation.requestId);
        if (!a)
            return HIGGS_HAND_NOT_FOUND;
        if (a->request.owner != owner || a->request.executor != invocation.executor)
            return HIGGS_HAND_WRONG_OWNER;
        // A claimed invocation is never retried, even when its callback failed or
        // a later deadline/load retired its authority. History remains bounded.
        if (a->authorizedDispatchClaimed)
            return HIGGS_HAND_CONFLICT;
        if (!snapshot_.worldReady || worldChangePending_ || contextMenus_)
            return HIGGS_HAND_NOT_READY;
        if (a->domainOwned || a->nativeStarted || !a->selected || !a->grantActive ||
            a->result.stage >= HIGGS_ACTION_COMPLETED || a->recognition.stage == HIGGS_RECOGNITION_WAITING)
            return HIGGS_HAND_WRONG_PHASE;
        if (a->request.inputId && a->result.stage < HIGGS_ACTION_COMMITTED &&
            a->inputContextGeneration != contextGeneration_)
            return HIGGS_HAND_STALE;
        if (invocation.epoch != snapshot_.epoch || a->request.epoch != invocation.epoch ||
            invocation.grant != a->request.grant)
            return HIGGS_HAND_STALE;
        auto executor = RegistrationLocked(invocation.executor);
        if (!executor || !executor->active || executor->kind != 2)
            return HIGGS_HAND_NOT_FOUND;
        constexpr uint32_t known = HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN |
                                   HIGGS_OBS_PALM_VALID | HIGGS_OBS_TRACKING_VALID;
        for (const auto &hand : snapshot_.hands)
            if (hand.physicalHand & a->request.physicalHands)
            {
                if (hand.grant != invocation.grant || hand.owner != owner)
                    return HIGGS_HAND_STALE;
                if (hand.availability != HIGGS_AVAILABILITY_RESERVED || hand.unavailableReasons ||
                    (hand.observationMask & known) != known || (hand.observationMask & HIGGS_OBS_SECONDARY_SUPPORT))
                    return HIGGS_HAND_NOT_READY;
            }
        a->authorizedDispatchClaimed = true;
        a->nativeStarted = true;
        request = a->request;
        snapshot = ObservationLocked(snapshot_);
    }
    HIGGS_HandStatus status = HIGGS_HAND_INTERNAL_ERROR;
    {
        CallbackScope scope(this, owner);
        try { status = callback(&request, &snapshot, context); }
        catch (...) { /* Arbitrary effects cannot be inferred or rolled back. */ }
    }
    {
        std::lock_guard lock(mutex_);
        // nativeStarted prevents eviction, revocation and deferred world reset.
        if (auto a = ActionLocked(invocation.requestId))
            a->nativeStarted = false;
        ExpireLocked(Now());
    }
    ApplyDeferredWorld();
    FlushEvents();
    return status;
}
std::unique_lock<std::recursive_mutex> Broker::NativePhase()
{
    return std::unique_lock<std::recursive_mutex>(dispatch_);
}
void Broker::BeginHandMutation(Clock now)
{
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    for (auto &a : actions_)
        if (a.physical.kind && a.dispatched && !Terminal(a.result.stage))
        {
            const auto &h = snapshot_.hands[a.physical.physicalHand == HIGGS_HAND_LEFT ? 0 : 1];
            if (h.grant == a.request.grant && h.owner == a.request.owner)
                a.nativeStarted = true;
        }
}
void Broker::EndHandMutation(uint32_t hand, uint32_t held, bool known, bool secondary, Clock now)
{
    auto phase = NativePhase();
    DeferredWorldDrain deferredWorldDrain{*this};
    std::lock_guard lock(mutex_);
    for (auto &a : actions_)
        if (a.physical.physicalHand == hand && a.nativeStarted)
        {
            a.nativeStarted = false;
            if (!Terminal(a.result.stage) &&
                ((a.physical.kind == HIGGS_PHYSICAL_GRAB && known && held == a.physical.itemRefId) ||
                 (a.physical.kind == HIGGS_PHYSICAL_RELEASE && known && !held && !secondary) ||
                 (a.physical.kind == HIGGS_PHYSICAL_SECONDARY_SUPPORT && secondary)))
            {
                a.result.receiptFlags |= HIGGS_RECEIPT_PHYSICAL_CONFIRMED;
                a.result.itemRefId = a.physical.itemRefId;
                a.result.quantity = 1;
                FinishLocked(a, HIGGS_ACTION_COMPLETED, 0);
                if (a.physical.kind == HIGGS_PHYSICAL_RELEASE)
                    a.grantActive = false;
            }
        }
    ExpireLocked(now);
}
void Broker::Publish(const std::array<HIGGS_HandRecord, 2> &records, Clock now)
{
    std::lock_guard dispatch(dispatch_);
    {
        std::lock_guard lock(mutex_);
        ExpireLocked(now);
        if (!snapshot_.worldReady)
            return;
        const auto old = snapshot_.hands[0];
        const auto oldRight = snapshot_.hands[1];
        const std::array<HIGGS_HandRecord, 2> previous{old, oldRight};
        snapshot_.structSize = sizeof(snapshot_);
        snapshot_.observationId = nextObservation_++;
        snapshot_.sampledAtNs = now;
        ++snapshot_.generation;
        for (size_t i = 0; i < 2; ++i)
        {
            auto h = records[i];
            nativeUnavailableReasons_[i] = h.unavailableReasons;
            h.physicalHand = i == 0 ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT;
            const auto &p = previous[i];
            const bool changed = p.heldRefId != h.heldRefId || p.equippedFormId != h.equippedFormId ||
                                 p.observationMask != h.observationMask ||
                                 ((p.unavailableReasons ^ h.unavailableReasons) & ~HIGGS_HAND_REASON_MENU) != 0;
            h.generation = changed || !p.generation ? nextToken_++ : p.generation;
            h.occupancy = (h.heldRefId ? HIGGS_OCCUPANCY_HIGGS_HELD : 0u) |
                          (h.equippedFormId ? HIGGS_OCCUPANCY_GAME_EQUIPPED : 0u);
            if (h.observationMask & HIGGS_OBS_PALM_VALID)
            {
                h.poseObservationId = snapshot_.observationId;
                h.poseObservedAtNs = now;
            }
            snapshot_.hands[i] = h;
            if (!(h.observationMask & HIGGS_OBS_TRACKING_VALID))
                CancelSequencesLocked(i, HIGGS_HAND_NOT_READY);
            if (!(h.observationMask & HIGGS_OBS_TRACKING_VALID))
                for (auto &a : actions_)
                    if (a.grantActive && (a.request.physicalHands & h.physicalHand))
                    {
                        a.grantActive = false;
                        if (!Terminal(a.result.stage))
                            FinishLocked(a, HIGGS_ACTION_CANCELLED, HIGGS_HAND_NOT_READY);
                    }
        }
        for (auto &a : actions_)
            if (a.physical.kind && a.dispatched && !Terminal(a.result.stage))
            {
                const auto &h = snapshot_.hands[a.physical.physicalHand == HIGGS_HAND_LEFT ? 0 : 1];
                if ((a.physical.kind == HIGGS_PHYSICAL_GRAB && h.heldRefId == a.physical.itemRefId) ||
                    (a.physical.kind == HIGGS_PHYSICAL_SECONDARY_SUPPORT &&
                     (h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT)) ||
                    (a.physical.kind == HIGGS_PHYSICAL_RELEASE && (h.observationMask & HIGGS_OBS_HELD_KNOWN) &&
                     !h.heldRefId && !(h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT)))
                {
                    a.result.receiptFlags |= HIGGS_RECEIPT_PHYSICAL_CONFIRMED;
                    a.result.itemRefId = a.physical.itemRefId;
                    a.result.quantity = 1;
                    FinishLocked(a, HIGGS_ACTION_COMPLETED, 0);
                    if (a.physical.kind == HIGGS_PHYSICAL_RELEASE)
                        a.grantActive = false;
                }
            }
        RefreshLocked();
        for (size_t i = 0; i < 2; ++i)
        {
            const auto &h = snapshot_.hands[i];
            const auto &p = previous[i];
            if (p.heldRefId != h.heldRefId)
            {
                if (p.heldRefId)
                    QueueEventLocked(HIGGS_EVENT_RELEASED, h.physicalHand, p.heldRefId, now, &h);
                if (h.heldRefId)
                    QueueEventLocked(HIGGS_EVENT_GRABBED, h.physicalHand, h.heldRefId, now, &h);
            }
            if ((p.observationMask ^ h.observationMask) & HIGGS_OBS_SECONDARY_SUPPORT)
                QueueEventLocked((h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT) ? HIGGS_EVENT_SECONDARY_STARTED
                                                                                   : HIGGS_EVENT_SECONDARY_ENDED,
                                 h.physicalHand, h.equippedFormId, now, &h);
        }
    }
    FlushEvents();
}
HIGGS_HandStatus Broker::Physical(uint64_t owner, const HIGGS_PhysicalRequest &p, uint64_t &out, Clock now)
{
    auto phase = NativePhase(); // Serialize new grants with native hand writes.
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (worldChangePending_ || (contextMenus_ && p.kind != HIGGS_PHYSICAL_RELEASE))
        return HIGGS_HAND_NOT_READY;
    if (p.structSize < sizeof(p) || !Hand(p.physicalHand) || p.reserved || !p.timeoutMs || p.timeoutMs > 60000 ||
        (!p.itemRefId && p.kind == HIGGS_PHYSICAL_GRAB))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (p.kind != HIGGS_PHYSICAL_GRAB && p.kind != HIGGS_PHYSICAL_RELEASE && p.kind != HIGGS_PHYSICAL_SECONDARY_SUPPORT)
        return HIGGS_HAND_UNSUPPORTED;
    if (p.epoch != snapshot_.epoch)
        return HIGGS_HAND_STALE;
    ExpireLocked(now);
    const auto &h = snapshot_.hands[p.physicalHand == HIGGS_HAND_LEFT ? 0 : 1];
    if (h.generation != p.handGeneration)
        return HIGGS_HAND_STALE;
    if (h.availability == HIGGS_AVAILABILITY_UNKNOWN ||
        (h.availability == HIGGS_AVAILABILITY_UNAVAILABLE &&
         !(p.kind == HIGGS_PHYSICAL_RELEASE && !(h.unavailableReasons & ~HIGGS_HAND_REASON_MENU))))
        return HIGGS_HAND_NOT_READY;
    if (h.grant && (h.grant != p.grant || h.owner != owner))
        return HIGGS_HAND_CONFLICT;
    if (p.grant && (!h.grant || h.owner != owner))
        return HIGGS_HAND_WRONG_OWNER;
    if (p.grant && !GrantReadyLocked(owner, p.grant))
        return HIGGS_HAND_WRONG_PHASE;
    if ((p.kind == HIGGS_PHYSICAL_GRAB || p.kind == HIGGS_PHYSICAL_SECONDARY_SUPPORT) &&
        (h.heldRefId || (h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT)))
        return HIGGS_HAND_CONFLICT;
    for (const auto &pending : actions_)
        if (pending.physical.kind && pending.physical.physicalHand == p.physicalHand &&
            pending.result.stage < HIGGS_ACTION_COMPLETED)
            return HIGGS_HAND_CONFLICT;
    for (const auto &operation : actions_)
        if (operation.operationActive && operation.operation.operation.physicalHand == p.physicalHand &&
            (operation.operation.operation.kind == HIGGS_OP_WORLD_SUPPORT ||
             operation.operation.operation.kind == HIGGS_OP_PHYSICAL_HAND_TARGET ||
             operation.operation.operation.kind == HIGGS_OP_HAND_GEOMETRY))
            return HIGGS_HAND_CONFLICT;
    if (p.kind == HIGGS_PHYSICAL_RELEASE && (!h.heldRefId && !(h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT)))
        return HIGGS_HAND_CONFLICT;
    if (p.kind == HIGGS_PHYSICAL_RELEASE && p.itemRefId && p.itemRefId != h.heldRefId)
        return HIGGS_HAND_STALE;
    auto a = NewActionLocked();
    if (!a)
        return HIGGS_HAND_CAPACITY;
    a->request.structSize = sizeof(a->request);
    a->request.requestId = nextToken_++;
    a->request.owner = owner;
    a->request.epoch = p.epoch;
    a->request.physicalHands = p.physicalHand;
    a->request.grant = p.grant ? p.grant : nextToken_++;
    a->physical = p;
    if (!a->physical.itemRefId)
        a->physical.itemRefId = h.heldRefId;
    a->selected = true;
    a->grantActive = !p.grant;
    a->expires = now + static_cast<Clock>(p.timeoutMs) * 1000000;
    FinishLocked(*a, HIGGS_ACTION_ACCEPTED, 0);
    out = a->request.requestId;
    RefreshLocked();
    return HIGGS_HAND_OK;
}
void Broker::DrainPhysical(PhysicalDispatch dispatch, void *context, Clock now)
{
    std::lock_guard serial(dispatch_);
    DeferredWorldDrain deferredWorldDrain{*this};
    std::vector<std::pair<uint64_t, HIGGS_PhysicalRequest>> work;
    {
        std::lock_guard lock(mutex_);
        ExpireLocked(now);
        for (auto &a : actions_)
            if (a.physical.kind && !a.dispatched && !Terminal(a.result.stage))
            {
                const auto &h = snapshot_.hands[a.physical.physicalHand == HIGGS_HAND_LEFT ? 0 : 1];
                if (a.request.epoch != snapshot_.epoch || h.generation != a.physical.handGeneration ||
                    h.grant != a.request.grant || h.owner != a.request.owner ||
                    h.availability == HIGGS_AVAILABILITY_UNKNOWN ||
                    (h.availability == HIGGS_AVAILABILITY_UNAVAILABLE &&
                     !(a.physical.kind == HIGGS_PHYSICAL_RELEASE && !(h.unavailableReasons & ~HIGGS_HAND_REASON_MENU))))
                {
                    a.grantActive = false;
                    FinishLocked(a, HIGGS_ACTION_CANCELLED, HIGGS_HAND_STALE);
                }
                else
                {
                    a.dispatched = true;
                    work.emplace_back(a.request.requestId, a.physical);
                }
            }
    }
    for (auto &task : work)
    {
        {
            std::lock_guard lock(mutex_);
            auto a = ActionLocked(task.first);
            if (!a || Terminal(a->result.stage))
                continue;
            const auto &h = snapshot_.hands[a->physical.physicalHand == HIGGS_HAND_LEFT ? 0 : 1];
            if (worldChangePending_ || (contextMenus_ && a->physical.kind != HIGGS_PHYSICAL_RELEASE) || a->request.epoch != snapshot_.epoch ||
                h.generation != a->physical.handGeneration || h.grant != a->request.grant ||
                h.owner != a->request.owner)
            {
                a->grantActive = false;
                FinishLocked(*a, HIGGS_ACTION_CANCELLED, HIGGS_HAND_STALE);
                continue;
            }
            a->nativeStarted = true;
        }
        bool accepted = false;
        try
        {
            accepted = dispatch && dispatch(task.second, context);
        }
        catch (...)
        {
        }
        std::lock_guard lock(mutex_);
        if (auto a = ActionLocked(task.first))
        {
            a->nativeStarted = false;
            if (!Terminal(a->result.stage))
            {
                if (accepted)
                    FinishLocked(*a, HIGGS_ACTION_COMMITTED, 0);
                else
                {
                    a->grantActive = false;
                    FinishLocked(*a, HIGGS_ACTION_REJECTED, HIGGS_HAND_NOT_READY);
                }
            }
        }
    }
    ApplyDeferredWorld();
    FlushEvents();
}
void Broker::SemanticEvent(uint32_t type, uint32_t hand, uint32_t ref, Clock now)
{
    std::lock_guard dispatch(dispatch_);
    {
        std::lock_guard lock(mutex_);
        if (Hand(hand))
            QueueEventLocked(type, hand, ref, now, &snapshot_.hands[hand == HIGGS_HAND_LEFT ? 0 : 1]);
    }
    FlushEvents();
}
bool Broker::HasGrant(uint32_t hand, Clock now)
{
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    return snapshot_.hands[hand == HIGGS_HAND_LEFT ? 0 : 1].grant != 0;
}
bool Broker::MayAcquire(uint32_t hand, Clock now)
{
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    if (contextMenus_) return false;
    const auto grant = snapshot_.hands[hand == HIGGS_HAND_LEFT ? 0 : 1].grant;
    if (!grant)
        return true;
    for (const auto &a : actions_)
        if (a.request.grant == grant && a.physical.kind == HIGGS_PHYSICAL_GRAB && a.dispatched &&
            !Terminal(a.result.stage))
            return true;
    return false;
}
bool Broker::PhysicalPending(uint32_t hand, Clock now)
{
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    const auto &h = snapshot_.hands[hand == HIGGS_HAND_LEFT ? 0 : 1];
    for (const auto &a : actions_)
        if (a.physical.physicalHand == hand &&
            (a.physical.kind == HIGGS_PHYSICAL_GRAB || a.physical.kind == HIGGS_PHYSICAL_SECONDARY_SUPPORT) &&
            a.dispatched && !Terminal(a.result.stage) && h.grant == a.request.grant && h.owner == a.request.owner)
            return true;
    return false;
}
bool Broker::MaySupport(uint32_t hand, Clock now)
{
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    if (contextMenus_) return false;
    const auto grant = snapshot_.hands[hand == HIGGS_HAND_LEFT ? 0 : 1].grant;
    if (!grant)
        return true;
    for (const auto &a : actions_)
        if (a.request.grant == grant && a.physical.kind == HIGGS_PHYSICAL_SECONDARY_SUPPORT && a.dispatched &&
            !Terminal(a.result.stage))
            return true;
    return false;
}
HIGGS_HandStatus Broker::NativeRegionDetector(uint64_t owner, HIGGS_NativeRegionDetector callback,
                                             void* context, uint64_t& out)
{
    if (!callback) return HIGGS_HAND_INVALID_ARGUMENT;
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    if (nativeRegionCallback_) return HIGGS_HAND_WRONG_PHASE;
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    for (const auto& r : registrations_)
        if (r->kind == 7 && (r->active || r->inFlight)) return HIGGS_HAND_CONFLICT;
    if (registrations_.size() >= 256) return HIGGS_HAND_CAPACITY;
    auto r = std::make_shared<Registration>();
    r->id = nextToken_++; r->owner = owner; r->kind = 7;
    r->nativeRegionDetector = callback; r->context = context;
    registrations_.push_back(r);
    out = r->id;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::RemoveNativeRegionDetector(uint64_t owner, uint64_t registration)
{
    auto phase = NativePhase();
    {
        std::lock_guard lock(mutex_);
        auto r = RegistrationLocked(registration);
        if (!r || r->kind != 7) return HIGGS_HAND_NOT_FOUND;
        if (r->owner != owner) return HIGGS_HAND_WRONG_OWNER;
    }
    return Remove(owner, registration);
}
HIGGS_HandStatus Broker::CopyNativeRegionFrame(uint64_t owner, uint32_t hand, HIGGS_NativeRegionFrame& out)
{
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner)) return HIGGS_HAND_INVALID_CLIENT;
    if (!Hand(hand)) return HIGGS_HAND_INVALID_ARGUMENT;
    const auto& frame = nativeRegionFrames_[hand == HIGGS_HAND_LEFT ? 0 : 1];
    if (!snapshot_.worldReady || worldChangePending_ || !frame.sampleId || frame.epoch != snapshot_.epoch ||
        !(frame.flags & HIGGS_REGION_SOURCE_VALID)) return HIGGS_HAND_NOT_READY;
    out = frame;
    return HIGGS_HAND_OK;
}
uint32_t Broker::EvaluateNativeRegions(HIGGS_NativeRegionFrame& frame, bool* usedDetector)
{
    if (usedDetector) *usedDetector = false;
    if (!Hand(frame.physicalHand)) return 0;
    auto phase = NativePhase();
    std::shared_ptr<Registration> detector;
    {
        std::lock_guard lock(mutex_);
        // Recursion cannot overwrite the outer native source or invoke a second detector.
        if (nativeRegionCallback_) return 0;
        frame.epoch = snapshot_.epoch;
        frame.sampleId = nextNativeRegionSample_++;
        frame.observedAtNs = Now();
        frame.handGeneration = snapshot_.hands[frame.physicalHand == HIGGS_HAND_LEFT ? 0 : 1].generation;
        if (!snapshot_.worldReady || worldChangePending_ || !frame.sourceGeneration || !NativeRegionGeometryValid(frame)) frame.flags = 0;
        nativeRegionFrames_[frame.physicalHand == HIGGS_HAND_LEFT ? 0 : 1] = frame;
        nativeRegionContexts_[frame.physicalHand == HIGGS_HAND_LEFT ? 0 : 1] = contextGeneration_;
        if (!frame.flags) return 0;
        for (const auto& r : registrations_)
            if (r->kind == 7 && r->active && ClientLocked(r->owner)) { detector = r; break; }
        if (detector) { ++detector->inFlight; nativeRegionCallback_ = true; }
    }
    if (!detector) return NativeRegionFallbackMask(frame);
    HIGGS_NativeRegionResult result{};
    result.structSize = sizeof(result);
    HIGGS_HandStatus status = HIGGS_HAND_INTERNAL_ERROR;
    {
        CallbackScope scope(this, detector->owner);
        try { status = detector->nativeRegionDetector(&frame, &result, detector->context); } catch (...) {}
    }
    {
        std::lock_guard lock(mutex_);
        nativeRegionCallback_ = false;
        --detector->inFlight;
        drained_.notify_all();
        if (!snapshot_.worldReady || worldChangePending_ || frame.epoch != snapshot_.epoch) return 0;
        if (!detector->active || !ClientLocked(detector->owner)) return NativeRegionFallbackMask(frame);
        if (status == HIGGS_HAND_OK && result.structSize == sizeof(result) && result.epoch == frame.epoch &&
            result.sampleId == frame.sampleId && !(result.insideMask & ~HIGGS_REGION_ALL)) {
            if (usedDetector) *usedDetector = true;
            return result.insideMask;
        }
    }
    return NativeRegionFallbackMask(frame);
}
bool Broker::MayNativeRegionAction(uint32_t hand, uint64_t epoch, Clock now)
{
    if (!Hand(hand)) return false;
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    const auto index = hand == HIGGS_HAND_LEFT ? 0 : 1;
    const auto& current = snapshot_.hands[index];
    const auto& frame = nativeRegionFrames_[index];
    return snapshot_.worldReady && !worldChangePending_ && !contextMenus_ && epoch == snapshot_.epoch &&
        frame.epoch == epoch && (frame.flags & HIGGS_REGION_SOURCE_VALID) &&
        nativeRegionContexts_[index] == contextGeneration_ &&
        current.availability != HIGGS_AVAILABILITY_UNKNOWN &&
        current.generation == frame.handGeneration && !current.grant && !current.unavailableReasons &&
        !(current.observationMask & HIGGS_OBS_SECONDARY_SUPPORT);
}
HIGGS_HandStatus Broker::Operation(uint64_t owner, const HIGGS_HandOperation &p, uint64_t &out, Clock now)
{
    auto phase = NativePhase();
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    if (worldChangePending_ || contextMenus_)
        return HIGGS_HAND_NOT_READY;
    if (p.structSize < sizeof(p) || !Hand(p.physicalHand) || p.reserved || p.reserved2 || !p.durationMs ||
        p.durationMs > 60000)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (p.kind < HIGGS_OP_WORLD_SUPPORT || p.kind > HIGGS_OP_HAND_GEOMETRY)
        return HIGGS_HAND_UNSUPPORTED;
    for (float v : p.position)
        if (!std::isfinite(v))
            return HIGGS_HAND_INVALID_ARGUMENT;
    for (float v : p.rotation)
        if (!std::isfinite(v))
            return HIGGS_HAND_INVALID_ARGUMENT;
    if (p.kind == HIGGS_OP_WORLD_SUPPORT || p.kind == HIGGS_OP_PHYSICAL_HAND_TARGET)
    {
        if (!std::isfinite(p.maxDistance) || p.maxDistance < 1 || p.maxDistance > 200)
            return HIGGS_HAND_INVALID_ARGUMENT;
    }
    if (p.kind == HIGGS_OP_WORLD_SUPPORT && !p.itemRefId)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (p.kind == HIGGS_OP_PHYSICAL_HAND_TARGET || p.kind == HIGGS_OP_GRIP_TRANSFORM)
    {
        if (!std::isfinite(p.scale) || p.scale <= 0 || p.scale > 10)
            return HIGGS_HAND_INVALID_ARGUMENT;
        for (size_t row = 0; row < 3; ++row)
            for (size_t col = 0; col < 3; ++col)
            {
                float dot = 0;
                for (size_t j = 0; j < 3; ++j)
                    dot += p.rotation[row * 3 + j] * p.rotation[col * 3 + j];
                if (std::fabs(dot - (row == col ? 1.f : 0.f)) > 0.001f)
                    return HIGGS_HAND_INVALID_ARGUMENT;
            }
        const auto *r = p.rotation;
        const auto determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
                                 r[2] * (r[3] * r[7] - r[4] * r[6]);
        if (determinant < 0.999f)
            return HIGGS_HAND_INVALID_ARGUMENT;
    }
    if (p.kind == HIGGS_OP_DRIVE_CAP && (!std::isfinite(p.linearForceCap) || !std::isfinite(p.angularForceCap) ||
                                         p.linearForceCap < 0 || p.angularForceCap < 0))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (p.kind == HIGGS_OP_HAPTIC &&
        (!std::isfinite(p.hapticStrength) || p.hapticStrength < 0 || p.hapticStrength > 1 || p.durationMs > 1000))
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (p.reserved3)
        return HIGGS_HAND_INVALID_ARGUMENT;
    if (p.kind == HIGGS_OP_HAND_GEOMETRY)
        for (float value : p.geometryHalfExtents)
            if (!std::isfinite(value) || value < 0.005f || value > 0.5f)
                return HIGGS_HAND_INVALID_ARGUMENT;
    ExpireLocked(now);
    if (p.epoch != snapshot_.epoch)
        return HIGGS_HAND_STALE;
    const auto &h = snapshot_.hands[p.physicalHand == HIGGS_HAND_LEFT ? 0 : 1];
    if (p.handGeneration != h.generation)
        return HIGGS_HAND_STALE;
    if (h.availability == HIGGS_AVAILABILITY_UNKNOWN || h.availability == HIGGS_AVAILABILITY_UNAVAILABLE)
        return HIGGS_HAND_NOT_READY;
    if (h.grant && (h.grant != p.grant || h.owner != owner))
        return HIGGS_HAND_CONFLICT;
    if (p.grant && (!h.grant || h.owner != owner))
        return HIGGS_HAND_WRONG_OWNER;
    if (p.grant && !GrantReadyLocked(owner, p.grant))
        return HIGGS_HAND_WRONG_PHASE;
    if ((p.kind == HIGGS_OP_WORLD_SUPPORT || p.kind == HIGGS_OP_PHYSICAL_HAND_TARGET ||
         p.kind == HIGGS_OP_HAND_GEOMETRY) &&
        (h.heldRefId || h.equippedFormId || (h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT)))
        return HIGGS_HAND_CONFLICT;
    for (const auto &pending : actions_)
        if (pending.ownedActuator && (pending.request.physicalHands & p.physicalHand) &&
            (p.kind == HIGGS_OP_WORLD_SUPPORT || p.kind == HIGGS_OP_PHYSICAL_HAND_TARGET ||
             p.kind == HIGGS_OP_HAND_GEOMETRY))
            return HIGGS_HAND_CONFLICT;
    for (const auto &pending : actions_)
        if (pending.physical.kind && pending.physical.physicalHand == p.physicalHand && !Terminal(pending.result.stage))
            return HIGGS_HAND_CONFLICT;
    const bool secondaryGrip = p.kind == HIGGS_OP_GRIP_TRANSFORM && (h.observationMask & HIGGS_OBS_SECONDARY_SUPPORT) &&
                               !h.heldRefId && p.grant && p.itemRefId;
    if ((p.kind == HIGGS_OP_GRIP_TRANSFORM || p.kind == HIGGS_OP_DRIVE_CAP) && !secondaryGrip &&
        (!h.heldRefId || h.heldRefId != p.itemRefId))
        return HIGGS_HAND_CONFLICT;
    for (const auto &a : actions_)
        if (a.operationActive && a.operation.operation.physicalHand == p.physicalHand)
        {
            const auto kind = a.operation.operation.kind;
            if (kind == p.kind || ((kind == HIGGS_OP_WORLD_SUPPORT || kind == HIGGS_OP_PHYSICAL_HAND_TARGET) &&
                                   (p.kind == HIGGS_OP_WORLD_SUPPORT || p.kind == HIGGS_OP_PHYSICAL_HAND_TARGET)))
                return HIGGS_HAND_CONFLICT;
        }
    auto a = NewActionLocked();
    if (!a)
        return HIGGS_HAND_CAPACITY;
    a->request.structSize = sizeof(a->request);
    a->request.requestId = nextToken_++;
    a->request.owner = owner;
    a->request.epoch = p.epoch;
    a->request.physicalHands = p.physicalHand;
    a->request.grant = p.grant ? p.grant : nextToken_++;
    a->selected = true;
    a->grantActive = !p.grant;
    a->operationActive = true;
    a->expires = now + static_cast<Clock>(p.durationMs) * 1000000;
    a->operation.structSize = sizeof(a->operation);
    a->operation.requestId = a->request.requestId;
    a->operation.owner = owner;
    a->operation.grant = a->request.grant;
    a->operation.epoch = p.epoch;
    a->operation.operation = p;
    FinishLocked(*a, HIGGS_ACTION_ACCEPTED, 0);
    RefreshLocked();
    out = a->request.requestId;
    return HIGGS_HAND_OK;
}
bool Broker::OperationActive(uint64_t id, Clock now)
{
    std::lock_guard lock(mutex_);
    ExpireLocked(now);
    auto a = ActionLocked(id);
    if (!a || !a->operationActive || a->request.epoch != snapshot_.epoch || (worldChangePending_ && !a->nativeStarted))
        return false;
    const auto &h = snapshot_.hands[a->operation.operation.physicalHand == HIGGS_HAND_LEFT ? 0 : 1];
    const bool active = h.grant == a->request.grant && h.owner == a->request.owner &&
                        h.generation == a->operation.operation.handGeneration &&
                        (h.observationMask & HIGGS_OBS_TRACKING_VALID);
    if (!active)
    {
        a->operationActive = false;
        a->grantActive = false;
        if (!Terminal(a->result.stage))
            FinishLocked(*a, HIGGS_ACTION_CANCELLED, HIGGS_HAND_STALE);
        RefreshLocked();
    }
    return active;
}
HIGGS_HandStatus Broker::EndOperation(uint64_t owner, uint64_t id, uint64_t epoch)
{
    std::lock_guard serial(dispatch_);
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto a = ActionLocked(id);
    if (!a || !a->operation.operation.kind)
        return HIGGS_HAND_NOT_FOUND;
    if (a->request.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    if (a->request.epoch != epoch || epoch != snapshot_.epoch)
        return HIGGS_HAND_STALE;
    if (a->nativeStarted)
        return HIGGS_HAND_PENDING;
    a->operationActive = false;
    a->grantActive = false;
    if (!Terminal(a->result.stage))
        FinishLocked(*a, HIGGS_ACTION_CANCELLED, 0);
    RefreshLocked();
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus Broker::CopyOperation(uint64_t owner, uint64_t id, HIGGS_HandOperationState &out, Clock now)
{
    const bool active = OperationActive(id, now);
    std::lock_guard lock(mutex_);
    if (!ClientLocked(owner))
        return HIGGS_HAND_INVALID_CLIENT;
    auto a = ActionLocked(id);
    if (!a || !a->operation.operation.kind)
        return HIGGS_HAND_NOT_FOUND;
    if (a->request.owner != owner)
        return HIGGS_HAND_WRONG_OWNER;
    out = a->operation;
    out.flags = (active ? HIGGS_OP_ACTIVE : 0u) | ((a->operation.flags & HIGGS_OP_APPLIED) ? HIGGS_OP_APPLIED : 0u);
    return HIGGS_HAND_OK;
}
bool Broker::CopyActiveOperation(uint32_t hand, uint32_t kind, HIGGS_HandOperationState &out, Clock now)
{
    uint64_t id = 0;
    {
        std::lock_guard lock(mutex_);
        for (const auto &a : actions_)
            if (a.operationActive && a.dispatched &&
                (a.result.stage == HIGGS_ACTION_COMMITTED || a.result.stage == HIGGS_ACTION_COMPLETED) &&
                a.operation.operation.physicalHand == hand && a.operation.operation.kind == kind)
            {
                id = a.request.requestId;
                break;
            }
    }
    if (!id || !OperationActive(id, now))
        return false;
    std::lock_guard lock(mutex_);
    if (auto a = ActionLocked(id))
    {
        out = a->operation;
        return true;
    }
    return false;
}
void Broker::ConfirmOperation(uint64_t id, Clock now)
{
    std::lock_guard lock(mutex_);
    auto a = ActionLocked(id);
    if (!a || !a->operationActive || !a->dispatched || a->result.stage != HIGGS_ACTION_COMMITTED)
        return;
    a->operation.flags |= HIGGS_OP_APPLIED;
    a->operation.observedAtNs = now;
    if (a->operation.operation.kind != HIGGS_OP_HAPTIC)
        a->result.receiptFlags |= HIGGS_RECEIPT_PHYSICAL_CONFIRMED;
    FinishLocked(*a, HIGGS_ACTION_COMPLETED, 0);
}
void Broker::BreakOperation(uint64_t id, uint32_t reason)
{
    std::lock_guard lock(mutex_);
    if (auto a = ActionLocked(id))
    {
        a->operationActive = false;
        a->grantActive = false;
        if (!Terminal(a->result.stage))
            FinishLocked(*a, HIGGS_ACTION_CANCELLED, reason);
        RefreshLocked();
    }
}
void Broker::DrainOperations(OperationDispatch dispatch, void *context, Clock now)
{
    std::lock_guard serial(dispatch_);
    DeferredWorldDrain deferredWorldDrain{*this};
    std::vector<HIGGS_HandOperationState> work;
    {
        std::lock_guard lock(mutex_);
        ExpireLocked(now);
        for (auto &a : actions_)
            if (a.operationActive && !a.dispatched)
            {
                a.dispatched = true;
                work.push_back(a.operation);
            }
    }
    for (auto state : work)
    {
        if (!OperationActive(state.requestId, now))
            continue;
        {
            std::lock_guard lock(mutex_);
            auto a = ActionLocked(state.requestId);
            if (!a || !a->operationActive || contextMenus_)
                continue;
            a->nativeStarted = true;
        }
        bool accepted = false;
        try
        {
            accepted = dispatch && dispatch(state.requestId, state, context);
        }
        catch (...)
        {
        }
        std::lock_guard lock(mutex_);
        if (auto a = ActionLocked(state.requestId))
        {
            a->nativeStarted = false;
            if (a->operationActive)
            {
                if (accepted)
                {
                    a->operation = state;
                    FinishLocked(*a, HIGGS_ACTION_COMMITTED, 0);
                }
                else
                {
                    a->operationActive = false;
                    a->grantActive = false;
                    FinishLocked(*a, HIGGS_ACTION_REJECTED, HIGGS_HAND_NOT_READY);
                    RefreshLocked();
                }
            }
        }
    }
    ApplyDeferredWorld();
    FlushEvents();
}
namespace
{
bool loading = false;
uintptr_t observedWorld = 0;
std::mutex lifecycle;
} // namespace
void ObserveWorld(uintptr_t world, bool)
{
    std::lock_guard lock(lifecycle);
    if(observedWorld!=world)OwnContacts().Invalidate();
    observedWorld = world;
    Service().ObserveWorld(world, loading, Now());
}
void BeginLoad()
{
    std::lock_guard lock(lifecycle);
    OwnContacts().Invalidate();
    loading = true;
    Service().ObserveWorld(0, true, Now());
}
void EndLoad()
{
    std::lock_guard lock(lifecycle);
    OwnContacts().Invalidate();
    loading = false;
    observedWorld = 0;
    Service().ObserveWorld(0, false, Now());
}
void NewGame()
{
    BeginLoad();
    EndLoad();
}
bool Input(uint32_t hand, uint32_t button, bool down)
{
    return Service().Input(hand, button, down, Now());
}
bool HasGrant(bool left)
{
    return Service().HasGrant(left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, Now());
}
bool MayAcquire(bool left)
{
    return Service().MayAcquire(left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, Now());
}
bool MaySupport(bool left)
{
    return Service().MaySupport(left ? HIGGS_HAND_LEFT : HIGGS_HAND_RIGHT, Now());
}
} // namespace HiggsHands
