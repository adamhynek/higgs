#include "hand_broker.h"
#include "higgshandapi.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>
#define CHECK(x)                                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(x))                                                                                                      \
        {                                                                                                              \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x);                                          \
            std::abort();                                                                                              \
        }                                                                                                              \
    } while (0)
using HiggsHands::Broker;
using HiggsHands::Clock;
extern "C" int hand_c_consumer(void);
struct Fixture
{
    Broker broker;
    Clock now = HiggsHands::Now();
    std::array<HIGGS_HandRecord, 2> hands{};
    Fixture()
    {
        for (auto &h : hands)
            h.observationMask =
                HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN | HIGGS_OBS_PALM_VALID | HIGGS_OBS_TRACKING_VALID;
        broker.ObserveWorld(1, false, now);
        broker.Publish(hands, ++now);
    }
    uint64_t Client(const char *name, int priority = 0)
    {
        HIGGS_HandClientInfo info{};
        info.structSize = sizeof(info);
        info.userPriority = priority;
        std::snprintf(info.name, sizeof(info.name), "%s", name);
        uint64_t id = 0;
        CHECK(!broker.Register(info, id));
        return id;
    }
    HIGGS_HandSnapshot Snapshot(uint64_t client)
    {
        HIGGS_HandSnapshot out{};
        CHECK(!broker.Copy(client, out, now));
        return out;
    }
    void Publish()
    {
        broker.Publish(hands, ++now);
    }
    bool Input(uint32_t hand, uint32_t button, bool down)
    {
        return broker.Input(hand, button, down, ++now);
    }
};
struct Consumer
{
    Fixture *fixture;
    uint64_t client = 0, executor = 0, registration = 0, submitted = 0, lastInput = 0;
    bool enabled = true;
    bool offerOnUp = false;
    uint32_t durationMs = 1000;
    HIGGS_HandStatus submitStatus = 999;
    uint32_t required = HIGGS_HAND_LEFT;
    int executions = 0;
    HIGGS_ActionRequest request{};
    Consumer(Fixture &f, const char *name, int priority = 0) : fixture(&f)
    {
        client = f.Client(name, priority);
        CHECK(!f.broker.Executor(client, Execute, this, executor));
        CHECK(!f.broker.Recognizer(client, Recognize, this, registration));
    }
    static void HIGGS_HAND_CALL Recognize(const HIGGS_HandInput *input, void *data)
    {
        auto &c = *static_cast<Consumer *>(data);
        c.lastInput = input->inputId;
        if (!c.enabled || (input->edge != HIGGS_INPUT_DOWN && !c.offerOnUp))
            return;
        HIGGS_ActionCandidate candidate{};
        candidate.structSize = sizeof(candidate);
        candidate.physicalHands = c.required;
        candidate.inputId = input->inputId;
        candidate.epoch = input->epoch;
        candidate.handGeneration = input->handGeneration;
        candidate.executor = c.executor;
        candidate.actionId = 700;
        candidate.contextId = 800;
        candidate.contextGeneration = 900;
        candidate.grantDurationMs = c.durationMs;
        c.submitStatus = c.fixture->broker.Submit(c.client, candidate, c.submitted, c.fixture->now);
    }
    static void HIGGS_HAND_CALL Execute(const HIGGS_ActionRequest *request, void *data)
    {
        auto &c = *static_cast<Consumer *>(data);
        ++c.executions;
        c.request = *request;
    }
    HIGGS_ActionResult Result()
    {
        HIGGS_ActionResult r{};
        CHECK(!fixture->broker.Result(client, submitted, r));
        return r;
    }
};
void Occupancy()
{
    Fixture f;
    auto client = f.Client("reader");
    auto s = f.Snapshot(client);
    CHECK(s.hands[0].occupancy == HIGGS_OCCUPANCY_EMPTY);
    CHECK(s.hands[0].availability == HIGGS_AVAILABILITY_READY);
    const auto generation = s.hands[0].generation;
    f.hands[0].equippedFormId = 10;
    f.Publish();
    s = f.Snapshot(client);
    CHECK(s.hands[0].occupancy == HIGGS_OCCUPANCY_GAME_EQUIPPED);
    CHECK(s.hands[0].generation != generation);
    f.hands[0].heldRefId = 20;
    f.Publish();
    s = f.Snapshot(client);
    CHECK(s.hands[0].occupancy == HIGGS_OCCUPANCY_BOTH);
    CHECK(s.hands[0].availability == HIGGS_AVAILABILITY_RESERVED);
    f.hands[0].equippedFormId = 0;
    f.Publish();
    s = f.Snapshot(client);
    CHECK(s.hands[0].occupancy == HIGGS_OCCUPANCY_HIGGS_HELD);
    f.hands[0].heldRefId = 0;
    f.hands[0].observationMask &= ~HIGGS_OBS_EQUIPMENT_KNOWN;
    f.Publish();
    s = f.Snapshot(client);
    CHECK(s.hands[0].availability == HIGGS_AVAILABILITY_UNKNOWN);
    CHECK(!(s.hands[0].observationMask & HIGGS_OBS_EQUIPMENT_KNOWN));
    CHECK(s.hands[0].poseObservationId == s.observationId);
    std::puts("PASS copied occupancy, unknown observations and pose identity");
}
void Arbitration()
{
    Fixture f;
    Consumer low(f, "low", 1), high(f, "high", 20);
    CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
    CHECK(!low.submitStatus && !high.submitStatus);
    CHECK(low.executions == 0 && high.executions == 1);
    CHECK(low.Result().stage == HIGGS_ACTION_REJECTED);
    CHECK(high.Result().stage == HIGGS_ACTION_ACCEPTED);
    CHECK(high.request.contextId == 800 && high.request.contextGeneration == 900);
    CHECK(high.request.poseObservationId == f.Snapshot(high.client).observationId);
    HIGGS_InputDecision d{};
    CHECK(!f.broker.Decision(low.client, high.lastInput, d));
    CHECK(d.consumed && d.owner == high.client && d.effectivePriority == 20);
    auto s = f.Snapshot(low.client);
    CHECK(s.hands[0].owner == high.client && s.hands[0].grant == high.request.grant);
    CHECK(f.broker.Release(low.client, high.request.grant) == HIGGS_HAND_WRONG_OWNER);
    high.enabled = false;
    low.enabled = false;
    CHECK(!f.broker.Release(high.client, high.request.grant));
    CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
    CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, false));
    CHECK(!f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, false));
    std::puts("PASS synchronous priority arbitration, one consumer and consumed press/up");
}
void InvalidAndBoth()
{
    Fixture f;
    Consumer c(f, "both");
    c.required = HIGGS_HAND_BOTH;
    f.hands[1].observationMask &= ~HIGGS_OBS_TRACKING_VALID;
    f.Publish();
    CHECK(!f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_TRIGGER, true));
    CHECK(c.submitStatus == HIGGS_HAND_CONFLICT);
    CHECK(!f.Snapshot(c.client).hands[0].grant);
    f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_TRIGGER, false);
    f.hands[1].observationMask |= HIGGS_OBS_TRACKING_VALID;
    f.Publish();
    CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_TRIGGER, true));
    auto s = f.Snapshot(c.client);
    CHECK(s.hands[0].grant == s.hands[1].grant && s.hands[0].grant);
    HIGGS_ActionCandidate outside{};
    uint64_t out = 0;
    CHECK(f.broker.Submit(c.client, outside, out, f.now) == HIGGS_HAND_WRONG_PHASE);
    f.broker.ObserveWorld(0, false, ++f.now);
    s = f.Snapshot(c.client);
    CHECK(!s.hands[0].grant && !s.worldReady);
    CHECK(c.Result().stage == HIGGS_ACTION_CANCELLED);
    std::puts("PASS all-or-none hand grants, tracking gate, phase and epoch invalidation");
}
void Receipts()
{
    Fixture f;
    Consumer c(f, "receipt"), other(f, "other", -1);
    CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
    HIGGS_ActionResult r{};
    r.structSize = sizeof(r);
    r.stage = HIGGS_ACTION_COMMITTED;
    r.requestId = c.request.requestId;
    r.executor = c.executor;
    r.epoch = c.request.epoch;
    r.inputId = c.request.inputId;
    r.receiptFlags = HIGGS_RECEIPT_INVENTORY_COMMITTED;
    r.itemRefId = 123;
    r.quantity = 1;
    CHECK(f.broker.Report(other.client, r) == HIGGS_HAND_WRONG_OWNER);
    CHECK(!f.broker.Report(c.client, r));
    CHECK(!f.broker.Report(c.client, r));
    r.stage = HIGGS_ACTION_REJECTED;
    CHECK(f.broker.Report(c.client, r) == HIGGS_HAND_CONFLICT);
    r.stage = HIGGS_ACTION_COMPLETED;
    r.receiptFlags |= HIGGS_RECEIPT_PHYSICAL_CONFIRMED;
    CHECK(!f.broker.Report(c.client, r));
    CHECK(!f.broker.Report(c.client, r));
    r.quantity = 2;
    CHECK(f.broker.Report(c.client, r) == HIGGS_HAND_CONFLICT);
    CHECK(f.broker.CancelRecognized(c.client, c.request.requestId, c.request.epoch, 99) == HIGGS_HAND_CONFLICT);
    std::puts("PASS assigned executor, monotonic stages, duplicate and correlated commitments");
}
void Routing()
{
    Fixture f;
    Consumer recognizer(f, "slot recognizer", 50), provider(f, "provider", 1);
    provider.enabled = false;
    recognizer.executor = provider.executor;
    CHECK(!f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
    CHECK(recognizer.submitStatus == HIGGS_HAND_WRONG_OWNER);
    CHECK(f.broker.Authorize(recognizer.client, provider.executor, recognizer.client, HIGGS_HAND_LEFT) ==
          HIGGS_HAND_WRONG_OWNER);
    CHECK(!f.broker.Authorize(provider.client, provider.executor, recognizer.client, HIGGS_HAND_LEFT));
    CHECK(!f.broker.CanRoute(recognizer.client, provider.executor, HIGGS_HAND_LEFT));
    CHECK(f.broker.CanRoute(recognizer.client, provider.executor, HIGGS_HAND_BOTH) == HIGGS_HAND_WRONG_OWNER);
    f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, false);
    CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
    CHECK(provider.executions == 1 && provider.request.owner == provider.client);
    HIGGS_ActionResult result{};
    CHECK(!f.broker.Result(recognizer.client, recognizer.submitted, result));
    CHECK(f.broker.Release(recognizer.client, provider.request.grant) == HIGGS_HAND_WRONG_OWNER);
    CHECK(!f.broker.CancelRecognized(recognizer.client, recognizer.submitted, provider.request.epoch, 42));
    CHECK(!f.Snapshot(provider.client).hands[0].grant);
    CHECK(!f.broker.Revoke(provider.client, provider.executor, recognizer.client));
    CHECK(f.broker.CanRoute(recognizer.client, provider.executor, HIGGS_HAND_LEFT) == HIGGS_HAND_WRONG_OWNER);
    std::puts("PASS opt-in routed executor authority and recognizer contextual cancellation");
}
bool DispatchAccept(const HIGGS_PhysicalRequest &, void *data)
{
    ++*static_cast<int *>(data);
    return true;
}
bool DispatchReject(const HIGGS_PhysicalRequest &, void *)
{
    return false;
}
void Physical()
{
    Fixture f;
    auto client = f.Client("physical");
    auto s = f.Snapshot(client);
    HIGGS_PhysicalRequest p{};
    p.structSize = sizeof(p);
    p.kind = HIGGS_PHYSICAL_GRAB;
    p.physicalHand = HIGGS_HAND_LEFT;
    p.itemRefId = 77;
    p.epoch = s.epoch;
    p.handGeneration = s.hands[0].generation;
    p.timeoutMs = 1000;
    uint64_t id = 0;
    CHECK(!f.broker.Physical(client, p, id, f.now));
    HIGGS_ActionResult r{};
    CHECK(!f.broker.Result(client, id, r));
    CHECK(r.stage == HIGGS_ACTION_ACCEPTED);
    int calls = 0;
    f.broker.DrainPhysical(DispatchAccept, &calls, ++f.now);
    CHECK(calls == 1);
    CHECK(!f.broker.Result(client, id, r));
    CHECK(r.stage == HIGGS_ACTION_COMMITTED && !(r.receiptFlags & HIGGS_RECEIPT_PHYSICAL_CONFIRMED));
    f.broker.DrainPhysical(DispatchAccept, &calls, ++f.now);
    CHECK(calls == 1);
    f.hands[0].heldRefId = 77;
    f.Publish();
    CHECK(!f.broker.Result(client, id, r));
    CHECK(r.stage == HIGGS_ACTION_COMPLETED && (r.receiptFlags & HIGGS_RECEIPT_PHYSICAL_CONFIRMED));
    s = f.Snapshot(client);
    p.kind = HIGGS_PHYSICAL_RELEASE;
    p.itemRefId = 77;
    p.handGeneration = s.hands[0].generation;
    p.grant = s.hands[0].grant;
    CHECK(!f.broker.Physical(client, p, id, f.now));
    f.broker.DrainPhysical(DispatchAccept, &calls, ++f.now);
    f.hands[0].heldRefId = 0;
    f.Publish();
    CHECK(!f.broker.Result(client, id, r));
    CHECK(r.stage == HIGGS_ACTION_COMPLETED);
    CHECK(!f.broker.Release(client, p.grant));
    s = f.Snapshot(client);
    p.kind = HIGGS_PHYSICAL_GRAB;
    p.itemRefId = 78;
    p.grant = 0;
    p.handGeneration = s.hands[0].generation;
    CHECK(!f.broker.Physical(client, p, id, f.now));
    f.broker.DrainPhysical(DispatchReject, nullptr, ++f.now);
    CHECK(!f.broker.Result(client, id, r));
    CHECK(r.stage == HIGGS_ACTION_REJECTED);
    std::puts("PASS queued physical acceptance, native rejection and observed completion only");
}
struct Mutation
{
    Broker *b;
    uint64_t owner = 0, token = 0;
    int calls = 0;
    HIGGS_HandStatus removed = 999;
};
void HIGGS_HAND_CALL SelfRemove(const HIGGS_HandInput *, void *data)
{
    auto &m = *static_cast<Mutation *>(data);
    ++m.calls;
    m.removed = m.b->Remove(m.owner, m.token);
}
struct Blocked
{
    std::mutex mutex;
    std::condition_variable cv;
    bool entered = false, released = false;
};
void HIGGS_HAND_CALL Block(const HIGGS_HandInput *, void *data)
{
    auto &b = *static_cast<Blocked *>(data);
    std::unique_lock lock(b.mutex);
    b.entered = true;
    b.cv.notify_all();
    b.cv.wait(lock, [&] { return b.released; });
}
void MutationAndDrain()
{
    Fixture f;
    auto owner = f.Client("callback");
    Mutation m{&f.broker, owner};
    CHECK(!f.broker.Recognizer(owner, SelfRemove, &m, m.token));
    f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true);
    CHECK(m.calls == 1 && m.removed == HIGGS_HAND_PENDING);
    CHECK(!f.broker.Remove(owner, m.token));
    f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, false);
    CHECK(m.calls == 1);
    Blocked block;
    uint64_t token = 0;
    CHECK(!f.broker.Recognizer(owner, Block, &block, token));
    std::thread input([&] { f.broker.Input(HIGGS_HAND_RIGHT, HIGGS_INPUT_TRIGGER, true, f.now); });
    {
        std::unique_lock lock(block.mutex);
        block.cv.wait(lock, [&] { return block.entered; });
    }
    std::atomic<bool> started{false}, drained{false};
    std::thread removal([&] {
        started = true;
        CHECK(!f.broker.Remove(owner, token));
        drained = true;
    });
    while (!started.load())
        std::this_thread::yield();
    CHECK(!drained.load());
    {
        std::lock_guard lock(block.mutex);
        block.released = true;
    }
    block.cv.notify_all();
    input.join();
    removal.join();
    CHECK(drained.load());
    CHECK(!f.broker.Unregister(owner));
    std::puts("PASS self-removal tombstones and concurrent confirmed callback drainage");
}
void DeadlineAndReuse()
{
    Fixture f;
    Consumer c(f, "expiry");
    CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
    f.now += UINT64_C(1000000001);
    auto s = f.Snapshot(c.client);
    CHECK(!s.hands[0].grant);
    CHECK(c.Result().stage == HIGGS_ACTION_CANCELLED);
    CHECK(!f.broker.Unregister(c.client));
    auto replacement = f.Client("replacement");
    CHECK(replacement != c.client);
    CHECK(f.broker.Release(replacement, c.request.grant) == HIGGS_HAND_WRONG_OWNER);
    std::puts("PASS deadlines and client identity non-reuse");
}

void QueuedReceiptAcknowledgement()
{
    // Exercise the real bounded history: an exact ACCEPTED echo retains the
    // outcome, without retaining a hand or renewing its finite lease.
    for (bool retain : {false, true})
    {
        auto storage = std::make_unique<Fixture>();
        auto &f = *storage;
        Consumer queued(f, "queued"), churn(f, "churn");
        churn.enabled = false;
        queued.durationMs = 60000;
        CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
        const auto request = queued.request;
        const auto accepted = queued.Result();
        CHECK(accepted.stage == HIGGS_ACTION_ACCEPTED && !accepted.receiptFlags);
        CHECK(f.broker.Report(churn.client, accepted) == HIGGS_HAND_WRONG_OWNER);
        auto altered = accepted;
        altered.reason = 123;
        CHECK(f.broker.Report(queued.client, altered) == HIGGS_HAND_CONFLICT);
        CHECK(queued.Result().stage == HIGGS_ACTION_ACCEPTED);
        if (retain)
        {
            CHECK(!f.broker.Report(queued.client, accepted));
            CHECK(!f.broker.Report(queued.client, accepted));
        }

        queued.enabled = false;
        f.now += UINT64_C(60000000001);
        const auto expired = f.Snapshot(queued.client);
        CHECK(!expired.hands[0].grant && !expired.hands[0].owner);
        HIGGS_ActionResult cancelled{};
        CHECK(!f.broker.Result(queued.client, request.requestId, cancelled));
        CHECK(cancelled.stage == HIGGS_ACTION_CANCELLED);
        CHECK(f.broker.Report(queued.client, accepted) == HIGGS_HAND_WRONG_PHASE);
        CHECK(!f.broker.Report(queued.client, cancelled));
        CHECK(!f.broker.Report(queued.client, cancelled));

        HIGGS_PhysicalRequest physical{};
        physical.structSize = sizeof(physical);
        physical.kind = HIGGS_PHYSICAL_GRAB;
        physical.physicalHand = HIGGS_HAND_LEFT;
        physical.itemRefId = 77;
        physical.epoch = request.epoch;
        physical.handGeneration = expired.hands[0].generation;
        physical.grant = request.grant;
        physical.timeoutMs = 1000;
        uint64_t unchanged = 900;
        CHECK(f.broker.Physical(queued.client, physical, unchanged, f.now) == HIGGS_HAND_WRONG_OWNER);
        CHECK(unchanged == 900);
        f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, false);

        const auto churnHistory = [&] {
            churn.enabled = true;
            for (unsigned i = 0; i < 300; ++i)
            {
                f.Publish();
                CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
                CHECK(churn.submitStatus == HIGGS_HAND_OK);
                CHECK(!f.broker.Release(churn.client, churn.request.grant));
                f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, false);
            }
            churn.enabled = false;
        };
        churnHistory();
        HIGGS_ActionResult copied{};
        if (retain)
        {
            CHECK(!f.broker.Result(queued.client, request.requestId, copied));
            CHECK(!std::memcmp(&copied, &cancelled, sizeof(copied)));
            CHECK(f.broker.Release(churn.client, request.grant) == HIGGS_HAND_WRONG_OWNER);
            CHECK(!f.broker.Release(queued.client, request.grant));
            churnHistory();
        }
        CHECK(f.broker.Result(queued.client, request.requestId, copied) == HIGGS_HAND_NOT_FOUND);
    }
    std::puts("PASS exact queued receipt opt-in, finite authority and explicit consumption acknowledgement");
}
bool OperationAccept(uint64_t, HIGGS_HandOperationState &state, void *data)
{
    ++*static_cast<int *>(data);
    state.supportGeneration = 123;
    state.supportPosition[2] = 4;
    return true;
}
bool OperationReject(uint64_t, HIGGS_HandOperationState &, void *)
{
    return false;
}
void Operations()
{
    Fixture f;
    auto owner = f.Client("support"), other = f.Client("other");
    auto s = f.Snapshot(owner);
    HIGGS_HandOperation op{};
    op.structSize = sizeof(op);
    op.kind = HIGGS_OP_WORLD_SUPPORT;
    op.physicalHand = HIGGS_HAND_LEFT;
    op.itemRefId = 901;
    op.epoch = s.epoch;
    op.handGeneration = s.hands[0].generation;
    op.durationMs = 1000;
    op.maxDistance = 20;
    uint64_t id = 0;
    CHECK(!f.broker.Operation(owner, op, id, f.now));
    HIGGS_HandOperationState state{};
    CHECK(!f.broker.CopyOperation(owner, id, state, f.now));
    CHECK((state.flags & HIGGS_OP_ACTIVE) && !(state.flags & HIGGS_OP_APPLIED));
    int calls = 0;
    f.broker.DrainOperations(OperationAccept, &calls, ++f.now);
    CHECK(calls == 1);
    HIGGS_ActionResult result{};
    CHECK(!f.broker.Result(owner, id, result));
    CHECK(result.stage == HIGGS_ACTION_COMMITTED);
    CHECK(!f.broker.CopyOperation(owner, id, state, f.now));
    CHECK(state.supportGeneration == 123 && state.supportPosition[2] == 4);
    f.broker.ConfirmOperation(id, ++f.now);
    CHECK(!f.broker.Result(owner, id, result));
    CHECK(result.stage == HIGGS_ACTION_COMPLETED && (result.receiptFlags & HIGGS_RECEIPT_PHYSICAL_CONFIRMED));
    CHECK(f.broker.EndOperation(other, id, op.epoch) == HIGGS_HAND_WRONG_OWNER);
    CHECK(!f.broker.EndOperation(owner, id, op.epoch));
    CHECK(!f.broker.OperationActive(id, f.now));
    CHECK(!f.Snapshot(owner).hands[0].grant);
    op.kind = HIGGS_OP_PHYSICAL_HAND_TARGET;
    op.scale = 1;
    op.rotation[0] = op.rotation[4] = op.rotation[8] = 1;
    CHECK(!f.broker.Operation(owner, op, id, f.now));
    f.broker.DrainOperations(OperationReject, nullptr, ++f.now);
    CHECK(!f.broker.Result(owner, id, result));
    CHECK(result.stage == HIGGS_ACTION_REJECTED);
    op.rotation[0] = -1;
    CHECK(f.broker.Operation(owner, op, id, f.now) == HIGGS_HAND_INVALID_ARGUMENT);
    op.rotation[0] = 1;
    CHECK(!f.broker.Operation(owner, op, id, f.now));
    f.hands[0].heldRefId = 44;
    f.Publish();
    CHECK(!f.broker.OperationActive(id, f.now));
    CHECK(!f.Snapshot(owner).hands[0].grant);
    s = f.Snapshot(owner);
    op.kind = HIGGS_OP_DRIVE_CAP;
    op.handGeneration = s.hands[0].generation;
    op.itemRefId = 44;
    op.linearForceCap = 10;
    op.angularForceCap = 5;
    CHECK(!f.broker.Operation(owner, op, id, f.now));
    f.broker.DrainOperations(OperationAccept, &calls, ++f.now);
    CHECK(f.broker.CopyActiveOperation(HIGGS_HAND_LEFT, HIGGS_OP_DRIVE_CAP, state, f.now));
    CHECK(state.operation.linearForceCap == 10);
    f.now += UINT64_C(1000000001);
    CHECK(!f.broker.OperationActive(id, f.now));
    op.kind = HIGGS_OP_HAPTIC;
    op.hapticStrength = 0.1f;
    CHECK(!f.broker.Operation(owner, op, id, f.now));
    f.broker.DrainOperations(OperationAccept, &calls, ++f.now);
    f.broker.ConfirmOperation(id, ++f.now);
    CHECK(!f.broker.Result(owner, id, result));
    CHECK(!(result.receiptFlags & HIGGS_RECEIPT_PHYSICAL_CONFIRMED));
    f.broker.ObserveWorld(0, false, ++f.now);
    CHECK(!f.broker.OperationActive(id, f.now));
    std::puts("PASS closed operation catalogue, native validation boundary, applied receipt, limits and lifecycle");
}
void Abi()
{
    CHECK(hand_c_consumer() == 0);
    HIGGS_HandInterface api{};
    CHECK(!HIGGS_QueryHandInterface(2, sizeof(api), &api));
    CHECK(api.abiVersion == 2 && api.capabilities & HIGGS_HAND_CAP_INPUT_BROKER);
    HIGGS_HandRoutingInterface routing{};
    CHECK(!HIGGS_QueryHandRoutingInterface(2, sizeof(routing), &routing));
    auto untouched = api;
    CHECK(HIGGS_QueryHandInterface(1, sizeof(api), &api) == HIGGS_HAND_UNSUPPORTED_VERSION);
    CHECK(!std::memcmp(&untouched, &api, sizeof(api)));
    HIGGS_HandClientInfo info{};
    info.structSize = sizeof(info);
    std::snprintf(info.name, sizeof(info.name), "abi");
    uint64_t client = 0;
    CHECK(!api.RegisterClient(&info, &client));
    struct Extended
    {
        HIGGS_HandSnapshot value;
        uint64_t tail;
    } out{};
    out.value.structSize = sizeof(out);
    out.tail = 0xabcdef;
    CHECK(!api.CopySnapshot(client, &out.value));
    CHECK(out.tail == 0xabcdef);
    out.value.structSize = 1;
    CHECK(api.CopySnapshot(client, &out.value) == HIGGS_HAND_BUFFER_TOO_SMALL);
    CHECK(out.value.structSize == sizeof(HIGGS_HandSnapshot));
    CHECK(!api.UnregisterClient(client));
    std::puts("PASS copied C ABI negotiation, extension tails and additive routing table");
}

struct CancelQueuedContext
{
    Broker *broker;
    uint64_t owner, grant;
    int calls = 0;
};
bool CancelSecondPhysical(const HIGGS_PhysicalRequest &, void *data)
{
    auto &c = *static_cast<CancelQueuedContext *>(data);
    ++c.calls;
    if (c.calls == 1)
        CHECK(!c.broker->Release(c.owner, c.grant));
    return true;
}
bool CancelSecondOperation(uint64_t, HIGGS_HandOperationState &, void *data)
{
    auto &c = *static_cast<CancelQueuedContext *>(data);
    ++c.calls;
    if (c.calls == 1)
        CHECK(!c.broker->Release(c.owner, c.grant));
    return true;
}
void AdversarialBoundaries()
{
    {
        Fixture f;
        auto owner = f.Client("native phase");
        auto s = f.Snapshot(owner);
        HIGGS_PhysicalRequest p{};
        p.structSize = sizeof(p);
        p.kind = HIGGS_PHYSICAL_GRAB;
        p.physicalHand = HIGGS_HAND_LEFT;
        p.itemRefId = 55;
        p.epoch = s.epoch;
        p.handGeneration = s.hands[0].generation;
        p.timeoutMs = 1000;
        uint64_t id = 0;
        CHECK(!f.broker.Physical(owner, p, id, f.now));
        int calls = 0;
        f.broker.DrainPhysical(DispatchAccept, &calls, f.now);
        s = f.Snapshot(owner);
        auto phase = f.broker.NativePhase();
        f.broker.BeginHandMutation(f.now);
        CHECK(f.broker.Release(owner, s.hands[0].grant) == HIGGS_HAND_PENDING);
        f.broker.EndHandMutation(HIGGS_HAND_LEFT, 55, true, false, f.now);
        HIGGS_ActionResult r{};
        CHECK(!f.broker.Result(owner, id, r));
        CHECK(r.stage == HIGGS_ACTION_COMPLETED && (r.receiptFlags & HIGGS_RECEIPT_PHYSICAL_CONFIRMED));
        CHECK(!f.broker.Release(owner, s.hands[0].grant));
    }
    {
        Fixture f;
        Consumer c(f, "up");
        c.offerOnUp = true;
        CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
        CHECK(!f.broker.Release(c.client, c.request.grant));
        CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, false));
        CHECK(c.executions == 1 && c.submitStatus == HIGGS_HAND_CONFLICT);
    }
    {
        Fixture f;
        Consumer c(f, "late");
        c.durationMs = 1;
        CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        HIGGS_ActionResult r{};
        r.structSize = sizeof(r);
        r.stage = HIGGS_ACTION_COMMITTED;
        r.requestId = c.request.requestId;
        r.executor = c.executor;
        r.epoch = c.request.epoch;
        r.inputId = c.request.inputId;
        CHECK(f.broker.Report(c.client, r) == HIGGS_HAND_CONFLICT);
    }
    {
        Fixture f;
        auto a = f.Client("first"), b = f.Client("second");
        auto s = f.Snapshot(a);
        HIGGS_PhysicalRequest p{};
        p.structSize = sizeof(p);
        p.kind = HIGGS_PHYSICAL_GRAB;
        p.physicalHand = HIGGS_HAND_LEFT;
        p.itemRefId = 11;
        p.epoch = s.epoch;
        p.handGeneration = s.hands[0].generation;
        p.timeoutMs = 1000;
        uint64_t first = 0, second = 0;
        CHECK(!f.broker.Physical(a, p, first, f.now));
        p.physicalHand = HIGGS_HAND_RIGHT;
        p.itemRefId = 12;
        p.handGeneration = s.hands[1].generation;
        CHECK(!f.broker.Physical(b, p, second, f.now));
        s = f.Snapshot(b);
        CancelQueuedContext context{&f.broker, b, s.hands[1].grant};
        f.broker.DrainPhysical(CancelSecondPhysical, &context, f.now);
        CHECK(context.calls == 1);
        HIGGS_ActionResult r{};
        CHECK(!f.broker.Result(b, second, r));
        CHECK(r.stage == HIGGS_ACTION_CANCELLED);
    }
    {
        Fixture f;
        auto a = f.Client("op first"), b = f.Client("op second");
        auto s = f.Snapshot(a);
        HIGGS_HandOperation p{};
        p.structSize = sizeof(p);
        p.kind = HIGGS_OP_HAPTIC;
        p.hapticStrength = 0.1f;
        p.physicalHand = HIGGS_HAND_LEFT;
        p.epoch = s.epoch;
        p.handGeneration = s.hands[0].generation;
        p.durationMs = 1000;
        uint64_t first = 0, second = 0;
        CHECK(!f.broker.Operation(a, p, first, f.now));
        p.physicalHand = HIGGS_HAND_RIGHT;
        p.handGeneration = s.hands[1].generation;
        CHECK(!f.broker.Operation(b, p, second, f.now));
        s = f.Snapshot(b);
        CancelQueuedContext context{&f.broker, b, s.hands[1].grant};
        f.broker.DrainOperations(CancelSecondOperation, &context, f.now);
        CHECK(context.calls == 1);
    }
    std::puts("PASS adversarial consumed Up, producer-clock late commitment and per-work native revocation");
}
void UserPriorities()
{
    Fixture f;
    Consumer low(f, "low", 1), high(f, "high", 20);
    CHECK(f.broker.ConfigurePriorities("LOW=100,high=-5"));
    CHECK(!f.broker.ConfigurePriorities("low=bad"));
    CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
    CHECK(low.executions == 1 && high.executions == 0);
    HIGGS_InputDecision d{};
    CHECK(!f.broker.Decision(low.client, low.lastInput, d));
    CHECK(d.effectivePriority == 100 && (d.reason & HIGGS_DECISION_USER_PRIORITY));
    CHECK(!f.broker.Release(low.client, low.request.grant));
    f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, false);
    CHECK(f.broker.ConfigurePriorities(""));
    CHECK(f.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
    CHECK(high.executions == 1);
    for (bool reverse : {false, true}) {
        Fixture tied;
        Consumer first(tied, reverse ? "alpha" : "zeta"), second(tied, reverse ? "zeta" : "alpha");
        auto &winner = reverse ? first : second;
        CHECK(tied.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
        CHECK(winner.executions == 1 && first.executions + second.executions == 1);
        CHECK(!tied.broker.Decision(winner.client, winner.lastInput, d));
        CHECK((d.reason & HIGGS_DECISION_STABLE_ID) && !(d.reason & HIGGS_DECISION_REGISTRATION_ORDER));
        HIGGS_HandClientInfo duplicate{sizeof(duplicate), 0, " ALPHA "};
        uint64_t unchanged = 42;
        CHECK(tied.broker.Register(duplicate, unchanged) == HIGGS_HAND_CONFLICT && unchanged == 42);
    }
    Fixture routed;
    Consumer recognizer(routed, "recognizer", 1000), provider(routed, "provider", 1),
        competitor(routed, "competitor", 10);
    provider.enabled = false;
    recognizer.executor = provider.executor;
    CHECK(!routed.broker.Authorize(provider.client, provider.executor, recognizer.client, HIGGS_HAND_LEFT));
    CHECK(routed.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
    CHECK(competitor.executions == 1 && provider.executions == 0);
    CHECK(!routed.broker.Release(competitor.client, competitor.request.grant));
    routed.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, false);
    CHECK(routed.broker.ConfigurePriorities("provider=100,recognizer=-1000"));
    CHECK(routed.Input(HIGGS_HAND_LEFT, HIGGS_INPUT_GRIP, true));
    CHECK(provider.executions == 1);
    std::puts("PASS end-user priority override, invalid-map atomicity and routed provider priority");
}

int main()
{
    Occupancy();
    Arbitration();
    InvalidAndBoth();
    Receipts();
    Routing();
    Physical();
    MutationAndDrain();
    DeadlineAndReuse();
    QueuedReceiptAcknowledgement();
    Operations();
    AdversarialBoundaries();
    UserPriorities();
    Abi();
    std::puts("All hand broker production-core checks passed (no engine/game qualification).");
}
