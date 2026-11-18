#include "hand_broker.h"
#include "native_regions.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
using namespace HiggsHands;
static unsigned checks = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #condition); std::abort(); } } while (0)
struct Fixture {
    Broker broker;
    uint64_t owner = 0, other = 0, registration = 0;
    unsigned calls = 0;
    enum Mode { Outside, Inside, NotReady, BadCorrelation, BadMask, BadSize, Throw, RemoveSelf,
                UnregisterSelf, ResetWorld, Recursive, Block } mode = Outside;
    HIGGS_HandStatus nestedStatus = HIGGS_HAND_INTERNAL_ERROR;
    std::mutex mutex;
    std::condition_variable changed;
    bool entered = false, resume = false;
    Fixture() {
        HIGGS_HandClientInfo info{}; info.structSize = sizeof(info);
        std::snprintf(info.name,sizeof(info.name),"detector"); CHECK(!broker.Register(info, owner));
        std::snprintf(info.name,sizeof(info.name),"other"); CHECK(!broker.Register(info, other));
        broker.ObserveWorld(1, false, Now());
        std::array<HIGGS_HandRecord,2> hands{};
        for (auto& hand : hands) hand.observationMask = HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN |
            HIGGS_OBS_PALM_VALID | HIGGS_OBS_TRACKING_VALID;
        hands[0].heldRefId = 42;
        broker.Publish(hands, Now());
        broker.MarkMainThread();
    }
    HIGGS_NativeRegionFrame Frame() const {
        HIGGS_NativeRegionFrame frame{}; frame.structSize = sizeof(frame); frame.physicalHand = HIGGS_HAND_LEFT;
        frame.flags = HIGGS_REGION_SOURCE_VALID; frame.sourceGeneration = 1; frame.hmdScale = 1;
        frame.hmdRotation[0] = frame.hmdRotation[4] = frame.hmdRotation[8] = 1;
        frame.handPosition[0] = 10;
        for (unsigned i = 0; i < 3; ++i) { frame.spheres[i].worldCenter[0] = float(i * 10); frame.spheres[i].worldRadius = 2; }
        return frame;
    }
    void Register() { CHECK(!broker.NativeRegionDetector(owner, Detect, this, registration)); }
    static HIGGS_HandStatus HIGGS_HAND_CALL Detect(const HIGGS_NativeRegionFrame* frame, HIGGS_NativeRegionResult* out, void* context) {
        auto& f = *static_cast<Fixture*>(context); ++f.calls;
        out->structSize = sizeof(*out); out->epoch = frame->epoch; out->sampleId = frame->sampleId;
        out->insideMask = f.mode == Inside ? HIGGS_REGION_LEFT_SHOULDER : 0;
        switch (f.mode) {
        case NotReady: return HIGGS_HAND_NOT_READY;
        case BadCorrelation: ++out->sampleId; break;
        case BadMask: out->insideMask = 8; break;
        case BadSize: --out->structSize; break;
        case Throw: throw std::runtime_error("synthetic provider failure");
        case RemoveSelf: f.nestedStatus = f.broker.RemoveNativeRegionDetector(f.owner, f.registration); break;
        case UnregisterSelf: f.nestedStatus = f.broker.Unregister(f.owner); break;
        case ResetWorld: f.broker.ObserveWorld(0, false, Now()); break;
        case Recursive: { auto nested = f.Frame(); bool used = true; const auto mask = f.broker.EvaluateNativeRegions(nested, &used);
            if (mask || used || nested.sampleId) std::abort(); break; }
        case Block: { std::unique_lock lock(f.mutex); f.entered = true; f.changed.notify_all(); f.changed.wait(lock,[&]{return f.resume;}); break; }
        default: break;
        }
        return HIGGS_HAND_OK;
    }
};
static void Detection() {
    Fixture f; auto frame = f.Frame(); bool used = true;
    CHECK(f.broker.EvaluateNativeRegions(frame, &used) == 3 && !used);
    CHECK(frame.sampleId && frame.epoch && frame.sourceGeneration == 1 && frame.observedAtNs);
    HIGGS_NativeRegionFrame copied{}; CHECK(!f.broker.CopyNativeRegionFrame(f.owner, HIGGS_HAND_LEFT, copied));
    CHECK(copied.sampleId == frame.sampleId);
    CHECK(f.broker.CopyNativeRegionFrame(f.owner, 3, copied) == HIGGS_HAND_INVALID_ARGUMENT);
    CHECK(f.broker.CopyNativeRegionFrame(999, HIGGS_HAND_LEFT, copied) == HIGGS_HAND_INVALID_CLIENT);
    f.Register(); uint64_t duplicate = 0;
    CHECK(f.broker.NativeRegionDetector(f.other, Fixture::Detect, &f, duplicate) == HIGGS_HAND_CONFLICT);
    CHECK(f.broker.RemoveNativeRegionDetector(f.other, f.registration) == HIGGS_HAND_WRONG_OWNER);
    frame = f.Frame(); CHECK(f.broker.EvaluateNativeRegions(frame, &used) == 0 && used && f.calls == 1);
    f.mode = Fixture::Inside; frame = f.Frame(); CHECK(f.broker.EvaluateNativeRegions(frame, &used) == 4 && used && f.calls == 2);
    for (auto mode : {Fixture::NotReady, Fixture::BadCorrelation, Fixture::BadMask, Fixture::BadSize, Fixture::Throw}) {
        f.mode = mode; frame = f.Frame(); CHECK(f.broker.EvaluateNativeRegions(frame, &used) == 3 && !used);
    }
    const auto calls = f.calls;
    frame = f.Frame(); frame.palmPosition[0] = std::numeric_limits<float>::quiet_NaN();
    CHECK(f.broker.EvaluateNativeRegions(frame, &used) == 0 && !used && f.calls == calls);
    CHECK(f.broker.CopyNativeRegionFrame(f.owner, HIGGS_HAND_LEFT, copied) == HIGGS_HAND_NOT_READY);
    frame = f.Frame(); frame.flags = 0; CHECK(f.broker.EvaluateNativeRegions(frame, &used) == 0 && f.calls == calls);
    frame = f.Frame(); frame.physicalHand = 3; CHECK(f.broker.EvaluateNativeRegions(frame, &used) == 0 && f.calls == calls);
    f.mode = Fixture::Recursive; frame = f.Frame(); CHECK(f.broker.EvaluateNativeRegions(frame, &used) == 0 && used);
    CHECK(!f.broker.RemoveNativeRegionDetector(f.owner, f.registration));
    frame = f.Frame(); CHECK(f.broker.EvaluateNativeRegions(frame, &used) == 3 && !used);
    frame = f.Frame(); frame.palmPosition[0] = 2; CHECK(NativeRegionFallbackMask(frame) == 2); // Exact mouth boundary is outside.
    frame.spheres[1].worldRadius = 0; CHECK(NativeRegionFallbackMask(frame) == 0);
    frame.spheres[2].worldRadius = -1; CHECK(NativeRegionFallbackMask(frame) == 0);
}
static void Lifecycle() {
    for (auto mode : {Fixture::RemoveSelf, Fixture::UnregisterSelf}) {
        Fixture f; f.Register(); f.mode = mode; auto frame = f.Frame();
        CHECK(f.broker.EvaluateNativeRegions(frame) == 3);
        CHECK(f.nestedStatus == HIGGS_HAND_PENDING);
        if (mode == Fixture::RemoveSelf) CHECK(!f.broker.RemoveNativeRegionDetector(f.owner, f.registration));
        else CHECK(!f.broker.Unregister(f.owner));
    }
    Fixture f; f.Register(); f.mode = Fixture::ResetWorld; auto frame = f.Frame();
    const auto oldEpoch = f.broker.Epoch(); CHECK(f.broker.EvaluateNativeRegions(frame) == 0);
    CHECK(f.broker.Epoch() != oldEpoch);
    HIGGS_NativeRegionFrame copied{};
    CHECK(f.broker.CopyNativeRegionFrame(f.owner, HIGGS_HAND_LEFT, copied) == HIGGS_HAND_NOT_READY);
    const auto calls = f.calls; frame = f.Frame(); CHECK(f.broker.EvaluateNativeRegions(frame) == 0 && f.calls == calls);
    f.broker.ObserveWorld(1, false, Now()); f.mode = Fixture::Outside;
    frame = f.Frame(); bool used = false; CHECK(f.broker.EvaluateNativeRegions(frame,&used) == 0 && used); // Registration survives reset.
}
static void Drain(bool unregister) {
    Fixture f; f.Register(); f.mode = Fixture::Block;
    std::thread update([&]{auto frame=f.Frame(); f.broker.EvaluateNativeRegions(frame);});
    { std::unique_lock lock(f.mutex); CHECK(f.changed.wait_for(lock,std::chrono::seconds(5),[&]{return f.entered;})); }
    std::atomic<bool> started{false}, finished{false}; HIGGS_HandStatus result=HIGGS_HAND_INTERNAL_ERROR;
    std::thread remove([&]{started=true; result=unregister?f.broker.Unregister(f.owner):f.broker.RemoveNativeRegionDetector(f.owner,f.registration); finished=true; f.changed.notify_all();});
    while (!started) std::this_thread::yield();
    { std::unique_lock lock(f.mutex); CHECK(!f.changed.wait_for(lock,std::chrono::milliseconds(20),[&]{return finished.load();}));
      f.resume=true; f.changed.notify_all(); }
    update.join(); remove.join(); CHECK(finished && result == HIGGS_HAND_OK);
    const auto calls=f.calls; auto frame=f.Frame(); CHECK(f.broker.EvaluateNativeRegions(frame)==3 && f.calls==calls);
}
static void HIGGS_HAND_CALL Execute(const HIGGS_ActionRequest*,void*) {}
static void GrantSerialization(bool operation) {
    Fixture f; auto frame=f.Frame(); f.broker.EvaluateNativeRegions(frame);
    auto phase=f.broker.NativePhase();
    const auto snapshot=f.broker.CopyPublished();
    std::atomic<bool> started{false},finished{false};
    std::mutex mutex; std::condition_variable changed;
    HIGGS_HandStatus status=HIGGS_HAND_INTERNAL_ERROR; uint64_t id=0;
    std::thread request([&] {
        started=true;
        if (operation) {
            HIGGS_HandOperation r{}; r.structSize=sizeof(r); r.physicalHand=HIGGS_HAND_LEFT;
            r.kind=HIGGS_OP_DRIVE_CAP; r.epoch=snapshot.epoch; r.handGeneration=snapshot.hands[0].generation;
            r.itemRefId=42; r.durationMs=1000;
            status=f.broker.Operation(f.other,r,id,Now());
        } else {
            HIGGS_PhysicalRequest r{}; r.structSize=sizeof(r); r.physicalHand=HIGGS_HAND_LEFT;
            r.kind=HIGGS_PHYSICAL_RELEASE; r.epoch=snapshot.epoch; r.handGeneration=snapshot.hands[0].generation;
            r.itemRefId=42; r.timeoutMs=1000;
            status=f.broker.Physical(f.other,r,id,Now());
        }
        finished=true; changed.notify_all();
    });
    while (!started) std::this_thread::yield();
    { std::unique_lock lock(mutex); CHECK(!changed.wait_for(lock,std::chrono::milliseconds(20),[&]{return finished.load();})); }
    CHECK(f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now()));
    phase.unlock(); request.join();
    CHECK(finished && status==HIGGS_HAND_OK && id);
    CHECK(!f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now()));
}
static void Fence() {
    Fixture f; auto frame=f.Frame(); CHECK(f.broker.EvaluateNativeRegions(frame)==3);
    CHECK(f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now()));
    uint64_t executor=0; CHECK(!f.broker.Executor(f.other,Execute,nullptr,executor));
    HIGGS_HandUseRequest request{}; request.structSize=sizeof(request); request.kind=HIGGS_USE_HAND_GOALS;
    request.physicalHands=HIGGS_HAND_LEFT; request.durationMs=1000; request.epoch=frame.epoch; request.executor=executor;
    HIGGS_HandUseState state{}; CHECK(!f.broker.HandUse(f.other,request,state,Now()));
    CHECK(!f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now()));
    CHECK(!f.broker.EndHandUse(f.other,state.authority.requestId,frame.epoch));
    CHECK(f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now()));
    f.broker.SetInputContext(1,Now()); CHECK(!f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now()));
    f.broker.SetInputContext(0,Now()); CHECK(!f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now()));
    frame=f.Frame(); f.broker.EvaluateNativeRegions(frame); CHECK(f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now()));
    f.broker.ObserveWorld(2,false,Now()); CHECK(!f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now()));
    frame=f.Frame(); f.broker.EvaluateNativeRegions(frame); std::array<HIGGS_HandRecord,2> hands{};
    const auto snapshot=f.broker.CopyPublished(); hands[0]=snapshot.hands[0]; hands[1]=snapshot.hands[1];
    CHECK(!f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now())); // Unknown post-reset hands.
    hands[0].unavailableReasons=HIGGS_HAND_REASON_RESTRICTION; f.broker.Publish(hands,Now());
    CHECK(!f.broker.MayNativeRegionAction(HIGGS_HAND_LEFT,frame.epoch,Now()));
}
int main() { Detection(); Lifecycle(); Drain(false); Drain(true); Fence(); GrantSerialization(false); GrantSerialization(true); std::printf("PASS native region detector: %u checks\n",checks); }
