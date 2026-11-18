#include "hand_broker.h"
#include "haptic_schedule.h"
#include "hand_physical_policy.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>
#include <cstring>
#define CHECK(x)                                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(x))                                                                                                      \
        {                                                                                                              \
            std::fprintf(stderr, "FAIL %d %s\n", __LINE__, #x);                                                        \
            std::abort();                                                                                              \
        }                                                                                                              \
    } while (0)
using namespace HiggsHands;
struct Fixture
{
    Broker broker;
    HapticSchedule haptics{broker};
    uint64_t owner = 0, other = 0, executor = 0;
    int notifications = 0;
    HIGGS_ActionRequest notification{};
    uint64_t now = Now();
    std::array<HIGGS_HandRecord, 2> hands{};
    Fixture()
    {
        HIGGS_HandClientInfo c{};
        c.structSize = sizeof(c);
        std::snprintf(c.name, sizeof(c.name), "primary");
        c.userPriority = 10;
        CHECK(!broker.Register(c, owner));
        std::snprintf(c.name, sizeof(c.name), "other");
        c.userPriority = 1;
        CHECK(!broker.Register(c, other));
        CHECK(!broker.Executor(owner, Notify, this, executor));
        broker.ObserveWorld(1, false, now);
        for (auto &h : hands)
            h.observationMask =
                HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN | HIGGS_OBS_PALM_VALID | HIGGS_OBS_TRACKING_VALID;
        broker.Publish(hands, ++now);
        broker.MarkMainThread();
    }
    static void HIGGS_HAND_CALL Notify(const HIGGS_ActionRequest *r, void *context)
    {
        auto &f = *static_cast<Fixture *>(context);
        ++f.notifications;
        f.notification = *r;
    }
    HIGGS_HandUseState Use(uint32_t duration = 1000, uint32_t handsMask = HIGGS_HAND_BOTH)
    {
        HIGGS_HandUseRequest r{};
        r.structSize = sizeof(r);
        r.kind = HIGGS_USE_HAND_GOALS;
        r.physicalHands = handsMask;
        r.durationMs = duration;
        r.epoch = broker.Epoch();
        r.executor = executor;
        r.contextId = 900;
        r.contextGeneration = 2;
        HIGGS_HandUseState s{};
        CHECK(!broker.HandUse(owner, r, s, now));
        return s;
    }
    HIGGS_HapticRequest Pulse(uint64_t context = 0, float strength = .3f)
    {
        HIGGS_HapticRequest r{};
        r.structSize = sizeof(r);
        r.physicalHand = HIGGS_HAND_LEFT;
        r.epoch = broker.Epoch();
        r.contextKey = context;
        r.durationMs = 100;
        r.startStrength = r.endStrength = strength;
        return r;
    }
};
void SessionOwnership()
{
    auto f = std::make_unique<Fixture>();
    auto s = f->Use();
    CHECK(!f->notifications);
    f->broker.DrainHandUses();
    CHECK(f->notifications == 1 && f->notification.inputId == 0 && f->notification.physicalHands == HIGGS_HAND_BOTH);
    f->broker.DrainHandUses();
    CHECK(f->notifications == 1);
    HIGGS_HandSnapshot snapshot{};
    CHECK(!f->broker.Copy(f->owner, snapshot, f->now));
    CHECK(snapshot.hands[0].grant == snapshot.hands[1].grant && snapshot.hands[0].owner == f->owner);
    HIGGS_HandUseState renewed{};
    CHECK(f->broker.ReadHandUse(f->other, s.authority.requestId, s.epoch, 1000, renewed, f->now) ==
          HIGGS_HAND_WRONG_OWNER);
    CHECK(!f->broker.ReadHandUse(f->owner, s.authority.requestId, s.epoch, 1000, renewed, f->now + 100));
    CHECK(!f->broker.EndHandUse(f->owner, s.authority.requestId, s.epoch));
    CHECK(f->broker.ReadHandUse(f->owner, s.authority.requestId, s.epoch, 1000, renewed, f->now + 101) ==
          HIGGS_HAND_STALE);
    std::puts("PASS external multi-hand use without fake input, owned renewal and end");
}
HIGGS_HandUseTransitionRequest Transition(Fixture &f,const HIGGS_HandUseState &s,uint64_t revision,uint32_t hands)
{
    const auto snapshot=f.broker.CopyPublished();
    HIGGS_HandUseTransitionRequest r{};
    r.structSize=sizeof(r);r.physicalHands=hands;r.durationMs=1000;r.epoch=s.epoch;
    r.requestId=s.authority.requestId;r.expectedRevision=revision;
    for (unsigned i=0;i<2;++i) r.expectedHandGenerations[i]=snapshot.hands[i].generation;
    return r;
}
void SessionTransitions()
{
    auto f=std::make_unique<Fixture>();
    auto s=f->Use(1000,HIGGS_HAND_LEFT);
    HIGGS_HandUseTransitionState state{};
    CHECK(!f->broker.CopyHandUseTransition(f->owner,s.authority.requestId,state,f->now));
    CHECK(state.revision==1 && state.use.authority.physicalHands==1);
    auto r=Transition(*f,s,state.revision,3);
    CHECK(f->broker.ResizeHandUse(f->other,r,state,f->now)==HIGGS_HAND_WRONG_OWNER);
    auto stale=r; ++stale.expectedHandGenerations[1];
    CHECK(f->broker.ResizeHandUse(f->owner,stale,state,f->now)==HIGGS_HAND_STALE);
    CHECK(!f->broker.ResizeHandUse(f->owner,r,state,f->now));
    CHECK(state.revision==2 && state.use.authority.physicalHands==3 &&
          state.use.authority.grant==s.authority.grant && state.use.authority.requestId==s.authority.requestId &&
          state.use.authority.contextGeneration==s.authority.contextGeneration);
    const auto owned=f->broker.CopyPublished();
    CHECK(owned.hands[0].grant==s.authority.grant && owned.hands[1].grant==s.authority.grant);
    const auto preserved=state;
    CHECK(f->broker.ResizeHandUse(f->owner,r,state,f->now)==HIGGS_HAND_CONFLICT);
    CHECK(!std::memcmp(&preserved,&state,sizeof(state)));
    HIGGS_ActionRequest child{};
    CHECK(!f->broker.ReserveDomain(f->owner,0,2,s.epoch,s.authority.grant,1000,child,f->now));
    CHECK(!f->broker.ClaimActuator(child.requestId,1));
    f->broker.DomainResult(child.requestId,HIGGS_ACTION_COMPLETED,0);
    r=Transition(*f,s,2,1);
    CHECK(f->broker.ResizeHandUse(f->owner,r,state,f->now)==HIGGS_HAND_CONFLICT);
    f->broker.UnclaimActuator(child.requestId);
    CHECK(f->broker.ResizeHandUse(f->owner,r,state,f->now)==HIGGS_HAND_CONFLICT);
    f->broker.RetainRequest(child.requestId,false);
    CHECK(!f->broker.ResizeHandUse(f->owner,r,state,f->now));
    CHECK(state.revision==3 && state.use.authority.grant==s.authority.grant &&
          f->broker.CopyPublished().hands[0].grant==s.authority.grant && !f->broker.CopyPublished().hands[1].grant);
    f->broker.DrainHandUses();
    CHECK(f->notifications==1 && f->notification.physicalHands==1);
    uint64_t otherExecutor=0;CHECK(!f->broker.Executor(f->other,Fixture::Notify,f.get(),otherExecutor));
    HIGGS_HandUseRequest other{sizeof(other),HIGGS_USE_HAND_GOALS,2,1000,s.epoch,0,0,otherExecutor};
    HIGGS_HandUseState otherUse{};CHECK(!f->broker.HandUse(f->other,other,otherUse,f->now));
    r=Transition(*f,s,3,3);
    CHECK(f->broker.ResizeHandUse(f->owner,r,state,f->now)==HIGGS_HAND_CONFLICT);
    CHECK(f->broker.CopyPublished().hands[0].grant==s.authority.grant &&
          f->broker.CopyPublished().hands[1].grant==otherUse.authority.grant);
    CHECK(!f->broker.EndHandUse(f->other,otherUse.authority.requestId,s.epoch));
    f->broker.SetInputContext(1,++f->now);
    CHECK(f->broker.ResizeHandUse(f->owner,r,state,f->now)==HIGGS_HAND_CONFLICT);
    r=Transition(*f,s,3,1);
    CHECK(!f->broker.ResizeHandUse(f->owner,r,state,f->now));
    CHECK(state.revision==4 && state.use.active);
    f->broker.SetInputContext(0,++f->now);
    r=Transition(*f,s,4,3);
    CHECK(!f->broker.ResizeHandUse(f->owner,r,state,f->now));
    f->broker.SetInputContext(1,++f->now);
    r=Transition(*f,s,5,1);
    CHECK(!f->broker.ResizeHandUse(f->owner,r,state,f->now));
    CHECK(state.revision==6 && !f->broker.CopyPublished().hands[1].grant);
    CHECK(!f->broker.EndHandUse(f->owner,s.authority.requestId,s.epoch));
    r=Transition(*f,s,6,1);
    CHECK(f->broker.ResizeHandUse(f->owner,r,state,f->now)==HIGGS_HAND_STALE);
    std::puts("PASS atomic one/two-hand transitions, stable grant, revision/generation checks, child drainage, conflict and menu cleanup");
}
void SessionTransitionRace()
{
    auto f=std::make_unique<Fixture>();auto s=f->Use(1000,1);
    const auto r=Transition(*f,s,1,3);
    std::atomic<int> ready=0;std::atomic<bool> go=false;
    HIGGS_HandStatus result[2]{};
    std::thread workers[2];
    for(unsigned i=0;i<2;++i) workers[i]=std::thread([&,i]{HIGGS_HandUseTransitionState state{};
        ++ready;while(!go.load())std::this_thread::yield();result[i]=f->broker.ResizeHandUse(f->owner,r,state,f->now);});
    while(ready.load()!=2)std::this_thread::yield();go=true;
    for(auto &worker:workers)worker.join();
    CHECK((result[0]==HIGGS_HAND_OK && result[1]==HIGGS_HAND_CONFLICT) ||
          (result[1]==HIGGS_HAND_OK && result[0]==HIGGS_HAND_CONFLICT));
    CHECK(f->broker.CopyPublished().hands[0].grant==s.authority.grant);
    f->broker.ObserveWorld(0,true,++f->now);
    HIGGS_HandUseTransitionState state{};
    CHECK(f->broker.ResizeHandUse(f->owner,r,state,f->now)==HIGGS_HAND_STALE);
    std::puts("PASS concurrent footprint CAS has one winner; load invalidates authority");
}
struct PresentationDriver
{
    bool applied=false,cleanupReady=false;
    static HIGGS_HandStatus Apply(HIGGS_PhysicalPolicyState &state,bool cleanup,void *context)
    {
        auto &driver=*static_cast<PresentationDriver*>(context);
        if (cleanup) {
            if (!driver.cleanupReady) return HIGGS_HAND_PENDING;
            driver.applied=false;state.flags|=HIGGS_POLICY_RESTORED;
        } else { driver.applied=true;state.appliedFields=state.request.presentation; }
        return HIGGS_HAND_OK;
    }
};
void SessionEffectDrainage()
{
    auto f=std::make_unique<Fixture>();auto s=f->Use();
    PhysicalPolicyService policies(f->broker);PresentationDriver driver;
    policies.Install(PresentationDriver::Apply,&driver,HIGGS_POLICY_CAP_CONTROLLER_NODE_CULL);
    HIGGS_PhysicalPolicyRequest effect{};effect.structSize=sizeof(effect);
    effect.kind=HIGGS_POLICY_PRESENTATION;effect.physicalHand=2;effect.durationMs=1000;
    effect.epoch=s.epoch;effect.handGeneration=f->broker.HandGeneration(2);effect.grant=s.authority.grant;
    effect.presentation=HIGGS_PRESENTATION_CULL_CONTROLLER;
    uint64_t policy=0;CHECK(!policies.Request(f->owner,effect,policy,f->now));
    policies.Drain(f->now);CHECK(driver.applied);
    auto r=Transition(*f,s,1,1);HIGGS_HandUseTransitionState state{};
    CHECK(f->broker.ResizeHandUse(f->owner,r,state,f->now)==HIGGS_HAND_CONFLICT);
    CHECK(!policies.End(f->owner,policy,s.epoch));policies.Drain(f->now);
    CHECK(driver.applied && f->broker.ResizeHandUse(f->owner,r,state,f->now)==HIGGS_HAND_CONFLICT);
    driver.cleanupReady=true;policies.Drain(f->now);CHECK(!driver.applied);
    CHECK(!f->broker.ResizeHandUse(f->owner,r,state,f->now));
    CHECK(f->broker.CopyPublished().hands[0].grant==s.authority.grant);
    CHECK(!f->broker.EndHandUse(f->owner,s.authority.requestId,s.epoch));
    s=f->Use();effect.grant=s.authority.grant;effect.handGeneration=f->broker.HandGeneration(2);
    driver.cleanupReady=false;CHECK(!policies.Request(f->owner,effect,policy,f->now));
    policies.Drain(f->now);CHECK(driver.applied);
    CHECK(!f->broker.EndHandUse(f->owner,s.authority.requestId,s.epoch));
    uint64_t otherExecutor=0;CHECK(!f->broker.Executor(f->other,Fixture::Notify,f.get(),otherExecutor));
    HIGGS_HandUseRequest next{sizeof(next),HIGGS_USE_HAND_GOALS,2,1000,s.epoch,0,0,otherExecutor};
    HIGGS_HandUseState nextUse{};
    CHECK(f->broker.HandUse(f->other,next,nextUse,f->now)==HIGGS_HAND_CONFLICT);
    policies.Drain(f->now);CHECK(driver.applied);
    CHECK(f->broker.HandUse(f->other,next,nextUse,f->now)==HIGGS_HAND_CONFLICT);
    driver.cleanupReady=true;policies.Drain(f->now);CHECK(!driver.applied);
    CHECK(!f->broker.HandUse(f->other,next,nextUse,f->now));
    CHECK(!f->broker.EndHandUse(f->other,nextUse.authority.requestId,s.epoch));
    std::puts("PASS passive grant-bound native effects block shrink and reassignment until actual cleanup");
}
struct SessionDispatcher
{
    Fixture *fixture;
    unsigned left=0,right=0;
    bool openUi=false;
    static void HIGGS_HAND_CALL Notify(const HIGGS_ActionRequest *request,void *context)
    {
        auto &d=*static_cast<SessionDispatcher*>(context);
        if (request->physicalHands==1) {
            ++d.left;
            if (d.openUi) d.fixture->broker.SetInputContext(1,++d.fixture->now);
            else d.fixture->broker.DrainHandUses();
        } else ++d.right;
    }
};
void SessionDispatchBoundaries()
{
    for(bool ui:{false,true}) {
        auto f=std::make_unique<Fixture>();SessionDispatcher dispatcher{f.get(),0,0,ui};
        uint64_t executor=0;CHECK(!f->broker.Executor(f->owner,SessionDispatcher::Notify,&dispatcher,executor));
        HIGGS_HandUseRequest r{sizeof(r),HIGGS_USE_HAND_GOALS,1,1000,f->broker.Epoch(),0,0,executor};
        HIGGS_HandUseState left{},right{};CHECK(!f->broker.HandUse(f->owner,r,left,f->now));
        r.physicalHands=2;CHECK(!f->broker.HandUse(f->owner,r,right,f->now));
        f->broker.DrainHandUses();
        CHECK(dispatcher.left==1 && dispatcher.right==(ui?0u:1u));
        if(ui) {f->broker.SetInputContext(0,++f->now);f->broker.DrainHandUses();}
        CHECK(dispatcher.left==1 && dispatcher.right==1);
        f->broker.DrainHandUses();CHECK(dispatcher.left==1 && dispatcher.right==1);
    }
    std::puts("PASS nested dispatch is exactly once; reentrant menu defers notification without losing it");
}
struct Mutation
{
    Fixture *f;
    int value = 0;
};
HIGGS_HandStatus HIGGS_HAND_CALL Commit(const HIGGS_ActionRequest *r, const HIGGS_HandSnapshot *snapshot, void *context)
{
    auto &m = *static_cast<Mutation *>(context);
    CHECK(snapshot->epoch == r->epoch);
    CHECK(m.f->broker.Release(m.f->owner, r->grant) == HIGGS_HAND_PENDING);
    CHECK(m.f->broker.Remove(m.f->owner, r->executor) == HIGGS_HAND_PENDING);
    CHECK(m.f->broker.Unregister(m.f->owner) == HIGGS_HAND_PENDING);
    HIGGS_HandSnapshot copy{};
    CHECK(!m.f->broker.Copy(m.f->owner, copy, Now()));
    m.value = 1;
    return HIGGS_HAND_OK;
}
void MutationFence()
{
    auto f = std::make_unique<Fixture>();
    auto s = f->Use();
    HIGGS_GrantedMutationRequest r{};
    r.structSize = sizeof(r);
    r.commitStage = HIGGS_ACTION_COMPLETED;
    r.requestId = s.authority.requestId;
    r.executor = s.authority.executor;
    r.epoch = s.epoch;
    r.grant = s.authority.grant;
    HIGGS_ActionResult result{};
    Mutation m{f.get()};
    std::thread wrong(
        [&] { CHECK(f->broker.WithAuthorizedAction(f->owner, r, Commit, &m, result) == HIGGS_HAND_WRONG_PHASE); });
    wrong.join();
    CHECK(!m.value);
    CHECK(!f->broker.WithAuthorizedAction(f->owner, r, Commit, &m, result));
    CHECK(m.value == 1 && result.stage == HIGGS_ACTION_COMPLETED && !result.receiptFlags);
    CHECK(!f->broker.Release(f->owner, r.grant));
    HIGGS_ActionResult after{};
    CHECK(!f->broker.Result(f->owner, r.requestId, after));
    CHECK(after.stage == HIGGS_ACTION_COMPLETED);
    CHECK(f->broker.WithAuthorizedAction(f->owner, r, Commit, &m, result) == HIGGS_HAND_WRONG_PHASE);
    std::puts("PASS main mutation fence, reentrant revocation refusal and committed result before fence exit");
}
HIGGS_HandStatus HIGGS_HAND_CALL SlowCommit(const HIGGS_ActionRequest *, const HIGGS_HandSnapshot *, void *context)
{
    auto &m = *static_cast<Mutation *>(context);
    std::this_thread::sleep_for(std::chrono::milliseconds(35));
    m.value = 2;
    return HIGGS_HAND_OK;
}
HIGGS_HandStatus HIGGS_HAND_CALL LifecycleCommit(const HIGGS_ActionRequest *request, const HIGGS_HandSnapshot *,
                                                 void *context)
{
    auto &m = *static_cast<Mutation *>(context);
    m.f->broker.ObserveWorld(0, true, Now());
    CHECK(m.f->broker.CopyPublished().epoch == request->epoch);
    CHECK(!m.f->broker.MutationAdmissionReady());
    m.value = 3;
    return HIGGS_HAND_OK;
}
void FenceLifecycle()
{
    auto f = std::make_unique<Fixture>();
    auto use = f->Use();
    HIGGS_GrantedMutationRequest r{};
    r.structSize = sizeof(r);
    r.commitStage = HIGGS_ACTION_COMPLETED;
    r.requestId = use.authority.requestId;
    r.executor = use.authority.executor;
    r.epoch = use.epoch;
    r.grant = use.authority.grant;
    Mutation mutation{f.get()};
    HIGGS_ActionResult result{};
    CHECK(!f->broker.WithAuthorizedAction(f->owner, r, LifecycleCommit, &mutation, result));
    CHECK(mutation.value == 3 && result.stage == HIGGS_ACTION_COMPLETED && result.epoch == use.epoch);
    CHECK(f->broker.CopyPublished().epoch > use.epoch && !f->broker.CopyPublished().worldReady);
    CHECK(!f->broker.Result(f->owner, r.requestId, result));
    CHECK(result.stage == HIGGS_ACTION_COMPLETED);
    std::puts("PASS reentrant lifecycle waits for entered mutation and committed receipt");
}
void FenceExpiry()
{
    auto f = std::make_unique<Fixture>();
    auto s = f->Use(25);
    HIGGS_GrantedMutationRequest r{};
    r.structSize = sizeof(r);
    r.commitStage = HIGGS_ACTION_COMPLETED;
    r.requestId = s.authority.requestId;
    r.executor = s.authority.executor;
    r.epoch = s.epoch;
    r.grant = s.authority.grant;
    Mutation m{f.get()};
    HIGGS_ActionResult result{};
    CHECK(!f->broker.WithAuthorizedAction(f->owner, r, SlowCommit, &m, result));
    CHECK(m.value == 2 && result.stage == HIGGS_ACTION_COMPLETED);
    HIGGS_ActionResult actual{};
    CHECK(!f->broker.Result(f->owner, r.requestId, actual));
    CHECK(actual.stage == HIGGS_ACTION_COMPLETED);
    std::puts("PASS authority cannot expire inside an entered mutation callback");
}
void HapticOwnership()
{
    auto f = std::make_unique<Fixture>();
    auto r = f->Pulse();
    uint64_t low = 0, high = 0;
    CHECK(!f->haptics.Request(f->other, r, low, f->now));
    r.startStrength = r.endStrength = .2f;
    CHECK(!f->haptics.Request(f->owner, r, high, f->now));
    auto sample = f->haptics.Sample(HIGGS_HAND_LEFT, f->now + 1);
    CHECK(sample.strength == .2f);
    f->haptics.Submitted(sample, true);
    f->haptics.Submitted(sample, true);
    HIGGS_HapticState state{};
    CHECK(!f->haptics.Copy(f->owner, high, state, f->now + 1));
    CHECK(state.submittedSamples == 1);
    CHECK(!f->haptics.Copy(f->other, low, state, f->now + 1));
    CHECK(state.suppressedSamples == 1);
    CHECK(f->haptics.Cancel(f->other, high) == HIGGS_HAND_WRONG_OWNER);
    CHECK(!f->haptics.Cancel(f->owner, high));
    sample = f->haptics.Sample(HIGGS_HAND_LEFT, f->now + 2);
    CHECK(sample.strength == .3f);
    f->broker.ConfigurePriorities("primary=0,other=0,HIGGS=0");
    r = f->Pulse(1, .7f);
    CHECK(!f->haptics.Request(f->owner, r, high, f->now));
    // The original HapticsManager now supplies the already-selected native envelope to the shared mixer.
    sample = f->haptics.Sample(HIGGS_HAND_LEFT, f->now + 3, .9f);
    CHECK(sample.strength == .9f);
    r = f->Pulse(1, .4f);
    uint64_t replacement = 0;
    CHECK(!f->haptics.Request(f->owner, r, replacement, f->now));
    CHECK(!f->haptics.Copy(f->owner, high, state, f->now));
    CHECK(state.stage == HIGGS_HAPTIC_CANCELLED);
    CHECK(!f->broker.Unregister(f->owner));
    sample = f->haptics.Sample(HIGGS_HAND_LEFT, f->now + 4, .9f);
    CHECK(sample.strength == .9f);
    f->broker.ObserveWorld(0, false, f->now + 5);
    sample = f->haptics.Sample(HIGGS_HAND_LEFT, f->now + 6);
    CHECK(sample.strength == 0);
    std::puts(
        "PASS coordinated priority/commutative haptics, owned context replacement, duplicate submission and lifecycle");
}
#include "hand_use_proposal_cases.h"
#include "authorized_dispatch_cases.h"
int main()
{
    AuthorizedDispatchOnce();
    AuthorizedDispatchAdmission();
    AuthorizedDispatchInputBoundary();
    AuthorizedDispatchLifecycle();
    ProposalPriorityRounds();
    ProposalLifetimes();
    ProposalReentrancy();
    ProposalNotificationExpiry();
    SessionOwnership();
    SessionTransitions();
    SessionTransitionRace();
    SessionEffectDrainage();
    SessionDispatchBoundaries();
    MutationFence();
    FenceExpiry();
    FenceLifecycle();
    HapticOwnership();
}
