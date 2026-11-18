#include "hand_broker.h"
#include "hand_physical_policy.h"
#include "hand_transactions.h"
extern "C" int policy_c(void);
#include <cstdio>
#include <cstdlib>
#include <memory>
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
    uint64_t owner = 0, other = 0, now = Now();
    unsigned calls = 0, cleanups = 0;
    bool cleanupReady = true, reset = false;
    std::array<HIGGS_HandRecord, 2> hands{};
    Fixture()
    {
        static uintptr_t world = 100;
        Service().ObserveWorld(++world, false, now);
        HIGGS_HandClientInfo info{};
        info.structSize = sizeof(info);
        std::snprintf(info.name, sizeof(info.name), "physical policy");
        CHECK(!Service().Register(info, owner));
        std::snprintf(info.name, sizeof(info.name), "other policy");
        CHECK(!Service().Register(info, other));
        for (auto &h : hands)
            h.observationMask =
                HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN | HIGGS_OBS_TRACKING_VALID | HIGGS_OBS_PALM_VALID;
        Service().Publish(hands, now);
        PhysicalPolicies().Install(Driver, this, 15);
    }
    ~Fixture()
    {
        Service().Unregister(owner);
        Service().Unregister(other);
        PhysicalPolicies().Drain(Now());
        PhysicalPolicies().Install(nullptr, nullptr, 0);
    }
    static HIGGS_HandStatus Driver(HIGGS_PhysicalPolicyState &state, bool cleanup, void *data)
    {
        auto &f = *static_cast<Fixture *>(data);
        if (cleanup)
        {
            ++f.cleanups;
            if (!f.cleanupReady)
                return HIGGS_HAND_PENDING;
            state.flags |= HIGGS_POLICY_RESTORED;
            return HIGGS_HAND_OK;
        }
        ++f.calls;
        if (f.reset)
        {
            Service().ObserveWorld(0, true, Now());
            CHECK(Service().Epoch() == state.epoch);
        }
        if (state.request.kind == 1)
            state.appliedFields = state.request.fields;
        return HIGGS_HAND_OK;
    }
    HIGGS_PhysicalPolicyRequest Request(uint32_t kind)
    {
        HIGGS_PhysicalPolicyRequest r{};
        r.structSize = sizeof(r);
        r.kind = kind;
        r.physicalHand = HIGGS_HAND_LEFT;
        r.durationMs = 1000;
        r.epoch = Service().Epoch();
        r.handGeneration = Service().HandGeneration(HIGGS_HAND_LEFT);
        return r;
    }
    HIGGS_PhysicalPolicyState State(uint64_t id)
    {
        HIGGS_PhysicalPolicyState state{};
        CHECK(!PhysicalPolicies().Copy(owner, id, state));
        return state;
    }
    void Held()
    {
        hands[0].heldRefId = 700;
        Transactions().ObserveHeld(HIGGS_HAND_LEFT, 1000, 700, 900, 1, now);
        Service().Publish(hands, ++now);
    }
};
void PassiveOwnership()
{
    Fixture f;
    f.Held();
    auto r = f.Request(1);
    CHECK(!Transactions().Held(f.owner, HIGGS_HAND_LEFT, r.item));
    r.fields = HIGGS_PROPERTY_MASS;
    r.massMultiplier = r.inertiaMultiplier = 1;
    uint64_t id = 0, other = 0;
    CHECK(!PhysicalPolicies().Request(f.owner, r, id, f.now));
    HIGGS_HandSnapshot snapshot{};
    CHECK(!Service().Copy(f.owner, snapshot, f.now));
    CHECK(!snapshot.hands[0].grant);
    CHECK(PhysicalPolicies().Request(f.other, r, other, f.now) == HIGGS_HAND_CONFLICT);
    auto com = r;
    com.fields = HIGGS_PROPERTY_CENTER_OF_MASS;
    CHECK(!PhysicalPolicies().Request(f.other, com, other, f.now));
    PhysicalPolicies().Drain(f.now);
    CHECK(f.State(id).stage == HIGGS_POLICY_ACTIVE && f.State(id).appliedRevision == 1);
    r.massMultiplier = 2;
    CHECK(!PhysicalPolicies().Update(f.owner, id, 1, r, f.now));
    CHECK(PhysicalPolicies().Update(f.owner, id, 1, r, f.now) == HIGGS_HAND_STALE);
    PhysicalPolicies().Drain(f.now);
    CHECK(f.State(id).appliedRevision == 2);
    const auto generation=Service().HandGeneration(1);
    Service().SetInputContext(1,f.now);
    f.hands[0].unavailableReasons=HIGGS_HAND_REASON_MENU;Service().Publish(f.hands,++f.now);PhysicalPolicies().Drain(f.now);
    CHECK(Service().HandGeneration(1)==generation&&f.State(id).stage==HIGGS_POLICY_ACTIVE);
    r.massMultiplier=3;CHECK(PhysicalPolicies().Update(f.owner,id,2,r,f.now)==HIGGS_HAND_NOT_READY);
    r.massMultiplier=2;r.durationMs=1500;CHECK(!PhysicalPolicies().Update(f.owner,id,2,r,f.now));
    PhysicalPolicies().Drain(f.now);CHECK(f.State(id).appliedRevision==3);
    Service().SetInputContext(0,f.now);
    f.hands[0].unavailableReasons=0;Service().Publish(f.hands,++f.now);CHECK(Service().HandGeneration(1)==generation);
    f.cleanupReady = false;
    CHECK(!PhysicalPolicies().End(f.owner, id, r.epoch));
    PhysicalPolicies().Drain(f.now);
    CHECK(f.State(id).stage == HIGGS_POLICY_ENDING);
    CHECK(PhysicalPolicies().Request(f.owner, r, other, f.now) == HIGGS_HAND_CONFLICT);
    f.cleanupReady = true;
    PhysicalPolicies().Drain(f.now);
    CHECK(f.State(id).stage == HIGGS_POLICY_ENDED);
    std::puts("PASS passive effects do not acquire input; coupled property resource/CAS/cleanup ownership");
}
void GrabAndFingerLifetime()
{
    Fixture f;
    auto r = f.Request(2);
    r.grabMode = HIGGS_GRAB_IGNORE_GLOBAL_FORCE;
    uint64_t id = 0;
    CHECK(!PhysicalPolicies().Request(f.owner, r, id, f.now));
    CHECK(PhysicsGrabPreference(true, true));
    PhysicalPolicies().Drain(f.now);
    CHECK(!PhysicsGrabPreference(true, true));
    CHECK(f.State(id).flags & HIGGS_POLICY_APPLIED);
    CHECK(PhysicsGrabPreference(false, true));
    CHECK(!PhysicalPolicies().End(f.owner, id, r.epoch));
    CHECK(PhysicsGrabPreference(true, true));
    PhysicalPolicies().Drain(f.now);
    r = f.Request(3);
    r.presentation = HIGGS_PRESENTATION_SUPPRESS_FINGERS;
    CHECK(!PhysicalPolicies().Request(f.owner, r, id, f.now));
    PhysicalPolicies().Drain(f.now);
    CHECK(SuppressHiggsFingers(true) && !SuppressHiggsFingers(false));
    f.hands[0].observationMask &= ~HIGGS_OBS_TRACKING_VALID;
    Service().Publish(f.hands, ++f.now);
    CHECK(!SuppressHiggsFingers(true));
    PhysicalPolicies().Drain(f.now);
    CHECK(f.State(id).stage == HIGGS_POLICY_BROKEN);
    std::puts("PASS per-hand grab preference/finger suppression, observed use and tracking cleanup");
}
void ExpiryAndLifecycle()
{
    Fixture f;
    auto r = f.Request(2);
    r.grabMode = HIGGS_GRAB_FORCE_PHYSICS;
    r.durationMs = 1;
    uint64_t id = 0;
    CHECK(!PhysicalPolicies().Request(f.owner, r, id, f.now));
    PhysicalPolicies().Drain(f.now);
    PhysicalPolicies().Drain(f.now + 2000000);
    CHECK(f.State(id).stage == HIGGS_POLICY_BROKEN);
    CHECK(PhysicalPolicies().Update(f.owner, id, 1, r, f.now + 2000000) == HIGGS_HAND_STALE);
    r = f.Request(2);
    r.grabMode = 1;
    f.reset = true;
    CHECK(!PhysicalPolicies().Request(f.owner, r, id, f.now));
    PhysicalPolicies().Drain(f.now);
    CHECK(Service().Epoch() != r.epoch);
    PhysicalPolicies().Drain(f.now);
    CHECK(f.State(id).stage == HIGGS_POLICY_BROKEN);
    std::puts("PASS completed passive authority still expires; lifecycle reset is fenced through native use");
}
int main()
{
    CHECK(!policy_c());
    PassiveOwnership();
    GrabAndFingerLifetime();
    ExpiryAndLifecycle();
}
