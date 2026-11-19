#include "hand_broker.h"
#include "hand_support.h"
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
    Broker broker;
    SupportService support{broker};
    uint64_t owner = 0, other = 0, now = Now();
    bool cleanupReady = true, reset = false;
    unsigned updates = 0, ends = 0;
    std::array<HIGGS_HandRecord, 2> hands{};
    Fixture()
    {
        HIGGS_HandClientInfo client{};
        client.structSize = sizeof(client);
        std::snprintf(client.name, sizeof(client.name), "support");
        CHECK(!broker.Register(client, owner));
        std::snprintf(client.name, sizeof(client.name), "other");
        CHECK(!broker.Register(client, other));
        broker.ObserveWorld(1, false, now);
        for (auto &hand : hands)
            hand.observationMask =
                HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN | HIGGS_OBS_PALM_VALID | HIGGS_OBS_TRACKING_VALID;
        broker.Publish(hands, now);
    }
    HIGGS_SupportRequest Request(uint32_t kind = HIGGS_SUPPORT_MOVING_WORLD)
    {
        HIGGS_SupportRequest request{};
        request.structSize = sizeof(request);
        request.kind = kind;
        request.physicalHand = HIGGS_HAND_LEFT;
        request.durationMs = 1000;
        request.epoch = broker.Epoch();
        request.handGeneration = broker.HandGeneration(HIGGS_HAND_LEFT);
        request.referenceFormId = 99;
        request.maxObservationAgeMs = 20;
        request.targetLocalRotation[0] = request.targetLocalRotation[4] = request.targetLocalRotation[8] = 1;
        request.maxDistanceWorld = 20;
        return request;
    }
    HIGGS_SupportState State(uint64_t id)
    {
        HIGGS_SupportState state{};
        CHECK(!support.Copy(owner, id, state));
        return state;
    }
    static HIGGS_HandStatus Driver(HIGGS_SupportState &state, bool ending, void *context)
    {
        auto &fixture = *static_cast<Fixture *>(context);
        if (ending)
        {
            ++fixture.ends;
            return fixture.cleanupReady ? HIGGS_HAND_OK : HIGGS_HAND_PENDING;
        }
        ++fixture.updates;
        if (fixture.reset)
        {
            fixture.broker.ObserveWorld(0, true, Now());
            CHECK(fixture.broker.Epoch() == state.epoch);
        }
        state.endpointGeneration = 55;
        state.handScale = 1;
        state.handPosition[0] = state.request.targetLocalPoint[0];
        state.flags |= HIGGS_SUPPORT_TARGET_VALID;
        if (state.request.kind == HIGGS_CONSTRAINT_HELD_TO_WORLD)
            state.flags |= HIGGS_SUPPORT_CONSTRAINT_ATTACHED | HIGGS_SUPPORT_NATIVE_APPLIED;
        return HIGGS_HAND_OK;
    }
    void Install()
    {
        support.Install(Driver, this, (UINT64_C(1) << 0) | (UINT64_C(1) << 3));
    }
};
void NoFabricatedCapabilities()
{
    auto f = std::make_unique<Fixture>();
    uint64_t id = 0;
    CHECK(!f->support.Capabilities());
    CHECK(f->support.Request(f->owner, f->Request(), id, f->now) == HIGGS_HAND_UNSUPPORTED);
    f->Install();
    CHECK(f->support.Request(f->owner, f->Request(HIGGS_SUPPORT_PLANCK_HAND), id, f->now) == HIGGS_HAND_UNSUPPORTED);
    std::puts("PASS support capabilities reflect installed finite backend only");
}
void TargetRevisionsAndFreshness()
{
    auto f = std::make_unique<Fixture>();
    f->Install();
    uint64_t id = 0;
    auto request = f->Request();
    CHECK(!f->support.Request(f->owner, request, id, f->now));
    CHECK(f->State(id).stage == HIGGS_SUPPORT_PENDING);
    HIGGS_SupportState target{};
    CHECK(!f->support.Goal(HIGGS_HAND_LEFT, target, f->now));
    f->support.Drain(f->now);
    auto state = f->State(id);
    CHECK(state.stage == HIGGS_SUPPORT_ACTIVE && !state.appliedRevision);
    CHECK(f->support.Goal(HIGGS_HAND_LEFT, target, f->now));
    f->support.Applied(id, f->now + 1);
    state = f->State(id);
    CHECK(state.appliedRevision == 1 && (state.flags & HIGGS_SUPPORT_NATIVE_APPLIED));
    CHECK(!f->support.Goal(HIGGS_HAND_LEFT, target, f->now + 21000000));
    f->support.Applied(id, f->now + 21000000);
    CHECK(f->State(id).validatedAtNs == f->now);
    HIGGS_SupportUpdate update{};
    update.structSize = sizeof(update);
    update.epoch = request.epoch;
    update.expectedRevision = 1;
    update.targetLocalPoint[0] = 2;
    update.targetLocalRotation[0] = update.targetLocalRotation[4] = update.targetLocalRotation[8] = 1;
    CHECK(f->support.Update(f->other, id, update) == HIGGS_HAND_WRONG_OWNER);
    CHECK(!f->support.Update(f->owner, id, update));
    CHECK(f->State(id).requestRevision == 2 && f->State(id).appliedRevision == 1);
    f->support.Drain(f->now + 2);
    CHECK(f->support.Goal(HIGGS_HAND_LEFT, target, f->now + 2) && target.handPosition[0] == 2);
    f->support.Applied(id, f->now + 3);
    CHECK(f->State(id).appliedRevision == 2);
    CHECK(f->support.Update(f->owner, id, update) == HIGGS_HAND_STALE);
    f->broker.SetInputContext(1,f->now);update.expectedRevision=2;update.targetLocalPoint[0]=3;
    CHECK(f->support.Update(f->owner,id,update)==HIGGS_HAND_NOT_READY);
    update.targetLocalPoint[0]=2;CHECK(!f->support.Update(f->owner,id,update));
    CHECK(f->State(id).requestRevision==3);f->broker.SetInputContext(0,f->now);
    CHECK(!f->support.End(f->owner, id, request.epoch));
    CHECK(!f->support.Goal(HIGGS_HAND_LEFT, target, f->now + 4));
    f->cleanupReady = false;
    f->support.Drain(f->now + 4);
    CHECK(f->State(id).stage == HIGGS_SUPPORT_ENDING);
    f->cleanupReady = true;
    f->support.Drain(f->now + 5);
    CHECK(f->State(id).stage == HIGGS_SUPPORT_RELEASED);
    std::puts("PASS support pending/applied revisions, bounded freshness and actual-cleanup completion");
}
void ConstraintAuthorityAndReset()
{
    auto f = std::make_unique<Fixture>();
    f->hands[0].heldRefId = 88;
    f->broker.Publish(f->hands, f->now);
    f->Install();
    auto request = f->Request(HIGGS_CONSTRAINT_HELD_TO_WORLD);
    request.sourceItemRefId = 88;
    uint64_t id = 0;
    CHECK(!f->support.Request(f->owner, request, id, f->now));
    f->support.Drain(f->now);
    CHECK(f->State(id).flags & HIGGS_SUPPORT_CONSTRAINT_ATTACHED);
    CHECK(!f->broker.Release(f->owner, f->State(id).grant));
    HIGGS_HandSnapshot snapshot{};
    CHECK(!f->broker.Copy(f->other, snapshot, f->now));
    CHECK(snapshot.hands[0].availability == HIGGS_AVAILABILITY_UNAVAILABLE);
    f->cleanupReady = false;
    f->support.Drain(f->now + 1);
    CHECK(f->State(id).stage == HIGGS_SUPPORT_ENDING);
    f->cleanupReady = true;
    f->support.Drain(f->now + 2);
    CHECK(f->State(id).stage == HIGGS_SUPPORT_BROKEN);
    CHECK(!f->broker.Copy(f->other, snapshot, f->now + 2));
    CHECK(snapshot.hands[0].heldRefId == 88 && snapshot.hands[0].availability == HIGGS_AVAILABILITY_RESERVED);
    auto lifecycle = std::make_unique<Fixture>();
    lifecycle->Install();
    lifecycle->reset = true;
    request = lifecycle->Request();
    CHECK(!lifecycle->support.Request(lifecycle->owner, request, id, lifecycle->now));
    lifecycle->support.Drain(lifecycle->now);
    CHECK(lifecycle->broker.Epoch() != request.epoch);
    lifecycle->support.Drain(lifecycle->now + 1);
    CHECK(lifecycle->State(id).stage == HIGGS_SUPPORT_BROKEN);
    std::puts("PASS constraint authority cannot be reused before cleanup; source possession survives; reentrant reset "
              "fenced");
}
int main()
{
    NoFabricatedCapabilities();
    TargetRevisionsAndFreshness();
    ConstraintAuthorityAndReset();
}
