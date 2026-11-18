#include "hand_broker.h"
#include "higgstransactions.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace HiggsHands;
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)

struct Backend {
    unsigned reads = 0, writes = 0;
    bool pending = true, unreadableAfter = false, rejectBefore = false;
    static HIGGS_HandStatus HIGGS_HAND_CALL Observe(const HIGGS_EquipmentTransition* r,
                                                   HIGGS_EquipmentEvidence* out, void* context) {
        auto& b = *static_cast<Backend*>(context);
        ++b.reads;
        if (!b.writes && b.rejectBefore) return HIGGS_HAND_UNSUPPORTED;
        if (b.writes && b.unreadableAfter) return HIGGS_HAND_INTERNAL_ERROR;
        out->observedMask = (1u << r->placeCount) - 1;
        for (unsigned i = 0; i < r->placeCount; ++i)
            out->places[i] = b.writes && !b.pending ? r->places[i].after : r->places[i].before;
        return HIGGS_HAND_OK;
    }
    static HIGGS_HandStatus HIGGS_HAND_CALL Apply(const HIGGS_EquipmentTransition*, void* context) {
        auto& b = *static_cast<Backend*>(context);
        ++b.writes;
        return b.pending ? HIGGS_HAND_PENDING : HIGGS_HAND_OK;
    }
    HIGGS_EquipmentBackend Table() {
        return {sizeof(HIGGS_EquipmentBackend), HIGGS_EQUIP_ORDINARY | HIGGS_EQUIP_MULTI_PLACE,
                Observe, Apply, this};
    }
};

struct Fixture {
    HIGGS_EquipmentInterface api{};
    uint64_t owner = 0, executor = 0;
    Fixture() {
        auto& b = Service();
        b.ObserveWorld(101, false, Now()); b.MarkMainThread();
        Publish();
        HIGGS_HandClientInfo info{}; info.structSize = sizeof(info);
        std::strcpy(info.name, "equipment-claims"); CHECK(!b.Register(info, owner));
        CHECK(!b.Executor(owner, Execute, nullptr, executor));
        CHECK(!HIGGS_QueryEquipmentInterface(1, sizeof(api), &api));
    }
    static void HIGGS_HAND_CALL Execute(const HIGGS_ActionRequest*, void*) {}
    void Publish() {
        std::array<HIGGS_HandRecord, 2> hands{};
        for (auto& h : hands) h.observationMask = HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN |
            HIGGS_OBS_PALM_VALID | HIGGS_OBS_TRACKING_VALID;
        Service().Publish(hands, Now());
    }
    static HIGGS_EquipmentItem Item(uint64_t instance) {
        return {HIGGS_EQUIP_ITEM, 100, 100, 100, 555, instance, 1};
    }
    HIGGS_EquipmentTransition Transition(uint32_t hand, uint64_t instance) {
        HIGGS_EquipmentTransition r{};
        r.structSize = sizeof(r); r.physicalHands = hand; r.placeCount = 1;
        r.requiredCapabilities = HIGGS_EQUIP_ORDINARY;
        r.places[0] = {HIGGS_EQUIP_PLACE_PHYSICAL_HAND, hand,
                       {HIGGS_EQUIP_EMPTY, 0, 0, 0, 0, 0, 0}, Item(instance)};
        return r;
    }
    HIGGS_HandStatus Request(HIGGS_EquipmentTransition& r, Backend& backend, bool endGrant = true) {
        auto& b = Service();
        HIGGS_HandUseRequest use{sizeof(use), HIGGS_USE_GAME_EQUIPMENT, r.physicalHands, 1,
                                 b.Epoch(), 0, 0, executor};
        HIGGS_HandUseState state{};
        const auto admission = b.HandUse(owner, use, state, Now());
        if (admission) return admission;
        const auto& a = state.authority;
        r.authority = {sizeof(r.authority), 0, a.requestId, a.executor, a.epoch, a.grant};
        auto table = backend.Table();
        const auto status = api.Request(owner, &r, &table);
        if (endGrant) CHECK(!b.Release(owner, a.grant));
        return status;
    }
    HIGGS_EquipmentOutcome Outcome(const HIGGS_EquipmentTransition& r) {
        HIGGS_EquipmentOutcome out{}; out.structSize = sizeof(out);
        CHECK(!api.CopyOutcome(owner, r.authority.epoch, r.authority.requestId, &out));
        return out;
    }
    HIGGS_EquipmentEvidence Evidence(const HIGGS_EquipmentTransition& r, bool after, bool settled) {
        HIGGS_EquipmentEvidence e{}; e.structSize = sizeof(e); e.placeCount = r.placeCount;
        e.observedMask = (1u << r.placeCount) - 1;
        e.flags = settled ? HIGGS_EQUIP_BACKEND_SETTLED : 0;
        for (unsigned i = 0; i < r.placeCount; ++i) e.places[i] = after ? r.places[i].after : r.places[i].before;
        return e;
    }
    void Report(const HIGGS_EquipmentTransition& r, HIGGS_EquipmentEvidence evidence) {
        CHECK(!api.ReportObservation(owner, r.authority.epoch, r.authority.requestId, &evidence));
    }
    void Expire() {
        HIGGS_HandSnapshot snapshot{};
        // Expire both the selected 1ms grant and the retained dependency's 60s lease.
        CHECK(!Service().Copy(owner, snapshot, Now() + UINT64_C(61000000000)));
    }
};

void PendingClaimsAndSettlement() {
    Fixture f; Backend first;
    auto pending = f.Transition(HIGGS_HAND_LEFT, 1001);
    CHECK(!f.Request(pending, first, false)); CHECK(first.writes == 1);
    CHECK(f.Outcome(pending).stage == HIGGS_EQUIP_AWAITING);
    f.Expire();
    Backend blocked;
    auto samePlace = f.Transition(HIGGS_HAND_LEFT, 1002);
    CHECK(f.Request(samePlace, blocked) == HIGGS_HAND_CONFLICT);
    CHECK(!blocked.reads && !blocked.writes);

    auto sameItem = f.Transition(HIGGS_HAND_RIGHT, 1001);
    sameItem.places[0].after.generation = 2; // A revision cannot rename an unsettled instance.
    CHECK(f.Request(sameItem, blocked) == HIGGS_HAND_CONFLICT);
    CHECK(!blocked.reads && !blocked.writes);

    Backend independent; independent.pending = false;
    auto other = f.Transition(HIGGS_HAND_RIGHT, 1002); // Same form, different exact instance.
    CHECK(!f.Request(other, independent)); CHECK(independent.writes == 1);
    CHECK(f.Outcome(other).stage == HIGGS_EQUIP_CONFIRMED);
    f.Report(pending, f.Evidence(pending, true, false));
    CHECK(f.Outcome(pending).stage == HIGGS_EQUIP_AWAITING);
    CHECK(f.Request(samePlace, blocked) == HIGGS_HAND_CONFLICT);
    f.Report(pending, f.Evidence(pending, true, true));
    CHECK(f.Outcome(pending).stage == HIGGS_EQUIP_CONFIRMED);
    Backend resumed; resumed.pending = false;
    CHECK(!f.Request(samePlace, resumed)); CHECK(resumed.writes == 1);
    CHECK(!Service().Unregister(f.owner));
}

void PartialAndUnknownClaims() {
    Fixture f; Backend first;
    auto pending = f.Transition(HIGGS_HAND_BOTH, 2001);
    pending.requiredCapabilities |= HIGGS_EQUIP_MULTI_PLACE; pending.placeCount = 2;
    pending.places[0].placeId = HIGGS_HAND_LEFT;
    pending.places[1] = {HIGGS_EQUIP_PLACE_PHYSICAL_HAND, HIGGS_HAND_RIGHT,
                         {HIGGS_EQUIP_EMPTY, 0, 0, 0, 0, 0, 0}, Fixture::Item(2002)};
    CHECK(!f.Request(pending, first, false)); f.Expire();
    auto partial = f.Evidence(pending, true, true); partial.observedMask = 1;
    f.Report(pending, partial);
    CHECK(f.Outcome(pending).stage == HIGGS_EQUIP_RECOVERY_REQUIRED);
    Backend blocked; auto next = f.Transition(HIGGS_HAND_RIGHT, 2003);
    CHECK(f.Request(next, blocked) == HIGGS_HAND_CONFLICT); CHECK(!blocked.writes);
    f.Report(pending, f.Evidence(pending, false, true));
    CHECK(f.Outcome(pending).stage == HIGGS_EQUIP_ORIGINAL_OBSERVED);

    Backend unknown; unknown.pending = false; unknown.unreadableAfter = true;
    CHECK(!f.Request(next, unknown, false)); CHECK(unknown.writes == 1);
    CHECK(f.Outcome(next).stage == HIGGS_EQUIP_RECOVERY_REQUIRED); f.Expire();
    auto retry = f.Transition(HIGGS_HAND_RIGHT, 2004);
    CHECK(f.Request(retry, blocked) == HIGGS_HAND_CONFLICT); CHECK(!blocked.writes);
    f.Report(next, f.Evidence(next, true, true));
    CHECK(f.Outcome(next).stage == HIGGS_EQUIP_CONFIRMED);
    CHECK(!Service().Unregister(f.owner));
}

void ClientLossAndDeclinedMutation() {
    Fixture f; Backend declined; declined.rejectBefore = true;
    auto rejected = f.Transition(HIGGS_HAND_LEFT, 3001);
    CHECK(!f.Request(rejected, declined)); CHECK(!declined.writes);
    CHECK(f.Outcome(rejected).stage == HIGGS_EQUIP_DECLINED);
    Backend pending;
    CHECK(!f.Request(rejected, pending, false)); CHECK(pending.writes == 1);
    CHECK(!Service().Unregister(f.owner));
    Fixture replacement; Backend blocked;
    auto next = replacement.Transition(HIGGS_HAND_LEFT, 3002);
    CHECK(replacement.Request(next, blocked) == HIGGS_HAND_CONFLICT);
    CHECK(!blocked.reads && !blocked.writes);
    auto proof = replacement.Evidence(rejected, true, true);
    CHECK(replacement.api.ReportObservation(replacement.owner, rejected.authority.epoch,
                                            rejected.authority.requestId, &proof) == HIGGS_HAND_WRONG_OWNER);
    Service().ObserveWorld(102, false, Now()); replacement.Publish();
    CHECK(replacement.Outcome(rejected).stage == HIGGS_EQUIP_RECOVERY_REQUIRED);
    Backend fresh; fresh.pending = false;
    CHECK(!replacement.Request(next, fresh)); CHECK(fresh.writes == 1);
    CHECK(!Service().Unregister(replacement.owner));
}

int main() {
    PendingClaimsAndSettlement(); PartialAndUnknownClaims(); ClientLossAndDeclinedMutation();
    std::printf("HIGGS production equipment claims: %u checks passed; zero game starts\n", checks);
}
