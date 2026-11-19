#include "hand_broker.h"
#include "hand_transactions.h"
#include "synergy_inventory_native.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

using namespace HiggsHands;
extern "C" int inventory_c_probe(void);
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)

// This is the service's abstract engine seam. It supplies explicit outcomes;
// it does not model Skyrim inventory, extra data, UID resolution or events.
struct Engine final : InventoryEngine {
    struct Handle final : InventorySelectionHandle {};
    unsigned selects = 0, executes = 0, polls = 0, forgets = 0, resets = 0;
    bool current = true;
    std::weak_ptr<InventorySelectionHandle> selected;
    InventoryOutcome polled{HIGGS_HAND_PENDING};
    uint64_t Capabilities() const override { return HIGGS_INVENTORY_CAP_PLAYER_SELECTION | HIGGS_INVENTORY_CAP_DROP_TO_HAND; }
    HIGGS_HandStatus Select(const HIGGS_InventorySelectionRequest&, std::shared_ptr<InventorySelectionHandle>& out,
                            uint32_t& container) override {
        ++selects; out = std::make_shared<Handle>(); selected = out; container = 0x14; return HIGGS_HAND_OK;
    }
    bool Current(const std::shared_ptr<InventorySelectionHandle>& handle) override { return current && bool(handle); }
    InventoryOutcome Execute(const HIGGS_TransferExecution&, const std::shared_ptr<InventorySelectionHandle>& handle,
                             InventoryPermit& permit) override {
        ++executes; CHECK(handle); CHECK(permit.Begin()); CHECK(!permit.Begin()); return {HIGGS_HAND_PENDING};
    }
    InventoryOutcome Poll(const HIGGS_TransferExecution&) override { ++polls; return polled; }
    void Forget(uint64_t) override { ++forgets; }
    void Reset() override { ++resets; }
};

struct Fixture {
    Engine engine;
    uint64_t owner = 0, other = 0, executor = 0;
    HIGGS_InventoryInterface api{};
    static uint64_t world;
    Fixture() {
        auto& b = Service(); b.ObserveWorld(++world, false, Now()); b.MarkMainThread();
        std::array<HIGGS_HandRecord, 2> hands{};
        for (auto& h : hands) h.observationMask = HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN |
            HIGGS_OBS_PALM_VALID | HIGGS_OBS_TRACKING_VALID;
        b.Publish(hands, Now());
        HIGGS_HandClientInfo c{}; c.structSize = sizeof(c); std::strcpy(c.name, "inventory-service");
        CHECK(!b.Register(c, owner)); std::strcpy(c.name, "other"); CHECK(!b.Register(c, other));
        Inventory().Install(&engine); Inventory().Drain();
        CHECK(!HIGGS_QueryInventoryInterface(HIGGS_INVENTORY_VERSION, sizeof(api), &api));
        CHECK(api.capabilities == engine.Capabilities()); CHECK(!api.GetExecutor(owner, &executor));
    }
    ~Fixture() {
        Service().ObserveWorld(++world, false, Now()); Inventory().Drain();
        if (Service().ClientValid(owner)) CHECK(!Service().Unregister(owner));
        CHECK(!Service().Unregister(other)); Inventory().Install(nullptr);
    }
    HIGGS_InventorySelection Select() {
        HIGGS_InventorySelectionRequest r{sizeof(r), 100, Service().Epoch(), 0, 1, {0, 0}};
        uint64_t id = 0; CHECK(!api.RequestSelection(owner, &r, &id));
        HIGGS_InventorySelection value{}; value.structSize = sizeof(value);
        CHECK(!api.CopySelection(owner, id, &value)); CHECK(value.state == HIGGS_SELECTION_QUEUED);
        const auto calls = engine.selects; Inventory().Drain(); CHECK(engine.selects == calls + 1);
        CHECK(!api.CopySelection(owner, id, &value)); CHECK(value.state == HIGGS_SELECTION_READY);
        CHECK(value.containerRefId == 0x14 && value.item.kind == HIGGS_ITEM_NATIVE_INVENTORY);
        CHECK(value.item.validFields & HIGGS_ITEM_NATIVE_OBSERVED); return value;
    }
    HIGGS_TransferRequest Transfer(const HIGGS_InventorySelection& selection) {
        HIGGS_TransferRequest r{}; r.structSize = sizeof(r); r.kind = HIGGS_TRANSFER_INVENTORY_TO_HAND;
        r.destinationHand = HIGGS_HAND_LEFT; r.epoch = Service().Epoch(); r.idempotencyKey = 1;
        r.original = selection.item; r.executor = executor; r.quantity = 1; r.timeoutMs = 1000; return r;
    }
    HIGGS_TransferState State(uint64_t id) {
        HIGGS_TransferState s{}; CHECK(!Transactions().Copy(owner, id, s)); return s;
    }
};
uint64_t Fixture::world = 400;

void SelectionAndGuards() {
    Fixture f;
    HIGGS_InventoryInterface unchanged{}; std::memset(&unchanged, 0xa5, sizeof(unchanged));
    const auto before = unchanged;
    CHECK(HIGGS_QueryInventoryInterface(999, sizeof(unchanged), &unchanged) == HIGGS_HAND_UNSUPPORTED_VERSION);
    CHECK(!std::memcmp(&before, &unchanged, sizeof(before)));
    CHECK(HIGGS_QueryInventoryInterface(2, sizeof(unchanged) - 1, &unchanged) == HIGGS_HAND_BUFFER_TOO_SMALL);
    CHECK(!std::memcmp(&before, &unchanged, sizeof(before)));
    auto selected = f.Select(); auto copy = selected;
    CHECK(f.api.CopySelection(f.other, selected.ticket, &copy) == HIGGS_HAND_WRONG_OWNER);
    CHECK(!std::memcmp(&copy, &selected, sizeof(copy)));
    f.engine.current = false; Inventory().Drain();
    CHECK(!f.api.CopySelection(f.owner, selected.ticket, &copy));
    CHECK(copy.state == HIGGS_SELECTION_INVALID && copy.reason == HIGGS_HAND_STALE);
    CHECK(!f.api.ReleaseSelection(f.owner, selected.ticket)); Inventory().Drain();
    CHECK(f.api.CopySelection(f.owner, selected.ticket, &copy) == HIGGS_HAND_NOT_FOUND);
    Inventory().Install(nullptr); Inventory().Drain();
    HIGGS_InventoryInterface absent{}; CHECK(!HIGGS_QueryInventoryInterface(2, sizeof(absent), &absent));
    CHECK(!absent.capabilities);
    HIGGS_InventorySelectionRequest r{sizeof(r), 100, Service().Epoch(), 0, 1, {0, 0}};
    uint64_t ticket = 91; CHECK(absent.RequestSelection(f.owner, &r, &ticket) == HIGGS_HAND_NOT_READY);
    CHECK(ticket == 91);
    Inventory().Install(&f.engine);
}

void PendingResultsAndPins() {
    Fixture f; const auto selected = f.Select(); const auto request = f.Transfer(selected);
    uint64_t id = 0, duplicate = 0; CHECK(!Transactions().Request(f.owner, request, id, Now()));
    Transactions().Drain(Now()); CHECK(f.engine.executes == 1); CHECK(f.State(id).stage == HIGGS_TX_EXECUTING);
    CHECK(!Transactions().Request(f.owner, request, duplicate, Now())); CHECK(duplicate == id);
    CHECK(f.engine.executes == 1 && Transactions().ItemInUse(selected.item.instance));
    CHECK(f.api.ReleaseSelection(f.owner, selected.ticket) == HIGGS_HAND_PENDING);
    Inventory().Drain(); CHECK(f.engine.polls == 1 && f.State(id).stage == HIGGS_TX_EXECUTING);
    f.engine.polled = {HIGGS_HAND_OK, HIGGS_TX_INVENTORY_COMMITTED, 555, 0, true};
    Inventory().Drain(); const auto committed = f.State(id);
    CHECK(committed.stage == HIGGS_TX_WAIT_GRIP); CHECK(committed.receiptFlags & HIGGS_TX_DOMAIN_ATTESTED);
    CHECK(!(committed.receiptFlags & HIGGS_TX_NATIVE_GRIP_CONFIRMED));
    CHECK(committed.actualRefId == 555 && f.engine.executes == 1);
    // No synthetic grip confirmation is supplied: an engine commitment alone is insufficient.
    CHECK(!Transactions().Cancel(f.owner, id, Service().Epoch()));
    CHECK(f.State(id).stage == HIGGS_TX_WAIT_COMPENSATION);
}

void UnresolvedAfterClientLoss() {
    Fixture f; const auto selected = f.Select(); const auto request = f.Transfer(selected);
    uint64_t id = 0; CHECK(!Transactions().Request(f.owner, request, id, Now())); Transactions().Drain(Now());
    CHECK(f.engine.executes == 1); CHECK(!Service().Unregister(f.owner));
    const auto polls = f.engine.polls, forgets = f.engine.forgets;
    Inventory().Drain(); Transactions().Drain(Now()); Inventory().Drain();
    HIGGS_TransferState native{};
    CHECK(Transactions().Copy(f.owner, id, native) == HIGGS_HAND_INVALID_CLIENT);
    CHECK(Transactions().NativePinRequired(f.owner, id, Service().Epoch(), native));
    CHECK(native.stage == HIGGS_TX_RECOVERY_REQUIRED);
    CHECK(f.engine.polls == polls && f.engine.forgets == forgets);
    CHECK(!f.engine.selected.expired() && Transactions().ItemInUse(selected.item.instance));
    CHECK(!Transactions().RetireInventory(f.owner, selected.ticket));
    Service().ObserveWorld(++Fixture::world, false, Now()); Inventory().Drain();
    CHECK(!Transactions().NativePinRequired(f.owner, id, Service().Epoch(), native));
    CHECK(!Transactions().ItemInUse(selected.item.instance)); CHECK(f.engine.selected.expired());
}

int main() {
    CHECK(!inventory_c_probe());
    SelectionAndGuards(); PendingResultsAndPins(); UnresolvedAfterClientLoss();
    std::printf("PASS inventory service %u checks (abstract engine outcomes only)\n", checks);
}
