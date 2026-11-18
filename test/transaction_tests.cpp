#include "hand_broker.h"
#include "hand_transactions.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
    TransactionService tx{broker};
    uint64_t now = Now(), owner = 0, other = 0, executor = 0;
    int executions = 0, compensations = 0;
    bool noReceipt = false;
    HIGGS_TransferReceipt receipt{};
    std::array<HIGGS_HandRecord, 2> hands{};
    Fixture()
    {
        HIGGS_HandClientInfo c{};
        c.structSize = sizeof(c);
        std::snprintf(c.name, sizeof(c.name), "provider");
        CHECK(!broker.Register(c, owner));
        std::snprintf(c.name, sizeof(c.name), "other");
        CHECK(!broker.Register(c, other));
        CHECK(!broker.TransferExecutor(owner, Execute, this, executor));
        broker.ObserveWorld(1, false, now);
        for (auto &h : hands)
            h.observationMask =
                HIGGS_OBS_HELD_KNOWN | HIGGS_OBS_EQUIPMENT_KNOWN | HIGGS_OBS_TRACKING_VALID | HIGGS_OBS_PALM_VALID;
        Publish();
    }
    void Publish()
    {
        broker.Publish(hands, ++now);
    }
    HIGGS_HandSnapshot Snapshot()
    {
        HIGGS_HandSnapshot s{};
        CHECK(!broker.Copy(owner, s, now));
        return s;
    }
    HIGGS_ItemIdentity DomainItem(uint64_t key = 10)
    {
        HIGGS_ItemDeclaration d{};
        d.structSize = sizeof(d);
        d.expectedEpoch = broker.Epoch();
        d.providerKey = key;
        d.instanceDataVersion = 1;
        d.quantity = 1;
        d.baseFormId = 100;
        d.validFields = HIGGS_ITEM_BASE_KNOWN | HIGGS_ITEM_QUANTITY_KNOWN | HIGGS_ITEM_INSTANCE_ATTESTED;
        HIGGS_ItemIdentity item{};
        CHECK(!tx.Declare(owner, d, item));
        return item;
    }
    HIGGS_ItemIdentity HeldItem(uint32_t hand = HIGGS_HAND_LEFT, uint32_t ref = 77)
    {
        const auto hi = hand == HIGGS_HAND_LEFT ? 0 : 1;
        hands[hi].heldRefId = ref;
        tx.ObserveHeld(hand, ref + 1000, ref, 100, 1, now);
        Publish();
        HIGGS_ItemIdentity item{};
        CHECK(!tx.Held(owner, hand, item));
        return item;
    }
    HIGGS_TransferRequest Request(uint32_t kind, HIGGS_ItemIdentity item, uint64_t key = 1)
    {
        HIGGS_TransferRequest r{};
        r.structSize = sizeof(r);
        r.kind = kind;
        r.sourceHand = kind == HIGGS_TRANSFER_INVENTORY_TO_HAND ? 0 : HIGGS_HAND_LEFT;
        r.destinationHand =
            (kind == HIGGS_TRANSFER_INVENTORY_TO_HAND || kind == HIGGS_TRANSFER_HAND_TO_HAND) ? HIGGS_HAND_RIGHT : 0;
        r.epoch = broker.Epoch();
        r.idempotencyKey = key;
        r.original = item;
        r.executor = executor;
        r.quantity = 1;
        r.timeoutMs = 1000;
        r.sourceResource = 10;
        r.destinationResource = 20;
        r.contextGeneration = 123;
        return r;
    }
    HIGGS_TransferState State(uint64_t id)
    {
        HIGGS_TransferState s{};
        CHECK(!tx.Copy(owner, id, s));
        return s;
    }
    static void HIGGS_HAND_CALL Execute(const HIGGS_TransferExecution *e, void *data)
    {
        auto &f = *static_cast<Fixture *>(data);
        uint64_t permit = 0;
        CHECK(!f.tx.Begin(f.owner, e->requestId, e->epoch, permit));
        uint64_t duplicate = 0;
        CHECK(f.tx.Begin(f.owner, e->requestId, e->epoch, duplicate) == HIGGS_HAND_CONFLICT);
        if (e->phase == HIGGS_TX_EXECUTE)
            ++f.executions;
        else
            ++f.compensations;
        if (f.noReceipt)
            return;
        HIGGS_TransferReceipt r{};
        r.structSize = sizeof(r);
        r.requestId = e->requestId;
        r.executor = e->executor;
        r.epoch = e->epoch;
        r.mutationPermit = permit;
        r.original = e->request.original;
        r.quantity = e->request.quantity;
        r.actualRefId = 200;
        r.instancePreserved = 1;
        r.kind = e->phase == HIGGS_TX_COMPENSATE
                     ? HIGGS_TX_COMPENSATED
                     : (e->request.kind == HIGGS_TRANSFER_CONSUME_HELD ? HIGGS_TX_CONSUMPTION_COMMITTED
                                                                       : HIGGS_TX_INVENTORY_COMMITTED);
        if (e->request.kind == HIGGS_TRANSFER_EQUIP_GAME || e->request.kind == HIGGS_TRANSFER_UNEQUIP_GAME)
        {
            if (e->phase != HIGGS_TX_COMPENSATE)
                r.kind = HIGGS_TX_EQUIPMENT_COMMITTED;
            r.actualEquippedFormId = (e->request.kind == HIGGS_TRANSFER_EQUIP_GAME) == (e->phase == HIGGS_TX_EXECUTE)
                                         ? e->request.expectedEquippedFormId
                                         : 0;
        }
        f.receipt = r;
        CHECK(f.tx.Report(f.other, r) == HIGGS_HAND_WRONG_OWNER);
        CHECK(!f.tx.Report(f.owner, r));
        CHECK(!f.tx.Report(f.owner, r));
    }
    void Drain(int times = 1)
    {
        for (int i = 0; i < times; ++i)
            tx.Drain(++now);
    }
};
bool Accept(const HIGGS_PhysicalRequest &, void *)
{
    return true;
}
bool Reject(const HIGGS_PhysicalRequest &, void *)
{
    return false;
}
void InventoryGripAndIdempotency()
{
    auto f = std::make_unique<Fixture>();
    auto item = f->DomainItem();
    auto r = f->Request(HIGGS_TRANSFER_INVENTORY_TO_HAND, item);
    uint64_t id = 0, retry = 0;
    CHECK(!f->tx.Request(f->owner, r, id, f->now));
    CHECK(!f->tx.Request(f->owner, r, retry, f->now) && id == retry);
    f->Drain();
    auto s = f->State(id);
    CHECK(f->executions == 1 && s.stage == HIGGS_TX_WAIT_GRIP && (s.receiptFlags & HIGGS_TX_DOMAIN_ATTESTED));
    CHECK(!(s.receiptFlags & HIGGS_TX_NATIVE_GRIP_CONFIRMED));
    f->broker.DrainPhysical(Accept, nullptr, ++f->now);
    f->hands[1].heldRefId = 200;
    f->tx.ObserveHeld(HIGGS_HAND_RIGHT, 1200, 200, 100, 1, f->now);
    f->Publish();
    f->Drain();
    s = f->State(id);
    CHECK(s.stage == HIGGS_TX_COMPLETED && s.disposition == HIGGS_DISPOSITION_HAND &&
          (s.receiptFlags & HIGGS_TX_NATIVE_GRIP_CONFIRMED));
    CHECK(!f->tx.Request(f->owner, r, retry, f->now) && id == retry);
    CHECK(f->executions == 1);
    r.idempotencyKey = 2;
    CHECK(f->tx.Request(f->owner, r, retry, f->now) == HIGGS_HAND_CONFLICT);
    CHECK(!f->Snapshot().hands[1].grant);
    std::puts("PASS typed inventory commitment is not grip, one-shot source and stable retry");
}
void ReleaseBeforeInventory()
{
    for (const auto kind : {HIGGS_TRANSFER_HAND_TO_INVENTORY, HIGGS_TRANSFER_CONSUME_HELD})
    {
        auto f = std::make_unique<Fixture>();
        auto item = f->HeldItem();
        auto r = f->Request(static_cast<uint32_t>(kind), item);
        uint64_t id = 0;
        CHECK(!f->tx.Request(f->owner, r, id, f->now));
        f->Drain();
        CHECK(f->executions == 0 && f->State(id).stage == HIGGS_TX_WAIT_RELEASE);
        f->broker.DrainPhysical(Accept, nullptr, ++f->now);
        f->hands[0].heldRefId = 0;
        f->tx.ObserveHeld(HIGGS_HAND_LEFT, 0, 0, 0, 0, f->now);
        f->Publish();
        f->Drain(2);
        auto s = f->State(id);
        CHECK(s.stage == HIGGS_TX_COMPLETED && (s.receiptFlags & HIGGS_TX_NATIVE_RELEASE_CONFIRMED));
        CHECK(s.disposition == static_cast<uint32_t>(kind == HIGGS_TRANSFER_CONSUME_HELD
                                                         ? HIGGS_DISPOSITION_CONSUMED
                                                         : HIGGS_DISPOSITION_INVENTORY));
        CHECK(f->executions == 1 && !f->Snapshot().hands[0].grant);
    }
    std::puts("PASS native release precedes assigned inventory/consumption mutation");
}
void HandToHand()
{
    auto f = std::make_unique<Fixture>();
    auto item = f->HeldItem();
    auto r = f->Request(HIGGS_TRANSFER_HAND_TO_HAND, item);
    r.executor = 0;
    uint64_t id = 0;
    CHECK(!f->tx.Request(f->owner, r, id, f->now));
    f->Drain();
    f->broker.DrainPhysical(Accept, nullptr, ++f->now);
    f->hands[0].heldRefId = 0;
    f->Publish();
    f->Drain(2);
    f->broker.DrainPhysical(Accept, nullptr, ++f->now);
    f->hands[1].heldRefId = 77;
    f->Publish();
    f->Drain();
    auto s = f->State(id);
    CHECK(s.stage == HIGGS_TX_COMPLETED && s.actualRefId == 77);
    CHECK(s.original.instance == item.instance && !(s.receiptFlags & HIGGS_TX_DOMAIN_ATTESTED));
    CHECK(!f->executions);
    std::puts("PASS two-hand reservation and same-reference physical handover without inventory executor");
}
void CompensationAndUncertainty()
{
    auto f = std::make_unique<Fixture>();
    auto item = f->DomainItem();
    auto r = f->Request(HIGGS_TRANSFER_INVENTORY_TO_HAND, item);
    uint64_t id = 0;
    CHECK(!f->tx.Request(f->owner, r, id, f->now));
    f->Drain();
    const auto firstReceipt = f->receipt;
    f->broker.DrainPhysical(Reject, nullptr, ++f->now);
    f->Drain(3);
    auto s = f->State(id);
    CHECK(f->executions == 1 && f->compensations == 1);
    CHECK(s.stage == HIGGS_TX_COMPLETED && s.disposition == HIGGS_DISPOSITION_INVENTORY &&
          (s.receiptFlags & HIGGS_TX_COMPENSATION_CONFIRMED));
    CHECK(!f->tx.Report(f->owner, firstReceipt));
    auto lost = std::make_unique<Fixture>();
    lost->noReceipt = true;
    item = lost->DomainItem();
    r = lost->Request(HIGGS_TRANSFER_INVENTORY_TO_HAND, item);
    CHECK(!lost->tx.Request(lost->owner, r, id, lost->now));
    lost->Drain();
    CHECK(lost->executions == 1);
    CHECK(!lost->tx.Cancel(lost->owner, id, r.epoch));
    s = lost->State(id);
    CHECK(s.stage == HIGGS_TX_RECOVERY_REQUIRED && (s.receiptFlags & HIGGS_TX_MUTATION_UNCERTAIN));
    r.idempotencyKey = 2;
    uint64_t other = 0;
    CHECK(lost->tx.Request(lost->owner, r, other, lost->now) == HIGGS_HAND_CONFLICT);
    HIGGS_TransferReceipt recovery{};
    recovery.structSize = sizeof(recovery);
    recovery.kind = HIGGS_TX_RECONCILED;
    recovery.requestId = id;
    recovery.executor = lost->executor;
    recovery.epoch = r.epoch;
    recovery.original = item;
    recovery.quantity = 1;
    recovery.instancePreserved = 1;
    recovery.disposition = HIGGS_DISPOSITION_INVENTORY;
    CHECK(!lost->tx.Report(lost->owner, recovery));
    CHECK(lost->State(id).receiptFlags & HIGGS_TX_RECOVERY_RECONCILED);
    CHECK(!lost->tx.Request(lost->owner, r, other, lost->now));
    std::puts("PASS bounded compensation, duplicate facts and uncertain mutation requires reconciliation");
}
void EquipmentObservation()
{
    for (auto kind : {HIGGS_TRANSFER_EQUIP_GAME, HIGGS_TRANSFER_UNEQUIP_GAME})
    {
        auto f = std::make_unique<Fixture>();
        auto item = f->DomainItem();
        if (kind == HIGGS_TRANSFER_UNEQUIP_GAME)
        {
            f->hands[1].equippedFormId = 100;
            f->Publish();
        }
        auto request = f->Request(static_cast<uint32_t>(kind), item);
        request.sourceHand = 0;
        request.destinationHand = HIGGS_HAND_RIGHT;
        request.expectedEquippedFormId = 100;
        uint64_t id = 0;
        CHECK(!f->tx.Request(f->owner, request, id, f->now));
        f->Drain();
        CHECK(f->State(id).stage == HIGGS_TX_WAIT_EQUIPMENT);
        CHECK(!(f->State(id).receiptFlags & HIGGS_TX_NATIVE_EQUIPMENT_CONFIRMED));
        f->Drain();
        CHECK(f->State(id).stage == HIGGS_TX_WAIT_EQUIPMENT);
        f->hands[1].equippedFormId = kind == HIGGS_TRANSFER_EQUIP_GAME ? 100 : 0;
        f->Publish();
        f->Drain();
        CHECK(f->State(id).stage == HIGGS_TX_COMPLETED);
        CHECK(f->State(id).receiptFlags & HIGGS_TX_NATIVE_EQUIPMENT_CONFIRMED);
        HIGGS_ActionResult receipt{};
        CHECK(!f->broker.Result(f->owner, id, receipt));
        CHECK(receipt.receiptFlags & HIGGS_RECEIPT_PHYSICAL_CONFIRMED);
        CHECK(!(receipt.receiptFlags & HIGGS_RECEIPT_INVENTORY_COMMITTED));
    }
    auto f = std::make_unique<Fixture>();
    auto request = f->Request(HIGGS_TRANSFER_EQUIP_GAME, f->DomainItem());
    request.sourceHand = 0;
    request.destinationHand = HIGGS_HAND_RIGHT;
    request.expectedEquippedFormId = 100;
    uint64_t id = 0;
    CHECK(!f->tx.Request(f->owner, request, id, f->now));
    f->Drain();
    CHECK(!f->tx.Cancel(f->owner, id, request.epoch));
    f->Drain();
    CHECK(f->State(id).stage == HIGGS_TX_WAIT_EQUIPMENT);
    f->Publish();
    f->Drain();
    CHECK(f->State(id).stage == HIGGS_TX_COMPLETED);
    CHECK(f->State(id).receiptFlags & HIGGS_TX_COMPENSATION_CONFIRMED);
    std::puts("PASS equipment executor commitment waits for authoritative drawn-equipment observation and compensation "
              "observation");
}
void Abi()
{
    HIGGS_TransactionsInterface api{};
    CHECK(!HIGGS_QueryTransactionsInterface(2, sizeof(api), &api));
    CHECK(api.capabilities & HIGGS_TX_CAP_TRANSFERS);
    auto copy = api;
    CHECK(HIGGS_QueryTransactionsInterface(1, sizeof(api), &api) == HIGGS_HAND_UNSUPPORTED_VERSION);
    CHECK(!std::memcmp(&copy, &api, sizeof(api)));
    std::puts("PASS additive transactions C table negotiation");
}
void MenuCompensation()
{
    auto f=std::make_unique<Fixture>();const auto item=f->DomainItem();auto request=f->Request(HIGGS_TRANSFER_INVENTORY_TO_HAND,item);
    uint64_t id=0;CHECK(!f->tx.Request(f->owner,request,id,f->now));f->Drain();
    CHECK(f->executions==1&&f->State(id).stage==HIGGS_TX_WAIT_GRIP);
    // Inventory already committed; UI cancels the not-yet-entered native grip,
    // then compensation gets its own permit without pretending a grip happened.
    f->broker.SetInputContext(1,++f->now);f->Drain(4);const auto result=f->State(id);
    CHECK(f->compensations==1&&result.stage==HIGGS_TX_COMPLETED&&result.disposition==HIGGS_DISPOSITION_INVENTORY);
    CHECK(result.receiptFlags&HIGGS_TX_COMPENSATION_CONFIRMED);CHECK(!(result.receiptFlags&HIGGS_TX_NATIVE_GRIP_CONFIRMED));
    auto lost=std::make_unique<Fixture>();lost->noReceipt=true;const auto original=lost->DomainItem();request=lost->Request(HIGGS_TRANSFER_INVENTORY_TO_HAND,original);
    CHECK(!lost->tx.Request(lost->owner,request,id,lost->now));lost->Drain();CHECK(lost->executions==1);
    lost->broker.SetInputContext(1,++lost->now);CHECK(lost->State(id).stage==HIGGS_TX_EXECUTING);
    CHECK(!lost->tx.Cancel(lost->owner,id,request.epoch));CHECK(lost->State(id).stage==HIGGS_TX_RECOVERY_REQUIRED);
    CHECK(lost->State(id).receiptFlags&HIGGS_TX_MUTATION_UNCERTAIN);
    std::puts("PASS UI after inventory commit compensates; entered uncertain mutation retains recovery path");
}
int main()
{
    InventoryGripAndIdempotency();
    ReleaseBeforeInventory();
    HandToHand();
    CompensationAndUncertainty();
    EquipmentObservation();
    MenuCompensation();
    Abi();
}
