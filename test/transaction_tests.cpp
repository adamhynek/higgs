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
bool Accept(const HIGGS_PhysicalRequest &, void *);
bool Reject(const HIGGS_PhysicalRequest &, void *);
// An actual selected generic action provides the correlation parent. Transaction
// executors remain the explicit backend seam used by the original fixture.
struct ActionFixture : Fixture
{
    uint64_t actionExecutor = 0, recognizer = 0, recognizerOwner = 0;
    uint64_t borrowedGrant = 0;
    uint32_t triggerHand = HIGGS_HAND_LEFT, requiredHands = HIGGS_HAND_BOTH;
    HIGGS_ActionRequest action{};

    void Select(bool routed = false, bool extended = false)
    {
        recognizerOwner = routed ? other : owner;
        CHECK(!broker.Executor(owner, Selected, this, actionExecutor));
        if (routed)
            CHECK(!broker.Authorize(owner, actionExecutor, other, requiredHands));
        if (extended)
        {
            HIGGS_InputRecognizerSpec spec{};
            spec.structSize = sizeof(spec);
            spec.physicalHands = requiredHands;
            spec.controlMask = UINT64_C(1) << HIGGS_CONTROL_GRIP;
            spec.callback = RecognizeExtended;
            spec.context = this;
            CHECK(!broker.InputRecognizer(recognizerOwner, spec, recognizer));
            ControllerGrip(false); // establish a released baseline before a fresh press
            CHECK(ControllerGrip(true));
        }
        else
        {
            CHECK(!broker.Recognizer(recognizerOwner, Recognize, this, recognizer));
            CHECK(broker.Input(triggerHand, HIGGS_INPUT_GRIP, true, ++now));
        }
        CHECK(action.requestId && action.owner == owner && action.physicalHands == requiredHands);
    }

    bool ControllerGrip(bool down)
    {
        HIGGS_ControllerFrame frame{};
        frame.structSize = sizeof(frame);
        frame.physicalHand = triggerHand;
        frame.deviceIndex = triggerHand;
        frame.flags = HIGGS_INPUT_NO_HARDWARE_TIME;
        frame.epoch = broker.Epoch();
        frame.observedAtNs = ++now;
        frame.handObservationId = broker.CopyPublished().observationId;
        frame.pressed = down ? UINT64_C(1) << HIGGS_CONTROL_GRIP : 0;
        // The production frame store/current-frame marker supplies the raw
        // observation; this fixture's own broker performs recognition/admission.
        BeginControllerFrame(frame);
        return broker.Input(triggerHand, HIGGS_INPUT_GRIP, down, now);
    }

    static void HIGGS_HAND_CALL RecognizeExtended(const HIGGS_InputEvent *input, void *context)
    {
        if (!(input->flags & HIGGS_INPUT_ACTIONABLE) || input->kind != HIGGS_RAW_PRESS)
            return;
        auto &f = *static_cast<ActionFixture *>(context);
        HIGGS_InputCandidate candidate{};
        candidate.structSize = sizeof(candidate);
        candidate.defaultPolicy = HIGGS_DEFAULT_DISCARD_AFTER_SELECTION;
        candidate.existingGrant = f.borrowedGrant;
        candidate.candidate = {sizeof(candidate.candidate), f.requiredHands, input->inputId,
            input->snapshot.epoch, input->snapshot.hands[input->physicalHand - 1].generation,
            f.actionExecutor, 701, 801, 901, 1000, 0};
        uint64_t request = 0;
        CHECK(!f.broker.SubmitInputCandidate(f.recognizerOwner, candidate, request, f.now));
    }

    static void HIGGS_HAND_CALL Recognize(const HIGGS_HandInput *input, void *context)
    {
        auto &f = *static_cast<ActionFixture *>(context);
        if (input->edge != HIGGS_INPUT_DOWN)
            return;
        HIGGS_ActionCandidate candidate{sizeof(candidate), f.requiredHands, input->inputId,
            input->epoch, input->handGeneration, f.actionExecutor, 701, 801, 901, 1000, 0};
        uint64_t request = 0;
        CHECK(!f.broker.Submit(f.recognizerOwner, candidate, request, f.now));
    }

    static void HIGGS_HAND_CALL Selected(const HIGGS_ActionRequest *request, void *context)
    {
        static_cast<ActionFixture *>(context)->action = *request;
    }

    uint64_t Child(HIGGS_ItemIdentity item, uint64_t key = 1)
    {
        auto request = Request(HIGGS_TRANSFER_INVENTORY_TO_HAND, item, key);
        request.grant = action.grant;
        uint64_t child = 0;
        CHECK(!tx.Request(owner, request, child, now));
        return child;
    }

    HIGGS_ActionResult ParentResult()
    {
        HIGGS_ActionResult result{};
        CHECK(!broker.Result(owner, action.requestId, result));
        return result;
    }

    void CancelParent()
    {
        auto result = ParentResult();
        result.stage = HIGGS_ACTION_CANCELLED;
        CHECK(!broker.Report(owner, result));
    }

    HIGGS_ActionInventoryEffect Effects(uint64_t observer = 0)
    {
        HIGGS_ActionInventoryEffect result{};
        CHECK(!tx.CopyInventoryEffect(observer ? observer : owner, action.requestId, result));
        CHECK(result.structSize == sizeof(result) && result.parentRequestId == action.requestId &&
              result.epoch == action.epoch && !result.reserved);
        return result;
    }

    void CompleteGrip(uint64_t child)
    {
        Drain();
        CHECK(State(child).stage == HIGGS_TX_WAIT_GRIP);
        broker.DrainPhysical(Accept, nullptr, ++now);
        hands[1].heldRefId = 200;
        tx.ObserveHeld(HIGGS_HAND_RIGHT, 1200, 200, 100, 1, now);
        Publish();
        Drain();
        CHECK(State(child).stage == HIGGS_TX_COMPLETED &&
              (State(child).receiptFlags & HIGGS_TX_NATIVE_GRIP_CONFIRMED));
    }

    void CompensateGrip(uint64_t child)
    {
        Drain();
        CHECK(State(child).stage == HIGGS_TX_WAIT_GRIP);
        broker.DrainPhysical(Reject, nullptr, ++now);
        Drain(4);
        CHECK(State(child).stage == HIGGS_TX_COMPLETED &&
              (State(child).receiptFlags & HIGGS_TX_COMPENSATION_CONFIRMED));
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

void InventoryEffectClosure()
{
    for (bool admitChild : {false, true})
    {
        auto f = std::make_unique<ActionFixture>();
        const auto item = f->DomainItem();
        f->Select(true);
        CHECK(f->Effects().flags == HIGGS_INVENTORY_EFFECT_UNCERTAIN);
        if (admitChild)
        {
            const auto child = f->Child(item);
            // No executor has entered mutation. Cancellation supplies a positive
            // nonmutation fact; lack of a parent inventory flag alone cannot.
            CHECK(f->Effects().flags & HIGGS_INVENTORY_EFFECT_UNCERTAIN);
            CHECK(!f->tx.Cancel(f->owner, child, f->action.epoch));
            CHECK(f->State(child).stage == HIGGS_TX_CANCELLED && !f->executions);
        }
        f->CancelParent();
        auto effect = f->Effects();
        CHECK(effect.flags == HIGGS_INVENTORY_EFFECT_CLOSED && effect.childCount == unsigned(admitChild));
        const auto routed = f->Effects(f->other);
        CHECK(routed.flags == effect.flags && routed.childCount == effect.childCount);
        // Read-only routed observation grants no reporting/acknowledgement right.
        auto result = f->ParentResult();
        CHECK(f->broker.Report(f->other, result) == HIGGS_HAND_WRONG_OWNER);
        CHECK(f->broker.Release(f->other, f->action.grant) == HIGGS_HAND_WRONG_OWNER);
        CHECK(!f->broker.Release(f->owner, f->action.grant));
        effect = f->Effects();
        CHECK(effect.flags == (admitChild ? HIGGS_INVENTORY_EFFECT_UNCERTAIN : HIGGS_INVENTORY_EFFECT_CLOSED));
    }
    auto f = std::make_unique<ActionFixture>();
    f->Select();
    HIGGS_ActionInventoryEffect untouched{};
    std::memset(&untouched, 0x5a, sizeof(untouched));
    const auto before = untouched;
    CHECK(f->tx.CopyInventoryEffect(f->other, f->action.requestId, untouched) == HIGGS_HAND_WRONG_OWNER);
    CHECK(!std::memcmp(&untouched, &before, sizeof(before)));
    f->broker.ObserveWorld(2, false, ++f->now);
    CHECK(f->tx.CopyInventoryEffect(f->owner, f->action.requestId, untouched) == HIGGS_HAND_STALE);
    CHECK(!std::memcmp(&untouched, &before, sizeof(before)));
    std::puts("PASS positive closed nonmutation, exact routed observer and acknowledged-history/epoch fences");
}

void AggregateInventoryEffects()
{
    for (unsigned mode = 0; mode < 3; ++mode)
    {
        auto f = std::make_unique<ActionFixture>();
        const auto firstItem = f->DomainItem(10), secondItem = f->DomainItem(20);
        f->Select();
        const auto first = f->Child(firstItem);
        f->CompensateGrip(first);
        auto effect = f->Effects();
        CHECK((effect.flags & (HIGGS_INVENTORY_EFFECT_COMMITTED | HIGGS_INVENTORY_EFFECT_UNCERTAIN)) ==
              (HIGGS_INVENTORY_EFFECT_COMMITTED | HIGGS_INVENTORY_EFFECT_UNCERTAIN));
        CHECK(!(effect.flags & (HIGGS_INVENTORY_EFFECT_CLOSED | HIGGS_INVENTORY_EFFECT_RESTORED)));
        const auto second = f->Child(secondItem, 2);
        if (mode == 0)
            f->CompensateGrip(second);
        else if (mode == 1)
        {
            f->noReceipt = true;
            f->Drain();
            CHECK(f->State(second).stage == HIGGS_TX_EXECUTING);
            CHECK(!f->tx.Cancel(f->owner, second, f->action.epoch));
            CHECK(f->State(second).stage == HIGGS_TX_RECOVERY_REQUIRED &&
                  (f->State(second).receiptFlags & HIGGS_TX_MUTATION_UNCERTAIN));
        }
        else
            f->CompleteGrip(second);
        f->CancelParent();
        // Deliberately keep the parent's summary empty: the service must use all
        // correlated transaction facts, not assume cancellation means rollback.
        CHECK(!f->ParentResult().receiptFlags);
        effect = f->Effects();
        CHECK(effect.childCount == 2 && (effect.flags & HIGGS_INVENTORY_EFFECT_CLOSED) &&
              (effect.flags & HIGGS_INVENTORY_EFFECT_COMMITTED));
        CHECK(bool(effect.flags & HIGGS_INVENTORY_EFFECT_UNCERTAIN) == (mode == 1));
        CHECK(bool(effect.flags & HIGGS_INVENTORY_EFFECT_RESTORED) == (mode == 0));
    }
    std::puts("PASS aggregate restoration requires every mutation compensated and no uncertain sibling");
}

void MissingTransactionIsNotNonmutation()
{
    auto f = std::make_unique<ActionFixture>();
    f->Select();
    HIGGS_ActionRequest child{};
    CHECK(!f->broker.ReserveDomain(f->owner, f->executor, HIGGS_HAND_RIGHT, f->action.epoch,
        f->action.grant, 1000, child, f->now));
    // Model a recorded native-domain effect without a retained transaction row.
    // Its public summary cannot be upgraded to complete inventory evidence.
    f->broker.DomainResult(child.requestId, HIGGS_ACTION_COMPLETED, HIGGS_RECEIPT_INVENTORY_COMMITTED, 200, 1);
    f->broker.RetainRequest(child.requestId, false);
    f->CancelParent();
    const auto effect = f->Effects();
    CHECK(effect.childCount == 1 && (effect.flags & HIGGS_INVENTORY_EFFECT_COMMITTED) &&
          (effect.flags & HIGGS_INVENTORY_EFFECT_UNCERTAIN) && !(effect.flags & HIGGS_INVENTORY_EFFECT_RESTORED));
    std::puts("PASS missing transaction history stays uncertain despite a committed child summary");
}

void FactualReceiptAfterAuthorityExpiry()
{
    for (unsigned invalidation = 0; invalidation < 4; ++invalidation)
    {
        auto f = std::make_unique<ActionFixture>();
        const auto item = f->DomainItem();
        // Extended recognizers carry their own registration identity. Legacy
        // generic recognizer removal cannot establish that per-source boundary.
        f->Select(true, invalidation == 2);
        const auto child = f->Child(item);
        f->CompleteGrip(child);
        HIGGS_ActionResult factual{};
        CHECK(!f->broker.Result(f->owner, child, factual));
        HIGGS_HandSnapshot expired{};
        if (invalidation == 0)
            CHECK(!f->broker.Copy(f->owner, expired, f->now + UINT64_C(2000000000)));
        else
        {
            if (invalidation == 1)
            {
                f->hands[0].observationMask &= ~HIGGS_OBS_TRACKING_VALID;
                f->Publish();
            }
            if (invalidation == 2)
                CHECK(!f->broker.Remove(f->other, f->recognizer));
            if (invalidation == 3)
                CHECK(!f->broker.Unregister(f->other));
            CHECK(!f->broker.Copy(f->owner, expired, f->now));
        }
        CHECK(!expired.hands[0].grant && !expired.hands[1].grant);
        CHECK(f->ParentResult().stage == HIGGS_ACTION_ACCEPTED);
        auto report = f->ParentResult();
        report.stage = HIGGS_ACTION_COMPLETED;
        report.receiptFlags = factual.receiptFlags;
        report.itemRefId = factual.itemRefId;
        report.quantity = factual.quantity;
        auto wrong = report;
        ++wrong.quantity;
        CHECK(f->broker.Report(f->owner, wrong) == HIGGS_HAND_CONFLICT);
        wrong = report;
        ++wrong.itemRefId;
        CHECK(f->broker.Report(f->owner, wrong) == HIGGS_HAND_CONFLICT);
        CHECK(!f->broker.Report(f->owner, report));
        CHECK(!f->broker.Report(f->owner, report));
        const auto effect = f->Effects();
        CHECK(effect.childCount == 1 && effect.flags ==
              (HIGGS_INVENTORY_EFFECT_CLOSED | HIGGS_INVENTORY_EFFECT_COMMITTED));
        HIGGS_PhysicalRequest physical{sizeof(physical), HIGGS_PHYSICAL_RELEASE, HIGGS_HAND_RIGHT,
            200, f->action.epoch, expired.hands[1].generation, f->action.grant, 1000, 0};
        uint64_t request = 0;
        CHECK(f->broker.Physical(f->owner, physical, request, f->now) != HIGGS_HAND_OK);
        CHECK(!f->broker.Release(f->owner, f->action.grant));
        CHECK(f->Effects().flags & HIGGS_INVENTORY_EFFECT_UNCERTAIN);
    }
    std::puts("PASS expiry/tracking/routed-source teardown retain exact facts without renewed authority");
}

void UnattributedContinuationCannotProveNonmutation()
{
    auto f = std::make_unique<ActionFixture>();
    const auto item = f->DomainItem();
    f->Select();
    const auto original = f->action;
    // Legacy recognizer removal retires callbacks, not an identified action;
    // the selected grant remains available for an explicit fresh continuation.
    CHECK(!f->broker.Remove(f->owner, f->recognizer));
    f->borrowedGrant = original.grant;
    f->Select(false, true);
    CHECK(f->action.grant == original.grant && f->action.inputId != original.inputId);
    HIGGS_ActionResult retired{};
    CHECK(!f->broker.Result(f->owner, original.requestId, retired));
    retired.stage = HIGGS_ACTION_CANCELLED;
    CHECK(!f->broker.Report(f->owner, retired));
    // The original episode is now terminal, but the continuation still owns
    // this same grant and can start a transaction in its own input episode.
    const auto child = f->Child(item);
    f->CompleteGrip(child);
    CHECK(f->State(child).receiptFlags & HIGGS_TX_DOMAIN_ATTESTED);
    f->CancelParent();
    CHECK(!f->broker.Remove(f->owner, f->recognizer));
    const auto snapshot = f->Snapshot();
    CHECK(!snapshot.hands[0].grant && !snapshot.hands[1].grant);
    HIGGS_ActionInventoryEffect effect{};
    CHECK(!f->tx.CopyInventoryEffect(f->owner, original.requestId, effect));
    // This original parent never borrowed a grant. Its empty correlated set
    // must still remain uncertain: another episode used the same grant after
    // it became terminal. Removing that continuation cannot erase this fact.
    CHECK(!effect.childCount && effect.flags == HIGGS_INVENTORY_EFFECT_UNCERTAIN);
    CHECK(!f->broker.Release(f->owner, original.grant));
    CHECK(!f->tx.CopyInventoryEffect(f->owner, original.requestId, effect));
    CHECK(!effect.childCount && effect.flags == HIGGS_INVENTORY_EFFECT_UNCERTAIN);
    std::puts("PASS removed same-grant continuation cannot manufacture closed nonmutation evidence");
}

void RetainedChildHistoryUntilAcknowledgement()
{
    auto f = std::make_unique<ActionFixture>();
    const auto item = f->DomainItem(10), untouchedItem = f->DomainItem(20);
    f->Select();
    const auto retained = f->Child(item);
    f->CompensateGrip(retained);
    f->CancelParent();
    const auto original = f->State(retained);
    auto churn = [&](unsigned offset)
    {
        // Exceed both finite service histories using successful request/cancel
        // pairs on an unrelated item, without executing any mutation backend.
        for (unsigned i = 0; i < 300; ++i)
        {
            auto request = f->Request(HIGGS_TRANSFER_INVENTORY_TO_HAND, untouchedItem, offset + i);
            uint64_t id = 0;
            CHECK(!f->tx.Request(f->owner, request, id, f->now));
            CHECK(!f->tx.Cancel(f->owner, id, request.epoch));
        }
    };
    churn(100);
    const auto retainedState = f->State(retained);
    CHECK(retainedState.stage == original.stage && retainedState.receiptFlags == original.receiptFlags);
    CHECK(f->Effects().flags == (HIGGS_INVENTORY_EFFECT_CLOSED | HIGGS_INVENTORY_EFFECT_COMMITTED |
                                HIGGS_INVENTORY_EFFECT_RESTORED));
    // Copying and duplicate terminal reports are observation, not consumption.
    CHECK(!f->broker.Report(f->owner, f->ParentResult()));
    CHECK(f->Effects().childCount == 1);
    CHECK(!f->broker.Release(f->owner, f->action.grant));
    CHECK(f->Effects().flags & HIGGS_INVENTORY_EFFECT_UNCERTAIN);
    churn(1000);
    HIGGS_TransferState removed{};
    CHECK(f->tx.Copy(f->owner, retained, removed) == HIGGS_HAND_NOT_FOUND);
    std::puts("PASS parent/child facts survive finite-history churn until explicit consumer acknowledgement");
}

void FollowOnlyFactualInventoryGrip()
{
    auto f = std::make_unique<ActionFixture>();
    f->triggerHand = f->requiredHands = HIGGS_HAND_RIGHT;
    HIGGS_ItemIdentity item{};
    CHECK(!f->tx.ObserveInventory(f->owner, 100, 1, 100, 1, item));
    f->Select();
    const auto child = f->Child(item);
    CHECK(f->tx.FollowInventoryGrip(f->owner, child) == HIGGS_HAND_WRONG_PHASE);
    CHECK(f->tx.FollowInventoryGrip(f->other, child) == HIGGS_HAND_WRONG_OWNER);
    f->CompleteGrip(child);
    CHECK(!f->tx.FollowInventoryGrip(f->owner, child));
    CHECK(!f->tx.FollowInventoryGrip(f->owner, child));
    auto incomplete = std::make_unique<ActionFixture>();
    incomplete->triggerHand = incomplete->requiredHands = HIGGS_HAND_RIGHT;
    CHECK(!incomplete->tx.ObserveInventory(incomplete->owner, 101, 1, 100, 1, item));
    incomplete->Select();
    const auto waiting = incomplete->Child(item);
    incomplete->Drain();
    CHECK(incomplete->State(waiting).stage == HIGGS_TX_WAIT_GRIP &&
          (incomplete->State(waiting).receiptFlags & HIGGS_TX_DOMAIN_ATTESTED));
    CHECK(incomplete->tx.FollowInventoryGrip(incomplete->owner, waiting) == HIGGS_HAND_WRONG_PHASE);
    auto compensated = std::make_unique<ActionFixture>();
    compensated->triggerHand = compensated->requiredHands = HIGGS_HAND_RIGHT;
    CHECK(!compensated->tx.ObserveInventory(compensated->owner, 102, 1, 100, 1, item));
    compensated->Select();
    const auto restored = compensated->Child(item);
    compensated->CompensateGrip(restored);
    CHECK(compensated->tx.FollowInventoryGrip(compensated->owner, restored) == HIGGS_HAND_WRONG_PHASE);
    std::puts("PASS inventory-grip continuation requires the exact owner's completed, uncompensated native hold");
}

int main()
{
    InventoryGripAndIdempotency();
    ReleaseBeforeInventory();
    HandToHand();
    CompensationAndUncertainty();
    EquipmentObservation();
    MenuCompensation();
    InventoryEffectClosure();
    AggregateInventoryEffects();
    MissingTransactionIsNotNonmutation();
    FactualReceiptAfterAuthorityExpiry();
    UnattributedContinuationCannotProveNonmutation();
    RetainedChildHistoryUntilAcknowledgement();
    FollowOnlyFactualInventoryGrip();
    Abi();
}
